/*
    coordinate system is the same for every leg, so a move in X will move all legs the same, this is done by applying the transform to the x y being moved

              Y
              |
              |
              |
    X---------+

    +Z is up

 leg positions

          0---------------5
         /                 \
        /                   \
       /                     \
      /                       \
     /                         \
    1                           4
     \                         /
      \                       /
       \                     /
        \                   /
         \                 /
          2---------------3


*/

#include "Leg.h"
#include "Servo.h"

#include <cmath>

const static float PI2 = M_PI_2;
const static float PI4 = M_PI_4;

#define RADIANS(a) (a * M_PI / 180.0F)

Leg::Leg(float pos_angle, float home_angle, uint8_t joint1, uint8_t joint2, uint8_t joint3, Servo &servo) : servo(servo)
{
    // which servo
    this->joint[0] = joint1;
    this->joint[1] = joint2;
    this->joint[2] = joint3;

    // leg position on body, 0° is the front left corner
    // this trnsformation matrix is applied to the move X, Y to generate the correct movements
    float r= RADIANS(pos_angle);
    mat[0][0]= cosf(r); mat[0][1]= -sinf(r);
    mat[1][0]= sinf(r); mat[1][1]= cosf(r);

    r= RADIANS(home_angle);
    home_mat[0][0]= cosf(r); home_mat[0][1]= -sinf(r);
    home_mat[1][0]= sinf(r); home_mat[1][1]= cosf(r);
    on_ground= true;

    // figure out the origin of the hip joint in robot coordinates, which is -x to right and +y up same as leg coordinates
    origin[0]= BASE_RADIUS * cosf(RADIANS(pos_angle)); // X
    origin[1]= BASE_RADIUS * sinf(RADIANS(pos_angle)); // Y

    home();
}

// transform x, y to match leg position
void  Leg::transform(const float m[2][2], float& x, float& y) const
{
    float nx= x * m[0][0] + y * m[1][0];
    float ny= x * m[0][1] + y * m[1][1];
    x= nx;
    y= ny;
}

Leg::Vec3 Leg::getHomeCoordinates() const
{
    float x= COXA+FEMUR, y= 0, z= -TIBIA;
    transform(home_mat, x, y);
    return Vec3(x, y, z);
}

// all servos will be at 90°
bool Leg::home()
{
    float x, y, z;
    std::tie(x, y, z) = getHomeCoordinates();
    return move(x, y, z);
}

float Leg::solveTriangle(float a, float b, float c)
{
    // Calculate the angle between a and b, opposite to c.
    a = std::abs(a);
    b = std::abs(b);
    c = std::abs(c);
    if (a + b < c || a + c < b || b + c < a) {
        return NAN;
    }
    return acosf((a * a + b * b - c * c) / (2 * a * b));
}

Leg::Vec3 Leg::inverseKinematics(float x, float y, float z)
{
    // Calculate angles for knee and ankle
    float ankle, knee, hip;
    float f = norm(x, y) - COXA;
    float d = norm(f, z);
    if (d > FEMUR + TIBIA) {
        return std::make_tuple(NAN, NAN, NAN);
    }

    hip = atan2f(y, x);
    knee = solveTriangle(FEMUR, d, TIBIA) - atan2f(-z, f);
    ankle = solveTriangle(FEMUR, TIBIA, d) - PI2;
    return Vec3(hip, knee, ankle);
}

bool Leg::move(float x, float y)
{
    if(!move(x, y, std::get<2>(position))) {
        printf("move out of range: %f, %f, %f\n", x, y, std::get<2>(position));
        return false;
    }
    return true;
}

bool Leg::move(float x, float y, float z)
{
    // Move the tip of the leg to x, y. Return false when out of range.
    float ankle = NAN;
    float knee = NAN;
    float hip = NAN;

    position= std::make_tuple(x, y, z);

    //printf("Move x:%f, y:%f, z:%f\n", x, y, z);
    transform(mat, x, y);
    //printf("transformed Move x:%f, y:%f, z:%f\n", x, y, z);

    std::tie(hip, knee, ankle) = inverseKinematics(x, y, z);
    if(isnan(hip) || isnan(knee) || isnan(ankle)) {
        printf("move out of range: %f, %f, %f\n", x, y, z);
        return false;
    }

    //hip -= PI4;
    servo.move(joint[0], ankle);
    servo.move(joint[1], knee);
    servo.move(joint[2], hip);
    return true;
}

bool Leg::moveBy(float dx, float dy, float dz)
{
    // Move the tip of the leg by dx, dy. Return false when out of range.
    return move( std::get<0>(position) + dx,
                 std::get<1>(position) + dy,
                 std::get<2>(position) + dz);
}

Leg::Vec3 Leg::calcRotation(float rad, bool abs) const
{
    // Rotate the tip of the leg around the center of robot's body.
    float x, y;
    if(abs) {
        // based on home position
        std::tie(x, y, std::ignore) = getHomeCoordinates();

    }else{
        // based on current position
        std::tie(x, y, std::ignore) = position;
    }
    x += origin[0];
    y += origin[1];
    float nx = x *  cosf(rad) + y * sinf(rad);
    float ny = x * -sinf(rad) + y * cosf(rad);
    nx -=  origin[0];
    ny -=  origin[1];
    return Vec3(nx, ny, std::get<2>(position));
}

bool Leg::rotateBy(float rad)
{
    float x, y, z;
    std::tie(x, y, z)= calcRotation(rad);
    return move(x, y, z);
}
