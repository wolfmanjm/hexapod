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
    home();
}

// transform x, y to match leg position
void  Leg::transform(float m[2][2], float& x, float& y)
{
    float nx= x * m[0][0] + y * m[1][0];
    float ny= x * m[0][1] + y * m[1][1];
    x= nx;
    y= ny;
}

// all servos will be at 90°
bool Leg::home()
{
    float x= COXA+FEMUR, y= 0;
    transform(home_mat, x, y);
    return move(x, y, -TIBIA);
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
    return move(x, y, std::get<2>(position));
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

bool Leg::rotateBy(float rad)
{
    // Rotate the tip of the leg around the center of robot's body.
    // TODO this maybe incorrect for a hexagon shaped body
    float x = std::get<0>(position) + BASE_RADIUS;
    float y = std::get<1>(position) + BASE_RADIUS;
    float nx = x * cosf(rad)  + y * sinf(rad) - BASE_RADIUS;
    float ny = x * -sinf(rad) + y * cosf(rad) - BASE_RADIUS;
    return move(nx, ny, std::get<2>(position));
}
