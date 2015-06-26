/*
    coordinate system is the same for every leg, so a move in X will move all legs the same, this is done by applying the transform to the x y being moved

              Y
              |
              |
              |
    X---------+

    +Z is up

    leg positions

    0---------------3
    |               |
    |               |
    |               |
    |               |
    |               |
    1---------------2

    at Home position the legs are out at a diagonal

*/

#include "Leg.h"
#include "Servo.h"

#include <cmath>

const static float PI2 = M_PI_2;
const static float PI4 = M_PI_4;

#define RADIANS(a) (a * M_PI / 180.0F)

Leg::Leg(float pos_angle, int8_t x, int8_t y, uint8_t joint1, uint8_t joint2, uint8_t joint3, Servo &servo) : servo(servo)
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

    this->xp= x;
    this->yp= y;
    home();
}

// transform x, y to match leg position
void  Leg::transform(float& x, float& y)
{
    float nx= x * mat[0][0] + y * mat[1][0];
    float ny= x * mat[0][1] + y * mat[1][1];
    x= nx;
    y= ny;
}

bool Leg::home()
{
    float x= xp, y= yp;
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

std::tuple<float, float, float> Leg::inverseKinematics(float x, float y, float z)
{
    // Calculate angles for knee and ankle, and put them in those variables.
    // Return true on success, and false if x and y are out of range.
    float ankle, knee, hip;
    float f = norm(x, y) - COXA;
    float d = norm(f, z);
    if (d > FEMUR + TIBIA) {
        return std::make_tuple(NAN, NAN, NAN);
    }

    hip = atan2f(y, x);
    knee = solveTriangle(FEMUR, d, TIBIA) - atan2f(-z, f);
    ankle = solveTriangle(FEMUR, TIBIA, d) - PI2;
    return std::make_tuple(hip, knee, ankle);
}

bool Leg::move(float x, float y, float z)
{
    // Move the tip of the leg to x, y. Return false when out of range.
    float ankle = NAN;
    float knee = NAN;
    float hip = NAN;


    position[0] = x;
    position[1] = y;
    position[2] = z;

    //printf("Move x:%f, y:%f, z:%f\n", x, y, z);
    transform(x, y);
    //printf("transformed Move x:%f, y:%f, z:%f\n", x, y, z);

    std::tie(hip, knee, ankle) = inverseKinematics(x, y, z);
    if(isnan(hip) || isnan(knee) || isnan(ankle)) {
        return false;
    }

    hip -= PI4;
    servo.move(joint[0], ankle);
    servo.move(joint[1], knee);
    servo.move(joint[2], hip);
    return true;
}

bool Leg::moveBy(float dx, float dy, float dz)
{
    // Move the tip of the leg by dx, dy. Return false when out of range.
    return move( position[0] + dx,
                 position[1] + dy,
                 position[2] + dz);
}

bool Leg::rotateBy(float rad)
{
    // Rotate the tip of the leg around the center of robot's body.
    float x = position[0] + BASE;
    float y = position[1] + BASE;
    float nx = x * cosf(rad) - y * sinf(rad) - BASE;
    float ny = x * sinf(rad) + y * cosf(rad) - BASE;
    return move(nx, ny, position[2]);
}
