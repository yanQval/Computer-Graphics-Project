#include "utils.hpp"

#include <random>
#include <vecmath.h>

using namespace std;

float frand(mt19937 *mt_rand)
{
    return (float)mt_rand->operator()() / mt_rand->max();
}

float clamp(float x)
{
    return x < 0 ? 0 : x > 1 ? 1 : x;
}

int toInt(float x)
{
    return int(pow(clamp(x), 1 / 2.2) * 255 + .5);
}

Vector3f uniformSample(mt19937 *rd)
{
    float theta, cosPhi, sinPhi;
    Vector3f v;
    do
    {
        theta = 2 * M_PI * frand(rd);
        cosPhi = 2 * frand(rd) - 1;
        sinPhi = sqrt(1 - cosPhi * cosPhi);
        v = Vector3f(cos(theta) * sinPhi, sin(theta) * sinPhi, cosPhi);
    } while (0);
    //while (Vector3f::dot(n, v) <= 0);
    return v;
}

float dis(Vector3f a, Vector3f b)
{
    return (a - b).length();
}