#include "utils.hpp"

#include <random>
#include <vecmath.h>

using namespace std;

double frand(mt19937 *mt_rand)
{
    return (double)mt_rand->operator()() / mt_rand->max();
}

double clamp(double x)
{
    return x < 0 ? 0 : x > 1 ? 1 : x;
}

int toInt(double x)
{
    return int(pow(clamp(x), 1 / 2.2) * 255 + .5);
}

Vector3f uniformSample(mt19937 *rd)
{
    while (true)
    {
        Vector3f d((frand(rd) - 0.5) * 2, (frand(rd) - 0.5) * 2, (frand(rd) - 0.5) * 2);
        if (d.length() <= 1)
            return d;
    }
}

Vector3f semi_uniformSample(Vector3f n, mt19937 *rd)
{
    //return n;
    while (true)
    {
        Vector3f d((frand(rd) - 0.5) * 2, (frand(rd) - 0.5) * 2, (frand(rd) - 0.5) * 2);
        if (d.length() <= 1 && Vector3f::dot(n, d) > 0)
            return d;
    }
}

double dis(Vector3f a, Vector3f b)
{
    return (a - b).length();
}

Vector3f min(const Vector3f &x, const Vector3f &y)
{
    return Vector3f(min(x[0], y[0]), min(x[1], y[1]), min(x[2], y[2]));
}

Vector3f max(const Vector3f &x, const Vector3f &y)
{
    return Vector3f(max(x[0], y[0]), max(x[1], y[1]), max(x[2], y[2]));
}