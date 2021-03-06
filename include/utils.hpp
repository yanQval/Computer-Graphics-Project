#ifndef UTILS_H
#define UTILS_H

#include <random>
#include <vecmath.h>

using namespace std;

double frand(mt19937 *mt_rand);

double clamp(double x);

int toInt(double x);

Vector3f uniformSample(mt19937 *rd);
Vector3f semi_uniformSample(Vector3f n, mt19937 *rd);

double dis(Vector3f a, Vector3f b);

Vector3f min(const Vector3f &x, const Vector3f &y);
Vector3f max(const Vector3f &x, const Vector3f &y);

#endif