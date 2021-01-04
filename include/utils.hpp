#ifndef UTILS_H
#define UTILS_H

#include <random>
#include <vecmath.h>

using namespace std;

float frand(mt19937 *mt_rand);

float clamp(float x);

int toInt(float x);

Vector3f uniformSample(mt19937 *rd);

float dis(Vector3f a, Vector3f b);

#endif