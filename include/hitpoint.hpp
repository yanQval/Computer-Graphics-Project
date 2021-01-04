#ifndef HITPOINT_H
#define HITPOINT_H

#include <vecmath.h>
#include <cmath>

//ajskl;dfjl;asjfklasj;lkfjalkdjfklasjflsa

class HitPoint
{
public:
    HitPoint(Vector3f p, Vector3f n, Vector3f d, int _x, int _y, Vector3f c) : pos(p), norm(n), dir(d), x(_x), y(_y), col(c)
    {
        radius = 0;
        N = 0;
        M = 0;
        tau = Vector3f::ZERO;
    }
    void print()
    {
        printf("Pixel : %d %d\n Position : ", x, y);
        pos.print();
    }

    Vector3f pos, norm, dir;
    int x, y;
    Vector3f col;
    float radius;
    int N;
    int M;
    Vector3f tau;
};

#endif