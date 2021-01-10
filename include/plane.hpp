#ifndef PLANE_H
#define PLANE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

// TODO: Implement Plane representing an infinite plane
// function: ax+by+cz=d
// choose your representation , add more fields and fill in the functions

class Plane : public Object3D
{
public:
    Plane() = delete;

    Plane(const Vector3f &_normal, double _d, Material *m, bool ut, Vector3f _x, Vector3f _y) : Object3D(m)
    {
        assert(_normal.length() == 1);
        normal = _normal;
        d = _d;
        material = m;
        useTexture = ut;
        x = _x;
        y = _y;
    }

    ~Plane() override = default;

    bool intersect(const Ray &r, Hit &h, double tmin) override
    {
        Vector3f O, R;
        double t;
        O = r.getOrigin();
        R = r.getDirection();
        t = (d - Vector3f::dot(normal, O)) / Vector3f::dot(normal, R);
        if (t < tmin)
            return false;
        if (t > h.getT())
            return false;
        bool into = (Vector3f::dot(normal, R) < 0);
        h = Hit(t, material, into ? normal : -normal, into);
        if (useTexture)
        {
            //puts("!!!!!");
            Vector3f P = r.pointAtParameter(t);
            //P.print();
            //x.print();
            h.texPos = Vector2f(Vector3f::dot(P, x), Vector3f::dot(P, y));
            //h.texPos.print();
        }
        //printf("%.4lf\n", t);
        return true;
    }

protected:
    Vector3f normal;
    double d;
    Material *material;

    bool useTexture;
    Vector3f x, y;
};

#endif //PLANE_H
