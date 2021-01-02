#ifndef SPHERE_H
#define SPHERE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

#include "constant.h"

// TODO: Implement functions and add more fields as necessary

class Sphere : public Object3D
{
public:
    Sphere() = delete;

    Sphere(const Vector3f &_center, float _radius, Material *_material) : Object3D(_material)
    {
        center = _center;
        radius = _radius;
        material = _material;
    }

    ~Sphere() override = default;

    bool intersect(const Ray &r, Hit &h, float tmin) override
    {
        Vector3f O, Dir, H, P;
        bool into = true;
        float flag = 1;
        O = r.getOrigin();
        Dir = r.getDirection();
        float t = Vector3f::dot(Dir, center - O);
        H = r.pointAtParameter(t);
        float CH = (H - center).length();
        if (CH > radius - eps)
            return false;
        assert(radius * radius - CH * CH >= 0);
        float tmp = sqrt(radius * radius - CH * CH);
        t = t - tmp;
        if (t < eps)
        {
            t += 2 * tmp;
            flag = -1;
            into = false;
        }
        if (t < tmin)
            return false;
        if (t > h.getT())
            return false;
        P = r.pointAtParameter(t);
        h = Hit(t, material, (P - center).normalized() * flag, into);
        return true;
    }

protected:
    Vector3f center;
    float radius;
    Material *material;
};

#endif
