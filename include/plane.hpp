#ifndef PLANE_H
#define PLANE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

// TODO: Implement Plane representing an infinite plane
// function: ax+by+cz=d
// choose your representation , add more fields and fill in the functions

class Plane : public Object3D {
public:
    Plane() = delete;

    Plane(const Vector3f &_normal, float _d, Material *m) : Object3D(m) {
        assert(_normal.length() == 1);
        normal = _normal;
        d = _d;
        material = m;
    }

    ~Plane() override = default;

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        Vector3f O, R; float t;
        O = r.getOrigin();
        R = r.getDirection();
        t = (d - Vector3f::dot(normal, O)) / Vector3f::dot(normal, R);
        if (t < tmin) return false;
        if (t > h.getT()) return false;
        bool into = (Vector3f::dot(normal,R) < 0);
        h = Hit(t, material, into ? normal : -normal, into);
        //printf("%.4lf\n", t);
        return true;
    }

protected:
    Vector3f normal;
    float d;
    Material *material;

};

#endif //PLANE_H
		

