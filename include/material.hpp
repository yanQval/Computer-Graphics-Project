#ifndef MATERIAL_H
#define MATERIAL_H

#include <cassert>
#include <vecmath.h>

#include "ray.hpp"
#include "hit.hpp"
#include <iostream>

typedef int MaterialType;

#define DIFUSE 0
#define SPEC 1
#define REFL 2

// TODO: Implement Shade function that computes Phong introduced in class.
class Material
{
public:
    explicit Material(const Vector3f &col, const MaterialType tp, float re = 0, const Vector3f &e = Vector3f::ZERO) : color(col), type(tp), reflRate(re), emi(e)
    {
    }

    virtual ~Material() = default;

    Vector3f getColor()
    {
        return color;
    }

    Vector3f getEmission()
    {
        return emi;
    }

    MaterialType getType()
    {
        return type;
    }

    float getReflRate()
    {
        return reflRate;
    }

    /*virtual Vector3f getDiffuseColor() const
    {
        return diffuseColor;
    }*/

    /*Vector3f Shade(const Ray &ray, const Hit &hit,
                   const Vector3f &dirToLight, const Vector3f &lightColor)
    {
        Vector3f shaded = Vector3f::ZERO;
        float t1 = Vector3f::dot(dirToLight, hit.getNormal());
        if (t1 < 0)
            t1 = 0;
        Vector3f R;
        R = 2 * t1 * hit.getNormal() - dirToLight;
        float t2 = Vector3f::dot(-ray.getDirection(), R);
        if (t2 < 0)
            t2 = 0;
        t2 = pow(t2, shininess);
        Vector3f t;
        t = t1 * diffuseColor + t2 * specularColor;
        shaded = lightColor * t;
        return shaded;
    }*/

protected:
    Vector3f color;
    MaterialType type;
    float reflRate;
    Vector3f emi;
};

#endif // MATERIAL_H
