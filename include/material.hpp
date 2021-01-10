#ifndef MATERIAL_H
#define MATERIAL_H

#include <cassert>
#include <vecmath.h>

#include "ray.hpp"
#include "hit.hpp"
#include "image.hpp"
#include <iostream>

typedef int MaterialType;

#define DIFUSE 0
#define SPEC 1
#define REFL 2

// TODO: Implement Shade function that computes Phong introduced in class.
class Material
{
public:
    explicit Material(const Vector3f &col, char *filename, const MaterialType tp, double re = 0, const Vector3f &e = Vector3f::ZERO) : color(col), type(tp), reflRate(re), emi(e)
    {
        if (filename[0] != 0)
        {
            useTexture = true;
            printf("%s\n", filename);
            texture = Image::LoadPPM(filename);
        }
    }

    virtual ~Material() = default;

    Vector3f getColor(Vector2f pos)
    {
        if (useTexture)
        {
            double x = pos.x() - floor(pos.x());
            double y = pos.y() - floor(pos.y());
            //pos.print();
            //printf("%.5lf %.5lf\n", x, y);
            //printf("%d %d\n", int(x * texture->Width()), int(y * texture->Height()));
            //texture->GetPixel(int(x * texture->Width()), int(y * texture->Height())).print();
            return texture->GetPixel(int(x * texture->Width()), int(y * texture->Height()));
        }
        else
        {
            return color;
        }
    }

    Vector3f getEmission()
    {
        return emi;
    }

    MaterialType getType()
    {
        return type;
    }

    double getReflRate()
    {
        return reflRate;
    }

protected:
    Vector3f color;
    MaterialType type;
    double reflRate;
    Vector3f emi;

    bool useTexture;
    Image *texture;
};

#endif // MATERIAL_H
