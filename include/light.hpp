#ifndef LIGHT_H
#define LIGHT_H

#include <Vector3f.h>
#include "object3d.hpp"
#include "utils.hpp"

class Light
{
public:
    Light() = default;

    virtual ~Light() = default;

    virtual void generatePhoton(Ray &r, Vector3f &col, mt19937 *mt_rand) const = 0;
};

class DirectionalLight : public Light
{
public:
    DirectionalLight() = delete;

    DirectionalLight(const Vector3f &d, const Vector3f &c)
    {
        direction = d.normalized();
        color = c;
    }

    ~DirectionalLight() override = default;

    ///@param p unsed in this function
    ///@param distanceToLight not well defined because it's not a point light
    void generatePhoton(Ray &r, Vector3f &col, mt19937 *mt_rand) const override
    {
        // the direction to the light is the opposite of the
        // direction of the directional light source
        //dir = -direction;
        col = color;
    }

private:
    Vector3f direction;
    Vector3f color;
};

class PointLight : public Light
{
public:
    PointLight() = delete;

    PointLight(const Vector3f &p, const Vector3f &c)
    {
        position = p;
        color = c;
    }

    ~PointLight() override = default;

    void generatePhoton(Ray &r, Vector3f &col, mt19937 *mt_rand) const override
    {
        // the direction to the light is the opposite of the
        // direction of the directional light source
        Vector3f dir = uniformSample(mt_rand);
        r = Ray(position, dir);
        col = color;
    }

private:
    Vector3f position;
    Vector3f color;
};

class CircleLight : public Light
{
public:
    CircleLight() = delete;

    CircleLight(const Vector3f &p, const float &r, const Vector3f &n, const Vector3f &c)
    {
        position = p;
        radius = r;
        normal = n;
        color = c;
    }

    ~CircleLight() override = default;

    void generatePhoton(Ray &r, Vector3f &col, mt19937 *mt_rand) const override
    {
        // the direction to the light is the opposite of the
        // direction of the directional light source
        Vector3f dir = semi_uniformSample(normal, mt_rand);
        Vector3f u = Vector3f::cross((fabs(normal.x()) > .1 ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0)), normal).normalized();
        Vector3f v = Vector3f::cross(normal, u).normalized();
        float x, y;
        do
        {
            x = (frand(mt_rand) - 0.5) * 2;
            y = (frand(mt_rand) - 0.5) * 2;
        } while (x * x + y * y > 1);

        Vector3f w = x * u + y * v;

        r = Ray(position + w * radius, dir);
        col = color * Vector3f::dot(dir, normal) / dir.length() / normal.length();
        assert(Vector3f::dot(w, normal) < 1e-2);
        //w.print();
    }

private:
    Vector3f position;
    float radius;
    Vector3f normal;
    Vector3f color;
};

#endif // LIGHT_H
