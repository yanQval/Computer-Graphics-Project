#ifndef HIT_H
#define HIT_H

#include <vecmath.h>
#include "ray.hpp"

class Material;

class Hit
{
public:
    // constructors
    Hit()
    {
        material = nullptr;
        t = 1e38;
    }

    Hit(double _t, Material *m, const Vector3f &n, const bool in)
    {
        t = _t;
        material = m;
        normal = n;
        into = in;
    }

    Hit(const Hit &h)
    {
        t = h.t;
        material = h.material;
        normal = h.normal;
    }

    // destructor
    ~Hit() = default;

    double getT() const
    {
        return t;
    }

    Material *getMaterial() const
    {
        return material;
    }

    const Vector3f &getNormal() const
    {
        return normal;
    }

    bool getInto() const
    {
        return into;
    }

    void set(double _t, Material *m, const Vector3f &n, const bool &in)
    {
        t = _t;
        material = m;
        normal = n;
        into = in;
    }

    Vector2f texPos;

private:
    double t;
    Material *material;
    Vector3f normal;
    bool into;
};

inline std::ostream &operator<<(std::ostream &os, const Hit &h)
{
    os << "Hit <" << h.getT() << ", " << h.getNormal() << ">";
    return os;
}

#endif // HIT_H
