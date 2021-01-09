#ifndef CAMERA_H
#define CAMERA_H

#include "ray.hpp"
#include "utils.hpp"
#include <vecmath.h>
#include <cmath>
#include <random>

using namespace std;

class Camera
{
public:
    Camera(const Vector3f &center, const Vector3f &direction, const Vector3f &up, int imgW, int imgH)
    {
        this->center = center;
        this->direction = direction.normalized();
        this->horizontal = Vector3f::cross(this->direction, up);
        this->up = Vector3f::cross(this->horizontal, this->direction);
        this->width = imgW;
        this->height = imgH;
    }

    // Generate rays for each screen-space coordinate
    virtual Ray generateRay(const Vector2f &point, mt19937 *mt_rand) = 0;
    virtual ~Camera() = default;

    int getWidth() const { return width; }
    int getHeight() const { return height; }

protected:
    // Extrinsic parameters
    Vector3f center;
    Vector3f direction;
    Vector3f up;
    Vector3f horizontal;
    // Intrinsic parameters
    int width;
    int height;
};

// TODO: Implement Perspective camera
// You can add new functions or variables whenever needed.
class PerspectiveCamera : public Camera
{

public:
    PerspectiveCamera(const Vector3f &_center, const Vector3f &_direction,
                      const Vector3f &_up, int _imgW, int _imgH, double _angle, bool uf, Vector3f fp) : Camera(_center, _direction, _up, _imgW, _imgH)
    {
        angle = _angle;
        distance = height / 2 / tan(angle / 2);
        useFocusPoint = uf;
        focusPoint = fp;
        focusLength = (fp - _center).length();
        radius = focusLength * 0.020;
        //printf("%d %d %.4lf\n", _imgW, _imgH, distance);
        // angle is in radian.
    }

    Ray generateRay(const Vector2f &point, mt19937 *mt_rand) override
    {
        Vector3f O, Dir;
        O = Vector3f(0, 0, 0);
        Dir = Vector3f(point[0] - width / 2, height / 2 - point[1], distance).normalized();
        Matrix3f trans(horizontal, -up, direction, true);
        //puts("!!!");
        //printf("%.5lf\n", Dir.length());
        Dir = (trans * Dir).normalized();
        //printf("%.5lf\n", Dir.length());
        if (useFocusPoint)
        {
            Dir = Dir / Vector3f::dot(Dir, direction);
            Vector3f d = semi_uniformSample(direction, mt_rand).normalized() * radius;
            return Ray(center + d, (Dir * focusLength - d).normalized());
        }
        else
        {
            return Ray(center, Dir);
        }
    }
    double angle;
    double distance;

    bool useFocusPoint;
    Vector3f focusPoint;
    double focusLength;
    double radius;
};

#endif //CAMERA_H
