#ifndef CAMERA_H
#define CAMERA_H

#include "ray.hpp"
#include <vecmath.h>
#include <cmath>


class Camera {
public:
    Camera(const Vector3f &center, const Vector3f &direction, const Vector3f &up, int imgW, int imgH) {
        this->center = center;
        this->direction = direction.normalized();
        this->horizontal = Vector3f::cross(this->direction, up);
        this->up = Vector3f::cross(this->horizontal, this->direction);
        this->width = imgW;
        this->height = imgH;
    }

    // Generate rays for each screen-space coordinate
    virtual Ray generateRay(const Vector2f &point) = 0;
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
class PerspectiveCamera : public Camera {

public:
    PerspectiveCamera(const Vector3f &_center, const Vector3f &_direction,
            const Vector3f &_up, int _imgW, int _imgH, double _angle) : Camera(_center, _direction, _up, _imgW, _imgH) {
        angle = _angle;
        distance = height / 2 / tan(angle / 2);
        //printf("%d %d %.4lf\n", _imgW, _imgH, distance);
        // angle is in radian.
    }

    Ray generateRay(const Vector2f &point) override {
        Vector3f O, Dir;
        O = Vector3f(0, 0, 0);
        Dir = Vector3f(point[0] - width / 2, height / 2 - point[1], distance).normalized();
        Matrix3f trans(horizontal, -up, direction, true);
        Dir = trans * Dir;
        return Ray(center, Dir);
    }
    double angle;
    double distance;
};

#endif //CAMERA_H
