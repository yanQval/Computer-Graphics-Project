#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>
#include <iostream>
using namespace std;

// TODO: implement this class and add more fields as necessary,
class Triangle: public Object3D {

public:
	Triangle() = delete;

    // a b c are three vertex positions of the triangle
	Triangle( const Vector3f& a, const Vector3f& b, const Vector3f& c, Material* m) : Object3D(m) {
		vertices[0] = a;
		vertices[1] = b;
		vertices[2] = c;
		material = m;
		normal = Vector3f::cross(b - a, c - a);
		normal = normal.normalized();
		d = Vector3f::dot(normal, a);
	}

	bool intersect( const Ray& ray,  Hit& hit , float tmin) override {
		Vector3f O, R, P; float t;
        O = ray.getOrigin();
        R = ray.getDirection();
        t = (d - Vector3f::dot(normal, O)) / Vector3f::dot(normal, R);
        if (t < tmin) return false;
		P = ray.pointAtParameter(t);
		if (Vector3f::dot(normal, Vector3f::cross(vertices[1] - P, vertices[2] - P)) < 0) return false;
		if (Vector3f::dot(normal, Vector3f::cross(vertices[2] - P, vertices[0] - P)) < 0) return false;
		if (Vector3f::dot(normal, Vector3f::cross(vertices[0] - P, vertices[1] - P)) < 0) return false;
        if (t > hit.getT()) return false;
		hit = Hit(t, material, normal);
        return true;
	}
	
	Vector3f normal;
protected:
	Vector3f vertices[3];
	Material *material;
	float  d;
};

#endif //TRIANGLE_H