#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "object3d.hpp"
#include "constant.h"
#include <vecmath.h>
#include <cmath>
#include <iostream>
using namespace std;

// TODO: implement this class and add more fields as necessary,
class Triangle : public Object3D
{

public:
	Triangle() = delete;

	// a b c are three vertex positions of the triangle
	Triangle(const Vector3f &a, const Vector3f &b, const Vector3f &c, const bool vn, Material *m) : Object3D(m)
	{
		vertices[0] = a;
		vertices[1] = b;
		vertices[2] = c;
		material = m;
		normal = Vector3f::cross(b - a, c - a);
		normal = normal.normalized();
		d = Vector3f::dot(normal, a);

		hasVN = vn;
	}

	bool intersect(const Ray &ray, Hit &hit, double tmin) override
	{
		Vector3f O, R, P;
		double t;
		O = ray.getOrigin();
		R = ray.getDirection();
		if (fabs(Vector3f::dot(normal, R)) < 1e-5)
			return false;
		t = (d - Vector3f::dot(normal, O)) / Vector3f::dot(normal, R);
		if (t < tmin)
			return false;
		P = ray.pointAtParameter(t);
		if (Vector3f::dot(normal, Vector3f::cross(vertices[1] - P, vertices[2] - P)) < 0)
			return false;
		if (Vector3f::dot(normal, Vector3f::cross(vertices[2] - P, vertices[0] - P)) < 0)
			return false;
		if (Vector3f::dot(normal, Vector3f::cross(vertices[0] - P, vertices[1] - P)) < 0)
			return false;
		//puts("!!!");
		//printf("%.5f %.5f\n", t, hit.getT());
		//normal.print();
		//R.print();
		//printf("%.5f\n", Vector3f::dot(normal, R));
		if (t > hit.getT())
			return false;
		bool into = (Vector3f::dot(normal, R) < 0);
		if (hasVN)
		{
			double a, b, c;
			a = Vector3f::cross(vertices[1] - P, vertices[2] - P).length();
			b = Vector3f::cross(vertices[2] - P, vertices[0] - P).length();
			c = Vector3f::cross(vertices[0] - P, vertices[1] - P).length();
			Vector3f vnormal = (vertices_normal[0] * a / (a + b + c) + vertices_normal[1] * b / (a + b + c) + vertices_normal[2] * c / (a + b + c)).normalized();

			assert(Vector3f::dot(vnormal, normal) > 0);
			/*puts("!!");
			vertices[0].print();
			vertices[1].print();
			vertices[2].print();
			P.print();
			vertices_normal[0].print();
			vertices_normal[1].print();
			vertices_normal[2].print();
			printf("%.5lf %.5lf %.5lf\n", a, b, c);
			normal.print();
			vnormal.print();*/
			hit = Hit(t, material, into ? vnormal : -vnormal, into);
		}
		else
		{
			hit = Hit(t, material, into ? normal : -normal, into);
		}
		return true;
	}

	Vector3f normal;
	Vector3f vertices_normal[3];

protected:
	Vector3f vertices[3];
	Material *material;
	double d;

	bool hasVN;
};

#endif //TRIANGLE_H
