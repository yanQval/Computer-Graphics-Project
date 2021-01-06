#ifndef MESH_H
#define MESH_H

#include <vector>
#include "object3d.hpp"
#include "triangle.hpp"
#include "Vector2f.h"
#include "Vector3f.h"

class Mesh : public Object3D
{

public:
    Mesh(const char *filename, Material *m);

    struct TriangleIndex
    {
        TriangleIndex()
        {
            x[0] = 0;
            x[1] = 0;
            x[2] = 0;
        }
        int &operator[](const int i) { return x[i]; }
        // By Computer Graphics convention, counterclockwise winding is front face
        int x[3]{};
    };

    struct Info
    {
        Vector3f rangeMin, rangeMax;
        TriangleIndex data;
        Vector3f normal;
    };

    std::vector<Vector3f> v;
    std::vector<TriangleIndex> t;
    std::vector<Vector3f> n;
    bool intersect(const Ray &r, Hit &h, double tmin) override;

private:
    // Normal can be used for light estimation
    void computeNormal();

    struct TreeNode
    {
        TreeNode *lc, *rc;
        Vector3f rangeMax, rangeMin;
        TriangleIndex data;
        Vector3f normal;
    };
    TreeNode *root;

    vector<Info> infoData;

    void build_mesh(TreeNode *&x, int l, int r, int depth, vector<Info> &t);
    bool query_mesh(TreeNode *x, const Ray &r, Hit &h, double tmin);
};

#endif
