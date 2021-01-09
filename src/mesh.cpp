#include "mesh.hpp"
#include "utils.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <utility>
#include <sstream>

int flag_mesh;

bool cmp_mesh(const Mesh::Info x, const Mesh::Info y)
{
    return x.rangeMin[flag_mesh] < y.rangeMin[flag_mesh];
}

void Mesh::build_mesh(TreeNode *&x, int l, int r, int depth, vector<Info> &t)
{
    if (l > r)
    {
        x = NULL;
        return;
    }
    flag_mesh = depth % 3;
    int m = (l + r) >> 1;
    nth_element(t.begin() + l, t.begin() + m, t.begin() + r + 1, cmp_mesh);
    x = new TreeNode;
    x->data = t[m].data;
    if (hasVN)
        x->vnID = t[m].vnID;
    x->normal = t[m].normal;
    x->rangeMin = t[m].rangeMin;
    x->rangeMax = t[m].rangeMax;
    build_mesh(x->lc, l, m - 1, depth + 1, t);
    build_mesh(x->rc, m + 1, r, depth + 1, t);
    if (x->lc != NULL)
    {
        x->rangeMin = min(x->rangeMin, x->lc->rangeMin);
        x->rangeMax = max(x->rangeMax, x->lc->rangeMax);
    }
    if (x->rc != NULL)
    {
        x->rangeMin = min(x->rangeMin, x->rc->rangeMin);
        x->rangeMax = max(x->rangeMax, x->rc->rangeMax);
    }
}

void update(double &t_min, double &t_max, double t0, double t1)
{
    if (t0 > t1)
        swap(t0, t1);
    t_min = max(t_min, t0);
    t_max = min(t_max, t1);
}

bool Mesh::query_mesh(TreeNode *x, const Ray &r, Hit &h, double tmin)
{
    //printf("%p\n", x);
    if (x == NULL)
        return false;
    double t_min = tmin, t_max = h.getT();
    Vector3f a = (x->rangeMin - r.getOrigin()) / r.getDirection();
    Vector3f b = (x->rangeMax - r.getOrigin()) / r.getDirection();
    update(t_min, t_max, a[0], b[0]);
    update(t_min, t_max, a[1], b[1]);
    update(t_min, t_max, a[2], b[2]);
    if (t_min > t_max)
        return false;
    Triangle triangle(v[x->data[0]],
                      v[x->data[1]], v[x->data[2]], hasVN, material);
    if (hasVN)
    {
        triangle.vertices_normal[0] = vn[x->vnID[0]];
        triangle.vertices_normal[1] = vn[x->vnID[1]];
        triangle.vertices_normal[2] = vn[x->vnID[2]];
    }
    triangle.normal = x->normal;
    bool result = triangle.intersect(r, h, tmin);
    result |= query_mesh(x->lc, r, h, tmin);
    result |= query_mesh(x->rc, r, h, tmin);
    //if(result == false){puts("???");printf("%.5f %.5f\n", t_min, t_max);}
    //else puts("!!!!");
    return result;
}

bool Mesh::intersect(const Ray &r, Hit &h, double tmin)
{
    //Hit h2 = h;
    //bool result1 = query_mesh(root, r, h2, tmin);
    return query_mesh(root, r, h, tmin);
    // Optional: Change this brute force method into a faster one.
    bool result = false;
    for (int triId = 0; triId < (int)t.size(); ++triId)
    {
        TriangleIndex &triIndex = t[triId];
        Triangle triangle(v[triIndex[0]],
                          v[triIndex[1]], v[triIndex[2]], hasVN, material);
        if (hasVN)
        {
            triangle.vertices_normal[0] = vn[triIndex[0]];
            triangle.vertices_normal[1] = vn[triIndex[1]];
            triangle.vertices_normal[2] = vn[triIndex[2]];
        }
        triangle.normal = n[triId];
        result |= triangle.intersect(r, h, tmin);
    }
    //assert(result == result1);
    //assert(fabs(h2.getT() - h.getT()) < 1e-2);
    return result;
}

Mesh::Mesh(const char *filename, Material *material) : Object3D(material)
{

    // Optional: Use tiny obj loader to replace this simple one.
    std::ifstream f;
    f.open(filename);
    if (!f.is_open())
    {
        std::cout << "Cannot open " << filename << "\n";
        return;
    }
    std::string line;
    std::string vTok("v");
    std::string vnTok("vn");
    std::string fTok("f");
    std::string texTok("vt");
    char bslash = '/', space = ' ';
    string dslash = "//";
    std::string tok;
    int texID;
    while (true)
    {
        std::getline(f, line);
        if (f.eof())
        {
            break;
        }
        if (line.size() < 3)
        {
            continue;
        }
        if (line.at(0) == '#')
        {
            continue;
        }
        std::stringstream ss(line);
        ss >> tok;
        if (tok == vTok)
        {
            Vector3f vec;
            ss >> vec[0] >> vec[1] >> vec[2];
            v.push_back(vec);
        }
        else if (tok == vnTok)
        {
            hasVN = true;
            Vector3f vec;
            ss >> vec[0] >> vec[1] >> vec[2];
            vn.push_back(vec);
        }
        else if (tok == fTok)
        {
            if (line.find(dslash) != std::string::npos)
            {
                std::replace(line.begin(), line.end(), bslash, space);
                std::stringstream facess(line);
                TriangleIndex trig, nor;
                facess >> tok;
                for (int ii = 0; ii < 3; ii++)
                {
                    facess >> trig[ii] >> nor[ii];
                    trig[ii]--;
                    nor[ii]--;
                }
                t.push_back(trig);
                tn.push_back(nor);
            }
            else if (line.find(bslash) != std::string::npos)
            {
                std::replace(line.begin(), line.end(), bslash, space);
                std::stringstream facess(line);
                TriangleIndex trig;
                facess >> tok;
                for (int ii = 0; ii < 3; ii++)
                {
                    facess >> trig[ii] >> texID;
                    trig[ii]--;
                }
                t.push_back(trig);
            }
            else
            {
                TriangleIndex trig;
                for (int ii = 0; ii < 3; ii++)
                {
                    ss >> trig[ii];
                    trig[ii]--;
                }
                t.push_back(trig);
            }
        }
        else if (tok == texTok)
        {
            Vector2f texcoord;
            ss >> texcoord[0];
            ss >> texcoord[1];
        }
    }

    printf("has vn %d %d %d\n", hasVN, (int)v.size(), (int)vn.size());
    if (hasVN)
    {
        assert(v.size() == vn.size());
    }

    computeNormal();

    int _n = t.size();

    infoData.clear();
    for (int i = 0; i < _n; i++)
    {
        TriangleIndex x = t[i];
        Info info;
        info.data = x;
        if (hasVN)
            info.vnID = tn[i];
        info.rangeMin = min(min(v[x[0]], v[x[1]]), v[x[2]]);
        info.rangeMax = max(max(v[x[0]], v[x[1]]), v[x[2]]);
        info.normal = n[i];
        infoData.push_back(info);
    }
    puts("!!!");
    build_mesh(root, 0, _n - 1, 0, infoData);

    f.close();
}

void Mesh::computeNormal()
{
    n.resize(t.size());
    for (int triId = 0; triId < (int)t.size(); ++triId)
    {
        TriangleIndex &triIndex = t[triId];
        Vector3f a = v[triIndex[1]] - v[triIndex[0]];
        Vector3f b = v[triIndex[2]] - v[triIndex[0]];
        b = Vector3f::cross(a, b);
        n[triId] = b / b.length();
    }
}
