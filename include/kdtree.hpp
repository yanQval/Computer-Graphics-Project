#ifndef KDT_H
#define KDT_H

#include "Vector3f.h"
#include "hitpoint.hpp"

#include <vector>
#include <map>

using namespace std;

struct TreeNode
{
    HitPoint *pointer;
    TreeNode *lc, *rc;
    float Rmax;
    Vector3f rangeMax, rangeMin;
};

class KDTree
{
public:
    KDTree() = delete;
    KDTree(vector<HitPoint> *hitPoints);
    void query(Vector3f pos, vector<HitPoint *> *result);

private:
    void build(TreeNode *&x, int l, int r, int depth);
    void query(TreeNode *x, Vector3f q_pos, vector<HitPoint *> *result);
    vector<HitPoint *> data;
    TreeNode *root;
};

#endif