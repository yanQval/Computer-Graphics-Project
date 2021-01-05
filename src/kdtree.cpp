#include "kdtree.hpp"
#include "hitpoint.hpp"
#include "utils.hpp"

#include <vector>
#include <map>

using namespace std;

int flag;

bool cmp(const HitPoint *x, const HitPoint *y)
{
    return x->pos[flag] < y->pos[flag];
}

void KDTree::build(TreeNode *&x, int l, int r, int depth)
{
    if (l > r)
    {
        x = NULL;
        return;
    }
    flag = depth % 3;
    int m = (l + r) >> 1;
    nth_element(data.begin() + l, data.begin() + m, data.begin() + r + 1, cmp);
    x = new TreeNode;
    x->pointer = data[m];
    x->Rmax = x->pointer->radius;
    x->rangeMin = x->rangeMax = x->pointer->pos;
    build(x->lc, l, m - 1, depth + 1);
    build(x->rc, m + 1, r, depth + 1);
    if (x->lc != NULL)
    {
        x->rangeMin = min(x->rangeMin, x->lc->rangeMin);
        x->rangeMax = max(x->rangeMax, x->lc->rangeMax);
        x->Rmax = max(x->Rmax, x->lc->Rmax);
    }
    if (x->rc != NULL)
    {
        x->rangeMin = min(x->rangeMin, x->rc->rangeMin);
        x->rangeMax = max(x->rangeMax, x->rc->rangeMax);
        x->Rmax = max(x->Rmax, x->rc->Rmax);
    }
}

KDTree::KDTree(vector<HitPoint> *hitPoints)
{
    for (int i = 0; i < hitPoints->size(); i++)
    {
        HitPoint hitPoint = (*hitPoints)[i];
        data.push_back(&(*hitPoints)[i]);
    }
    int n = data.size();
    build(root, 0, n - 1, 0);
}

void KDTree::query(TreeNode *x, Vector3f q_pos, vector<HitPoint *> *result)
{
    if (x == NULL)
        return;
    if (max(Vector3f::ZERO, max(q_pos - x->rangeMax, x->rangeMin - q_pos)).length() > x->Rmax)
        return;
    if ((q_pos - x->pointer->pos).length() < x->pointer->radius)
        result->push_back(x->pointer);
    query(x->lc, q_pos, result);
    query(x->rc, q_pos, result);
}

void KDTree::query(Vector3f q_pos, vector<HitPoint *> *result)
{
    result->clear();
    query(root, q_pos, result);
}