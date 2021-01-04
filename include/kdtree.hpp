#ifndef KDT_H
#define KDT_H

#include "Vector3f.h"
#include "hitpoint.hpp"

#include <vector>
#include <map>

using namespace std;

class KDTree
{
public:
    KDTree() = delete;
    KDTree(const vector<HitPoint> *hitPoints);
    void query(Vector3f pos, float Rmax, vector<int> *result);

private:
    vector<Vector3f> pos;
    map<pair<pair<int, int>, int>, vector<int>> mp;
};

#endif