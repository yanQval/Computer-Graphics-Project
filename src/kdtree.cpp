#include "kdtree.hpp"
#include "hitpoint.hpp"
#include "utils.hpp"

#include <vector>
#include <map>

using namespace std;

KDTree::KDTree(vector<HitPoint> *hitPoints)
{
    int id = 0;
    for (int i = 0; i < hitPoints->size(); i++)
    {
        HitPoint hitPoint = (*hitPoints)[i];
        pos.push_back(hitPoint.pos);
        int x = int(hitPoint.pos.x());
        int y = int(hitPoint.pos.y());
        int z = int(hitPoint.pos.z());
        mp[make_pair(make_pair(x, y), z)].push_back(&(*hitPoints)[i]);
    }
}

void KDTree::query(Vector3f q_pos, float Rmax, vector<HitPoint *> *result)
{
    result->clear();
    int x = int(q_pos.x());
    int y = int(q_pos.y());
    int z = int(q_pos.z());
    for (int tx = x - 6; tx <= x + 5; tx++)
        for (int ty = y - 6; ty <= y + 5; ty++)
            for (int tz = z - 6; tz <= z + 5; tz++)
                if (dis(Vector3f(tx, ty, tz), q_pos) < Rmax + 1.5)
                    for (auto x : mp[make_pair(make_pair(tx, ty), tz)])
                        result->push_back(x);
}