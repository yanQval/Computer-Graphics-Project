#include "kdtree.hpp"
#include "hitpoint.hpp"
#include "utils.hpp"

#include <vector>
#include <map>

using namespace std;

KDTree::KDTree(const vector<HitPoint> *hitPoints)
{
    int id = 0;
    for (auto hitPoint : *hitPoints)
    {
        pos.push_back(hitPoint.pos);
        int x = int(hitPoint.pos.x());
        int y = int(hitPoint.pos.y());
        int z = int(hitPoint.pos.z());
        mp[make_pair(make_pair(x, y), z)].push_back(id++);
    }
}

void KDTree::query(Vector3f q_pos, float Rmax, vector<int> *result)
{
    result->clear();
    int x = int(q_pos.x());
    int y = int(q_pos.y());
    int z = int(q_pos.z());
    for (int tx = x - 4; tx <= x + 4; tx++)
        for (int ty = y - 4; ty <= y + 4; ty++)
            for (int tz = z - 4; tz <= z + 4; tz++)
                for (auto x : mp[make_pair(make_pair(tx, ty), tz)])
                    result->push_back(x);
}