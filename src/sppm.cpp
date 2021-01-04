#include "sppm.hpp"
#include "image.hpp"
#include "scene_parser.hpp"
#include "camera.hpp"
#include "hitpoint.hpp"
#include "utils.hpp"
#include "constant.h"
#include "group.hpp"
#include "light.hpp"
#include "kdtree.hpp"

#include <vector>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>
#include <random>

using namespace std;

const string tmpOut = "output/tmp.bmp";

SPPM::SPPM(SceneParser *scene)
{
    sceneParser = scene;
}

SPPM::~SPPM()
{
    delete sceneParser;
}

void rayTracing(const Ray &r, int depth, const SceneParser *sceneParser, vector<HitPoint> *hitPoints, int _x, int _y, Vector3f col, mt19937 *mt_rand)
{
    Group *basegroup = sceneParser->getGroup();
    Hit hit;
    bool isIntersect = basegroup->intersect(r, hit, eps);
    if (!isIntersect)
    {
        return;
    }
    Vector3f x = r.getOrigin() + r.getDirection() * hit.getT();
    Vector3f c = hit.getMaterial()->getColor() * col;
    Vector3f n = hit.getNormal().normalized();

    float p = max(max(hit.getMaterial()->getColor().x(), hit.getMaterial()->getColor().y()), hit.getMaterial()->getColor().z());
    //float p = max(max(c.x(), c.y()), c.z());
    //printf("%d %.5f\n", depth, p);
    if (++depth > 4)
    {
        if (frand(mt_rand) < p && depth < 100)
            c = c * (1 / p);
        else
            return;
    }
    if (hit.getMaterial()->getType() == DIFUSE)
    {
        HitPoint hitPoint(x, n, r.getDirection(), _x, _y, c);
        hitPoints->push_back(hitPoint);

        return;
    }
    else if (hit.getMaterial()->getType() == SPEC)
    {
        rayTracing(Ray(x, r.getDirection() - n * 2 * Vector3f::dot(n, (r.getDirection()))), depth, sceneParser, hitPoints, _x, _y, c, mt_rand);
    }
    else if (hit.getMaterial()->getType() == REFL)
    {
        bool into = hit.getInto();
        float nc = 1.;
        float nt = hit.getMaterial()->getReflRate();
        float nnt = into ? nc / nt : nt / nc;
        float ddn = Vector3f::dot(r.getDirection(), n);
        float cos2t = 1 + nnt * nnt * (ddn * ddn - 1);
        Ray reflRay(x, r.getDirection() - n * 2 * Vector3f::dot(n, r.getDirection()));
        if (cos2t < 0)
        {
            rayTracing(Ray(x, r.getDirection() - n * 2 * Vector3f::dot(n, (r.getDirection()))), depth, sceneParser, hitPoints, _x, _y, c, mt_rand);
            return;
        }
        Vector3f tdir = (r.getDirection() * nnt - n * (ddn * nnt + sqrt(cos2t))).normalized();
        float a = nt - nc, b = nt + nc, R0 = a * a / (b * b), tc = 1 - (into ? -ddn : -Vector3f::dot(tdir, n));
        float Re = R0 + (1 - R0) * tc * tc * tc * tc * tc, Tr = 1 - Re, P = .25 + .5 * Re, RP = Re / P, TP = Tr / (1 - P);
        if (depth > 4)
        {
            if (frand(mt_rand) < P)
                rayTracing(reflRay, depth, sceneParser, hitPoints, _x, _y, c * RP, mt_rand);
            else
                rayTracing(Ray(x, tdir), depth, sceneParser, hitPoints, _x, _y, c * TP, mt_rand);
        }
        else
        {
            rayTracing(reflRay, depth, sceneParser, hitPoints, _x, _y, c * Re, mt_rand);
            rayTracing(Ray(x, tdir), depth, sceneParser, hitPoints, _x, _y, c * Tr, mt_rand);
        }
    }
}

void photonTracing(const Ray &r, int depth, const SceneParser *sceneParser, vector<HitPoint> *hitPoints, Vector3f col, KDTree *kdt, float Rmax, mt19937 *mt_rand)
{
    //printf("%d\n", depth);
    Group *basegroup = sceneParser->getGroup();
    Hit hit;
    bool isIntersect = basegroup->intersect(r, hit, eps);
    if (!isIntersect)
    {
        return;
    }
    Vector3f x = r.getOrigin() + r.getDirection() * hit.getT();
    Vector3f c = col * hit.getMaterial()->getColor();
    Vector3f n = hit.getNormal().normalized();

    float p = max(max(hit.getMaterial()->getColor().x(), hit.getMaterial()->getColor().y()), hit.getMaterial()->getColor().z());
    if (++depth > 4)
    {
        if (frand(mt_rand) < p && depth < 100)
            c = c * (1 / p);
        else
            return;
    }
    if (hit.getMaterial()->getType() == DIFUSE)
    {
        HitPoint hitPoint(x, n, r.getDirection(), 0, 0, col);
        vector<int> indices;
        indices.clear();
        kdt->query(x, Rmax, &indices);
        for (auto index : indices)
        {
            hitPoint.x = index;
            hitPoints->push_back(hitPoint);
        }

        float r1 = 2 * M_PI * frand(mt_rand), r2 = frand(mt_rand), r2s = sqrt(r2);
        assert(r2s >= 0);

        Vector3f w = n, u = Vector3f::cross((fabs(w.x()) > .1 ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0)), w).normalized(), v = Vector3f::cross(w, u);
        Vector3f d = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2)).normalized();
        photonTracing(Ray(x, d), depth, sceneParser, hitPoints, c, kdt, Rmax, mt_rand);
    }
    else if (hit.getMaterial()->getType() == SPEC)
    {
        photonTracing(Ray(x, r.getDirection() - n * 2 * Vector3f::dot(n, (r.getDirection()))), depth, sceneParser, hitPoints, c, kdt, Rmax, mt_rand);
    }
    else if (hit.getMaterial()->getType() == REFL)
    {
        bool into = hit.getInto();
        float nc = 1.;
        float nt = hit.getMaterial()->getReflRate();
        float nnt = into ? nc / nt : nt / nc;
        float ddn = Vector3f::dot(r.getDirection(), n);
        float cos2t = 1 + nnt * nnt * (ddn * ddn - 1);
        Ray reflRay(x, r.getDirection() - n * 2 * Vector3f::dot(n, r.getDirection()));
        if (cos2t < 0)
        {
            photonTracing(Ray(x, r.getDirection() - n * 2 * Vector3f::dot(n, (r.getDirection()))), depth, sceneParser, hitPoints, c, kdt, Rmax, mt_rand);
            return;
        }
        Vector3f tdir = (r.getDirection() * nnt - n * (ddn * nnt + sqrt(cos2t))).normalized();
        float a = nt - nc, b = nt + nc, R0 = a * a / (b * b), tc = 1 - (into ? -ddn : -Vector3f::dot(tdir, n));
        float Re = R0 + (1 - R0) * tc * tc * tc * tc * tc, Tr = 1 - Re, P = .25 + .5 * Re, RP = Re / P, TP = Tr / (1 - P);
        //printf("%.5f %.5f %.5f %.5f\n", RP, TP, Re, Tr);
        if (depth > 4)
        {
            if (frand(mt_rand) < P)
                photonTracing(reflRay, depth, sceneParser, hitPoints, c * RP, kdt, Rmax, mt_rand);
            else
                photonTracing(Ray(x, tdir), depth, sceneParser, hitPoints, c * TP, kdt, Rmax, mt_rand);
        }
        else
        {
            photonTracing(reflRay, depth, sceneParser, hitPoints, c * Re, kdt, Rmax, mt_rand);
            photonTracing(Ray(x, tdir), depth, sceneParser, hitPoints, c * Tr, kdt, Rmax, mt_rand);
        }
    }
}

Image SPPM::run()
{
    Camera *camera = sceneParser->getCamera();
    printf("%d %d\n", camera->getWidth(), camera->getHeight());
    Image image(camera->getWidth(), camera->getHeight());

    vector<HitPoint> hitPoints;
    hitPoints.clear();

    vector<HitPoint> tmphit[camera->getWidth()];

    int n_threads = 25;

#pragma omp parallel for schedule(dynamic, 1) num_threads(n_threads)
    for (int x = 0; x < camera->getWidth(); x++)
    {
        tmphit[x].clear();
        mt19937 mt_rand(x);
        for (int y = 0; y < camera->getHeight(); y++)
        {
            float px = x + .5;
            float py = y + .5;
            Ray camRay = camera->generateRay(Vector2f(px, py));
            rayTracing(camRay, 0, sceneParser, &tmphit[x], x, y, Vector3f(1, 1, 1), &mt_rand);
        }
    }
    for (int x = 0; x < camera->getWidth(); x++)
    {
        for (auto hitPoint : tmphit[x])
        {
            //hitPoint.print();
            hitPoints.push_back(hitPoint);
            //hitPoint.col.print();
            image.SetPixel(hitPoint.x, hitPoint.y, Vector3f(toInt(hitPoint.col.x()), toInt(hitPoint.col.y()), toInt(hitPoint.col.z())));
        }
    }

    int t_round = 100;
    int num_photons = 300000;
    float alpha = .7;
    float Rmax = 7;

    int n_hitPoints = hitPoints.size();
    printf("%d\n", n_hitPoints);
    KDTree kdtree(&hitPoints);

    for (int round = 1; round <= t_round; round++)
    {
        if (round == 1)
        {
            for (int i = 0; i < n_hitPoints; i++)
                hitPoints[i].radius = Rmax;
        }
        else
        {
            Rmax = 0;
            for (int i = 0; i < n_hitPoints; i++)
                Rmax = max(Rmax, hitPoints[i].radius);
        }

        vector<HitPoint> photonPairs[num_photons];
        for (int li = 0; li < sceneParser->getNumLights(); li++)
        {
            Light *light = sceneParser->getLight(li);
            Vector3f lightColor;
            Ray phoRay(Vector3f::ZERO, Vector3f::ZERO);
#pragma omp parallel for schedule(dynamic, 1) num_threads(n_threads)
            for (int photon = 0; photon < num_photons; photon++)
            {
                mt19937 mt_rand(round * num_photons + photon);
                light->generatePhoton(phoRay, lightColor, &mt_rand);
                photonTracing(phoRay, 0, sceneParser, &photonPairs[photon], lightColor, &kdtree, Rmax, &mt_rand);
            }
        }
        int numpair = 0;
        if (round == 1)
        {
            for (int li = 0; li < sceneParser->getNumLights(); li++)
                for (int photon = 0; photon < num_photons; photon++)
                    for (auto hitpair : photonPairs[photon])
                    {
                        int id = hitpair.x;
                        if (dis(hitpair.pos, hitPoints[id].pos) < hitPoints[id].radius && Vector3f::dot(hitpair.norm, hitPoints[id].norm) > eps)
                        {
                            hitPoints[id].N += 1;
                            hitPoints[id].tau += hitpair.col;
                            numpair++;
                        }
                    }
        }
        else
        {
            for (int li = 0; li < sceneParser->getNumLights(); li++)
                for (int photon = 0; photon < num_photons; photon++)
                    for (auto hitpair : photonPairs[photon])
                    {
                        int id = hitpair.x;
                        if (dis(hitpair.pos, hitPoints[id].pos) < hitPoints[id].radius && Vector3f::dot(hitpair.norm, hitPoints[id].norm) > eps)
                        {
                            hitPoints[id].M += 1;
                            hitPoints[id].tau += hitpair.col;
                            numpair++;
                        }
                    }
            for (int i = 0; i < n_hitPoints; i++)
            {
                int N = hitPoints[i].N;
                int M = hitPoints[i].M;
                if (N + M != 0)
                {
                    hitPoints[i].radius *= sqrt((N + alpha * M) / (N + M));
                    hitPoints[i].tau *= (N + alpha * M) / (N + M);
                    hitPoints[i].N = N + alpha * M;
                    hitPoints[i].M = 0;
                }
            }
        }
        printf("hit photon %d \n", numpair);
        image.SetAllPixels(Vector3f::ZERO);
        for (auto hitPoint : hitPoints)
        {
            int x = hitPoint.x, y = hitPoint.y;
            Vector3f color = image.GetPixel(x, y);
            color += hitPoint.col * hitPoint.tau / M_PI / hitPoint.radius / hitPoint.radius / (num_photons * round);
            /*if (color.x() > 0.5)
            {
                printf("%d %d %.5f %.5f\n", x, y, hitPoint.N, hitPoint.radius);
                hitPoint.tau.print();
                hitPoint.pos.print();
                hitPoint.col.print();
                color.print();
            }*/
            image.SetPixel(x, y, color);
        }
        for (int x = 0; x < camera->getWidth(); x++)
        {
            for (int y = 0; y < camera->getHeight(); y++)
            {
                Vector3f color = image.GetPixel(x, y);
                image.SetPixel(x, y, Vector3f(toInt(color.x()), toInt(color.y()), toInt(color.z())));
            }
        }
        printf("round : %d\n", round);
        image.SaveBMP(tmpOut.c_str());
    }

    return image;
}