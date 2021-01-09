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
#include <time.h>

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

    double p = max(max(hit.getMaterial()->getColor().x(), hit.getMaterial()->getColor().y()), hit.getMaterial()->getColor().z());
    //double p = max(max(c.x(), c.y()), c.z());
    //printf("%d %.5f\n", depth, p);
    if (++depth > 50)
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
        double nc = 1.;
        double nt = hit.getMaterial()->getReflRate();
        double nnt = into ? nc / nt : nt / nc;
        double ddn = Vector3f::dot(r.getDirection(), n);
        double cos2t = 1 + nnt * nnt * (ddn * ddn - 1);
        Ray reflRay(x, r.getDirection() - n * 2 * Vector3f::dot(n, r.getDirection()));
        if (cos2t < 0)
        {
            rayTracing(Ray(x, r.getDirection() - n * 2 * Vector3f::dot(n, (r.getDirection()))), depth, sceneParser, hitPoints, _x, _y, c, mt_rand);
            return;
        }
        Vector3f tdir = (r.getDirection() * nnt - n * (ddn * nnt + sqrt(cos2t))).normalized();
        double a = nt - nc, b = nt + nc, R0 = a * a / (b * b), tc = 1 - (into ? -ddn : -Vector3f::dot(tdir, n));
        double Re = R0 + (1 - R0) * tc * tc * tc * tc * tc, Tr = 1 - Re, P = .25 + .5 * Re, RP = Re / P, TP = Tr / (1 - P);
        if (depth > -1)
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

void photonTracing(const Ray &r, int depth, const SceneParser *sceneParser, vector<HitPoint *> *tmp, Vector3f col, KDTree *kdt, mt19937 *mt_rand)
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

    //if(fabs(x[0] - 69) < 1 && fabs(x[1] - 0) < 1 && fabs(x[2] - 101) < 1)puts("!!!");

    double p = max(max(hit.getMaterial()->getColor().x(), hit.getMaterial()->getColor().y()), hit.getMaterial()->getColor().z());
    if (++depth > 20)
    {
        if (frand(mt_rand) < p && depth < 100)
            c = c * (1 / p);
        else
            return;
    }
    if (hit.getMaterial()->getType() == DIFUSE)
    {
        tmp->clear();
        //printf("%d\n",tmp->size());
        kdt->query(x, tmp);
        //printf("%d\n",tmp->size());
        for (HitPoint *pointer : *tmp)
        {
            if (dis(x, pointer->pos) < pointer->radius && Vector3f::dot(n, pointer->norm) > eps)
            {
                pointer->M += 1;
                pointer->tau += col;
            }
        }

        /*Vector3f w = n;
        Vector3f u = Vector3f::cross(fabs(w[0]) > 0.1 ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0), w).normalized();
        Vector3f v = Vector3f::cross(w, u);
        double phi = frand(mt_rand) * M_PI * 2, sinTheta2 = frand(mt_rand), sinTheta = sqrt(sinTheta2), cosTheta = sqrt(max((double)0.0, (double)1.0 - sinTheta2));
        Ray reflectedRay(r.getDirection() * hit.getT() + r.getOrigin(),
         u * cos(phi) * cosTheta + v * sin(phi) * cosTheta + w * sinTheta);
        photonTracing(reflectedRay, depth, sceneParser, tmp, c, kdt, mt_rand);*/
        double r1 = 2 * M_PI * frand(mt_rand), r2 = frand(mt_rand), r2s = sqrt(r2);
        assert(r2s >= 0);

        Vector3f w = n, u = Vector3f::cross((fabs(w.x()) > .1 ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0)), w).normalized(), v = Vector3f::cross(w, u);
        Vector3f d = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2)).normalized();
        photonTracing(Ray(x, d), depth, sceneParser, tmp, c, kdt, mt_rand);
    }
    else if (hit.getMaterial()->getType() == SPEC)
    {
        photonTracing(Ray(x, r.getDirection() - n * 2 * Vector3f::dot(n, (r.getDirection()))), depth, sceneParser, tmp, c, kdt, mt_rand);
    }
    else if (hit.getMaterial()->getType() == REFL)
    {
        bool into = hit.getInto();
        double nc = 1.;
        double nt = hit.getMaterial()->getReflRate();
        double nnt = into ? nc / nt : nt / nc;
        double ddn = Vector3f::dot(r.getDirection(), n);
        double cos2t = 1 + nnt * nnt * (ddn * ddn - 1);
        Ray reflRay(x, r.getDirection() - n * 2 * Vector3f::dot(n, r.getDirection()));
        if (cos2t < 0)
        {
            photonTracing(Ray(x, r.getDirection() - n * 2 * Vector3f::dot(n, (r.getDirection()))), depth, sceneParser, tmp, c, kdt, mt_rand);
            return;
        }
        Vector3f tdir = (r.getDirection() * nnt - n * (ddn * nnt + sqrt(cos2t))).normalized();
        double a = nt - nc, b = nt + nc, R0 = a * a / (b * b), tc = 1 - (into ? -ddn : -Vector3f::dot(tdir, n));
        double Re = R0 + (1 - R0) * tc * tc * tc * tc * tc, Tr = 1 - Re, P = .25 + .5 * Re, RP = Re / P, TP = Tr / (1 - P);
        //printf("%.5f %.5f %.5f %.5f\n", RP, TP, Re, Tr);
        if (depth > -1)
        {
            if (frand(mt_rand) < P)
                photonTracing(reflRay, depth, sceneParser, tmp, c * RP, kdt, mt_rand);
            else
                photonTracing(Ray(x, tdir), depth, sceneParser, tmp, c * TP, kdt, mt_rand);
        }
        else
        {
            photonTracing(reflRay, depth, sceneParser, tmp, c * Re, kdt, mt_rand);
            photonTracing(Ray(x, tdir), depth, sceneParser, tmp, c * Tr, kdt, mt_rand);
        }
    }
}

HitPoint pixelData[1920][1300];

Image SPPM::run()
{
    int n_threads = 30;
    int t_round = 200000;
    int num_photons = 500000;
    num_photons = num_photons / n_threads * n_threads;
    double alpha = .8;
    double Rmax = .3;

    Camera *camera = sceneParser->getCamera();
    printf("%d %d\n", camera->getWidth(), camera->getHeight());
    Image image(camera->getWidth(), camera->getHeight());

    //HitPoint *pixelData = new HitPoint[camera->getWidth()][camera->getHeight()];
    for (int x = 0; x < camera->getWidth(); x++)
        for (int y = 0; y < camera->getHeight(); y++)
        {
            pixelData[x][y].N = 0;
            pixelData[x][y].M = 0;
            pixelData[x][y].radius = Rmax;
            pixelData[x][y].tau = Vector3f::ZERO;
            pixelData[x][y].col = Vector3f::ZERO;
        }

    for (int round = 1; round <= t_round; round++)
    {
        vector<HitPoint> hitPoints;
        hitPoints.clear();

        vector<HitPoint> tmphit[camera->getWidth()];

#pragma omp parallel for schedule(dynamic, 1) num_threads(n_threads)
        for (int x = 0; x < camera->getWidth(); x++)
        {
            //fprintf(stderr, "\rRay Rendering  %5.2f%%", 100. * x / (camera->getWidth() - 1));
            tmphit[x].clear();
            mt19937 mt_rand(round * camera->getWidth() + x + 1000000);
            for (int y = 0; y < camera->getHeight(); y++)
            {
                double px = x + frand(&mt_rand);
                double py = y + frand(&mt_rand);
                Ray camRay = camera->generateRay(Vector2f(px, py), &mt_rand);
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
                //image.SetPixel(hitPoint.x, hitPoint.y, Vector3f(toInt(hitPoint.col.x()), toInt(hitPoint.col.y()), toInt(hitPoint.col.z())));
            }
        }

        //image.SaveBMP(tmpOut.c_str());

        int n_hitPoints = hitPoints.size();
        //printf("%.5lf \n", pixelData[0][0].radius);
        //printf("%.5lf \n", pixelData[0][0].N);
        //pixelData[0][0].tau.print();
        printf("%d\n", n_hitPoints);

        for (int i = 0; i < n_hitPoints; i++)
        {
            int x = hitPoints[i].x, y = hitPoints[i].y;
            hitPoints[i].radius = pixelData[x][y].radius;
        }
        KDTree kdtree(&hitPoints);

        for (int li = 0; li < sceneParser->getNumLights(); li++)
        {
            Light *light = sceneParser->getLight(li);
#pragma omp parallel for schedule(dynamic, 1) num_threads(n_threads)
            for (int th = 0; th < n_threads; th++)
            {
                mt19937 mt_rand(round * n_threads + th + 1000000);
                for (int photon = 0; photon < num_photons / n_threads; photon++)
                {
                    //fprintf(stderr, "\rRendering  %5.2f%%", 100. * photon / (num_photons - 1));
                    Vector3f lightColor;
                    vector<HitPoint *> tmp;
                    Ray phoRay(Vector3f::ZERO, Vector3f::ZERO);
                    light->generatePhoton(phoRay, lightColor, &mt_rand);
                    photonTracing(phoRay, 0, sceneParser, &tmp, lightColor, &kdtree, &mt_rand);
                }
            }
        }

        for (int x = 0; x < camera->getWidth(); x++)
            for (int y = 0; y < camera->getHeight(); y++)
            {
                pixelData[x][y].x = 0;
            }

        for (auto hitPoint : hitPoints)
        {
            int x = hitPoint.x, y = hitPoint.y;
            int &num = pixelData[x][y].x;
            pixelData[x][y].M = (pixelData[x][y].M * num + hitPoint.M) / (num + 1);
            //pixelData[x][y].col = (pixelData[x][y].col * num + hitPoint.tau * hitPoint.col) / (num + 1);
            pixelData[x][y].col = (pixelData[x][y].col + hitPoint.tau * hitPoint.col);
            num++;
        }

        if (round == 1)
        {
            for (int x = 0; x < camera->getWidth(); x++)
                for (int y = 0; y < camera->getHeight(); y++)
                {
                    pixelData[x][y].N = pixelData[x][y].M;
                    pixelData[x][y].M = 0;
                    pixelData[x][y].tau = pixelData[x][y].col;
                    //pixelData[x][y].tau.print();
                    pixelData[x][y].col = Vector3f::ZERO;
                }
        }
        else
        {
            for (int x = 0; x < camera->getWidth(); x++)
                for (int y = 0; y < camera->getHeight(); y++)
                {
                    int N = pixelData[x][y].N;
                    int M = pixelData[x][y].M;
                    if (N + M != 0)
                    {
                        pixelData[x][y].radius *= sqrt((N + alpha * M) / (N + M));
                        pixelData[x][y].tau += pixelData[x][y].col;
                        pixelData[x][y].col = Vector3f::ZERO;
                        pixelData[x][y].tau *= (N + alpha * M) / (N + M);
                        pixelData[x][y].N = N + alpha * M;
                        pixelData[x][y].M = 0;
                    }
                }
        }

        image.SetAllPixels(Vector3f::ZERO);
        for (int x = 0; x < camera->getWidth(); x++)
            for (int y = 0; y < camera->getHeight(); y++)
            {
                image.SetPixel(x, y, pixelData[x][y].tau / M_PI / pixelData[x][y].radius / pixelData[x][y].radius / num_photons / round);
            }
        for (int x = 0; x < camera->getWidth(); x++)
        {
            for (int y = 0; y < camera->getHeight(); y++)
            {
                Vector3f color = image.GetPixel(x, y);
                image.SetPixel(x, y, Vector3f(toInt(color.x()), toInt(color.y()), toInt(color.z())));
            }
        }
        time_t timep;
        struct tm *p;
        time(&timep);
        p = gmtime(&timep);
        fprintf(stderr, "\n");
        fprintf(stderr, "%d:", 8 + p->tm_hour);
        fprintf(stderr, "%d:", p->tm_min);
        fprintf(stderr, "%d ", p->tm_sec);

        fprintf(stderr, "round : %d\n", round);
        image.SaveBMP(tmpOut.c_str());
    }

    /*vector<HitPoint> hitPoints;
    hitPoints.clear();

    vector<HitPoint> tmphit[camera->getWidth()];

#pragma omp parallel for schedule(dynamic, 1) num_threads(n_threads)
    for (int x = 0; x < camera->getWidth(); x++)
    {
        fprintf(stderr, "\rRay Rendering  %5.2f%%", 100. * x / (camera->getWidth() - 1));
        tmphit[x].clear();
        mt19937 mt_rand(x);
        for (int y = 0; y < camera->getHeight(); y++)
        {
            double px = x + frand(&mt_rand);
            double py = y + frand(&mt_rand);
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

    int n_hitPoints = hitPoints.size();
    printf("%d\n", n_hitPoints);

    for (int i = 0; i < n_hitPoints; i++)
        hitPoints[i].radius = Rmax;

    KDTree kdtree(&hitPoints);

    for (int round = 1; round <= t_round; round++)
    {

        for (int li = 0; li < sceneParser->getNumLights(); li++)
        {
            Light *light = sceneParser->getLight(li);
#pragma omp parallel for schedule(dynamic, 1) num_threads(n_threads)
            for (int th = 0; th < n_threads; th++)
            {
                mt19937 mt_rand(round * n_threads + th);
                for (int photon = 0; photon < num_photons / n_threads; photon++)
                {
                    fprintf(stderr, "\rRendering  %5.2f%%", 100. * photon / (num_photons - 1));
                    Vector3f lightColor;
                    vector<HitPoint *> tmp;
                    Ray phoRay(Vector3f::ZERO, Vector3f::ZERO);
                    light->generatePhoton(phoRay, lightColor, &mt_rand);
                    photonTracing(phoRay, 0, sceneParser, &tmp, lightColor, &kdtree, &mt_rand);
                }
            }
        }
        if (round == 1)
        {
            for (int i = 0; i < n_hitPoints; i++)
            {
                hitPoints[i].N = hitPoints[i].M;
                hitPoints[i].M = 0;
            }
        }
        else
        {
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
        image.SetAllPixels(Vector3f::ZERO);
        for (auto hitPoint : hitPoints)
        {
            int x = hitPoint.x, y = hitPoint.y;
            Vector3f color = image.GetPixel(x, y);
            color += hitPoint.col * hitPoint.tau / M_PI / hitPoint.radius / hitPoint.radius / num_photons / round;
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
        time_t timep;
        struct tm *p;
        time(&timep);
        p = gmtime(&timep);
        fprintf(stderr, "\n");
        fprintf(stderr, "%d:", 8 + p->tm_hour); 
        fprintf(stderr, "%d:", p->tm_min);      
        fprintf(stderr, "%d ", p->tm_sec);      

        fprintf(stderr, "round : %d\n", round);
        image.SaveBMP(tmpOut.c_str());
    }*/

    return image;
}