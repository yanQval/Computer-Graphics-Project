#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>
#include <random>

#include "scene_parser.hpp"
#include "image.hpp"
#include "camera.hpp"
#include "group.hpp"
#include "light.hpp"
#include "sppm.hpp"
#include "utils.hpp"

#include "constant.h"

#include <string>

int SPP = 10;

using namespace std;

Vector3f PathTracing(const Ray &r, int depth, const SceneParser &sceneParser, mt19937 *mt_rand)
{
    Group *basegroup = sceneParser.getGroup();
    Hit hit;
    bool isIntersect = basegroup->intersect(r, hit, eps);
    if (!isIntersect)
    {
        return sceneParser.getBackgroundColor();
    }
    Vector3f x = r.getOrigin() + r.getDirection() * hit.getT();
    Vector3f c = hit.getMaterial()->getColor();
    Vector3f n = hit.getNormal().normalized();

    float p = max(max(c.x(), c.y()), c.z());
    if (++depth > 5)
    {
        if (frand(mt_rand) < p)
            c = c * (1 / p);
        else
            return hit.getMaterial()->getEmission();
    }
    if (hit.getMaterial()->getType() == DIFUSE)
    {
        float r1 = 2 * M_PI * frand(mt_rand), r2 = frand(mt_rand), r2s = sqrt(r2);
        assert(r2s >= 0);

        Vector3f w = n, u = Vector3f::cross((fabs(w.x()) > .1 ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0)), w).normalized(), v = Vector3f::cross(w, u);
        Vector3f d = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2)).normalized();
        return hit.getMaterial()->getEmission() + c * PathTracing(Ray(x, d), depth, sceneParser, mt_rand);
    }
    else if (hit.getMaterial()->getType() == SPEC)
    {
        return hit.getMaterial()->getEmission() + c * PathTracing(Ray(x, r.getDirection() - n * 2 * Vector3f::dot(n, (r.getDirection()))), depth, sceneParser, mt_rand);
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
            return hit.getMaterial()->getEmission() + c * PathTracing(Ray(x, r.getDirection() - n * 2 * Vector3f::dot(n, (r.getDirection()))), depth, sceneParser, mt_rand);
        }
        Vector3f tdir = (r.getDirection() * nnt - n * (ddn * nnt + sqrt(cos2t))).normalized();
        float a = nt - nc, b = nt + nc, R0 = a * a / (b * b), tc = 1 - (into ? -ddn : -Vector3f::dot(tdir, n));
        float Re = R0 + (1 - R0) * tc * tc * tc * tc * tc, Tr = 1 - Re, P = .25 + .5 * Re, RP = Re / P, TP = Tr / (1 - P);
        return hit.getMaterial()->getEmission() + c * (depth > 2 ? (frand(mt_rand) < P ? PathTracing(reflRay, depth, sceneParser, mt_rand) * RP : PathTracing(Ray(x, tdir), depth, sceneParser, mt_rand) * TP)
                                                                 : PathTracing(reflRay, depth, sceneParser, mt_rand) * Re + PathTracing(Ray(x, tdir), depth, sceneParser, mt_rand) * Tr);
    }
}

int main(int argc, char *argv[])
{
    for (int argNum = 1; argNum < argc; ++argNum)
    {
        std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
    }

    if (argc != 4)
    {
        cout << "Usage: ./bin/PA1 <input scene file> <output bmp file> <Samples Per Pixel>" << endl;
        return 1;
    }
    string inputFile = argv[1];
    string outputFile = argv[2]; // only bmp is allowed.
    SPP = atoi(argv[3]);

    // TODO: Main RayCasting Logic
    // First, parse the scene using SceneParser.
    // Then loop over each pixel in the image, shooting a ray
    // through that pixel and finding its intersection with
    // the scene.  Write the color at the intersection to that
    // pixel in your output image.

    SceneParser *sceneParser = new SceneParser(inputFile.c_str());
    SPPM sppm(sceneParser);
    Image image = sppm.run();
    /*
    Camera *camera = sceneParser.getCamera();
    printf("%d %d\n", camera->getWidth(), camera->getHeight());
    Image image(camera->getWidth(), camera->getHeight());
#pragma omp parallel for schedule(dynamic, 1) num_threads(10)
    for (int x = 0; x < camera->getWidth(); x++)
    {
        fprintf(stderr, "\rRendering (%d spp) %5.2f%%", SPP, 100. * x / (camera->getWidth() - 1));
        mt19937 mt_rand(x);
        for (int y = 0; y < camera->getHeight(); y++)
        {
            Vector3f finalColor = Vector3f::ZERO;
            for (int t = 0; t < SPP; t++)
            {
                float px = x + frand(&mt_rand);
                float py = y + frand(&mt_rand);
                Ray camRay = camera->generateRay(Vector2f(px, py));
                finalColor += PathTracing(camRay, 0, sceneParser, &mt_rand) * (1. / SPP);
            }
            Vector3f resultColor(toInt(finalColor.x()), toInt(finalColor.y()), toInt(finalColor.z()));
            image.SetPixel(x, y, resultColor);
        }
    }*/
    image.SaveBMP(outputFile.c_str());
    cout << endl
         << "Hello! Computer Graphics!" << endl;
    return 0;
}
