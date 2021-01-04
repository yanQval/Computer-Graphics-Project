#ifndef SPPM_H
#define SPPM_H

#include "Vector3f.h"

class SceneParser;
class Image;

class SPPM
{
public:
    SPPM() = delete;
    SPPM(SceneParser *scene);

    ~SPPM();

    Image run();

private:
    SceneParser *sceneParser;
};

#endif