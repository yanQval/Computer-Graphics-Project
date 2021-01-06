#ifndef GROUP_H
#define GROUP_H


#include "object3d.hpp"
#include "ray.hpp"
#include "hit.hpp"
#include <iostream>
#include <vector>


// TODO: Implement Group - add data structure to store a list of Object*
class Group : public Object3D {

public:
    
    Group() {
        list.clear();
        num_objects = 0;
    }

    explicit Group (int _num_objects) {
        num_objects = _num_objects;
        for(int i = 0; i < num_objects; i++)list.push_back(nullptr);
    }

    ~Group() override {

    }

    bool intersect(const Ray &r, Hit &h, double tmin) override {
        bool hit = false;
        for(auto obj:list){
            assert(obj != nullptr);
            hit |= obj->intersect(r, h, tmin);
        }
        return hit;
    }

    void addObject(int index, Object3D *obj) {
        list[index] = obj;
    }

    int getGroupSize() {
        return num_objects;
    }

private:
    int num_objects;
    std::vector<Object3D*> list;
};

#endif
	
