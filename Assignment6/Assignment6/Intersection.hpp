//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_INTERSECTION_H
#define RAYTRACING_INTERSECTION_H
#include "Vector.hpp"
#include "Material.hpp"
class Object;
class Sphere;
/*
    coords:光线经过的坐标点，
    happend:光线和三角形是否相交，
    m:材质，
    distanse:光线经过的时间t,
    obj:本对象
*/
struct Intersection
{
    Intersection(){
        happened=false;
        coords=Vector3f();
        normal=Vector3f();
        distance= std::numeric_limits<double>::max();
        obj =nullptr;
        m=nullptr;
    }
    bool happened;
    Vector3f coords;
    Vector3f normal;
    double distance;
    Object* obj;
    Material* m;
};
#endif //RAYTRACING_INTERSECTION_H
