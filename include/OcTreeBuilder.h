//
//  OcTreeBuilder.h
//  tvs
//
//  Created by Federico Ferri on 2/12/2016.
//  Copyright (c) 2016 Federico Ferri. All rights reserved.
//

#ifndef OCTREEBUILDER_H_INCLUDED
#define OCTREEBUILDER_H_INCLUDED

#include <map>
#include <set>
#include <fstream>
#include <ode/ode.h>
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

class Environment;

struct Point3D {
    dReal x, y, z;

    Point3D(dReal x_, dReal y_, dReal z_) : x(x_), y(y_), z(z_) {}

    inline bool operator<(const Point3D& o) const {
        if(x != o.x) return x < o.x;
        if(y != o.y) return y < o.y;
        return z < o.z;
    }
};

class OcTreeBuilder {
public:
    OcTreeBuilder(Environment *e, int maxLevel, dReal extents);
    static void callback(void *data, dGeomID o1, dGeomID o2);
    void check();
    void savePCD(const char *filename, int level = -1);
    void savePCD(std::ofstream &f, int level = -1);
    void saveXYZ(const char *filename, int level = -1);
    void saveXYZ(std::ofstream &f, int level = -1, const char *sep = " ");
    void saveOcTree(const char *filename);

private:
    bool check(int level, dReal x, dReal y, dReal z);

    Environment *env;
    int maxLevel;
    dReal extents;
    dGeomID testCube;
    octomap::OcTree octree;

    std::set<Point3D> occupiedPoints;
    bool occupied;
    int numChecks;
};

#endif // OCTREEBUILDER_H_INCLUDED
