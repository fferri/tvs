//
//  OcTreeBuilder.cpp
//  tvs
//
//  Created by Federico Ferri on 2/12/2016.
//  Copyright (c) 2016 Federico Ferri. All rights reserved.
//

#include "OcTreeBuilder.h"

#include "Environment.h"

#include <cmath>
#include <iostream>
#include <boost/foreach.hpp>

OcTreeBuilder::OcTreeBuilder(Environment *env_, int maxLevel_, dReal extents_)
    : env(env_),
      maxLevel(maxLevel_),
      extents(extents_),
      octree(2 * extents * pow(2, -maxLevel))
{
}

void OcTreeBuilder::callback(void *data, dGeomID o1, dGeomID o2) {
    OcTreeBuilder *d = (OcTreeBuilder *)data;
    if(o1 != d->testCube && o2 != d->testCube) return;
    if(dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
        dSpaceCollide2(o1, o2, data, &OcTreeBuilder::callback);
        if(dGeomIsSpace(o1))
            dSpaceCollide((dSpaceID)o1, data, &OcTreeBuilder::callback);
        if(dGeomIsSpace(o2))
            dSpaceCollide((dSpaceID)o2, data, &OcTreeBuilder::callback);
    } else {
        // collision of two non-space geoms
        int num_contacts = 1;
        int contacts_flags = num_contacts | CONTACTS_UNIMPORTANT;
        dContactGeom dcg;
        int nc = dCollide(o1, o2, contacts_flags, &dcg, sizeof(dContactGeom));
        d->numChecks++;
        d->occupied = nc > 0;
    }
}

bool OcTreeBuilder::check(int level, dReal x, dReal y, dReal z) {
    dGeomSetPosition(this->testCube, x, y, z);
    dSpaceCollide(this->env->space, this, &OcTreeBuilder::callback);
    return this->occupied;
}

void OcTreeBuilder::check() {
    this->numChecks = 0;

    dReal sz = 2 * this->extents;
    this->testCube = dCreateBox(this->env->space, sz, sz, sz);

    this->occupiedPoints.clear();
    check(0, 0., 0., this->extents);
    if(!this->occupied) return;
    occupiedPoints.insert(Point3D(0., 0., this->extents));

    // estimate # of measures to do (for progress report):
    float avgBranching = 4.5;
    int total = 0, complete = 0, oldPercent = 0, percent = 0;
    for(int level = 0; level <= this->maxLevel; level++)
        total += int(pow(avgBranching, level));

    for(int level = 1; level <= this->maxLevel; level++) {
        dReal ext = this->extents * pow(2, -level);
        dReal sz = 2 * ext;
        dGeomBoxSetLengths(this->testCube, sz, sz, sz);

        // update progress estimation with actual number of measures to do:
        total -= int(pow(avgBranching, level));
        total += occupiedPoints.size();

        std::set<Point3D> pts;

        BOOST_FOREACH(const Point3D &p, occupiedPoints) {
            // report progress:
            percent = int(complete++ * 100 / total);
            if(percent > oldPercent) {
                oldPercent = percent;
                std::cout << "Building occupancy grid: " << percent << "%...\r";
                std::flush(std::cout);
            }

            // measure occupancy in the 8 subnodes of this node:
            for(int dz = -1; dz <= 1; dz += 2) {
                for(int dy = -1; dy <= 1; dy += 2) {
                    for(int dx = -1; dx <= 1; dx += 2) {
                        dReal x = p.x + ext * dx;
                        dReal y = p.y + ext * dy;
                        dReal z = p.z + ext * dz;
                        check(level, x, y, z);
                        if(this->occupied)
                            pts.insert(Point3D(x, y, z));
                    }
                }
            }
        }

        occupiedPoints.clear();
        occupiedPoints = pts;
    }

    // now occupiedPoints contain the voxel grid at maximum resolution
    BOOST_FOREACH(const Point3D &p, occupiedPoints) {
        octree.updateNode(p.x, p.y, p.z, true);
    }

    std::cout << "Built occupancy grid using " << this->numChecks << " collision checks" << std::endl;
    dGeomDestroy(this->testCube);
}

void OcTreeBuilder::savePCD(const char *filename, int level) {
    std::ofstream f;
    f.open(filename);
    savePCD(f, level);
    f.close();
}

void OcTreeBuilder::savePCD(std::ofstream &f, int level) {
    while(level < 0) level += this->maxLevel;
    f << "# .PCD v0.7" << std::endl;
    f << "VERSION 0.7" << std::endl;
    f << "FIELDS x y z" << std::endl;
    f << "SIZE 4 4 4" << std::endl;
    f << "TYPE F F F" << std::endl;
    f << "COUNT 1 1 1" << std::endl;
    f << "WIDTH " << this->occupiedPoints.size() << std::endl;
    f << "HEIGHT 1" << std::endl;
    f << "VIEWPOINT 0 0 0 0 0 0 1" << std::endl;
    f << "POINTS " << this->occupiedPoints.size() << std::endl;
    f << "DATA ascii" << std::endl;
    saveXYZ(f, level);
}

void OcTreeBuilder::saveXYZ(const char *filename, int level) {
    std::ofstream f;
    f.open(filename);
    saveXYZ(f, level);
    f.close();
}

void OcTreeBuilder::saveXYZ(std::ofstream &f, int level, const char *sep) {
    while(level < 0) level += this->maxLevel;
    int n = 0;
    BOOST_FOREACH(const Point3D &p, this->occupiedPoints) {
        f << p.x << sep << p.y << sep << p.z << std::endl;
        n++;
    }
    std::cout << "Saved " << n << " points" << std::endl;
}

void OcTreeBuilder::saveOcTree(const char *filename) {
    this->octree.writeBinary(filename);
}

