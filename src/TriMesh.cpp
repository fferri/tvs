//
//  TriMesh.cpp
//  tvs
//
//  Created by Federico Ferri on 8/1/2015.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#include "Environment.h"
#include "TriMesh.h"
#include "STLFile.h"
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <cmath>
#include <vector>
#include <fstream>
#include <iostream>
#include <drawstuff/drawstuff.h>

#ifdef dDOUBLE
#define dGeomTriMeshDataBuild dGeomTriMeshDataBuildDouble
#else
#define dGeomTriMeshDataBuild dGeomTriMeshDataBuildSingle
#endif

TriMesh::TriMesh() {
}

TriMesh::~TriMesh() {
}

void TriMesh::create(Environment *environment, const char *inputfile, bool binary, float scaleFactor) {
    STLFile f;
    if(binary) f.readBinary(inputfile);
    else f.readAscii(inputfile);

    this->vertex_count = 3 * f.facets.size();
    this->triangle_count = f.facets.size();
    this->vertices = new dReal[3 * this->vertex_count];
    this->triangles = new dTriIndex[3 * this->triangle_count];
    size_t vertex_idx = 0;
    for(size_t i = 0; i < f.facets.size(); i++) {
        this->vertices[3 * vertex_idx + 0] = scaleFactor * f.facets[i].ax;
        this->vertices[3 * vertex_idx + 1] = scaleFactor * f.facets[i].ay;
        this->vertices[3 * vertex_idx + 2] = scaleFactor * f.facets[i].az;
        this->triangles[vertex_idx] = vertex_idx;
        vertex_idx++;
        this->vertices[3 * vertex_idx + 0] = scaleFactor * f.facets[i].bx;
        this->vertices[3 * vertex_idx + 1] = scaleFactor * f.facets[i].by;
        this->vertices[3 * vertex_idx + 2] = scaleFactor * f.facets[i].bz;
        this->triangles[vertex_idx] = vertex_idx;
        vertex_idx++;
        this->vertices[3 * vertex_idx + 0] = scaleFactor * f.facets[i].cx;
        this->vertices[3 * vertex_idx + 1] = scaleFactor * f.facets[i].cy;
        this->vertices[3 * vertex_idx + 2] = scaleFactor * f.facets[i].cz;
        this->triangles[vertex_idx] = vertex_idx;
        vertex_idx++;
    }

    this->triMeshDataID = dGeomTriMeshDataCreate();
    dGeomTriMeshDataBuild(this->triMeshDataID, this->vertices, 3 * sizeof(dReal), this->vertex_count, this->triangles, 3 * this->triangle_count, 3 * sizeof(dTriIndex));
    this->geom = dCreateTriMesh(environment->space, this->triMeshDataID, 0, 0, 0);
    dGeomSetCategoryBits(this->geom, Category::TERRAIN);
    dGeomSetCollideBits(this->geom, Category::TRACK_GROUSER | Category::OBSTACLE);
}

void TriMesh::destroy() {
    dGeomTriMeshDataDestroy(this->triMeshDataID);
}

void TriMesh::draw() {
    const dReal* pos = dGeomGetPosition(this->geom);
    const dReal* R = dGeomGetRotation(this->geom);
#ifdef HAVE_DSDRAWTRIANGLES_FUNCTION
    dsDrawTrianglesD(pos, R, v, n, 1);
#else
    for(size_t i = 0; i < this->triangle_count; i++) {
        const dReal *p0 = &this->vertices[3*this->triangles[3*i+0]];
        const dVector3 v0 = { p0[0], p0[1], p0[2] };
        const dReal *p1 = &this->vertices[3*this->triangles[3*i+1]];
        const dVector3 v1 = { p1[0], p1[1], p1[2] };
        const dReal *p2 = &this->vertices[3*this->triangles[3*i+2]];
        const dVector3 v2 = { p2[0], p2[1], p2[2] };
        dsDrawTriangleD(pos, R, v0, v1, v2, 1);
    }
#endif // HAVE_DSDRAWTRIANGLES_FUNCTION
}

