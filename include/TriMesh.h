//
//  TriMesh.h
//  tvs
//
//  Created by Federico Ferri on 8/1/2015.
//  Copyright (c) 2014 Federico Ferri. All rights reserved.
//

#ifndef TRIMESH_H_INCLUDED
#define TRIMESH_H_INCLUDED

#include <ode/ode.h>
#include <inttypes.h>

class Environment;

class TriMesh {
public:
    dGeomID geom;
    dTriMeshDataID triMeshDataID;
    
    dReal *vertices;
    int vertex_count;
    dTriIndex *triangles;
    int triangle_count;
    
    TriMesh();
    virtual ~TriMesh();
    void create(Environment *environment, const char *inputfile);
    void destroy();
    void draw();
};

#endif // TRIMESH_H_INCLUDED
