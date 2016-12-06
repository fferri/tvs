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
#include <boost/shared_ptr.hpp>

class Environment;

class TriMesh {
public:
    dGeomID geom;
    dTriMeshDataID triMeshDataID;
    
    dReal *vertices;
    int vertex_count;
    dTriIndex *triangles;
    int triangle_count;
    
    struct BoundsXYZ {
        struct Bounds {
            dReal min, max;
        } x, y, z;
    } bounds;

    struct Color {
        float r, g, b, a;
    } color;

    TriMesh();
    virtual ~TriMesh();
    void create(Environment *environment, const char *inputfile, bool binary = false, float scaleFactor = 1.0);
    void destroy();
    void draw();
};

typedef boost::shared_ptr<TriMesh> TriMeshPtr;

#endif // TRIMESH_H_INCLUDED
