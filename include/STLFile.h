//
//  STLFile.h
//  tvs
//
//  Created by Federico Ferri on 30/11/2016.
//  Copyright (c) 2016 Federico Ferri. All rights reserved.
//

#ifndef STLFILE_H_INCLUDED
#define STLFILE_H_INCLUDED

#include <inttypes.h>
#include <fstream>
#include <vector>

struct STLFacet {
    float ni, nj, nk;
    float ax, ay, az;
    float bx, by, bz;
    float cx, cy, cz;
    uint16_t attr_count;

    bool readAscii(std::ifstream &f);
    bool readBinary(std::ifstream &f);
};

struct STLFile {
    std::vector<STLFacet> facets;
    
    bool readAscii(const char *filename);
    bool readBinary(const char *filename);
    
    bool readAscii(std::ifstream &f);
    bool readBinary(std::ifstream &f);
};

#endif // STLFILE_H_INCLUDED
