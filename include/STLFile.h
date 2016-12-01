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
#include <string>
#include <vector>

struct STLFacet {
    float ni, nj, nk;
    float ax, ay, az;
    float bx, by, bz;
    float cx, cy, cz;
    uint16_t attr_count;

    void readAscii(std::vector<std::string> &lines, size_t &pos);
    void readBinary(std::ifstream &f);
};

struct STLFile {
    std::vector<STLFacet> facets;
    
    void readAscii(const char *filename);
    void readBinary(const char *filename);
    
    void readAscii(std::ifstream &f);
    void readBinary(std::ifstream &f);

    void readAscii(std::vector<std::string> &lines, size_t pos);
};

#endif // STLFILE_H_INCLUDED
