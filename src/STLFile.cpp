//
//  STLFile.cpp
//  tvs
//
//  Created by Federico Ferri on 30/11/2016.
//  Copyright (c) 2016 Federico Ferri. All rights reserved.
//

#include "STLFile.h"

#include <fstream>
#include <string>
#include <sstream>
#include <iostream>

bool STLFacet::readAscii(std::ifstream &f) {
    {
        std::string line, facet, normal;
        if(!std::getline(f, line)) return false;
        std::istringstream iss(line);
        if(!(iss >> facet >> normal >> ni >> nj >> nk)) return false;
        if(facet != "facet" || normal != "normal") return false;
    }
    {
        std::string line, outer, loop;
        if(!std::getline(f, line)) return false;
        std::istringstream iss(line);
        if(!(iss >> outer >> loop)) return false;
        if(outer != "outer" || loop != "loop") return false;
    }
    {
        std::string line, vertex;
        if(!std::getline(f, line)) return false;
        std::istringstream iss(line);
        if(!(iss >> vertex >> ax >> ay >> az)) return false;
        if(vertex != "vertex") return false;
    }
    {
        std::string line, vertex;
        if(!std::getline(f, line)) return false;
        std::istringstream iss(line);
        if(!(iss >> vertex >> bx >> by >> bz)) return false;
        if(vertex != "vertex") return false;
    }
    {
        std::string line, vertex;
        if(!std::getline(f, line)) return false;
        std::istringstream iss(line);
        if(!(iss >> vertex >> cx >> cy >> cz)) return false;
        if(vertex != "vertex") return false;
    }
    {
        std::string line, endloop;
        if(!std::getline(f, line)) return false;
        std::istringstream iss(line);
        if(!(iss >> endloop)) return false;
        if(endloop != "endloop") return false;
    }
    {
        std::string line, endfacet;
        if(!std::getline(f, line)) return false;
        std::istringstream iss(line);
        if(!(iss >> endfacet)) return false;
        if(endfacet != "endfacet") return false;
    }
    return true;
}

bool STLFacet::readBinary(std::ifstream &f) {
    f.read((char *)this, 12 * sizeof(float) + sizeof(uint16_t));
    if(!f) return false;
    return true;
}

bool STLFile::readAscii(const char *filename) {
    std::ifstream f(filename, std::ios::in);
    return readAscii(f);
}

bool STLFile::readBinary(const char *filename) {
    std::ifstream f(filename, std::ios::in | std::ios::binary);
    return readBinary(f);
}

bool STLFile::readAscii(std::ifstream &f) {
    std::string solidName;
    facets.clear();
    {
        std::string line, solid;
        if(!std::getline(f, line)) return false;
        std::istringstream iss(line);
        if(!(iss >> solid >> solidName)) return false;
        if(solid != "solid") return false;
    }
    {
        while(true) {
            STLFacet facet;
            if(!facet.readAscii(f)) break;
            facets.push_back(facet);
        }
    }
    {
        std::string line, endsolid, solidName1;
        if(!std::getline(f, line)) return false;
        std::istringstream iss(line);
        if(!(iss >> endsolid >> solidName1)) return false;
        if(endsolid != "endsolid") return false;
        if(solidName1 != solidName) return false;
    }
    return true;
}

bool STLFile::readBinary(std::ifstream &f) {
    f.ignore(80); // skip header
    if(!f) return false;
    uint32_t n;
    f.read((char *)&n, 4);
    if(!f) return false;
    facets.resize(n);
    for(int i = 0; i < n; i++) {
        if(!facets[i].readBinary(f))
            return false;
    }
    return true;
}

