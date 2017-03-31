//
//  STLFile.cpp
//  tvs
//
//  Created by Federico Ferri on 30/11/2016.
//  Copyright (c) 2016 Federico Ferri. All rights reserved.
//

#include "STLFile.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>

void STLFacet::readAscii(std::vector<std::string> &lines, size_t &pos) {
    size_t p = pos;
    {
        std::string facet, normal;
        std::istringstream iss(lines[p++]);
        if(!(iss >> facet >> normal >> ni >> nj >> nk) || facet != "facet" || normal != "normal")
            throw std::runtime_error(std::string("cannot read facet: bad format: ") + iss.str());
    }
    {
        std::string outer, loop;
        std::istringstream iss(lines[p++]);
        if(!(iss >> outer >> loop) || outer != "outer" || loop != "loop")
            throw std::runtime_error(std::string("cannot read facet: bad format: ") + iss.str());
    }
    {
        std::string vertex;
        std::istringstream iss(lines[p++]);
        if(!(iss >> vertex >> ax >> ay >> az) || vertex != "vertex")
            throw std::runtime_error(std::string("cannot read facet: bad format: ") + iss.str());
    }
    {
        std::string vertex;
        std::istringstream iss(lines[p++]);
        if(!(iss >> vertex >> bx >> by >> bz) || vertex != "vertex")
            throw std::runtime_error(std::string("cannot read facet: bad format: ") + iss.str());
    }
    {
        std::string vertex;
        std::istringstream iss(lines[p++]);
        if(!(iss >> vertex >> cx >> cy >> cz) || vertex != "vertex")
            throw std::runtime_error(std::string("cannot read facet: bad format: ") + iss.str());
    }
    {
        std::string endloop;
        std::istringstream iss(lines[p++]);
        if(!(iss >> endloop) || endloop != "endloop")
            throw std::runtime_error(std::string("cannot read facet: bad format: ") + iss.str());
    }
    {
        std::string endfacet;
        std::istringstream iss(lines[p++]);
        if(!(iss >> endfacet) || endfacet != "endfacet")
            throw std::runtime_error(std::string("cannot read facet: bad format: ") + iss.str());
    }
    pos = p;
}

void STLFacet::readBinary(std::ifstream &f) {
    f.read((char *)this, 12 * sizeof(float) + sizeof(uint16_t));
    if(!f)
        throw std::runtime_error("cannot read facet: read failed");
}

void STLFile::readAscii(const char *filename) {
    std::ifstream f(filename, std::ios::in);
    readAscii(f);
}

void STLFile::readBinary(const char *filename) {
    std::ifstream f(filename, std::ios::in | std::ios::binary);
    readBinary(f);
}

void STLFile::readAscii(std::ifstream &f) {
    std::string line;
    std::vector<std::string> lines;
    while(std::getline(f, line))
        if(line.size())
            lines.push_back(line);
    size_t pos = 0;

    std::string solidName;
    facets.clear();
    {
        std::string solid;
        std::istringstream iss(lines[pos]);
        if(!(iss >> solid >> solidName) || solid != "solid")
            throw std::runtime_error(std::string("cannot read solid: bad format") + iss.str());
        pos++;
    }
    {
        while(true) {
            STLFacet facet;
            try {
                facet.readAscii(lines, pos);
            } catch(std::exception &ex) {
                break;
            }
            facets.push_back(facet);
        }
    }
    {
        std::string endsolid, solidName1 = solidName;
        std::istringstream iss(lines[pos]);
        if((!(iss >> endsolid) && !(iss >> endsolid >> solidName1)) || endsolid != "endsolid" || solidName1 != solidName)
            throw std::runtime_error(std::string("cannot read solid: bad format") + iss.str());
        pos++;
    }
}

void STLFile::readBinary(std::ifstream &f) {
    f.ignore(80); // skip header
    if(!f) throw std::runtime_error("");
    uint32_t n;
    f.read((char *)&n, 4);
    if(!f) throw std::runtime_error("");
    facets.resize(n);
    for(int i = 0; i < n; i++) {
        facets[i].readBinary(f);
    }
}

