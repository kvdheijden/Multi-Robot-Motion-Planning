//
// Created by koen on 12-4-19.
//

#include "FreeSpace.h"

FreeSpace::FreeSpace() = default;

FreeSpace::~FreeSpace() = default;

std::vector<Polygon>::iterator FreeSpace::begin() {
    return F.begin();
}

std::vector<Polygon>::const_iterator FreeSpace::begin() const {
    return F.begin();
}

std::vector<Polygon>::iterator FreeSpace::end() {
    return F.end();
}

std::vector<Polygon>::const_iterator FreeSpace::end() const {
    return F.end();
}

void FreeSpace::clear() {
    F.clear();
}

std::vector<Polygon>& FreeSpace::container() {
    return F;
}
