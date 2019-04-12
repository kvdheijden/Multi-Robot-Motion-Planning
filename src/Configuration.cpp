#include "Configuration.h"

Configuration::Configuration(Point &&p, bool is_source, int index) : p(p), is_source(is_source), index(index) {

}

Configuration::Configuration(const Configuration &other) = default;

Configuration::~Configuration() = default;

const Point &Configuration::getPoint() const {
    return p;
}

const bool &Configuration::isStart() const {
    return is_source;
}

const int &Configuration::getIndex() const {
    return index;
}

CGAL::Bbox_2 Configuration::bbox() const {
    return p.bbox();
}