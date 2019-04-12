#include "ConfigurationSet.h"

ConfigurationSet::ConfigurationSet() {
    next_source_index = 1;
    next_target_index = 1;
}

ConfigurationSet::~ConfigurationSet() = default;

void ConfigurationSet::addSourceConfiguration(Point &p) {
    configurations.emplace_back(std::move(p), true, next_source_index++);
}

void ConfigurationSet::addTargetConfiguration(Point &p) {
    configurations.emplace_back(std::move(p), false, next_target_index++);
}

ConfigurationSet::iterator ConfigurationSet::begin() {
    return configurations.begin();
}

ConfigurationSet::const_iterator ConfigurationSet::begin() const {
    return configurations.begin();
}

ConfigurationSet::iterator ConfigurationSet::end() {
    return configurations.end();
}

ConfigurationSet::const_iterator ConfigurationSet::end() const {
    return configurations.end();
}

void ConfigurationSet::clear() {
    configurations.clear();
    next_source_index = 1;
    next_target_index = 1;
}

size_t ConfigurationSet::size() const {
    return configurations.size();
}