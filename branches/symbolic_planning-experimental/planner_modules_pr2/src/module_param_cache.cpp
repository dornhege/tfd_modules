/*
 * module_param_cache.cpp
 *
 *  Created on: 19 Jul 2012
 *      Author: andreas
 */

#include "planner_modules_pr2/module_param_cache.h"

std::string ModuleParamCache::baseNamespace = "/tfd_modules/module_cache";

ModuleParamCache::ModuleParamCache()
{
    node = NULL;
}

void ModuleParamCache::initialize(const std::string& moduleNamespace, ros::NodeHandle* node)
{
    this->node = node;
    keyPrefix = ModuleParamCache::baseNamespace + "/" + moduleNamespace + "/";
}

ModuleParamCache::~ModuleParamCache()
{
}

void ModuleParamCache::clearAll()
{
    if(node->hasParam(keyPrefix))
    {
        node->deleteParam(keyPrefix);
    }
    _localCache.clear();
}

void ModuleParamCache::set(const std::string& key, double value, bool allowCacheAsParam)
{
//    ROS_INFO("[cache]: writing to cache: %s -> %f", key.c_str(), value);
    if(allowCacheAsParam) {
        std::map<std::string, double>::iterator it = _localCache.find(key);
        // if we found the same key,value in the local cache, it was inserted by this function and thus
        // is already on the param server, no need to make an extra setParam call here.
        if(it == _localCache.end() || it->second != value) {
            node->setParam(keyPrefix + key, value);
        }
    }
    _localCache.insert(std::make_pair(key, value));
}

bool ModuleParamCache::get(const std::string& key, double& value)
{
//    ROS_INFO("[cache]: lookup in cache: %s -> %f", key.c_str(), value);
    std::map<std::string, double>::iterator it = _localCache.find(key);
    if(it != _localCache.end()) {       // local cache hit
        value = it->second;
        return true;
    }
    // local miss, look on param server
    bool found = node->getParamCached(keyPrefix + key, value);
    if(found) { // we found this on the param server, insert locally to prevent additional setParam calls
        _localCache.insert(std::make_pair(key, value));
    }
    return found;
}

