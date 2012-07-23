/*
 * module_param_cache.cpp
 *
 *  Created on: 19 Jul 2012
 *      Author: andreas
 */

#include "planner_modules_pr2/module_param_cache.h"

std::string ModuleParamCache::baseNamespace = "/tfd_modules/cache";


ModuleParamCache::ModuleParamCache()
{
    node = NULL;
}

void ModuleParamCache::initialize(const std::string& moduleNamespace, ros::NodeHandle* node)
{
    this->node = node;
    this->baseNamespace = baseNamespace;
    keyPrefix = baseNamespace + "/" + moduleNamespace + "/";
}

ModuleParamCache::~ModuleParamCache()
{
}

void ModuleParamCache::clearAll()
{
    ros::NodeHandle s_node = ros::NodeHandle();
    if (s_node.hasParam(baseNamespace))
    {
        s_node.deleteParam(baseNamespace);
    }
}

void ModuleParamCache::set(const std::string& key, double value)
{
//    ROS_INFO("[cache]: writing to cache: %s -> %f", key.c_str(), value);
    node->setParam(keyPrefix + key, value);
}

bool ModuleParamCache::get(const std::string& key, double& value) const
{
//    ROS_INFO("[cache]: lookup in cache: %s -> %f", key.c_str(), value);
    return node->getParamCached(keyPrefix + key, value);
}

