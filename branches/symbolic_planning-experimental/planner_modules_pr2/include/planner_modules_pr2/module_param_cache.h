/*
 * module_param_cache.h
 *
 *  Created on: 19 Jul 2012
 *      Author: andreas
 */

#ifndef MODULE_PARAM_CACHE_H_
#define MODULE_PARAM_CACHE_H_

#include <string>
#include <map>
#include <ros/ros.h>

class ModuleParamCache
{
public:
    ModuleParamCache();
    virtual ~ModuleParamCache();

    static void clearAll();

    void initialize(const std::string& moduleNamespace, ros::NodeHandle* node);
    void set(const std::string& key, double value);
    bool get(const std::string& key, double& value) const;

private:
    static std::string baseNamespace;
    std::string moduleNamespace;
    ros::NodeHandle* node;
    std::string keyPrefix;
};

#endif /* MODULE_PARAM_CACHE_H_ */
