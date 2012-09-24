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
#include <geometry_msgs/Pose.h>

/// Module cache that allows to cache entries locally (for one run) or additionally
/// on the param server between multiple runs. ValueType needs to be able to be written
/// as a ROS param.
/**
 * It is assumed that during a planner run only this class accesses the
 * module cache stored on the param server, i.e. nobody interferes.
 */
template <class ValueType>
class ModuleParamCache
{
public:
    ModuleParamCache();
    ~ModuleParamCache();

    /// Clear all cache entries for this modules namespace (especially on the param server)
    void clearAll();

    /// Initialize the module param cache for param lookup.
    void initialize(const std::string& moduleNamespace, ros::NodeHandle* node);

    /// Add a new cache entry.
    /**
     * \param [in] key the key for the entry
     * \param [in] value the value to cache for key
     * \param [in] allowCacheAsParam if true, cache entries are entered on the
     * param server and thus are available between multiple planner calls.
     * Therefore their value should obviously not change in the world
     * throughout multiple calls.
     */
    void set(const std::string& key, ValueType value, bool allowCacheAsParam);

    /// Retrieve a cached value for key.
    /**
     * \param [in] key the key for the entry
     * \param [out] value the retrieved value, if found.
     * \returns true, if the value was found in the cache.
     */
    bool get(const std::string& key, ValueType& value);

private:
    static std::string baseNamespace;

    /// Param cache across multiple runs.
    ros::NodeHandle* node;
    std::string keyPrefix;

    std::map<std::string, ValueType> _localCache;  ///< local cache only for this planner run
};

/// Create a string from a Pose that is unique and can be stored in the param daemon.
std::string createPoseParamString(const geometry_msgs::Pose & ps, double precPose = 0.0001, double precQuat = 0.0001);

#include "module_param_cache.hpp"

typedef ModuleParamCache<double> ModuleParamCacheDouble;
typedef ModuleParamCache<std::string> ModuleParamCacheString;

#endif /* MODULE_PARAM_CACHE_H_ */

