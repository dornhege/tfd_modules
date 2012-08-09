/*
 * module_param_cache.cpp
 *
 *  Created on: 19 Jul 2012
 *      Author: andreas
 */

template <class ValueType>
std::string ModuleParamCache<ValueType>::baseNamespace = "/tfd_modules/module_cache";

template <class ValueType>
ModuleParamCache<ValueType>::ModuleParamCache()
{
    node = NULL;
}

template <class ValueType>
void ModuleParamCache<ValueType>::initialize(const std::string& moduleNamespace, ros::NodeHandle* node)
{
    this->node = node;
    keyPrefix = ModuleParamCache<ValueType>::baseNamespace + "/" + moduleNamespace + "/";
}

template <class ValueType>
ModuleParamCache<ValueType>::~ModuleParamCache()
{
}

template <class ValueType>
void ModuleParamCache<ValueType>::clearAll()
{
    if(node->hasParam(keyPrefix))
    {
        node->deleteParam(keyPrefix);
    }
    _localCache.clear();
}

template <class ValueType>
void ModuleParamCache<ValueType>::set(const std::string& key, ValueType value, bool allowCacheAsParam)
{
//    ROS_INFO("[cache]: writing to cache: %s -> %f", key.c_str(), value);
    if(allowCacheAsParam) {
        typename std::map<std::string, ValueType>::iterator it = _localCache.find(key);
        // if we found the same key,value in the local cache, it was inserted by this function and thus
        // is already on the param server, no need to make an extra setParam call here.
        if(it == _localCache.end() || it->second != value) {
            node->setParam(keyPrefix + key, value);
        }
    }
    _localCache.insert(std::make_pair(key, value));
}

template <class ValueType>
bool ModuleParamCache<ValueType>::get(const std::string& key, ValueType& value)
{
//    ROS_INFO("[cache]: lookup in cache: %s -> %f", key.c_str(), value);
    typename std::map<std::string, ValueType>::iterator it = _localCache.find(key);
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

