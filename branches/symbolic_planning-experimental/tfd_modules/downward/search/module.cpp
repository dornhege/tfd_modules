#include "module.h"
#include <cassert>
#include <iostream>

Module::Module(istream &in)
{
    string function_name, lib;
    in >> function_name >> lib;
    libCall = function_name;
    libCall.append("@").append(lib);

    int count;
    in >> count;
    for (int i = 0; i < count; ++i) {
        string paramname, type, value;
        in >> paramname >> type >> value;
        params.push_back(Parameter(paramname, type, value));
    }
}

void Module::dump()
{
    std::cout << "LibCall: " << libCall.c_str() << std::endl; 
    std::cout << "Params: " << params << std::endl;
}

ConditionModule::ConditionModule(istream &in) :
    Module(in)
{
    in >> var;
    g_variable_types[var] = module;
    checkCondition = g_module_loader->getConditionChecker(libCall);
    if (checkCondition == NULL) {
        printf("BAD Bad bad :-/\n");
        assert(false);
    }
}

void ConditionModule::dump()
{
    std::cout << "ConditionModule" << std::endl;
    Module::dump();
    std::cout << "Variable: " << var << std::endl;

    std::cout << "checkCondition call is: " << (long)(checkCondition) << std::endl;
}

EffectModule::EffectModule(istream &in) :
    Module(in)
{
    in >> internal_name;
    int count;
    in >> count;
    writtenVars.reserve(count);
    int var_no;
    for (int i = 0; i < count; ++i) {
        in >> var_no;
        writtenVars.push_back(var_no);
    }
    applyEffect = g_module_loader->getApplyEffect(libCall);
    if (applyEffect == NULL) {
        printf("BAD Bad bad :-/\n");
        assert(false);
    }
}

void EffectModule::dump()
{
    std::cout << "EffectModule" << std::endl;
    Module::dump();
    std::cout << "Internal name: " << internal_name << std::endl;
    std::cout << "Written vars: ";
    for(vector<int>::iterator it = writtenVars.begin(); it != writtenVars.end(); it++)
        std::cout << *it << " ";
    std::cout << std::endl;

    std::cout << "ApplyEffect call is: " << (long)(applyEffect) << std::endl;
}

CostModule::CostModule(istream &in) :
    Module(in)
{
    in >> var;
    assert(g_variable_types[var] == costmodule);
    checkCost = g_module_loader->getCostChecker(libCall);
    if (checkCost == NULL) {
        printf("BAD Bad bad :-/\n");
        assert(false);
    }
}

InitModule::InitModule(istream &in)
{
    string function_name, lib;
    in >> function_name >> lib;
    libCall = function_name;
    libCall.append("@").append(lib);

    int count;
    in >> count;
    string param;
    for (int i = 0; i < count; ++i) {
        in >> param;
        parameters.push_back(param);
    }

    initModule = g_module_loader->getModuleInit(libCall);
    if (initModule == NULL) {
        printf("BAD Bad bad :-/\n");
        assert(false);
    }
}

void InitModule::execInit()
{
    int argc = parameters.size() + 1;
    char** argv = new char*[argc];

    argv[0] = strdup(libCall.c_str());
    for (int i = 0; i < parameters.size(); i++) {
        argv[i + 1] = strdup(parameters.at(i).c_str());
    }
    initModule(argc, argv);
    for (int i = 0; i < argc; i++) {
        free(argv[i]);
    }
    delete[] argv;
}

Module::~Module()
{
}
