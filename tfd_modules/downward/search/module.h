#ifndef MODULE_H_
#define MODULE_H_

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include "tfd_modules/module_api/pddlModuleTypes.h"
#include <tr1/tuple>
#include "globals.h"
#include <string.h>
using namespace std;
using namespace modules;

class Module
{
    public:
        string libCall;
        //string function_name;
        //string lib;
        ParameterList params;

        Module(istream &in);
        Module(string _libCall) :
            libCall(_libCall)
        {
        }
        Module()
        {
        }
        virtual ~Module();
};

class ConditionModule: public Module
{
    public:
        int var;

        conditionCheckerType checkCondition;

        ConditionModule(istream &in);
};

class EffectModule: public Module
{
    public:
        string internal_name;
        vector<int> writtenVars;

        applyEffectType applyEffect;

        EffectModule(istream &in);
};

class CostModule: public Module
{
    public:
        int var;

        conditionCheckerType checkCost;

        CostModule(istream &in);
};

class InitModule: public Module
{
    public:
        vector<string> parameters;

        InitModule(istream &in);
        void execInit();

        moduleInitType initModule;
};

#endif /* MODULE_H_ */
