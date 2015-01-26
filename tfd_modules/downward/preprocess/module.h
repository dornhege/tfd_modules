#ifndef MODULE_H_
#define MODULE_H_

#include "variable.h"
#include<vector>
#include<string>
#include<iostream>
#include<fstream>
using namespace std;

class Parameter
{
    public:
        string name;
        string type;
        string obj;

        Parameter(istream &in);
        virtual ~Parameter();
};

class Module
{
    private:
        void extractNameAndLib(const string &name);
    public:
        string name;
        string lib;
        vector<Parameter> params;

        Module(istream &in);
        void generate_cpp_input(ostream &outfile) const;
        virtual ~Module();
};

class ConditionModule: public Module
{
    public:
        Variable *var;

        ConditionModule(istream &in, const vector<Variable*> &variables);
        void generate_cpp_input(ostream &outfile) const;
};

class EffectModule: public Module
{
    public:
        string name;
        vector<Variable*> writtenVars;

        EffectModule(istream &in, const vector<Variable*> &variables);
        void generate_cpp_input(ostream &outfile) const;
};

class GroundingModule: public Module
{
    public:
        string name;

        GroundingModule(istream &in, const vector<Variable*> &variables);
        void generate_cpp_input(ostream &outfile) const;
};

class Translate
{
    public:
        string name;
        vector<string> params;
        Variable *var;
        Translate(istream &in, vector<Variable*> variables);
        void generate_cpp_input(ostream &outfile) const;
        virtual ~Translate();
};

class TranslatePredicate: public Translate
{
    public:
        int value;
        TranslatePredicate(istream &in, vector<Variable*> variables);
        void generate_cpp_input(ostream &outfile) const;
};

class TranslateFunction: public Translate
{
    public:
        TranslateFunction(istream &in, vector<Variable*> variables);
        void generate_cpp_input(ostream &outfile) const;
};

#endif /* MODULE_H_ */
