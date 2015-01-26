#include "module.h"
#include <cassert>

Module::Module(istream &in)
{
    string complete_name;
    in >> complete_name;
    extractNameAndLib(complete_name);
    int count;
    in >> count;
    for (int i = 0; i < count; ++i) {
        params.push_back(Parameter(in));
    }
}

void Module::generate_cpp_input(ostream &outfile) const
{
    outfile << name << " " << lib << " " << params.size() << " ";
    for (int i = 0; i < params.size(); ++i) {
        outfile << " " << params[i].name << " " << params[i].type << " "
                << params[i].obj << " ";
    }
}

ConditionModule::ConditionModule(istream &in,
        const vector<Variable*> &variables) :
    Module(in)
{
    int var_no;
    in >> var_no;
    var = variables[var_no];
    var->set_module();

}

void ConditionModule::generate_cpp_input(ostream &outfile) const
{
    Module::generate_cpp_input(outfile);
    outfile << var->get_level() << endl;
}

EffectModule::EffectModule(istream &in, const vector<Variable*> &variables) :
    Module(in)
{
    in >> name;
    int count;
    in >> count;
    writtenVars.reserve(count);
    int var_no;
    for (int i = 0; i < count; ++i) {
        in >> var_no;
        Variable *var = variables[var_no];
        writtenVars.push_back(var);
        var->set_module();
    }
}

void EffectModule::generate_cpp_input(ostream &outfile) const
{
    Module::generate_cpp_input(outfile);
    outfile << name << " " << writtenVars.size();
    for (int i = 0; i < writtenVars.size(); ++i) {
        outfile << " " << writtenVars[i]->get_level();
    }
    outfile << endl;
}

GroundingModule::GroundingModule(istream &in, const vector<Variable*> &variables) :
    Module(in)
{
    in >> name;
}

void GroundingModule::generate_cpp_input(ostream &outfile) const
{
    Module::generate_cpp_input(outfile);
    outfile << name << endl; 
}

void Module::extractNameAndLib(const string &complete_name)
{
    int posOfAt = complete_name.find("@");
    name = complete_name.substr(0, posOfAt);
    lib = complete_name.substr(posOfAt + 1);
}

Translate::Translate(istream &in, vector<Variable*> variables)
{
    in >> name;
    int count;
    in >> count;
    params.reserve(count);
    string param;
    for (int i = 0; i < count; ++i) {
        in >> param;
        params.push_back(param);
    }
    int var_no;
    in >> var_no;
    var = variables[var_no];
}

void Translate::generate_cpp_input(ostream &outfile) const
{
    outfile << name;
    outfile << " " << params.size();
    for (int i = 0; i < params.size(); ++i) {
        outfile << " " << params[i];
    }
    outfile << " " << var->get_level();
}

TranslatePredicate::TranslatePredicate(istream &in, vector<Variable*> variables) :
    Translate(in, variables)
{
    in >> value;
}

void TranslatePredicate::generate_cpp_input(ostream &outfile) const
{
    Translate::generate_cpp_input(outfile);
    outfile << " " << value << endl;
}

TranslateFunction::TranslateFunction(istream &in, vector<Variable*> variables) :
    Translate(in, variables)
{
}

void TranslateFunction::generate_cpp_input(ostream &outfile) const
{
    Translate::generate_cpp_input(outfile);
    outfile << endl;
}

Parameter::Parameter(istream &in)
{
    in >> name >> type >> obj;
}

Module::~Module()
{
}

Parameter::~Parameter()
{
}

Translate::~Translate()
{
}

