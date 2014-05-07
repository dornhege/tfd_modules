#include "helper_functions.h"
#include "operator.h"
#include "variable.h"

#include <cassert>
#include <iostream>
#include <fstream>
using namespace std;

Operator::Operator(istream &in, const vector<Variable *> &variables) {
    check_magic(in, "begin_operator");
    in >> ws;
    getline(in, name);
    int groundingCount;
    in >> groundingCount;
    for(int i = 0; i < groundingCount; i++) {
        string groundingCall;
        in >> groundingCall;
        assert(groundingCall.at(0) == 'g');
        groundingCalls.push_back(groundingCall);
    }
    int varNo;
    compoperator cop;
    in >> cop >> varNo;
    duration_cond = DurationCond(cop, variables[varNo]);
    int count;
    variables[varNo]->set_used_in_duration_condition();
    in >> count; // number of prevail at-start conditions
    for(int i = 0; i < count; i++) {
        int varNo, val;
        in >> varNo >> val;
        prevail_start.push_back(Prevail(variables[varNo], val));
    }
    in >> count; // number of prevail overall conditions
    for(int i = 0; i < count; i++) {
        int varNo, val;
        in >> varNo >> val;
        prevail_overall.push_back(Prevail(variables[varNo], val));
    }
    in >> count; // number of prevail at-end conditions
    for(int i = 0; i < count; i++) {
        int varNo, val;
        in >> varNo >> val;
        prevail_end.push_back(Prevail(variables[varNo], val));
    }
    in >> count; // number of pre_post_start conditions
    for(int i = 0; i < count; i++) {
        int eff_conds;
        vector<EffCond> ecs_start;
        vector<EffCond> ecs_overall;
        vector<EffCond> ecs_end;
        in >> eff_conds;
        for(int j = 0; j < eff_conds; j++) {
            int var, value;
            in >> var >> value;
            ecs_start.push_back(EffCond(variables[var], value));
        }
        in >> eff_conds;
        for(int j = 0; j < eff_conds; j++) {
            int var, value;
            in >> var >> value;
            ecs_overall.push_back(EffCond(variables[var], value));
        }
        in >> eff_conds;
        for(int j = 0; j < eff_conds; j++) {
            int var, value;
            in >> var >> value;
            ecs_end.push_back(EffCond(variables[var], value));
        }
        string effect;
        in >> effect;
        if(effect.at(0) != 'm') {
            int varNo = atoi(effect.c_str());
            if(variables[varNo]->is_functional()) {
                foperator assignop;
                int varNo2;
                in >> assignop >> varNo2;
                if(eff_conds)
                    numerical_effs_start.push_back(NumericalEffect(
                            variables[varNo], ecs_start, ecs_overall, ecs_end,
                            assignop, variables[varNo2]));
                else
                    numerical_effs_start.push_back(NumericalEffect(
                            variables[varNo], assignop, variables[varNo2]));
            } else {
                int val, newVal;
                in >> val >> newVal;
                if(eff_conds)
                    pre_post_start.push_back(PrePost(variables[varNo], ecs_start,
                            ecs_overall, ecs_end, val, newVal));
                else
                    pre_post_start.push_back(PrePost(variables[varNo], val, newVal));
            }
        } else {
            if (eff_conds)
                module_effs_start.push_back(ModuleEffect(effect, ecs_start,
                        ecs_overall, ecs_end));
            else
                module_effs_start.push_back(ModuleEffect(effect));
        }
    }
    in >> count; // number of pre_post_end conditions
    for(int i = 0; i < count; i++) {
        int eff_conds;
        vector<EffCond> ecs_start;
        vector<EffCond> ecs_overall;
        vector<EffCond> ecs_end;
        in >> eff_conds;
        for(int j = 0; j < eff_conds; j++) {
            int var, value;
            in >> var >> value;
            ecs_start.push_back(EffCond(variables[var], value));
        }
        in >> eff_conds;
        for(int j = 0; j < eff_conds; j++) {
            int var, value;
            in >> var >> value;
            ecs_overall.push_back(EffCond(variables[var], value));
        }
        in >> eff_conds;
        for(int j = 0; j < eff_conds; j++) {
            int var, value;
            in >> var >> value;
            ecs_end.push_back(EffCond(variables[var], value));
        }
        string effect;
        in >> effect;
        if(effect.at(0) != 'm') {
            int varNo = atoi(effect.c_str());
            if(variables[varNo]->is_functional()) {
                foperator assignop;
                int varNo2;
                in >> assignop >> varNo2;
                if(eff_conds)
                    numerical_effs_end.push_back(NumericalEffect(variables[varNo],
                            ecs_start, ecs_overall, ecs_end, assignop,
                            variables[varNo2]));
                else
                    numerical_effs_end.push_back(NumericalEffect(variables[varNo],
                            assignop, variables[varNo2]));
            } else {
                int val, newVal;
                in >> val >> newVal;
                if(eff_conds)
                    pre_post_end.push_back(PrePost(variables[varNo], ecs_start,
                            ecs_overall, ecs_end, val, newVal));
                else
                    pre_post_end.push_back(PrePost(variables[varNo], val, newVal));
            }
        } else {
            if (eff_conds)
                module_effs_end.push_back(ModuleEffect(effect, ecs_start,
                        ecs_overall, ecs_end));
            else
                module_effs_end.push_back(ModuleEffect(effect));
        }
    }
    check_magic(in, "end_operator");
}

void Operator::dump() const {
    cout << name << ":" << endl;
    cout << "grounding: " << endl;
    for(int i = 0; i < groundingCalls.size(); i++)
        cout << groundingCalls[i] << endl;
    cout << "duration:" << duration_cond.op << " "
        << duration_cond.var->get_name();
    cout << endl;
    cout << "prevail (at-start):";
    for(int i = 0; i < prevail_start.size(); i++)
        cout << "  " << prevail_start[i].var->get_name() << " := "
            << prevail_start[i].prev;
    cout << endl;
    cout << "prevail (overall):";
    for(int i = 0; i < prevail_overall.size(); i++)
        cout << "  " << prevail_overall[i].var->get_name() << " := "
            << prevail_overall[i].prev;
    cout << endl;
    cout << "prevail (at-end):";
    for(int i = 0; i < prevail_end.size(); i++)
        cout << "  " << prevail_end[i].var->get_name() << " := "
            << prevail_end[i].prev;
    cout << endl;
    cout << "pre-post (at-start):";
    for(int i = 0; i < pre_post_start.size(); i++) {
        if(pre_post_start[i].is_conditional_effect) {
            cout << "  if at-start(";
            for(int j = 0; j < pre_post_start[i].effect_conds_start.size(); j++)
                cout << pre_post_start[i].effect_conds_start[j].var->get_name()
                    << " := "
                    << pre_post_start[i].effect_conds_start[j].cond;
            cout << ")";
            cout << "  if overall(";
            for(int j = 0; j < pre_post_start[i].effect_conds_overall.size(); j++)
                cout
                    << pre_post_start[i].effect_conds_overall[j].var->get_name()
                    << " := "
                    << pre_post_start[i].effect_conds_overall[j].cond;
            cout << ")";
            cout << "  if at-end(";
            for(int j = 0; j < pre_post_start[i].effect_conds_end.size(); j++)
                cout << pre_post_start[i].effect_conds_end[j].var->get_name()
                    << " := " << pre_post_start[i].effect_conds_end[j].cond;
            cout << ")";
        }
        cout << " " << pre_post_start[i].var->get_name() << " : "
            << pre_post_start[i].pre <<" -> "<< pre_post_start[i].post;
    }
    cout << endl;
    cout << "pre-post (at-end):";
    for(int i = 0; i < pre_post_end.size(); i++) {
        if(pre_post_end[i].is_conditional_effect) {
            cout << "  if at-start(";
            for(int j = 0; j < pre_post_end[i].effect_conds_start.size(); j++)
                cout << pre_post_end[i].effect_conds_start[j].var->get_name()
                    << " := " << pre_post_end[i].effect_conds_start[j].cond;
            cout << ")";
            cout << "  if overall(";
            for(int j = 0; j < pre_post_end[i].effect_conds_overall.size(); j++)
                cout << pre_post_end[i].effect_conds_overall[j].var->get_name()
                    << " := "
                    << pre_post_end[i].effect_conds_overall[j].cond;
            cout << ")";
            cout << "  if at-end(";
            for(int j = 0; j < pre_post_end[i].effect_conds_end.size(); j++)
                cout << pre_post_end[i].effect_conds_end[j].var->get_name()
                    << " := " << pre_post_end[i].effect_conds_end[j].cond;
            cout << ")";
        }
        cout << " " << pre_post_end[i].var->get_name() << " : "
            << pre_post_end[i].pre <<" -> "<< pre_post_end[i].post;
    }
    cout << endl;
    cout << "numerical effects (at-start):";
    for(int i = 0; i< numerical_effs_start.size(); i++) {
        if(numerical_effs_start[i].is_conditional_effect) {
            cout << "  if at-start(";
            for(int j = 0; j
                    < numerical_effs_start[i].effect_conds_start.size(); j++)
                cout
                    << numerical_effs_start[i].effect_conds_start[j].var->get_name()
                    << " := "
                    << numerical_effs_start[i].effect_conds_start[j].cond;
            cout << ")";
            cout << "  if overall(";
            for(int j = 0; j
                    < numerical_effs_start[i].effect_conds_overall.size(); j++)
                cout
                    << numerical_effs_start[i].effect_conds_overall[j].var->get_name()
                    << " := "
                    << numerical_effs_start[i].effect_conds_overall[j].cond;
            cout << ")";
            cout << "  if at-end(";
            for(int j = 0; j
                    < numerical_effs_start[i].effect_conds_end.size(); j++)
                cout
                    << numerical_effs_start[i].effect_conds_end[j].var->get_name()
                    << " := "
                    << numerical_effs_start[i].effect_conds_end[j].cond;
            cout << ")";

        }
        cout << " " << numerical_effs_start[i].var->get_name() << " "
            << numerical_effs_start[i].fop << " "
            << numerical_effs_start[i].foperand->get_name();
    }
    cout << endl;
    cout << "numerical effects (at-end):";
    for(int i = 0; i< numerical_effs_end.size(); i++) {
        if(numerical_effs_end[i].is_conditional_effect) {
            cout << "  if at-start(";
            for(int j = 0; j < numerical_effs_end[i].effect_conds_start.size(); j++)
                cout
                    << numerical_effs_end[i].effect_conds_start[j].var->get_name()
                    << " := "
                    << numerical_effs_end[i].effect_conds_start[j].cond;
            cout << ")";
            cout << "  if overall(";
            for(int j = 0; j
                    < numerical_effs_end[i].effect_conds_overall.size(); j++)
                cout
                    << numerical_effs_end[i].effect_conds_overall[j].var->get_name()
                    << " := "
                    << numerical_effs_end[i].effect_conds_overall[j].cond;
            cout << ")";
            cout << "  if at-end(";
            for(int j = 0; j < numerical_effs_end[i].effect_conds_end.size(); j++)
                cout
                    << numerical_effs_end[i].effect_conds_end[j].var->get_name()
                    << " := "
                    << numerical_effs_end[i].effect_conds_end[j].cond;
            cout << ")";
        }
        cout << " " << numerical_effs_end[i].var->get_name() << " "
            << numerical_effs_end[i].fop << " "
            << numerical_effs_end[i].foperand->get_name();
    }
    cout << endl;
}

void Operator::strip_unimportant_effects() {
    int new_index = 0;
    for(int i = 0; i < pre_post_start.size(); i++) {
        if(pre_post_start[i].var->get_level() != -1)
            pre_post_start[new_index++] = pre_post_start[i];
    }
    pre_post_start.erase(pre_post_start.begin() + new_index,
            pre_post_start.end());
    new_index = 0;
    for(int i = 0; i < pre_post_end.size(); i++) {
        if(pre_post_end[i].var->get_level() != -1)
            pre_post_end[new_index++] = pre_post_end[i];
    }
    pre_post_end.erase(pre_post_end.begin() + new_index, pre_post_end.end());
    new_index = 0;
    for(int i = 0; i < numerical_effs_start.size(); i++) {
        if(numerical_effs_start[i].var->get_level() != -1)
            numerical_effs_start[new_index++] = numerical_effs_start[i];
    }
    numerical_effs_start.erase(numerical_effs_start.begin() + new_index, numerical_effs_start.end());
    new_index = 0;
    for(int i = 0; i < numerical_effs_end.size(); i++) {
        if(numerical_effs_end[i].var->get_level() != -1)
            numerical_effs_end[new_index++] = numerical_effs_end[i];
    }
    numerical_effs_end.erase(numerical_effs_end.begin() + new_index, numerical_effs_end.end());
}

bool Operator::is_redundant() const {
    return (pre_post_start.empty() && pre_post_end.empty()
            && module_effs_start.empty() && module_effs_end.empty() && groundingCalls.empty()
            && numerical_effs_start.empty() && numerical_effs_end.empty());//FIXME: Is this correct?
}

void strip_operators(vector<Operator> &operators) {
    int old_count = operators.size();
    int new_index = 0;
    for(int i = 0; i < operators.size(); i++) {
        operators[i].strip_unimportant_effects();
        if(!operators[i].is_redundant())
            operators[new_index++] = operators[i];
    }
    operators.erase(operators.begin() + new_index, operators.end());
    cout << operators.size() << " of " << old_count << " operators necessary."
        << endl;
}
void Operator::write_prevails(ostream &outfile, const vector<Prevail> &prevails) const {
    outfile << prevails.size() << endl;
    for(int i = 0; i < prevails.size(); i++) {
        assert(prevails[i].var->get_level() != -1);
        if(prevails[i].var->get_level() != -1)
            outfile << prevails[i].var->get_level() << " "
                << prevails[i].prev << endl;
    }
}

void Operator::write_effect_conds(ostream &outfile, const vector<EffCond> &conds) const {
    outfile << conds.size() << endl;
    for (int j = 0; j < conds.size(); j++)
        outfile << conds[j].var->get_level() << " " << conds[j].cond << endl;
}

void Operator::write_pre_posts(ostream &outfile, const vector<PrePost> &pre_posts) const {
    outfile << pre_posts.size() << endl;
    for (int i = 0; i < pre_posts.size(); i++) {
        assert(pre_posts[i].var->get_level() != -1);
        write_effect_conds(outfile, pre_posts[i].effect_conds_start);
        write_effect_conds(outfile, pre_posts[i].effect_conds_overall);
        write_effect_conds(outfile, pre_posts[i].effect_conds_end);
        outfile << pre_posts[i].var->get_level() << " " << pre_posts[i].pre << " "
            << pre_posts[i].post << endl;
    }
}

void Operator::write_num_effect(ostream &outfile,
        const vector<NumericalEffect> &num_effs) const {
    outfile << num_effs.size() << endl;
    for (int i = 0; i < num_effs.size(); i++) {
        assert(num_effs[i].var->get_level() != -1);
        write_effect_conds(outfile, num_effs[i].effect_conds_start);
        write_effect_conds(outfile, num_effs[i].effect_conds_overall);
        write_effect_conds(outfile, num_effs[i].effect_conds_end);
        outfile << num_effs[i].var->get_level() << " " << num_effs[i].fop
            << " " << num_effs[i].foperand->get_level() << endl;
    }
}

void Operator::write_module_effect(ostream &outfile,
        const vector<ModuleEffect> &mod_effs) const
{
    outfile << mod_effs.size() << endl;
    for (int i = 0; i < mod_effs.size(); ++i) {
        // whoever C&P this - fix it    assert(numeffs[i].var->get_level() != -1);
        write_effect_conds(outfile, mod_effs[i].effect_conds_start);
        write_effect_conds(outfile, mod_effs[i].effect_conds_overall);
        write_effect_conds(outfile, mod_effs[i].effect_conds_end);
        outfile << mod_effs[i].name << endl;
    }
}

void Operator::write_grounding_calls(ostream &outfile, const vector<string> & grounding_calls) const
{
    outfile << grounding_calls.size() << endl;
    for (int i = 0; i < grounding_calls.size(); ++i) {
        outfile << grounding_calls[i] << endl;
    }
}

void Operator::generate_cpp_input(ostream &outfile) const {
    // beim Einlesen in search feststellen, ob leerer Operator
    outfile << "begin_operator" << endl;
    outfile << name << endl;
    write_grounding_calls(outfile, groundingCalls);
    //duration
    outfile << duration_cond.op << " " << duration_cond.var->get_level()
        << endl;
    write_prevails(outfile, prevail_start);
    write_prevails(outfile, prevail_overall);
    write_prevails(outfile, prevail_end);

    write_pre_posts(outfile, pre_post_start);
    write_pre_posts(outfile, pre_post_end);

    write_num_effect(outfile, numerical_effs_start);
    write_num_effect(outfile, numerical_effs_end);

    write_module_effect(outfile, module_effs_start);
    write_module_effect(outfile, module_effs_end);

    outfile << "end_operator" << endl;
}
