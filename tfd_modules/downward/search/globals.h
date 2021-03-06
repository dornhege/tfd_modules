#ifndef GLOBALS_H
#define GLOBALS_H

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <string>
#include "module.h"
#include <cmath>
#include <limits>

using namespace std;

#include "causal_graph.h"

class AxiomEvaluator;
//class CausalGraph;
class DomainTransitionGraph;
class Operator;
class Axiom;
class LogicAxiom;
class NumericAxiom;
class TimeStampedState;
class SuccessorGenerator;
class Analysis;

struct PlanStep
{
    double start_time;
    double duration;
    const Operator* op;
    const TimeStampedState* pred;   ///< Optional pointer to the predecessor state

    PlanStep(double st, double dur, const Operator* o,
        const TimeStampedState* p) :
            start_time(st), duration(dur), op(o), pred(p)
    {
    }

        void dump() const;
};

typedef std::vector<PlanStep> Plan;
typedef std::vector<TimeStampedState*> PlanTrace;

bool state_equals(double a, double b);
bool time_equals(double a, double b);
bool double_equals(double a, double b);

const double REALLYBIG = numeric_limits<double>::max();
const double REALLYSMALL = -numeric_limits<double>::max();

void read_everything(istream &in);
void dump_everything();
void dump_DTGs();

void check_magic(istream &in, string magic);

enum variable_type
{
    logical,
    primitive_functional,
    subterm_functional,
    comparison,
    module,
    costmodule
};

extern int g_last_arithmetic_axiom_layer;
extern int g_comparison_axiom_layer;
extern int g_first_logic_axiom_layer;
extern int g_last_logic_axiom_layer;
extern vector<string> g_variable_name;
extern vector<int> g_variable_domain;
extern vector<int> g_axiom_layers;
extern vector<double> g_default_axiom_values;
extern vector<variable_type> g_variable_types;
extern TimeStampedState *g_initial_state;
extern ObjectTypeMap g_objectTypes;
extern vector<pair<int, double> > g_goal;
extern vector<Operator> g_operators;
extern set<Operator> g_grounded_operators;
extern vector<Axiom*> g_axioms;
extern AxiomEvaluator *g_axiom_evaluator;
extern SuccessorGenerator *g_successor_generator;
extern vector<DomainTransitionGraph *> g_transition_graphs;
extern CausalGraph *g_causal_graph;

extern Analysis g_analysis;

class PlannerParameters;
extern PlannerParameters g_parameters;

inline bool is_functional(int var)
{
    const variable_type& vt = g_variable_types[var];
    return (vt == primitive_functional || vt == subterm_functional);
}

extern Operator *g_let_time_pass;
extern Operator *g_wait_operator;

enum assignment_op
{
    assign = 0, scale_up = 1, scale_down = 2, increase = 3, decrease = 4
};
enum binary_op
{
    add = 0,
    subtract = 1,
    mult = 2,
    divis = 3,
    lt = 4,
    le = 5,
    eq = 6,
    ge = 7,
    gt = 8,
    ue = 9
};
enum trans_type
{
    start = 0, end = 1, compressed = 2, ax = 3
};
enum condition_type
{
    start_cond = 0, overall_cond = 1, end_cond = 2, ax_cond
};

istream& operator>>(istream &is, assignment_op &aop);
ostream& operator<<(ostream &os, const assignment_op &aop);

istream& operator>>(istream &is, binary_op &bop);
ostream& operator<<(ostream &os, const binary_op &bop);

istream& operator>>(istream &is, trans_type &tt);
ostream& operator<<(ostream &os, const trans_type &tt);

istream& operator>>(istream &is, condition_type &fop);
ostream& operator<<(ostream &os, const condition_type &fop);

void printSet(const set<int> s);

#endif
