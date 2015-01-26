#ifndef PARTIAL_ORDER_LIFTER_H
#define PARTIAL_ORDER_LIFTER_H

#include "state.h"
#include "operator.h"
#include "scheduler.h"
#include "plannerParameters.h"
#include <list>

enum ActionType
{
    start_action = 0,
    end_action = 1,
    dummy_start_action = 2,
    dummy_end_action = 3
};

struct InstantPlanStep
{

        ActionType type;
        int endAction;
        int actionFinishingImmeadatlyAfterThis;
        double timepoint;
        double duration;
        const Operator* op;
        string name;

        int correspondingPlanStep;

        std::set<int> precondition_vars;
        std::set<int> effect_vars;

        std::vector<Prevail> preconditions;
        std::set<int> effect_cond_vars;
        std::vector<PrePost> effects;
        std::vector<Prevail> overall_conds;

        //    void addCondVars(std::set<int>& effect_cond_vars,
        //            const std::vector<Prevail>& conds) {
        //        for(int i = 0; i < conds.size(); ++i) {
        //            effect_cond_vars.insert(conds[i].var);
        //        }
        //    }
        //
        //    void initializePreconditionVars(const std::vector<Prevail> &prevails,
        //            const std::vector<PrePost> &pre_posts) {
        //        for(size_t i = 0; i < prevails.size(); i++) {
        //            precondition_vars.insert(prevails[i].var);
        //            preconditions.push_back(prevails[i]);
        //        }
        //        for(int i = 0; i < pre_posts.size(); ++i) {
        //            if(g_variable_types[pre_posts[i].var] == logical && pre_posts[i].pre != -1) {
        //                precondition_vars.insert(pre_posts[i].var);
        //                preconditions.push_back(Prevail(pre_posts[i].var,pre_posts[i].post));
        //            }
        //        }
        //        for(size_t i = 0; i < pre_posts.size(); i++) {
        //            effect_vars.insert(pre_posts[i].var);
        //            addCondVars(effect_cond_vars,pre_posts[i].cond_start);
        //            addCondVars(effect_cond_vars,pre_posts[i].cond_overall);
        //            addCondVars(effect_cond_vars,pre_posts[i].cond_end);
        //        }
        //
        //        //      todo();
        //    }


        //    void initializePreconditionVars() {
        //        if(type == start_action) {
        //            initializePreconditionVars(op->get_prevail_start(),
        //                    op->get_pre_post_start());
        //        } else {
        //            initializePreconditionVars(op->get_prevail_end(),
        //                    op->get_pre_post_end());
        //        }
        //    }
        //
        //    void initializeEffectVars(const std::vector<PrePost> &pre_posts) {
        //        //    todo();
        //    }
        //
        //    void initializeEffectVars() {
        //        if(type == start_action) {
        //            initializeEffectVars(op->get_pre_post_start());
        //        } else {
        //            initializeEffectVars(op->get_pre_post_end());
        //        }
        //    }
        //
        //    void initializeRelevantPreconditions(const PlanTrace &trace) {
        //        //    todo();
        //    }
        //
        //    void initializeTriggeredEffects(const PlanTrace &trace) {
        //        //    todo();
        //    }
        //
        //    void initialize(const PlanTrace &trace) {
        //        initializePreconditionVars();
        //        initializeEffectVars();
        //        initializeRelevantPreconditions(trace);
        //        initializeTriggeredEffects(trace);
        //    }

        InstantPlanStep(ActionType _type, double _timepoint, double _duration,
                const Operator* _op) :
            type(_type), timepoint(_timepoint), duration(_duration), op(_op)
        {
            endAction = -1;
            actionFinishingImmeadatlyAfterThis = -1;
            name = type + "." + op->get_name();
            //        cout << "NAME: " << name << endl;
            //        initialize(trace);
        }

        bool operator<(const InstantPlanStep &other) const
        {
            return timepoint < other.timepoint;
        }

        void dump();
        void print_name();

};

typedef std::vector<InstantPlanStep> InstantPlan;

/*
 vector<Prevail> prevail_start; // var, val
 vector<Prevail> prevail_overall; // var, val
 vector<Prevail> prevail_end; // var, val
 vector<PrePost> pre_post_start; // var, old-val, new-val
 vector<PrePost> pre_post_end; // var, old-val, new-val
 */

typedef std::pair<int, int> Ordering;

class PartialOrderLifter
{
        struct doubleEquComp
        {
                bool operator()(const double& lhs, const double& rhs) const
                {
                    return lhs + g_parameters.epsTimeComparison < rhs;
                }
        };
    private:
        Plan plan;
        const PlanTrace trace;

        InstantPlan instant_plan;
        std::set<Ordering> partial_order;

        void findTriggeringEffects(
                const TimeStampedState* stateBeforeHappening,
                const TimeStampedState* stateAfterHappening,
                vector<PrePost>& las);
        void findTriggeringEffectsForInitialState(
                const TimeStampedState* tsstate, vector<PrePost>& effects);
        void findAllEffectCondVars(const ScheduledOperator& new_op,
                set<int>& effect_cond_vars, ActionType type);
        void findPreconditions(const ScheduledOperator& new_op,
                vector<Prevail>& preconditions, ActionType type);
        //      todo();

        int getIndexOfPlanStep(const ScheduledOperator& op, double timestamp);
        void buildInstantPlan();
        void dumpInstantPlan();
        void dumpOrdering();
        void buildTrace();
        void buildPartialOrder();
        void sortPlan(Plan& plan);
        Plan createAndSolveSTN();

    public:
        PartialOrderLifter(const Plan &_plan, const PlanTrace &_trace) :
            plan(_plan), trace(_trace)
        {
            instant_plan.reserve(sizeof(InstantPlanStep)
                    * (plan.size() * 2 + 2));
        }

        Plan lift();
};

#endif

/*
 PSEUDOCODE:
 ===========

 zusätzlich:
 constraints, die dafür sorgen, dass die duration variable passt + constraints
 die dafür sorgen, dass numerische effekte passen (aufeinander aufbauen).

 Alle constraints, die es auch bei crikey gibt, plus constraints, die
 erzwingen, dass instant actions in derselben Reihenfolge bleiben,
 falls sie bzgl. bedingter effekte interferieren, d.h. wenn die eine
 aktion eine variable im effekt hat, die bei der anderen in einer
 effektvorbedingung oder der bedingung vorkommt. Wenn wir alle diese
 Constraints haben, dann ist garantiert, dass alle Aktionen genau die
 Effekte haben, die sie auch im urspruenglichen Plan besitzen. Dann
 dürfen wir aber die instant actions so zu unbedingten Aktionen
 umschreiben, dass sie (unbedingt) eben genau die effekte haben, die
 sie in dem gegebenen konkreten Plan haben. Auf diesen Aktionen kann
 man dann den Partial-Order-Lifting-Algorithmus von Crikey anwenden.


 Schritt 1) Instant-Actions erstellen, die Start- und End-Aktionen
 entsprechen, noch mit bedingten Effekten. Dummy-Aktionen
 für Init und Goal hinzufuegen.

 Schritt 2) Fuer jede Aktion a sei pre_var(a) die Menge aller
 Variablen, die in der Vorbedingung von a oder der Bedingung
 eines bedingten Effekts von a erwähnt werden. Ferner sei
 eff_var(a) die Menge aller Variablen, die in irgendeinem
 (bedingten oder unbedingten) Effekt von a manipuliert
 werden können. Füge nun ein ordering a_i < a_j hinzu für
 alle i < j so dass (pre_var(a_j) \intersect eff_var(a_i))
 \union (pre_var(a_i) \intersect eff_var(a_j)) != empty
 (d.h. ordering bleibt erhalten, wenn die Aktionen
 *irgendwie* interferieren könnten).

 Schritt 3) Für jede Nicht-Instant-Aktion a sei overall_cond_var(a) die
 Menge aller Variablen, die in der Overall-Bedingung von a
 erwähnt werden. Für jede solche Aktion a und alle
 Instant-Aktionen a', die im Originalplan NICHT zwischen den
 Anfangs- und End-Instant-Aktionen von a auftreten, füge den
 Constraint a' < start_instant_action(a)
 bzw. end_instant_action(a) < a' zu den gesammelten
 Constraints hinzu, je nachdem, wie die Reihenfolge in dem
 Orignalplan ist.

 Schritt 4) Schreibe alle Aktionen so um, dass sie nur noch bedingte
 Effekte haben, und zwar genau die, die sie in der
 Planausführung tatsächlich haben. Die Vorbedingungen seien
 genau die ursprünglichen Aktionsvorbedingungen plus die
 effektvorbedingungen der Effekte, die tatsächlich
 triggern. Ab jetzt sei (abuse of notation) a immer eine so
 umgeschriebene Aktion a.

 Schritt 5) Führe auf dieser Aktionsmenge den
 Partial-Order-Lifting-Algorithmus von Crikey durch und wirf
 die dabei ermittelten orederings mit denen von oben
 (Schritt 1) zusammen.

 Schritt 6) Erzeuge entsprechendes STN (am besten gleich mit
 epsilon-Abständen drin) und löse das STN mit entsprechendem
 Scheduler.

 Schritt 7) Übersetzte Scheduling-Ergebnis in Plan zurück.


 + ABGELEITETE PRAEDIKATE!!!
 */
