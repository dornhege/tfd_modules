#include "partial_order_lifter.h"

#include<iomanip>

void InstantPlanStep::print_name()
{
    if (type == dummy_start_action) {
        cout << "dummy_start_action";
    } else if (type == dummy_end_action) {
        cout << "dummy_end_action";
    } else {
        cout << name;
    }
}

void InstantPlanStep::dump()
{
    print_name();
    cout << endl;
    cout << "     Timepoint:" << timepoint << endl;
    cout << "     precond vars:" << endl;
    set<int>::iterator it;
    for (it = precondition_vars.begin(); it != precondition_vars.end(); ++it) {
        cout << "      " << *it << endl;
    }
    cout << "     effect vars:" << endl;
    for (it = effect_vars.begin(); it != effect_vars.end(); ++it) {
        cout << "      " << *it << endl;
    }
    cout << "     effect_cond_vars:" << endl;
    for (it = effect_cond_vars.begin(); it != effect_cond_vars.end(); ++it) {
        cout << "      " << *it << endl;
    }
    cout << "     effects:" << endl;
    for (int i = 0; i < effects.size(); ++i) {
        cout << "      " << effects[i].var << ": " << effects[i].post << endl;
    }
    cout << "     preconditions:" << endl;
    for (int i = 0; i < preconditions.size(); ++i) {
        cout << "      " << preconditions[i].var << ": "
            << preconditions[i].prev << endl;
    }
    cout << "     overall conds:" << endl;
    for (int i = 0; i < overall_conds.size(); ++i) {
        cout << "      " << overall_conds[i].var << ": "
            << overall_conds[i].prev << endl;
    }
    if (endAction != -1) {
        cout << "     endAction: " << endAction << endl;
    }
    if (actionFinishingImmeadatlyAfterThis) {
        cout << "     actionFinishingImmeadatlyAfterThis: "
            << actionFinishingImmeadatlyAfterThis << endl;
    }
}

void PartialOrderLifter::sortPlan(Plan& plan)
{
    bool changePerformed = false;
    do {
        changePerformed = false;
        for (int i = 0; i < plan.size() - 1; ++i) {
            if (plan[i].start_time - EPSILON > plan[i + 1].start_time) {
                swap(plan[i], plan[i + 1]);
                changePerformed = true;
            }
        }
    } while (changePerformed);
}

Plan PartialOrderLifter::createAndSolveSTN()
{
    vector<string> variable_names;
    for (int i = 0; i < instant_plan.size(); ++i) {
        variable_names.push_back(instant_plan[i].name);
    }

    SimpleTemporalProblem stn(variable_names);

    for (int i = 0; i < instant_plan.size(); ++i) {
        if (instant_plan[i].type == start_action) {
            assert(instant_plan[i].endAction != -1);
            // assert that start time point of actions are non-negative
            stn.setUnboundedIntervalFromXZero(i, 0.0);
            // assert that differences between start and end time points are exactly
            // the durations of the actions
            stn.setSingletonInterval(i, instant_plan[i].endAction,
                    instant_plan[i].duration);
        } else {
            assert(instant_plan[i].type == end_action);
            // assert that two actions ending at the same time point in the original plan
            // also do so in the scheduled one
            if (instant_plan[i].actionFinishingImmeadatlyAfterThis != -1) {
                //                stn.setSingletonInterval(i,instant_plan[i].actionFinishingImmeadatlyAfterThis,EPSILON);
            }
        }
    }

    // assert that causal relationships are preserved
    for (set<Ordering>::iterator it = partial_order.begin(); it
            != partial_order.end(); ++it) {
        stn.setUnboundedInterval(it->first, it->second, EPSILON);
    }

    //    // assert that overall conditions are preserved
    //    for(int i = 0; i < plan.size(); ++i) {
    //        const vector<Prevail>& overall_conds = plan[i].op->get_prevail_overall();
    //        for(int j = 0; j < overall_conds.size(); ++j) {
    //            for(int k = 0; k < instant_plan.size(); ++k) {
    //                InstantPlanStep& step = instant_plan[k];
    //                for(int l = 0; l < step.effects.size(); ++l) {
    //                    if(overall_conds[j].var == step.effects[l].var && overall_conds[j].prev != step.effects[l].post) {
    //                        if(plan[i].start_time < step.start_time) {
    //                            stn.st
    //                        }
    //                    }
    //                }
    //            }
    //        }
    //
    //    }

    //    std::cout << "Unsolved Simple Temporal Network:" << std::endl;
    //
    //
    //    std::cout << "=================================" << std::endl;
    //    stn.dump();

    //    stn.solve();
    bool isValid = stn.solveWithP3C();
    assert(isValid);
    vector<Happening> happenings = stn.getHappenings(true);

    map<string, int> startedActions;

    for (int i = 0; i < happenings.size(); ++i) {
        if (instant_plan[happenings[i].second].type == start_action) {
            assert(instant_plan[happenings[i].second].correspondingPlanStep < instant_plan.size());
            plan[instant_plan[happenings[i].second].correspondingPlanStep].start_time
                = happenings[i].first;
        }
    }

    sortPlan(plan);

    return plan;

    //    double h = stn.getMaximalTimePointInTightestSchedule();
    //
    //    std::cout << "Solved Simple Temporal Network:" << std::endl;
    //    std::cout << "===============================" << std::endl;
    //    stn.dump();
    //
    //    std::cout << "Corresponding happenings:" << std::endl;
    //    std::cout << "=========================" << std::endl;
    //    stn.dumpSolution();
    //
    //    std::cout << "Corresponding heuristic value:" << std::endl;
    //    std::cout << "==============================" << std::endl;
    //    std::cout << h << std::endl;

}

Plan PartialOrderLifter::lift()
{
    //    instant_plan.clear();
    //    partial_order.clear();
    buildInstantPlan();
    //    dumpInstantPlan();
    buildPartialOrder();
    //    dumpOrdering();
    return createAndSolveSTN();
}

void PartialOrderLifter::buildPartialOrder()
{
    std::vector<std::vector<Prevail> > primary_add;
    primary_add.resize(instant_plan.size());

    for (int i = instant_plan.size() - 1; i >= 0; --i) {
        //        cout << "Looking at op: ";
        //        instant_plan[i].print_name();
        //        cout << endl;

        // step (a) find achievers for all action preconditions and add corresponding ordering constraints
        InstantPlanStep &step = instant_plan[i];
        for (int j = 0; j < step.preconditions.size(); ++j) {
            //            cout << "  Search achiever for ";
            //            step.preconditions[j].dump();
            //            cout << endl;
            bool achiever_found = false;
            for (int k = i - 1; k >= 0; --k) {
                if (!achiever_found) {
                    InstantPlanStep &temp_step = instant_plan[k];
                    for (int l = 0; l < temp_step.effects.size(); ++l) {
                        //                        cout << " " << step.preconditions[j].var << " " << temp_step.effects[l].var << " " << step.preconditions[j].prev << " " << temp_step.effects[l].post << endl;
                        if (step.preconditions[j].var
                                == temp_step.effects[l].var
                                && step.preconditions[j].prev
                                == temp_step.effects[l].post && step.op
                                != temp_step.op) {
                            // achiever found!
                            //                            cout << "   achiever found: ";
                            //                            temp_step.print_name();
                            //                            cout << endl;
                            partial_order.insert(make_pair(k, i));
                            primary_add[k].push_back(step.preconditions[j]);
                            achiever_found = true;
                            break;
                        }
                    }
                } else {
                    achiever_found = false;
                    break;
                }
            }
        }

        // step (b) demote actions threatening preconditions
        for (int j = 0; j < step.effects.size(); ++j) {
            for (int k = i - 1; k >= 0; --k) {
                InstantPlanStep &temp_step = instant_plan[k];
                for (int l = 0; l < temp_step.preconditions.size(); ++l) {
                    if (temp_step.preconditions[l].var == step.effects[j].var
                            && temp_step.preconditions[l].prev
                            != step.effects[j].post && step.op
                            != temp_step.op) {
                        //                        step.print_name();
                        //                        cout << " threatens ";
                        //                        temp_step.print_name();
                        //                        cout << endl;
                        partial_order.insert(make_pair(k, i));
                    }
                }
            }
        }

        // mutex conditions: no two actions may affect the same variable at the same time!
        for (int j = 0; j < step.effects.size(); ++j) {
            for (int k = i - 1; k >= 0; --k) {
                InstantPlanStep &temp_step = instant_plan[k];
                for (int l = 0; l < temp_step.effects.size(); ++l) {
                    if (temp_step.effects[l].var == step.effects[j].var
                            && temp_step.effects[l].post
                            != step.effects[j].post && step.op
                            != temp_step.op) {
                        partial_order.insert(make_pair(k, i));
                    }
                }
            }
        }

        // step (c)
        for (int j = 0; j < primary_add[i].size(); ++j) {
            for (int k = i - 1; k >= 0; --k) {
                InstantPlanStep &temp_step = instant_plan[k];
                for (int l = 0; l < temp_step.effects.size(); ++l) {
                    if (temp_step.effects[l].var == primary_add[i][j].var
                            && temp_step.effects[l].post
                            != primary_add[i][j].prev && step.op
                            != temp_step.op) {
                        //                        step.print_name();
                        //                        cout << " threatens primary add of ";
                        //                        temp_step.print_name();
                        //                        cout << endl;
                        partial_order.insert(make_pair(k, i));
                    }
                }
            }
        }

        // overallconds must be satiesfied!
        for (int j = 0; j < step.overall_conds.size(); ++j) {
            for (int k = 0; k < instant_plan.size(); ++k) {
                InstantPlanStep &temp_step = instant_plan[k];
                for (int l = 0; l < temp_step.effects.size(); ++l) {
                    if (temp_step.effects[l].var == step.overall_conds[j].var
                            /*&& temp_step.effects[l].post != step.overall_conds[j].prev*/&& step.op
                            != temp_step.op) {
                        if (step.timepoint >= temp_step.timepoint && step.type
                                == start_action) {
                            //                            temp_step.print_name();
                            //                            cout << " could break overall cond of ";
                            //                            step.print_name();
                            //                            cout << endl;
                            partial_order.insert(make_pair(k, i));
                        } else if (step.timepoint <= temp_step.timepoint
                                && step.type == end_action) {
                            //                            step.print_name();
                            //                            cout << " could break overall cond of ";
                            //                            temp_step.print_name();
                            //                            cout << endl;
                            partial_order.insert(make_pair(i, k));
                        }
                    }
                }

            }
        }

    }

    //    fixme();
}

void PartialOrderLifter::dumpInstantPlan()
{
    for (int i = 0; i < instant_plan.size(); ++i) {
        cout << i << ": ";
        instant_plan[i].dump();
        cout << "-----------------------------------" << endl;
    }
}

void PartialOrderLifter::dumpOrdering()
{
    cout << "numberOfOrderingConstraints: " << partial_order.size() << endl;
    cout << "Ordering:" << endl;
    for (std::set<Ordering>::iterator it = partial_order.begin(); it
            != partial_order.end(); ++it) {
        cout << " ";
        instant_plan[it->first].print_name();
        cout << " < ";
        instant_plan[it->second].print_name();
        cout << endl;
    }
}

void PartialOrderLifter::findTriggeringEffects(
        const TimeStampedState* stateBeforeHappening,
        const TimeStampedState* stateAfterHappening, vector<PrePost>& effects)
{
    effects.clear();
    assert(stateAfterHappening->state.size() == stateBeforeHappening->state.size());
    for (int i = 0; i < stateAfterHappening->state.size(); ++i) {
        if (!(double_equals(stateBeforeHappening->state[i],
                        stateAfterHappening->state[i]))) {
            effects.push_back(PrePost(i, stateAfterHappening->state[i]));
        }
    }
}

//void PartialOrderLifter::findTriggeringEffectsForInitialState(const TimeStampedState* tsstate, vector<PrePost>& effects) {
//    for(int i = 0; i < tsstate->state.size(); ++i) {
////        effects.push_back(PrePost(i,tsstate->state[i]));
//    }
//}

void PartialOrderLifter::findAllEffectCondVars(const ScheduledOperator& new_op,
        set<int>& effect_cond_vars, ActionType type)
{
    // FIXME: start_type -> start_conds, end_type -> end_conds, overall_conds??
    const vector<PrePost>* pre_posts;
    if (type == start_action) {
        pre_posts = &new_op.get_pre_post_start();
    } else {
        assert(type == end_action);
        pre_posts = &new_op.get_pre_post_end();
    }
    const vector<Prevail>* prevails;
    for (int i = 0; i < pre_posts->size(); ++i) {
        if (type == start_action) {
            prevails = &(*pre_posts)[i].cond_start;
        } else {
            assert(type == end_action);
            prevails = &(*pre_posts)[i].cond_end;
        }
        for (int j = 0; j < prevails->size(); ++j) {
            effect_cond_vars.insert((*prevails)[i].var);
        }
    }
}

void PartialOrderLifter::findPreconditions(const ScheduledOperator& new_op,
        vector<Prevail>& preconditions, ActionType type)
{
    // FIXME: start_type -> start_conds, end_type -> end_conds, overall_conds??
    const std::vector<Prevail> *prevails;
    const std::vector<PrePost> *pre_posts;
    if (type == start_action) {
        prevails = &new_op.get_prevail_start();
        pre_posts = &new_op.get_pre_post_start();
    } else {
        prevails = &new_op.get_prevail_end();
        pre_posts = &new_op.get_pre_post_end();
    }
    for (size_t i = 0; i < prevails->size(); i++) {
        preconditions.push_back((*prevails)[i]);
    }
    for (int i = 0; i < pre_posts->size(); ++i) {
        if ((*pre_posts)[i].pre != -1) {
            preconditions.push_back(Prevail((*pre_posts)[i].var,
                        (*pre_posts)[i].pre));
        }
    }
}

int PartialOrderLifter::getIndexOfPlanStep(const ScheduledOperator& op,
        double timestamp)
{
    for (int i = 0; i < plan.size(); ++i) {
        if (plan[i].op->get_name().compare(op.get_name()) == 0
                && double_equals(plan[i].start_time, timestamp)) {
            return i;
        }
    }
    assert(false);
    return -1;
}

void PartialOrderLifter::buildInstantPlan()
{
    const TimeStampedState* stateBeforeHappening;
    const TimeStampedState* stateAfterHappening;
    map<double, vector<const ScheduledOperator*> , doubleEquComp>
        actionsEndingAtGivenTime;
    map<double, list<InstantPlanStep*> , doubleEquComp>
        instantActionsStartingAtGivenTime;
    list<InstantPlanStep*>::iterator it;
    //    instant_plan.push_back(InstantPlanStep(dummy_start_action, 0.0, NULL));
    //    instant_plan.push_back(InstantPlanStep(dummy_end_action, trace.back()->timestamp, NULL));
    //    assert(instant_plan.size() == 2);
    int startPoints = 0;
    int endPoints = 0;
    for (int i = 0; i < trace.size() - 1; ++i) {
        stateBeforeHappening = trace[i];
        // effects of dummy start instant is initial state
        vector<PrePost> effects;
        //        findTriggeringEffectsForInitialState(stateBeforeHappening,effects);
        //        instant_plan[0].effects = effects;
        stateAfterHappening = trace[i + 1];
        findTriggeringEffects(stateBeforeHappening, stateAfterHappening,
                effects);
        double currentTimeStamp = stateAfterHappening->timestamp;
        if (stateBeforeHappening->operators.size()
                < stateAfterHappening->operators.size()) {
            // A new action starts at this happening!
            startPoints++;
            assert(stateBeforeHappening->operators.size() + 1 == stateAfterHappening->operators.size());
            if (!(double_equals(stateBeforeHappening->timestamp,
                            stateAfterHappening->timestamp))) {
                cout << stateBeforeHappening->timestamp << ", "
                    << stateAfterHappening->timestamp << endl;
                assert(false);
            }
            const ScheduledOperator& new_op =
                stateAfterHappening->operators.back();
            double startTime = currentTimeStamp;
            double time_increment = new_op.time_increment;
            double endTime = startTime + time_increment;
            vector<Prevail> preconditions;
            findPreconditions(new_op, preconditions, start_action);
            set<int> effect_cond_vars;
            findAllEffectCondVars(new_op, effect_cond_vars, start_action);
            actionsEndingAtGivenTime[endTime].push_back(&new_op);

            double duration = new_op.get_duration(stateBeforeHappening);
            instant_plan.push_back(InstantPlanStep(start_action, startTime,
                        duration, &new_op));
            instant_plan.back().effects = effects;
            instant_plan.back().effect_cond_vars = effect_cond_vars;
            instant_plan.back().preconditions = preconditions;
            instant_plan.back().overall_conds = new_op.get_prevail_overall();
            instant_plan.back().correspondingPlanStep = getIndexOfPlanStep(
                    new_op, startTime);
            //            assert(instant_plan.size() >= 3);
            instantActionsStartingAtGivenTime[startTime].push_back(
                    &(instant_plan.back()));
            //            // Insert ordering constraint between dummy_start and current start
            //            assert(instant_plan[0].type == dummy_start_action);
            //            partial_order.push_back(make_pair(&instant_plan[0], &instant_plan.back()));
        } else if (stateBeforeHappening->operators.size() >= stateAfterHappening->operators.size()) {
            // At least on action ends at this happening!
            assert(i> 0);
            assert(stateBeforeHappening->operators.size() > stateAfterHappening->operators.size());
            assert(stateBeforeHappening->timestamp < stateAfterHappening->timestamp + EPSILON);
            InstantPlanStep *lastEndingAction = NULL;
            double endTime = currentTimeStamp;
            //            endTime = (int)(endTime*1000000)/1000000.0;
            for (int j = 0; j < actionsEndingAtGivenTime[endTime].size(); ++j) {
                endPoints++;
                instant_plan.push_back(InstantPlanStep(end_action, endTime, -1,
                            actionsEndingAtGivenTime[endTime][j]));
                vector<Prevail> preconditions;
                findPreconditions((*actionsEndingAtGivenTime[endTime][j]),
                        preconditions, end_action);
                set<int> effect_cond_vars;
                findAllEffectCondVars((*actionsEndingAtGivenTime[endTime][j]),
                        effect_cond_vars, end_action);
                instant_plan.back().effects = effects;
                instant_plan.back().effect_cond_vars = effect_cond_vars;
                instant_plan.back().preconditions = preconditions;
                instant_plan.back().overall_conds
                    = (*actionsEndingAtGivenTime[endTime][j]).get_prevail_overall();
                double time_increment =
                    actionsEndingAtGivenTime[endTime][j]->time_increment;
                double startingTime = endTime - time_increment;
                bool bad = true;
                for (it
                        = instantActionsStartingAtGivenTime[startingTime].begin(); it
                        != instantActionsStartingAtGivenTime[startingTime].end(); ++it) {
                    if ((*it)->op == instant_plan.back().op) {
                        //                       instant_plan.back().correspondingAction = *(it);
                        (*it)->endAction = instant_plan.size() - 1;
                        bad = false;
                        // Insert ordering between corresponding start and end actions.
                        //                        partial_order.push_back(make_pair(*(it),&instant_plan.back()));
                        // Insert ordering constraint between dummy_end and current end
                        //                        assert(instant_plan[1].type == dummy_end_action);
                        //                        partial_order.push_back(make_pair(&(instant_plan.back()),&(instant_plan[1])));
                    }
                }
                assert(!bad);
                if (lastEndingAction != NULL) {
                    // If two actions finish at the same time point, let the one lasting longer finish first
                    // by inserting an according ordering constraint

                    //                    partial_order.push_back(make_pair(lastEndingAction,
                    //                            &instant_plan.back()));
                    lastEndingAction->actionFinishingImmeadatlyAfterThis
                        = instant_plan.size() - 1;
                }
                lastEndingAction = &instant_plan.back();
            }
            actionsEndingAtGivenTime[endTime].clear();
        } else {
            continue;
            //            assert(false);
        }
        cout << "startPoints: " << startPoints << ", endPoints: " << endPoints << endl;
    }
    assert(startPoints == endPoints);

    //// note: we need to use stable_sort instead of sort since reversing the order of actions
    //// occurring at the same time might break causal constraints that are still preserved after
    //// plan extraction from the closed list
    //stable_sort(instant_plan.begin(), instant_plan.end());
}
