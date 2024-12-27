#ifndef TASK_UTILS_TASK_PROPERTIES_H
#define TASK_UTILS_TASK_PROPERTIES_H

#include "../per_task_information.h"
#include "../task_proxy.h"

#include "../algorithms/int_packer.h"

namespace task_properties {
inline bool is_applicable(OperatorProxy op, const State &state) {
    for (FactProxy precondition : op.get_preconditions()) {
        if (state[precondition.get_variable()] != precondition)
            return false;
    }
    return true;
}

inline bool is_goal_state(TaskProxy task, const State &state) {
    for (FactProxy goal : task.get_goals()) {
        if (state[goal.get_variable()] != goal)
            return false;
    }
    return true;
}

/*
  Return true iff all operators have cost 1.

  Runtime: O(n), where n is the number of operators.
*/
extern bool is_unit_cost(TaskProxy task);

// Runtime: O(1)
extern bool has_axioms(TaskProxy task);

/*
  Report an error and exit with ExitCode::UNSUPPORTED if the task has axioms.
  Runtime: O(1)
*/
extern void verify_no_axioms(TaskProxy task);

// Runtime: O(n), where n is the number of operators.
extern bool has_conditional_effects(TaskProxy task);

/*
  Report an error and exit with ExitCode::UNSUPPORTED if the task has
  conditional effects.
  Runtime: O(n), where n is the number of operators.
*/
extern void verify_no_conditional_effects(TaskProxy task);

extern void verify_factored_effect_task(TaskProxy task);

extern std::vector<int> get_operator_costs(const TaskProxy &task_proxy);
extern double get_average_operator_cost(TaskProxy task_proxy);
extern int get_min_operator_cost(TaskProxy task_proxy);

/*
  Return the number of facts of the task.
  Runtime: O(n), where n is the number of state variables.
*/
extern int get_num_facts(const TaskProxy &task_proxy);

/*
  Return the total number of effects of the task, including the
  effects of axioms.
  Runtime: O(n), where n is the number of operators and axioms.
*/
extern int get_num_total_effects(const TaskProxy &task_proxy);

template<class FactProxyCollection>
std::vector<FactPair> get_fact_pairs(const FactProxyCollection &facts) {
    std::vector<FactPair> fact_pairs;
    fact_pairs.reserve(facts.size());
    for (FactProxy fact : facts) {
        fact_pairs.push_back(fact.get_pair());
    }
    return fact_pairs;
}

template<class EffectProxyCollection>
std::vector<FactoredEffectPair> get_factored_effect_pairs(
    const EffectProxyCollection &effects) {
    std::vector<FactoredEffectPair> factored_effect_pairs;
    factored_effect_pairs.reserve(effects.size());
    std::unordered_set<int> unconditioned_effect_vars;
    for (EffectProxy effect : effects) {
        const EffectConditionsProxy &conditions = effect.get_conditions();
        /*
          We assume all tasks are factored, i.e., they can have at most one
          condition on an effect and that condition must be on the same
          variable as the effect.
        */
        int effect_var_id = effect.get_fact().get_variable().get_id();
        assert(unconditioned_effect_vars.count(effect_var_id) == 0);
        if (conditions.empty()) {
            unconditioned_effect_vars.insert(effect_var_id);
            // We use the special value -1 for unconditional effects.
            factored_effect_pairs.emplace_back(
                effect_var_id, -1, effect.get_fact().get_value());
        } else {
            assert(conditions.size() == 1);
            assert(conditions[0].get_variable().get_id() == effect_var_id);
            factored_effect_pairs.emplace_back(
                effect_var_id, conditions[0].get_value(),
                effect.get_fact().get_value());
        }
    }
    return factored_effect_pairs;
}

extern void print_variable_statistics(const TaskProxy &task_proxy);
extern void dump_pddl(const State &state);
extern void dump_fdr(const State &state);
extern void dump_goals(const GoalsProxy &goals);
extern void dump_task(const TaskProxy &task_proxy);

extern PerTaskInformation<int_packer::IntPacker> g_state_packers;
}

#endif
