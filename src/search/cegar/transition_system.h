#ifndef CEGAR_TRANSITION_SYSTEM_H
#define CEGAR_TRANSITION_SYSTEM_H

#include "types.h"

#include <vector>

struct FactPair;
class OperatorProxy;
class OperatorsProxy;
class VariablesProxy;
struct FactoredEffectPair;

namespace utils {
class LogProxy;
}

namespace cegar {
/*
  Rewire transitions after each split.
*/
class TransitionSystem {
    const std::vector<std::vector<FactPair>> preconditions_by_operator;
    const std::vector<std::vector<FactPair>> postconditions_by_operator;

    const std::vector<std::vector<bool>> vars_affected_by_operator;
    const std::vector<std::vector<FactoredEffectPair>>
    factored_effect_pairs_by_operator;
    const std::vector<std::vector<std::vector<int>>>
    resulting_facts_by_operator;

    // Transitions from and to other abstract states.
    std::vector<Transitions> incoming;
    std::vector<Transitions> outgoing;

    // Store self-loops (operator indices) separately to save space.
    std::vector<Loops> loops;

    int num_non_loops;
    int num_loops;

    void enlarge_vectors_by_one();

    // Add self-loops to single abstract state in trivial abstraction.
    void add_loops_in_trivial_abstraction();

    int get_precondition_value(int op_id, int var) const;
    int get_postcondition_value(int op_id, int var) const;

    void add_transition(int src_id, int op_id, int target_id);
    void add_loop(int state_id, int op_id);

    void rewire_incoming_transitions(
        const Transitions &old_incoming, const AbstractStates &states,
        int v_id, const AbstractState &v1, const AbstractState &v2, int var);
    void rewire_outgoing_transitions(
        const Transitions &old_outgoing, const AbstractStates &states,
        int v_id, const AbstractState &v1, const AbstractState &v2, int var);
    void rewire_loops(
        const Loops &old_loops,
        const AbstractState &v1, const AbstractState &v2, int var);

    std::vector<std::vector<int>> precompute_resulting_facts_for_operator(
        const VariablesProxy &vars, const OperatorProxy &op_id) const;
    std::vector<std::vector<std::vector<int>>> precompute_resulting_facts(
        const VariablesProxy &variables, const OperatorsProxy &ops);

    bool reachable(
        const AbstractState &a, int op_id, int var, const AbstractState &b) const;

public:
    TransitionSystem(const VariablesProxy &variables,
                     const OperatorsProxy &ops);

    // Update transition system after v has been split for var into v1 and v2.
    void rewire(
        const AbstractStates &states, int v_id,
        const AbstractState &v1, const AbstractState &v2, int var);

    const std::vector<Transitions> &get_incoming_transitions() const;
    const std::vector<Transitions> &get_outgoing_transitions() const;
    const std::vector<Loops> &get_loops() const;

    const std::vector<FactPair> &get_preconditions(int op_id) const;

    int get_num_states() const;
    int get_num_operators() const;
    int get_num_non_loops() const;
    int get_num_loops() const;

    const std::vector<FactoredEffectPair> &get_factored_effect_pairs(
        int op_id) const;

    void print_statistics(utils::LogProxy &log) const;
    void dump() const;
};
}

#endif
