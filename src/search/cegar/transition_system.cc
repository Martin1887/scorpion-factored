#include "transition_system.h"

#include "abstract_state.h"
#include "transition.h"
#include "utils.h"

#include "../task_proxy.h"

#include "../task_utils/task_properties.h"
#include "../utils/logging.h"

#include <algorithm>
#include <map>

using namespace std;

namespace cegar {
static vector<vector<FactPair>> get_preconditions_by_operator(
    const OperatorsProxy &ops) {
    vector<vector<FactPair>> preconditions_by_operator;
    preconditions_by_operator.reserve(ops.size());
    for (OperatorProxy op : ops) {
        vector<FactPair> preconditions = task_properties::get_fact_pairs(op.get_preconditions());
        sort(preconditions.begin(), preconditions.end());
        preconditions_by_operator.push_back(move(preconditions));
    }
    return preconditions_by_operator;
}

static vector<FactPair> get_postconditions(
    const OperatorProxy &op) {
    // Use map to obtain sorted postconditions.
    map<int, int> var_to_post;
    for (FactProxy fact : op.get_preconditions()) {
        var_to_post[fact.get_variable().get_id()] = fact.get_value();
    }
    for (EffectProxy effect : op.get_effects()) {
        FactPair fact = effect.get_fact().get_pair();
        var_to_post[fact.var] = fact.value;
    }
    vector<FactPair> postconditions;
    postconditions.reserve(var_to_post.size());
    for (const pair<const int, int> &fact : var_to_post) {
        postconditions.emplace_back(fact.first, fact.second);
    }
    return postconditions;
}

static vector<vector<FactPair>> get_postconditions_by_operator(
    const OperatorsProxy &ops) {
    vector<vector<FactPair>> postconditions_by_operator;
    postconditions_by_operator.reserve(ops.size());
    for (OperatorProxy op : ops) {
        postconditions_by_operator.push_back(get_postconditions(op));
    }
    return postconditions_by_operator;
}

static vector<vector<bool>> get_vars_affected_by_operator(
    int num_variables, const OperatorsProxy &ops) {
    vector<vector<bool>> vars_affected_by_operator(
        ops.size(), vector<bool>(num_variables, false));
    for (OperatorProxy op : ops) {
        int op_id = op.get_id();
        for (const FactProxy &precondition : op.get_preconditions()) {
            int var_id = precondition.get_variable().get_id();
            vars_affected_by_operator[op_id][var_id] = true;
        }
        for (const EffectProxy &effect : op.get_effects()) {
            int var_id = effect.get_fact().get_variable().get_id();
            vars_affected_by_operator[op_id][var_id] = true;
        }
    }
    return vars_affected_by_operator;
}

static vector<vector<FactoredEffectPair>> get_factored_effect_pairs_by_operator(
    const OperatorsProxy &ops) {
    vector<vector<FactoredEffectPair>> factored_effect_pairs_by_operator;
    factored_effect_pairs_by_operator.reserve(ops.size());
    for (OperatorProxy op : ops) {
        factored_effect_pairs_by_operator.push_back(
            task_properties::get_factored_effect_pairs(op.get_effects()));
    }
    return factored_effect_pairs_by_operator;
}

vector<vector<int>> TransitionSystem::precompute_resulting_facts_for_operator(
    const VariablesProxy &vars, const OperatorProxy &op) const {
    const PreconditionsProxy &preconditions = op.get_preconditions();
    unordered_map<int, int> preconditioned;
    for (const FactProxy &precondition : preconditions) {
        FactPair pre = precondition.get_pair();
        preconditioned[pre.var] = pre.value;
    }

    vector<vector<int>> resulting_facts(vars.size());
    for (const VariableProxy &var : vars) {
        int id = var.get_id();
        resulting_facts[id] = vector<int>(var.get_domain_size(), -1);
        if (preconditioned.count(id)) {
            int val = preconditioned[id];
            resulting_facts[id][val] = val;
        } else {
            for (int i = 0; i < var.get_domain_size(); ++i) {
                // Initialize to "value unchanged".
                resulting_facts[id][i] = i;
            }
        }
    }
    for (FactoredEffectPair fep : factored_effect_pairs_by_operator.at(op.get_id())) {
        if (fep.value_before == -1) {
            for (int val = 0; val < vars[fep.var].get_domain_size(); ++val) {
                resulting_facts[fep.var][val] = fep.value_after;
            }
        } else {
            resulting_facts[fep.var][fep.value_before] = fep.value_after;
        }
    }
    return resulting_facts;
}

vector<vector<vector<int>>> TransitionSystem::precompute_resulting_facts(
    const VariablesProxy &variables, const OperatorsProxy &ops) {
    vector<vector<vector<int>>> resulting_facts;
    resulting_facts.reserve(ops.size());
    for (const OperatorProxy &op : ops) {
        resulting_facts.push_back(
            precompute_resulting_facts_for_operator(variables, op));
    }
    return resulting_facts;
}

static int lookup_value(const vector<FactPair> &facts, int var) {
    assert(is_sorted(facts.begin(), facts.end()));
    for (const FactPair &fact : facts) {
        if (fact.var == var) {
            return fact.value;
        } else if (fact.var > var) {
            return UNDEFINED;
        }
    }
    return UNDEFINED;
}

static void remove_transitions_with_given_target(
    Transitions &transitions, int state_id) {
    auto new_end = remove_if(
        transitions.begin(), transitions.end(),
        [state_id](const Transition &t) {return t.target_id == state_id;});
    assert(new_end != transitions.end());
    transitions.erase(new_end, transitions.end());
}


TransitionSystem::TransitionSystem(const VariablesProxy &variables,
                                   const OperatorsProxy &ops)
    : preconditions_by_operator(get_preconditions_by_operator(ops)),
      postconditions_by_operator(get_postconditions_by_operator(ops)),
      vars_affected_by_operator(
          get_vars_affected_by_operator(variables.size(), ops)),
      factored_effect_pairs_by_operator(
          get_factored_effect_pairs_by_operator(ops)),
      resulting_facts_by_operator(
          precompute_resulting_facts(variables, ops)),
      num_non_loops(0),
      num_loops(0) {
    add_loops_in_trivial_abstraction();
}

int TransitionSystem::get_precondition_value(int op_id, int var) const {
    return lookup_value(preconditions_by_operator[op_id], var);
}

int TransitionSystem::get_postcondition_value(int op_id, int var) const {
    return lookup_value(postconditions_by_operator[op_id], var);
}

void TransitionSystem::enlarge_vectors_by_one() {
    int new_num_states = get_num_states() + 1;
    outgoing.resize(new_num_states);
    incoming.resize(new_num_states);
    loops.resize(new_num_states);
}

void TransitionSystem::add_loops_in_trivial_abstraction() {
    assert(get_num_states() == 0);
    enlarge_vectors_by_one();
    int init_id = 0;
    for (int i = 0; i < get_num_operators(); ++i) {
        add_loop(init_id, i);
    }
}

void TransitionSystem::add_transition(int src_id, int op_id, int target_id) {
    assert(src_id != target_id);
    outgoing[src_id].emplace_back(op_id, target_id);
    incoming[target_id].emplace_back(op_id, src_id);
    ++num_non_loops;
}

void TransitionSystem::add_loop(int state_id, int op_id) {
    assert(utils::in_bounds(state_id, loops));
    loops[state_id].push_back(op_id);
    ++num_loops;
}

void TransitionSystem::rewire_incoming_transitions(
    const Transitions &old_incoming, const AbstractStates &states, int v_id,
    const AbstractState &v1, const AbstractState &v2, int var) {
    /* State v has been split into v1 and v2. Now for all transitions
       u->v we need to add transitions u->v1, u->v2, or both. */
    int v1_id = v1.get_id();
    int v2_id = v2.get_id();

    unordered_set<int> updated_states;
    for (const Transition &transition : old_incoming) {
        int u_id = transition.target_id;
        bool is_new_state = updated_states.insert(u_id).second;
        if (is_new_state) {
            remove_transitions_with_given_target(outgoing[u_id], v_id);
        }
    }
    num_non_loops -= old_incoming.size();

    for (const Transition &transition : old_incoming) {
        int op_id = transition.op_id;
        int u_id = transition.target_id;
        const AbstractState &u = *states[u_id];

        if (vars_affected_by_operator[op_id][var]) {
            if (reachable(u, op_id, var, v1)) {
                add_transition(u_id, op_id, v1_id);
            }
            if (reachable(u, op_id, var, v2)) {
                add_transition(u_id, op_id, v2_id);
            }
        } else {
            // op has no precondition and no effect on var.
            bool u_and_v1_intersect = u.domain_subsets_intersect(v1, var);
            if (u_and_v1_intersect) {
                add_transition(u_id, op_id, v1_id);
            }
            /* If u and v1 don't intersect, we must add the other transition
               and can avoid an intersection test. */
            if (!u_and_v1_intersect || u.domain_subsets_intersect(v2, var)) {
                add_transition(u_id, op_id, v2_id);
            }
        }
    }
}

void TransitionSystem::rewire_outgoing_transitions(
    const Transitions &old_outgoing, const AbstractStates &states, int v_id,
    const AbstractState &v1, const AbstractState &v2, int var) {
    /* State v has been split into v1 and v2. Now for all transitions
       v->w we need to add transitions v1->w, v2->w, or both. */
    int v1_id = v1.get_id();
    int v2_id = v2.get_id();

    unordered_set<int> updated_states;
    for (const Transition &transition : old_outgoing) {
        int w_id = transition.target_id;
        bool is_new_state = updated_states.insert(w_id).second;
        if (is_new_state) {
            remove_transitions_with_given_target(incoming[w_id], v_id);
        }
    }
    num_non_loops -= old_outgoing.size();

    for (const Transition &transition : old_outgoing) {
        int op_id = transition.op_id;
        int w_id = transition.target_id;
        const AbstractState &w = *states[w_id];
        int pre = get_precondition_value(op_id, var);
        if (pre == UNDEFINED) {
            if (reachable(v1, op_id, var, w)) {
                add_transition(v1_id, op_id, w_id);
            }
            if (reachable(v2, op_id, var, w)) {
                add_transition(v2_id, op_id, w_id);
            }
        } else if (v1.contains(var, pre)) {
            // op can only start in v1.
            add_transition(v1_id, op_id, w_id);
        } else {
            // op can only start in v2.
            assert(v2.contains(var, pre));
            add_transition(v2_id, op_id, w_id);
        }
    }
}

void TransitionSystem::rewire_loops(
    const Loops &old_loops, const AbstractState &v1, const AbstractState &v2, int var) {
    /* State v has been split into v1 and v2. Now for all self-loops
       v->v we need to add one or two of the transitions v1->v1, v1->v2,
       v2->v1 and v2->v2. */
    int v1_id = v1.get_id();
    int v2_id = v2.get_id();
    for (int op_id : old_loops) {
        int pre = get_precondition_value(op_id, var);
        if (pre == UNDEFINED) {
            if (vars_affected_by_operator[op_id][var]) {
                if (reachable(v1, op_id, var, v1)) {
                    add_loop(v1_id, op_id);
                }
                if (reachable(v2, op_id, var, v2)) {
                    add_loop(v2_id, op_id);
                }
                if (reachable(v1, op_id, var, v2)) {
                    add_transition(v1_id, op_id, v2_id);
                }
                if (reachable(v2, op_id, var, v1)) {
                    add_transition(v2_id, op_id, v1_id);
                }
            } else {
                add_loop(v1_id, op_id);
                add_loop(v2_id, op_id);
            }
        } else if (v1.contains(var, pre)) {
            // op must start in v1.
            if (reachable(v1, op_id, var, v1)) {
                add_loop(v1_id, op_id);
            }
            if (reachable(v1, op_id, var, v2)) {
                add_transition(v1_id, op_id, v2_id);
            }
        } else {
            // op must start in v2.
            assert(v2.contains(var, pre));
            if (reachable(v2, op_id, var, v1)) {
                add_transition(v2_id, op_id, v1_id);
            }
            if (reachable(v2, op_id, var, v2)) {
                add_loop(v2_id, op_id);
            }
        }
    }
    num_loops -= old_loops.size();
}

bool TransitionSystem::reachable(
    const AbstractState &a, int op_id, int var, const AbstractState &b) const {
    return any_of(
        a.get_cartesian_set().begin_values(var), a.get_cartesian_set().end_values(var), [&](int value) {
            int resulting_fact = resulting_facts_by_operator[op_id][var][value];
            if (resulting_fact < 0) {
                return false;
            } else {
                b.contains(var, resulting_fact);
                return b.contains(var, resulting_fact);
            }
        });
}

void TransitionSystem::rewire(
    const AbstractStates &states, int v_id,
    const AbstractState &v1, const AbstractState &v2, int var) {
    // Retrieve old transitions and make space for new transitions.
    Transitions old_incoming = move(incoming[v_id]);
    Transitions old_outgoing = move(outgoing[v_id]);
    Loops old_loops = move(loops[v_id]);
    enlarge_vectors_by_one();
    int v1_id = v1.get_id();
    int v2_id = v2.get_id();
    utils::unused_variable(v1_id);
    utils::unused_variable(v2_id);
    assert(incoming[v1_id].empty() && outgoing[v1_id].empty() && loops[v1_id].empty());
    assert(incoming[v2_id].empty() && outgoing[v2_id].empty() && loops[v2_id].empty());

    // Remove old transitions and add new transitions.
    rewire_incoming_transitions(old_incoming, states, v_id, v1, v2, var);
    rewire_outgoing_transitions(old_outgoing, states, v_id, v1, v2, var);
    rewire_loops(old_loops, v1, v2, var);
}

const vector<Transitions> &TransitionSystem::get_incoming_transitions() const {
    return incoming;
}

const vector<Transitions> &TransitionSystem::get_outgoing_transitions() const {
    return outgoing;
}

const vector<Loops> &TransitionSystem::get_loops() const {
    return loops;
}

const vector<FactPair> &TransitionSystem::get_preconditions(int op_id) const {
    assert(utils::in_bounds(op_id, preconditions_by_operator));
    return preconditions_by_operator[op_id];
}

int TransitionSystem::get_num_states() const {
    assert(incoming.size() == outgoing.size());
    assert(loops.size() == outgoing.size());
    return outgoing.size();
}

int TransitionSystem::get_num_operators() const {
    return preconditions_by_operator.size();
}

int TransitionSystem::get_num_non_loops() const {
    return num_non_loops;
}

int TransitionSystem::get_num_loops() const {
    return num_loops;
}

const vector<FactoredEffectPair> &TransitionSystem::get_factored_effect_pairs(
    int op_id) const {
    return factored_effect_pairs_by_operator[op_id];
}

void TransitionSystem::print_statistics(utils::LogProxy &log) const {
    if (log.is_at_least_normal()) {
        int total_incoming_transitions = 0;
        int total_outgoing_transitions = 0;
        int total_loops = 0;
        for (int state_id = 0; state_id < get_num_states(); ++state_id) {
            total_incoming_transitions += incoming[state_id].size();
            total_outgoing_transitions += outgoing[state_id].size();
            total_loops += loops[state_id].size();
        }
        assert(total_outgoing_transitions == total_incoming_transitions);
        assert(get_num_loops() == total_loops);
        assert(get_num_non_loops() == total_outgoing_transitions);
        log << "Looping transitions: " << total_loops << endl;
        log << "Non-looping transitions: " << total_outgoing_transitions << endl;
    }
}

void TransitionSystem::dump() const {
    for (int i = 0; i < get_num_states(); ++i) {
        cout << "State " << i << endl;
        cout << "  in: " << incoming[i] << endl;
        cout << "  out: " << outgoing[i] << endl;
        cout << "  loops: " << loops[i] << endl;
    }
}
}
