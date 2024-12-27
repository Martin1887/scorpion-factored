#include <set>
#include "domain_abstraction_factory.h"

#include "domain_abstraction.h"
#include "match_tree.h"

#include "../algorithms/priority_queues.h"

#include "../task_utils/task_properties.h"
#include "../tasks/root_task.h"

#include "../utils/math.h"
#include "../utils/logging.h"
#include "../utils/rng.h"


using namespace std;

namespace domain_abstractions {
AbstractOperator::AbstractOperator(const vector<FactPair> &prev_pairs,
                                   const vector<FactPair> &pre_pairs,
                                   const vector<FactPair> &eff_pairs,
                                   int cost,
                                   const vector<int> &hash_multipliers,
                                   int concrete_op_id)
    : concrete_op_id(concrete_op_id),
      cost(cost),
      regression_preconditions(prev_pairs) {
    regression_preconditions.insert(regression_preconditions.end(),
                                    eff_pairs.begin(), eff_pairs.end());
    // Sort preconditions for MatchTree construction.
    sort(regression_preconditions.begin(), regression_preconditions.end());
    hash_effect = 0;
    assert(pre_pairs.size() == eff_pairs.size());
    for (size_t i = 0; i < pre_pairs.size(); ++i) {
        int var = pre_pairs[i].var;
        assert(var == eff_pairs[i].var);
        int old_val = eff_pairs[i].value;
        int new_val = pre_pairs[i].value;
        assert(new_val != -1);
        int effect = (new_val - old_val) * hash_multipliers[var];
        hash_effect += effect;
    }
}

void AbstractOperator::dump(const VariablesProxy &variables,
                            utils::LogProxy &log) const {
    if (log.is_at_least_debug()) {
        log << "AbstractOperator:" << endl;
        log << "Regression preconditions:" << endl;
        for (size_t i = 0; i < regression_preconditions.size(); ++i) {
            int var_id = regression_preconditions[i].var;
            int val = regression_preconditions[i].value;
            log << "Variable: " << var_id << " (True name: "
                << variables[var_id].get_name()
                << ", Index: " << i << ") Value: " << val << endl;
        }
        log << "Hash effect:" << hash_effect << endl;
    }
}

DomainAbstractionFactory::DomainAbstractionFactory (
    const TaskProxy &task_proxy,
    const DomainMapping &domain_mapping,
    const vector<int> &domain_sizes,
    bool compute_plan,
    const shared_ptr<utils::RandomNumberGenerator> &rng,
    bool compute_wildcard_plan)
    : domain_mapping(domain_mapping) {
    task_properties::verify_no_axioms(task_proxy);
    task_properties::verify_factored_effect_task(task_proxy);

    int num_variables = task_proxy.get_variables().size();
    hash_multipliers.reserve(num_variables);
    num_states = 1;
    for (int var_id = 0; var_id < num_variables; ++var_id) {
        hash_multipliers.push_back(num_states);
        if (utils::is_product_within_limit(num_states, domain_sizes[var_id],
                                           numeric_limits<int>::max())) {
            num_states *= domain_sizes[var_id];
        } else {
            cerr << "Given domain mapping is too large! (Overflow occurred). "
                 << "Domain sizes: " << domain_sizes << endl;
            utils::exit_with(utils::ExitCode::SEARCH_CRITICAL_ERROR);
        }
    }

    vector<AbstractOperator> operators =
        compute_abstract_operators(task_proxy, domain_sizes);
    MatchTree match_tree = build_match_tree(domain_sizes, operators);
    vector<FactPair> abstract_goals = compute_abstract_goals(task_proxy);
    compute_distances(operators, match_tree, abstract_goals,
                      domain_sizes, compute_plan);
    if (compute_plan) {
        compute_abstract_plan(
            task_proxy, operators, match_tree, abstract_goals,
            domain_sizes, rng, compute_wildcard_plan);
    }
}

vector<AbstractOperator> DomainAbstractionFactory::compute_abstract_operators(
    const TaskProxy &task_proxy, const vector<int> &domain_sizes) {
    vector<AbstractOperator> operators;
    for (OperatorProxy op : task_proxy.get_operators()) {
        build_abstract_operators(op, task_proxy.get_variables().size(),
                                 domain_sizes, operators);
    }
    return operators;
}

MatchTree DomainAbstractionFactory::build_match_tree(
    const vector<int> &domain_sizes,
    const vector<AbstractOperator> &operators) {
    MatchTree match_tree(domain_sizes, hash_multipliers);
    for (size_t op_id = 0; op_id < operators.size(); ++op_id) {
        const AbstractOperator &op = operators[op_id];
        match_tree.insert(op_id, op.get_regression_preconditions());
    }
    return match_tree;
}

vector<FactPair> DomainAbstractionFactory::compute_abstract_goals(
    const TaskProxy &task_proxy) {
    vector<FactPair> abstract_goals;
    for (FactProxy goal : task_proxy.get_goals()) {
        int var_id = goal.get_variable().get_id();
        if (!variable_is_trivial(var_id)) {
            int val = goal.get_value();
            abstract_goals.emplace_back(var_id, domain_mapping[var_id][val]);
        }
    }
    return abstract_goals;
}

void DomainAbstractionFactory::compute_distances(
    const vector<AbstractOperator> &operators, const MatchTree &match_tree,
    const vector<FactPair> &abstract_goals, const vector<int> &domain_sizes,
    bool compute_plan) {
    distances.reserve(num_states);
    // first implicit entry: priority, second entry: index for an abstract state
    priority_queues::AdaptiveQueue<int> pq;

    // initialize queue
    for (int state_index = 0; state_index < num_states; ++state_index) {
        if (is_goal_state(state_index, abstract_goals, domain_sizes)) {
            pq.push(0, state_index);
            distances.push_back(0);
        } else {
            distances.push_back(numeric_limits<int>::max());
        }
    }

    if (compute_plan) {
        /*
          If computing a plan during Dijkstra, we store, for each state,
          an operator leading from that state to another state on a
          strongly optimal plan of the PDB. We store the first operator
          encountered during Dijkstra and only update it if the goal distance
          of the state was updated. Note that in the presence of zero-cost
          operators, this does not guarantee that we compute a strongly
          optimal plan because we do not minimize the number of used zero-cost
          operators.
         */
        generating_op_ids.resize(num_states);
    }

    // Dijkstra loop
    while (!pq.empty()) {
        pair<int, int> node = pq.pop();
        int distance = node.first;
        int state_index = node.second;
        if (distance > distances[state_index]) {
            continue;
        }

        // regress abstract_state
        vector<int> applicable_operator_ids;
        match_tree.get_applicable_operator_ids(state_index, applicable_operator_ids);
        for (int op_id : applicable_operator_ids) {
            const AbstractOperator &op = operators[op_id];
            int predecessor = state_index + op.get_hash_effect();
            int alternative_cost = distances[state_index] + op.get_cost();
            if (alternative_cost < distances[predecessor]) {
                distances[predecessor] = alternative_cost;
                pq.push(alternative_cost, predecessor);
                if (compute_plan) {
                    generating_op_ids[predecessor] = op_id;
                }
            }
        }
    }
}

void DomainAbstractionFactory::compute_abstract_plan(
    const TaskProxy &task_proxy,
    const vector<AbstractOperator> &operators,
    const MatchTree &match_tree,
    const vector<FactPair> &abstract_goals,
    const vector<int> &domain_sizes,
    const shared_ptr<utils::RandomNumberGenerator> &rng,
    bool compute_wildcard_plan) {
    /*
      Using the generating operators computed during Dijkstra, we start
      from the initial state and follow the generating operator to the
      next state. Then we compute all operators of the same cost inducing
      the same abstract transition and randomly pick one of them to
      set for the next state. We iterate until reaching a goal state.
      Note that this kind of plan extraction does not uniformly at random
      consider all successor of a state but rather uses the arbitrarily
      chosen generating operator to settle on one successor state, which
      is biased by the number of operators leading to the same successor
      from the given state.
    */
    State initial_state = task_proxy.get_initial_state();
    initial_state.unpack();
    int current_state = hash_index(initial_state.get_unpacked_values());
    state_trace.push_back(current_state);
    if (distances[current_state] != numeric_limits<int>::max()) {
        while (!is_goal_state(current_state, abstract_goals, domain_sizes)) {
            int op_id = generating_op_ids[current_state];
            assert(op_id != -1);
            const AbstractOperator &op = operators[op_id];
            int successor_state = current_state - op.get_hash_effect();

            // Compute equivalent ops
            vector<OperatorID> cheapest_operators;
            vector<int> applicable_operator_ids;
            match_tree.get_applicable_operator_ids(successor_state, applicable_operator_ids);
            for (int applicable_op_id : applicable_operator_ids) {
                const AbstractOperator &applicable_op = operators[applicable_op_id];
                int predecessor = successor_state + applicable_op.get_hash_effect();
                if (predecessor == current_state && op.get_cost() == applicable_op.get_cost()) {
                    cheapest_operators.emplace_back(applicable_op.get_concrete_op_id());
                }
            }
            if (compute_wildcard_plan) {
                rng->shuffle(cheapest_operators);
                wildcard_plan.push_back(move(cheapest_operators));
            } else {
                OperatorID random_op_id = *rng->choose(cheapest_operators);
                wildcard_plan.emplace_back();
                wildcard_plan.back().push_back(random_op_id);
            }

            current_state = successor_state;
            state_trace.push_back(current_state);
        }
    }
    utils::release_vector_memory(generating_op_ids);
}

void DomainAbstractionFactory::multiply_out(
    int pos, int cost, vector<FactPair> &prev_pairs,
    vector<FactPair> &pre_pairs,
    vector<FactPair> &eff_pairs,
    const vector<pair<int, unordered_map<int, int>>> &original_effects_without_pre,
    const int concrete_op_id,
    const vector<int> &domain_sizes,
    vector<AbstractOperator> &operators) {
    if (pos == static_cast<int>(original_effects_without_pre.size())) {
        // All effects without precondition have been checked: insert op.
        if (!eff_pairs.empty()) {
            operators.emplace_back(prev_pairs, pre_pairs, eff_pairs, cost,
                                   hash_multipliers, concrete_op_id);
        }
    } else {
        // For each possible value for the current variable, build an
        // abstract operator.
        int var_id = original_effects_without_pre[pos].first;
        assert(domain_sizes[var_id] > 1);
        const unordered_map<int, int> &original_effects =
            original_effects_without_pre[pos].second;
        if (original_effects.count(-1)) {
            // Only one effect allowed if it is unconditional.
            assert(original_effects.size() == 1);
            int eff = domain_mapping[var_id][original_effects.at(-1)];
            for (int i = 0; i < domain_sizes[var_id]; ++i) {
                if (i != eff) {
                    pre_pairs.emplace_back(var_id, i);
                    eff_pairs.emplace_back(var_id, eff);
                } else {
                    prev_pairs.emplace_back(var_id, i);
                }
                multiply_out(pos + 1, cost, prev_pairs, pre_pairs,
                             eff_pairs, original_effects_without_pre,
                             concrete_op_id, domain_sizes, operators);
                if (i != eff) {
                    pre_pairs.pop_back();
                    eff_pairs.pop_back();
                } else {
                    prev_pairs.pop_back();
                }
            }
        } else {
            /* We have to loop over original domain size because some values
               mapped to x may have a conditional effect while others don't. */
            vector<unordered_set<int>> abstract_effects(domain_sizes[var_id]);
            for (int i = 0; i < static_cast<int>(domain_mapping[var_id].size()); ++i) {
                int abstract_val = domain_mapping[var_id][i];
                if (original_effects.count(i)) {
                    int abstract_effect = domain_mapping[var_id][original_effects.at(i)];
                    abstract_effects[abstract_val].insert(abstract_effect);
                } else {
                    abstract_effects[abstract_val].insert(abstract_val);
                }
            }
            for (int i = 0; i < domain_sizes[var_id]; ++i) {
                if (abstract_effects[i].empty()) {
                    prev_pairs.emplace_back(var_id, i);
                    multiply_out(pos + 1, cost, prev_pairs, pre_pairs,
                                 eff_pairs, original_effects_without_pre,
                                 concrete_op_id, domain_sizes, operators);
                    prev_pairs.pop_back();
                } else {
                    for (int eff : abstract_effects[i]) {
                        if (i != eff) {
                            pre_pairs.emplace_back(var_id, i);
                            eff_pairs.emplace_back(var_id, eff);
                        } else {
                            prev_pairs.emplace_back(var_id, i);
                        }
                        multiply_out(pos + 1, cost, prev_pairs, pre_pairs,
                                     eff_pairs, original_effects_without_pre,
                                     concrete_op_id, domain_sizes, operators);
                        if (i != eff) {
                            pre_pairs.pop_back();
                            eff_pairs.pop_back();
                        } else {
                            prev_pairs.pop_back();
                        }
                    }
                }
            }
        }
    }
}

void DomainAbstractionFactory::build_abstract_operators(
    const OperatorProxy &op,
    int num_variables,
    const vector<int> &domain_sizes,
    vector<AbstractOperator> &operators) {
    // All variable value pairs that are a prevail condition
    vector<FactPair> prev_pairs;
    // All variable value pairs that are a precondition (value != -1)
    vector<FactPair> pre_pairs;
    // All variable value pairs that are an effect
    vector<FactPair> eff_pairs;
    // All variable value pairs that are a precondition (value = -1)
    unordered_map<int, unordered_map<int, int>> original_effects_without_pre_map;

    vector<int> has_precondition_on_var(num_variables, -1);
    vector<int> has_effect_on_var(num_variables, -1);

    for (FactProxy pre : op.get_preconditions()) {
        int var_id = pre.get_variable().get_id();
        if (variable_is_trivial(var_id)) {
            has_precondition_on_var[var_id] = 0;
        } else {
            has_precondition_on_var[var_id] =
                domain_mapping[var_id][pre.get_value()];
        }
    }

    for (EffectProxy eff : op.get_effects()) {
        int var_id = eff.get_fact().get_variable().get_id();
        if (!variable_is_trivial(var_id)) {
            int val = domain_mapping[var_id][eff.get_fact().get_value()];
            int pre_val = has_precondition_on_var[var_id];
            if (pre_val < 0) {
                if (eff.get_conditions().empty()) {
                    original_effects_without_pre_map[var_id][-1] =
                        eff.get_fact().get_value();
                } else {
                    assert(eff.get_conditions().size() == 1);
                    original_effects_without_pre_map[var_id][eff.get_conditions()[0].get_value()] =
                        eff.get_fact().get_value();
                }
            } else if (pre_val != val) {
                has_effect_on_var[var_id] = val;
                assert(eff.get_conditions().empty());
                eff_pairs.emplace_back(var_id, val);
            }
        }
    }
    vector<pair<int, unordered_map<int, int>>> original_effects_without_pre;
    for (auto eff : original_effects_without_pre_map) {
        original_effects_without_pre.emplace_back(eff.first, eff.second);
    }

    for (FactProxy pre : op.get_preconditions()) {
        int var_id = pre.get_variable().get_id();
        if (!variable_is_trivial(var_id)) {
            int val = domain_mapping[var_id][pre.get_value()];
            if (has_effect_on_var[var_id] >= 0) {
                pre_pairs.emplace_back(var_id, val);
            } else {
                prev_pairs.emplace_back(var_id, val);
            }
        }
    }
    multiply_out(0, op.get_cost(), prev_pairs, pre_pairs, eff_pairs,
                 original_effects_without_pre, op.get_id(), domain_sizes,
                 operators);
}

bool DomainAbstractionFactory::is_goal_state(
    int state_index,
    const vector<FactPair> &abstract_goals,
    const vector<int> &domain_sizes) const {
    for (const FactPair &abstract_goal : abstract_goals) {
        int var_id = abstract_goal.var;
        int temp = state_index / hash_multipliers[var_id];
        int val = temp % domain_sizes[var_id];
        if (val != abstract_goal.value) {
            return false;
        }
    }
    return true;
}

int DomainAbstractionFactory::hash_index(const vector<int> &state) const {
    int index = 0;
    for (size_t i = 0; i < state.size(); ++i) {
        if (!variable_is_trivial(i)) {
            index += hash_multipliers[i] * domain_mapping[i][state[i]];
        }
    }
    return index;
}

bool DomainAbstractionFactory::variable_is_trivial(int var_id) const {
    return domain_mapping[var_id].empty();
}

DomainAbstraction DomainAbstractionFactory::generate() {
    return DomainAbstraction(move(domain_mapping), move(hash_multipliers),
                             move(distances), move(wildcard_plan),
                             move(state_trace));
}
}
