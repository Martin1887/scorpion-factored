#include "cegar.h"

#include "domain_abstraction.h"
#include "domain_abstraction_factory.h"

#include "../option_parser.h"
#include "../task_proxy.h"

#include "../utils/countdown_timer.h"
#include "../utils/logging.h"
#include "../utils/rng.h"
#include "../utils/rng_options.h"

using namespace std;

namespace domain_abstractions {
static const int memory_padding_in_mb = 75;

class CEGAR {
private:
    const int max_abstraction_size;
    const double max_time;
    const bool use_wildcard_plans;
    const FlawTreatment flaw_treatment;
    const InitSplitMethod init_split_method;
    utils::LogProxy &log;
    const shared_ptr<utils::RandomNumberGenerator> &rng;
    const std::unordered_set<int> init_split_var_ids;
    std::unordered_set<int> blacklisted_variables;

    vector<vector<vector<pair<int, int>>>> factored_effects_by_variable;

    std::vector<int> abstract_domain_sizes;
    std::vector<int> real_domain_sizes;

    DomainMapping compute_initial_domain_mapping(const TaskProxy &task_proxy);
    vector<int> compute_initial_split(
        int var_id, const TaskProxy &task_proxy, int &abstraction_size);
    bool initialization_fits_size_limit(int abstraction_size, int var_id);
    pair<int, vector<int>> get_goal_value_split(
        int var_id, const TaskProxy &task_proxy);
    pair<int, vector<int>> get_init_value_split(
        int var_id, const TaskProxy &task_proxy);
    pair<int, vector<int>> get_random_value_split(
        int var_id, const TaskProxy &task_proxy);
    pair<int, vector<int>> get_identity_split(
        int var_id, const TaskProxy &task_proxy);
    pair<int, vector<int>> get_random_partition_split(
        int var_id, const TaskProxy &task_proxy, int abstraction_size);
    pair<int, vector<int>> get_random_init_goal_partition_split(
        int var_id, const TaskProxy &task_proxy);

    bool termination_criterion_satisfied(utils::CountdownTimer &timer);

    std::vector<FactPair> get_flaws(const TaskProxy &task_proxy,
                                    const State &concrete_init,
                                    const DomainAbstraction &abstraction) const;
    bool fix_flaws(std::vector<FactPair> &&flaws,
                   DomainMapping &domain_mapping, int abstraction_size);
    bool fix_single_random_flaw(std::vector<FactPair> &&flaws,
                                DomainMapping &domain_mapping,
                                int abstraction_size);
    bool fix_single_flaw_max_refined(
            vector<FactPair> &&flaws, DomainMapping &domain_mapping,
            int abstraction_size);
    bool fix_flaws_per_atom(std::vector<FactPair> &&flaws,
                            DomainMapping &domain_mapping,
                            int abstraction_size);
    bool fix_flaws_per_variable(std::vector<FactPair> &&flaws,
                                DomainMapping &domain_mapping,
                                int abstraction_size);

    bool can_refine_variable(int old_abstraction_size, int var_id);

    void add_variable_to_abstraction_if_necessary(
        int var, DomainMapping &abstraction);

    void precompute_factored_effects_by_variable(const TaskProxy &task_proxy);

    void print_statistics(const TaskProxy &task_proxy);
public:
    CEGAR(int max_abstraction_size,
          double max_time,
          bool use_wildcard_plans,
          FlawTreatment flaw_treatment,
          InitSplitMethod init_split_method,
          utils::LogProxy &log,
          const shared_ptr<utils::RandomNumberGenerator> &rng,
          unordered_set<int> &&init_split_var_ids,
          unordered_set<int> &&blacklisted_variables);

    DomainAbstraction build_abstraction(const TaskProxy &task_proxy);
};

CEGAR::CEGAR(
        int max_abstraction_size,
        double max_time,
        bool use_wildcard_plans,
        FlawTreatment flaw_treatment,
        InitSplitMethod init_split_method,
        utils::LogProxy &log,
        const shared_ptr<utils::RandomNumberGenerator> &rng,
        unordered_set<int> &&init_split_var_ids,
        unordered_set<int> &&blacklisted_variables)
    : max_abstraction_size(max_abstraction_size),
      max_time(max_time),
      use_wildcard_plans(use_wildcard_plans),
      flaw_treatment(flaw_treatment),
      init_split_method(init_split_method),
      log(log),
      rng(rng),
      init_split_var_ids(move(init_split_var_ids)),
      blacklisted_variables(move(blacklisted_variables)) {
    /* TODO: Should we check somewhere that *init_split_var_ids* does not
        contain elements that are blacklisted? */
}

DomainMapping CEGAR::compute_initial_domain_mapping(
    const TaskProxy &task_proxy) {
    const int num_variables = task_proxy.get_variables().size();
    abstract_domain_sizes.resize(num_variables, 1);
    real_domain_sizes.reserve(num_variables);
    for (int i = 0; i < num_variables; ++i) {
        real_domain_sizes.push_back(
            task_proxy.get_variables()[i].get_domain_size());
    }

    DomainMapping domain_mapping(num_variables);
    if (!init_split_var_ids.empty()) {
        vector<int> split_vars(
            init_split_var_ids.begin(), init_split_var_ids.end());
        rng->shuffle(split_vars);
        int abstraction_size = 1;
        for (const int var_id : split_vars) {
            if (blacklisted_variables.count(var_id) == 0) {
                domain_mapping[var_id] = compute_initial_split(
                    var_id, task_proxy, abstraction_size);
            }
        }
    }
    return domain_mapping;
}

#ifndef NDEBUG
static bool variable_specified_in_goal(int var_id,
                                       const TaskProxy &task_proxy) {
    bool is_goal = false;
    for (const FactProxy &goal: task_proxy.get_goals()) {
        if (goal.get_variable().get_id() == var_id) {
            is_goal = true;
            break;
        }
    }
    return is_goal;
}
#endif

vector<int> CEGAR::compute_initial_split(
    int var_id, const TaskProxy &task_proxy, int &abstraction_size) {
    if (init_split_method == InitSplitMethod::RANDOM_PARTITION
        || initialization_fits_size_limit(abstraction_size, var_id)) {
        pair<int, vector<int>> init_split;
        switch (init_split_method) {
        case InitSplitMethod::GOAL_VALUE:
            assert(variable_specified_in_goal(var_id, task_proxy));
        case InitSplitMethod::GOAL_VALUE_OR_RANDOM_IF_NON_GOAL:
            init_split = get_goal_value_split(var_id, task_proxy);
            break;
        case InitSplitMethod::INIT_VALUE:
            init_split = get_init_value_split(var_id, task_proxy);
            break;
        case InitSplitMethod::RANDOM_VALUE:
            init_split = get_random_value_split(var_id, task_proxy);
            break;
        case InitSplitMethod::IDENTITY:
            init_split = get_identity_split(var_id, task_proxy);
            break;
        case InitSplitMethod::RANDOM_PARTITION:
            init_split = get_random_partition_split(var_id, task_proxy, abstraction_size);
            break;
        case InitSplitMethod::RANDOM_BINARY_PARTITION_SEPARATING_INIT_GOAL:
            init_split = get_random_init_goal_partition_split(
                var_id, task_proxy);
            break;
        }
        abstract_domain_sizes[var_id] = init_split.first;
        abstraction_size *= init_split.first;
        return init_split.second;
    }
    return {};
}

bool CEGAR::initialization_fits_size_limit(int old_abstraction_size,
                                           int var_id) {
    int split_domain_size = -1;
    switch (init_split_method) {
    case InitSplitMethod::GOAL_VALUE:
    case InitSplitMethod::INIT_VALUE:
    case InitSplitMethod::RANDOM_VALUE:
    case InitSplitMethod::GOAL_VALUE_OR_RANDOM_IF_NON_GOAL:
    case InitSplitMethod::RANDOM_BINARY_PARTITION_SEPARATING_INIT_GOAL:
        split_domain_size = 2;
        break;
    case InitSplitMethod::IDENTITY:
        assert(utils::in_bounds(var_id, real_domain_sizes));
        split_domain_size = real_domain_sizes[var_id];
        break;
    case InitSplitMethod::RANDOM_PARTITION:
        assert(false);
        break;
    }
    assert(split_domain_size > 0);
    if (utils::is_product_within_limit(
        old_abstraction_size, split_domain_size, max_abstraction_size)) {
        return true;
    }
    if (log.is_at_least_debug()) {
        log << "Initial split for variable " << var_id
            << " exceeds size limit." << endl;
    }
    // Do not blacklist because we might still be able to fix single flaws.
    //blacklisted_variables.insert(var_id);
    return false;
}

pair<int, vector<int>> CEGAR::get_goal_value_split(
    int var_id, const TaskProxy &task_proxy) {
    vector<int> init_split(
        task_proxy.get_variables()[var_id].get_domain_size(), 0);

    int goal_value = -1;
    for (FactProxy goal : task_proxy.get_goals()) {
        if (goal.get_variable().get_id() == var_id) {
            goal_value = goal.get_value();
            break;
        }
    }

    if (goal_value < 0) {
        // Variable not relevant in goal.
        return get_random_value_split(var_id, task_proxy);
    } else {
        init_split[goal_value] = 1;
        return make_pair(2, move(init_split));
    }
}

pair<int, vector<int>> CEGAR::get_init_value_split(
    int var_id, const TaskProxy &task_proxy) {
    vector<int> init_split(
        task_proxy.get_variables()[var_id].get_domain_size(), 0);

    int init_value =
        task_proxy.get_initial_state().get_unpacked_values()[var_id];
    assert(utils::in_bounds(init_value, init_split));
    init_split[init_value] = 1;
    return make_pair(2, move(init_split));
}

pair<int, vector<int>> CEGAR::get_random_value_split(
    int var_id, const TaskProxy &task_proxy) {
    int domain_size = task_proxy.get_variables()[var_id].get_domain_size();
    int split_val = rng->random(domain_size);
    vector<int> init_split(domain_size, 0);
    init_split[split_val] = 1;
    return make_pair(2, move(init_split));
}

pair<int, vector<int>> CEGAR::get_identity_split(
    int var_id, const TaskProxy &task_proxy) {
    int domain_size = task_proxy.get_variables()[var_id].get_domain_size();
    vector<int> init_split(domain_size);
    for (int i = 0; i < domain_size; ++i) {
        init_split[i] = i;
    }
    return make_pair(domain_size, move(init_split));
}

pair<int, vector<int>> CEGAR::get_random_partition_split(
    int var_id, const TaskProxy &task_proxy, int abstraction_size) {
    int domain_size = task_proxy.get_variables()[var_id].get_domain_size();
    int abstract_domain_size = 0;
    vector<int> init_split(domain_size);
    vector<int> value_map(domain_size, -1);
    for (int i = 0; i < domain_size; ++i) {
        int r = rng->random(domain_size);
        if (value_map[r] == -1) {
            value_map[r] = abstract_domain_size++;
        }
        init_split[i] = value_map[r];
    }
    assert(abstract_domain_size > 0);
    if (abstract_domain_size > 1) {
        if (abstract_domain_size > 1 && utils::is_product_within_limit(
            abstraction_size, abstract_domain_size, max_abstraction_size)) {
            return make_pair(abstract_domain_size, move(init_split));
        } else if (log.is_at_least_debug()) {
            log << "Initial split for variable " << var_id
                << " exceeds size limit." << endl;
        }
    }
    return make_pair(1, vector<int>{});
}

pair<int, vector<int>> CEGAR::get_random_init_goal_partition_split(
    int var_id, const TaskProxy &task_proxy) {
    int domain_size = task_proxy.get_variables()[var_id].get_domain_size();
    vector<int> init_split(domain_size);
    for (int i = 0; i < domain_size; ++i) {
        init_split[i] = rng->random(2);
    }

    /*
      If a goal value is specified for the given variable, and if it is
      different from the initial value, make sure both values end up in
      different partitions.
    */
    for (FactProxy goal : task_proxy.get_goals()) {
        if (goal.get_variable().get_id() == var_id) {
            int init_val =
                task_proxy.get_initial_state().get_unpacked_values()[var_id];
            int goal_val = goal.get_value();
            if (init_val != goal_val) {
                init_split[init_val] = 0;
                init_split[goal_val] = 1;
            }
        }
    }

    int first_val = init_split[0];
    for (int i = 1; i < domain_size; ++i) {
        if (init_split[i] != first_val) {
            return make_pair(2, move(init_split));
        }
    }
    // Partition resulted in trivial variable.
    return make_pair(1, vector<int>{});
}

static vector<FactPair> get_precondition_flaws(
    const OperatorProxy &op, const vector<int> &current_state,
    const unordered_set<int> &blacklisted_variables) {
    vector<FactPair> flaws;
    for (FactProxy pre : op.get_preconditions()) {
        int var_id = pre.get_variable().get_id();
        if (blacklisted_variables.count(var_id) == 0
            && current_state[var_id] != pre.get_value()) {
            flaws.emplace_back(var_id, pre.get_value());
        }
    }
    return flaws;
}

static vector<FactPair> get_goal_flaws(
    const GoalsProxy &goals, const vector<int> &current_state,
    const unordered_set<int> &blacklisted_variables) {
    vector<FactPair> flaws;
    for (const FactProxy &goal : goals) {
        int var_id = goal.get_variable().get_id();
        if (blacklisted_variables.count(var_id) == 0
            && current_state[var_id] != goal.get_value()) {
            flaws.emplace_back(var_id, goal.get_value());
        }
    }
    return flaws;
}

static vector<FactPair> get_state_mismatch_flaws(
    const DomainAbstraction &abstraction,
    const vector<int> &current_state,
    int expected_state_hash,
    int predecessor_hash,
    const vector<int> &abstract_domain_sizes,
    const vector<vector<pair<int, int>>> &last_op_factored_effects,
    const unordered_set<int> &blacklisted_variables) {
    if (abstraction.get_abstract_state_hash(current_state)
        == expected_state_hash) {
        return {};
    }

    vector<FactPair> flaws;
    const vector<vector<int>> &domain_mapping =
        abstraction.get_domain_mapping();
    int num_variables = static_cast<int>(domain_mapping.size());
    for (int var_id = 0; var_id < num_variables; ++var_id) {
        if (domain_mapping[var_id].empty()
            || blacklisted_variables.count(var_id)) {
            continue;
        }
        int multiplier = abstraction.get_hash_multiplier(var_id);
        int abstract_domain_size = abstract_domain_sizes[var_id];
        int expected_value =
            (expected_state_hash / multiplier) % abstract_domain_size;
        int actual_value = domain_mapping[var_id][current_state[var_id]];
        if (expected_value == actual_value) {
            continue;
        }
        int predecessor_value =
            (predecessor_hash / multiplier) % abstract_domain_size;
        assert(utils::in_bounds(var_id, last_op_factored_effects));
        for (const pair<int, int> &factored_effect : last_op_factored_effects[var_id]) {
            int condition = factored_effect.first;
            int effect = factored_effect.second;
            if (domain_mapping[var_id][effect] == expected_value
                && domain_mapping[var_id][condition] == predecessor_value) {
                flaws.emplace_back(var_id, condition);
            }
        }
    }
    return flaws;
}

/*
  Apply the operator to the given state, ignoring that the operator is
  potentially not applicable in the state, and expecting that the operator
  has not conditional effects.
*/
static void apply_op_to_state(vector<int> &state, const OperatorProxy &op) {
    assert(!op.is_axiom());
    vector<bool> changed_variables(state.size(), false);
    for (EffectProxy effect : op.get_effects()) {
        FactPair effect_fact = effect.get_fact().get_pair();
        if (changed_variables[effect_fact.var]) {
            assert(!effect.get_conditions().empty());
            /* We assume effects are conflict-free, so another effect on the
               same variable will not trigger. */
            continue;
        }
        if (effect.get_conditions().empty()) {
            // Unconditional effect.
            state[effect_fact.var] = effect_fact.value;
            changed_variables[effect_fact.var] = true;
        } else {
            // Effect is factored.
            assert(effect.get_conditions().size() == 1);
            FactPair effect_condition = effect.get_conditions()[0].get_pair();
            assert(effect_condition.var == effect_fact.var);
            if (state[effect_fact.var] == effect_condition.value) {
                state[effect_fact.var] = effect_fact.value;
                changed_variables[effect_fact.var] = true;
            }
        }
    }
}


vector<FactPair> CEGAR::get_flaws(
    const TaskProxy &task_proxy, const State &concrete_init,
    const DomainAbstraction &abstraction) const {
    vector<int> current_state = concrete_init.get_unpacked_values();
    vector<vector<OperatorID>> wildcard_plan = abstraction.get_plan();
    vector<int> trace = abstraction.get_trace();
    vector<FactPair> flaws;

    assert(wildcard_plan.size() + 1 == trace.size());
    for (int i = 0; i < static_cast<int>(wildcard_plan.size()); ++i) {
        assert(flaws.empty());
        vector<OperatorID> &equivalent_ops = wildcard_plan[i];
        int last_op_id = -1;
        for (OperatorID op_id : equivalent_ops) {
            OperatorProxy op = task_proxy.get_operators()[op_id];
            vector<FactPair> operator_flaws = get_precondition_flaws(
                op, current_state, blacklisted_variables);

            if (operator_flaws.empty()) {
                flaws.clear();
                apply_op_to_state(current_state, op);
                last_op_id = op.get_id();
                break;
            } else {
                for (FactPair &flaw : operator_flaws) {
                    flaws.emplace_back(flaw.var, flaw.value);
                }
            }
        }
        if (flaws.empty()) {
            /* Theoretically only necessary when last_op has (factored)
               conditional effects (I think). */
            int abstract_predecessor = trace[i];
            int abstract_successor = trace[i + 1];
            assert(utils::in_bounds(last_op_id, factored_effects_by_variable));
            vector<vector<pair<int, int>>> last_op_factored_effects =
                factored_effects_by_variable[last_op_id];
            flaws = get_state_mismatch_flaws(
                abstraction, current_state, abstract_successor,
                abstract_predecessor, abstract_domain_sizes,
                last_op_factored_effects, blacklisted_variables);
        }
        if (!flaws.empty()) {
            return flaws;
        }
    }

    assert(flaws.empty());
    flaws = get_goal_flaws(task_proxy.get_goals(), current_state,
                           blacklisted_variables);
    return flaws;
}

bool CEGAR::fix_flaws(
    vector<FactPair> &&flaws, DomainMapping &domain_mapping,
    int abstraction_size) {
    bool fixed = false;
    switch(flaw_treatment) {
        case FlawTreatment::RANDOM_SINGLE_ATOM:
            fixed = fix_single_random_flaw(
                move(flaws), domain_mapping, abstraction_size);
            break;
        case FlawTreatment::ONE_SPLIT_PER_ATOM:
            fixed = fix_flaws_per_atom(
                move(flaws), domain_mapping, abstraction_size);
            break;
        case FlawTreatment::ONE_SPLIT_PER_VARIABLE:
            fixed = fix_flaws_per_variable(
                move(flaws), domain_mapping, abstraction_size);
            break;
        case FlawTreatment::MAX_REFINED_SINGLE_ATOM:
            fixed = fix_single_flaw_max_refined(
                move(flaws), domain_mapping, abstraction_size);
            break;
    }
#ifndef NDEBUG
    for (int var_id = 0; var_id < static_cast<int>(domain_mapping.size()); ++var_id) {
        const vector<int> &mapping = domain_mapping[var_id];
        if (mapping.empty()) {
            continue;
        }
        for (int i = 0; i < abstract_domain_sizes[var_id]; ++i) {
            assert(count(mapping.begin(), mapping.end(), i));
        }
    }
#endif
    return fixed;
}

bool CEGAR::fix_single_random_flaw(
    vector<FactPair> &&flaws, DomainMapping &domain_mapping,
    int abstraction_size) {
    // TODO: Number of repetitions set to log(|flaws|) + 1 is somewhat arbitrary...
    int repetitions = ceil(1 + std::log(flaws.size()));
    for (int i = 0; i < repetitions; ++i) {
        FactPair fact(*rng->choose(flaws));
        if (can_refine_variable(abstraction_size, fact.var)) {
            add_variable_to_abstraction_if_necessary(fact.var, domain_mapping);
            domain_mapping[fact.var][fact.value] =
                abstract_domain_sizes[fact.var];
            abstract_domain_sizes[fact.var] += 1;
            return true;
        }
    }
    return false;
}

/* Chooses a flaw for that the increase in abstraction size is the smallest among all given ones
 * -> Leads to the smallest possible increase in abstraction size in every iteration */
bool CEGAR::fix_single_flaw_max_refined(
        vector<FactPair> &&flaws, DomainMapping &domain_mapping,
        int abstraction_size) {
    // determine domain sizes of flaws, select the ones with max refinement
    int current_max_domain_size = 0;
    vector<int> current_flaw_candidates;
    int num_flaws = (int) flaws.size();

    for (int i = 0; i < num_flaws; ++i) {
        // determine domain size
        int domain_size = abstract_domain_sizes[flaws[i].var];
        // check how domain size of flaw.var ranks
        if (domain_size > current_max_domain_size) {
            current_flaw_candidates.clear();
            current_flaw_candidates.emplace_back(i);
            current_max_domain_size = domain_size;
        } else if (domain_size == current_max_domain_size) {
            current_flaw_candidates.emplace_back(i);
        }
    }
    /* We do not repeat the selection of flaws since with this Method the abstraction size increase can not get lower
     * with another choice -> If the first choice does not work, none will! */
    FactPair fact (flaws[*rng->choose(current_flaw_candidates)]);
    if (can_refine_variable(abstraction_size, fact.var)) {
        add_variable_to_abstraction_if_necessary(fact.var, domain_mapping);
        domain_mapping[fact.var][fact.value] = abstract_domain_sizes[fact.var];
        abstract_domain_sizes[fact.var] += 1;
        return true;
    }
    return false;
}

bool CEGAR::fix_flaws_per_atom(
    vector<FactPair> &&flaws, DomainMapping &domain_mapping,
    int abstraction_size) {
    // FIXME: Bias for variables with low index.
    sort(flaws.begin(), flaws.end());
    FactPair last_flaw(-1, -1);
    for (const FactPair &flaw : flaws) {
        if (flaw == last_flaw) {
            // duplicate
            continue;
        }
        if (can_refine_variable(abstraction_size, flaw.var)) {
            add_variable_to_abstraction_if_necessary(flaw.var, domain_mapping);
            domain_mapping[flaw.var][flaw.value] =
                abstract_domain_sizes[flaw.var];
            abstract_domain_sizes[flaw.var] += 1;
            last_flaw = flaw;
        }
    }
    return last_flaw != FactPair(-1, -1);
}

bool CEGAR::fix_flaws_per_variable(
    vector<FactPair> &&flaws, DomainMapping &domain_mapping,
    int abstraction_size) {
    // FIXME: Bias for variables with low index.
    sort(flaws.begin(), flaws.end());
    FactPair last_flaw(-1, -1);
    for (const FactPair &flaw : flaws) {
        if (flaw.var > last_flaw.var
            && can_refine_variable(abstraction_size, flaw.var)) {
            /* Introduce new abstract value only for every new variable,
               opposed to for every atom as in *fix_flaws_per_atom* above. */
            add_variable_to_abstraction_if_necessary(flaw.var, domain_mapping);
            abstract_domain_sizes[flaw.var] += 1;
        } else if (flaw.var != last_flaw.var || flaw.value == last_flaw.value) {
            // Duplicate or does not fit size limit.
            continue;
        }
        domain_mapping[flaw.var][flaw.value] =
            abstract_domain_sizes[flaw.var] - 1;
        last_flaw = flaw;
    }
    return last_flaw != FactPair(-1, -1);
}

void CEGAR::print_statistics(
    const TaskProxy &task_proxy) {
    assert(log.is_at_least_normal());

    int num_variables = task_proxy.get_variables().size();
    int abstraction_size = 1;
    int num_trivial_variables = 0;
    int num_complete_variables = 0;
    double avg_domain_size = 0;
    for (int i = 0; i < num_variables; ++i) {
        abstraction_size *= abstract_domain_sizes[i];
        int original_domain_size =
            task_proxy.get_variables()[i].get_domain_size();
        if (abstract_domain_sizes[i] == 1) {
            ++num_trivial_variables;
        }
        if (abstract_domain_sizes[i] == original_domain_size) {
            ++num_complete_variables;
        }
        double domain_size_ratio =
            ((double) abstract_domain_sizes[i]) / original_domain_size;
        avg_domain_size += domain_size_ratio / num_variables;
    }

    log << "Final abstraction size: " << abstraction_size << endl;
    log << "Total variables: " << num_variables << endl;
    log << "Trivial variables: " << num_trivial_variables << endl;
    log << "Complete variables: " << num_complete_variables << endl;
    log << "Average domain size: " << avg_domain_size << endl;
}

void CEGAR::precompute_factored_effects_by_variable(
    const TaskProxy &task_proxy) {
    assert(factored_effects_by_variable.empty());
    factored_effects_by_variable.reserve(task_proxy.get_operators().size());
    for (const OperatorProxy &op : task_proxy.get_operators()) {
        assert(op.get_id() ==
            static_cast<int>(factored_effects_by_variable.size()));
        vector<vector<pair<int, int>>> effects_by_variable(
            task_proxy.get_variables().size(), vector<pair<int, int>>{});
        for (const auto &effect : op.get_effects()) {
            FactPair eff = effect.get_fact().get_pair();
            if (!effect.get_conditions().empty()) {
                assert(effect.get_conditions().size() == 1);
                FactPair condition = effect.get_conditions()[0].get_pair();
                assert(condition.var == eff.var);
                effects_by_variable[eff.var].emplace_back(
                    condition.value, eff.value);
            }
        }
        factored_effects_by_variable.push_back(move(effects_by_variable));
    }
}

void CEGAR::add_variable_to_abstraction_if_necessary(
    int var, DomainMapping &abstraction) {
    if (abstraction[var].empty()) {
        int real_domain_size = real_domain_sizes[var];
        abstraction[var].resize(real_domain_size, 0);
    }
}

DomainAbstraction CEGAR::build_abstraction(
    const TaskProxy &task_proxy) {
    if (log.is_at_least_normal()) {
        log << "Building domain abstraction..." << endl;
    }
    utils::reserve_extra_memory_padding(memory_padding_in_mb);
    utils::CountdownTimer timer(max_time);

    precompute_factored_effects_by_variable(task_proxy);
    DomainMapping domain_mapping =
        compute_initial_domain_mapping(task_proxy);
    if (log.is_at_least_debug()) {
        log << "Initial domain mapping: " << domain_mapping << endl;
    }
    DomainAbstractionFactory factory(
        task_proxy, domain_mapping, abstract_domain_sizes, true, rng,
        use_wildcard_plans);
    DomainAbstraction abstraction = factory.generate();

    int iteration = 1;
    State concrete_init = task_proxy.get_initial_state();
    concrete_init.unpack();
    while (!termination_criterion_satisfied(timer)) {
        if (log.is_at_least_debug()) {
            abstraction.dump(log);
            log << "iteration #" << iteration << endl;
        }

        vector<FactPair> flaws =
            get_flaws(task_proxy, concrete_init, abstraction);

        if (flaws.empty()) {
            if (log.is_at_least_normal()) {
                log << "No more flaws found, terminating CEGAR refinement."
                    << endl;
            }
            break;
        }

        bool flaws_fixed =
            fix_flaws(move(flaws), domain_mapping, abstraction.size());
#ifndef NDEBUG
        for (int i = 0; i < static_cast<int>(abstract_domain_sizes.size()); ++i) {
            assert(abstract_domain_sizes[i] <= real_domain_sizes[i]);
        }
#endif
        if (!flaws_fixed) {
            assert(max_abstraction_size != numeric_limits<int>::max());
            if (log.is_at_least_normal()) {
                log << "Terminating CEGAR loop because fixing flaws "
                    << "surpasses abstraction size limit of "
                    << max_abstraction_size << " states." << endl;
            }
            break;
        }

        DomainAbstractionFactory new_factory(
            task_proxy, domain_mapping, abstract_domain_sizes, true, rng, true);
        abstraction = new_factory.generate();
        ++iteration;
    }

    if (utils::extra_memory_padding_is_reserved()) {
        utils::release_extra_memory_padding();
    }

    if (log.is_at_least_normal()) {
        print_statistics(task_proxy);
        log << "Number of CEGAR iterations: " << iteration << endl;
        if (log.is_at_least_debug()) {
            abstraction.dump(log);
        }
    }

    return abstraction;
}

bool CEGAR::termination_criterion_satisfied(
    utils::CountdownTimer &timer) {
    if (timer.is_expired()) {
        if (log.is_at_least_normal()) {
            log << "Terminating CEGAR; time limit reached." << endl;
        }
        return true;
    }
    if (!utils::extra_memory_padding_is_reserved()) {
        if (log.is_at_least_normal()) {
            log << "Terminating CEGAR; memory limit reached." << endl;
        }
        return true;
    }
    return false;
}

bool CEGAR::can_refine_variable(
    int old_abstraction_size, int var_id) {
    // TODO: ideally, at this point, we would have checked if var_id is already
    // blacklisted. However, this doesn't seem to work with all of the fix
    // methods.
//    assert(!blacklisted_variables.count(var_id));
    if (blacklisted_variables.count(var_id)) {
        return false;
    }
    int domain_size = abstract_domain_sizes[var_id];
    int abs_size_without_var = old_abstraction_size / domain_size;
    if (utils::is_product_within_limit(abs_size_without_var, domain_size + 1,
                                       max_abstraction_size)) {
        return true;
    }
    if (log.is_at_least_debug()) {
        log << "Cannot refine " << var_id << "; blacklisting" << endl;
    }
    blacklisted_variables.insert(var_id);
    return false;
}

DomainAbstraction generate_domain_abstraction_with_cegar(
        int max_abstraction_size,
        double max_time,
        bool use_wildcard_plans,
        FlawTreatment flaw_treatment,
        InitSplitMethod init_split_method,
        utils::LogProxy &log,
        const shared_ptr<utils::RandomNumberGenerator> &rng,
        const TaskProxy &task_proxy,
        unordered_set<int> &&init_split_var_ids,
        unordered_set<int> &&blacklisted_variables) {
    CEGAR cegar(
        max_abstraction_size,
        max_time,
        use_wildcard_plans,
        flaw_treatment,
        init_split_method,
        log,
        rng,
        move(init_split_var_ids),
        move(blacklisted_variables));
    return cegar.build_abstraction(task_proxy);
}

void add_cegar_implementation_notes_to_parser(options::OptionParser &parser) {
    parser.document_note(
        "Short description of the CEGAR algorithm",
        "The CEGAR algorithm computes a pattern collection for a given planning "
        "task and a given (sub)set of its goals in a randomized order as "
        "follows. Starting from the pattern collection consisting of a singleton "
        "pattern for each goal variable, it repeatedly attempts to execute an "
        "optimal plan of each pattern in the concrete task, collects reasons why "
        "this is not possible (so-called flaws) and refines the pattern in "
        "question by adding a variable to it.\n"
        "Further parameters allow blacklisting a (sub)set of the non-goal "
        "variables which are then never added to the collection, limiting PDB "
        "and collection size, setting a time limit and switching between "
        "computing regular or wildcard plans, where the latter are sequences of "
        "parallel operators inducing the same abstract transition.",
        true);
    parser.document_note(
        "Implementation notes about the CEGAR algorithm",
        "The following describes differences of the implementation to "
        "the original implementation used and described in the paper.\n\n"
        "Conceptually, there is one larger difference which concerns the "
        "computation of (regular or wildcard) plans for PDBs. The original "
        "implementation used an enforced hill-climbing (EHC) search with the "
        "PDB as the perfect heuristic, which ensured finding strongly optimal "
        "plans, i.e., optimal plans with a minimum number of zero-cost "
        "operators, in domains with zero-cost operators. The original "
        "implementation also slightly modified EHC to search for a best-"
        "improving successor, chosen uniformly at random among all best-"
        "improving successors.\n\n"
        "In contrast, the current implementation computes a plan alongside the "
        "computation of the PDB itself. A modification to Dijkstra's algorithm "
        "for computing the PDB values stores, for each state, the operator "
        "leading to that state (in a regression search). This generating "
        "operator is updated only if the algorithm found a cheaper path to "
        "the state. After Dijkstra finishes, the plan computation starts at the "
        "initial state and iteratively follows the generating operator, computes "
        "all operators of the same cost inducing the same transition, until "
        "reaching a goal. This constitutes a wildcard plan. It is turned into a "
        "regular one by randomly picking a single operator for each transition. "
        "\n\n"
        "Note that this kind of plan extraction does not consider all successors "
        "of a state uniformly at random but rather uses the previously deterministically "
        "chosen generating operator to settle on one successor state, which is "
        "biased by the number of operators leading to the same successor from "
        "the given state. Further note that in the presence of zero-cost "
        "operators, this procedure does not guarantee that the computed plan is "
        "strongly optimal because it does not minimize the number of used "
        "zero-cost operators leading to the state when choosing a generating "
        "operator. Experiments have shown (issue1007) that this speeds up the "
        "computation significantly while not having a strongly negative effect "
        "on heuristic quality due to potentially computing worse plans.\n\n"
        "Two further changes fix bugs of the original implementation to match "
        "the description in the paper. The first bug fix is to raise a flaw "
        "for all goal variables of the task if the plan for a PDB can be "
        "executed on the concrete task but does not lead to a goal state. "
        "Previously, such flaws would not have been raised because all goal "
        "variables are part of the collection from the start on and therefore "
        "not considered. This means that the original implementation "
        "accidentally disallowed merging patterns due to goal violation "
        "flaws. The second bug fix is to actually randomize the order of "
        "parallel operators in wildcard plan steps.",
        true);
}

void add_domain_abstraction_cegar_options_to_parser(
    options::OptionParser &parser) {
    parser.add_option<bool>(
        "use_wildcard_plans",
        "Consider parallel transitions in abstraction.",
        "true");
    vector<string> init_split_method;
    init_split_method.emplace_back("goal_value");
    init_split_method.emplace_back("goal_value_or_random_if_non_goal");
    init_split_method.emplace_back("init_value");
    init_split_method.emplace_back("random_value");
    init_split_method.emplace_back("random_partition");
    init_split_method.emplace_back(
        "random_binary_partition_separating_init_goal");
    init_split_method.emplace_back("identity");
    parser.add_enum_option<InitSplitMethod>(
        "init_split_method",
        init_split_method,
        "Choose how to initialize splits to seed diversification.",
        "init_value");
    vector<string> flaw_treatment;
    flaw_treatment.emplace_back("random_single_atom");
    flaw_treatment.emplace_back("one_split_per_atom");
    flaw_treatment.emplace_back("one_split_per_variable");
    flaw_treatment.emplace_back("max_refined_single_atom");
    parser.add_enum_option<FlawTreatment>(
        "flaw_treatment",
        flaw_treatment,
        "Flaws are found in collections and can be treated in different ways. "
        "This option allows to switch between them.");
}
}
