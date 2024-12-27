#include "utils.h"

#include "domain_abstraction.h"

#include "../task_proxy.h"

#include "../task_utils/causal_graph.h"
#include "../task_utils/task_properties.h"

#include "../utils/logging.h"
#include "../utils/markup.h"
#include "../utils/math.h"
#include "../utils/rng.h"

#include <limits>

using namespace std;

namespace domain_abstractions {
vector<FactPair> get_goals_in_random_order(
    const TaskProxy &task_proxy, utils::RandomNumberGenerator &rng) {
    vector<FactPair> goals = task_properties::get_fact_pairs(task_proxy.get_goals());
    rng.shuffle(goals);
    return goals;
}

vector<int> get_non_goal_variables(const TaskProxy &task_proxy) {
    size_t num_vars = task_proxy.get_variables().size();
    GoalsProxy goals = task_proxy.get_goals();
    vector<bool> is_goal(num_vars, false);
    for (FactProxy goal : goals) {
        is_goal[goal.get_variable().get_id()] = true;
    }

    vector<int> non_goal_variables;
    non_goal_variables.reserve(num_vars - goals.size());
    for (int var_id = 0; var_id < static_cast<int>(num_vars); ++var_id) {
        if (!is_goal[var_id]) {
            non_goal_variables.push_back(var_id);
        }
    }
    return non_goal_variables;
}

vector<vector<int>> compute_cg_neighbors(
    const TaskProxy &task_proxy,
    bool bidirectional) {
    int num_vars = task_proxy.get_variables().size();
    const causal_graph::CausalGraph &cg = task_proxy.get_causal_graph();
    vector<vector<int>> cg_neighbors(num_vars);
    for (int var_id = 0; var_id < num_vars; ++var_id) {
        cg_neighbors[var_id] = cg.get_predecessors(var_id);
        if (bidirectional) {
            const vector<int> &successors = cg.get_successors(var_id);
            cg_neighbors[var_id].insert(cg_neighbors[var_id].end(), successors.begin(), successors.end());
        }
        utils::sort_unique(cg_neighbors[var_id]);
    }
    return cg_neighbors;
}

void dump_domain_abstraction_generation_statistics(
    const string &identifier,
    utils::Duration runtime,
    const DomainAbstraction &abstraction,
    utils::LogProxy &log) {
    if (log.is_at_least_normal()) {
        log << identifier << " domain mapping: " << abstraction.get_domain_mapping() << endl;
        log << identifier << " abstraction size: " << abstraction.size() << endl;
        log << identifier << " computation time: " << runtime << endl;
    }
}

void dump_domain_abstraction_collection_generation_statistics(
    const string &identifier,
    utils::Duration runtime,
    const DomainAbstractionCollection &abstractions,
    utils::LogProxy &log) {
    if (log.is_at_least_normal()) {
        log << identifier << " number of abstractions: " << abstractions.size()
            << endl;
        log << identifier << " computation time: " << runtime << endl;
    }
}

string get_rovner_et_al_reference() {
    return utils::format_conference_reference(
        {"Alexander Rovner", "Silvan Sievers", "Malte Helmert"},
        "Counterexample-Guided Abstraction Refinement for Pattern Selection "
        "in Optimal Classical Planning",
        "https://ai.dmi.unibas.ch/papers/rovner-et-al-icaps2019.pdf",
        "Proceedings of the 29th International Conference on Automated "
        "Planning and Scheduling (ICAPS 2019)",
        "362-367",
        "AAAI Press",
        "2019");
}
}
