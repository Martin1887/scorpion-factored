#ifndef DOMAIN_ABSTRACTIONS_UTILS_H
#define DOMAIN_ABSTRACTIONS_UTILS_H

#include "types.h"

#include "../task_proxy.h"

#include "../utils/timer.h"

#include <memory>
#include <string>

namespace utils {
class LogProxy;
class RandomNumberGenerator;
}

namespace domain_abstractions {
extern std::vector<FactPair> get_goals_in_random_order(
    const TaskProxy &task_proxy, utils::RandomNumberGenerator &rng);
extern std::vector<int> get_non_goal_variables(const TaskProxy &task_proxy);

/*
  Compute the causal graph neighbors for each variable of the task. If
  bidirectional is false, then only predecessors of variables are considered
  neighbors. If bidirectional is true, then the causal graph is treated as
  undirected graph and also successors of variables are considered neighbors.
*/
extern std::vector<std::vector<int>> compute_cg_neighbors(
    const TaskProxy &task_proxy,
    bool bidirectional);

/*
  Dump the given pattern, the number of variables contained, the size of the
  corresponding PDB, and the runtime used for computing it. All output is
  prepended with the given string identifier.
*/
extern void dump_domain_abstraction_generation_statistics(
    const std::string &identifier,
    utils::Duration runtime,
    const DomainAbstraction &abstraction,
    utils::LogProxy &log);

/*
  Compute and dump the number of patterns, the total size of the corresponding
  PDBs, and the runtime used for computing the collection. All output is
  prepended with the given string identifier.
*/
extern void dump_domain_abstraction_collection_generation_statistics(
    const std::string &identifier,
    utils::Duration runtime,
    const DomainAbstractionCollection &abstractions,
    utils::LogProxy &log);

extern std::string get_rovner_et_al_reference();
}

#endif
