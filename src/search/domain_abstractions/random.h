#ifndef DOMAIN_ABSTRACTIONS_RANDOM_H
#define DOMAIN_ABSTRACTIONS_RANDOM_H

#include "types.h"

#include <memory>

class TaskProxy;

namespace options {
class OptionParser;
}

namespace utils {
class LogProxy;
class RandomNumberGenerator;
}

namespace domain_abstractions {
/*
  This function computes a pattern for the given task. Starting with the given
  goal variable, the algorithm executes a random walk on the causal graph. In
  each iteration, it selects a random causal graph neighbor of the current
  variable (given via cg_neighbors). It terminates if no neighbor fits the
  pattern due to the size limit or if the time limit is reached.
*/
extern DomainAbstraction generate_random_domain_abstraction(
    int max_abstraction_size,
    double max_time,
    utils::LogProxy &log,
    const std::shared_ptr<utils::RandomNumberGenerator> &rng,
    const TaskProxy &task_proxy,
    int goal_variable,
    std::vector<std::vector<int>> &cg_neighbors);

extern void add_random_domain_abstraction_implementation_notes_to_parser(
    options::OptionParser &parser);
extern void add_random_domain_abstraction_bidirectional_option_to_parser(
    options::OptionParser &parser);
}

#endif
