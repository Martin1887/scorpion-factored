#include "random.h"

#include "domain_abstraction.h"

#include "../option_parser.h"

using namespace std;

namespace domain_abstractions {
DomainAbstraction generate_random_domain_abstraction(
    int /*max_abstraction_size*/,
    double /*max_time*/,
    utils::LogProxy &/*log*/,
    const std::shared_ptr<utils::RandomNumberGenerator> &/*rng*/,
    const TaskProxy &/*task_proxy*/,
    int /*goal_variable*/,
    std::vector<std::vector<int>> &/*cg_neighbors*/) {
    // TODO
    cerr << "random domain abstraction generation not implemented" << endl;
    utils::exit_with(utils::ExitCode::SEARCH_INPUT_ERROR);
}

void add_random_domain_abstraction_implementation_notes_to_parser(
    options::OptionParser &parser) {
    parser.document_note(
        "Short description of the random pattern algorithm",
        "The random pattern algorithm computes a pattern for a given planning "
        "task and a single goal of the task as follows. Starting with the given "
        "goal variable, the algorithm executes a random walk on the causal "
        "graph. In each iteration, it selects a random causal graph neighbor of "
        "the current variable. It terminates if no neighbor fits the pattern due "
        "to the size limit or if the time limit is reached.",
        true);
    parser.document_note(
        "Implementation notes about the random pattern algorithm",
        "In the original implementation used in the paper, the algorithm "
        "selected a random neighbor and then checked if selecting it would "
        "violate the PDB size limit. If so, the algorithm would not select "
        "it and terminate. In the current implementation, the algorithm instead "
        "loops over all neighbors of the current variable in random order and "
        "selects the first one not violating the PDB size limit. If no such "
        "neighbor exists, the algorithm terminates.",
        true);
}

void add_random_domain_abstraction_bidirectional_option_to_parser(
    options::OptionParser &parser) {
    parser.add_option<bool>(
        "bidirectional",
        "this option decides if the causal graph is considered to be "
        "directed or undirected selecting predecessors of already selected "
        "variables. If true (default), it is considered to be undirected "
        "(precondition-effect edges are bidirectional). If false, it is "
        "considered to be directed (a variable is a neighbor only if it is a "
        "predecessor.",
        "true");
}
}
