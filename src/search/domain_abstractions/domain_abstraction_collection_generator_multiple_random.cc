#include "domain_abstraction_collection_generator_multiple_random.h"

#include "domain_abstraction.h"
#include "random.h"
#include "utils.h"

#include "../option_parser.h"
#include "../plugin.h"
#include "../task_proxy.h"

#include "../utils/logging.h"

#include <vector>

using namespace std;

namespace domain_abstractions {
DomainAbstractionCollectionGeneratorMultipleRandom::DomainAbstractionCollectionGeneratorMultipleRandom(
    options::Options &opts)
    : DomainAbstractionCollectionGeneratorMultiple(opts),
      bidirectional(opts.get<bool>("bidirectional")) {
}

string DomainAbstractionCollectionGeneratorMultipleRandom::id() const {
    return "random patterns";
}

void DomainAbstractionCollectionGeneratorMultipleRandom::initialize(
    const TaskProxy &task_proxy) {
    // Compute CG neighbors once. They are shuffled when used.
    cg_neighbors = compute_cg_neighbors(task_proxy, bidirectional);
}

DomainAbstraction DomainAbstractionCollectionGeneratorMultipleRandom::compute_abstraction(
    int max_abstraction_size,
    double max_time,
    const shared_ptr<utils::RandomNumberGenerator> &rng,
    const TaskProxy &task_proxy,
    const FactPair &goal,
    unordered_set<int> &&/*init_split_var_ids*/,
    unordered_set<int> &&/*blacklisted_variables*/) {
    // TODO: add support for blacklisting in single RCG?
    utils::LogProxy silent_log = utils::get_silent_log();
    return generate_random_domain_abstraction(
        max_abstraction_size,
        max_time,
        silent_log,
        rng,
        task_proxy,
        goal.var,
        cg_neighbors);
}

static shared_ptr<DomainAbstractionCollectionGenerator> _parse(options::OptionParser &parser) {
    parser.document_synopsis(
        "Multiple Random DomainAbstractions",
        "This pattern collection generator implements the 'multiple "
        "randomized causal graph' (mRCG) algorithm described in experiments of "
        "the paper" + get_rovner_et_al_reference() +
        "It is an instantiation of the 'multiple algorithm framework'. "
        "To compute a pattern in each iteration, it uses the random "
        "pattern algorithm, called 'single randomized causal graph' (sRCG) "
        "in the paper. See below for descriptions of the algorithms.");
    add_random_domain_abstraction_implementation_notes_to_parser(parser);
    add_multiple_algorithm_implementation_notes_to_parser(parser);
    add_multiple_options_to_parser(parser);
    add_random_domain_abstraction_bidirectional_option_to_parser(parser);

    Options opts = parser.parse();
    if (parser.dry_run()) {
        return nullptr;
    }

    return make_shared<DomainAbstractionCollectionGeneratorMultipleRandom>(opts);
}

static Plugin<DomainAbstractionCollectionGenerator> _plugin("multiple_domain_abstractions_random", _parse);
}
