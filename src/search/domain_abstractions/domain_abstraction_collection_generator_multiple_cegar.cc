#include "domain_abstraction_collection_generator_multiple_cegar.h"

#include "cegar.h"
#include "domain_abstraction.h"
#include "utils.h"

#include "../option_parser.h"
#include "../plugin.h"

using namespace std;

namespace domain_abstractions {
DomainAbstractionCollectionGeneratorMultipleCegar::DomainAbstractionCollectionGeneratorMultipleCegar(
    options::Options &opts)
    : DomainAbstractionCollectionGeneratorMultiple(opts),
      use_wildcard_plans(opts.get<bool>("use_wildcard_plans")),
      flaw_treatment(opts.get<FlawTreatment>("flaw_treatment")),
      init_split_method(opts.get<InitSplitMethod>("init_split_method")) {
    if (init_split_method == InitSplitMethod::GOAL_VALUE
        && init_split_variables != VariableSubset::GOALS) {
        cerr << "CEGAR domain abstraction generator was called with "
             << "init-split method *goal_value* but candidates that "
             << "are not goal variables." << endl;
        utils::exit_with(utils::ExitCode::SEARCH_UNSUPPORTED);
    }
}

string DomainAbstractionCollectionGeneratorMultipleCegar::id() const {
    return "CEGAR";
}

DomainAbstraction DomainAbstractionCollectionGeneratorMultipleCegar::compute_abstraction(
    int max_abstraction_size,
    double max_time,
    const shared_ptr<utils::RandomNumberGenerator> &rng,
    const TaskProxy &task_proxy,
    const FactPair &,
    unordered_set<int> &&init_split_var_ids,
    unordered_set<int> &&blacklisted_variables) {
    // TODO: use goal?
    utils::LogProxy silent_log = utils::get_silent_log();
    return generate_domain_abstraction_with_cegar(
        max_abstraction_size,
        max_time,
        use_wildcard_plans,
        flaw_treatment,
        init_split_method,
        silent_log,
        rng,
        task_proxy,
        move(init_split_var_ids),
        move(blacklisted_variables));
}

static shared_ptr<DomainAbstractionCollectionGenerator> _parse(options::OptionParser &parser) {
    parser.document_synopsis(
        "Multiple CEGAR",
        "This pattern collection generator implements the multiple CEGAR "
        "algorithm described in the paper" + get_rovner_et_al_reference() +
        "It is an instantiation of the 'multiple algorithm framework'. "
        "To compute a pattern in each iteration, it uses the CEGAR algorithm "
        "restricted to a single goal variable. See below for descriptions of "
        "the algorithms.");
    add_cegar_implementation_notes_to_parser(parser);
    add_multiple_algorithm_implementation_notes_to_parser(parser);
    add_multiple_options_to_parser(parser);
    add_domain_abstraction_cegar_options_to_parser(parser);

    Options opts = parser.parse();
    if (parser.dry_run()) {
        return nullptr;
    }

    return make_shared<DomainAbstractionCollectionGeneratorMultipleCegar>(opts);
}

static Plugin<DomainAbstractionCollectionGenerator> _plugin("multiple_domain_abstractions_cegar", _parse);
}
