#include "domain_abstraction_heuristic.h"

#include "domain_abstraction_generator.h"

#include "../option_parser.h"
#include "../plugin.h"

#include "../task_utils/task_properties.h"

using namespace std;

namespace domain_abstractions {
static DomainAbstraction get_domain_abstraction_from_options(
    const options::Options &opts, const TaskProxy &task_proxy) {
    task_properties::verify_no_axioms(task_proxy);
    task_properties::verify_factored_effect_task(task_proxy);

    shared_ptr<DomainAbstractionGenerator> generator =
        opts.get<shared_ptr<DomainAbstractionGenerator>>(
            "domain_abstraction_generator");

    return generator->build_abstraction(task_proxy);
}

DomainAbstractionHeuristic::DomainAbstractionHeuristic(
    const options::Options &opts)
    : Heuristic(opts),
    abstraction(get_domain_abstraction_from_options(opts, task_proxy)) {
}

int DomainAbstractionHeuristic::compute_heuristic(const State &ancestor_state) {
    const State &state = convert_ancestor_state(ancestor_state);
    int h = abstraction.get_value(ancestor_state.get_unpacked_values());
    if (h == numeric_limits<int>::max()) {
        return DEAD_END;
    }
    return h;
}

static shared_ptr<Heuristic> _parse(OptionParser &parser) {
    parser.document_synopsis("Domain DomainMapping Heuristic", "TODO");
    parser.document_language_support("action costs", "supported");
    parser.document_language_support("conditional effects", "not supported");
    parser.document_language_support("axioms", "not supported");
    parser.document_property("admissible", "yes");
    parser.document_property("consistent", "yes");
    parser.document_property("safe", "yes");
    parser.document_property("preferred operators", "no");

    parser.add_option<shared_ptr<DomainAbstractionGenerator>>(
        "domain_abstraction_generator", "TODO");
    Heuristic::add_options_to_parser(parser);

    Options opts = parser.parse();
    if (parser.help_mode() || parser.dry_run()) {
        return nullptr;
    }
    return make_shared<DomainAbstractionHeuristic>(opts);
}

static Plugin<Evaluator> _plugin("domain_abstraction", _parse);
}
