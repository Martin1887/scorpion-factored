#include "domain_abstraction.h"

#include "../utils/logging.h"

using namespace std;

namespace domain_abstractions {

int DomainAbstraction::hash_index(const vector<int> &state) const {
    int index = 0;
    for (size_t i = 0; i < state.size(); ++i) {
        if (!domain_mapping[i].empty()) {
            index += hash_multipliers[i] * domain_mapping[i][state[i]];
        }
    }
    return index;
}

int DomainAbstraction::get_value(const vector<int> &state) const {
    return distances[hash_index(state)];
}

void DomainAbstraction::dump(utils::LogProxy &log) const {
    if (log.is_at_least_debug()) {
        log << "domain mapping: " << domain_mapping << endl;
        //log << "hash multipliers: " << hash_multipliers << endl;
        //log << "distances: " << distances << endl;
        log << "Domain abstraction has " << distances.size() << " states."
            << endl;
    }
}

}
