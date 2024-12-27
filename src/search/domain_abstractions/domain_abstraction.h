#ifndef DOMAIN_ABSTRACTION_DOMAIN_ABSTRACTION_H
#define DOMAIN_ABSTRACTION_DOMAIN_ABSTRACTION_H

#include "types.h"

#include "../operator_id.h"

#include <vector>

namespace utils {
class LogProxy;
}

namespace domain_abstractions {
class DomainAbstraction {
    DomainMapping domain_mapping;
    std::vector<int> hash_multipliers;
    std::vector<int> distances;
    // TODO: get rid of this here and return it from the factory optionally.
    std::vector<std::vector<OperatorID>> wildcard_plan;
    std::vector<int> state_trace;

    int hash_index(const std::vector<int> &state) const;

public:
    DomainAbstraction(DomainMapping &&domain_mapping,
                      std::vector<int> &&hash_multipliers,
                      std::vector<int> &&distances,
                      std::vector<std::vector<OperatorID>> &&wildcard_plan,
                      std::vector<int> &&state_trace)
        : domain_mapping(std::move(domain_mapping)),
          hash_multipliers(std::move(hash_multipliers)),
          distances(std::move(distances)),
          wildcard_plan(std::move(wildcard_plan)),
          state_trace(std::move(state_trace)) {
    }

    const DomainMapping &get_domain_mapping() const {
        return domain_mapping;
    }

    const DomainMapping &&extract_domain_mapping() {
        return std::move(domain_mapping);
    }

    int get_value(const std::vector<int> &state) const;

    std::vector<std::vector<OperatorID>> get_plan() const {
        return wildcard_plan;
    }

    std::vector<int> get_trace() const {
        return state_trace;
    }

    int get_abstract_state_hash(const std::vector<int> &state) const {
        return hash_index(state);
    }

    int get_hash_multiplier(int var_id) const {
        return hash_multipliers[var_id];
    }

    int size() const {
        return distances.size();
    }

    void dump(utils::LogProxy &log) const;
};
}

#endif
