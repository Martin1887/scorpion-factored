#ifndef DOMAIN_ABSTRACTIONS_DOMAIN_ABSTRACTION_COLLECTION_GENERATOR_MULTIPLE_RANDOM_H
#define DOMAIN_ABSTRACTIONS_DOMAIN_ABSTRACTION_COLLECTION_GENERATOR_MULTIPLE_RANDOM_H

#include "domain_abstraction_collection_generator_multiple.h"

namespace domain_abstractions {
class DomainAbstractionCollectionGeneratorMultipleRandom : public DomainAbstractionCollectionGeneratorMultiple {
    const bool bidirectional;
    std::vector<std::vector<int>> cg_neighbors;

    virtual std::string id() const override;
    virtual void initialize(const TaskProxy &task) override;
    virtual DomainAbstraction compute_abstraction(
        int max_abstraction_size,
        double max_time,
        const std::shared_ptr<utils::RandomNumberGenerator> &rng,
        const TaskProxy &task_proxy,
        const FactPair &goal,
        std::unordered_set<int> &&init_split_var_ids,
        std::unordered_set<int> &&blacklisted_variables) override;
public:
    explicit DomainAbstractionCollectionGeneratorMultipleRandom(options::Options &opts);
};
}

#endif
