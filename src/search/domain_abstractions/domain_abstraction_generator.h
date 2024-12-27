#ifndef DOMAIN_ABSTRACTIONS_DOMAIN_ABSTRACTION_GENERATOR_H
#define DOMAIN_ABSTRACTIONS_DOMAIN_ABSTRACTION_GENERATOR_H

#include "domain_abstraction.h"

#include "../utils/logging.h"

class TaskProxy;

namespace utils {
class RandomNumberGenerator;
}

namespace domain_abstractions {

class DomainAbstractionGenerator {
protected:
    mutable utils::LogProxy log;
    std::shared_ptr<utils::RandomNumberGenerator> rng;

public:
    explicit DomainAbstractionGenerator(const options::Options &opts);
    virtual ~DomainAbstractionGenerator() = default;

    virtual DomainAbstraction build_abstraction(
        const TaskProxy &task_proxy) = 0;
};

extern void add_domain_abstraction_generator_options_to_parser(
    options::OptionParser &parser);

class DomainAbstractionGeneratorIdentity : public DomainAbstractionGenerator {
public:
    explicit DomainAbstractionGeneratorIdentity(const options::Options &opts);

    DomainAbstraction build_abstraction(const TaskProxy &task_proxy) override;
};
}

#endif
