#ifndef DOMAIN_ABSTRACTIONS_DOMAIN_ABSTRACTION_HEURISTIC_H
#define DOMAIN_ABSTRACTIONS_DOMAIN_ABSTRACTION_HEURISTIC_H

#include "domain_abstraction.h"

#include "../heuristic.h"

namespace domain_abstractions {
class DomainAbstractionHeuristic : public Heuristic {
private:
    DomainAbstraction abstraction;

protected:
    virtual int compute_heuristic(const State &ancestor_state) override;

public:
    explicit DomainAbstractionHeuristic(const options::Options &opts);
    virtual ~DomainAbstractionHeuristic() = default;
};
}

#endif
