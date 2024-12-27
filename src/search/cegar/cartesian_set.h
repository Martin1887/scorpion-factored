#ifndef CEGAR_CARTESIAN_SET_H
#define CEGAR_CARTESIAN_SET_H

#include "../algorithms/dynamic_bitset.h"

#include <ostream>
#include <vector>

namespace cegar {
using Bitset = dynamic_bitset::DynamicBitset<unsigned short>;

class BitsetIterator {
    // Store a pointer instead of a reference because iterators have to be copy assignable.
    const Bitset *bitset;
    int pos;

    void find_next() {
        while (pos < static_cast<int>(bitset->size()) && !bitset->test(pos)) {
            ++pos;
        }
    }
public:
    using iterator_category = std::input_iterator_tag;
    using value_type = int;
    using difference_type = int;
    using pointer = const value_type *;
    using reference = value_type;

    BitsetIterator(const Bitset &bitset, int pos)
        : bitset(&bitset), pos(pos) {
        find_next();
    }

    reference operator*() const {
        return pos;
    }

    BitsetIterator &operator++() {
        ++pos;
        find_next();
        return *this;
    }

    bool operator==(const BitsetIterator &other) const {
        assert(bitset == other.bitset);
        return pos == other.pos;
    }

    bool operator!=(const BitsetIterator &other) const {
        return !(*this == other);
    }
};

/*
  For each variable store a subset of its domain.

  The underlying data structure is a vector of bitsets.
*/
class CartesianSet {
    std::vector<Bitset> domain_subsets;

public:
    explicit CartesianSet(const std::vector<int> &domain_sizes);

    void add(int var, int value);
    void set_single_value(int var, int value);
    void remove(int var, int value);
    void add_all(int var);
    void remove_all(int var);

    bool test(int var, int value) const {
        return domain_subsets[var][value];
    }

    int count(int var) const;
    std::vector<int> get_values(int var) const;
    BitsetIterator begin_values(int var) const;
    BitsetIterator end_values(int var) const;
    bool intersects(const CartesianSet &other, int var) const;
    bool is_superset_of(const CartesianSet &other) const;

    friend std::ostream &operator<<(
        std::ostream &os, const CartesianSet &cartesian_set);

    int get_number_of_variables() const {
        return domain_subsets.size();
    }
};
}

#endif
