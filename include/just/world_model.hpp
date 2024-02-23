#ifndef __JUST__WORLD_MODEL_HPP__
#define __JUST__WORLD_MODEL_HPP__

#include <cstdint>
#include <optional>

namespace just
{

class HistogramGrid
{
public:
    // ctor/dtor
    explicit HistogramGrid(unsigned width, unsigned height);
    ~HistogramGrid() { delete data_; }

    // Access to the underlying array.  Not sure what this would be used for as of yet (testing?)
    //const uint8_t* data() const { return data_; };

    // Public facing, bounds checked element access (cartesian coords).
    // Returns nullopt if the requested x/y is out of bounds.
    std::optional<uint8_t> at(int x, int y) const;

    // Checks if given cartesian coordinates are within possible values of the array
    inline bool within_bounds(int x, int y) const;

private:
    // array data/info
    uint8_t* data_;
    unsigned width_;
    unsigned height_;

    // precomputed min/max (cartesian space)
    int x_max_;
    int x_min_;
    int y_max_;
    int y_min_;

    // Looks up a value in the internal array, using cartesian coords as the reference system.
    // DOES NOT do any bounds checking, to allow a single bounds check (before fn calls)
    // for multiple array accesses.
    uint8_t& unsafe_at(int x, int y);
    const uint8_t& unsafe_at(int x, int y) const;
};

} // namespace just

#endif // __JUST__WORLD_MODEL_HPP__
