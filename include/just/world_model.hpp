#ifndef __JUST__WORLD_MODEL_HPP__
#define __JUST__WORLD_MODEL_HPP__

#include <cstdint>
#include <optional>

namespace just
{

class HistogramGrid
{
public:
    // Constants used by the alg
    static constexpr uint8_t CV_MIN = 0;    // certainty value minimum
    static constexpr uint8_t CV_MAX = 15;   // certainty value maximum
    static constexpr uint8_t CV_INC = 3;    // certainty value increment magnitude
    static constexpr uint8_t CV_DEC = 1;    // certainty value decrement magnitude

    // ctor/dtor
    explicit HistogramGrid(unsigned width, unsigned height);
    ~HistogramGrid() { delete data_; }

    // Access to the underlying array and it's dimensions
    // This is mostly useful for visualization(s) using raylib
    const uint8_t* data() const { return data_; };
    unsigned witdth() const { return width_; };
    unsigned height() const { return height_; };

    // Public facing, bounds checked element access (cartesian coords).
    // Returns nullopt if the requested x/y is out of bounds.
    std::optional<uint8_t> at(int x, int y) const;

    // Checks if given cartesian coordinates are within possible values of the array
    inline bool within_bounds(int x, int y) const;

    // Ingest information from a new percept ('sensor' distance measurment) to the grid
    // returns false if the percept couldn't be processed, true otherwise
    bool add_percept(int x0, int y0, float theta, float distance);

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

    inline void increment_cell(int x, int y);
    inline void decrement_cell(int x, int y);
};

} // namespace just

#endif // __JUST__WORLD_MODEL_HPP__
