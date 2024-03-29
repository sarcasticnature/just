#ifndef __JUST__WORLD_MODEL_HPP__
#define __JUST__WORLD_MODEL_HPP__

#include <cstdint>
#include <optional>
#include <array>

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
    unsigned width() const { return width_; };
    unsigned height() const { return height_; };

    // Public facing, bounds checked element access (cartesian coords).
    // Returns nullopt if the requested x/y is out of bounds.
    std::optional<uint8_t> at(int x, int y) const;

    // Checks if given cartesian coordinates are within possible values of the array
    inline bool within_bounds(int x, int y) const;

    // Ingest information from a new percept ('sensor' distance measurment) to the grid
    // Takes in the position of the base, the angle and distance of the reading,
    // and whether or not an obstacle was detected at that range
    // returns false if the percept couldn't be processed, true otherwise
    bool add_percept(int x0, int y0, float theta, float distance, bool detected);

    // Get a subset of the grid
    template <size_t W, size_t H>
    std::optional<std::array<uint8_t, W * H>> subgrid(int x, int y) const;

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

template <size_t W, size_t H>
std::optional<std::array<uint8_t, W * H>> HistogramGrid::subgrid(int x, int y) const
{
    int sub_x_max = x + W / 2;
    int sub_y_max = y + H / 2;

    // See the constructor for a summary of why this is necessary
    int sub_x_min = W % 2 ? x - W / 2 : x - (W / 2 - 1);
    int sub_y_min = H % 2 ? y - H / 2 : y - (H / 2 - 1);

    if (sub_x_max > x_max_ || sub_y_max > y_max_ || sub_x_min < x_min_ || sub_y_min < y_min_) {
        return std::nullopt;
    }

    std::array<uint8_t, W * H> subarray;
    size_t idx = 0;
    for (int y = sub_y_min; y <= sub_y_max; ++y) {
        for (int x = sub_x_min; x <= sub_x_max; ++x) {
            subarray.at(idx) = unsafe_at(x, y);
            ++idx;
        }
    }

    return { subarray };
}


} // namespace just

#endif // __JUST__WORLD_MODEL_HPP__
