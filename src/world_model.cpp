#include "doctest/doctest.h"

#include "just/world_model.hpp"

namespace just
{

HistogramGrid::HistogramGrid(unsigned width, unsigned height)
{
    data_ = new uint8_t[width * height]();
    width_ = width;
    height_ = height;

    // Potential off by one issue when switching coordinate systems.
    // consider the 1x3 grid: 3/2 == 1 in integer division for the width, which properly gives an
    // x min and max value of +/- 1, resulting in the following grid (number line in this case):
    //
    // (-1) -- (0) -- (1)
    //
    // In a 1x4 grid example, however, we end up having to lose out on one cell on either the
    // min or max side because room for (0) is needed.
    // If we chose to keep the max value as-is, we end up with:
    //
    // (-1) -- (0) -- (1) -- (2)
    //
    // where x min = -(4 / 2 - 1) == -1, where `4` is the width in this case
    // note the equation for the max, x max = 4 / 2 == 2, still holds.

    x_max_ = width / 2;
    y_max_ = height / 2;

    x_min_ = width % 2 ? -x_max_ : -(x_max_ - 1);
    y_min_ = height % 2 ? -y_max_ : -(y_max_ - 1);
}

std::optional<uint8_t> HistogramGrid::at(int x, int y) const
{
    if (!within_bounds(x, y)) {
        return std::nullopt;
    }
    return { unsafe_at(x, y) };
}

bool HistogramGrid::within_bounds(int x, int y) const
{
    return (x >= x_min_ && x <= x_max_ && y >= y_min_ && y <= y_max_);
}

uint8_t& HistogramGrid::unsafe_at(int x, int y)
{
    unsigned col = x - x_min_;
    unsigned row = y - y_min_;

    return data_[row * width_ + col];
}

const uint8_t& HistogramGrid::unsafe_at(int x, int y) const
{
    unsigned col = x - x_min_;
    unsigned row = y - y_min_;

    return data_[row * width_ + col];
}

} // namespace just

TEST_CASE("HistogramGrid.within_bounds()") {
    just::HistogramGrid grid(10, 10);

    CHECK(grid.within_bounds(0,0));
    CHECK(grid.within_bounds(5,5));
    CHECK(grid.within_bounds(-4,-4));

    CHECK_FALSE(grid.within_bounds(10,10));
    CHECK_FALSE(grid.within_bounds(-10,-10));
    CHECK_FALSE(grid.within_bounds(6,6));
    CHECK_FALSE(grid.within_bounds(-5,-5));
}

TEST_CASE("HistogramGrid 3x3") {
    just::HistogramGrid grid(3, 3);

    CHECK(grid.at(-1,1) == 0);
    CHECK(grid.at(0,1) == 0);
    CHECK(grid.at(1,1) == 0);

    CHECK(grid.at(-1,0) == 0);
    CHECK(grid.at(0,0) == 0);
    CHECK(grid.at(1,0) == 0);

    CHECK(grid.at(-1,-1) == 0);
    CHECK(grid.at(0,-1) == 0);
    CHECK(grid.at(1,-1) == 0);

    CHECK(grid.at(2,2) == std::nullopt);
    CHECK(grid.at(2,0) == std::nullopt);
    CHECK(grid.at(0,2) == std::nullopt);
    CHECK(grid.at(-2,0) == std::nullopt);
    CHECK(grid.at(0,-2) == std::nullopt);
    CHECK(grid.at(-2,-2) == std::nullopt);
}

TEST_CASE("HistogramGrid 4x4") {
    just::HistogramGrid grid(4, 4);

    CHECK(grid.at(0,0) == 0);
    CHECK(grid.at(2,2) == 0);
    CHECK(grid.at(-1,-1) == 0);

    CHECK(grid.at(3,3) == std::nullopt);
    CHECK(grid.at(-2,-2) == std::nullopt);
}

TEST_CASE("Histogram Grid 10000x10001") {
    just::HistogramGrid grid(10000, 10001);

    CHECK(grid.at(0,0) == 0);
    CHECK(grid.at(5000,5000) == 0);
    CHECK(grid.at(-4999,-5000) == 0);

    CHECK(grid.at(1000000,1000000) == std::nullopt);
    CHECK(grid.at(-1000000,-1000000) == std::nullopt);
}
