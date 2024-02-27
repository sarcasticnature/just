#include <cmath>
#include <algorithm>

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

bool HistogramGrid::add_percept(int x0, int y0, float theta, float distance)
{
    if (!within_bounds(x0, y0)) {
        return false;
    }

    int x1 = std::round(distance * std::cos(theta));
    int y1 = std::round(distance * std::sin(theta));

    // TODO: double bounds checks... not sure it really matters though
    if (!within_bounds(x1, y1)) {
        float m = std::tan(theta);
        float b = y0 - m * x0;

        if (x1 < x_min_ || x1 > x_max_) {
            x1 = std::clamp(x1, x_min_, x_max_);
            y1 = std::trunc(m * (double)x1 + b);
        }

        // Note y may still be out of bounds even after clipping x, so this should not be an if else
        if (y1 < y_min_ || y1 > y_max_) {
            y1 = std::clamp(y1, y_min_, y_max_);
            x1 = std::trunc(((double)y1 - b) / m);
        }
    }

    // Bresenham's line algorithm
    // https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm#All_cases
    int dx = std::abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int e2;

    while (x0 != x1 || y0 != y1) {
        decrement_cell(x0, y0);
        e2 = 2 * err;
        if (e2 >= dy) {
            err = err + dy;
            // Note: clamping this value as there is an edge case where it could go over max
            // when the end point is at the limit(s) of the grid
            x0 = std::clamp(x0 + sx, x_min_, x_max_);
        }
        if (e2 <= dx) {
            err = err + dx;
            // Same as above
            y0 = std::clamp(y0 + sy, y_min_, y_max_);
        }
    }
    increment_cell(x0, y0);

    return true;
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

void HistogramGrid::increment_cell(int x, int y)
{
    uint8_t& cell = unsafe_at(x, y);
    cell = std::clamp(static_cast<uint8_t>(cell + CV_INC), CV_MIN, CV_MAX);
}

void HistogramGrid::decrement_cell(int x, int y)
{
    uint8_t& cell = unsafe_at(x, y);
    if (static_cast<int>(cell) - static_cast<int>(CV_DEC) < 0) {
        cell = 0;
    } else {
        cell -= CV_DEC;
    }
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

    CHECK(grid.at(-1,1).value() == 0);
    CHECK(grid.at(0,1).value() == 0);
    CHECK(grid.at(1,1).value() == 0);

    CHECK(grid.at(-1,0).value() == 0);
    CHECK(grid.at(0,0).value() == 0);
    CHECK(grid.at(1,0).value() == 0);

    CHECK(grid.at(-1,-1).value() == 0);
    CHECK(grid.at(0,-1).value() == 0);
    CHECK(grid.at(1,-1).value() == 0);

    CHECK(grid.at(2,2) == std::nullopt);
    CHECK(grid.at(2,0) == std::nullopt);
    CHECK(grid.at(0,2) == std::nullopt);
    CHECK(grid.at(-2,0) == std::nullopt);
    CHECK(grid.at(0,-2) == std::nullopt);
    CHECK(grid.at(-2,-2) == std::nullopt);
}

TEST_CASE("HistogramGrid 4x4") {
    just::HistogramGrid grid(4, 4);

    CHECK(grid.at(0,0).value() == 0);
    CHECK(grid.at(2,2).value() == 0);
    CHECK(grid.at(-1,-1).value() == 0);

    CHECK(grid.at(3,3) == std::nullopt);
    CHECK(grid.at(-2,-2) == std::nullopt);
}

TEST_CASE("HistogramGrid 10000x10001") {
    just::HistogramGrid grid(10000, 10001);

    CHECK(grid.at(0,0).value() == 0);
    CHECK(grid.at(5000,5000).value() == 0);
    CHECK(grid.at(-4999,-5000).value() == 0);

    CHECK(grid.at(1000000,1000000) == std::nullopt);
    CHECK(grid.at(-1000000,-1000000) == std::nullopt);
}

TEST_CASE("HistogramGrid.add_percept") {
    SUBCASE("Add percept cardinal directions") {
        just::HistogramGrid grid(10,10);

        REQUIRE(grid.add_percept(0, 0, 0.0, 3.0));
        CHECK(grid.at(3,0).value() == just::HistogramGrid::CV_INC);

        REQUIRE(grid.add_percept(0, 0, M_PI / 4, 3.0));
        CHECK(grid.at(2,2).value() == just::HistogramGrid::CV_INC);

        REQUIRE(grid.add_percept(0, 0, M_PI / 2, 3.0));
        CHECK(grid.at(0,3).value() == just::HistogramGrid::CV_INC);

        REQUIRE(grid.add_percept(0, 0, 3 * M_PI / 4, 3.0));
        CHECK(grid.at(-2,2).value() == just::HistogramGrid::CV_INC);

        REQUIRE(grid.add_percept(0, 0, M_PI, 3.0));
        CHECK(grid.at(-3,0).value() == just::HistogramGrid::CV_INC);

        REQUIRE(grid.add_percept(0, 0, -3 * M_PI / 4, 3.0));
        CHECK(grid.at(-2,-2).value() == just::HistogramGrid::CV_INC);

        REQUIRE(grid.add_percept(0, 0, -M_PI / 2, 3.0));
        CHECK(grid.at(0,-3).value() == just::HistogramGrid::CV_INC);

        REQUIRE(grid.add_percept(0, 0, -M_PI / 4, 3.0));
        CHECK(grid.at(2,-2).value() == just::HistogramGrid::CV_INC);
    }

    SUBCASE("Add percept odd angles") {
        just::HistogramGrid grid(10,10);

        // Note: hand calculations were performed to obtain the 'correct' cell coordinates
        // for these cases
        REQUIRE(grid.add_percept(0, 0, M_PI / 12, 4.0));
        CHECK(grid.at(4,1).value() == just::HistogramGrid::CV_INC);

        REQUIRE(grid.add_percept(0, 0, M_PI / 6, 4.0));
        CHECK(grid.at(3,2).value() == just::HistogramGrid::CV_INC);

        // Now we verify that the ray is following the correct path (in addition to its destination)
        // First, we will breadcrumb cells along the path (working backwards)

        // To increase the CV of the cell at (1,2), we need a magnitude of sqrt(5)
        // and a theta of acos(1/sqrt(5))
        // I solved for this analytically, but the math is a bit too long to type out here.
        // Was it necessary to find the exact values?
        // No, an approximation/emperically determined value would have been fine... but /shrug
        REQUIRE(grid.add_percept(0, 0, 1.107, 2.236));
        REQUIRE(grid.at(1,2).value() == just::HistogramGrid::CV_INC);

        // To increase the CV of the cell at (1,1), we use the unit circle to find
        // a magnitue of 2/sqrt(2) and a theta of pi/4
        REQUIRE(grid.add_percept(0, 0, M_PI / 4, 1.414));
        REQUIRE(grid.at(1,1).value()== just::HistogramGrid::CV_INC);

        // Add the ray we actually want to check
        REQUIRE(grid.add_percept(0, 0, M_PI / 3, 4.0));
        REQUIRE(grid.at(2,3).value() == just::HistogramGrid::CV_INC);
        // The other two cells should have been decremented, if the ray followed the 'correct'
        // path
        REQUIRE(grid.at(1,2).value() == just::HistogramGrid::CV_INC - 1);
        REQUIRE(grid.at(1,1).value() == just::HistogramGrid::CV_INC - 1);

        // Verify that no other cells were harmed in the making of this film
        REQUIRE(grid.at(0,0).value() == 0);
        REQUIRE(grid.at(0,1).value() == 0);
        REQUIRE(grid.at(0,2).value() == 0);
        REQUIRE(grid.at(1,0).value() == 0);
        REQUIRE(grid.at(1,3).value() == 0);
        REQUIRE(grid.at(2,1).value() == 0);
        REQUIRE(grid.at(2,2).value() == 0);
        REQUIRE(grid.at(3,0).value() == 0);
        REQUIRE(grid.at(3,1).value() == 0);
    }

    SUBCASE("Add percepts until max CV") {
        just::HistogramGrid grid(10, 10);

        // Load up a cell to max CV
        for (int i = 1; i <= 5; ++i) {
            grid.add_percept(0, 0, 0.0, 3.0);
            REQUIRE(grid.at(0,0).value() == 0);
            REQUIRE(grid.at(1,0).value() == 0);
            REQUIRE(grid.at(2,0).value() == 0);
            REQUIRE(grid.at(3,0).value() == just::HistogramGrid::CV_INC * i);
            REQUIRE(grid.at(4,0).value() == 0);
        }

        // Verify that the cell doesn't go over max CV
        grid.add_percept(0, 0, 0.0, 3.0);

        REQUIRE(grid.at(0,0).value() == 0);
        REQUIRE(grid.at(1,0).value() == 0);
        REQUIRE(grid.at(2,0).value() == 0);
        REQUIRE(grid.at(3,0).value() == just::HistogramGrid::CV_MAX);
        REQUIRE(grid.at(4,0).value() == 0);
    }

    SUBCASE("Add percepts until min CV") {
        just::HistogramGrid grid(10, 10);

        // TODO: use the CV constants defined in the class instead of hardcoding values
        // Note that I am way too lazy to fix right now, esp. given I have no intention of
        // changing them anytime soon

        // Load up the grid w/ a max CV cell
        for (int i = 0; i < 5; ++i) {
            grid.add_percept(0, 0, 0.0, 3.0);
        }
        // Add another cell (to be decremented shortly)
        grid.add_percept(0, 0, 0.0, 2.0);

        // Add a percept further away to decrement the other cells
        grid.add_percept(0, 0, 0.0, 5.0);

        REQUIRE(grid.at(2,0).value() == 2);
        REQUIRE(grid.at(3,0).value() == 14);
        REQUIRE(grid.at(5,0).value() == 3);

        grid.add_percept(0, 0, 0.0, 5.0);

        REQUIRE(grid.at(2,0).value() == 1);
        REQUIRE(grid.at(3,0).value() == 13);
        REQUIRE(grid.at(5,0).value() == 6);

        grid.add_percept(0, 0, 0.0, 5.0);

        REQUIRE(grid.at(2,0).value() == 0);
        REQUIRE(grid.at(3,0).value() == 12);
        REQUIRE(grid.at(5,0).value() == 9);

        grid.add_percept(0, 0, 0.0, 5.0);

        REQUIRE(grid.at(2,0).value() == 0);
        REQUIRE(grid.at(3,0).value() == 11);
        REQUIRE(grid.at(5,0).value() == 12);
    }
}
