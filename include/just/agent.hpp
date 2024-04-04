#ifndef __JUST__AGENT_HPP__
#define __JUST__AGENT_HPP__

#include <cmath>
#include <memory>

#include "box2d/box2d.h"
#include "toml++/toml.hpp"
#include "highfive/highfive.hpp"

#include "world_model.hpp"
#include "sensor.hpp"

namespace just
{

class Agent
{
public:
    Agent(const toml::table& config, b2World* world);
    virtual ~Agent();

    virtual void step(float delta_t) = 0;

    const b2Body* get_body() { return body_; }

protected:
    b2Body* body_;
};

class PatrolAgent : public Agent
{
public:
    PatrolAgent(const toml::table& config, b2World* world);

    void step(float delta_t) override;
private:
    b2Vec2 a_;
    b2Vec2 b_;
    float tolerance_;
    float speed_;
    bool reverse_{false};
};

class VFHAgent : public Agent
{
public:
    // Constants for the VFH equations
    static constexpr size_t WINDOW_SIZE = 30;
    static constexpr size_t WINDOW_SIZE_SQUARED = WINDOW_SIZE * WINDOW_SIZE;
    static constexpr int ALPHA_DEG = 5;    // TODO: reduce to 5 after testing
    static constexpr float ALPHA = ALPHA_DEG * M_PI / 180.0;
    static constexpr int K = 360 / ALPHA_DEG;   // number of sectors

    static constexpr float B = 500.0;
    // NOTE: difference from the paper here. Their equation for d_max, and thus 'a', is only
    // accurate for *odd* window sizes and will cause the magnitude of the obstacle vector to
    // be negative at the extremes of the window for *even* window sizes.
    // This causes almost imperceptible errors, but they are errors none the less.
    //
    // Using WINDOW_SIZE (not WINDOW_SIZE - 1) causes 'a' to be (ever so) slightly larger
    // than it needs to be for odd window sizes, but alleviates the more problematic
    // negative magnitude issue in all cases
    //
    // NOTE: std::sqrt isn't constexpr until C++26, using a hardcoded sqrt(2) instead
    static constexpr float A = B * 1.414213562 * WINDOW_SIZE / 2.0;
    static constexpr int L = 5;         // polar histogram smoothing, 5 in paper

    static constexpr size_t S_MAX = 18; // selected valley size, 18 in the paper


    VFHAgent(const toml::table& config, b2World* world);

    void step(float delta_t) override;

private:
    // Adapter class to make a subgrid more ergonomic to use
    // TODO: not sure if this is the right place for this to live
    class SubgridAdapter
    {
    public:
        SubgridAdapter() = delete;
        explicit SubgridAdapter(std::array<uint8_t, WINDOW_SIZE_SQUARED>&& arr)
            : arr_(std::move(arr)) {}

        // NOTE: no bounds checking is done and an exception may be thrown
        uint8_t at(int x, int y) {
            return arr_[y * WINDOW_SIZE + x];
        }
    private:
        std::array<uint8_t, WINDOW_SIZE_SQUARED> arr_;
    };

    class Logger
    {
    public:
        Logger(const std::string& filename, unsigned grid_size);
        void log_polar_histogram(const std::array<float, K>& polar_histogram);
        void log_window(const std::array<uint8_t, WINDOW_SIZE_SQUARED>& window);
        void log_full_grid(const HistogramGrid& grid);
        void log_motion(float angle, float speed, float x, float y);
    private:
        std::unique_ptr<HighFive::File> file_;
        size_t steering_idx_{0};
    };

    struct SteeringCommand
    {
        float angle;
        float speed;
    };

    HistogramGrid grid_;
    UltrasonicArray sensor_;
    std::unique_ptr<Logger> logger_;
    b2Vec2 goal_;
    float valley_threshold_;
    float v_max_;

    void sense();
    std::optional<std::array<float, K>> create_polar_histogram();
    SteeringCommand compute_steering(const std::array<float, K>& polar_histogram);
};

} // namespace just

#endif // __JUST__AGENT_HPP__
