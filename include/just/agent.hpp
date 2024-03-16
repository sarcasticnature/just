#ifndef __JUST__AGENT_HPP__
#define __JUST__AGENT_HPP__

#include "box2d/box2d.h"
#include "toml++/toml.hpp"

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
    static constexpr size_t WINDOW_SIZE = 100;
    static constexpr size_t WINDOW_SIZE_SQUARED = WINDOW_SIZE * WINDOW_SIZE;

    // Adapter class to make a subgrid more ergonomic to use
    class SubgridAdapter {
    public:
        SubgridAdapter() = delete;
        explicit SubgridAdapter(std::array<uint8_t, WINDOW_SIZE_SQUARED>&& arr)
            : arr_(std::move(arr)) {}

        // NOTE: no bounds checking is done and an exception may be thrown
        uint8_t at(int x, int y) {
            return arr_.at(y * WINDOW_SIZE + x);
        }
    private:
        std::array<uint8_t, WINDOW_SIZE_SQUARED> arr_;
    };

    VFHAgent(const toml::table& config, b2World* world);

    void step(float delta_t) override;

private:
    HistogramGrid grid_;
    UltrasonicArray sensor_;
};

} // namespace just

#endif // __JUST__AGENT_HPP__
