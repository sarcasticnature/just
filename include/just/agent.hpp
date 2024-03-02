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

    virtual void step() = 0;

protected:
    b2Body* body_;
};

class VFHAgent : public Agent
{
public:
    VFHAgent(const toml::table& config, b2World* world);
    ~VFHAgent() override {}

    void step() override;

private:
    HistogramGrid grid_;
    UltrasonicArray sensor_;
};

} // namespace just

#endif // __JUST__AGENT_HPP__
