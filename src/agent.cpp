#include <string_view>
#include <exception>

#include "just/agent.hpp"

namespace just
{

Agent::Agent(const toml::table& config, b2World* world)
{
    b2BodyDef body_def;
    body_def.type = b2_dynamicBody;
    body_def.position.Set(config["x"].value_or(0.0), config["y"].value_or(0.0));
    body_def.angle = config["theta"].value_or(0.0);
    body_ = world->CreateBody(&body_def);

    std::string_view shape = config["shape"].value_or("circle");
    if (shape == "circle") {
        b2CircleShape shape;
        shape.m_radius = config["radius"].value_or(1.0);

        b2FixtureDef fixture_def;
        fixture_def.shape = &shape;
        fixture_def.density = config["density"].value_or(1.0);
        body_->CreateFixture(&fixture_def);

    } else if (shape == "box") {
        b2PolygonShape shape;
        shape.SetAsBox(config["width"].value_or(1.0) / 2, config["height"].value_or(1.0) / 2);

        b2FixtureDef fixture_def;
        fixture_def.shape = &shape;
        fixture_def.density = config["density"].value_or(1.0);
        body_->CreateFixture(&fixture_def);
    } else {
        throw std::runtime_error("Agent constructed with invalid 'shape' field in TOML config");
    }
}

Agent::~Agent()
{
    b2World* world = body_->GetWorld();
    world->DestroyBody(body_);
}

PatrolAgent::PatrolAgent(const toml::table& config, b2World* world)
    : Agent(config, world)
{
    a_ = {config["x"].value_or(0.0f), config["y"].value_or(0.0f)};
    b_ = {config["waypoint"]["x"].value_or(0.0f), config["waypoint"]["y"].value_or(0.0f)};
    speed_ = config["speed"].value_or(1.0f);
    tolerance_ = config["goal_tolerance"].value_or(0.5f);
}

void PatrolAgent::step(float delta_t)
{
    (void)delta_t;
    b2Vec2 goal;

    // TODO: this is ugly, whatever
    if (reverse_) {
        goal = body_->GetLocalPoint(a_);
        if (goal.Length() < tolerance_) {
            reverse_ = !reverse_;
            goal = body_->GetLocalPoint(b_);
        }
    } else {
        goal = body_->GetLocalPoint(b_);
        if (goal.Length() < tolerance_) {
            reverse_ = !reverse_;
            goal = body_->GetLocalPoint(a_);
        }
    }

    goal.Normalize();
    goal *= speed_;

    body_->SetLinearVelocity(goal);
    body_->SetAngularVelocity(0.0f);
}


VFHAgent::VFHAgent(const toml::table& config, b2World* world)
    : Agent(config, world),
      grid_(*config["grid"]["width"].value<unsigned>(), *config["grid"]["width"].value<unsigned>()),
      sensor_(*config["sensor"]["count"].value<unsigned>(),
              *config["sensor"]["range"].value<float>(),
              body_)
{
}

void VFHAgent::step(float delta_t)
{
    // TODO: use delta_t to simulate firing the "ultrasonic sensors" in series at a fixed interval,
    // to make the simulation more 'realistic'.
    // This mimics the real deal more closely, as crosstalk prevents firing all sensors at the same
    // time. It also matches the case of a rotating LIDAR or RADAR, as an added bonus.
    (void)delta_t;
    auto sensor_readings = sensor_.sense_all();

    b2Vec2 position = body_->GetPosition();
    int x = std::round(position.x);
    int y = std::round(position.y);

    for (const auto& [distance, angle] : sensor_readings) {
        grid_.add_percept(x, y, angle, distance);
    }
}

} // namespace just
