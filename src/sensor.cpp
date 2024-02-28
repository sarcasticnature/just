#include <cmath>
#include <algorithm>
#include <memory>

#include "doctest/doctest.h"

#include "just/sensor.hpp"

namespace just
{

UltrasonicArray::UltrasonicArray(unsigned sensor_cnt, float max_range, b2Body* body)
{
    body_ = body;

    beams_.resize(sensor_cnt);
    Beam beam;

    for (unsigned i = 0; i < sensor_cnt; ++i) {
        beam.relative_angle = 2.0 * M_PI * static_cast<float>(i) / static_cast<float>(sensor_cnt);
        beam.local_endpoint.x = std::cos(beam.relative_angle) * max_range;
        beam.local_endpoint.y = std::sin(beam.relative_angle) * max_range;

        beams_.at(i) = beam;
    }
}

UltrasonicArray::SensorReading UltrasonicArray::sense_one()
{
    SensorReading reading;
    const Beam& beam = beams_.at(active_beam_idx_);
    active_beam_idx_ = (active_beam_idx_ + 1) % beams_.size();

    b2Vec2 world_endpoint = body_->GetWorldPoint(beam.local_endpoint);
    RaycastCb cb(body_);
    b2World* world = body_->GetWorld();

    world->RayCast(&cb, body_->GetPosition(), world_endpoint);

    reading.distance = cb.min_distance != std::numeric_limits<float>::max() ? cb.min_distance : 0.0;
    reading.angle = beam.relative_angle;

    return reading;
}

std::vector<UltrasonicArray::SensorReading> UltrasonicArray::sense_all()
{
    std::vector<SensorReading> readings;
    readings.resize(beams_.size());
    for (unsigned i = 0; i < beams_.size(); ++i) {
        readings.at(i) = sense_one();
    }
    return readings;
}

float UltrasonicArray::RaycastCb::ReportFixture(b2Fixture* fixture,
                                          const b2Vec2& world_point,
                                          const b2Vec2& normal,
                                          float fraction)
{
    (void)normal;
    (void)fraction;

    if (fixture->GetBody() != body_) {
        b2Vec2 local_point = body_->GetLocalPoint(world_point);
        min_distance = std::min(min_distance, local_point.Length());
    }

    return 1.0;
}

} // namespace just

TEST_CASE("UltrasonicArray sensor tests") {
    // Set up the world
    auto world = std::make_unique<b2World>(b2Vec2{0.0, 0.0});
    b2BodyDef dummy_body_def;
    dummy_body_def.type = b2_staticBody;
    dummy_body_def.position.Set(0.0, 0.0);
    dummy_body_def.angle = 0.0;

    b2Body* dummy_body = world->CreateBody(&dummy_body_def);
    b2CircleShape dummy_shape;
    dummy_shape.m_p.Set(0.0, 0.0);
    dummy_shape.m_radius = 0.1;

    b2FixtureDef dummy_fixture_def;
    dummy_fixture_def.shape = &dummy_shape;
    dummy_fixture_def.density = 1.0;
    dummy_body->CreateFixture(&dummy_fixture_def);

    SUBCASE("Single sensor") {
        just::UltrasonicArray sensor(1, 5.0, dummy_body);

        auto reading = sensor.sense_one();
        REQUIRE(reading.angle == 0.0);
        REQUIRE(reading.distance == 0.0);

        reading = sensor.sense_one();
        REQUIRE(reading.angle == 0.0);
        REQUIRE(reading.distance == 0.0);

        auto reading_vec = sensor.sense_all();
        REQUIRE(reading_vec.size() == 1);
        REQUIRE(reading_vec.at(0).angle == 0.0);
        REQUIRE(reading_vec.at(0).distance == 0.0);
    }

    SUBCASE("Multiple sensors") {
        just::UltrasonicArray sensor(10, 1.0, dummy_body);

        just::UltrasonicArray::SensorReading reading;
        for (int i = 0; i < 10; ++i) {
            reading = sensor.sense_one();
            REQUIRE(reading.angle == doctest::Approx(i * 2.0 * M_PI / 10.0));
            REQUIRE(reading.distance == 0.0);
        }

        auto reading_vec = sensor.sense_all();
        REQUIRE(reading_vec.size() == 10);
        for (int i = 0; i < 10; ++i) {
            REQUIRE(reading_vec.at(i).angle == doctest::Approx(i * 2.0 * M_PI / 10.0));
            REQUIRE(reading_vec.at(i).distance == 0.0);
        }
    }

    // Create actual obstacles to sense
    // Common stuff
    b2CircleShape obstacle_shape;
    obstacle_shape.m_p.Set(0.0, 0.0);
    obstacle_shape.m_radius = 1.0;

    b2FixtureDef obstacle_fixture_def;
    obstacle_fixture_def.shape = &obstacle_shape;
    obstacle_fixture_def.density = 1.0;

    // Obstacle #1 -- should be detected at (1,0)
    b2BodyDef obstacle1_body_def;
    obstacle1_body_def.type = b2_staticBody;
    obstacle1_body_def.position.Set(2.0, 0.0);
    obstacle1_body_def.angle = 0.0;
    b2Body* obstacle1_body = world->CreateBody(&obstacle1_body_def);
    obstacle1_body->CreateFixture(&obstacle_fixture_def);

    // Obstacle #2 -- should be detected at (0,5)
    b2BodyDef obstacle2_body_def;
    obstacle2_body_def.type = b2_staticBody;
    obstacle2_body_def.position.Set(0.0, 6.0);
    obstacle2_body_def.angle = 0.0;
    b2Body* obstacle2_body = world->CreateBody(&obstacle2_body_def);
    obstacle2_body->CreateFixture(&obstacle_fixture_def);

    // Obstacle #3 -- should NOT be detected (outside of the intended max range of 10)
    b2BodyDef obstacle3_body_def;
    obstacle3_body_def.type = b2_staticBody;
    obstacle3_body_def.position.Set(-11.001, 0.0);
    obstacle3_body_def.angle = 0.0;
    b2Body* obstacle3_body = world->CreateBody(&obstacle3_body_def);
    obstacle3_body->CreateFixture(&obstacle_fixture_def);

    SUBCASE("Test raycasts") {
        just::UltrasonicArray sensor(4, 10.0, dummy_body);

        auto reading = sensor.sense_one();
        REQUIRE(reading.angle == 0.0);
        CHECK(reading.distance == doctest::Approx(1.0));

        reading = sensor.sense_one();
        REQUIRE(reading.angle == doctest::Approx(M_PI / 2));
        CHECK(reading.distance == doctest::Approx(5.0));

        reading = sensor.sense_one();
        REQUIRE(reading.angle == doctest::Approx(M_PI));
        CHECK(reading.distance == 0.0);

        reading = sensor.sense_one();
        REQUIRE(reading.angle == doctest::Approx(3 * M_PI / 2));
        CHECK(reading.distance == 0.0);
    }
}
