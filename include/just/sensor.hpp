#ifndef __JUST__SENSOR_HPP__
#define __JUST__SENSOR_HPP__

#include <vector>
#include <limits>

#include "box2d/box2d.h"

namespace just
{

class UltrasonicArray
{
public:
    struct SensorReading
    {
        float distance{0.0};
        float angle{0.0};
    };

    UltrasonicArray(unsigned sensor_cnt, float max_range, b2Body* body);

    SensorReading sense_one();
    std::vector<SensorReading> sense_all();

    float max_range()
    {
        return beams_[0].local_endpoint.x;
    }

private:
    struct Beam
    {
        float relative_angle;
        b2Vec2 local_endpoint;
    };

    class RaycastCb : public b2RayCastCallback
    {
    public:
        RaycastCb(b2Body* body) : body_(body) {}

        float ReportFixture(b2Fixture* fixture,
                            const b2Vec2& world_point,
                            const b2Vec2& normal,
                            float fraction);

        float min_distance{std::numeric_limits<float>::max()};
    private:
        b2Body* body_;
    };

    std::vector<Beam> beams_;
    size_t active_beam_idx_{0};
    b2Body* body_;
};

} // namespace just

#endif // __JUST__SENSOR_HPP__
