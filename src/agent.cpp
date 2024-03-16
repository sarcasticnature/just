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
              body_),
      logger_("/tmp/just/" + *config["name"].value<std::string>() + "/foo.h5",
              grid_.height() * grid_.width())
{
}


VFHAgent::Logger::Logger(std::string filename, unsigned grid_size)
    : file_(filename, HighFive::File::OpenOrCreate | HighFive::File::Truncate)
{
    // TODO: remove hardcoding
    // TODO: create directory if it doesn't already exist
    HighFive::DataSpace dataspace({K, 1}, {K, HighFive::DataSpace::UNLIMITED});
    HighFive::DataSetCreateProps props;
    props.add(HighFive::Chunking(std::vector<hsize_t>{K, 1}));
    file_.createDataSet("/vfh_agent/polar_histogram",
                        dataspace,
                        HighFive::create_datatype<float>(),
                        props);

    file_.createDataSet("/vfh_agent/window_histogram",
                        {WINDOW_SIZE_SQUARED},
                        HighFive::create_datatype<uint8_t>());

    file_.createDataSet("/vfh_agent/full_histogram",
                        {grid_size},
                        HighFive::create_datatype<uint8_t>());
}

void VFHAgent::Logger::log_polar_histogram(std::array<float, K> polar_histogram)
{
    auto dataset = file_.getDataSet("/vfh_agent/polar_histogram");
    auto dims = dataset.getDimensions();
    dims.at(1) += 1;
    dataset.resize(dims);
    dataset.select({0, dims.at(1) - 1}, {K, 1}).write(polar_histogram);
}

void VFHAgent::Logger::log_window(std::array<uint8_t, WINDOW_SIZE_SQUARED> window)
{
    auto dataset = file_.getDataSet("/vfh_agent/window_histogram");
    dataset.write(window);
}

void VFHAgent::Logger::log_full_grid(const HistogramGrid& grid)
{
    auto dataset = file_.getDataSet("/vfh_agent/full_histogram");
    dataset.write(grid.data());
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

    logger_.log_full_grid(grid_);

    auto window_grid_opt = grid_.subgrid<WINDOW_SIZE, WINDOW_SIZE>(x, y);
    if (!window_grid_opt) {
        // Hit the edge of the map, not much to be done about it
        // TODO: figure out what's to be done about it?
        return;
    }

    logger_.log_window(*window_grid_opt);
    std::array<float, K> sectors;
    SubgridAdapter window(std::move(*window_grid_opt));

    // construct the polar histogram
    float beta, m, cv, d;
    int x_j, y_i;
    int offset = WINDOW_SIZE % 2 ? 0 : 1;
    for (size_t i = 0; i < WINDOW_SIZE; ++i) {
        y_i = offset + i - (WINDOW_SIZE / 2);
        for (size_t j = 0; j < WINDOW_SIZE; ++j) {
            x_j = offset + j - (WINDOW_SIZE / 2);
            beta = std::atan2(y_i, x_j);
            while (beta < 0.0) {
                beta += 2 * M_PI;
            }
            cv = static_cast<float>(window.at(j,i));
            d = std::sqrt(x_j * x_j + y_i * y_i);
            m = cv * cv * (A - B * d);
            sectors.at(std::floor(beta / ALPHA)) += m;  // TODO: is floor what we want?
        }
    }

    // smooth the polar histogram
    int h_prime, idx;
    std::array<float, K> smoothed_sectors;
    for (int i = 0; i < K; ++i) {
        h_prime = 0;
        for (int l = -L; l <= L; ++l) {
            idx = i + l;

            if (idx < 0) {
                idx += K;
            } else if (idx >= K) {
                idx -= K;
            }

            // Slight difference from the paper here:
            // I think there's a typo/error in the original publicaion (equation 5)
            h_prime += sectors.at(idx) * (1 + L - std::abs(l));
        }
        h_prime /= 2 * L + 1;
        smoothed_sectors.at(i) = h_prime;
    }

    logger_.log_polar_histogram(smoothed_sectors);
}

} // namespace just
