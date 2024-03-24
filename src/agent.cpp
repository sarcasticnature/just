#include <string_view>
#include <exception>
#include <filesystem>

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
      valley_threshold_(*config["valley_threshold"].value<float>())
{
    if (config["logging"].value_or(true)) {
        std::string filename = "/tmp/just/" + *config["name"].value<std::string>() + "/foo.h5";
        logger_ = std::make_unique<Logger>(filename, grid_.height() * grid_.width());
    }
    goal_ = {*config["goal"]["x"].value<float>(), *config["goal"]["y"].value<float>()};
}


VFHAgent::Logger::Logger(const std::string& filename, unsigned grid_size)
{
    std::filesystem::path path(filename);
    std::filesystem::create_directories(path.parent_path());

    unsigned file_opts = HighFive::File::OpenOrCreate | HighFive::File::Truncate;
    file_ = std::make_unique<HighFive::File>(filename, file_opts);

    // TODO: remove hardcoding
    HighFive::DataSpace polar_histogram_dataspace({K, 1}, {K, HighFive::DataSpace::UNLIMITED});
    HighFive::DataSetCreateProps polar_histogram_props;
    polar_histogram_props.add(HighFive::Chunking(std::vector<hsize_t>{K, 1}));
    file_->createDataSet("/vfh_agent/polar_histogram",
                         polar_histogram_dataspace,
                         HighFive::create_datatype<float>(),
                         polar_histogram_props);

    HighFive::DataSpace window_histogram_dataspace(
        {WINDOW_SIZE_SQUARED, 1},
        {WINDOW_SIZE_SQUARED, HighFive::DataSpace::UNLIMITED});
    HighFive::DataSetCreateProps window_histogram_props;
    window_histogram_props.add(HighFive::Chunking(std::vector<hsize_t>{WINDOW_SIZE_SQUARED, 1}));
    file_->createDataSet("/vfh_agent/window_histogram",
                         window_histogram_dataspace,
                         HighFive::create_datatype<uint8_t>(),
                         window_histogram_props);

    file_->createDataSet("/vfh_agent/full_histogram",
                         {grid_size},
                         HighFive::create_datatype<uint8_t>());

    HighFive::DataSpace angle_dataspace({100}, {HighFive::DataSpace::UNLIMITED});
    HighFive::DataSetCreateProps angle_props;
    angle_props.add(HighFive::Chunking(std::vector<hsize_t>{100}));
    file_->createDataSet("/vfh_agent/steering_angle",
                         angle_dataspace,
                         HighFive::create_datatype<float>(),
                         angle_props);
    // TODO: log speed;
}

void VFHAgent::Logger::log_polar_histogram(const std::array<float, K>& polar_histogram)
{
    auto dataset = file_->getDataSet("/vfh_agent/polar_histogram");
    auto dims = dataset.getDimensions();
    dims.at(1) += 1;
    dataset.resize(dims);
    dataset.select({0, dims.at(1) - 1}, {K, 1}).write(polar_histogram);
}

void VFHAgent::Logger::log_window(const std::array<uint8_t, WINDOW_SIZE_SQUARED>& window)
{
    auto dataset = file_->getDataSet("/vfh_agent/window_histogram");
    auto dims = dataset.getDimensions();
    dims.at(1) += 1;
    dataset.resize(dims);
    dataset.select({0, dims.at(1) - 1}, {WINDOW_SIZE_SQUARED, 1}).write(window);
}

void VFHAgent::Logger::log_full_grid(const HistogramGrid& grid)
{
    auto dataset = file_->getDataSet("/vfh_agent/full_histogram");
    dataset.write(grid.data());
}

void VFHAgent::Logger::log_steering(float angle, float speed)
{
    // TODO: log speed;
    (void)speed;

    auto dataset = file_->getDataSet("/vfh_agent/steering_angle");
    auto dims = dataset.getDimensions();
    if (dims.at(0) <= steering_idx_) {
        dims.at(0) += 100;
        dataset.resize(dims);
    }
    dataset.select({steering_idx_}, {1}).write(angle);
    ++steering_idx_;
}

void VFHAgent::step(float delta_t)
{
    // TODO: use delta_t to simulate firing the "ultrasonic sensors" in series at a fixed interval,
    // to make the simulation more 'realistic'.
    // This mimics the real deal more closely, as crosstalk prevents firing all sensors at the same
    // time. It also matches the case of a rotating LIDAR or RADAR, as an added bonus.
    (void)delta_t;

    sense();
    if (logger_) {
        logger_->log_full_grid(grid_);
    }

    auto polar_histogram_opt = create_polar_histogram();
    if (!polar_histogram_opt) {
        // Hit the edge of the map, not much to be done about it.
        // TODO: figure out what's to be done about it?

        // Sit still and question life choices.
        body_->SetLinearVelocity({0.0f, 0.0f});
        body_->SetAngularVelocity(0.0f);
        return;
    }

    if (logger_) {
        logger_->log_polar_histogram(*polar_histogram_opt);
    }

    auto [angle, speed] = compute_steering(*polar_histogram_opt);

    if (logger_) {
        logger_->log_steering(angle, speed);
    }

    // TODO: remove after testing
    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    b2Vec2 vel{2 * std::cos(angle), 2 * std::sin(angle)};
    body_->SetLinearVelocity(vel);
}

void VFHAgent::sense()
{
    auto sensor_readings = sensor_.sense_all();

    b2Vec2 position = body_->GetPosition();
    int x = std::round(position.x);
    int y = std::round(position.y);

    for (const auto& [distance, angle] : sensor_readings) {
        if (distance < 0.0) {
            grid_.add_percept(x, y, angle, sensor_.max_range(), false);
        } else {
            grid_.add_percept(x, y, angle, distance, true);
        }
    }
}

std::optional<std::array<float, VFHAgent::K>> VFHAgent::create_polar_histogram()
{
    b2Vec2 position = body_->GetPosition();
    int x = std::round(position.x);
    int y = std::round(position.y);

    auto window_grid_opt = grid_.subgrid<WINDOW_SIZE, WINDOW_SIZE>(x, y);
    if (!window_grid_opt) {
        // Hit the edge of the map, unable to create polar histogram
        return std::nullopt;
    }

    if (logger_) {
        logger_->log_window(*window_grid_opt);
    }

    std::array<float, K> sectors{};
    SubgridAdapter window(std::move(*window_grid_opt));

    // construct the polar histogram
    size_t sector_idx;
    float beta, m, cv, d;
    int x_j, y_i;
    int offset = WINDOW_SIZE % 2 ? 0 : 1;
    for (size_t i = 0; i < WINDOW_SIZE; ++i) {
        y_i = offset + i - (WINDOW_SIZE / 2);
        for (size_t j = 0; j < WINDOW_SIZE; ++j) {
            x_j = offset + j - (WINDOW_SIZE / 2);
            if (x_j == 0 && y_i == 0) {
                continue;
            }
            beta = std::atan2(y_i, x_j);
            while (beta < 0.0) {
                beta += 2 * M_PI;
            }
            cv = static_cast<float>(window.at(j,i));
            d = std::sqrt(x_j * x_j + y_i * y_i);
            m = cv * cv * (A - B * d);
            sector_idx = std::round(beta / ALPHA);
            if (sector_idx >= K) {
                sector_idx -= K;
            }
            sectors.at(sector_idx) += m;
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

    return { smoothed_sectors };
}

VFHAgent::SteeringCommand VFHAgent::compute_steering(const std::array<float, K>& polar_histogram)
{
    // Get the target sector
    b2Vec2 goal_local = body_->GetLocalPoint(goal_);
    float goal_theta = std::atan2(goal_local.y, goal_local.x);
    while (goal_theta < 0.0) {
        goal_theta += 2 * M_PI;
    }
    size_t k_target = std::round(goal_theta / ALPHA);
    if (k_target >= K) {
        k_target -= K;
    }

    float theta;    // output steering angle
    bool target_in_valley = polar_histogram.at(k_target) <= valley_threshold_;

    if (target_in_valley) {
        theta = k_target * ALPHA;
    } else {
        // Determine the start/end sectors of the 'selected valley'
        // This gets a bit messy as there are a lot of edge cases... TODO: improve?
        size_t l, r;
        l = r = k_target;

        // Find the left edge of the peak (exclusive)
        do {
            l = l != 0 ? l - 1 : K - 1;
        } while (polar_histogram.at(l) > valley_threshold_ && l != k_target);

        if (l == k_target) {
            // The only way this can happen is if *all* sectors are above the threshold.
            // When this is the case there is nothing we can (should?) do,
            // other than return a zero'ed out steering command;
            // Note that this check should only be necessary once (on the left side in this case)
            return {0.0, 0.0};
        }

        // Find the right edge of the peak (exclusive)
        do {
            r = r != K - 1 ? r + 1 : 0;
        } while (polar_histogram.at(r) > valley_threshold_);

        size_t distance_l = l <= k_target ? k_target - l : k_target + K - l;
        size_t distance_r = r >= k_target ? r - k_target : r + K - k_target;

        size_t k_n, k_f;
        if (distance_l <= distance_r) {
            k_n = k_f = l;

            do {
                k_f = k_f != 0 ? k_f - 1 : K - 1;
            } while (polar_histogram.at(k_f) <= valley_threshold_);

            k_f = k_f != K - 1 ? k_f + 1 : 0;

            if (k_f <= k_n) {
                k_f = k_n - k_f < S_MAX ? k_f : k_n - S_MAX;
                float idx = (k_f + k_n) / 2.0;
                theta = idx * ALPHA;
            } else {
                // TODO: clean this up if possible
                if (k_n + K - k_f > S_MAX) {
                    int tmp = k_n - S_MAX;
                    while (tmp < 0) {
                        tmp += K;
                    }
                    k_f = tmp;
                }
                // magnitude of the averaged distance from k_target (direction is negative)
                float idx = (distance_l + 1 + (K - 1) - k_f) / 2.0;
                idx = k_target - idx;
                if (idx < 0) {
                    // TODO: I think this is always the case, but... whatever
                    idx += K;
                }
                theta = idx * ALPHA;    // TODO: this could put theta >= 2*PI
            }
        } else {
            k_n = k_f = r;

            do {
                k_f = k_f != K - 1 ? k_f + 1 : 0;
            } while (polar_histogram.at(k_f) <= valley_threshold_);

            k_f = k_f != 0 ? k_f - 1 : K - 1;

            if (k_f >= k_n) {
                k_f = k_f - k_n < S_MAX ? k_f : k_n + S_MAX;
                float idx = (k_f + k_n) / 2.0;
                theta = idx * ALPHA;
            } else {
                // TODO: clean this up if possible
                if (k_f + K - k_n > S_MAX) {
                    k_f = (k_n + S_MAX) % K;
                }
                float idx = (distance_r * 2 + 1 + k_f) / 2.0;
                idx = k_target + idx;
                if (idx > K - 1) {
                    // TODO: I think this is always the case, but... whatever
                    idx -= K;
                }
                theta = idx * ALPHA;    // TODO: could this could put theta >= 2*PI?
            }
        }
    }

    return {theta, 0.0};
}

} // namespace just
