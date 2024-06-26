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

    std::string_view shape_str = config["shape"].value_or("circle");
    if (shape_str == "circle") {
        b2CircleShape shape;
        shape.m_radius = config["radius"].value_or(1.0);

        b2FixtureDef fixture_def;
        fixture_def.shape = &shape;
        fixture_def.density = config["density"].value_or(1.0);
        body_->CreateFixture(&fixture_def);

    } else if (shape_str == "box") {
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
      valley_threshold_(*config["valley_threshold"].value<float>()),
      v_max_(config["speed"].value_or(1.0))
{
    if (config["logging"].value_or(true)) {
        std::string filename = "/tmp/just/" + *config["name"].value<std::string>() + "/log.h5";
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
    HighFive::DataSpace polar_histogram_dataspace({K, 0}, {K, HighFive::DataSpace::UNLIMITED});
    HighFive::DataSetCreateProps polar_histogram_props;
    polar_histogram_props.add(HighFive::Chunking(std::vector<hsize_t>{K, 1}));
    file_->createDataSet("/vfh_agent/polar_histogram",
                         polar_histogram_dataspace,
                         HighFive::create_datatype<float>(),
                         polar_histogram_props);

    HighFive::DataSpace window_histogram_dataspace(
        {WINDOW_SIZE_SQUARED, 0},
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

    HighFive::DataSpace packed_dataspace({4, 0}, {4, HighFive::DataSpace::UNLIMITED});
    HighFive::DataSetCreateProps packed_props;
    packed_props.add(HighFive::Chunking(std::vector<hsize_t>{4, 1}));
    auto packed_dataset = file_->createDataSet("/vfh_agent/packed_motion",
                                             packed_dataspace,
                                             HighFive::create_datatype<float>(),
                                             packed_props);
    packed_dataset.createAttribute("angle_index", 0);
    packed_dataset.createAttribute("speed_index", 1);
    packed_dataset.createAttribute("x_index", 2);
    packed_dataset.createAttribute("y_index", 3);
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

void VFHAgent::Logger::log_motion(float angle, float speed, float x, float y)
{
    // TODO: update this to match
    (void)speed;

    auto dataset = file_->getDataSet("/vfh_agent/packed_motion");
    auto dims = dataset.getDimensions();
    dims.at(1) += 1;
    dataset.resize(dims);
    std::array<float, 4> packed{angle, speed, x, y};
    dataset.select({0, dims.at(1) - 1}, {4, 1}).write(packed);
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
        b2Vec2 position = body_->GetPosition();
        logger_->log_motion(angle, speed, position.x, position.y);
    }

    b2Vec2 vel{speed * std::cos(angle), speed * std::sin(angle)};
    body_->SetLinearVelocity(vel);
}

void VFHAgent::sense()
{
    auto sensor_readings = sensor_.sense_all();

    b2Vec2 position = body_->GetPosition();
    int x = std::lround(position.x);
    int y = std::lround(position.y);

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
    int x = std::lround(position.x);
    int y = std::lround(position.y);

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
            sector_idx = std::lround(beta / ALPHA);
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
    size_t k_target = std::lround(goal_theta / ALPHA);
    if (k_target >= K) {
        k_target -= K;
    }

    size_t heading;    // sector of the output steering angle
    bool target_in_valley = polar_histogram.at(k_target) <= valley_threshold_;

    if (target_in_valley) {
        heading = k_target;
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
                heading = std::lround((k_f + k_n) / 2.0);
            } else {
                // TODO: clean this up if possible
                size_t wrap = K;
                if (k_n + K - k_f > S_MAX) {
                    int tmp = k_n - S_MAX;
                    while (tmp < 0) {
                        tmp += K;
                    }
                    k_f = tmp;
                    // There may no longer be a wrap-around after adjusting k_f, check for that case
                    wrap = k_f <= k_n ? 0 : K;
                }
                heading = std::lround((k_f + k_n + wrap) / 2.0) % K;
            }
        } else {
            k_n = k_f = r;

            do {
                k_f = k_f != K - 1 ? k_f + 1 : 0;
            } while (polar_histogram.at(k_f) <= valley_threshold_);

            k_f = k_f != 0 ? k_f - 1 : K - 1;

            if (k_f >= k_n) {
                k_f = k_f - k_n < S_MAX ? k_f : k_n + S_MAX;
                heading = std::lround((k_f + k_n) / 2.0);
            } else {
                // TODO: clean this up if possible
                size_t wrap = K;
                if (k_f + K - k_n > S_MAX) {
                    k_f = (k_n + S_MAX) % K;
                    // There may no longer be a wrap-around after adjusting k_f, check for that case
                    wrap = k_f >= k_n ? 0 : K;
                }
                heading = std::lround((k_n + wrap + k_f) / 2.0) % K;
            }
        }
    }

    // h_m is intended to be emperically determined (per the paper).
    // Here we will simply use the valley threshold as a heuristic to make tuning easier.
    // Note that the min of h_c and h_m is not needed, as it is not possible for h_c to exceed
    // the valley threshold in the current implementation
    //
    // TODO: determine emperically?

    float v = v_max_ * (1 - polar_histogram.at(heading) / (valley_threshold_ * 1.1));

    return {heading * ALPHA, v};
}

} // namespace just
