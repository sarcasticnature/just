#include <iostream>
#include <vector>
#include <string_view>
#include <memory>
#include <utility>

#include "raylib.h"
#include "raymath.h"
#include "box2d/box2d.h"
#include "toml++/toml.hpp"

#include "just/agent.hpp"
#include "just/world_model.hpp"
#include "just/visualization.hpp"

std::unique_ptr<just::Visualization> viz_factory(const toml::table& config,
                                                 const just::Visualizer& visualizer)
{
    if (auto viz_shape_opt = config["shape"].value<std::string>()) {
        std::string color = config["color"].value_or("blue");
        if (*viz_shape_opt == "box") {
            float width = config["width"].value_or(1);
            float height = config["height"].value_or(1);
            auto viz = visualizer.create_rectangle_viz(width, height, color);
            return std::make_unique<just::RectangleViz>(viz);
        } else if (*viz_shape_opt == "circle") {
            float radius = config["radius"].value_or(1.0);
            auto viz = visualizer.create_circle_viz(radius, color);
            return std::make_unique<just::CircleViz>(viz);
        }
    }

    return nullptr;
}

std::unique_ptr<just::Agent> agent_factory(const toml::table& agent_config, b2World* world)
{
    if (auto agent_type_opt = agent_config["type"].value<std::string>()) {
        if (*agent_type_opt == "vfh") {
            return std::make_unique<just::VFHAgent>(agent_config, world);
        } else if (*agent_type_opt == "patrol") {
            return std::make_unique<just::PatrolAgent>(agent_config, world);
        }
    }

    return nullptr;
}

b2Body* obstacle_body_factory(const toml::table& obstacle_config, b2World* world)
{
    // Obstacle
    b2BodyDef body_def;
    body_def.type = b2_staticBody;
    body_def.position.Set(obstacle_config["x"].value_or(0.0), obstacle_config["y"].value_or(0.0));
    body_def.angle = obstacle_config["theta"].value_or(0.0);
    b2Body* body = world->CreateBody(&body_def);

    b2FixtureDef fixture_def;
    fixture_def.density = obstacle_config["density"].value_or(1.0);
    std::string_view shape_str = obstacle_config["shape"].value_or("box");
    if (shape_str == "circle") {
        b2CircleShape shape;
        shape.m_radius = obstacle_config["radius"].value_or(1.0);
        fixture_def.shape = &shape;
        body->CreateFixture(&fixture_def);
    } else if (shape_str == "box") {
        b2PolygonShape shape;
        float width = obstacle_config["width"].value_or(1.0) / 2.0;
        float height = obstacle_config["height"].value_or(1.0) / 2.0;
        shape.SetAsBox(width, height);
        fixture_def.shape = &shape;
        body->CreateFixture(&fixture_def);
    } else {
        return nullptr;
    }

    return body;
}

int main(int argc, char** argv)
{
    toml::table config;

    if (argc == 2) {
        try {
            config = toml::parse_file(argv[1]);
        } catch (const toml::parse_error& err) {
            std::cerr << "Parsing the TOML config file failed with error: " << err << std::endl;
            return 2;
        }
    } else {
        std::cerr << "Error: Pass in a configuration TOML file" << std::endl;
        return 1;
    }

    int width = config["world"]["width"].value_or(1000);
    int height = config["world"]["height"].value_or(1000);
    float scale = config["world"]["scale"].value_or(10.0);
    int fps = config["world"]["fps"].value_or(100.0);

    just::Visualizer visualizer(width, height, scale, fps);

    b2World* world = new b2World({0.0, 0.0});

    // Agent
    using AgentPair = std::pair<std::unique_ptr<just::Agent>, std::unique_ptr<just::Visualization>>;
    std::vector<AgentPair> agent_pairs;
    if (toml::array* agent_configs = config["agents"].as_array()) {
        agent_configs->for_each([&agent_pairs, &world, &visualizer](toml::table agent_config) {
            auto viz_ptr = viz_factory(agent_config, visualizer);

            if (!viz_ptr) {
                std::cout << "Agent visualization options are missing or invalid, "
                          << "skipping agent: "
                          << agent_config["name"].value_or("<name missing>")
                          << std::endl;
                return;
            }

            auto agent_ptr = agent_factory(agent_config, world);

            if (!agent_ptr) {
                std::cout << "Agent type is missing or invalid, skipping agent: "
                          << agent_config["name"].value_or("<name missing>")
                          << std::endl;
                return;
            }
            agent_pairs.emplace_back(std::move(agent_ptr), std::move(viz_ptr));
        });
        if (agent_pairs.empty()) {
            std::cout << "Error parsing 'agents' array in config, exiting" << std::endl;
            return 3;
        }
    }

    using ObstacleTuple = std::tuple<float, float, std::unique_ptr<just::Visualization>>;
    std::vector<ObstacleTuple> obstacles;
    if (toml::array* obstacle_configs = config["obstacles"].as_array()) {
        obstacle_configs->for_each([&obstacles, &world, &visualizer](toml::table obstacle_config) {
            auto viz_ptr = viz_factory(obstacle_config, visualizer);

            if (!viz_ptr) {
                std::cout << "Obstacle visualization options are missing or invalid, "
                          << "skipping obstacle."
                          << std::endl;
                return;
            }

            if (auto body_ptr = obstacle_body_factory(obstacle_config, world)) {
                const auto& [x, y] = body_ptr->GetPosition();
                obstacles.emplace_back(x, y, std::move(viz_ptr));
            } else {
                std::cout << "Obstacle body options are missing or invalid, "
                          << "skipping obstacle."
                          << std::endl;
            }
        });
    }

    using MarkerTuple = std::tuple<float, float, std::unique_ptr<just::Visualization>>;
    std::vector<MarkerTuple> markers;
    if (toml::array* marker_configs = config["markers"].as_array()) {
        marker_configs->for_each([&markers, &visualizer](toml::table marker_config) {
            auto viz_ptr = viz_factory(marker_config, visualizer);

            if (!viz_ptr) {
                std::cout << "Marker visualization options are missing or invalid, "
                          << "skipping marker."
                          << std::endl;
                return;
            }

            float x = marker_config["x"].value_or(0.0);
            float y = marker_config["y"].value_or(0.0);

            markers.emplace_back(x, y, std::move(viz_ptr));
        });
    }

    b2Vec2 pos;
    float rot;
    while (!WindowShouldClose()) {
        float delta = GetFrameTime();
        world->Step(delta, 10, 8);

        visualizer.begin_drawing();

        const char* txt = "Hello Just";
        int txt_width = MeasureText(txt, 36);
        DrawText(txt, (width - txt_width) / 2.0, 0, 36, GRAY);

        for (const auto& [x, y, viz_ptr] : obstacles) {
            visualizer.draw_viz(x, y, 0.0, *viz_ptr);
        }

        for (const auto& [x, y, viz_ptr] : markers) {
            visualizer.draw_viz(x, y, 0.0, *viz_ptr);
        }

        for (const auto& [agent_ptr, viz_ptr] : agent_pairs) {
            const auto body = agent_ptr->get_body();

            pos = body->GetPosition();
            rot = -body->GetAngle() * RAD2DEG;
            visualizer.draw_viz(pos.x, pos.y, rot, *viz_ptr);

            agent_ptr->step(delta);
        }

        visualizer.end_drawing();
    }

    agent_pairs.clear();
    delete world;

    return 0;
}
