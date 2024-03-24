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

std::pair<std::unique_ptr<just::Agent>, std::unique_ptr<just::Visualization>>
agent_pair_factory(const toml::table& agent_config, b2World* world, just::Visualizer visualizer)
{
    auto agent_type = *agent_config["type"].value<std::string>();

    std::unique_ptr<just::Agent> agent_ptr;
    if (agent_type == "vfh") {
        agent_ptr = std::make_unique<just::VFHAgent>(agent_config, world);
    } else if (agent_type == "patrol") {
        agent_ptr = std::make_unique<just::PatrolAgent>(agent_config, world);
    } else {
        std::cout << "Agent type is invalid, skipping agent: "
                  << agent_config["name"].value_or("<name missing>")
                  << std::endl;
        return {nullptr, nullptr};
    }

    std::unique_ptr<just::Visualization> viz_ptr;
    std::string color = agent_config["color"].value_or("blue");

    auto viz_type_opt = agent_config["shape"].value<std::string>();
    if (!viz_type_opt) {
        std::cout << "Agent shape is missing, skipping agent: "
                  << agent_config["name"].value_or("<name missing>")
                  << std::endl;
        return {nullptr, nullptr};
    } else if (*viz_type_opt == "box") {
        int width = agent_config["width"].value_or(1);
        int height = agent_config["height"].value_or(1);
        auto viz = visualizer.create_rectangle_viz(width, height, color);
        viz_ptr = std::make_unique<just::RectangleViz>(viz);
    } else if (*viz_type_opt == "circle") {
        int radius = agent_config["radius"].value_or(1);
        auto viz = visualizer.create_circle_viz(radius, color);
        viz_ptr = std::make_unique<just::CircleViz>(viz);
    } else {
        std::cout << "Agent shape is invalid, skipping agent: "
                  << agent_config["name"].value_or("<name missing>")
                  << std::endl;
        return {nullptr, nullptr};
    }

    return {std::move(agent_ptr), std::move(viz_ptr)};
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
    std::vector<std::pair<std::unique_ptr<just::Agent>, std::unique_ptr<just::Visualization>>> agents;
    if (toml::array* agent_configs = config["agents"].as_array()) {
        agent_configs->for_each([&agents, &world, &visualizer](toml::table agent_config) {
            if (agent_config["type"]) {
                auto [agent_ptr, viz_ptr] = agent_pair_factory(agent_config, world, visualizer);
                if (agent_ptr && viz_ptr) {
                    agents.emplace_back(std::move(agent_ptr), std::move(viz_ptr));
                }
            } else {
                std::cout << "Agent type missing, skipping agent: "
                          << agent_config["name"].value_or("<name missing>")
                          << std::endl;
            }
        });
        if (agents.empty()) {
            std::cout << "Error parsing 'agents' array in config, exiting" << std::endl;
            return 3;
        }
    }

    // Obstacle
    b2BodyDef obstacle_body_def;
    obstacle_body_def.type = b2_staticBody;
    obstacle_body_def.position.Set(15.0, 0.0);
    b2Body* obstacle_body_1 = world->CreateBody(&obstacle_body_def);

    b2PolygonShape obstacle_shape;
    obstacle_shape.SetAsBox(5.0, 10.0);
    b2FixtureDef obstacle_fixture_def;
    obstacle_fixture_def.shape = &obstacle_shape;
    obstacle_body_1->CreateFixture(&obstacle_fixture_def);

    obstacle_body_def.position.Set(-15.0, 0.0);
    b2Body* obstacle_body_2 = world->CreateBody(&obstacle_body_def);
    obstacle_body_2->CreateFixture(&obstacle_fixture_def);

    obstacle_body_def.position.Set(0.0, -30.0);
    b2Body* obstacle_body_3 = world->CreateBody(&obstacle_body_def);
    obstacle_shape.SetAsBox(25.0, 2.5);
    obstacle_body_3->CreateFixture(&obstacle_fixture_def);

    //float theta = 0.0;
    b2Vec2 pos;
    //float rot;    // TODO: use rotation
    while (!WindowShouldClose()) {
        float delta = GetFrameTime();
        world->Step(delta, 10, 8);

        //theta = theta >= (2.0 * M_PI) ? 0.0 : theta + 0.1;

        visualizer.begin_drawing();

        const char* txt = "Hello Just";
        int txt_width = MeasureText(txt, 36);
        DrawText(txt, (width - txt_width) / 2.0, 0, 36, GRAY);

        // TODO: replace with the goal in the config file
        DrawCircle(width / 2.0 + scale * 25.0, height / 2.0 - scale * 10.0, 0.5 * scale, GREEN);

        // TODO: replace with the obstacle(s) in the config file
        Rectangle rectangle{width / 2.0f + scale * 15.0f, height / 2.0f, 5.0f * 2.0f * scale, 10.0f * 2.0f * scale};
        DrawRectanglePro(rectangle, {5.0f * scale, 10.0f * scale}, 0.0f, WHITE);

        rectangle.x = width / 2.0f + scale * -15.0f;
        DrawRectanglePro(rectangle, {5.0f * scale, 10.0f * scale}, 0.0f, WHITE);

        rectangle.x = width / 2.0f + scale * 0.0f;
        rectangle.y = height / 2.0f - scale * -30.0f;
        rectangle.width = 25.0f * 2.0f * scale;
        rectangle.height = 2.5f * 2.0f * scale;
        DrawRectanglePro(rectangle, {25.0f * scale, 2.5f * scale}, 0.0f, WHITE);

        for (const auto& [agent_ptr, viz_ptr] : agents) {
            const auto body = agent_ptr->get_body();

            pos = body->GetPosition();
            //rot = -body->GetAngle() * RAD2DEG;    // TODO: use rotation
            visualizer.draw_viz(pos.x, pos.y, *viz_ptr);

            agent_ptr->step(delta);
        }

        visualizer.end_drawing();
    }

    agents.clear();
    delete world;

    return 0;
}
