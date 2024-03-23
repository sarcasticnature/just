#include <iostream>
#include <vector>
#include <string_view>
#include <memory>

#include "raylib.h"
#include "raymath.h"
#include "box2d/box2d.h"
#include "toml++/toml.hpp"

#include "just/agent.hpp"
#include "just/world_model.hpp"

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
        return 1;
    }

    int width = config["world"]["width"].value_or(1000);
    int height = config["world"]["height"].value_or(1000);
    float scale = config["world"]["scale"].value_or(10.0);

    InitWindow(width, height, "just");
    SetTargetFPS(config["world"]["fps"].value_or(60));

    b2World* world = new b2World({0.0, 0.0});

    // Agent
    std::vector<std::unique_ptr<just::Agent>> agents;
    if (toml::array* agent_configs = config["agents"].as_array()) {
        bool success = false;
        agent_configs->for_each([&agents, &success, &world](toml::table agent_config) {
            if (auto type_opt = agent_config["type"].value<std::string_view>()) {
                if (type_opt.value() == "vfh") {
                    agents.push_back(std::make_unique<just::VFHAgent>(agent_config, world));
                    success = true;
                } else if (type_opt.value() == "patrol") {
                    agents.push_back(std::make_unique<just::PatrolAgent>(agent_config, world));
                    success = true;
                }
            } else {
                std::cout << "Agent type missing or invalid, skipping agent: "
                          << agent_config["name"].value_or("<name missing>")
                          << std::endl;
            }
        });
        if (success == false) {
            std::cout << "Error parsing 'agents' array in config, exiting" << std::endl;
            return 3;
        }
    }

    // Obstacle
    //b2BodyDef obstacle_body_def;
    //obstacle_body_def.type = b2_staticBody;
    //obstacle_body_def.position.Set(15.0, 0.0);
    //b2Body* obstacle_body_1 = world->CreateBody(&obstacle_body_def);

    //b2PolygonShape obstacle_shape;
    //obstacle_shape.SetAsBox(5.0, 10.0);
    //b2FixtureDef obstacle_fixture_def;
    //obstacle_fixture_def.shape = &obstacle_shape;
    //obstacle_body_1->CreateFixture(&obstacle_fixture_def);

    //obstacle_body_def.position.Set(-15.0, 0.0);
    //b2Body* obstacle_body_2 = world->CreateBody(&obstacle_body_def);
    //obstacle_body_2->CreateFixture(&obstacle_fixture_def);

    //obstacle_body_def.position.Set(0.0, -30.0);
    //b2Body* obstacle_body_3 = world->CreateBody(&obstacle_body_def);
    //obstacle_shape.SetAsBox(25.0, 2.5);
    //obstacle_body_3->CreateFixture(&obstacle_fixture_def);

    //float theta = 0.0;
    b2Vec2 pos;
    float rot;
    while (!WindowShouldClose()) {
        float delta = GetFrameTime();
        world->Step(delta, 10, 8);

        //theta = theta >= (2.0 * M_PI) ? 0.0 : theta + 0.1;

        BeginDrawing();
        ClearBackground(BLACK);

        const char* txt = "Hello Just";
        int txt_width = MeasureText(txt, 36);
        DrawText(txt, (width - txt_width) / 2.0, 0, 36, GRAY);

        // TODO: replace with the goal in the config file
        DrawCircle(width / 2.0, height / 2.0, 0.5 * scale, GREEN);

        // TODO: replace with the obstacle(s) in the config file
        //Rectangle rectangle{width / 2.0f + scale * 15.0f, height / 2.0f, 5.0f * 2.0f * scale, 10.0f * 2.0f * scale};
        //DrawRectanglePro(rectangle, {5.0f * scale, 10.0f * scale}, 0.0f, WHITE);

        //rectangle.x = width / 2.0f + scale * -15.0f;
        //DrawRectanglePro(rectangle, {5.0f * scale, 10.0f * scale}, 0.0f, WHITE);

        //rectangle.x = width / 2.0f + scale * 0.0f;
        //rectangle.y = height / 2.0f - scale * -30.0f;
        //rectangle.width = 25.0f * 2.0f * scale;
        //rectangle.height = 2.5f * 2.0f * scale;
        //DrawRectanglePro(rectangle, {25.0f * scale, 2.5f * scale}, 0.0f, WHITE);

        for (auto it = agents.cbegin(); it != agents.cend(); ++it) {
            const auto body = (*it)->get_body();
            auto shape = body->GetFixtureList()->GetShape();
            auto type = shape->GetType();

            pos = body->GetPosition();
            rot = -body->GetAngle() * RAD2DEG;
            Vector2 screen_pos = {pos.x * scale + width / 2.0f, height / 2.0f - pos.y * scale};

            if (type == b2Shape::Type::e_circle) {
                //std::cout << "circle_pos: " << pos.x << ", " << pos.y << std::endl;
                DrawCircleV(screen_pos, shape->m_radius * scale, WHITE);
            } else if (type == b2Shape::Type::e_polygon) {
                auto rectangle_shape = reinterpret_cast<const b2PolygonShape*>(shape);
                float half_width = std::abs(rectangle_shape->m_vertices[0].x);
                float half_height = std::abs(rectangle_shape->m_vertices[0].y);
                Rectangle rectangle{screen_pos.x, screen_pos.y, half_width * 2.0f * scale, half_height * 2.0f * scale};

                DrawRectanglePro(rectangle, {half_width * scale, half_height * scale}, rot, BLUE);
                //DrawRectangleRec({(pos.x - half_width) * scale + width / 2.0f, (-pos.y - half_height) * scale + height / 2.0f, half_width * 2.0f * scale, half_height * 2.0f * scale}, RED);
                //std::cout << "box_pos: " << pos.x << ", " << pos.y << std::endl;
                //std::cout << "box_rot: " << body->GetAngle() << std::endl;
            }
            (*it)->step(delta);
        }

        EndDrawing();
    }

    CloseWindow();

    agents.clear();
    delete world;

    return 0;
}
