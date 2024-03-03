#include <iostream>
#include <vector>
#include <string_view>
#include <memory>

#include "raylib.h"
#include "raymath.h"
#include "box2d/box2d.h"
#include "toml++/toml.hpp"

#include "just/agent.hpp"

int main(int argc, char** argv)
{
    static constexpr auto demo_config = R"([world]
height = 1000
width = 1000
scale = 10.0
fps = 60

[[agents]]
name = "tom"
type = "vfh"
grid = { width = 1000, height = 1000 }
sensor = { count = 8, range = 5.0 }
color = "red"
shape = "circle"
radius = 2.0
x = 5.0
y = 0.0
theta = 0.0

[[agents]]
name = "jerry"
type = "vfh"
grid = { width = 1000, height = 1000 }
sensor = { count = 8, range = 5.0 }
color = "red"
shape = "box"
width = 4.0
height = 4.0
x = 5.01
y = 10.0
theta = 0.25
    )";

    toml::table config;

    if (argc == 2) {
        try {
            config = toml::parse_file(argv[1]);
        } catch (const toml::parse_error& err) {
            std::cerr << "Parsing the TOML config file failed with error: " << err << std::endl;
            return 2;
        }
    } else {
        config = toml::parse(demo_config);
    }



    std::cout << "TOML config is:\n\n" << config << std::endl;
    int width = config["world"]["width"].value_or(1000);
    int height = config["world"]["height"].value_or(1000);
    float scale = config["world"]["scale"].value_or(10.0);

    InitWindow(width, height, "just");
    SetTargetFPS(config["world"]["fps"].value_or(60));

    b2World* world = new b2World({0.0, -10.0});

    // Agent
    std::vector<std::unique_ptr<just::Agent>> agents;
    if (toml::array* agent_configs = config["agents"].as_array()) {
        bool success = false;
        agent_configs->for_each([&agents, &success, &world](toml::table agent_config) {
            if (auto type_opt = agent_config["type"].value<std::string_view>()) {
                if (type_opt.value() == "vfh") {
                    agents.push_back(std::make_unique<just::VFHAgent>(agent_config, world));
                    success = true;
                    return;
                }
            }
            std::cout << "Agent type missing or invalid, skipping agent: "
                      << agent_config["name"].value_or("<name missing>")
                      << std::endl;
        });
        if (success == false) {
            std::cout << "Error parsing 'agents' array in config, exiting" << std::endl;
            return 3;
        }
    }

    // Ground
    b2BodyDef ground_body_def;
    ground_body_def.type = b2_staticBody;
    ground_body_def.position.Set(0.0, -5.0);
    b2Body* ground_body = world->CreateBody(&ground_body_def);

    b2EdgeShape ground_shape;
    ground_shape.SetTwoSided({-10.0, 0.0}, {10.0, 0.0});
    b2FixtureDef ground_fixture_def;
    ground_fixture_def.shape = &ground_shape;
    ground_body->CreateFixture(&ground_fixture_def);

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


        for (b2Body* body = world->GetBodyList(); body; body = body->GetNext()) {
            auto shape = body->GetFixtureList()->GetShape();
            auto type = shape->GetType();

            pos = body->GetPosition();
            rot = -body->GetAngle() * RAD2DEG;
            Vector2 screen_pos = {pos.x * scale + width / 2.0f, height / 2.0f - pos.y * scale};

            if (type == b2Shape::Type::e_circle) {
                std::cout << "circle_pos: " << pos.x << ", " << pos.y << std::endl;
                DrawCircleV(screen_pos, shape->m_radius * scale, WHITE);   // TODO: make 50.0 a real thing
            } else if (type == b2Shape::Type::e_polygon) {
                auto rectangle_shape = reinterpret_cast<b2PolygonShape*>(shape);
                float half_width = std::abs(rectangle_shape->m_vertices[0].x);
                float half_height = std::abs(rectangle_shape->m_vertices[0].y);
                //Rectangle rectangle{(pos.x - half_width) * scale + width / 2.0f, (-pos.y - half_height) * scale + height / 2.0f, half_width * 2.0f * scale, half_height * 2.0f * scale};
                Rectangle rectangle{screen_pos.x, screen_pos.y, half_width * 2.0f * scale, half_height * 2.0f * scale};

                DrawRectanglePro(rectangle, {half_width * scale, half_height * scale}, rot, BLUE);
                //DrawRectanglePro(rectangle, {0.0f, 0.0f}, rot, BLUE);
                DrawRectangleRec({(pos.x - half_width) * scale + width / 2.0f, (-pos.y - half_height) * scale + height / 2.0f, half_width * 2.0f * scale, half_height * 2.0f * scale}, RED);
                std::cout << "box_pos: " << pos.x << ", " << pos.y << std::endl;
                std::cout << "box_rot: " << body->GetAngle() << std::endl;
            }
        }

        //pos = agent_body->GetPosition();
        //Vector2 agent_pos = {pos.x * 50.0f + width / 2.0f, height / 2.0f - pos.y * 50.0f};
        //float agent_rot = -agent_body->GetAngle();
        //Vector2 agent_dir = Vector2Scale(Vector2Rotate({0.0, -1.0}, agent_rot), 50.0 + 20.0);

        //DrawCircleV(agent_pos, 50.0, WHITE);
        //DrawPoly(Vector2Add(agent_pos, agent_dir), 3, 20.0, RAD2DEG * (agent_rot + M_PI / 6.0), RED);

        //DrawLine(0, height / 2 + 250.0, width, height / 2 + 250.0, GRAY);

        EndDrawing();
    }

    CloseWindow();

    agents.clear();
    delete world;

    return 0;
}
