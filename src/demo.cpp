#include <iostream>

#include "raylib.h"
#include "raymath.h"
#include "box2d/box2d.h"
#include "toml++/toml.hpp"

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "Incorrect number of arguments specified. "
                  << "A single argument with the path to a configuration file is required."
                  << std::endl;
        return 1;
    }

    toml::table config;

    try {
        config = toml::parse_file(argv[1]);
    } catch (const toml::parse_error& err) {
        std::cerr << "Parsing the TOML config file failed with error: " << err << std::endl;
        return 2;
    }

    std::cout << "TOML config is:\n\n" << config << std::endl;
    int width = config["world"]["width"].value_or(1000);
    int height = config["world"]["height"].value_or(1000);

    InitWindow(width, height, "just");
    SetTargetFPS(config["world"]["fps"].value_or(60));

    b2World world({0.0f, -10.0f});

    // Agent
    b2BodyDef agent_body_def;
    agent_body_def.type = b2_dynamicBody;
    agent_body_def.position.Set(0.0f, 0.0f);
    agent_body_def.angle = 0.0f;

    b2Body* agent_body = world.CreateBody(&agent_body_def);
    agent_body->SetLinearVelocity({1.0f, 0.0f});

    b2CircleShape agent_shape;
    agent_shape.m_p.Set(0.0f, 0.0f);
    agent_shape.m_radius = 1.0f;

    b2FixtureDef agent_fixture_def;
    agent_fixture_def.shape = &agent_shape;
    agent_fixture_def.density = 1.0f;
    agent_body->CreateFixture(&agent_fixture_def);

    // Ground
    b2BodyDef ground_body_def;
    ground_body_def.type = b2_staticBody;
    ground_body_def.position.Set(0.0f, -5.0f);
    b2Body* ground_body = world.CreateBody(&ground_body_def);

    b2EdgeShape ground_shape;
    ground_shape.SetTwoSided({-10.0f, 0.0f}, {10.0f, 0.0f});
    b2FixtureDef ground_fixture_def;
    ground_fixture_def.shape = &ground_shape;
    ground_body->CreateFixture(&ground_fixture_def);

    //float theta = 0.0;
    b2Vec2 pos;
    while (!WindowShouldClose()) {
        float delta = GetFrameTime();
        world.Step(delta, 10, 8);

        //theta = theta >= (2.0 * M_PI) ? 0.0 : theta + 0.1;

        BeginDrawing();
        ClearBackground(BLACK);

        const char* txt = "Hello Just";
        int txt_width = MeasureText(txt, 36);
        DrawText(txt, (width - txt_width) / 2.0f, 0, 36, GRAY);

        pos = agent_body->GetPosition();
        Vector2 agent_pos = {pos.x * 50 + width / 2.0f, height / 2.0f - pos.y * 50};
        float agent_rot = -agent_body->GetAngle();
        Vector2 agent_dir = Vector2Scale(Vector2Rotate({0.0f, -1.0f}, agent_rot), 50.0f + 20.0f);

        DrawCircleV(agent_pos, 50.0f, WHITE);
        DrawPoly(Vector2Add(agent_pos, agent_dir), 3, 20.0f, RAD2DEG * (agent_rot + M_PI / 6.0f), RED);

        DrawLine(0, height / 2 + 250.0f, width, height / 2 + 250.0f, GRAY);

        EndDrawing();
    }

    CloseWindow();

    return 0;
}
