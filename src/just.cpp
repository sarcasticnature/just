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

    float theta = 0.0;
    while (!WindowShouldClose()) {
        // TODO: box2d time step
        //float delta = GetFrameTime();
        theta = theta >= (2.0 * M_PI) ? 0.0 : theta + 0.1;

        BeginDrawing();
        ClearBackground(BLACK);

        const char* txt = "Hello Just";
        int txt_width = MeasureText(txt, 36);
        DrawText(txt, (width - txt_width) / 2.0f, 0, 36, GRAY);

        Vector2 agent_pos = {width / 2.0f, height / 2.0f};
        float agent_rot = theta;
        Vector2 agent_dir = Vector2Scale(Vector2Rotate({0.0f, -1.0f}, agent_rot), 50.0f + 20.0f);

        DrawCircle(width / 2.0f, height / 2.0f, 50.0f, WHITE);
        DrawPoly(Vector2Add(agent_pos, agent_dir), 3, 20.0f, RAD2DEG * (agent_rot + M_PI / 6.0f), RED);

        EndDrawing();
    }

    CloseWindow();

    return 0;
}
