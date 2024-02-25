#include <iostream>

#include "raylib.h"
//#include "raymath.h"

#include "just/world_model.hpp"

int main()
{
    InitWindow(1000, 1000, "just");
    SetTargetFPS(60);

    int scale = 100;
    Rectangle rec;
    rec.width = scale / 2;
    rec.height = scale / 2;

    int grid_size = 10;

    just::HistogramGrid grid(grid_size, grid_size);
    const uint8_t* data = grid.data();

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(BLACK);

        if (IsKeyPressed(KEY_W)) {
            grid.add_percept(0, 0, -PI / 2, 3.0);
        }
        if (IsKeyPressed(KEY_S)) {
            grid.add_percept(0, 0, PI / 2, 3.0);
        }
        if (IsKeyPressed(KEY_A)) {
            grid.add_percept(0, 0, PI, 3.0);
        }
        if (IsKeyPressed(KEY_D)) {
            grid.add_percept(0, 0, 0.0, 3.0);
        }

        if (IsKeyPressed(KEY_L)) {
            grid.add_percept(0, 0, 0.0, 5.0);
        }

        for (int row = 0; row < grid_size; ++row) {
            rec.y = row * scale + scale / 2;

            for (int col = 0; col < grid_size; ++col) {
                rec.x = col * scale + scale / 2;

                uint8_t cv = data[row * grid_size + col];
                cv *= 10;
                DrawRectangleRec(rec, {cv, cv, cv, 255});
            }
        }


        //const char* txt = "Hello Just";
        //int txt_width = MeasureText(txt, 36);
        //DrawText(txt, (width - txt_width) / 2.0f, 0, 36, GRAY);

        EndDrawing();
    }

    CloseWindow();

    return 0;
}
