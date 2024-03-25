#ifndef __JUST__VISULAIZATION_HPP__
#define __JUST__VISULAIZATION_HPP__

#include <string>

#include "raylib.h"

namespace just
{

using Color = Color;

class Visualization
{
public:
    Visualization() = default;
    virtual ~Visualization() = default;
    virtual void draw(float screen_w, float screen_h, float scale, float x, float y) const = 0;
};

struct RectangleViz : public Visualization
{
public:
    RectangleViz(float width, float height, Color color)
        : width_(width), height_(height), color_(color)
    {
    }

    void draw(float screen_w, float screen_h, float scale, float x, float y) const override
    {
        Rectangle rect;
        rect.x = screen_w / 2.0f + scale * x;
        rect.y = screen_h / 2.0f - scale * y;
        rect.width = width_ * scale;
        rect.height = height_ * scale;

        Vector2 vec;
        vec.x = scale * width_ / 2.0f;
        vec.y = scale * height_ / 2.0f;
        DrawRectanglePro(rect, vec, 0.0f, color_);
    }

private:
    float width_;
    float height_;
    Color color_;
};

struct CircleViz : public Visualization
{
public:
    CircleViz(float radius, Color color)
        : radius_(radius), color_(color)
    {
    }

    void draw(float screen_w, float screen_h, float scale, float x, float y) const override
    {
        Vector2 vec;
        vec.x = screen_w / 2.0 + scale * x;
        vec.y = screen_h / 2.0 - scale * y;
        DrawCircleV(vec, scale * radius_, color_);
    }

private:
    float radius_;
    Color color_;
};

class Visualizer
{
public:
    Visualizer(float width, float height, float scale, float fps)
        : width_(width), height_(height), scale_(scale)
    {
        InitWindow(width_, height_, "just");
        SetTargetFPS(fps);
    }

    ~Visualizer()
    {
        CloseWindow();
    }

    bool should_close() const
    {
        return WindowShouldClose();
    }

    void begin_drawing() const
    {
        BeginDrawing();
        ClearBackground(BLACK);
    }

    void end_drawing() const
    {
        EndDrawing();
    }

    RectangleViz create_rectangle_viz(float width, float height, const std::string& color) const
    {
        return RectangleViz(width, height, string_to_color(color));
    }

    CircleViz create_circle_viz(float radius, const std::string& color) const
    {
        return CircleViz(radius, string_to_color(color));
    }

    void draw_viz(float x, float y, const Visualization& viz) const
    {
        viz.draw(width_, height_, scale_, x, y);
    }

private:
    float width_;
    float height_;
    float scale_;

    Color string_to_color(const std::string& color) const {
        if (color == "red") {
            return RED;
        } else if (color == "blue") {
            return BLUE;
        } else if (color == "green") {
            return GREEN;
        } else {
            return WHITE;
        }
    }
};

} // namespace just

#endif // __JUST__VISULAIZATION_HPP__
