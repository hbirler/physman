#include "raylib.h"
#include "math/Physics.hpp"
#include <fmt/core.h>
#include <memory>
#include <catch2/catch_session.hpp>
#if defined(PLATFORM_WEB)
#include <emscripten/emscripten.h>
#endif

using namespace std;
using namespace physman;

unique_ptr<math::Physics> phys;

void UpdateDrawFrame();
Vector2 ballPosition;
const int screenWidth = 800;
const int screenHeight = 450;
unsigned numPoints = 10;
float ballRadius = 5;
//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(int argc, const char* argv[])
{
    int result = Catch::Session().run(argc, argv);
    if (result)
        return result;


    phys = make_unique<math::Physics>(Vec(numPoints * 3 + 3), Vec(numPoints * 3 + 3), Vec(numPoints * 3 + 3, 10.0), 0.0);

    vector<unsigned> grav(numPoints);
    grav[0] = 1;
    phys->addConstraint({math::Constraint::getFixed(300.0, 100.0, 0.0), {0, 1, 2}});
    phys->state[0] = 300.0;
    phys->state[1] = 100.0;

    for (size_t i = 1; i < numPoints; i++) {
        grav[i] = i * 3 + 1;
        phys->state[i * 3 + 0] = 300.0 + i * 10.0;
        phys->state[i * 3 + 1] = 100.0;
        vector<unsigned> comps(6);
        for (size_t j = 0; j < 6; j++)
            comps[j] = (i - 1) * 3 + j;
        phys->addConstraint({math::Constraint::getDistance(10.0), comps});
    }
    phys->state[numPoints * 3 + 0] = 300.0;
    phys->state[numPoints * 3 + 1] = 100.0 + (numPoints - 1) * 10.0;
    phys->ms[numPoints * 3 + 0] = 100.0;
    phys->ms[numPoints * 3 + 1] = 100.0;
    phys->ms[numPoints * 3 + 2] = 100.0;
    phys->addConstraint({math::Constraint::getSphereCollision(ballRadius, ballRadius), {numPoints * 3 - 3, numPoints * 3 - 2, numPoints * 3 - 1, numPoints * 3 + 0, numPoints * 3 + 1, numPoints * 3 + 2}});

    phys->addForce({math::Force::getGravity(100.0), grav});


    // Initialization
    //--------------------------------------------------------------------------------------

    InitWindow(screenWidth, screenHeight, "raylib [core] example - keyboard input");

    ballPosition = { (float)screenWidth/2, (float)screenHeight/2 };

    SetTargetFPS(60);               // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------

#if defined(PLATFORM_WEB)
    emscripten_set_main_loop(UpdateDrawFrame, 0, 1);
#else
    SetTargetFPS(60);   // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------

    // Main game loop
    while (!WindowShouldClose())    // Detect window close button or ESC key
    {
        UpdateDrawFrame();
    }
#endif

    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow();        // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}
//----------------------------------------------------------------------------------
// Module Functions Definition
//----------------------------------------------------------------------------------
void UpdateDrawFrame()
{
    // Update
    //----------------------------------------------------------------------------------
    if (IsKeyDown(KEY_RIGHT)) ballPosition.x += 2.0f;
    if (IsKeyDown(KEY_LEFT)) ballPosition.x -= 2.0f;
    if (IsKeyDown(KEY_UP)) ballPosition.y -= 2.0f;
    if (IsKeyDown(KEY_DOWN)) ballPosition.y += 2.0f;
    //----------------------------------------------------------------------------------

    // Draw
    //----------------------------------------------------------------------------------
    BeginDrawing();

    ClearBackground(RAYWHITE);

    int fps = GetFPS();
    auto txt = fmt::format("physman. FPS: {}", fps);
    DrawText(txt.c_str(), 10, 10, 20, DARKGRAY);

    int rotation = 0;

    // NOTE: Using DrawTexturePro() we can easily rotate and scale the part of the texture we draw
    // sourceRec defines the part of the texture we use for drawing
    // destRec defines the rectangle where our texture part will fit (scaling it to fit)
    // origin defines the point of the texture used as reference for rotation and scaling
    // rotation defines the texture rotation (using origin as rotation point)
    //DrawTexturePro(scarfy, sourceRec, destRec, origin, (float)rotation, WHITE);

    //DrawCircleV(ballPosition, 50, MAROON);
    for (size_t i = 0; i < numPoints + 1; i++)
        DrawCircleV({(float)phys->state[i * 3 + 0], (float)phys->state[i * 3 + 1]}, ballRadius, (i == numPoints ? DARKBLUE : MAROON));

    size_t substep = 10;
    auto frameTime = GetFrameTime();
    for (size_t i = 0; i < substep; i++) {
        //fmt::println("Phys {} {}", i, phys->t);
        phys->step(frameTime / substep);
    }

    EndDrawing();
}