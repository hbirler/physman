#include "Game.hpp"
#include "math/Physics.hpp"
#include <raylib.h>
#include <fmt/format.h>
//---------------------------------------------------------------------------
using namespace std;
//---------------------------------------------------------------------------
namespace physman {
//---------------------------------------------------------------------------
Game::~Game() noexcept = default;
//---------------------------------------------------------------------------
struct GameImpl : Game {
    Camera camera{0};
    unique_ptr<math::Physics> phys;
    unsigned numPoints = 3;
    float ballRadius = 1.0f;
    float ballDistance = 3.0f;

    GameImpl() {
        phys = make_unique<math::Physics>(Vec(numPoints * 3 + 3), Vec(numPoints * 3 + 3), Vec(numPoints * 3 + 3, 10.0), 0.0);

        vector<unsigned> grav(numPoints);
        grav[0] = 1;
        float startX = 0.0;
        float startY = (numPoints - 1) * ballDistance + ballRadius;
        phys->addConstraint({math::Constraint::getFixed(startX, startY, 0.0), {0, 1, 2}});
        phys->state[0] = startX;
        phys->state[1] = startY;

        for (size_t i = 1; i < numPoints; i++) {
            grav[i] = i * 3 + 1;
            phys->state[i * 3 + 0] = startX + i * ballDistance;
            phys->state[i * 3 + 1] = startY;
            vector<unsigned> comps(6);
            for (size_t j = 0; j < 6; j++)
                comps[j] = (i - 1) * 3 + j;
            phys->addConstraint({math::Constraint::getDistance(ballDistance), comps});
        }
        phys->state[numPoints * 3 + 0] = startX;
        phys->state[numPoints * 3 + 1] = startY - (numPoints - 1) * ballDistance + 1.0f;
        phys->state[numPoints * 3 + 2] = ballRadius;
        phys->ms[numPoints * 3 + 0] = 100.0;
        phys->ms[numPoints * 3 + 1] = 100.0;
        phys->ms[numPoints * 3 + 2] = 100.0;
        phys->addConstraint({math::Constraint::getSphereCollision(ballRadius, ballRadius), {numPoints * 3 - 3, numPoints * 3 - 2, numPoints * 3 - 1, numPoints * 3 + 0, numPoints * 3 + 1, numPoints * 3 + 2}});
        phys->addConstraint({math::Constraint::getAxisCollision(ballRadius), {numPoints * 3 + 1}});

        phys->addForce({math::Force::getGravity(-100.0), grav});
        phys->addForce({math::Force::getGravity(-1000.0), {numPoints * 3 + 1}});

        camera.position = Vector3{10.0f, 2.0f, 5.0f};
        camera.target = Vector3{(float)startX, (float)phys->state[numPoints * 3 + 1], 0.0f};
        camera.up = Vector3{0.0f, 1.0f, 0.0f};
        camera.fovy = 60.0f;
        camera.projection = CAMERA_PERSPECTIVE;
    }

    int getScreenWidth() final { return 1024; }
    int getScreenHeight() final { return 768; }
    string getTitle() final { return "physman"; }

    void update(num deltaTime, num totalTime) final {
        UpdateCameraPro(&camera,
                        (Vector3){
                            (IsKeyDown(KEY_W))*0.1f - // Move forward-backward
                                (IsKeyDown(KEY_S))*0.1f,
                            (IsKeyDown(KEY_D))*0.1f - // Move right-left
                                (IsKeyDown(KEY_A))*0.1f,
                            0.0f// Move up-down
                        },
                        (Vector3){
                            -IsKeyDown(KEY_LEFT)*0.5f + IsKeyDown(KEY_RIGHT)*0.5f, // Rotation: yaw
                            -IsKeyDown(KEY_UP)*0.5f + IsKeyDown(KEY_DOWN)*0.5f, // Rotation: pitch
                            0.0f // Rotation: roll
                        },
                        GetMouseWheelMove()*2.0f); // Move to target (zoom)
        if (totalTime > 2.0f && deltaTime < 0.1f) {
            size_t substep = 10;
            auto frameTime = GetFrameTime();
            for (size_t i = 0; i < substep; i++) {
                //fmt::println("Phys {} {}", i, phys->t);
                phys->step(frameTime / substep);
            }
        }
    }

    void draw(num deltaTime, num totalTime) final {
        ClearBackground(RAYWHITE);

        int fps = GetFPS();
        auto txt = fmt::format("physman. FPS: {}", fps);
        DrawText(txt.c_str(), 10, 10, 20, DARKGRAY);

        BeginMode3D(camera);
        // Draw ground
        DrawGrid(10, 1.0f);
        DrawPlane(Vector3{ 0.0f, 0.0f, 0.0f }, Vector2{ 32.0f, 32.0f }, LIGHTGRAY);
        /// Y is up


        if (totalTime > 1.0f) {
            for (size_t i = 0; i < numPoints + 1; i++) {
                Vector3 pos{(float) phys->state[i * 3 + 0], (float) phys->state[i * 3 + 1], (float) phys->state[i * 3 + 2]};
                Vector3 posNext{(float) phys->state[i * 3 + 3], (float) phys->state[i * 3 + 4], (float) phys->state[i * 3 + 5]};
                DrawSphere(pos, ballRadius, (i == numPoints ? DARKBLUE : MAROON));
                DrawSphereWires(pos, ballRadius, 16, 16, BLACK);
                if (i < numPoints - 1) {
                    DrawLine3D(pos, posNext, BLACK);
                }
            }
        }

        EndMode3D();
    }
};
//---------------------------------------------------------------------------
std::unique_ptr<Game> Game::makeGame() { return make_unique<GameImpl>(); }
//---------------------------------------------------------------------------
}