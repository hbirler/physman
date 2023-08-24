#include "Game.hpp"
#include <raylib.h>
#include <catch2/catch_session.hpp>
#if defined(PLATFORM_WEB)
#include <emscripten/emscripten.h>
#endif

using namespace std;
using namespace physman;

unique_ptr<Game> currentGame;

void UpdateDrawFrame();

int main(int argc, const char* argv[])
{
    int result = Catch::Session().run(argc, argv);
    if (result)
        return result;

    currentGame = Game::makeGame();
    currentGame->init();
    auto title = currentGame->getTitle();
    InitWindow(currentGame->getScreenWidth(), currentGame->getScreenHeight(), title.c_str());

    SetTargetFPS(60);
#if defined(PLATFORM_WEB)
    emscripten_set_main_loop(UpdateDrawFrame, 0, 1);
#else
    while (!WindowShouldClose()) {
        UpdateDrawFrame();
    }
#endif
    CloseWindow();
    return 0;
}

void UpdateDrawFrame() {
    double deltaTime = GetFrameTime();
    double totalTime = GetTime();
    currentGame->update(deltaTime, totalTime);

    BeginDrawing();
    currentGame->draw(deltaTime, totalTime);
    EndDrawing();
}
