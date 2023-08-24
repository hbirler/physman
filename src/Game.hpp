#pragma once
//---------------------------------------------------------------------------
#include <memory>
#include <string>
#include "math/Num.hpp"
//---------------------------------------------------------------------------
namespace physman {
//---------------------------------------------------------------------------
class Game {
    public:
    virtual ~Game() noexcept;
    virtual void update(num deltaTime, num totalTime) = 0;
    virtual void draw(num deltaTime, num totalTime) = 0;

    virtual int getScreenWidth() = 0;
    virtual int getScreenHeight() = 0;
    virtual std::string getTitle() = 0;

    static std::unique_ptr<Game> makeGame();
};
//---------------------------------------------------------------------------
}