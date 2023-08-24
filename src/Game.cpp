#include "Game.hpp"
#include "Vec3.hpp"
#include "math/Physics.hpp"
#include <raylib.h>
#include <fmt/format.h>
#include <entt/entity/registry.hpp>
//---------------------------------------------------------------------------
using namespace std;
//---------------------------------------------------------------------------
namespace physman {
//---------------------------------------------------------------------------
Game::~Game() noexcept = default;
//---------------------------------------------------------------------------
struct Position {
    Vec3 x;
};
struct Particle {
    num m;
    Vec3 v;
    unsigned offset = 0;
};
struct Gravity {};
struct Force {
    Vec3 f;
};
enum class ColliderType {
    Sphere, Ground
};
struct Collider {
    ColliderType type = ColliderType::Sphere;
    num radius = 0.0;
};
struct RenderSphere {
    Color color = {};
    num radius = 0.0;
};
struct RenderPlane {
    Color color = {};
    num w = 0.0, h = 0.0;
};
struct FixConstraint {
    Vec3 pos;
};
struct DistanceConstraint {
    entt::entity otherEntity{};
    num distance = 0.0;
};
//---------------------------------------------------------------------------
static void push(Vec& v, const Vec3& val) {
    v.reserve(v.size() + 3);
    v.push_back(val.x);
    v.push_back(val.y);
    v.push_back(val.z);
}
//---------------------------------------------------------------------------
Vec3::operator Vector3() const {
    return {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)};
}
//---------------------------------------------------------------------------
struct GameImpl : Game {
    Camera camera{0};
    entt::registry registry;

    GameImpl() {
    }

    int getScreenWidth() final { return 1024; }
    int getScreenHeight() final { return 768; }
    string getTitle() final { return "physman"; }

    void updatePhysics(num deltaTime, num totalTime) {
        auto physView = registry.view<Position, Particle>();
        Vec xs, vs, ms;
        size_t szHint = physView.size_hint();
        xs.reserve(szHint);
        vs.reserve(szHint);
        ms.reserve(szHint);
        physView.each([&](const Position& pos, Particle& part) {
            part.offset = xs.size();
            push(xs, pos.x);
            push(vs, part.v);
            push(ms, {part.m, part.m, part.m});
        });
        size_t numComps = xs.size();

        math::Physics phys(move(xs), move(vs), move(ms), totalTime - deltaTime);

        auto* constantForce = math::Force::getConstant();
        num gravitationalConstant = 9.81;
        registry.view<const Gravity, const Particle>().each([&](const Particle& part) {
            phys.addForce(constantForce, {part.offset, part.offset + 1, part.offset + 2}, {0.0, -gravitationalConstant * part.m, 0.0});
        });
        registry.view<const Force, const Particle>().each([&](const Force& forceComponent, const Particle& part) {
            phys.addForce(constantForce, {part.offset, part.offset + 1, part.offset + 2}, {forceComponent.f.x, forceComponent.f.y, forceComponent.f.z});
        });
        registry.view<const FixConstraint, const Particle>().each([&](const FixConstraint& fix, const Particle& part) {
            phys.addConstraint(math::Constraint::getFixed(), {part.offset, part.offset + 1, part.offset + 2}, {fix.pos.x, fix.pos.y, fix.pos.z});
        });
        registry.view<const Position, const Particle, const DistanceConstraint>().each([&](const Position& pos1, const Particle& part1, const DistanceConstraint& dist) {
            auto* pos2 = registry.try_get<const Position>(dist.otherEntity);
            assert(pos2);
            auto* part2 = registry.try_get<const Particle>(dist.otherEntity);
            if (part2) {
                phys.addConstraint(math::Constraint::getDistance2(), {part1.offset, part1.offset + 1, part1.offset + 2, part2->offset, part2->offset + 1, part2->offset + 2}, {dist.distance});
            } else {
                phys.addConstraint(math::Constraint::getDistance1(), {part1.offset, part1.offset + 1, part1.offset + 2}, {pos2->x.x, pos2->x.y, pos2->x.z, dist.distance});
            }
        });

        auto handleCollide1 = [&](const Position& p1, const Collider& c1, const Particle& part1, const Position& p2, const Collider& c2) {
            assert(c1.type == ColliderType::Sphere);
            if (c2.type == ColliderType::Ground) {
                if (p1.x.y - p2.x.y > c1.radius + epsilon)
                    return;
                phys.addConstraint(math::Constraint::getAxisCollision1(), {part1.offset + 1}, {p2.x.y + c1.radius});
            } else {
                if ((p1.x - p2.x).sqrlen() > (c1.radius + c2.radius + epsilon))
                    return;
                phys.addConstraint(math::Constraint::getSphereCollision1(), {part1.offset, part1.offset + 1, part1.offset + 2}, {p2.x.x, p2.x.y, p2.x.z, c1.radius + c2.radius});
            }
        };
        auto handleCollide2 = [&](const Position& p1, const Collider& c1, const Particle& part1, const Position& p2, const Collider& c2, const Particle& part2) {
            assert(c1.type == ColliderType::Sphere);
            assert(c2.type == ColliderType::Sphere);
            if ((p1.x - p2.x).sqrlen() > (c1.radius + c2.radius + epsilon))
                return;
            phys.addConstraint(math::Constraint::getSphereCollision2(), {part1.offset, part1.offset + 1, part1.offset + 2, part2.offset, part2.offset + 1, part2.offset + 2}, {c1.radius + c2.radius});
        };

        auto colliders = registry.view<const Position, const Collider>();
        colliders.each([&](entt::entity e1, const Position& p1, const Collider& c1) {
            Particle* part1 = registry.try_get<Particle>(e1);
            // Axis collider cannot have physics
            assert(c1.type == ColliderType::Sphere || !part1);
            colliders.each([&](entt::entity e2, const Position& p2, const Collider& c2) {
                if (e2 <= e1)
                    return;
                Particle* part2 = registry.try_get<Particle>(e2);
                if (part1 && part2) {
                    handleCollide2(p1, c1, *part1, p2, c2, *part2);
                } else if (part1) {
                    handleCollide1(p1, c1, *part1, p2, c2);
                } else if (part2) {
                    handleCollide1(p2, c2, *part2, p1, c1);
                }
            });
        });

        size_t substeps = 1;
        for (size_t i = 0; i < substeps; i++)
            phys.step(deltaTime / substeps);
        physView.each([&](Position& pos, Particle& part) {
            pos.x = {phys.state[part.offset], phys.state[part.offset + 1], phys.state[part.offset + 2]};
            part.v = {phys.state[numComps + part.offset], phys.state[numComps + part.offset + 1], phys.state[numComps + part.offset + 2]};
        });
    }

    void draw(num deltaTime, num totalTime) final {
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);

        registry.view<const RenderSphere, const Position>().each([&](const RenderSphere& sphere, const Position& position) {
            DrawSphere(position.x, sphere.radius, sphere.color);
            DrawSphereWires(position.x, sphere.radius, 16, 16, BLACK);
        });
        registry.view<const RenderPlane, const Position>().each([&](const RenderPlane& ground, const Position& position) {
            DrawPlane(position.x, Vector2{ static_cast<float>(ground.w), static_cast<float>(ground.h) }, ground.color);
        });
        registry.view<const Position, const DistanceConstraint>().each([&](const Position& pos, const DistanceConstraint& dc) {
            auto& pos2 = registry.get<const Position>(dc.otherEntity);
            DrawLine3D(pos.x, pos2.x, BLACK);
        });

        EndMode3D();
    }

    void init() final {
        camera.position = Vector3{5.0f, 2.0f, 2.0f};
        camera.target = Vector3{0.0f, 0.0f, 0.0f};
        camera.up = Vector3{0.0f, 1.0f, 0.0f};
        camera.fovy = 60.0f;
        camera.projection = CAMERA_PERSPECTIVE;

        auto ground = registry.create();
        registry.emplace<Position>(ground, Position{{}});
        registry.emplace<RenderPlane>(ground, RenderPlane{GRAY, 100.0f, 100.0f});
        registry.emplace<Collider>(ground, Collider{ColliderType::Ground});

        size_t numBalls = 3;
        num ballRadius = 0.2f;
        num ballDist = 1.0f;
        num topY = ballRadius * 1.5 + (numBalls - 1) * ballDist;
        num topX = 0.0f;

        entt::entity lastBall = entt::null;
        for (size_t i = 0; i < numBalls; i++) {
            auto ball = registry.create();
            registry.emplace<Position>(ball, Position{Vec3{topX + i * ballDist, topY}});
            registry.emplace<RenderSphere>(ball, RenderSphere{MAROON, ballRadius});
            registry.emplace<Particle>(ball, Particle{10.0});
            registry.emplace<Gravity>(ball);
            registry.emplace<Collider>(ball, Collider{ColliderType::Sphere, ballRadius});
            if (lastBall != entt::null) {
                registry.emplace<DistanceConstraint>(ball, DistanceConstraint{lastBall, ballDist});
            } else {
                registry.emplace<FixConstraint>(ball, FixConstraint{registry.get<Position>(ball).x});
            }
            lastBall = ball;
        }
        {
            auto ball = registry.create();
            registry.emplace<Position>(ball, Position{Vec3{topX, topY + ballRadius * 2}});
            registry.emplace<RenderSphere>(ball, RenderSphere{DARKBLUE, ballRadius});
            registry.emplace<Particle>(ball, Particle{10.0});
            registry.emplace<Gravity>(ball);
            registry.emplace<Collider>(ball, Collider{ColliderType::Sphere, ballRadius});
        }
    }

    bool spaceup = true;

    void update(num deltaTime, num totalTime) final {
        UpdateCameraPro(&camera,
                        Vector3{
                            (IsKeyDown(KEY_W))*0.1f - (IsKeyDown(KEY_S))*0.1f,
                            (IsKeyDown(KEY_D))*0.1f - (IsKeyDown(KEY_A))*0.1f,
                            0.0f// Move up-down
                        },
                        Vector3{
                            -IsKeyDown(KEY_LEFT)*1.0f + IsKeyDown(KEY_RIGHT)*1.0f, // Rotation: yaw
                            -IsKeyDown(KEY_UP)*1.0f + IsKeyDown(KEY_DOWN)*1.0f, // Rotation: pitch
                            0.0f // Rotation: roll
                        },
                        GetMouseWheelMove()*2.0f); // Move to target (zoom)

        if (totalTime < 1.0)
            return;

        if (IsKeyDown(KEY_SPACE) && spaceup) {
            num ballRadius = 0.1f;
            auto ball = registry.create();
            auto campos = Vec3{camera.position.x, camera.position.y, camera.position.z};
            auto camtar = Vec3{camera.target.x, camera.target.y, camera.target.z};
            registry.emplace<Position>(ball, Position{campos});
            registry.emplace<RenderSphere>(ball, RenderSphere{DARKBLUE, ballRadius});
            registry.emplace<Particle>(ball, Particle{10.0, (camtar - campos).normalized() * 10.0f});
            registry.emplace<Gravity>(ball);
            registry.emplace<Collider>(ball, Collider{ColliderType::Sphere, ballRadius});
        }
        spaceup = !IsKeyDown(KEY_SPACE);

        updatePhysics(deltaTime, totalTime);
    }
};
//---------------------------------------------------------------------------
std::unique_ptr<Game> Game::makeGame() { return make_unique<GameImpl>(); }
//---------------------------------------------------------------------------
}