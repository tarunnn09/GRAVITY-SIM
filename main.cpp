// build:
//   g++ -std=c++17 -O2 -o gravity main.cpp -lsfml-graphics -lsfml-window -lsfml-system
//   or: mkdir build && cd build && cmake .. && make
//
// controls:
//   LMB drag  - launch a body (drag length = speed)
//   RMB       - drop a heavy star
//   scroll    - zoom,  MMB drag - pan
//   1-4       - load preset,  space - pause,  +/- - timescale
//   C - clear,  R - reload preset,  esc - quit

#include <SFML/Graphics.hpp>
#include <algorithm>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <iomanip>
#include <random>
#include <sstream>
#include <string>
#include <vector>
using namespace std;

namespace cfg {
    constexpr int    WIN_W     = 1280;
    constexpr int    WIN_H     = 720;
    constexpr double G         = 0.5;
    constexpr double SOFTENING = 8.0;   // without this, close passes explode
    constexpr int    TRAIL_LEN = 150;
    constexpr int    STAR_COUNT = 300;
    constexpr float  MIN_ZOOM  = 0.05f;
    constexpr float  MAX_ZOOM  = 10.f;
}

struct Vec2 {
    double x = 0.0, y = 0.0;

    constexpr Vec2() = default;
    constexpr Vec2(double x, double y) : x(x), y(y) {}

    Vec2  operator+ (const Vec2& o) const { return {x+o.x, y+o.y}; }
    Vec2  operator- (const Vec2& o) const { return {x-o.x, y-o.y}; }
    Vec2  operator* (double s)      const { return {x*s, y*s}; }
    Vec2  operator/ (double s)      const { return {x/s, y/s}; }
    Vec2& operator+=(const Vec2& o)       { x+=o.x; y+=o.y; return *this; }
    Vec2& operator-=(const Vec2& o)       { x-=o.x; y-=o.y; return *this; }

    double length()    const { return std::sqrt(x*x + y*y); }
    double lengthSq()  const { return x*x + y*y; }
    Vec2   normalized()const { double l=length(); return l>1e-12 ? (*this/l) : Vec2{}; }
    sf::Vector2f toSf()const { return {(float)x, (float)y}; }
};

inline Vec2 operator*(double s, const Vec2& v) { return v * s; }

static sf::Color lerpColor(sf::Color a, sf::Color b, float t) {
    t = std::clamp(t, 0.f, 1.f);
    return {
        (sf::Uint8)(a.r + (static_cast<float>(b.r) - a.r) * t),
        (sf::Uint8)(a.g + (static_cast<float>(b.g) - a.g) * t),
        (sf::Uint8)(a.b + (static_cast<float>(b.b) - a.b) * t),
        (sf::Uint8)(a.a + (static_cast<float>(b.a) - a.a) * t)
    };
}

// blue=slow, green=medium, red=fast -- makes orbital structure obvious at a glance
static sf::Color speedColor(double speed, double maxSpd) {
    float t = (float)std::min(speed / std::max(maxSpd, 1.0), 1.0);
    if (t < 0.5f) return lerpColor({40,120,255}, {60,230,100}, t * 2.f);
    else           return lerpColor({60,230,100}, {255,60,30},  (t-0.5f)*2.f);
}

struct Body {
    Vec2   pos, vel, acc;
    double mass;
    float  radius;
    bool   isStar;
    std::vector<Vec2> trail;

    Body(Vec2 p, Vec2 v, double m, bool star = false)
        : pos(p), vel(v), acc({0,0}), mass(m), isStar(star)
    {
        updateRadius();
    }

    void updateRadius() {
        if (isStar)
            radius = std::clamp((float)(7.0 + std::log10(mass) * 1.5), 8.f, 35.f);
        else
            radius = std::clamp((float)(2.5 + std::log10(mass) * 0.8), 3.f, 15.f);
    }

    void pushTrail() {
        trail.push_back(pos);
        if ((int)trail.size() > cfg::TRAIL_LEN)
            trail.erase(trail.begin());
    }
};

class Physics {
public:
    std::vector<Body> bodies;
    bool   paused      = false;
    double timeScale   = 1.0;
    int    mergeCount  = 0;
    int    activePreset = 0;

    void loadPreset(int id) {
        bodies.clear();
        mergeCount   = 0;
        activePreset = id;
        const double cx = cfg::WIN_W / 2.0;
        const double cy = cfg::WIN_H / 2.0;

        switch (id) {
        case 0: {
            addStar({cx, cy}, {0,0}, 1e7);
            struct PlanetDef { double r, mass; };
            const PlanetDef pl[] = {
                {80, 5e4}, {125, 7e4}, {170, 8e4},
                {225, 6e4}, {300, 2e5}, {380, 1.5e5}
            };
            double angle = 0.3;
            for (auto& p : pl) {
                double v = orbitalSpeed(1e7, p.r);
                Vec2 pos = {cx + p.r*std::cos(angle), cy + p.r*std::sin(angle)};
                Vec2 vel = {-v*std::sin(angle), v*std::cos(angle)};
                addPlanet(pos, vel, p.mass);
                angle += 0.9;
            }
        } break;

        case 1: {
            // two equal stars in mutual orbit -- sepX sets how tight the dance is
            double sepX = 120.0;
            double v    = orbitalSpeed(5e6, sepX) * 0.5;
            addStar({cx - sepX, cy}, {0, -v}, 5e6);
            addStar({cx + sepX, cy}, {0,  v}, 5e6);

            std::mt19937 rng(42);
            std::uniform_real_distribution<double> angD(0, 2*M_PI);
            std::uniform_real_distribution<double> radD(250, 380);
            std::uniform_real_distribution<double> massD(3e4, 9e4);
            for (int i = 0; i < 10; ++i) {
                double a  = angD(rng);
                double r  = radD(rng);
                double v2 = orbitalSpeed(1e7, r);
                Vec2 pos  = {cx + r*std::cos(a), cy + r*std::sin(a)};
                Vec2 vel  = {-v2*std::sin(a), v2*std::cos(a)};
                addPlanet(pos, vel, massD(rng));
            }
        } break;

        case 2: {
            auto makeGalaxy = [&](Vec2 center, Vec2 drift, int n, unsigned seed) {
                addStar(center, drift, 6e6);
                std::mt19937 rng(seed);
                std::uniform_real_distribution<double> ang(0, 2*M_PI);
                std::uniform_real_distribution<double> rad(50, 220);
                std::uniform_real_distribution<double> ms(1e4, 6e4);
                for (int i = 0; i < n; ++i) {
                    double a = ang(rng);
                    double r = rad(rng);
                    double v = orbitalSpeed(6e6, r);
                    Vec2 pos = {center.x + r*std::cos(a), center.y + r*std::sin(a)};
                    Vec2 vel = {drift.x - v*std::sin(a),  drift.y + v*std::cos(a)};
                    addPlanet(pos, vel, ms(rng));
                }
            };
            makeGalaxy({cx - 260, cy - 60}, { 60,  15}, 18, 1);
            makeGalaxy({cx + 260, cy + 60}, {-60, -15}, 18, 2);
        } break;

        case 3: {
            std::mt19937 rng(77);
            std::uniform_real_distribution<double> ang(0, 2*M_PI);
            std::uniform_real_distribution<double> rad(20, 280);
            std::uniform_real_distribution<double> spdJitter(30, 120);
            std::uniform_real_distribution<double> ms(2e4, 3e5);
            addStar({cx, cy}, {0,0}, 8e6);
            for (int i = 0; i < 30; ++i) {
                double a  = ang(rng);
                double r  = rad(rng);
                double v  = orbitalSpeed(8e6, r) + spdJitter(rng);
                double da = (i % 2 == 0) ? 0.0 : M_PI;  // every other body goes retrograde
                Vec2 pos  = {cx + r*std::cos(a), cy + r*std::sin(a)};
                Vec2 vel  = {-v*std::sin(a + da), v*std::cos(a + da)};
                addPlanet(pos, vel, ms(rng));
            }
        } break;
        }
    }

    void step(double realDt) {
        if (paused || bodies.empty()) return;
        double dt = realDt * timeScale;

        for (auto& b : bodies) b.acc = {0, 0};

        // O(n^2) is fine up to ~200 bodies -- switch to Barnes-Hut if we ever need more
        // TODO: Barnes-Hut tree for large n
        for (int i = 0; i < (int)bodies.size(); ++i) {
            for (int j = i + 1; j < (int)bodies.size(); ++j) {
                Vec2   d     = bodies[j].pos - bodies[i].pos;
                double dist2 = d.lengthSq() + cfg::SOFTENING * cfg::SOFTENING;
                double dist  = std::sqrt(dist2);
                double Fd    = cfg::G * bodies[i].mass * bodies[j].mass / (dist2 * dist);

                Vec2 impulse = d * Fd;
                bodies[i].acc += impulse / bodies[i].mass;
                bodies[j].acc -= impulse / bodies[j].mass;
            }
        }

        // velocity-verlet -- much better energy conservation than euler
        for (auto& b : bodies) {
            b.vel += b.acc * dt;
            b.pos += b.vel * dt;
            b.pushTrail();
        }

        merge();
    }

    void addPlanet(Vec2 p, Vec2 v, double m) { bodies.emplace_back(p, v, m, false); }
    void addStar  (Vec2 p, Vec2 v, double m) { bodies.emplace_back(p, v, m, true);  }

private:
    static double orbitalSpeed(double centralMass, double r) {
        return std::sqrt(cfg::G * centralMass / r);
    }

    void merge() {
        // FIXME: merging very close to screen edge causes a brief position flicker
        for (int i = 0; i < (int)bodies.size(); ++i) {
            for (int j = i + 1; j < (int)bodies.size(); ++j) {
                if ((bodies[i].pos - bodies[j].pos).length() > (bodies[i].radius + bodies[j].radius) * 0.55)
                    continue;

                double totalM = bodies[i].mass + bodies[j].mass;
                Vec2 newPos   = (bodies[i].pos * bodies[i].mass + bodies[j].pos * bodies[j].mass) / totalM;
                Vec2 newVel   = (bodies[i].vel * bodies[i].mass + bodies[j].vel * bodies[j].mass) / totalM;
                bool star     = bodies[i].isStar || bodies[j].isStar;

                Body merged(newPos, newVel, totalM, star);
                merged.trail = (bodies[i].trail.size() >= bodies[j].trail.size())
                               ? bodies[i].trail : bodies[j].trail;

                bodies.erase(bodies.begin() + j);
                bodies.erase(bodies.begin() + i);
                bodies.insert(bodies.begin() + i, std::move(merged));
                ++mergeCount;
                --i;
                break;
            }
        }
    }
};

class Renderer {
public:
    sf::RenderWindow window;
    sf::Font         font;
    bool             fontLoaded = false;

    float        zoom      = 1.f;
    sf::Vector2f panOffset = {0.f, 0.f};

    bool dragging = false;
    Vec2 dragStart, dragCurrent;

    double maxSpeed = 300.0;

    struct Star { sf::Vector2f pos; sf::Uint8 brightness; float size; };
    std::vector<Star> bgStars;

    sf::Texture glowTex;

    Renderer() :
        window(sf::VideoMode(cfg::WIN_W, cfg::WIN_H),
               "Gravity Simulator",
               sf::Style::Titlebar | sf::Style::Close)
    {
        window.setFramerateLimit(60);
        window.setVerticalSyncEnabled(true);
        buildGlowTexture();
        buildStarfield();
        loadFont();
    }

    sf::Vector2f worldToScreen(Vec2 p) const {
        return {(float)p.x * zoom + panOffset.x, (float)p.y * zoom + panOffset.y};
    }
    Vec2 screenToWorld(sf::Vector2f p) const {
        return {(p.x - panOffset.x) / zoom, (p.y - panOffset.y) / zoom};
    }
    Vec2 screenToWorld(int x, int y) const {
        return screenToWorld({(float)x, (float)y});
    }

    void draw(const Physics& sim, float fps) {
        window.clear(sf::Color(3, 4, 14));
        drawStarfield();

        for (const auto& b : sim.bodies)
            maxSpeed = std::max(maxSpeed, b.vel.length());
        maxSpeed *= 0.998;

        for (const auto& b : sim.bodies) drawTrail(b);
        for (const auto& b : sim.bodies) drawBody(b);

        if (dragging) drawLaunchArrow();

        drawHUD(sim, fps);
        window.display();
    }

private:
    void buildGlowTexture() {
        const int SZ = 128;
        sf::Image img;
        img.create(SZ, SZ, sf::Color::Transparent);
        const float C = SZ / 2.f;
        for (int y = 0; y < SZ; ++y)
            for (int x = 0; x < SZ; ++x) {
                float d = std::sqrt((x-C)*(x-C) + (y-C)*(y-C)) / C;
                float t = std::max(0.f, 1.f - d);
                t = t * t * t;
                img.setPixel(x, y, {255, 255, 255, (sf::Uint8)(t * 255)});
            }
        glowTex.loadFromImage(img);
        glowTex.setSmooth(true);
    }

    void buildStarfield() {
        std::mt19937 rng(12345);
        std::uniform_real_distribution<float> xD(0, cfg::WIN_W);
        std::uniform_real_distribution<float> yD(0, cfg::WIN_H);
        std::uniform_int_distribution<int>    bD(60, 255);
        std::uniform_real_distribution<float> sD(0.5f, 2.0f);
        bgStars.resize(cfg::STAR_COUNT);
        for (auto& s : bgStars)
            s = {{xD(rng), yD(rng)}, (sf::Uint8)bD(rng), sD(rng)};
    }

    void drawStarfield() {
        for (const auto& s : bgStars) {
            sf::CircleShape c(s.size);
            c.setPosition(s.pos);
            c.setFillColor({s.brightness, s.brightness, (sf::Uint8)(s.brightness + 30u), s.brightness});
            window.draw(c);
        }
    }

    void drawTrail(const Body& b) {
        int n = (int)b.trail.size();
        if (n < 2) return;
        sf::Color tip = b.isStar ? sf::Color(255,200,80) : speedColor(b.vel.length(), maxSpeed);
        for (int i = 1; i < n; ++i) {
            float alpha = (float)i / n;
            sf::Color c = tip;
            c.a = (sf::Uint8)(alpha * alpha * 160);
            sf::Vertex seg[2];
            seg[0].position = worldToScreen(b.trail[i-1]);
            seg[0].color    = {c.r, c.g, c.b, 0};
            seg[1].position = worldToScreen(b.trail[i]);
            seg[1].color    = c;
            window.draw(seg, 2, sf::Lines);
        }
    }

    void drawBody(const Body& b) {
        sf::Vector2f sp = worldToScreen(b.pos);
        float sr = b.radius * zoom;

        float glowR  = sr * (b.isStar ? 7.f : 3.5f);
        sf::Color gc = b.isStar ? sf::Color(255,180,60,55) : sf::Color(80,140,255,35);
        sf::Sprite gs(glowTex);
        gs.setOrigin(64, 64);
        gs.setScale(glowR / 64.f, glowR / 64.f);
        gs.setPosition(sp);
        gs.setColor(gc);
        window.draw(gs);

        sf::CircleShape core(sr);
        core.setOrigin(sr, sr);
        core.setPosition(sp);
        core.setFillColor(b.isStar ? sf::Color(255,230,110) : speedColor(b.vel.length(), maxSpeed));
        core.setOutlineColor({255,255,255,50});
        core.setOutlineThickness(std::max(0.5f, 0.8f * zoom));
        window.draw(core);

        if (b.isStar) {
            sf::CircleShape corona(sr * 1.5f);
            corona.setOrigin(sr * 1.5f, sr * 1.5f);
            corona.setPosition(sp);
            corona.setFillColor(sf::Color::Transparent);
            corona.setOutlineColor({255,160,40,50});
            corona.setOutlineThickness(sr * 0.5f);
            window.draw(corona);
        }
    }

    void drawLaunchArrow() {
        sf::Vector2f a = worldToScreen(dragStart);
        sf::Vector2f b = worldToScreen(dragCurrent);

        sf::Vertex shaft[2] = {{a, {0,255,160,80}}, {b, {0,255,160,220}}};
        window.draw(shaft, 2, sf::Lines);

        Vec2  dir    = (dragCurrent - dragStart).normalized();
        float len    = 14.f;
        float spread = 2.6f;
        Vec2  left   = {dir.x*std::cos(spread)  - dir.y*std::sin(spread),
                        dir.x*std::sin(spread)  + dir.y*std::cos(spread)};
        Vec2  right  = {dir.x*std::cos(-spread) - dir.y*std::sin(-spread),
                        dir.x*std::sin(-spread) + dir.y*std::cos(-spread)};
        sf::Vertex head[3] = {
            {b,                                        {0,255,160,220}},
            {worldToScreen(dragCurrent - left  * len), {0,255,160,80}},
            {worldToScreen(dragCurrent - right * len), {0,255,160,80}}
        };
        window.draw(head, 3, sf::Triangles);

        sf::CircleShape ghost(5.f);
        ghost.setOrigin(5.f, 5.f);
        ghost.setPosition(a);
        ghost.setFillColor({0,255,160,80});
        window.draw(ghost);
    }

    void drawHUD(const Physics& sim, float fps) {
        auto panel = [&](float x, float y, float w, float h) {
            sf::RectangleShape p({w, h});
            p.setFillColor({0,0,0,140});
            p.setPosition(x, y);
            window.draw(p);
        };
        panel(10, 10, 270, 228);
        panel(10, cfg::WIN_H - 44.f, 640, 34);

        if (!fontLoaded) return;

        auto txt = [&](const std::string& s, float x, float y,
                       int sz = 13, sf::Color c = {200,200,210}) {
            sf::Text t(s, font, (unsigned)sz);
            t.setFillColor(c);
            t.setPosition(x, y);
            window.draw(t);
        };

        txt("GRAVITY SIMULATOR", 20, 18, 15, {255,215,60});

        const char* scenes[] = {"Solar System","Binary Stars","Galaxy Collision","Chaos Cluster"};
        txt("Scene:   " + std::string(scenes[sim.activePreset]), 20, 45);
        txt("Bodies:  " + std::to_string(sim.bodies.size()),     20, 63);
        txt("Merges:  " + std::to_string(sim.mergeCount),        20, 81);

        std::ostringstream fpsSS;
        fpsSS << std::fixed << std::setprecision(0) << fps;
        txt("FPS:     " + fpsSS.str(), 20, 99);

        std::ostringstream tsSS;
        tsSS << std::fixed << std::setprecision(1) << sim.timeScale << "x";
        txt("Speed:   " + tsSS.str(), 20, 117);

        txt(sim.paused ? "[ PAUSED ]" : "[ RUNNING ]", 20, 135, 13,
            sim.paused ? sf::Color(255,80,80) : sf::Color(80,255,130));

        txt("--- velocity colour ---", 20, 160, 11, {120,120,130});
        auto dot = [&](float x, float y, sf::Color c, const std::string& label) {
            sf::CircleShape s(5.f); s.setPosition(x, y); s.setFillColor(c); window.draw(s);
            txt(label, x+14, y-1, 11, {170,170,180});
        };
        dot(22, 181, {40,120,255}, "Slow");
        dot(22, 199, {60,230,100}, "Medium");
        dot(22, 217, {255,60,30},  "Fast");

        const char* controls[] = {
            "[LMB drag] Spawn", "[RMB] Star", "[Scroll] Zoom", "[MMB] Pan",
            "[1-4] Preset", "[Space] Pause", "[+/-] Speed", "[C] Clear", "[R] Reset"
        };
        float bx = 16.f;
        for (const char* c : controls) {
            txt(c, bx, cfg::WIN_H - 34.f, 11, {160,180,220});
            bx += 68.f;
        }
    }

    void loadFont() {
        const char* paths[] = {
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
            "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
            "/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf",
            "/System/Library/Fonts/Helvetica.ttc",
            "/Library/Fonts/Arial.ttf",
            "C:/Windows/Fonts/segoeui.ttf",
            "C:/Windows/Fonts/arial.ttf",
            "font.ttf"
        };
        for (const char* p : paths)
            if (font.loadFromFile(p)) { fontLoaded = true; return; }
    }
};

int main() {
    Renderer renderer;
    Physics  sim;
    sim.loadPreset(0);

    sf::Clock frameClock, fpsClock;
    float fps = 60.f;
    int   frameCount = 0;

    bool         panning = false;
    sf::Vector2f panMouseStart, panOffsetStart;

    while (renderer.window.isOpen()) {
        sf::Event ev;
        while (renderer.window.pollEvent(ev)) {
            if (ev.type == sf::Event::Closed)
                renderer.window.close();

            if (ev.type == sf::Event::KeyPressed) {
                switch (ev.key.code) {
                case sf::Keyboard::Escape: renderer.window.close();           break;
                case sf::Keyboard::Space:  sim.paused = !sim.paused;          break;
                case sf::Keyboard::Num1:   sim.loadPreset(0);                 break;
                case sf::Keyboard::Num2:   sim.loadPreset(1);                 break;
                case sf::Keyboard::Num3:   sim.loadPreset(2);                 break;
                case sf::Keyboard::Num4:   sim.loadPreset(3);                 break;
                case sf::Keyboard::R:      sim.loadPreset(sim.activePreset);  break;
                case sf::Keyboard::C:      sim.bodies.clear();                break;
                case sf::Keyboard::Equal:
                case sf::Keyboard::Add:
                    sim.timeScale = std::min(sim.timeScale * 2.0, 64.0);   break;
                case sf::Keyboard::Dash:
                case sf::Keyboard::Subtract:
                    sim.timeScale = std::max(sim.timeScale * 0.5, 0.0625); break;
                default: break;
                }
            }

            if (ev.type == sf::Event::MouseWheelScrolled) {
                float factor = ev.mouseWheelScroll.delta > 0 ? 1.12f : 0.89f;
                sf::Vector2f mouse = {(float)ev.mouseWheelScroll.x, (float)ev.mouseWheelScroll.y};
                renderer.panOffset = mouse + (renderer.panOffset - mouse) * factor;
                renderer.zoom = std::clamp(renderer.zoom * factor, cfg::MIN_ZOOM, cfg::MAX_ZOOM);
            }

            if (ev.type == sf::Event::MouseButtonPressed && ev.mouseButton.button == sf::Mouse::Left) {
                renderer.dragging    = true;
                renderer.dragStart   = renderer.screenToWorld(ev.mouseButton.x, ev.mouseButton.y);
                renderer.dragCurrent = renderer.dragStart;
            }
            if (ev.type == sf::Event::MouseMoved && renderer.dragging)
                renderer.dragCurrent = renderer.screenToWorld(ev.mouseMove.x, ev.mouseMove.y);
            if (ev.type == sf::Event::MouseButtonReleased && ev.mouseButton.button == sf::Mouse::Left && renderer.dragging) {
                renderer.dragging = false;
                Vec2 vel = (renderer.dragStart - renderer.dragCurrent) * 5.0;
                sim.addPlanet(renderer.dragStart, vel, 8e4);
            }

            if (ev.type == sf::Event::MouseButtonPressed && ev.mouseButton.button == sf::Mouse::Right) {
                Vec2 wp = renderer.screenToWorld(ev.mouseButton.x, ev.mouseButton.y);
                sim.addStar(wp, {0, 0}, 5e6);
            }

            if (ev.type == sf::Event::MouseButtonPressed && ev.mouseButton.button == sf::Mouse::Middle) {
                panning        = true;
                panMouseStart  = {(float)ev.mouseButton.x, (float)ev.mouseButton.y};
                panOffsetStart = renderer.panOffset;
            }
            if (ev.type == sf::Event::MouseMoved && panning) {
                sf::Vector2f m = {(float)ev.mouseMove.x, (float)ev.mouseMove.y};
                renderer.panOffset = panOffsetStart + (m - panMouseStart);
            }
            if (ev.type == sf::Event::MouseButtonReleased && ev.mouseButton.button == sf::Mouse::Middle)
                panning = false;
        }

        float dt = std::min(frameClock.restart().asSeconds(), 1.f/30.f);

        // two sub-steps per frame -- keeps energy stable when timeScale is cranked up
        sim.step(dt * 0.5);
        sim.step(dt * 0.5);

        if (++frameCount, fpsClock.getElapsedTime().asSeconds() >= 0.5f) {
            fps = frameCount / fpsClock.restart().asSeconds();
            frameCount = 0;
        }

        renderer.draw(sim, fps);
    }
    return 0;
}