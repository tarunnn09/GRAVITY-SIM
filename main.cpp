// gravity sim project
// compile: g++ -std=c++17 main.cpp -lsfml-graphics -lsfml-window -lsfml-system -o gravity
// if that doesnt work try adding -o2 flag idk

#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <string>
#include <random>
#include <sstream>
#define _USE_MATH_DEFINES
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

// global constants, ill put these in a class later maybe
const int WINDOW_W = 1280;
const int WINDOW_H = 720;
const double G_CONST = 0.5;
const double SOFT = 8.0;
const int MAX_TRAIL = 150;
const int BG_STAR_COUNT = 300;

struct Vec2 {
	double x, y;

	Vec2() { x = 0; y = 0; }
	Vec2(double _x, double _y) { x = _x; y = _y; }

	Vec2 operator+(Vec2 other) { return Vec2(x + other.x, y + other.y); }
	Vec2 operator-(Vec2 other) { return Vec2(x - other.x, y - other.y); }
	Vec2 operator*(double s)   { return Vec2(x * s, y * s); }
	Vec2 operator/(double s)   { return Vec2(x / s, y / s); }

	void operator+=(Vec2 other) { x += other.x; y += other.y; }
	void operator-=(Vec2 other) { x -= other.x; y -= other.y; }

	double len() { return sqrt(x * x + y * y); }
	double lenSq() { return x * x + y * y; }

	Vec2 norm() {
		double l = len();
		if (l < 0.000001) return Vec2(0, 0);
		return Vec2(x / l, y / l);
	}

	sf::Vector2f toSFML() { return sf::Vector2f((float)x, (float)y); }
};

// needed for some multiplications where scalar is on left side
Vec2 operator*(double s, Vec2 v) { return Vec2(v.x * s, v.y * s); }

sf::Color mixColors(sf::Color a, sf::Color b, float t) {
	if (t < 0) t = 0;
	if (t > 1) t = 1;
	sf::Color result;
	result.r = (sf::Uint8)(a.r + (b.r - a.r) * t);
	result.g = (sf::Uint8)(a.g + (b.g - a.g) * t);
	result.b = (sf::Uint8)(a.b + (b.b - a.b) * t);
	result.a = (sf::Uint8)(a.a + (b.a - a.a) * t);
	return result;
}

// color based on speed, blue = slow, red = fast
sf::Color getSpeedColor(double speed, double topSpeed) {
	float t = (float)(speed / max(topSpeed, 1.0));
	if (t > 1.0f) t = 1.0f;
	if (t < 0.5f)
		return mixColors(sf::Color(40, 120, 255), sf::Color(60, 230, 100), t * 2.0f);
	else
		return mixColors(sf::Color(60, 230, 100), sf::Color(255, 60, 30), (t - 0.5f) * 2.0f);
}

struct Body {
	Vec2 pos;
	Vec2 vel;
	Vec2 acc;
	double mass;
	float radius;
	bool isStar;
	vector<Vec2> trail;

	Body(Vec2 p, Vec2 v, double m, bool star) {
		pos = p;
		vel = v;
		acc = Vec2(0, 0);
		mass = m;
		isStar = star;
		updateR();
	}

	void updateR() {
		if (isStar) {
			radius = (float)(7.0 + log10(mass) * 1.5);
			if (radius < 8.f)  radius = 8.f;
			if (radius > 35.f) radius = 35.f;
		} else {
			radius = (float)(2.5 + log10(mass) * 0.8);
			if (radius < 3.f)  radius = 3.f;
			if (radius > 15.f) radius = 15.f;
		}
	}

	void addToTrail() {
		trail.push_back(pos);
		if ((int)trail.size() > MAX_TRAIL)
			trail.erase(trail.begin());
		// TODO: this erase from front is slow for big trails, use circular buffer?
	}
};

// helper - speed needed to orbit circularly around mass M at distance r
double circularOrbitSpeed(double M, double r) {
	return sqrt(G_CONST * M / r);
}

class Simulation {
public:
	vector<Body> bodies;
	bool paused = false;
	double timeScale = 1.0;
	int merges = 0;
	int currentPreset = 0;

	void loadScene(int which) {
		bodies.clear();
		merges = 0;
		currentPreset = which;

		double cx = WINDOW_W / 2.0;
		double cy = WINDOW_H / 2.0;

		if (which == 0) {
			// basic solar system
			spawnStar(Vec2(cx, cy), Vec2(0, 0), 1e7);

			double radii[6]  = {80, 125, 170, 225, 300, 380};
			double masses[6] = {5e4, 7e4, 8e4, 6e4, 2e5, 1.5e5};
			double angle = 0.3;
			for (int i = 0; i < 6; i++) {
				double v = circularOrbitSpeed(1e7, radii[i]);
				Vec2 p = Vec2(cx + radii[i] * cos(angle), cy + radii[i] * sin(angle));
				Vec2 vel = Vec2(-v * sin(angle), v * cos(angle));
				spawnPlanet(p, vel, masses[i]);
				angle += 0.9;
			}
		}
		else if (which == 1) {
			// two stars orbiting each other
			double sep = 120.0;
			double v = circularOrbitSpeed(5e6, sep) * 0.5;
			spawnStar(Vec2(cx - sep, cy), Vec2(0, -v), 5e6);
			spawnStar(Vec2(cx + sep, cy), Vec2(0,  v), 5e6);

			mt19937 rng(42);
			uniform_real_distribution<double> aRand(0, 2 * M_PI);
			uniform_real_distribution<double> rRand(250, 380);
			uniform_real_distribution<double> mRand(3e4, 9e4);
			for (int i = 0; i < 10; i++) {
				double a = aRand(rng);
				double r = rRand(rng);
				double spd = circularOrbitSpeed(1e7, r);
				Vec2 p   = Vec2(cx + r * cos(a), cy + r * sin(a));
				Vec2 vel = Vec2(-spd * sin(a), spd * cos(a));
				spawnPlanet(p, vel, mRand(rng));
			}
		}
		else if (which == 2) {
			// two galaxies colliding
			auto spawnGalaxy = [&](Vec2 center, Vec2 drift, int count, unsigned seed) {
				spawnStar(center, drift, 6e6);
				mt19937 rng(seed);
				uniform_real_distribution<double> aRand(0, 2 * M_PI);
				uniform_real_distribution<double> rRand(50, 220);
				uniform_real_distribution<double> mRand(1e4, 6e4);
				for (int i = 0; i < count; i++) {
					double a = aRand(rng);
					double r = rRand(rng);
					double spd = circularOrbitSpeed(6e6, r);
					Vec2 p   = Vec2(center.x + r * cos(a), center.y + r * sin(a));
					Vec2 vel = Vec2(drift.x - spd * sin(a), drift.y + spd * cos(a));
					spawnPlanet(p, vel, mRand(rng));
				}
			};
			spawnGalaxy(Vec2(cx - 260, cy - 60), Vec2( 60,  15), 18, 1);
			spawnGalaxy(Vec2(cx + 260, cy + 60), Vec2(-60, -15), 18, 2);
		}
		else if (which == 3) {
			// chaotic cluster, some retrograde
			mt19937 rng(77);
			uniform_real_distribution<double> aRand(0, 2 * M_PI);
			uniform_real_distribution<double> rRand(20, 280);
			uniform_real_distribution<double> spdJitter(30, 120);
			uniform_real_distribution<double> mRand(2e4, 3e5);

			spawnStar(Vec2(cx, cy), Vec2(0, 0), 8e6);
			for (int i = 0; i < 30; i++) {
				double a   = aRand(rng);
				double r   = rRand(rng);
				double spd = circularOrbitSpeed(8e6, r) + spdJitter(rng);
				double da  = (i % 2 == 0) ? 0.0 : M_PI; // every other one goes backward
				Vec2 p   = Vec2(cx + r * cos(a), cy + r * sin(a));
				Vec2 vel = Vec2(-spd * sin(a + da), spd * cos(a + da));
				spawnPlanet(p, vel, mRand(rng));
			}
		}
	}

	void update(double realDt) {
		if (paused || bodies.empty()) return;
		double dt = realDt * timeScale;

		// reset accelerations
		for (int i = 0; i < (int)bodies.size(); i++)
			bodies[i].acc = Vec2(0, 0);

		// gravity between all pairs - n squared but its fine for small n
		for (int i = 0; i < (int)bodies.size(); i++) {
			for (int j = i + 1; j < (int)bodies.size(); j++) {
				Vec2 d = bodies[j].pos - bodies[i].pos;
				double dist2 = d.lenSq() + SOFT * SOFT;
				double dist  = sqrt(dist2);
				double force = G_CONST * bodies[i].mass * bodies[j].mass / (dist2 * dist);

				Vec2 f = d * force;
				bodies[i].acc += f / bodies[i].mass;
				bodies[j].acc -= f / bodies[j].mass;
			}
		}

		// integrate
		for (int i = 0; i < (int)bodies.size(); i++) {
			bodies[i].vel += bodies[i].acc * dt;
			bodies[i].pos += bodies[i].vel * dt;
			bodies[i].addToTrail();
		}

		handleMerges();
	}

	void spawnPlanet(Vec2 p, Vec2 v, double m) {
		bodies.push_back(Body(p, v, m, false));
	}

	void spawnStar(Vec2 p, Vec2 v, double m) {
		bodies.push_back(Body(p, v, m, true));
	}

private:
	void handleMerges() {
		for (int i = 0; i < (int)bodies.size(); i++) {
			for (int j = i + 1; j < (int)bodies.size(); j++) {
				Vec2 diff = bodies[i].pos - bodies[j].pos;
				float combinedR = (bodies[i].radius + bodies[j].radius) * 0.55f;
				if (diff.len() > combinedR)
					continue;

				// momentum conserving merge
				double newMass = bodies[i].mass + bodies[j].mass;
				Vec2 newPos = (bodies[i].pos * bodies[i].mass + bodies[j].pos * bodies[j].mass) / newMass;
				Vec2 newVel = (bodies[i].vel * bodies[i].mass + bodies[j].vel * bodies[j].mass) / newMass;
				bool starResult = bodies[i].isStar || bodies[j].isStar;

				Body merged(newPos, newVel, newMass, starResult);
				// keep the longer trail
				if (bodies[i].trail.size() >= bodies[j].trail.size())
					merged.trail = bodies[i].trail;
				else
					merged.trail = bodies[j].trail;

				bodies.erase(bodies.begin() + j);
				bodies.erase(bodies.begin() + i);
				bodies.insert(bodies.begin() + i, merged);
				merges++;
				i--;
				break;
			}
		}
	}
};

// background star (not simulated, just decoration)
struct BgStar {
	sf::Vector2f pos;
	sf::Uint8 bright;
	float sz;
};

class GravityApp {
public:
	sf::RenderWindow window;
	sf::Font font;
	bool hasFont = false;

	float zoom = 1.0f;
	sf::Vector2f pan = sf::Vector2f(0, 0);

	bool isLaunching = false;
	Vec2 launchFrom, launchTo;

	double fastestSpeed = 300.0;

	vector<BgStar> backgroundStars;
	sf::Texture glowTex;

	Simulation sim;

	GravityApp() : window(sf::VideoMode(WINDOW_W, WINDOW_H), "Gravity Simulator",
	                      sf::Style::Titlebar | sf::Style::Close)
	{
		window.setFramerateLimit(60);
		window.setVerticalSyncEnabled(true);
		makeGlowTex();
		makeBackground();
		tryLoadFont();
		sim.loadScene(0);
	}

	void run() {
		sf::Clock clock, fpsClock;
		float fps = 60.f;
		int fcount = 0;

		bool panning = false;
		sf::Vector2f panStartMouse, panStartOffset;

		while (window.isOpen()) {
			sf::Event ev;
			while (window.pollEvent(ev)) {
				if (ev.type == sf::Event::Closed)
					window.close();

				if (ev.type == sf::Event::KeyPressed) {
					if      (ev.key.code == sf::Keyboard::Escape) window.close();
					else if (ev.key.code == sf::Keyboard::Space)  sim.paused = !sim.paused;
					else if (ev.key.code == sf::Keyboard::Num1)   sim.loadScene(0);
					else if (ev.key.code == sf::Keyboard::Num2)   sim.loadScene(1);
					else if (ev.key.code == sf::Keyboard::Num3)   sim.loadScene(2);
					else if (ev.key.code == sf::Keyboard::Num4)   sim.loadScene(3);
					else if (ev.key.code == sf::Keyboard::R)      sim.loadScene(sim.currentPreset);
					else if (ev.key.code == sf::Keyboard::C)      sim.bodies.clear();
					else if (ev.key.code == sf::Keyboard::Equal || ev.key.code == sf::Keyboard::Add)
						sim.timeScale = min(sim.timeScale * 2.0, 64.0);
					else if (ev.key.code == sf::Keyboard::Dash || ev.key.code == sf::Keyboard::Subtract)
						sim.timeScale = max(sim.timeScale * 0.5, 0.0625);
				}

				if (ev.type == sf::Event::MouseWheelScrolled) {
					float f = ev.mouseWheelScroll.delta > 0 ? 1.12f : 0.89f;
					sf::Vector2f mpos((float)ev.mouseWheelScroll.x, (float)ev.mouseWheelScroll.y);
					pan  = mpos + (pan - mpos) * f;
					zoom = zoom * f;
					if (zoom < 0.05f) zoom = 0.05f;
					if (zoom > 10.0f) zoom = 10.0f;
				}

				// left mouse = launch planet
				if (ev.type == sf::Event::MouseButtonPressed && ev.mouseButton.button == sf::Mouse::Left) {
					isLaunching = true;
					launchFrom = screenToWorld(ev.mouseButton.x, ev.mouseButton.y);
					launchTo   = launchFrom;
				}
				if (ev.type == sf::Event::MouseMoved && isLaunching)
					launchTo = screenToWorld(ev.mouseMove.x, ev.mouseMove.y);
				if (ev.type == sf::Event::MouseButtonReleased && ev.mouseButton.button == sf::Mouse::Left && isLaunching) {
					isLaunching = false;
					Vec2 launchVel = (launchFrom - launchTo) * 5.0;
					sim.spawnPlanet(launchFrom, launchVel, 8e4);
				}

				// right mouse = drop heavy star
				if (ev.type == sf::Event::MouseButtonPressed && ev.mouseButton.button == sf::Mouse::Right) {
					Vec2 wp = screenToWorld(ev.mouseButton.x, ev.mouseButton.y);
					sim.spawnStar(wp, Vec2(0, 0), 5e6);
				}

				// middle mouse = pan camera
				if (ev.type == sf::Event::MouseButtonPressed && ev.mouseButton.button == sf::Mouse::Middle) {
					panning = true;
					panStartMouse  = sf::Vector2f((float)ev.mouseButton.x, (float)ev.mouseButton.y);
					panStartOffset = pan;
				}
				if (ev.type == sf::Event::MouseMoved && panning) {
					sf::Vector2f cur((float)ev.mouseMove.x, (float)ev.mouseMove.y);
					pan = panStartOffset + (cur - panStartMouse);
				}
				if (ev.type == sf::Event::MouseButtonReleased && ev.mouseButton.button == sf::Mouse::Middle)
					panning = false;
			}

			float dt = clock.restart().asSeconds();
			if (dt > 1.0f / 30.0f) dt = 1.0f / 30.0f;

			// do two half steps, more stable
			sim.update(dt * 0.5);
			sim.update(dt * 0.5);

			fcount++;
			if (fpsClock.getElapsedTime().asSeconds() >= 0.5f) {
				fps = fcount / fpsClock.restart().asSeconds();
				fcount = 0;
			}

			drawFrame(fps);
		}
	}

private:
	sf::Vector2f worldToScreen(Vec2 p) {
		return sf::Vector2f((float)p.x * zoom + pan.x, (float)p.y * zoom + pan.y);
	}
	Vec2 screenToWorld(float sx, float sy) {
		return Vec2((sx - pan.x) / zoom, (sy - pan.y) / zoom);
	}
	Vec2 screenToWorld(int sx, int sy) {
		return screenToWorld((float)sx, (float)sy);
	}

	void makeGlowTex() {
		const int SZ = 128;
		sf::Image img;
		img.create(SZ, SZ, sf::Color::Transparent);
		float center = SZ / 2.0f;
		for (int y = 0; y < SZ; y++) {
			for (int x = 0; x < SZ; x++) {
				float dx = x - center, dy = y - center;
				float dist = sqrt(dx*dx + dy*dy) / center;
				float val = 1.0f - dist;
				if (val < 0) val = 0;
				val = val * val * val;
				img.setPixel(x, y, sf::Color(255, 255, 255, (sf::Uint8)(val * 255)));
			}
		}
		glowTex.loadFromImage(img);
		glowTex.setSmooth(true);
	}

	void makeBackground() {
		mt19937 rng(12345);
		uniform_real_distribution<float> xr(0, WINDOW_W);
		uniform_real_distribution<float> yr(0, WINDOW_H);
		uniform_int_distribution<int> br(60, 255);
		uniform_real_distribution<float> sr(0.5f, 2.0f);
		backgroundStars.resize(BG_STAR_COUNT);
		for (auto& s : backgroundStars) {
			s.pos = sf::Vector2f(xr(rng), yr(rng));
			s.bright = (sf::Uint8)br(rng);
			s.sz = sr(rng);
		}
	}

	void drawBackground() {
		for (auto& s : backgroundStars) {
			sf::CircleShape c(s.sz);
			c.setPosition(s.pos);
			sf::Uint8 b = s.bright;
			c.setFillColor(sf::Color(b, b, (sf::Uint8)min(255, (int)b + 30), b));
			window.draw(c);
		}
	}

	void drawTrail(Body& b) {
		int n = (int)b.trail.size();
		if (n < 2) return;

		sf::Color tipColor;
		if (b.isStar)
			tipColor = sf::Color(255, 200, 80);
		else
			tipColor = getSpeedColor(b.vel.len(), fastestSpeed);

		for (int i = 1; i < n; i++) {
			float alpha = (float)i / n;
			sf::Color c = tipColor;
			c.a = (sf::Uint8)(alpha * alpha * 160);

			sf::Vertex line[2];
			line[0].position = worldToScreen(b.trail[i - 1]);
			line[0].color    = sf::Color(c.r, c.g, c.b, 0);
			line[1].position = worldToScreen(b.trail[i]);
			line[1].color    = c;
			window.draw(line, 2, sf::Lines);
		}
	}

	void drawBody(Body& b) {
		sf::Vector2f sp = worldToScreen(b.pos);
		float sr = b.radius * zoom;

		// draw glow sprite
		float glowSize = sr * (b.isStar ? 7.0f : 3.5f);
		sf::Color glowColor = b.isStar
			? sf::Color(255, 180, 60, 55)
			: sf::Color(80, 140, 255, 35);

		sf::Sprite glow(glowTex);
		glow.setOrigin(64, 64);
		glow.setScale(glowSize / 64.f, glowSize / 64.f);
		glow.setPosition(sp);
		glow.setColor(glowColor);
		window.draw(glow);

		// the main circle
		sf::CircleShape circle(sr);
		circle.setOrigin(sr, sr);
		circle.setPosition(sp);
		if (b.isStar)
			circle.setFillColor(sf::Color(255, 230, 110));
		else
			circle.setFillColor(getSpeedColor(b.vel.len(), fastestSpeed));
		circle.setOutlineColor(sf::Color(255, 255, 255, 50));
		circle.setOutlineThickness(max(0.5f, 0.8f * zoom));
		window.draw(circle);

		// stars get an extra ring
		if (b.isStar) {
			float ringR = sr * 1.5f;
			sf::CircleShape ring(ringR);
			ring.setOrigin(ringR, ringR);
			ring.setPosition(sp);
			ring.setFillColor(sf::Color::Transparent);
			ring.setOutlineColor(sf::Color(255, 160, 40, 50));
			ring.setOutlineThickness(sr * 0.5f);
			window.draw(ring);
		}
	}

	void drawLaunchPreview() {
		sf::Vector2f a = worldToScreen(launchFrom);
		sf::Vector2f b = worldToScreen(launchTo);

		sf::Vertex shaft[2];
		shaft[0] = sf::Vertex(a, sf::Color(0, 255, 160, 80));
		shaft[1] = sf::Vertex(b, sf::Color(0, 255, 160, 220));
		window.draw(shaft, 2, sf::Lines);

		// arrowhead
		Vec2 dir = (launchTo - launchFrom).norm();
		float arrowLen = 14.0f;
		float spread = 2.6f;
		Vec2 left  = Vec2(dir.x*cos(spread)  - dir.y*sin(spread),
		                  dir.x*sin(spread)  + dir.y*cos(spread));
		Vec2 right = Vec2(dir.x*cos(-spread) - dir.y*sin(-spread),
		                  dir.x*sin(-spread) + dir.y*cos(-spread));
		sf::Vertex head[3];
		head[0] = sf::Vertex(b,                                               sf::Color(0, 255, 160, 220));
		head[1] = sf::Vertex(worldToScreen(launchTo - left  * arrowLen),      sf::Color(0, 255, 160, 80));
		head[2] = sf::Vertex(worldToScreen(launchTo - right * arrowLen),      sf::Color(0, 255, 160, 80));
		window.draw(head, 3, sf::Triangles);

		sf::CircleShape dot(5.f);
		dot.setOrigin(5.f, 5.f);
		dot.setPosition(a);
		dot.setFillColor(sf::Color(0, 255, 160, 80));
		window.draw(dot);
	}

	void drawText(const string& str, float x, float y, int size = 13, sf::Color color = sf::Color(200, 200, 210)) {
		sf::Text t(str, font, (unsigned)size);
		t.setFillColor(color);
		t.setPosition(x, y);
		window.draw(t);
	}

	void drawHUD() {
		// semi transparent panel behind stats
		sf::RectangleShape panel(sf::Vector2f(270, 228));
		panel.setFillColor(sf::Color(0, 0, 0, 140));
		panel.setPosition(10, 10);
		window.draw(panel);

		sf::RectangleShape bottomBar(sf::Vector2f(640, 34));
		bottomBar.setFillColor(sf::Color(0, 0, 0, 140));
		bottomBar.setPosition(10, WINDOW_H - 44);
		window.draw(bottomBar);

		if (!hasFont) return;

		const char* sceneNames[] = {"Solar System", "Binary Stars", "Galaxy Collision", "Chaos Cluster"};

		drawText("GRAVITY SIMULATOR", 20, 18, 15, sf::Color(255, 215, 60));
		drawText("Scene:   " + string(sceneNames[sim.currentPreset]), 20, 45);
		drawText("Bodies:  " + to_string(sim.bodies.size()),          20, 63);
		drawText("Merges:  " + to_string(sim.merges),                 20, 81);
		drawText("FPS:     " + to_string((int)fps_cached),            20, 99);

		// show timescale
		ostringstream ts;
		ts.precision(1);
		ts << fixed << sim.timeScale << "x";
		drawText("Speed:   " + ts.str(), 20, 117);

		if (sim.paused)
			drawText("[ PAUSED ]",  20, 135, 13, sf::Color(255, 80, 80));
		else
			drawText("[ RUNNING ]", 20, 135, 13, sf::Color(80, 255, 130));

		// speed color legend
		drawText("--- velocity colour ---", 20, 160, 11, sf::Color(120, 120, 130));

		auto colorDot = [&](float x, float y, sf::Color col, string label) {
			sf::CircleShape dot(5.f);
			dot.setPosition(x, y);
			dot.setFillColor(col);
			window.draw(dot);
			drawText(label, x + 14, y - 1, 11, sf::Color(170, 170, 180));
		};
		colorDot(22, 181, sf::Color(40, 120, 255), "Slow");
		colorDot(22, 199, sf::Color(60, 230, 100), "Medium");
		colorDot(22, 217, sf::Color(255, 60, 30),  "Fast");

		// controls hint at bottom
		const char* hints[] = {
			"[LMB drag] Spawn", "[RMB] Star", "[Scroll] Zoom", "[MMB] Pan",
			"[1-4] Preset", "[Space] Pause", "[+/-] Speed", "[C] Clear", "[R] Reset"
		};
		float bx = 16.f;
		for (auto h : hints) {
			drawText(h, bx, WINDOW_H - 34.f, 11, sf::Color(160, 180, 220));
			bx += 68.f;
		}
	}

	float fps_cached = 60.0f;

	void drawFrame(float fps) {
		fps_cached = fps;
		window.clear(sf::Color(3, 4, 14));
		drawBackground();

		// track fastest body for color scaling
		for (auto& b : sim.bodies)
			if (b.vel.len() > fastestSpeed)
				fastestSpeed = b.vel.len();
		fastestSpeed *= 0.998;

		for (auto& b : sim.bodies) drawTrail(b);
		for (auto& b : sim.bodies) drawBody(b);

		if (isLaunching) drawLaunchPreview();

		drawHUD();
		window.display();
	}

	void tryLoadFont() {
		// just gonna try common system paths, whatever works first
		vector<string> fontPaths = {
			"/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
			"/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
			"/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf",
			"/System/Library/Fonts/Helvetica.ttc",
			"/Library/Fonts/Arial.ttf",
			"C:/Windows/Fonts/segoeui.ttf",
			"C:/Windows/Fonts/arial.ttf",
			"font.ttf"
		};
		for (auto& path : fontPaths) {
			if (font.loadFromFile(path)) {
				hasFont = true;
				return;
			}
		}
		// if no font found, HUD text just wont show up, not a big deal
	}
};

int main() {
	GravityApp app;
	app.run();
	return 0;
}