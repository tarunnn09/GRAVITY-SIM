# 🌌 Gravity Simulator

A real-time N-body gravitational simulation written in C++ using SFML. Planets orbit, galaxies collide, bodies merge — all running at 60fps.

> Built this to understand how orbital mechanics actually work under the hood, not just the formulas.

![C++](https://img.shields.io/badge/C++-17-blue?style=flat&logo=cplusplus)
![SFML](https://img.shields.io/badge/SFML-2.5-green?style=flat)
![License](https://img.shields.io/badge/license-MIT-orange?style=flat)

---

## Scenes

There are 4 presets you can load:

**1 — Solar System** — one heavy star, 6 planets in stable-ish circular orbits. Good starting point.

**2 — Binary Stars** — two equal stars dancing around each other with a few planets in the outer belt. Watch what happens to the planets over time, they don't always survive.

**3 — Galaxy Collision** — two mini galaxies heading toward each other. Takes a minute to get interesting but worth it.

**4 — Chaos Cluster** — one massive star, 30 bodies with random speeds, half of them in retrograde orbits. Everything goes wrong quickly.

---

## Controls

| Input | What it does |
|---|---|
| Left click + drag | Launch a planet (drag distance = speed) |
| Right click | Drop a heavy star wherever you click |
| Scroll wheel | Zoom in/out |
| Middle mouse drag | Pan the camera |
| `1` `2` `3` `4` | Load a scene |
| `Space` | Pause/unpause |
| `+` / `-` | Speed up or slow down time |
| `R` | Reload current scene |
| `C` | Clear everything |
| `Esc` | Quit |

---

## Build instructions

You need SFML 2.5 installed first.

**Mac:**
```bash
brew install sfml
```

**Linux (Ubuntu/Debian):**
```bash
sudo apt install libsfml-dev
```

**Then build:**
```bash
mkdir build && cd build
cmake ..
make
./gravity
```

Or if you just want to compile manually:
```bash
g++ -std=c++17 main.cpp -lsfml-graphics -lsfml-window -lsfml-system -o gravity
```

---

## How the physics works

The simulation runs an **O(n²) gravity calculation** — every body pulls on every other body each frame. For the number of bodies here (usually under 50) this is perfectly fast.

The force between two bodies is just Newton's law:

```
F = G * m1 * m2 / r²
```

One thing that took me a while to get right was **softening**. Without it, when two bodies get very close the force becomes huge and they fly off to infinity. Adding a small softening constant to the distance before dividing fixes this and keeps close passes stable.

```cpp
double dist2 = d.lenSq() + SOFT * SOFT;  // SOFT = 8.0
```

For integration I used a basic **semi-implicit Euler** method — update velocity first, then position using the new velocity. Running two half-steps per frame instead of one full step helped with energy conservation when the timescale is cranked up.

**Merging** happens when two bodies overlap past a threshold — the result conserves momentum:
```
new_velocity = (m1*v1 + m2*v2) / (m1 + m2)
```

---

## Things I want to add

- [ ] Barnes-Hut tree so it can handle thousands of bodies without dying
- [ ] Save/load simulation state
- [ ] Toggleable velocity vectors on each body
- [ ] Some kind of energy graph so you can see if the simulation is stable

---

## Known issues

- Merging very close to the screen edge sometimes causes a brief position flicker, haven't figured out why yet
- The background starfield doesn't move with the camera (it's purely decorative, not simulated)
- Font won't load on some Linux setups depending on which fonts are installed — the simulation still runs, you just won't see the HUD text

---

## Project structure

```
.
├── main.cpp          # everything lives here for now
├── CMakeLists.txt    # cmake build config
└── README.md
```

---

## Why I built this

Wanted a project that wasn't just another CRUD app or todo list. Physics simulations are a good way to practice working with real-time loops, numerical methods, and rendering — things you don't get from web tutorials. The galaxy collision preset alone is worth it.

---

*If something doesn't build or the physics blows up, open an issue.*