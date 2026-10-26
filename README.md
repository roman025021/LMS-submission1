<div align="center">

# 🤖 LMS / Meshmerize Selection
## A line-following maze robot that thinks before it turns

**ESP32 · PID control · lost-line recovery · maze solver · ₹7,425 total build**

![Board](https://img.shields.io/badge/board-ESP32--WROOM--32-blue)
![Control](https://img.shields.io/badge/control-PID%20%2B%20state%20machine-orange)
![Sensors](https://img.shields.io/badge/sensors-5%C3%97%20TCRT5000-red)
![Loop](https://img.shields.io/badge/loop-100%20Hz-brightgreen)
![Budget](https://img.shields.io/badge/BOM-%E2%82%B97%2C425%20of%20%E2%82%B910%2C000-success)
![Tests](https://img.shields.io/badge/tests-passing-brightgreen)
![Python](https://img.shields.io/badge/python-3.7%2B-yellow)

*Nine selection tasks, answered with one complete robot design.*
*You can run all of it on your laptop, no hardware needed.*

</div>

---

## 🎯 The 60-second summary

This repo is a full answer to the nine Meshmerize selection tasks: a **plan, a parts list, and working code** for a robot that follows a line through a maze.

- 🧠 **It drives well.** The chosen PID gains (Kp 60, Ki 0.4, Kd 12) settle to 2 mm in **0.36 s** with **zero weaving**. A plain P-only controller takes 5.2 s and weaves 11 times (simulation).
- 🛟 **It recovers from mistakes.** If the line vanishes, the robot coasts over short gaps, pivots to search, then safely stops. Nothing runs away.
- 🧭 **It solves mazes.** Depth-first search with memory finds a route. In map-everything mode it finds the **90 cm shortest path** instead of the 150 cm first-found route.
- 💸 **It is cheap and safe.** The full build costs **₹7,425**, which is ₹2,575 under the ₹10,000 budget. It includes a fuse, a low-voltage alarm and a proper balance charger.
- 🔁 **One brain, two languages.** The same logic runs in Python (for testing) and C++ (on the ESP32).

---

## ⚡ Try it in 30 seconds

```bash
cd src/python
python3 test_logic.py        # ✅ line position, motor clamp, lost-line behaviour
python3 sim.py               # 📈 PID gain sweep: see why the gains were chosen
python3 ../maze/maze_dfs.py  # 🧭 maze solver: walk, map, shortest path
```

Needs Python 3.7+ and **nothing else to install**.

**On the real robot:** open `src/line_follower/line_follower.ino` in the Arduino IDE, choose **ESP32 Dev Module**, and upload.

---

## 🧩 How it works

| Stage | What happens |
|---|---|
| 👁️ **1. Sense** | 5 IR sensors are read twice and averaged. A 3-second wiggle at start-up calibrates each one, and a dead sensor is flagged automatically. |
| 📍 **2. Locate** | A weighted centroid turns the 5 readings into one line position, from −2 (far left) to +2 (far right). |
| 🎛️ **3. Steer** | A PID controller turns the position error into a steering correction. It has anti-windup, a filtered derivative and an output clamp. |
| 🐢 **4. Slow down** | Speed drops as the error grows, so the robot brakes for corners. |
| 🛟 **5. Recover** | Line lost → **COAST** (120 ms) → **SEARCH** (up to 1.2 s) → **STOP** (latched). |
| ⚙️ **6. Drive** | Commands are clamped to ±255 and sent to the L298N motor driver as PWM. |

---

## 🗂️ What is in each folder

| Folder | What you will find | Tasks |
|---|---|---|
| 📦 `BOM/` | Parts list with prices, power diagram, power budget, battery comparison | 4, 5 |
| 📝 `docs/` | Written answers in four notes: `control_pid.md`, `datasheet_hardware.md`, `wireless_diagnostics.md`, `maze_analysis.md` | 2, 3, 6, 7, 8 |
| 🐍 `src/python/` | Python logic, tests and the PID simulator. Runs on any computer. | 1, 3, 9 |
| 🔌 `src/line_follower/` | The ESP32 program in Arduino C++, same logic as the Python code | 9 |
| 🧭 `src/maze/` | Maze solver example | 8 |

### 📖 Where to read the answers

| Task | Topic | Read |
|---|---|---|
| 1 | Line position and motor clamp | `src/python/line_logic.py` |
| 2 | L298N datasheet | `docs/datasheet_hardware.md` |
| 3 | PID design and tuning | `docs/control_pid.md` |
| 4, 5 | Parts list, power and battery | `BOM/BOM.md` |
| 6 | Wiring review and troubleshooting | `docs/datasheet_hardware.md` |
| 7 | Wireless sensor diagnostics (BLE) | `docs/wireless_diagnostics.md` |
| 8 | Maze solving | `docs/maze_analysis.md` |
| 9 | Full robot firmware | `src/line_follower/` |

---

## ⚠️ Read this before you trust it

- 💸 **Prices are estimates.** The links in the parts list must be filled in from the real shops.
- 📌 **Two pins differ from the given sheet**, because they would not work on a real ESP32:
  - `GPIO11` is wired to the flash memory, so using it crashes the chip. IN4 moved to `GPIO4`.
  - `GPIO25` stops working when Wi-Fi is on. IR_S5 moved to `GPIO39`.
  - You can get the literal sheet mapping back by compiling with `APPENDIX_STRICT_PINS`.
- 🎚️ **PID numbers come from a simulation**, not a real robot, so expect small adjustments on the track.
- 🏎️ **No speed sensors (encoders).** Speed drifts a little as the battery drains.
- 💻 **The maze code runs on a computer**, not on the ESP32.
- ⏱️ **The loop runs at 100 Hz**, because printing data faster than that is too much for the serial connection.

---

## 📚 Sources and credits

- ST **L298** datasheet
- Espressif **ESP32** datasheet
- **Arduino-ESP32** documentation
- **MP1584** datasheet
- **Ziegler-Nichols** tuning method
- 🤖 **Claude** (Anthropic's AI assistant), used to help with design planning, code, tests and documentation
