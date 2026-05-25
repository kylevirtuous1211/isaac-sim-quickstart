# Isaac Sim Quickstart

Minimal template for running NVIDIA Isaac Sim 5.1.0 via Docker with TCP-based script execution. Includes Isaac Lab v2.3.2.

Hands-on example scripts live in a separate repo: [Digital_Twin_semester_examples](https://github.com/kylevirtuous1211/Digital_Twin_semester_examples).

## Prerequisites

- NVIDIA GPU (RTX 2080+ recommended)
- [Docker](https://docs.docker.com/engine/install/) with [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- (Optional) [Chrome Remote Desktop](https://remotedesktop.google.com/) for GUI access on headless servers

## Quick Start

```bash
# 1. Build and start Isaac Sim (first run pulls ~15GB base image)
docker compose up -d

# 2. Wait for Isaac Sim to initialize (~2 min on first boot)
#    Watch logs: docker compose logs -f isaac-sim

# 3. Run a script (see Digital_Twin_semester_examples for hands-on examples)
./run_in_isaac.py path/to/your_script.py --wait
```

## How It Works

Isaac Sim's `isaacsim.code_editor.vscode` extension opens a TCP socket on **port 8226**. The `run_in_isaac.py` script connects to this socket, sends your Python file, and prints the output.

```
Host                          Docker (network_mode: host)
┌─────────────┐    TCP:8226   ┌──────────────────────┐
│ run_in_isaac │ ───────────> │ Isaac Sim 5.1.0      │
│    .py       │ <─────────── │ VS Code Extension    │
│              │  JSON reply  │ (code executor)      │
└─────────────┘               └──────────────────────┘
```

Scripts support top-level `await` and have access to all Isaac Sim / Isaac Lab APIs.

## Writing Your Own Scripts

Follow the `BaseSample` lifecycle pattern used by Isaac Sim's Robotics Examples:

```python
from isaacsim.examples.interactive.base_sample import BaseSample

class MyTask(BaseSample):
    def setup_scene(self):           # Add objects to the scene
        ...
    async def setup_post_load(self): # Get references, init controllers
        ...
    def physics_step(self, step_size): # Per-tick control logic
        ...
    async def setup_post_reset(self):  # Reset state variables
        ...

sample = MyTask()
await sample.load_world_async()
# physics_step runs automatically via callback
for _ in range(10000):
    await omni.kit.app.get_app().next_update_async()
```

Key: `load_world_async()` handles the full init sequence — new stage, World creation, `initialize_simulation_context_async()`, `reset_async()`.

## GUI Access

If using Chrome Remote Desktop (display :20), the Isaac Sim window appears automatically on startup. If you close it:

```bash
./relaunch_gui.sh
```

## Common Commands

```bash
docker compose up -d              # Start Isaac Sim (background)
docker compose logs -f isaac-sim  # Watch logs
docker compose down               # Stop
docker compose exec isaac-sim bash # Shell into container
```

## Troubleshooting

**Connection refused** — Isaac Sim is still starting. Use `--wait` flag or check `docker compose logs`.

**GPU not detected** — Ensure `nvidia-smi` works on the host and NVIDIA Container Toolkit is installed.

**No display / GUI crash** — Set `DISPLAY` in `.env` or pass it: `DISPLAY=:0 docker compose up -d`.

**Script too large** — The TCP protocol sends the entire file in one shot. Scripts over ~64KB may be truncated. For large scripts, mount them as a volume and use `docker exec`.
