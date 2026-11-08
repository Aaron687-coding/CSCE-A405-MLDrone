# Drone Rescue Sim

A Godot 4 drone that learns to find and rescue victims using reinforcement learning (RL).
Godot runs the world and the drone. Python trains the drone's brain through the
[Godot RL Agents](https://github.com/edbeeching/godot_rl_agents) plugin.

## Requirements

- **Godot 4.7.2 .NET** (everyone must use the same version)
- **.NET SDK 8** (`sudo apt install dotnet-sdk-8.0`)
- **Python 3** with a virtual environment
- The `godot_rl_agents` repo cloned locally (for the training script)

## Files

| File | What it is |
|---|---|
| `addons/godot_rl_agents/` | The RL plugin. Enable it in Project → Project Settings → Plugins |
| `drone.gd` | Drone body: movement, rewards, crashes, rescues, resets. Attach to the Drone node |
| `drone_ai_controller.gd` | Bridge to Python: what the drone senses and how actions are applied. Attach to the AIController3D node |
| `drone.tscn` | The drone scene (to be built) |

## One-time Python setup

```bash
sudo apt install python3-full git
python3 -m venv ~/rl-env
source ~/rl-env/bin/activate
pip install godot-rl
git clone https://github.com/edbeeching/godot_rl_agents.git ~/godot_rl_agents
```

## Training

1. Open the project in Godot and open the map scene with the Sync node.
2. In a terminal:

```bash
source ~/rl-env/bin/activate
cd ~/godot_rl_agents
python examples/stable_baselines3_example.py --save_model_path=drone_model.zip
```

3. When it says it's waiting, press **Play** in Godot.
4. Watch `ep_rew_mean` in the terminal. Rising means it's learning; flat means it's done learning.

**Resume training:**

```bash
python examples/stable_baselines3_example.py --resume_model_path=drone_model.zip --save_model_path=drone_model.zip
```


## Important

- **Don't change what the drone senses or its actions after training starts.** Saved models only work with the exact same observations and actions.
- **Only one person edits a given `.tscn` scene at a time.** Scene files don't merge well in Git.
