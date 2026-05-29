import gymnasium as gym
import torch
from isaaclab.devices import Se2Keyboard # Helper for W/A/S/D

def main():
    env = gym.make("JetAuto-Pick-v0")
    keyboard = Se2Keyboard()
    obs, _ = env.reset()

    while True:
        # Get X-Y-Z deltas from keyboard
        dx, dy, dz = keyboard.get_delta()
        
        # Action: [dx, dy, dz, 0, 0, 0, gripper_val]
        action = torch.zeros((1, 7), device=env.unwrapped.device)
        action[0, :3] = torch.tensor([dx, dy, dz]) * 0.05
        
        # O/P for gripper toggle
        if keyboard.is_pressed("P"): action[0, 6] = 1.0 # Close
        if keyboard.is_pressed("O"): action[0, 6] = 0.0 # Open

        obs, reward, terminated, truncated, _ = env.step(action)
        env.render()
        
        if terminated or truncated: env.reset()

if __name__ == "__main__":
    main()