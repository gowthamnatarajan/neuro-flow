import h5py
import numpy as np
from pathlib import Path
from datetime import datetime


class HDF5Writer:
    """Writes one HDF5 file per episode in LeRobot-compatible format.

    Output structure:
        episode_000.hdf5
          attrs: fps, task, state_dim, action_dim, image_shape, num_frames
          observations/images/wrist  [T, H, W, 3]  uint8
          observations/state         [T, 6]         float32  (servo pulses joint1-5 + gripper)
          actions                    [T, 6]         float32
          timestamps                 [T]            float64
    """

    def __init__(self, output_dir: str, task: str = "cube_pick", fps: int = 25):
        self.output_dir = Path(output_dir).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.task = task
        self.fps = fps
        self.episode_count = self._count_existing_episodes()

    def _count_existing_episodes(self) -> int:
        existing = sorted(self.output_dir.glob("episode_*.hdf5"))
        if not existing:
            return 0
        return int(existing[-1].stem.split("_")[1]) + 1

    def save_episode(
        self,
        images: list,
        states: list,
        actions: list,
        timestamps: list,
    ) -> str | None:
        if not actions:
            return None

        imgs = np.array(images, dtype=np.uint8)       # [T, H, W, 3]
        sts = np.array(states, dtype=np.float32)       # [T, 6]
        acts = np.array(actions, dtype=np.float32)     # [T, 6]
        ts = np.array(timestamps, dtype=np.float64)    # [T]

        path = self.output_dir / f"episode_{self.episode_count:03d}.hdf5"

        with h5py.File(path, "w") as f:
            f.attrs["fps"] = self.fps
            f.attrs["task"] = self.task
            f.attrs["state_dim"] = sts.shape[1]
            f.attrs["action_dim"] = acts.shape[1]
            f.attrs["image_shape"] = list(imgs.shape[1:])
            f.attrs["num_frames"] = len(acts)
            f.attrs["created_at"] = datetime.now().isoformat()

            obs = f.create_group("observations")
            obs.create_group("images").create_dataset(
                "wrist", data=imgs, compression="lzf"
            )
            obs.create_dataset("state", data=sts)
            f.create_dataset("actions", data=acts)
            f.create_dataset("timestamps", data=ts)

        self.episode_count += 1
        return str(path)
