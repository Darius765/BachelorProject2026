import numpy as np
import mujoco
from tasks.base_task import BaseTask

class NutAssemblyTask(BaseTask):
    """
    Task: Pick up the square nut and place it on the peg through the hole.
    Success condition: nut is within threshold distance of hole center.
    """

    def __init__(self, model, data):
        super().__init__(model, data)
        self.nut_body_id = -1
        self.hole_site_id = -1
        self.table_geom_id = -1
        self.success_threshold = 0.03  # 3cm
        self.nut_picked = False

    def setup(self):
        self.nut_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "nut")
        self.peg_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "peg1")
        self.table_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "nut_table_surface")

        print(f"NutAssemblyTask ready")
        print(f"Nut body ID: {self.nut_body_id}")
        print(f"Peg body ID: {self.peg_body_id}")

    def step(self, ee_body_id):
        if self.completed:
            return

        nut_pos = self.data.xpos[self.nut_body_id].copy()
        peg_pos = self.data.xpos[self.peg_body_id].copy()

        # Check if nut is picked up
        if nut_pos[2] > 0.48:
            self.nut_picked = True

        # Check success - nut is around peg
        dist = np.linalg.norm(nut_pos[:2] - peg_pos[:2])
        height_diff = abs(nut_pos[2] - peg_pos[2])

        if self.nut_picked and dist < self.success_threshold and height_diff < 0.05:
            self.completed = True
            print("Task complete! Nut placed on peg!")

    def get_status(self):
        if self.completed:
            return "Complete! Nut placed!"
        if self.nut_picked:
            nut_pos = self.data.xpos[self.nut_body_id].copy()
            peg_pos = self.data.xpos[self.peg_body_id].copy()
            dist = np.linalg.norm(nut_pos[:2] - peg_pos[:2])
            return f"Nut picked up! Distance to peg: {dist:.3f}m"
        return "Pick up the nut and place it on the peg"
