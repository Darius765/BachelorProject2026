import numpy as np
import mujoco
import time
from tasks.base_task import BaseTask

class DrawerTask(BaseTask):
    """
    Task:
    1. Pull open the drawer
    2. Pick up the bowl and place it in the drawer
    3. Close the drawer with the bowl inside
    """

    def __init__(self, model, data):
        super().__init__(model, data)
        self.drawer_joint_id = -1
        self.bowl_body_id = -1
        self.table_geom_id = -1
        self.drawer_body_id = -1

        # Task stages
        self.stage = 0
        # 0 = open drawer
        # 1 = pick up bowl and place in drawer
        # 2 = close drawer

        # Thresholds
        self.drawer_open_threshold = 0.12
        self.drawer_closed_threshold = 0.02
        self.bowl_in_drawer_threshold = 0.08

        # Stage timers
        self.task_start_time = None
        self.stage1_complete_time = None
        self.stage2_complete_time = None
        self.stage3_complete_time = None


    def setup(self):
        self.drawer_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "drawer_slide")
        self.bowl_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "bowl")
        self.drawer_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "drawer")
        self.table_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "drawer_table_surface")

        self.task_start_time = time.time()
        print(f"DrawerTask ready - Timer started")

    def get_drawer_opening(self):
        """Returns current drawer opening distance in metres"""
        qpos_addr = self.model.jnt_qposadr[self.drawer_joint_id]
        return self.data.qpos[qpos_addr]

    def is_bowl_in_drawer(self):
        """Check if bowl is inside the drawer"""
        bowl_pos = self.data.xpos[self.bowl_body_id].copy()
        drawer_pos = self.data.xpos[self.drawer_body_id].copy()
       
        # Check horizontal distance and height
        xy_dist = np.linalg.norm(bowl_pos[:2] - drawer_pos[:2])
        height_ok = abs(bowl_pos[2] - drawer_pos[2]) < 0.08
       
        return xy_dist < self.bowl_in_drawer_threshold and height_ok

    def step(self, ee_body_id):
        if self.completed:
            return

        drawer_open = self.get_drawer_opening()
        bowl_in_drawer = self.is_bowl_in_drawer()
        cur_time = time.time()

        if self.stage == 0:
            # Stage 1: Open the drawer
            if drawer_open > self.drawer_open_threshold:
                self.stage = 1
                self.stage1_complete_time = cur_time - self.task_start_time
                print("Stage 1 complete! Time: {self.stage1_complete_time:.1f} seconds")

        elif self.stage == 1:
            # Stage 2: Place bowl in drawer
            if bowl_in_drawer and drawer_open > self.drawer_open_threshold:
                self.stage = 2
                self.stage2_complete_time = cur_time - self.task_start_time
                print("Stage 2 complete! Time: {self.stage2_complete_time:.1f} seconds")

        elif self.stage == 2:
            # Stage 3: Close drawer with bowl inside
            if bowl_in_drawer and drawer_open < self.drawer_closed_threshold:
                self.completed = True
                self.stage3_complete_time = cur_time - self.task_start_time
                print("Task complete! Time: {self.stage3_complete_time:.1f} seconds")

    def get_contact_geoms(self):
        return []

    def get_status(self):
        if self.completed:
            return "Complete! Drawer closed with bowl!"

        drawer_open = self.get_drawer_opening()

        if self.stage == 0:
            return f"Stage 1/3: Open the drawer ({drawer_open*100:.1f}cm / {self.drawer_open_threshold*100:.0f}cm)"
        elif self.stage == 1:
            bowl_in = self.is_bowl_in_drawer()
            return f"Stage 2/3: Place bowl in drawer (bowl in drawer: {bowl_in})"
        elif self.stage == 2:
            return f"Stage 3/3: Close the drawer ({drawer_open*100:.1f}cm open)"
        return "Unknown stage"