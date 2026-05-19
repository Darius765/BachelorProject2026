import numpy as np
import mujoco
from tasks.base_task import BaseTask

class WipeTask(BaseTask):
    def __init__(self, model, data, num_markers=10):
        super().__init__(model, data)
        self.num_markers = num_markers
        self.marker_ids = []
        self.marker_active = []
        self.wiped_count = 0
        self.table_geom_id = -1

    def setup(self):
        # Get table geom
        self.table_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "wipe_table_surface")
        self.left_finger_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "left_finger")
        self.right_finger_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "right_finger")

        for i in range(self.num_markers):
            bid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, f"marker_{i}")
            if bid >= 0:
                self.marker_ids.append(bid)
                self.marker_active.append(True)

        for i in range(self.model.ngeom):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i)
            if name and "marker" in name:
                print(f"Marker geom: {name}, ID: {i}")

        print(f"WipeTask ready: {len(self.marker_ids)} markers to wipe")

    def step(self, ee_body_id):
        self.left_finger_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "left_finger")
        self.right_finger_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "right_finger")

        ee_pos = self.data.xpos[ee_body_id].copy()

        for i, (bid, active) in enumerate(zip(self.marker_ids, self.marker_active)):
            if active:
                marker_pos = self.data.xpos[bid].copy()
                dist = np.linalg.norm(ee_pos[:2] - marker_pos[:2])
                if dist < 0.01:
                    self.marker_active[i] = False
                    self.wiped_count += 1

                    gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, f"marker_{i}_geom")
                    print(f"Hiding marker {i}, geom ID: {gid}")
                    if gid >= 0:
                        self.model.geom_rgba[gid][3] = 0.0
                        print(f"Set alpha to 0 for geom {gid}")
                    print(f"Wiped marker {i}! ({self.wiped_count}/{len(self.marker_ids)})")

        if self.wiped_count >= len(self.marker_ids):
            self.completed = True

    def get_contact_geoms(self):
        return [self.table_geom_id]

    def get_status(self):
        return f"Wiped: {self.wiped_count}/{len(self.marker_ids)}"