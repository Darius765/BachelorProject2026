import numpy as np
import mujoco
from tasks.base_task import BaseTask

class WipeTask(BaseTask):
    def __init__(self, model, data, num_markers=20):
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

        # Get marker body and geom IDs
        for i in range(self.num_markers):
            bid = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, f"marker_{i}")
            if bid >= 0:
                self.marker_ids.append(bid)
                self.marker_active.append(True)

        print(f"WipeTask ready: {len(self.marker_ids)} markers to wipe")

    def step(self, ee_body_id):
        ee_pos = self.data.xpos[ee_body_id].copy()

        for i, (bid, active) in enumerate(zip(self.marker_ids, self.marker_active)):
            if active:
                marker_pos = self.data.xpos[bid].copy()
                dist = np.linalg.norm(ee_pos - marker_pos)
                if dist < 0.05:
                    self.marker_active[i] = False
                    self.wiped_count += 1
                    # Hide marker
                    gid = mujoco.mj_name2id(
                        self.model, mujoco.mjtObj.mjOBJ_GEOM, f"marker_{i}_geom")
                    if gid >= 0:
                        self.model.geom_rgba[gid][3] = 0.0
                    print(f"Wiped marker {i}! ({self.wiped_count}/{len(self.marker_ids)})")

        if self.wiped_count >= len(self.marker_ids):
            self.completed = True
            print("Task complete! All markers wiped!")

    def get_contact_geoms(self):
        return [self.table_geom_id]

    def get_status(self):
        return f"Wiped: {self.wiped_count}/{len(self.marker_ids)}"