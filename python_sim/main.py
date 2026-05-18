import mujoco
import mujoco.viewer
import numpy as np
import asyncio
import websockets
import json
import threading
import queue

# ── Configuration ────────────────────────────────────────────
MODEL_PATH = "../models/panda.xml"
HAPLY_WS_URL = "ws://localhost:10001"
HAPLY_DEVICE_ID = "05DA"

# ── Shared state between threads ─────────────────────────────
haply_state = {
    "position": np.array([0.0, 0.0, 0.0]),
    "orientation": np.array([0.0, 0.0, 0.0, 1.0]),  # x y z w
    "has_position": False,
    "has_orientation": False,
}
force_command = {"x": 0.0, "y": 0.0, "z": 0.0}
state_lock = threading.Lock()

# ── Load MuJoCo model ────────────────────────────────────────
model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# Set ready pose
data.qpos[:7] = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
mujoco.mj_forward(model, data)

ee_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand")
q_arm_init = data.xquat[ee_body].copy()

print("Model loaded successfully")
print(f"Number of joints: {model.njnt}")

# ── Haply WebSocket client ───────────────────────────────────
async def haply_client():
    async with websockets.connect(HAPLY_WS_URL) as ws:
        print("Connected to Haply service")
        while True:
            # Send force command
            with state_lock:
                force = force_command.copy()
           
            msg = json.dumps({
                "inverse3": [{
                    "device_id": HAPLY_DEVICE_ID,
                    "commands": {
                        "set_cursor_force": {"vector": force},
                        "set_cursor_position": {"vector": {"x": 0.0, "y": -0.15, "z": 0.19}}
                    }
                }]
            })
            await ws.send(msg)

            # Receive state
            try:
                response = await asyncio.wait_for(ws.recv(), timeout=0.005)
                data_json = json.loads(response)

                with state_lock:
                    # Parse Inverse3 position
                    inv3 = data_json.get("inverse3", [])
                    if inv3:
                        pos = inv3[0]["state"]["cursor_position"]
                        vel = inv3[0]["state"]["cursor_velocity"]
                        haply_state["position"] = np.array([pos["x"], pos["y"], pos["z"]])
                        haply_state["velocity"] = np.array([vel["x"], vel["y"], vel["z"]])
                        haply_state["has_position"] = True

                    # Parse VerseGrip orientation
                    grip = data_json.get("wireless_verse_grip", [])
                    if grip:
                        ori = grip[0]["state"]["orientation"]
                        haply_state["orientation"] = np.array([ori["x"], ori["y"], ori["z"], ori["w"]])
                        haply_state["has_orientation"] = True

            except asyncio.TimeoutError:
                pass

def start_haply_thread():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(haply_client())

# Start Haply client in background thread
haply_thread = threading.Thread(target=start_haply_thread, daemon=True)
haply_thread.start()

print("Waiting for Haply data...")
import time
while not haply_state["has_orientation"] or not haply_state["has_position"]:
    time.sleep(0.01)
print("Haply connected!")
print(f"Initial position: {haply_state['position']}")
print(f"Initial orientation: {haply_state['orientation']}")

# ── Coordinate transform ─────────────────────────────────────
neutral_pos = haply_state["position"].copy()
ref_orientation = haply_state["orientation"].copy()  # x y z w

# Franka home position
home_pos = np.array([0.304, 0.000, 0.644])
scale = 0.5

def transform_position(haply_pos):
    rel = haply_pos - neutral_pos
    # Axis mapping: Haply x -> Franka -x, Haply y -> Franka -y, Haply z -> Franka z
    return home_pos + np.array([-rel[0], -rel[1], rel[2]]) * scale

def transform_orientation(haply_ori):
    # haply_ori is x y z w
    q_current = np.array([haply_ori[3], haply_ori[0], haply_ori[1], haply_ori[2]])
    q_ref = np.array([ref_orientation[3], ref_orientation[0], ref_orientation[1], ref_orientation[2]])
   
    # Compute relative rotation
    def quat_multiply(q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])
   
    def quat_inverse(q):
        w, x, y, z = q
        return np.array([w, -x, -y, -z])
   
    q_relative = quat_multiply(q_current, quat_inverse(q_ref))
    q_target = quat_multiply(q_relative, q_arm_init)
    q_target /= np.linalg.norm(q_target)
    return q_target

print(f"Neutral position set: {neutral_pos}")

# ── IK solver ────────────────────────────────────────────────
ee_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand")
table_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "table_surface")
box_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "box_geom")

# Joint limits
joint_min = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
joint_max = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])

target_qpos = data.qpos[:7].copy()

def solve_ik(target_pos, target_quat_wxyz, dt=0.01):
    global target_qpos
   
    # Position error
    cur_pos = data.xpos[ee_body].copy()
    pos_error = target_pos - cur_pos
    pos_mag = np.linalg.norm(pos_error)
    if pos_mag > 0.1:
        pos_error *= 0.1 / pos_mag

    # Orientation error
    cur_quat = data.xquat[ee_body].copy()  # w x y z
    q_cur = np.quaternion(*cur_quat) if hasattr(np, 'quaternion') else None
   
    # Manual quaternion error
    w1, x1, y1, z1 = target_quat_wxyz
    w2, x2, y2, z2 = cur_quat
    # q_err = q_target * q_cur_inverse
    q_err = np.array([
        w1*w2 + x1*x2 + y1*y2 + z1*z2,
        w1*(-x2) + x1*w2 + y1*(-z2) - z1*(-y2),
        w1*(-y2) - x1*(-z2) + y1*w2 + z1*(-x2),
        w1*(-z2) + x1*(-y2) - y1*(-x2) + z1*w2
    ])
    ori_error = q_err[1:]  # x y z
    if q_err[0] < 0:
        ori_error = -ori_error
    ori_mag = np.linalg.norm(ori_error)
    if ori_mag > 0.3:
        ori_error *= 0.3 / ori_mag

    # Full error vector
    error = np.concatenate([pos_error, ori_error])

    # Jacobian
    nv = model.nv
    jacp = np.zeros((3, nv))
    jacr = np.zeros((3, nv))
    mujoco.mj_jacBody(model, data, jacp, jacr, ee_body)
    J = np.vstack([jacp, jacr])

    # Damped pseudoinverse
    damping = 0.05
    U, s, Vt = np.linalg.svd(J, full_matrices=False)
    s_inv = s / (s**2 + damping**2)
    J_pinv = Vt.T @ np.diag(s_inv) @ U.T

    # Joint corrections
    dq = J_pinv @ error
    target_qpos = target_qpos + dq[:7] * dt
    target_qpos = np.clip(target_qpos, joint_min, joint_max)

# ── PD Controller ────────────────────────────────────────────
def pd_control():
    kp, kd = 300.0, 30.0
    for i in range(7):
        pos_err = target_qpos[i] - data.qpos[i]
        vel_err = -data.qvel[i]
        data.ctrl[i] = kp * pos_err + kd * vel_err + data.qfrc_bias[i]

# ── Main simulation loop ─────────────────────────────────────
with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = False
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = False
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_ACTUATOR] = False
    # viewer.cam.distance = 2.0
    # viewer.cam.elevation = -20
    # viewer.cam.azimuth = 135

    print("Simulation running!")
    while viewer.is_running():
        # Get Haply data
        with state_lock:
            haply_pos = haply_state["position"].copy()
            haply_ori = haply_state["orientation"].copy()
            has_pos = haply_state["has_position"]
            has_ori = haply_state["has_orientation"]

        if has_pos and has_ori:
            franka_pos = transform_position(haply_pos)
            franka_quat = transform_orientation(haply_ori)

            solve_ik(franka_pos, franka_quat)

        pd_control()

        # Contact forces
        contact_force = np.zeros(3)
        smoothing_factor = 0.2
        smooth_force = np.zeros(3)
        for i in range(data.ncon):
            con = data.contact[i]
            if con.geom1 == table_geom or con.geom2 == table_geom or \
               con.geom1 == box_geom or con.geom2 == box_geom:
                force = np.zeros(6)
                mujoco.mj_contactForce(model, data, i, force)
                contact_force += force[:3]

        # Apply smoothing
        smooth_force = smooth_force * (1 - smoothing_factor) + contact_force * smoothing_factor

        force_scale = 0.05 # min 0.005 max 0.05 tried
        max_force = 1.0 # min 1.0 max 5.0 tried

        if np.linalg.norm(smooth_force) > 0.1:

            with state_lock:
                cursor_velocity = haply_state.get("velocity", np.zeros(3))

            franka_vel = np.array([-cursor_velocity[0], -cursor_velocity[1], cursor_velocity[2]])

            force_direction = smooth_force / (np.linalg.norm(smooth_force) + 1e-6)
            moving_into = np.dot(franka_vel, force_direction)

            if moving_into > 0:
                fx = float(-smooth_force[0] * force_scale)
                fy = float(-smooth_force[1] * force_scale)
                fz = float(smooth_force[2] * force_scale)

                fx = max(-max_force, min(max_force, fx))
                fy = max(-max_force, min(max_force, fy))
                fz = max(-max_force, min(max_force, fz))
            else:
                fx = fy = fz = 0.0
        else:
            fx = fy = fz = 0.0

        # Send force feedback
        with state_lock:
            force_command["x"] = fx
            force_command["y"] = fy
            force_command["z"] = fz

        # Step simulation
        mujoco.mj_step(model, data)
        viewer.sync()