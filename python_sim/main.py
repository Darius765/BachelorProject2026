import mujoco
import mujoco.viewer
import numpy as np
import asyncio
import websockets
import json
import threading
import time
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from tasks.wipe_task import WipeTask
from tasks.nut_pole_task import NutAssemblyTask
from tasks.drawer_task import DrawerTask
from safety import SafetyLimits

# ── Configuration ────────────────────────────────────────────
WIPE_MODEL_PATH = "../models/panda_wipe.xml"
NUT_MODEL_PATH = "../models/panda_nut.xml"
DRAWER_MODEL_PATH = "../models/panda_drawer.xml"
HAPLY_WS_URL = "ws://localhost:10001"
HAPLY_DEVICE_ID = "05DA"

task_choice = "nut"  # "wipe", "nut", or "drawer"

# ── Shared state between threads ─────────────────────────────
haply_state = {
    "position": np.array([0.0, 0.0, 0.0]),
    "orientation": np.array([0.0, 0.0, 0.0, 1.0]),
    "velocity": np.zeros(3),
    "has_position": False,
    "has_orientation": False,
}
force_command = {"x": 0.0, "y": 0.0, "z": 0.0}
state_lock = threading.Lock()

# ── Load MuJoCo model ────────────────────────────────────────
match task_choice:
    case "wipe":
        model = mujoco.MjModel.from_xml_path(WIPE_MODEL_PATH)
    case "nut":
        model = mujoco.MjModel.from_xml_path(NUT_MODEL_PATH)
    case "drawer":
        model = mujoco.MjModel.from_xml_path(DRAWER_MODEL_PATH)
    case _:
        print(f"Unknown task choice: {task_choice}")
        sys.exit(1)

data = mujoco.MjData(model)

data.qpos[:7] = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
mujoco.mj_forward(model, data)

ee_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand")
q_arm_init = data.xquat[ee_body].copy()

print("Model loaded successfully")

for i in range(model.nu):
    joint_id = model.actuator_trnid[i, 0]
    joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
    print(f"Actuator {i}: controls joint '{joint_name}'")

safety = SafetyLimits(model, data)


# ── Task setup ─────────────────────────────────────────────────
match task_choice:
    case "wipe":
        task = WipeTask(model, data, num_markers=10)
    case "nut":
        task = NutAssemblyTask(model, data)
    case "drawer":
        task = DrawerTask(model, data)
    case _:
        print(f"Unknown task choice: {task_choice}")
        sys.exit(1)

task.setup()

# ── Haply WebSocket client ───────────────────────────────────
async def haply_client():
    async with websockets.connect(HAPLY_WS_URL) as ws:
        print("Connected to Haply service")
        while True:
            with state_lock:
                force = force_command.copy()

            msg = json.dumps({
                "inverse3": [{
                    "device_id": HAPLY_DEVICE_ID,
                    "commands": {
                        "set_cursor_force": {"vector": force}
                    }
                }]
            })
            await ws.send(msg)

            try:
                response = await asyncio.wait_for(ws.recv(), timeout=0.005)
                data_json = json.loads(response)

                with state_lock:
                    inv3 = data_json.get("inverse3", [])
                    if inv3:
                        pos = inv3[0]["state"]["cursor_position"]
                        vel = inv3[0]["state"]["cursor_velocity"]
                        haply_state["position"] = np.array([pos["x"], pos["y"], pos["z"]])
                        haply_state["velocity"] = np.array([vel["x"], vel["y"], vel["z"]])
                        haply_state["has_position"] = True

                    grip = data_json.get("wireless_verse_grip", [])
                    if grip:
                        ori = grip[0]["state"]["orientation"]
                        haply_state["orientation"] = np.array([ori["x"], ori["y"], ori["z"], ori["w"]])
                        haply_state["has_orientation"] = True
                        buttons = grip[0]["state"]["buttons"]
                        haply_state["button_a"] = buttons.get("a", False)
                        haply_state["button_b"] = buttons.get("b", False)
                        haply_state["button_c"] = buttons.get("c", False)

            except asyncio.TimeoutError:
                pass

def start_haply_thread():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(haply_client())

haply_thread = threading.Thread(target=start_haply_thread, daemon=True)
haply_thread.start()

print("Waiting for Haply data...")
while not haply_state["has_orientation"] or not haply_state["has_position"]:
    time.sleep(0.01)
print("Haply connected!")
print(f"Initial position: {haply_state['position']}")
print(f"Initial orientation: {haply_state['orientation']}")

# ── Coordinate transform ─────────────────────────────────────
neutral_pos = haply_state["position"].copy()
ref_orientation = haply_state["orientation"].copy()
clutch_neutral_pos = neutral_pos.copy()
clutch_ref_ori = ref_orientation.copy()
clutch_saved_qpos = None
home_pos = np.array([0.304, 0.000, 0.65])
scale = 1.0

def transform_position(haply_pos):
    rel = haply_pos - clutch_neutral_pos
    return home_pos + np.array([rel[1], -rel[0], rel[2]]) * scale

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

def transform_orientation(haply_ori):
    q_current = np.array([haply_ori[3], haply_ori[0], haply_ori[1], haply_ori[2]])
    q_ref = np.array([clutch_ref_ori[3], clutch_ref_ori[0], clutch_ref_ori[1], clutch_ref_ori[2]])
    q_relative = quat_multiply(q_current, quat_inverse(q_ref))
    q_target = quat_multiply(q_relative, q_arm_init)
    q_target /= np.linalg.norm(q_target)
    return q_target

print(f"Neutral position set: {neutral_pos}")

# ── IK solver ────────────────────────────────────────────────
joint_min = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
joint_max = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])
target_qpos = data.qpos[:7].copy()

def solve_ik(target_pos, target_quat_wxyz, dt=0.01):
    global target_qpos

    cur_pos = data.xpos[ee_body].copy()
    pos_error = target_pos - cur_pos
    pos_mag = np.linalg.norm(pos_error)
    if pos_mag > 0.1:
        pos_error *= 0.1 / pos_mag

    cur_quat = data.xquat[ee_body].copy()
    w1, x1, y1, z1 = target_quat_wxyz
    w2, x2, y2, z2 = cur_quat
    q_err = np.array([
        w1*w2 + x1*x2 + y1*y2 + z1*z2,
        w1*(-x2) + x1*w2 + y1*(-z2) - z1*(-y2),
        w1*(-y2) - x1*(-z2) + y1*w2 + z1*(-x2),
        w1*(-z2) + x1*(-y2) - y1*(-x2) + z1*w2
    ])
    ori_error = q_err[1:]
    if q_err[0] < 0:
        ori_error = -ori_error
    ori_mag = np.linalg.norm(ori_error)
    if ori_mag > 0.2:
        ori_error *= 0.2 / ori_mag

    error = np.concatenate([pos_error, ori_error])

    nv = model.nv
    jacp = np.zeros((3, nv))
    jacr = np.zeros((3, nv))
    mujoco.mj_jacBody(model, data, jacp, jacr, ee_body)
    J = np.vstack([jacp, jacr])

    damping = 0.05
    U, s, Vt = np.linalg.svd(J, full_matrices=False)
    s_inv = s / (s**2 + damping**2)
    J_pinv = Vt.T @ np.diag(s_inv) @ U.T

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

# ── Force feedback state (outside loop so smoothing persists) ─
smooth_force = np.zeros(3)
smoothing_factor = 0.2
force_scale = 0.02
max_force = 1.0
smooth_ori = haply_state["orientation"].copy()
clutch_saved_smooth_ori = smooth_ori.copy()
alpha_ori = 0.1

# ── Keyboard interrupts ─────────────────────────────────────────
def key_callback(keycode):
    if keycode == ord(''):
        safety.trigger_estop("Spacebar pressed")
    elif keycode == ord('r'):
        safety.reset_estop()
        print("ESTOP reset")

# ── Main simulation loop ─────────────────────────────────────
prev_time = time.time()
time_interval = 0.5

with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = False
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = False
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_ACTUATOR] = False

    clutch_active = False
    prev_clutch = False

    print("Simulation running!")
    while viewer.is_running():
        # Get Haply data
        with state_lock:
            haply_pos = haply_state["position"].copy()
            haply_ori = haply_state["orientation"].copy()
            cursor_velocity = haply_state.get("velocity", np.zeros(3))
            has_pos = haply_state["has_position"]
            has_ori = haply_state["has_orientation"]

        if has_pos and has_ori and not safety.is_estop():
            if clutch_active and clutch_saved_qpos is not None:
                target_qpos = clutch_saved_qpos.copy()
            else:
                smooth_ori = smooth_ori * (1 - alpha_ori) + haply_ori * alpha_ori
                smooth_ori /= np.linalg.norm(smooth_ori)
                franka_pos = transform_position(haply_pos)
                franka_quat = transform_orientation(smooth_ori)
                solve_ik(franka_pos, franka_quat)
                
        nv = model.nv
        jacp = np.zeros((3, nv))
        jacr = np.zeros((3, nv))
        mujoco.mj_jacBody(model, data, jacp, jacr, ee_body)

        qfrc_ext = data.qfrc_constraint.copy()

        ee_force = jacp @ qfrc_ext
        ee_torque = jacr @ qfrc_ext

        if task_choice == "nut":
            nut_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "nut")
            peg_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "peg1")
            force_world = np.zeros(3)

            for i in range(data.ncon):
                con = data.contact[i]
                geom1_body = model.geom_bodyid[con.geom1]
                geom2_body = model.geom_bodyid[con.geom2]
                if (geom1_body == nut_body_id and geom2_body == peg_body_id) or \
                    (geom2_body == nut_body_id and geom1_body == peg_body_id):
                    nut_force = np.zeros(6)
                    mujoco.mj_contactForce(model, data, i, nut_force)

                    frame = con.frame.reshape(3, 3)
                    force_world = frame.T @ nut_force[:3]

            if np.linalg.norm(force_world) > 0.01:
                amplified_force = force_world * 500.0
                ee_force += amplified_force

        # Safety checks
        safe_qpos, safety_status = safety.apply(target_qpos, ee_body, ee_force, ee_torque)
        target_qpos = safe_qpos

        if safety_status["estop"]:
            pass
        elif safety_status["workspace_violation"]:
            print("Workspace limit violated! Holding position.")
        elif safety_status["joint_limit"]:
            print("Joint limit violated! Holding position.")

        # Gripper control
        with state_lock:
            btn_a = haply_state.get("button_a", False)
            btn_b = haply_state.get("button_b", False)
            btn_c = haply_state.get("button_c", False)

        if btn_c and not prev_clutch:
            clutch_active = True
            clutch_saved_qpos = target_qpos.copy()
            clutch_saved_smooth_ori = smooth_ori.copy()
            print("Clutch engaged")

        elif not btn_c and prev_clutch:
            clutch_active = False
            clutch_neutral_pos = haply_pos.copy()
            clutch_ref_ori = clutch_saved_smooth_ori.copy()
            home_pos = data.xpos[ee_body].copy()
            print("Clutch released")

        prev_clutch = btn_c
        
        gripper_open = 255
        gripper_closed = 0.0

        if btn_a:
            data.ctrl[7] = gripper_open
        elif btn_b:
            data.ctrl[7] = gripper_closed

        pd_control()

        # Task update
        task.step(ee_body)

        if np.linalg.norm(ee_force) > 0.5:
            smooth_force = smooth_force * (1 - smoothing_factor) + ee_force * smoothing_factor
        else:
            smooth_force = smooth_force * (1 - smoothing_factor)

        # Apply force feedback
        if np.linalg.norm(smooth_force) > 0.1: 
            fx = float(-smooth_force[0] * force_scale)
            fy = float(-smooth_force[1] * force_scale)
            fz = float(smooth_force[2] * force_scale)
            fx = max(-max_force, min(max_force, fx))
            fy = max(-max_force, min(max_force, fy))
            fz = max(-max_force, min(max_force, fz))
        else:
            fx = fy = fz = 0.0

        with state_lock:
            force_command["x"] = fx
            force_command["y"] = fy
            force_command["z"] = fz

        # Step simulation
        mujoco.mj_step(model, data)
        viewer.sync()

        # Periodic status print
        cur_time = time.time()
        if cur_time - prev_time >= time_interval:
            prev_time = cur_time
            if safety.is_estop():
                print("ESTOP ENGAGED! Holding position.")
            if task.is_complete():
                print("Task complete!")