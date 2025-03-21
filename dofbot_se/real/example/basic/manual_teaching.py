import threading
import time

import keyboard
import numpy as np
import roboticstoolbox as rtb
from Arm_Lib import Arm_Device
from roboticstoolbox import DHRobot, RevoluteDH

# === Global Variables ===
running = False
all_angle_lists = []
main_thread_event = threading.Event()
plot_data = None

Arm = Arm_Device(com="COM13")
time.sleep(1)
Arm.Arm_serial_set_torque(0)

# === Robot Model ===
DFbot = DHRobot(
    [
        RevoluteDH(d=0.04145, alpha=np.pi / 2, qlim=np.array([-np.pi / 2, np.pi / 2])),
        RevoluteDH(a=-0.08285, qlim=np.array([-np.pi, 0])),
        RevoluteDH(a=-0.08285, qlim=np.array([-np.pi / 2, np.pi / 2])),
        RevoluteDH(alpha=-np.pi / 2, qlim=np.array([0, np.pi])),
        RevoluteDH(d=0.11, qlim=np.array([-np.pi / 2, np.pi])),
    ],
    name="DFbot",
)


# === Real2Sim Conversion ===
def convert_real_to_sim(real_deg):
    s1 = real_deg[0] - 90
    s2 = -real_deg[1]
    s3 = 90 - real_deg[2]
    s4 = 180 - real_deg[3]
    s5 = real_deg[4] - 90
    return [s1, s2, s3, s4, s5]


def convert_sim_to_real(sim_deg):
    d1 = sim_deg[0] + 90
    d2 = -sim_deg[1]
    d3 = 90 - sim_deg[2]
    d4 = 180 - sim_deg[3]
    d5 = sim_deg[4] + 90
    return [d1, d2, d3, d4, d5]


# === Data Collection Thread ===
def run_task():
    global running

    Arm.Arm_serial_set_torque(0)
    while True:
        if running:
            angle_list = []
            for i in range(1, 7):
                angle = Arm.Arm_serial_servo_read(i)
                angle_list.append(angle)
                time.sleep(0.1)
            print(f"Recording: {angle_list}")
            all_angle_lists.append(angle_list)
            time.sleep(2)
        else:
            time.sleep(0.1)


# === After Stop Task (in Worker Thread) ===
def after_stop_task():
    global plot_data, main_thread_event, Arm

    Arm.Arm_serial_set_torque(1)

    # Drop invalid frames (with None)
    trimmed_angle_lists = [
        angles[:5] for angles in all_angle_lists
        if None not in angles[:5]
    ]

    if len(trimmed_angle_lists) < 2:
        print("Not enough data to process.")
        return

    # Real to Sim (degrees)
    sim_angle_lists = [convert_real_to_sim(angle) for angle in trimmed_angle_lists]
    sim_angle_lists = np.array(sim_angle_lists)

    # Sim degrees to radians
    sim_angle_lists_in_radians = np.deg2rad(sim_angle_lists)

    # Trajectory interpolation
    interpolated_trajectories = []
    for i in range(len(sim_angle_lists_in_radians) - 1):
        traj = rtb.tools.trajectory.jtraj(sim_angle_lists_in_radians[i], sim_angle_lists_in_radians[i + 1], 10)
        interpolated_trajectories.extend(traj.q)

    plot_data = np.array(interpolated_trajectories)

    # Notify main thread to plot
    main_thread_event.set()

    for q in interpolated_trajectories:
        sim_deg = np.rad2deg(q)
        real_deg = convert_sim_to_real(sim_deg)

        Arm.Arm_serial_servo_write6(
            int(real_deg[0]),
            int(real_deg[1]),
            int(real_deg[2]),
            int(real_deg[3]),
            int(real_deg[4]),
            0,  # 6th fixed at 0
            800  # Move duration
        )
        time.sleep(0.1)

    # Clear recorded data for next time
    all_angle_lists.clear()

    Arm.Arm_serial_set_torque(0)

    print("Finished")


# === Main Thread Plotter ===
def main_thread_plot_loop():
    global plot_data

    while True:
        main_thread_event.wait()
        if plot_data is not None:
            DFbot.plot(plot_data, backend='pyplot', block=True)
            plot_data = None
        main_thread_event.clear()


# === Toggle Run State ===
def toggle_run():
    global running

    if running:
        running = False
        print("Recording stopped.")
        after_stop_task()
    else:
        running = True
        all_angle_lists.clear()
        print("Recording started.")


# === Main Program ===
if __name__ == '__main__':
    # Space key to start/stop
    keyboard.add_hotkey("space", toggle_run)

    # Start worker thread (data collection)
    threading.Thread(target=run_task, daemon=True).start()

    # Start plot listener (main thread safe)
    threading.Thread(target=main_thread_plot_loop, daemon=True).start()

    print("Press SPACE to start/stop recording. Press Ctrl+C to exit.")

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nExiting.")
