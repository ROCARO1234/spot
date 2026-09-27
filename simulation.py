"""Offline trajectory validation and optional matplotlib kinematic preview."""
from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np

from controller import MotionController
from gait_core import convex_hull
from robot_config import JOINTS, LEGS, SafetyError, finite_number
from robot_io import DryRunOutput


def simulate(config, command=(.003, 0., 0.), duration=12., initial_pose="STAND"):
    finite_number(duration, "duration", .02, 300.)
    controller = MotionController(config, initial_pose)
    controller.set_command(*command)
    output = DryRunOutput(config)
    output.open(controller.last_pose)
    dt = 1 / config.control["rate_hz"]
    records = []
    def record(t):
        frame = controller.frame
        records.append({"time": t, "pose": controller.last_pose.copy(),
                        "feet": {leg: p.copy() for leg, p in frame.feet.items()},
                        "body_shift": frame.body_shift.copy(), "swing": frame.swing_legs,
                        "state": controller.state, "velocity": frame.effective_velocity.copy(),
                        "margin": frame.support_margin_m, "tracking": controller.tracking_error_deg})
    record(0.)
    deadline = duration + 3*controller.planner.cycle_time + 2*config.control["pose_transition_s"]
    try:
        for step in range(1, math.ceil(deadline/dt) + 1):
            t = step*dt
            if (step-1)*dt >= duration - 1e-10:
                controller.set_command()
            pose = controller.update(dt)
            output.write(pose, dt)
            record(t)
            if t >= duration and controller.settled:
                break
        else:
            raise SafetyError("Normal stop did not settle within the simulation deadline")
    finally:
        output.close()
    return records


def summary(records):
    return {"frames": len(records), "simulated_seconds": round(records[-1]["time"], 3),
            "final_state": records[-1]["state"],
            "max_swing_legs": max(len(r["swing"]) for r in records),
            "min_servo_deg": round(min(min(r["pose"].values()) for r in records), 4),
            "max_servo_deg": round(max(max(r["pose"].values()) for r in records), 4),
            "max_tracking_error_deg": round(max(r["tracking"] for r in records), 5),
            "min_model_support_margin_m": round(min(r["margin"] for r in records), 5)}


def write_csv(records, path):
    path = Path(path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    header = ["time_s", "state", "swing_legs", "model_support_margin_m", "tracking_error_deg",
              "body_shift_x_m", "body_shift_y_m", "effective_vx_mps", "effective_vy_mps", "effective_wz_rps"]
    header += [j + "_deg" for j in JOINTS]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for r in records:
            writer.writerow([r["time"], r["state"], "+".join(r["swing"]), r["margin"], r["tracking"],
                             *r["body_shift"][:2], *r["velocity"], *[r["pose"][j] for j in JOINTS]])
    return path


def preview(config, records, *, output=None, animate=False, live_source=None):
    # GUI dependencies are not imported during hardware control or dry-run.
    import matplotlib
    if output and not animate:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    from gait_core import IKBinding
    binding = IKBinding(config)
    colors = {"FL": "#078b83", "FR": "#e6a32c", "RL": "#4074cf", "RR": "#c96082"}
    fig = plt.figure(figsize=(14, 7.2), facecolor="#f5f7fa")
    ax = fig.add_subplot(121, projection="3d")
    top = fig.add_subplot(122)
    fig.subplots_adjust(top=.78, bottom=.14, left=.055, right=.97, wspace=.38)
    fig.suptitle("SM5 | Comenzi live" if live_source else "SM5 | Previzualizare crawl",
                 x=.06, y=.965, ha="left", fontsize=20, fontweight="bold")
    fig.text(.06, .91 if live_source else .90,
             "DOAR AFIȘARE  •  Poziții comandate, nu unghiuri măsurate și nu imagine de cameră" if live_source else
             "SIMULARE FĂRĂ MOTOARE  •  Unghiurile de ieșire sunt reconstruite prin FK",
             fontsize=10, color="#435169")
    connection = fig.text(.06, .87, "", fontsize=10)
    status = fig.text(.06, .825 if live_source else .84, "", fontsize=11)
    if live_source:
        fig.text(.06, .073, "Comenzile se dau în terminalul runnerului. Închiderea acestei ferestre NU oprește motoarele.",
                 fontsize=10, color="#96342c")
    fig.text(.06, .035, "Model geometric, fără fizică sau contacte măsurate. Stabilitatea pe robot rămâne de verificat.",
             fontsize=9, color="#435169")

    def draw_record(r):
        ax.clear(); top.clear()
        joints = binding.forward(r["pose"], r["body_shift"])
        shift = np.asarray(r["body_shift"])
        support_valid = r.get("support_model_valid", True)
        order = ("FL", "FR", "RR", "RL", "FL")
        body = np.array([config.hips[leg] + shift for leg in order])
        ax.plot(body[:, 0], body[:, 1], body[:, 2], color="#273b59", linewidth=3)
        for leg in LEGS:
            p = joints[leg]
            ax.plot(p[:, 0], p[:, 1], p[:, 2], "o-", color=colors[leg], linewidth=2.4, markersize=4, label=leg)
            top.plot(p[:, 0], p[:, 1], "o-", color=colors[leg], linewidth=1.5, markersize=3)
            if support_valid:
                top.scatter(*r["feet"][leg][:2], marker="x", color=colors[leg], s=30)
            top.annotate(leg, p[-1, :2], xytext=(5, 7), textcoords="offset points", fontsize=9)
        if support_valid:
            contacts = [r["feet"][leg][:2] for leg in LEGS if leg not in r["swing"]]
            hull = convex_hull(contacts)
            top.add_patch(Polygon(hull, closed=True, facecolor="#d1eadf", edgecolor="#609c83", alpha=.7))
        top.plot(body[:, 0], body[:, 1], color="#273b59", linewidth=2.5)
        com = shift[:2] + np.asarray(config.gait["com_offset_xy_m"])
        top.scatter(*com, color="#273b59", s=65, zorder=5, label="COM aproximat")
        top.scatter(0, 0, marker="+", color="#929aab", s=50)
        ax.set(xlim=(-.39, .30), ylim=(-.25, .25), zlim=(-.26, .06),
               xlabel="X înainte (m)", ylabel="Y stânga (m)", zlabel="Z (m)")
        ax.set_box_aspect((.69, .50, .32))
        ax.view_init(elev=24, azim=-57)
        ax.legend(loc="lower left", ncol=4, fontsize=8)
        top.set(xlim=(-.39, .30), ylim=(-.25, .25), xlabel="X înainte (m)", ylabel="Y stânga (m)")
        top.set_aspect("equal")
        top.grid(alpha=.16)
        top.set_title("Vedere de sus · sprijin presupus" if support_valid else "Vedere de sus · poziție / tranziție", fontsize=12)
        top.legend(loc="lower right", fontsize=8)
        swing = ", ".join(r["swing"]) or "niciunul"
        detail = f"Picior ridicat: {swing}   |   Marjă model: {r['margin']*1000:.0f} mm" if support_valid else "Contactele nu sunt evaluate în această poziție"
        status.set_text(f"t = {r['time']:.2f} s   |   {r['state']}   |   {detail}")
        return []

    def draw(index):
        return draw_record(records[index])

    previous_live = [None]
    def draw_live(_):
        record, label, fresh = live_source()
        connection.set_text(label)
        connection.set_color("#087453" if fresh else "#a5342f")
        if record is not None and record is not previous_live[0]:
            draw_record(record)
            previous_live[0] = record
        return []

    selected = max(range(len(records)), key=lambda i: max(records[i]["feet"][leg][2] - config.nominal[leg][2] for leg in LEGS))
    draw(selected)
    if live_source:
        draw_live(0)
    if output:
        path = Path(output).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=150, facecolor=fig.get_facecolor())
    if animate:
        from matplotlib.animation import FuncAnimation
        if live_source:
            animation = FuncAnimation(fig, draw_live, interval=200, cache_frame_data=False)
        else:
            stride = max(1, round(config.control["rate_hz"] / 20))
            animation = FuncAnimation(fig, draw, frames=range(0, len(records), stride),
                                      interval=1000*stride/config.control["rate_hz"], repeat=True, cache_frame_data=False)
        fig._sm5_animation = animation  # retain until the GUI is closed
        plt.show()
    plt.close(fig)
