"""
Sysiphus Rocket - Flight Data Analyzer
---------------------------------------
Usage: set LOG_FILE to your CSV path and run.
Requires: pip install pandas matplotlib
"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import sys

# ── CONFIG ──────────────────────────────────────────────────────────────────
LOG_FILE   = "LOG000.CSV"   # <-- change to your log file
BURN_TIME  = 3.45           # seconds, from firmware constant
IGN_DELAY  = 0.2            # seconds, from firmware constant
# ────────────────────────────────────────────────────────────────────────────

# --- Load -------------------------------------------------------------------
try:
    df = pd.read_csv(LOG_FILE)
except FileNotFoundError:
    print(f"File not found: {LOG_FILE}")
    sys.exit(1)

df.columns = df.columns.str.strip()
df["Time_s"] = df["Time(ms)"] / 1000.0
t = df["Time_s"]

# --- Detect key events ------------------------------------------------------
launch_t   = t.iloc[0]

# Burnout: estimated from ignition time (first sample) + burn time
burnout_t  = launch_t + BURN_TIME + IGN_DELAY

# Apogee: peak altitude
apogee_idx = df["Altitude(m)"].idxmax()
apogee_t   = t[apogee_idx]
apogee_alt = df["Altitude(m)"][apogee_idx]

# Chute: first point after apogee where altitude < 75% of apogee
chute_mask = (t > apogee_t) & (df["Altitude(m)"] < apogee_alt * 0.65)
chute_t    = t[chute_mask].iloc[0] if chute_mask.any() else None

# Emergency: any gyro > 90 deg
emerg_mask = (df["GyroX"].abs() > 90) | (df["GyroY"].abs() > 90)
emerg_t    = t[emerg_mask].iloc[0] if emerg_mask.any() else None

# Max tilt during powered flight
powered = (t >= launch_t) & (t <= burnout_t)
max_tilt_powered = df.loc[powered, ["GyroX", "GyroY"]].abs().max().max()

# --- Print summary ----------------------------------------------------------
print("=" * 45)
print("  FLIGHT SUMMARY")
print("=" * 45)
print(f"  Apogee altitude   : {apogee_alt:.1f} m")
print(f"  Time to apogee    : {apogee_t - launch_t:.2f} s")
print(f"  Max descent vel   : {df['VertVel(m/s)'].min():.2f} m/s")
print(f"  Max ascent vel    : {df['VertVel(m/s)'].max():.2f} m/s")
print(f"  Max tilt (powered): {max_tilt_powered:.1f} deg")
print(f"  Emergency trigger : {'YES at t=' + f'{emerg_t:.2f}s' if emerg_t is not None else 'No'}")
print(f"  Chute deploy alt  : {apogee_alt * 0.75:.1f} m  (75% of apogee)")
print("=" * 45)

# --- Helper: draw event lines on an axis ------------------------------------
def draw_events(ax, show_labels=False):
    events = [
        (burnout_t, "Burnout",  "orange"),
        (apogee_t,  "Apogee",   "green"),
    ]
    if chute_t is not None:
        events.append((chute_t, "Chute", "blue"))
    if emerg_t is not None:
        events.append((emerg_t, "EMERGENCY", "red"))

    for ev_t, label, color in events:
        ax.axvline(ev_t, color=color, linestyle="--", linewidth=1, alpha=0.7)
        if show_labels:
            ax.text(ev_t + 0.05, ax.get_ylim()[1] * 0.95, label,
                    color=color, fontsize=7, va="top")

# --- Plot -------------------------------------------------------------------
fig = plt.figure(figsize=(14, 10))
fig.suptitle(f"Flight Analysis — {LOG_FILE}", fontsize=13, fontweight="bold")
gs  = gridspec.GridSpec(3, 2, figure=fig, hspace=0.45, wspace=0.35)

# 1. Altitude
ax1 = fig.add_subplot(gs[0, 0])
ax1.plot(t, df["Altitude(m)"], color="steelblue")
ax1.set_title("Altitude")
ax1.set_ylabel("m")
ax1.set_xlabel("Time (s)")
draw_events(ax1, show_labels=True)
ax1.axhline(apogee_alt * 0.65, color="blue", linestyle=":", linewidth=1, alpha=0.5, label="Chute trigger alt")
ax1.legend(fontsize=7)

# 2. Vertical velocity
ax2 = fig.add_subplot(gs[0, 1])
ax2.plot(t, df["VertVel(m/s)"], color="darkcyan")
ax2.axhline(0, color="gray", linewidth=0.8, linestyle="--")
ax2.set_title("Vertical Velocity")
ax2.set_ylabel("m/s")
ax2.set_xlabel("Time (s)")
draw_events(ax2)

# 3. Tilt angles (gyro)
ax3 = fig.add_subplot(gs[1, 0])
ax3.plot(t, df["GyroX"], label="GyroX (roll)", color="crimson")
ax3.plot(t, df["GyroY"], label="GyroY (pitch)", color="tomato", linestyle="--")
ax3.axhline(90,  color="black", linewidth=0.8, linestyle=":", alpha=0.5)
ax3.axhline(-90, color="black", linewidth=0.8, linestyle=":", alpha=0.5)
ax3.set_title("Tilt Angles")
ax3.set_ylabel("deg")
ax3.set_xlabel("Time (s)")
ax3.legend(fontsize=8)
draw_events(ax3)

# 4. Servo commands
ax4 = fig.add_subplot(gs[1, 1])
ax4.plot(t, df["ServoX"], label="ServoX", color="darkorchid")
ax4.plot(t, df["ServoY"], label="ServoY", color="mediumpurple", linestyle="--")
ax4.axhline(0,  color="gray", linewidth=0.8, linestyle="--")
ax4.set_title("TVC Servo Commands")
ax4.set_ylabel("deg (from neutral)")
ax4.set_xlabel("Time (s)")
ax4.legend(fontsize=8)
draw_events(ax4)

# 5. Angular velocity
ax5 = fig.add_subplot(gs[2, 0])
ax5.plot(t, df["AngVelX"], label="AngVelX", color="darkorange")
ax5.plot(t, df["AngVelY"], label="AngVelY", color="gold", linestyle="--")
ax5.axhline(0, color="gray", linewidth=0.8, linestyle="--")
ax5.set_title("Angular Velocity")
ax5.set_ylabel("deg/s")
ax5.set_xlabel("Time (s)")
ax5.legend(fontsize=8)
draw_events(ax5)

# 6. Vertical acceleration
ax6 = fig.add_subplot(gs[2, 1])
ax6.plot(t, df["AccelZ"], color="seagreen")
ax6.axhline(0, color="gray", linewidth=0.8, linestyle="--")
ax6.set_title("Vertical Acceleration (AccelZ)")
ax6.set_ylabel("m/s²")
ax6.set_xlabel("Time (s)")
draw_events(ax6)

plt.savefig(LOG_FILE.replace(".CSV", "_analysis.png"), dpi=150, bbox_inches="tight")
print(f"\nPlot saved: {LOG_FILE.replace('.CSV', '_analysis.png')}")
plt.show()
