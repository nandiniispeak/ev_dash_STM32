"""
EV ADAS Python Dashboard — Phase 9
Reads UART telemetry from STM32 Blue Pill / PICSimLab
Run: python dashboard.py --port COM4
     python dashboard.py --demo        (no hardware needed)
"""

import argparse
import serial
import threading
import time
import re
import collections
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import numpy as np

# ── Shared state ─────────────────────────────────────────────────────
state = {
    'speed': 0.0, 'soc': 80.0, 'torque': 0.0,
    'temp': 25.0,  'range': 2666.0,
    'accel': 0.0,  'brake': 0.0,
    'front': 400.0,'left':  400.0, 'right': 400.0,
    'ttc':   99.9, 'col':   0,
    'bsd_l': 0,    'bsd_r': 0,
    'alarm': 0,    'fault': 0,
    'drive_mode': 1,        # ← ADD THIS
    'connected': False,
}

spd_history = collections.deque([0.0]*60, maxlen=60)

# ── Parse one UART line ───────────────────────────────────────────────
def parse_line(line):
    line = line.strip()

    # Line 1: SPD:72.5 SOC:79.3 TRQ:75 TMP:27.1 RNG:2600 ACC:50 BRK:0
    m = re.search(
        r'SPD:(\d+)\.(\d+)\s+SOC:(\d+)\.(\d+)\s+TRQ:(-?\d+)'
        r'\s+TMP:(\d+)\.(\d+)\s+RNG:(\d+)\s+ACC:(\d+)\s+BRK:(\d+)', line)
    if m:
        g = m.groups()
        state['speed']  = float(g[0]) + float(g[1])/10
        state['soc']    = float(g[2]) + float(g[3])/10
        state['torque'] = float(g[4])
        state['temp']   = float(g[5]) + float(g[6])/10
        state['range']  = float(g[7])
        state['accel']  = float(g[8])
        state['brake']  = float(g[9])
        spd_history.append(state['speed'])
        return

    # Line 2: F:40 L:400 R:400 TTC:2.1s COL:1 BSD:00 ALM:2 FLT:04
    m = re.search(
        r'F:(\d+)\s+L:(\d+)\s+R:(\d+)\s+TTC:(\d+)\.(\d+)s?\s+'
        r'COL:(\d+)\s+BSD:(\d)(\d)\s+ALM:(\d+)\s+FLT:([0-9A-Fa-f]+)', line)
    if m:
        g = m.groups()
        state['front'] = float(g[0])
        state['left']  = float(g[1])
        state['right'] = float(g[2])
        state['ttc']   = float(g[3]) + float(g[4])/10
        state['col']   = int(g[5])
        state['bsd_l'] = int(g[6])
        state['bsd_r'] = int(g[7])
        state['alarm'] = int(g[8])
        state['fault'] = int(g[9], 16)

# ── Serial reader thread ──────────────────────────────────────────────
def serial_reader(port, baud):
    while True:
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                state['connected'] = True
                print(f"[UART] Connected: {port} @ {baud}")
                while True:
                    line = ser.readline().decode('ascii', errors='replace')
                    if line:
                        parse_line(line)
        except Exception as e:
            state['connected'] = False
            print(f"[UART] {e} — retry in 2s")
            time.sleep(2)

# ── Demo mode — animate without hardware ─────────────────────────────
_t = 0.0
def demo_tick():
    global _t
    _t += 0.1
    state['speed']  = 60 + 40 * np.sin(_t * 0.3)
    state['soc']    = max(5, 80 - _t * 0.3)
    state['torque'] = 80 * abs(np.sin(_t * 0.5))
    state['temp']   = 25 + 40 * (1 - np.exp(-_t * 0.05))
    state['range']  = state['soc'] / 100 * 60000 / 18
    state['accel']  = 50 + 30 * np.sin(_t * 0.4)
    state['front']  = 100 + 80 * np.sin(_t * 0.2)
    state['left']   = 200 + 180 * np.sin(_t * 0.15)
    state['right']  = 200 + 180 * np.cos(_t * 0.15)
    state['ttc']    = state['front']/100 / max(state['speed']/3.6, 0.1)
    state['col']    = 2 if state['front']<20 else 1 if state['front']<50 else 0
    state['bsd_l']  = 1 if state['left']<30 and state['speed']>20 else 0
    state['bsd_r']  = 1 if state['right']<30 and state['speed']>20 else 0
    state['alarm']  = state['col']
    state['fault']  = 1 if state['temp'] > 90 else 0
    state['connected'] = True
    spd_history.append(state['speed'])

# ── Dashboard layout ──────────────────────────────────────────────────
fig = plt.figure(figsize=(15, 8), facecolor='#0d1117')
fig.canvas.manager.set_window_title('EV ADAS Dashboard — Blue Pill')

gs  = fig.add_gridspec(2, 3, hspace=0.38, wspace=0.32,
                       left=0.05, right=0.97, top=0.93, bottom=0.06)
ax_speed  = fig.add_subplot(gs[0, 0])
ax_soc    = fig.add_subplot(gs[0, 1])
ax_adas   = fig.add_subplot(gs[:, 2])
ax_trend  = fig.add_subplot(gs[1, 0])
ax_info   = fig.add_subplot(gs[1, 1])

for ax in fig.get_axes():
    ax.set_facecolor('#0d1b2a')

ALARM_COLORS = ['#2ecc71', '#f39c12', '#e67e22', '#e74c3c']

# ── Speedometer ───────────────────────────────────────────────────────
def draw_speed(ax, speed):
    ax.cla(); ax.set_facecolor('#0d1b2a')
    ax.set_xlim(-1.6,1.6); ax.set_ylim(-1.4,1.3)
    ax.axis('off')
    ax.set_title('SPEED', color='#90caf9', fontsize=10, pad=3)

    theta_bg = np.linspace(np.pi*1.2, -np.pi*0.2, 200)
    ax.plot(np.cos(theta_bg), np.sin(theta_bg),
            color='#1e3a5c', linewidth=10, solid_capstyle='round')

    frac  = min(speed/200.0, 1.0)
    color = '#2ecc71' if speed<100 else '#f39c12' if speed<140 else '#e74c3c'
    if frac > 0.01:
        theta_sp = np.linspace(np.pi*1.2, np.pi*1.2-frac*np.pi*1.4, 200)
        ax.plot(np.cos(theta_sp), np.sin(theta_sp),
                color=color, linewidth=10, solid_capstyle='round')

    for kmh in range(0, 201, 40):
        f   = kmh/200.0
        ang = np.pi*1.2 - f*np.pi*1.4
        ax.plot([0.78*np.cos(ang),0.92*np.cos(ang)],
                [0.78*np.sin(ang),0.92*np.sin(ang)],
                color='#546e7a', linewidth=1.5)
        ax.text(0.65*np.cos(ang), 0.65*np.sin(ang), str(kmh),
                ha='center', va='center', color='#78909c', fontsize=7)

    ang = np.pi*1.2 - frac*np.pi*1.4
    ax.annotate('', xy=(0.80*np.cos(ang), 0.80*np.sin(ang)),
                xytext=(0,0),
                arrowprops=dict(arrowstyle='->', color='white', lw=2.5))

    ax.text(0,-0.42, f'{speed:.0f}', ha='center', color='white',
            fontsize=30, fontweight='bold')
    ax.text(0,-0.75, 'km/h', ha='center', color='#78909c', fontsize=11)

# ── SOC bar ───────────────────────────────────────────────────────────
def draw_soc(ax, soc, rng, mode_idx):
    ax.cla(); ax.set_facecolor('#0d1b2a')
    ax.set_xlim(0,100); ax.set_ylim(0,5); ax.axis('off')
    ax.set_title('BATTERY', color='#90caf9', fontsize=10, pad=3)

    col = '#2ecc71' if soc>30 else '#f39c12' if soc>10 else '#e74c3c'
    ax.barh([2.5],[100], color='#1e3a5c', height=0.8)
    ax.barh([2.5],[soc],  color=col,       height=0.8)

    ax.text(50, 4.0, f'{soc:.1f}%',   ha='center', color='white',
            fontsize=22, fontweight='bold')
    ax.text(50, 1.3, f'~{rng:.0f} km', ha='center', color='#78909c', fontsize=10)

    modes   = ['ECO','NORMAL','SPORT']
    mcols   = ['#2ecc71','#00b4d8','#f39c12']
    ax.text(50, 0.3, f'Mode: {modes[mode_idx]}',
            ha='center', color=mcols[mode_idx], fontsize=10, fontweight='bold')

# ── Speed trend ───────────────────────────────────────────────────────
def draw_trend(ax, history):
    ax.cla(); ax.set_facecolor('#0d1b2a')
    ax.plot(list(history), color='#00b4d8', linewidth=1.5)
    ax.set_ylim(0, 220)
    ax.set_title('Speed History', color='#90caf9', fontsize=10, pad=3)
    ax.tick_params(colors='#546e7a', labelsize=8)
    ax.spines['bottom'].set_color('#1e3a5c')
    ax.spines['left'].set_color('#1e3a5c')
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.set_ylabel('km/h', color='#546e7a', fontsize=8)

# ── Info panel ────────────────────────────────────────────────────────
# ── Info panel ────────────────────────────────────────────────────────
def draw_info(ax):
    ax.cla()
    ax.set_facecolor('#0d1b2a')
    ax.axis('off')

    ax.set_title('EV METRICS',
                 color='#90caf9',
                 fontsize=10,
                 pad=3)

    fault_str = f'0x{state["fault"]:02X}'
    fault_col = '#e74c3c' if state['fault'] else '#2ecc71'

    alarm_str = ['NONE', 'ADVISORY', 'WARNING', 'CRITICAL'][
        min(state['alarm'], 3)
    ]

    alarm_col = ALARM_COLORS[min(state['alarm'], 3)]

    rows = [
        ('TORQUE',   f'{state["torque"]:.0f} Nm', '#00b4d8'),

        ('ACCEL',    f'{state["accel"]:.0f} %', '#2ecc71'),

        ('BRAKE',    f'{state["brake"]:.0f} %', '#f39c12'),

        ('MOT TEMP', f'{state["temp"]:.1f} °C',
            '#e74c3c' if state['temp'] > 80 else '#2ecc71'),

        ('ALARM',    alarm_str, alarm_col),

        ('FAULT',    fault_str, fault_col),

        ('SIGNAL',
            'ON' if state['connected'] else 'OFF',
            '#2ecc71' if state['connected'] else '#e74c3c'),
    ]

    for i, (lbl, val, col) in enumerate(rows):

        y = 0.88 - i * 0.135

        # Left label
        ax.text(
            0.05, y,
            lbl,
            transform=ax.transAxes,
            color='#546e7a',
            fontsize=9,
            fontweight='bold'
        )

        # Right value
        ax.text(
            0.95, y,
            val,
            transform=ax.transAxes,
            color=col,
            fontsize=10,
            fontweight='bold',
            ha='right'
        )

        # Divider line (FIXED)
        ax.plot(
            [0.02, 0.98],
            [y - 0.06, y - 0.06],
            color='#1e3a5c',
            linewidth=0.5,
            transform=ax.transAxes
        )

# ── ADAS bird-eye ─────────────────────────────────────────────────────
def draw_adas(ax):
    ax.cla(); ax.set_facecolor('#0d1b2a')
    ax.set_xlim(-6,6); ax.set_ylim(-3,16)
    ax.axis('off')
    ax.set_title('ADAS — Bird Eye View', color='#90caf9', fontsize=10, pad=3)

    for x in [-5,5]:
        ax.plot([x,x],[-3,16], color='#546e7a', linewidth=2)
    for x in [-1.8,1.8]:
        for y in range(-2,15,3):
            ax.plot([x,x],[y,y+1.5],'--',color='#37474f',linewidth=1)

    # Ego vehicle
    ego = patches.FancyBboxPatch((-1.2,0),2.4,3.5,
                                  boxstyle='round,pad=0.15',
                                  facecolor='#1565c0',
                                  edgecolor='#42a5f5', linewidth=2)
    ax.add_patch(ego)
    ax.text(0,1.75,'EV', ha='center', va='center',
            color='white', fontsize=11, fontweight='bold')

    # Front obstacle
    dist_m   = state['front']/100.0
    obs_y    = 3.5 + dist_m*2.5
    col_cols = ['#37474f','#e67e22','#e74c3c']
    obs_col  = col_cols[min(state['col'],2)]

    if dist_m < 5.0:
        obs = patches.FancyBboxPatch((-1.2,obs_y),2.4,3.5,
                                      boxstyle='round,pad=0.1',
                                      facecolor=obs_col,
                                      edgecolor='white', linewidth=1.5,
                                      alpha=0.9)
        ax.add_patch(obs)
        ax.text(0, obs_y+1.75, f'{state["front"]:.0f}cm',
                ha='center', va='center', color='white', fontsize=9)

        arc_col = ['#2ecc71','#f39c12','#e74c3c'][min(state['col'],2)]
        from matplotlib.patches import Arc
        arc = Arc((0,3.5), dist_m*2.5, dist_m*2.5,
                  theta1=10, theta2=170,
                  color=arc_col, linewidth=2.5)
        ax.add_patch(arc)

    ttc_col = '#e74c3c' if state['ttc']<3 else '#f39c12' if state['ttc']<6 else '#546e7a'
    ax.text(0,-1.5, f'TTC: {state["ttc"]:.1f}s',
            ha='center', color=ttc_col, fontsize=11, fontweight='bold')
    ax.text(0,-2.4, f'Speed: {state["speed"]:.0f} km/h',
            ha='center', color='#546e7a', fontsize=9)

    # Left BSD
    if state['left'] < 150:
        lcol = '#f39c12' if state['bsd_l'] else '#37474f'
        lb   = patches.FancyBboxPatch((-5.2,0.5),2.4,3.0,
                                       boxstyle='round,pad=0.1',
                                       facecolor=lcol, alpha=0.85)
        ax.add_patch(lb)
        ax.text(-4.0,2.0, f'{state["left"]:.0f}cm',
                ha='center', color='white', fontsize=8)
        if state['bsd_l']:
            ax.text(-4.0,-0.3,'⚠ BSD L',
                    ha='center', color='#f39c12',
                    fontsize=9, fontweight='bold')

    # Right BSD
    if state['right'] < 150:
        rcol = '#f39c12' if state['bsd_r'] else '#37474f'
        rb   = patches.FancyBboxPatch((2.8,0.5),2.4,3.0,
                                       boxstyle='round,pad=0.1',
                                       facecolor=rcol, alpha=0.85)
        ax.add_patch(rb)
        ax.text(4.0,2.0, f'{state["right"]:.0f}cm',
                ha='center', color='white', fontsize=8)
        if state['bsd_r']:
            ax.text(4.0,-0.3,'⚠ BSD R',
                    ha='center', color='#f39c12',
                    fontsize=9, fontweight='bold')

# ── Animation update ──────────────────────────────────────────────────
IS_DEMO = False

def animate(frame):
    if IS_DEMO:
        demo_tick()

    alarm = min(state['alarm'],3)
    col   = ALARM_COLORS[alarm]

    # Flash background on CRITICAL
    if alarm == 3:
        fig.patch.set_facecolor('#1a0000' if int(time.time()*2)%2 else '#0d1117')
    else:
        fig.patch.set_facecolor('#0d1117')

    draw_speed(ax_speed, state['speed'])
    draw_soc(ax_soc, state['soc'], state['range'], state['drive_mode'])
    draw_trend(ax_trend, spd_history)
    draw_info(ax_info)
    draw_adas(ax_adas)

    fig.suptitle(
        f'⚡ EV ADAS Dashboard  |  Alarm: {["NONE","ADVISORY","WARNING","CRITICAL"][alarm]}',
        color=col, fontsize=13, fontweight='bold')

    return []

# ── Entry point ───────────────────────────────────────────────────────
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='EV ADAS Dashboard')
    parser.add_argument('--port',  default=None, help='Serial port (COM4 / /dev/ttyUSB0)')
    parser.add_argument('--baud',  type=int, default=115200)
    parser.add_argument('--demo',  action='store_true', help='Demo mode — no hardware needed')
    args = parser.parse_args()

    if args.demo or args.port is None:
        IS_DEMO = True
        print("[INFO] Demo mode — no serial port required")
        print("[INFO] Run with --port COM4 to connect to hardware")
    else:
        IS_DEMO = False
        t = threading.Thread(target=serial_reader,
                             args=(args.port, args.baud), daemon=True)
        t.start()

    ani = animation.FuncAnimation(
        fig, animate, interval=100, blit=False, cache_frame_data=False)
    plt.show()