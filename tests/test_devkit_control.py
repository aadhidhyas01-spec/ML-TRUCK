"""
Interactive Wireless Robot & 6-DOF Robotic Arm Remote Control Tester
=====================================================================
Supports FULL drive combinations:

 MOVEMENT (type the keys then press Enter):
   [W]        All 4 wheels drive FORWARD
   [S]        All 4 wheels drive BACKWARD (REVERSE)
   [A]        Pivot steer LEFT in place
   [D]        Pivot steer RIGHT in place
   [WA]       Forward + Curve LEFT  (right wheels full, left wheels stop)
   [WD]       Forward + Curve RIGHT (left wheels full, right wheels stop)
   [SA]       Backward + Curve LEFT
   [SD]       Backward + Curve RIGHT
   [X]        STOP all motors
   [Space]    STOP all motors

 6-DOF ARM:
   [1]        Pick & Sort PLASTIC (Bin 1 - 165 deg Left)
   [2]        Pick & Sort METAL / CAN (Bin 2 - 135 deg Right)
   [3]        Pick & Sort GLASS / GENERAL (Bin 3 - 15 deg Center)
   [H]        Reset Arm to HOME position
   [J]        Manual Joint Test  (enter: <channel> <angle>)

 SENSORS:
   [T]        Query Live Telemetry (Ultrasonic distance)
   [I]        I2C Scan (Check PCA9685 Servo Driver at 0x40)
   [Q]        Quit
"""

import socket
import time
import sys

# Ensure UTF-8 output on Windows terminals (prevents UnicodeEncodeError)
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")

DEVKIT_IP   = "192.168.137.72"   # DevKit IP from hotspot
DEVKIT_PORT = 8888
ser_conn    = None
is_serial   = False

# Check for Bluetooth / Serial override or auto-detection
target_arg = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else None
candidate_coms = []

if target_arg and target_arg.upper().startswith("COM"):
    candidate_coms = [target_arg]
elif not target_arg or "--bt" in sys.argv:
    try:
        import serial.tools.list_ports as lp
        bt_ports = [p.device for p in lp.comports() if "BTHENUM" in p.hwid.upper() or "BLUETOOTH" in p.description.upper()]
        def port_num(d):
            try: return int(d.upper().replace("COM", ""))
            except: return 0
        bt_ports.sort(key=port_num, reverse=True)
        candidate_coms = bt_ports
    except Exception:
        pass
elif target_arg:
    DEVKIT_IP = target_arg

# Try connecting to candidate serial ports
if candidate_coms:
    import serial
    for p in candidate_coms:
        try:
            ser_conn = serial.Serial(p, 115200, timeout=1.5)
            is_serial = True
            print(f"  📶 [Bluetooth Serial Connected] DevKit active on {p} (Zero IP needed!)")
            break
        except Exception:
            ser_conn = None
            continue
    if not ser_conn:
        print(f"  ⚠️ Could not open candidate Bluetooth ports {candidate_coms}. Falling back to UDP {DEVKIT_IP}:{DEVKIT_PORT}")

sock   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
except Exception:
    pass

# Auto-discover DevKit IP if running over Wi-Fi UDP
if not is_serial and (not target_arg or target_arg in ("192.168.137.204", "192.168.137.43")):
    try:
        import subprocess
        arp = subprocess.check_output("arp -a", shell=True).decode("utf-8", errors="ignore")
        in_hs = False
        for line in arp.splitlines():
            if "Interface:" in line:
                in_hs = "192.168.137.1" in line
            elif in_hs:
                parts = line.split()
                if len(parts) >= 2 and parts[0].startswith("192.168.137.") and not parts[0].endswith(".255") and parts[0] != "192.168.137.1":
                    mac = parts[1].lower().replace("-", ":")
                    if "b0:cb:d8" in mac:
                        DEVKIT_IP = parts[0]
                        print(f"  📶 [Auto-Discovered from MAC] ESP32 DevKit detected at {DEVKIT_IP}:{DEVKIT_PORT}")
                        break
        sock.settimeout(0.35)
        sock.sendto(b"CMD:PING\n", ("192.168.137.255", DEVKIT_PORT))
        try:
            _, addr = sock.recvfrom(512)
            if addr[0].startswith("192.168.137.") and addr[0] != "192.168.137.1":
                DEVKIT_IP = addr[0]
                print(f"  📶 [Auto-Discovered from Broadcast] ESP32 DevKit detected at {DEVKIT_IP}:{DEVKIT_PORT}")
        except Exception:
            pass
    except Exception:
        pass
    finally:
        sock.settimeout(None)

target = (DEVKIT_IP, DEVKIT_PORT)

# ─────────────────────────────────────────────────────────────
# Communications helpers (Bluetooth Serial OR UDP)
# ─────────────────────────────────────────────────────────────
def send_cmd(cmd: str, silent: bool = False):
    if is_serial and ser_conn and ser_conn.is_open:
        try:
            ser_conn.write((cmd.strip() + "\n").encode("utf-8"))
            ser_conn.flush()
            if not silent:
                print(f"  📶 [Sent via BT {ser_conn.port}]  {cmd}")
        except Exception as e:
            print(f"  ❌ [BT Serial Error] {e}")
    else:
        try:
            sock.sendto(cmd.encode("utf-8"), target)
            if not silent:
                print(f"  📡 [Sent -> {DEVKIT_IP}:{DEVKIT_PORT}]  {cmd}")
        except Exception as e:
            print(f"  ❌ [UDP Error] {e}")

def get_telemetry():
    send_cmd("CMD:TELEMETRY", silent=True)
    if is_serial and ser_conn and ser_conn.is_open:
        try:
            time.sleep(0.1)
            msg = ser_conn.readline().decode("utf-8", errors="ignore").strip()
            if msg:
                print(f"\n  📊 [TELEMETRY via BT]\n     {msg}\n")
            else:
                print("  ⏳ No response – is DevKit powered on?")
        except Exception as e:
            print(f"  ❌ [Error] {e}")
    else:
        sock.settimeout(1.5)
        try:
            data, addr = sock.recvfrom(1024)
            if addr[0].startswith("192.168.137.") and addr[0] != "192.168.137.1":
                DEVKIT_IP = addr[0]
                target = (DEVKIT_IP, DEVKIT_PORT)
            msg = data.decode("utf-8", errors="ignore")
            print(f"\n  📊 [TELEMETRY from {addr[0]}]\n     {msg}\n")
        except socket.timeout:
            print("  ⏳ No response – is the DevKit powered on and on the hotspot?")
        except Exception as e:
            print(f"  ❌ [Error] {e}")
        finally:
            sock.settimeout(None)

def i2c_scan():
    send_cmd("CMD:I2C_SCAN", silent=True)
    if is_serial and ser_conn and ser_conn.is_open:
        try:
            time.sleep(0.15)
            msg = ser_conn.readline().decode("utf-8", errors="ignore").strip()
            print(f"\n  🔌 [I2C via BT]\n     {msg}\n")
        except Exception as e:
            print(f"  ❌ [Error] {e}")
    else:
        sock.settimeout(1.5)
        try:
            data, addr = sock.recvfrom(1024)
            msg = data.decode("utf-8", errors="ignore")
            print(f"\n  🔌 [I2C from {addr[0]}]\n     {msg}\n")
        except socket.timeout:
            print("  ⏳ No I2C response – is the DevKit connected?")
        except Exception as e:
            print(f"  ❌ [Error] {e}")
        finally:
            sock.settimeout(None)

# ─────────────────────────────────────────────────────────────
# Full drive command map (order-insensitive combos)
# ─────────────────────────────────────────────────────────────
DRIVE_CMDS = {
    # Single key
    "W":  ("CMD:FORWARD",        "All 4 wheels FORWARD"),
    "S":  ("CMD:BACKWARD",       "All 4 wheels REVERSE"),
    "A":  ("CMD:LEFT",           "Pivot steer LEFT in place (FR+RR FWD, FL+RL REV)"),
    "D":  ("CMD:RIGHT",          "Pivot steer RIGHT in place (FL+RL FWD, FR+RR REV)"),
    "Q":  ("CMD:FORWARD_LEFT",   "Forward Left [Q] (FR+RR rotate forward, FL+RL stop)"),
    "E":  ("CMD:FORWARD_RIGHT",  "Forward Right [E] (FL+RL rotate forward, FR+RR stop)"),
    "Z":  ("CMD:BACK_LEFT",      "Reverse Left [Z] (FR+RR rotate reverse, FL+RL stop)"),
    "C":  ("CMD:BACK_RIGHT",     "Reverse Right [C] (FL+RL rotate reverse, FR+RR stop)"),
    "X":  ("CMD:STOP",           "STOP all motors"),
    " ":  ("CMD:STOP",           "STOP all motors"),

    # Diagnostics & Configuration
    "TL": ("CMD:TEST:LEFT",      "Diagnostic: Run Left Motors ONLY (1.2s)"),
    "TR": ("CMD:TEST:RIGHT",     "Diagnostic: Run Right Motors ONLY (1.2s)"),
    "M1": ("CMD:MOTOR_MODE:1",   "Set Motor Mode 1 (Split Mode: 25=Left Fwd, 26=Right Fwd)"),
    "M2": ("CMD:MOTOR_MODE:2",   "Set Motor Mode 2 (Standard L298N: 25=IN1, 26=IN2, 32=IN3, 33=IN4)"),
    "SR": ("CMD:SWAP_REV",       "Swap Reverse Pins (D32 <-> D33)"),
    "25": ("CMD:TEST:PIN25",     "Test GPIO 25 directly (1.2s)"),
    "26": ("CMD:TEST:PIN26",     "Test GPIO 26 directly (1.2s)"),
    "32": ("CMD:TEST:PIN32",     "Test GPIO 32 directly (1.2s)"),
    "33": ("CMD:TEST:PIN33",     "Test GPIO 33 directly (1.2s)"),
    "BOFF": ("CMD:AUTOBRAKE:OFF", "Disable Ultrasonic Auto-Brake"),
    "BON":  ("CMD:AUTOBRAKE:ON",  "Enable Ultrasonic Auto-Brake"),
}

ARM_CMDS = {
    "1": ("ARM:PICK:DEGRADABLE",         "Pick Ground Object → Deposit in RIGHT Bin (Degradable / Banana Peel · 180° Right)"),
    "2": ("ARM:PICK:NON_DEGRADABLE",     "Pick Ground Object → Deposit in LEFT Bin (Non-Degradable · 180° Left)"),
    "BR": ("ARM:BIN:RIGHT",              "Rotate 180° Right (CW) → Deposit in Degradable Bin"),
    "BL": ("ARM:BIN:LEFT",               "Rotate 180° Left (CCW) → Deposit in Non-Degradable Bin"),
    "L": ("ARM:PICK_TARGET:NON_DEGRADABLE:LEFT:200",  "Test Pick LEFT side item → Deposit in LEFT Bin (180° Left)"),
    "R": ("ARM:PICK_TARGET:DEGRADABLE:RIGHT:200",    "Test Pick RIGHT side item → Deposit in RIGHT Bin (180° Right)"),
    "H": ("ARM:HOME",                    "Reset 6-DOF Arm to HOME pose"),
}

claw_is_open = True

# ─────────────────────────────────────────────────────────────
def print_header():
    print("\n" + "=" * 66)
    print("  🚛  Wireless Smart Waste Truck - Dual Dustbin Controller")
    print(f"  Target DevKit : {DEVKIT_IP}:{DEVKIT_PORT}")
    print("=" * 66)
    print("  ── DRIVE (Single & Combos) ──────────────────────────────")
    print("   W  = Forward             S  = Reverse")
    print("   A  = Pivot Left          D  = Pivot Right")
    print("   WA = Forward + Left      WD = Forward + Right")
    print("   SA = Reverse + Left      SD = Reverse + Right")
    print("   X  = STOP (Space also stops)")
    print()
    print("  ── DUAL DUSTBIN ROBOTIC ARM ──────────────────────────────")
    print("   1  = Pick & Deposit in RIGHT Bin (Degradable / Banana Peel · 180° Right)")
    print("   2  = Pick & Deposit in LEFT Bin  (Non-Degradable / Plastic · 180° Left)")
    print("   BR = Empty into RIGHT Dustbin (180° Right CW)")
    print("   BL = Empty into LEFT Dustbin  (180° Left CCW)")
    print("   L  = Pick from Left Ground -> Left Bin")
    print("   R  = Pick from Right Ground -> Right Bin")
    print("   H  = Reset Arm to HOME Driving Position")
    print("   C  = Toggle Gripper Claw (Open / Close)")
    print("   B  = Test 360° Base Rotation (SG996R)")
    print("   K  = Calibrate 180° Bin Turn Time (ms)")
    print("   J  = Manual Joint Test [CH0: speed -100..+100 | CH1-5: 0..180°]")
    print()
    print("  ── SYSTEM & SENSORS ──────────────────────────────────────")
    print("   P = Set Motor PWM Speed (0-255) [default 180, ~70%]")
    print("   T = Live Ultrasonic Distance Telemetry")
    print("   I = I2C Bus Scan (Check PCA9685 at 0x40)")
    print("   Q = Quit & Stop All Motors")
    print("=" * 66)

# ─────────────────────────────────────────────────────────────
def main():
    print_header()
    send_cmd("CMD:STOP")   # safety stop on connect

    while True:
        try:
            raw = input("\n  Enter key(s): ").strip().upper()
            if not raw:
                continue

            # ── Drive commands ───────────────────────────────
            if raw in DRIVE_CMDS:
                cmd, desc = DRIVE_CMDS[raw]
                print(f"  🔸 {desc}")
                send_cmd(cmd)

            # ── Arm preset sequences ─────────────────────────
            elif raw in ARM_CMDS:
                cmd, desc = ARM_CMDS[raw]
                print(f"  🦾 {desc}")
                send_cmd(cmd)

            # ── Claw Open / Close Toggle ─────────────────────
            elif raw == "C":
                global claw_is_open
                claw_is_open = not claw_is_open
                angle = 30 if claw_is_open else 105
                action = "OPEN" if claw_is_open else "CLOSED (Clamped)"
                print(f"  🦀 Claw {action} ({angle}°)")
                send_cmd(f"ARM:GRIPPER:{angle}")
                send_cmd(f"ARM:SERVO:4:{angle}", silent=True)
                send_cmd(f"ARM:SERVO:5:{angle}", silent=True)

            # ── Calibrate 180° Rear Turn Duration ────────────
            elif raw == "K":
                val = input("  Enter 180° rotation duration in milliseconds (e.g. 1100): ").strip()
                if val.isdigit() and int(val) > 100:
                    ms = int(val)
                    print(f"  ⏱️ Calibrating 180° turn duration → {ms} ms")
                    send_cmd(f"ARM:CALIB:180:{ms}")
                else:
                    print("  ❌ Enter a valid millisecond number (e.g. 1000, 1100, 1200)")

            # ── Manual joint test ────────────────────────────
            elif raw == "J":
                joints = input(
                    "  CH0=base speed(-100..+100), CH1-5=angle(0..180). Enter: channel value: "
                ).strip()
                parts = joints.split()
                if len(parts) == 2:
                    ch, val = int(parts[0]), int(parts[1])
                    cmd = f"ARM:SERVO:{ch}:{val}"
                    label = f"speed {val}%" if ch == 0 else f"angle {val}°"
                    print(f"  🦾 CH{ch} → {label}")
                    send_cmd(cmd)
                else:
                    print("  ❌ Format: <channel> <value>   e.g.  0 50  or  2 90")

            # ── Base M1 Degree Control ─────────────────────
            elif raw == "B":
                print("  🔄 Base M1 Positional Degree Control (0°..180°)")
                print("     Enter angle: 0 (Left), 90 (Center), 180 (Right)")
                inp = input("  Enter angle (0-180) or LEFT/RIGHT/CENTER: ").strip().upper()
                if inp in ("LEFT", "RIGHT", "CENTER"):
                    send_cmd(f"ARM:BASE:{inp}")
                elif inp.isdigit():
                    ang = max(0, min(180, int(inp)))
                    print(f"  🔄 Base M1 -> {ang}°")
                    send_cmd(f"ARM:SERVO:0:{ang}")
                else:
                    print("  ❌ Enter a valid angle (0-180)")

            # ── Sensors ──────────────────────────────────────
            elif raw == "T":
                get_telemetry()
            elif raw == "I":
                i2c_scan()

            # ── Speed control ────────────────────────────────
            elif raw == "P":
                val = input("  Enter speed (0-255): ").strip()
                if val.isdigit() and 0 <= int(val) <= 255:
                    spd = int(val)
                    pct = round(spd * 100 / 255)
                    print(f"  ⚙️  Setting motor speed → {spd}/255 ({pct}%)")
                    send_cmd(f"CMD:SPEED:{spd}")
                else:
                    print("  ❌ Enter a number between 0 and 255")

            # ── Quit ─────────────────────────────────────────
            elif raw == "Q":
                print("  Sending STOP and exiting...")
                send_cmd("CMD:STOP")
                break

            else:
                print(f"  ❓ Unknown: '{raw}'  –  valid drive keys: W S A D X WA WD SA SD")

        except (KeyboardInterrupt, EOFError):
            send_cmd("CMD:STOP")
            print("\n  Interrupted – motors stopped.")
            break

    sock.close()


if __name__ == "__main__":
    main()
