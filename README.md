# RobStride Motor Control (Raspberry Pi / Linux)

This directory contains the minimal Python code to run and test RobStride motors over CAN. The `Robstride_control` (or this `python/`) directory can be shared and used on a Raspberry Pi or any Ubuntu/Debian system.

---

## 1. System dependencies (Ubuntu/Debian)

Install required system packages:

```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential cmake \
    python3 python3-pip \
    rustc cargo \
    can-utils \
    gcc-avr avr-libc arduino-core
```

---

## 2. CAN interface setup

Load the CAN kernel module and bring the interface up (e.g. `can0` at 1 Mbps):

```bash
sudo modprobe can
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```

To bring the interface up again after a reboot or after it was brought down:

```bash
sudo ip link set can0 up type can bitrate 1000000
```

---

## 3. Python virtual environment and dependencies

Use a virtual environment (venv) to isolate project dependencies.

### 3.1 Create and activate a venv

From this directory (`python/` or project root):

```bash
cd python   # if you're in the parent directory
python3 -m venv .venv
source .venv/bin/activate   # Linux / macOS
```

On Windows (if applicable):

```bash
.venv\Scripts\activate
```

Your prompt will show `(.venv)` when the environment is active.

### 3.2 Install Python dependencies

With the venv activated:

```bash
pip install -r requirements.txt
```

Or install the required packages directly:

```bash
pip install python-can>=4.0.0 numpy>=1.21.0 tqdm>=4.62.0
```

**Note:** Each time you open a new terminal to run the scripts, activate the venv again with `source .venv/bin/activate` (Linux/macOS) or `.venv\Scripts\activate` (Windows).

---

## 4. Motor ID assignment (Motor Studio)

**Motor IDs must be set in software using Motor Studio** before using this code. Assign the desired IDs to each motor as per your requirement, then use those IDs in the commands below.

---

## 5. How to run (after motor IDs are set)

All commands that use the CAN bus typically need `sudo`.

### 5.1 Bring CAN interface up

```bash
sudo ip link set can0 up type can bitrate 1000000
```

### 5.2 Check connected motors (ping)

From the `python/` directory:

```bash
python3 src/ping.py
```

This scans the default channel `can0` and lists motors connected to the bus.

To scan a different channel (e.g. `can1`):

```bash
python3 src/ping.py can1
```

### 5.3 Run and test motors (move_actuator)

`move_actuator.py` is used to run and test motors. Pass the motor IDs you want to control, and optionally the CAN channel.

**Example: test motors with IDs 2 and 4 on `can0`**

```bash
python3 src/move_actuator.py 2 4 -c can0
```

Or omit the channel (default is `can0`):

```bash
python3 src/move_actuator.py 2 4
```

**Usage summary:**

```text
python3 src/move_actuator.py <motor_id1> [motor_id2] ... [ -c can0 ]
```

- **Motor IDs**: one or more space-separated IDs (e.g. `2 4`).
- `**-c` / channel**: CAN interface (default: `can0`). Use e.g. `-c can1` for a second bus.

Additional options (see script help):

```bash
python3 src/move_actuator.py --help
```

Example with frequency and amplitude:

```bash
python3 src/move_actuator.py 2 4 -c can0 --frequency 1.0 --amplitude 1.0
```

---

