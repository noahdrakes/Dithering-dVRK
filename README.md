# dVRK Dithering and Force Estimation Tools

This repository contains a collection of Python scripts developed to analyze and implement **dithering-based friction compensation** and **external force estimation** on the **da Vinci Research Kit (dVRK)** platform.  
The tools cover both the **control-side implementation** of joint-level dithering (in position and torque) and the **offline analysis** of recorded ROS bag data.

---

## Repository Structure

| File | Description |
|------|-------------|
|**`arm_constant_acc.py`**|Drives a selected joint of the dVRK arm back and forth with a **piecewise constant acceleration** profile, using the CRTK interface.|
| **`dithering_servo_js_with_accelerometer.py`** | Applies a **torque-based dithering** signal to a selected joint of the dVRK arm using the CRTK interface (`servo_js`). Optionally adjusts the amplitude online using accelerometer feedback. |
| **`dithering_servo_jp_with_accelerometer.py`** | Applies a **position-based dithering** signal to a selected joint of the dVRK arm using the CRTK interface (`servo_jp`). Optionally adjusts the amplitude online using accelerometer feedback.|
| **`read_friction_id.py`** | Reads ROS bag data to plot joint **effort vs velocity**, highlighting friction characteristics (Coulomb and viscous). |
| **`read_palpation_exp.py`** | Estimates external Cartesian forces from **joint torques** and **Jacobian matrices**, compares them against a **measured force sensor**, and computes the **Mean Absolute Error (MAE)**. |
| **`read_weights_exp.py`** | Similar to `read_palpation_exp.py`, but compares contact data against a **theoretical force**, derived from the known weight mass |

---


## Requirements

All scripts are written in **Python 3.8+** and rely on the following main packages:

- `numpy`
- `scipy`
- `matplotlib`
- `sklearn`
- `rosbags` (for reading `.bag` files without ROS)
- `crtk` (for CRTK-based control interfaces)

To install the dependencies:
```bash
pip install numpy scipy matplotlib scikit-learn rosbags 
```

If you are using the control-side scripts (`*_servo_*` or `arm_constant_acc.py`), you must have:

* a running ROS1/ROS2 environment configured with the dVRK CRTK interface

* the relevant robot arm enabled

---

## Accelerometer Integration

Some scripts (e.g., `dithering_servo_js_with_accelerometer.py` and `dithering_servo_jp_with_accelerometer.py`) rely on real-time accelerometer data to adapt the dithering amplitude based on vibration intensity.

To acquire and publish accelerometer data within the ROS framework, you can use the following repository:

[MPU-9250_serial2ros](https://github.com/saramartuscelli/MPU-9250_serial2ros.git)

That repository provides a Python interface for reading data from an **MPU-9250** sensor via serial connection and publishing it as a ROS topic (`/acceleration/data`).  
The published data can be directly subscribed by the dithering scripts for online adaptation.

---

## Usage

The scripts in this repository can be divided into **two categories**:

1. **Control scripts** — used to command the dVRK arm and generate motion or dithering:
   - `arm_constant_acc.py`
   - `dithering_servo_js_with_accelerometer.py`
   - `dithering_servo_jp_with_accelerometer.py`

2. **Analysis scripts** — used to process recorded data (ROS bag files):
   - `read_friction_id.py`
   - `read_palpation_exp.py`
   - `read_weights_exp.py`

---

### 1. Running Control Scripts

Each control script can be executed directly from the terminal.  
For example:

```bash
python dithering_servo_js_with_accelerometer.py -a PSM1 -j 0 -f 10 -A 0.5
```

where:

`-a` / `--arm_name` specifies the arm (e.g. `PSM1`),

`-j` / `--joint_index` selects the joint to command,

`-f` / `--frequency` sets the dithering frequency (Hz),

`-A` / `--amplitude` sets the dithering amplitude.

While running these scripts, **record a ROS bag** to capture all relevant topics (e.g., joint states, force sensor data) for later analysis.

Example command:

```bash
rosbag record -O my_experiment.bag /PSM1/measured_js /PSM1/gravity_compensation/setpoint_js /PSM1/spatial/jacobian
```

The specific topics may vary depending on your setup.


### 2. Analyzing Recorded Data

Once the experiment is complete and the .bag file is available, use one of the analysis scripts.

These scripts read and process the recorded data to:

* Extract joint torque and velocity information

* Estimate external forces

* Compare estimated vs. measured or theoretical forces

* Compute performance metrics such as the Mean Absolute Error (MAE)

* Plot results for visualization and comparison

---


## Citation

If you use this repository in your research, please cite the following work:

> **An Effectiveness Study of Dithering for Improved Force Estimation on the dVRK-Si System**  
> *Sara Martuscelli, Hao Yang, Elena De Momi, Jie Ying Wu, Peter Kazanzides*
>  
> [Under review (ISMR 2026)]

---

## Acknowledgments

This work was developed within a collaboration between the **Politecnico di Milano**, the **Johns Hopkins University**, and the **Vanderbilt University**.  
The authors acknowledge support from mobility funds from Politecnico di Milano and internal funds from Johns Hopkins University.

