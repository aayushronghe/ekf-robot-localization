# EKF-Based 2D Localization: GPS + Motion Model Fusion

## Overview

This project implements a nonlinear Extended Kalman Filter (EKF) for 2D robot localization using simulated GPS measurements and a nonlinear motion model with constant velocity and angular rate.

The objective is to demonstrate:

- State estimation under sensor noise  
- Probabilistic fusion of motion and measurement models  
- Sensitivity analysis under varying noise conditions  
- Systematic experimental evaluation of filter performance  

This project was developed to strengthen understanding of probabilistic robotics, state estimation, and SLAM foundations relevant to modern mobile robot autonomy.

---

## Problem Statement

Mobile robots operating in dynamic environments must localize reliably despite sensor noise and motion uncertainty.

Given:
- Noisy GPS position measurements  
- A nonlinear motion model (velocity + heading)  

Estimate the robot state:

```
x = [x, y, v, θ]
```

where:
- `x, y` = position  
- `v` = velocity  
- `θ` = heading angle  

using an Extended Kalman Filter.

---

## Motion Model

The robot follows a nonlinear kinematic model:

```
x(t+1) = x(t) + v * cos(θ) * dt
y(t+1) = y(t) + v * sin(θ) * dt
θ(t+1) = θ(t) + ω * dt
```

where:
- `v` = linear velocity  
- `ω` = angular velocity  
- `dt` = timestep  

The Jacobian of this nonlinear model is used during EKF prediction.

---

## Measurement Model

GPS provides noisy position measurements:

```
z = [x, y] + N(0, R)
```

where:
- `R` = measurement noise covariance  

The EKF update step fuses:
- Predicted state from motion model  
- Observed GPS measurement  

---

## EKF Implementation

The filter performs:

### 1. Prediction Step
- Nonlinear state propagation  
- Jacobian-based covariance update  
- Process noise injection  

### 2. Update Step
- Innovation (residual) computation  
- Kalman gain calculation  
- State correction  
- Covariance update  

---

## Experiments Performed

The system runs multiple experiments automatically with varying:

- GPS noise levels  
- Process noise covariance  
- Initial covariance assumptions  

Each experiment generates:

- Trajectory comparison plot  
- Position error plot  
- Quantitative summary metrics  

Results are stored in structured experiment folders.

---

## Evaluation Metrics

For each experiment, the following metrics are computed:

- Mean position error  
- Maximum position error  
- Final steady-state error  
- Convergence trend over time  

All results are aggregated into:

```
summary_results.csv
```

for comparative analysis.

---

## Directory Structure

```
EKF_Localization/
│
├── ekf_simulation.py
├── results/
│   ├── experiment_1/
│   ├── experiment_2/
│   ├── ...
    └── summary_results.csv
└── README.md
```

---

## How to Run

### 1. Install Dependencies

```bash
pip install numpy matplotlib pandas
```

### 2. Run All Experiments

```bash
python ekf_simulation.py
```

Results will be generated under:

```
results/
```

and a consolidated comparison file:

```
summary_results.csv
```

---

## Example Output

- True trajectory vs noisy GPS vs EKF estimate  
- Error convergence plots  
- Quantitative comparison across noise settings  

---

## Key Observations

- EKF significantly reduces positional variance compared to raw GPS.  
- Estimation accuracy degrades predictably with increased measurement noise.  
- Proper tuning of Q (process noise) and R (measurement noise) is critical.  
- Initial covariance influences convergence rate.  
- Under high noise conditions, probabilistic filtering remains stable while raw measurements diverge.  

---

## Relevance to Mobile Robotics

This project demonstrates foundational concepts required for:

- SLAM (Simultaneous Localization and Mapping)  
- Sensor fusion  
- Probabilistic robotics  
- State estimation under uncertainty  
- Navigation in dynamic environments  

These principles form the backbone of modern autonomy stacks used in:

- Autonomous mobile robots  
- UAVs / drones  
- Self-driving systems  

---

## Future Extensions

- Add IMU acceleration integration  
- Implement Unscented Kalman Filter (UKF)  
- Extend to landmark-based EKF-SLAM  
- Compare against Particle Filter  
- Integrate into ROS2 simulation  

---

## Author

**Aayush Ronghe**  
B.E. Electronics & Communication Engineering  
Deep Learning Engineer  
Aspiring Robotics Autonomy Engineer  
