import numpy as np
import matplotlib.pyplot as plt
import os
import pandas as pd

np.random.seed(42)

summary_results = []

# -----------------------------
# Core EKF Function
# -----------------------------
def run_ekf_experiment(experiment_type,
                       parameter_value,
                       gps_noise_std,
                       process_noise_scale,
                       omega,
                       folder_name):

    dt = 0.1
    steps = 400

    os.makedirs(folder_name, exist_ok=True)

    # Ground truth
    x_true = np.array([0.0, 0.0, 1.0, 0.0])
    true_states = []
    gps_measurements = []

    for _ in range(steps):
        x_true[0] += x_true[2] * np.cos(x_true[3]) * dt
        x_true[1] += x_true[2] * np.sin(x_true[3]) * dt
        x_true[3] += omega * dt

        true_states.append(x_true.copy())

        gps_x = x_true[0] + np.random.normal(0, gps_noise_std)
        gps_y = x_true[1] + np.random.normal(0, gps_noise_std)
        gps_measurements.append([gps_x, gps_y])

    true_states = np.array(true_states)
    gps_measurements = np.array(gps_measurements)

    # EKF init
    x_est = np.array([0.0, 0.0, 1.0, 0.0])
    P = np.eye(4)
    Q = np.eye(4) * process_noise_scale
    R = np.eye(2) * gps_noise_std**2

    estimated_states = []
    ekf_errors = []
    gps_errors = []

    for i in range(steps):

        theta = x_est[3]
        v = x_est[2]

        # Prediction
        x_pred = np.array([
            x_est[0] + v * np.cos(theta) * dt,
            x_est[1] + v * np.sin(theta) * dt,
            v,
            theta + omega * dt
        ])

        F = np.array([
            [1, 0, np.cos(theta)*dt, -v*np.sin(theta)*dt],
            [0, 1, np.sin(theta)*dt,  v*np.cos(theta)*dt],
            [0, 0, 1,                0],
            [0, 0, 0,                1]
        ])

        P = F @ P @ F.T + Q

        # Update
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        z = gps_measurements[i]
        z_pred = H @ x_pred

        y = z - z_pred
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)

        x_est = x_pred + K @ y
        P = (np.eye(4) - K @ H) @ P

        estimated_states.append(x_est.copy())

        true_xy = true_states[i, :2]
        ekf_errors.append(np.linalg.norm(true_xy - x_est[:2]))
        gps_errors.append(np.linalg.norm(true_xy - gps_measurements[i]))

    estimated_states = np.array(estimated_states)
    ekf_errors = np.array(ekf_errors)
    gps_errors = np.array(gps_errors)

    ekf_rmse = np.sqrt(np.mean(ekf_errors**2))
    gps_rmse = np.sqrt(np.mean(gps_errors**2))
    improvement = (1 - ekf_rmse/gps_rmse) * 100

    # Save metrics text
    with open(os.path.join(folder_name, "metrics.txt"), "w") as f:
        f.write(f"GPS RMSE: {gps_rmse:.4f}\n")
        f.write(f"EKF RMSE: {ekf_rmse:.4f}\n")
        f.write(f"Improvement (%): {improvement:.2f}\n")

    # Save trajectory plot
    plt.figure(figsize=(8,6))
    plt.plot(true_states[:,0], true_states[:,1], label="True Path")
    plt.scatter(gps_measurements[:,0], gps_measurements[:,1],
                s=10, alpha=0.3, label="Noisy GPS")
    plt.plot(estimated_states[:,0], estimated_states[:,1],
             label="EKF Estimate", linewidth=2)
    plt.legend()
    plt.grid()
    plt.title("2D EKF Localization")
    plt.savefig(os.path.join(folder_name, "trajectory.png"))
    plt.close()

    # Save error plot
    plt.figure(figsize=(8,4))
    plt.plot(gps_errors, label="GPS Error", alpha=0.6)
    plt.plot(ekf_errors, label="EKF Error", linewidth=2)
    plt.legend()
    plt.grid()
    plt.title("Error Over Time")
    plt.savefig(os.path.join(folder_name, "error_plot.png"))
    plt.close()

    # Append to global summary
    summary_results.append({
        "experiment_type": experiment_type,
        "parameter_value": parameter_value,
        "gps_rmse": gps_rmse,
        "ekf_rmse": ekf_rmse,
        "improvement_percent": improvement
    })


# -----------------------------
# RUN ALL EXPERIMENTS
# -----------------------------

base_dir = "results"
os.makedirs(base_dir, exist_ok=True)

# 1️⃣ GPS Noise Sweep
gps_values = [0.2, 0.5, 1.0, 2.0]
for gps in gps_values:
    folder = os.path.join(base_dir, f"gps_noise_{gps}")
    run_ekf_experiment("gps_noise", gps,
                       gps_noise_std=gps,
                       process_noise_scale=0.01,
                       omega=0.2,
                       folder_name=folder)

# 2️⃣ Process Noise Sweep
process_values = [0.001, 0.01, 0.1]
for q in process_values:
    folder = os.path.join(base_dir, f"process_noise_{q}")
    run_ekf_experiment("process_noise", q,
                       gps_noise_std=0.5,
                       process_noise_scale=q,
                       omega=0.2,
                       folder_name=folder)

# 3️⃣ Angular Velocity Sweep
omega_values = [0.05, 0.2, 0.5]
for w in omega_values:
    folder = os.path.join(base_dir, f"omega_{w}")
    run_ekf_experiment("angular_velocity", w,
                       gps_noise_std=0.5,
                       process_noise_scale=0.01,
                       omega=w,
                       folder_name=folder)

# -----------------------------
# Save Summary CSV
# -----------------------------

df = pd.DataFrame(summary_results)
df.to_csv(os.path.join(base_dir, "summary.csv"), index=False)

print("All experiments completed.")
print("Summary saved to results/summary.csv")
