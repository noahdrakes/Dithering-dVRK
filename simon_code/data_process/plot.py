import pandas as pd
import matplotlib.pyplot as plt

# Load the result CSV with effort differences
df = pd.read_csv('effort_difference_test.csv')

# Shift time to start at 0
df['time_shifted'] = df['Time'] - df['Time'].iloc[0]

# Filter for time >= 3.8s
df_filtered = df[df['time_shifted'] >= 1]

# Extract values for plotting
time = df['time_shifted']
effort_diff_1 = df['effort_diff_1']
velocity_1 = df['velocity_1']
position_1 = df['position_1']

effort_diff_1_filtered = df_filtered['effort_diff_1']
velocity_1_filtered = df_filtered['velocity_1']
position_1_filtered = df_filtered['position_1']

# --- Plot 1: Effort Difference vs Time (full trace) ---
plt.figure(figsize=(10, 4))
plt.plot(time, effort_diff_1, label='effort_diff_2', color='tab:blue')
plt.title('Effort Difference vs Time (Joint 1)')
plt.xlabel('Time [s]')
plt.ylabel('Effort Difference')
plt.grid(True)
plt.legend()
plt.tight_layout()

plt.savefig('eff_dif_time_test.png', dpi=300) 

# --- Plot 2: Effort Difference vs Velocity (after 3.8s) ---
plt.figure(figsize=(6, 6))
plt.scatter(velocity_1_filtered, effort_diff_1_filtered, s=10, color='tab:red', alpha=0.7)
plt.title('Effort Difference vs Velocity (Joint 1) [t ≥ 1s]')
plt.xlabel('Velocity [rad/s]')
plt.ylabel('Effort Difference')
plt.grid(True)
plt.tight_layout()

plt.savefig('eff_dif_vel_test.png', dpi=300) 

# --- Plot 2: Effort Difference vs Position (after 3.8s) ---
plt.figure(figsize=(6, 6))
plt.scatter(position_1_filtered, effort_diff_1_filtered, s=10, color='tab:red', alpha=0.7)
plt.title('Effort Difference vs Position (Joint 1) [t ≥ 1s]')
plt.xlabel('Position [rad]')
plt.ylabel('Effort Difference')
plt.grid(True)
plt.tight_layout()

plt.savefig('eff_dif_pos_test.png', dpi=300) 

plt.show()
