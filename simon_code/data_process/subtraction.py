import pandas as pd

# Load both CSVs
measured_file = 'coulomb_fric_test/PSM2-measured_js.csv'  # update if needed
gravity_file = 'coulomb_fric_test/PSM2-gravity_compensation-setpoint_js.csv'

df_measured = pd.read_csv(measured_file)
df_gravity = pd.read_csv(gravity_file)

# Trim to the length of the shorter DataFrame
min_len = min(len(df_measured), len(df_gravity))
df_measured = df_measured.iloc[:min_len].reset_index(drop=True)
df_gravity = df_gravity.iloc[:min_len].reset_index(drop=True)

# Subtract each effort column
for i in range(6):
    measured_col = f'effort_{i}'
    gravity_col = f'effort_{i}'
    result_col = f'effort_diff_{i}'
    df_measured[result_col] = df_measured[measured_col] - df_gravity[gravity_col]

# Save to new file
output_file = 'effort_difference_test.csv'
df_measured.to_csv(output_file, index=False)

print(f'Saved difference CSV to {output_file}')
