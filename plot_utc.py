import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV data
data = pd.read_csv('data_log.csv')

# Extract and normalize UTC_Timestamp
min_timestamp = data['UTC_Timestamp'].min()
data['Normalized_Time'] = data['UTC_Timestamp'] - min_timestamp
x_values = data['Normalized_Time']

# Compute delta times between successive timestamps in seconds
data['Delta_Time'] = data['Normalized_Time'].diff()

# Convert delta times to milliseconds
data['Delta_Time_ms'] = data['Delta_Time'] * 1000

# Compute mean and standard deviation of delta times in milliseconds
delta_times_ms = data['Delta_Time_ms'].iloc[1:]  # Exclude the first NaN value
mean_delta_ms = delta_times_ms.mean()
std_delta_ms = delta_times_ms.std()

print(f"Mean Delta Time: {mean_delta_ms:.6f} ms")
print(f"Standard Deviation of Delta Time: {std_delta_ms:.6f} ms")

# Create subplots: one for vertical lines, one for delta times
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

# Plot vertical lines for each normalized timestamp on the first subplot
for x in x_values:
    ax1.axvline(x=x, color='b', alpha=0.5)

ax1.set_ylabel('Data Points')
ax1.set_title('Distribution of Data Points Over Time')

# Plot delta times in milliseconds on the second subplot
# Exclude the first timestamp (since the first delta time is NaN)
ax2.plot(x_values.iloc[1:], delta_times_ms, marker='o', linestyle='-', color='r', label='Delta Time')

# Plot mean delta time as a horizontal dotted line
ax2.axhline(y=mean_delta_ms, color='g', linestyle='--', linewidth=2, label='Mean Delta Time')

# Set labels and title
ax2.set_xlabel('Normalized Time (s)')
ax2.set_ylabel('Delta Time (ms)')
ax2.set_title('Delta Times Between Data Points (in milliseconds)')

# Display mean and std deviation on the second subplot
textstr = f'Mean Δt: {mean_delta_ms:.6f} ms\nStd Δt: {std_delta_ms:.6f} ms'
props = dict(boxstyle='round', facecolor='white', alpha=0.5)
ax2.text(0.05, 0.95, textstr, transform=ax2.transAxes, fontsize=10,
         verticalalignment='top', bbox=props)

# Add legend to the second subplot
ax2.legend()

# Adjust layout and display the plot
plt.tight_layout()
plt.show()
