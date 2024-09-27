import pandas as pd
import os
from datetime import datetime
import plotly.graph_objects as go
from plotly.subplots import make_subplots

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

# Create subplots using Plotly
fig = make_subplots(rows=2, cols=1, shared_xaxes=True, vertical_spacing=0.1,
                    subplot_titles=('Distribution of Data Points Over Time', 'Delta Times Between Data Points (in milliseconds)'))

# Plot vertical lines for each normalized timestamp on the first subplot
for x in x_values:
    fig.add_trace(go.Scatter(x=[x, x], y=[0, 1], mode='lines', line=dict(color='blue', width=1), showlegend=False), row=1, col=1)

# Plot delta times in milliseconds on the second subplot
fig.add_trace(go.Scatter(x=x_values.iloc[1:], y=delta_times_ms, mode='lines+markers', marker=dict(color='red'), name='Delta Time'), row=2, col=1)

# Plot mean delta time as a horizontal dotted line
fig.add_trace(go.Scatter(x=[x_values.iloc[1:].min(), x_values.iloc[1:].max()], y=[mean_delta_ms, mean_delta_ms], mode='lines', line=dict(color='green', dash='dash', width=2), name='Mean Delta Time'), row=2, col=1)

# Set labels and title for the second subplot
fig.update_xaxes(title_text='Normalized Time (s)', row=2, col=1)
fig.update_yaxes(title_text='Delta Time (ms)', row=2, col=1)

# Set layout and titles
fig.update_layout(
    autosize=True,
    title_text='Data Points and Delta Times Analysis',
    margin=dict(l=20, r=20, t=40, b=20)
)

# Display mean and std deviation on the second subplot
textstr = f'Mean Δt: {mean_delta_ms:.6f} ms\nStd Δt: {std_delta_ms:.6f} ms'
fig.add_annotation(
    text=textstr,
    xref="paper", yref="paper",
    x=0.05, y=0.95, showarrow=False,
    font=dict(size=12),
    align="left",
    bordercolor="black", borderwidth=1,
    borderpad=4, bgcolor="white", opacity=0.8,
    row=2, col=1
)


# Create the subfolder if it doesn't exist
subfolder = "plot_html"
if not os.path.exists(subfolder):
    os.makedirs(subfolder)

# Generate the timestamped filename
timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
html_filename = f"{timestamp_str}.html"

# Construct the full path for the HTML file
full_path = os.path.join(subfolder, html_filename)

# Save the plot as an HTML file
fig.write_html(full_path)

print(f"Plot saved as {full_path}")

# Show the plot
fig.show()