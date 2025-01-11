import matplotlib.pyplot as plt
import pandas as pd
import math

# Load the data
file = '../data/03_faulty.csv'
frame = pd.read_csv(file)

# Define fault occurrence and detection points
fault_occurred_at = 30
fault_detected_at = 20

# Convert angles to degrees
frame['sut.desired_angle'] = frame['sut.desired_angle'] * 180 / math.pi
frame['sut.crane_angle'] = frame['sut.crane_angle'] * 180 / math.pi

# Create the subplots
fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

# Plot each variable on a separate subplot
axes[0].plot(frame['time'], frame['sut.control_force'], label='Control force', color='blue')
axes[0].axvline(x=fault_occurred_at, color='red', linestyle='--', label='Fault occurred')
axes[0].axvline(x=fault_detected_at, color='green', linestyle='--', label='Fault detected')
axes[0].set_ylabel('Control force [N]')
axes[0].legend()

axes[1].plot(frame['time'], frame['sut.m_load'], label='Weight of load', color='orange')
axes[1].axvline(x=fault_occurred_at, color='red', linestyle='--', label='Fault occurred')
axes[1].axvline(x=fault_detected_at, color='green', linestyle='--', label='Fault detected')
axes[1].set_ylabel('Weight of load [kg]')
axes[1].legend()

axes[2].plot(frame['time'], frame['sut.desired_angle'], label='Desired angle', color='purple')
axes[2].axvline(x=fault_occurred_at, color='red', linestyle='--', label='Fault occurred')
axes[2].axvline(x=fault_detected_at, color='green', linestyle='--', label='Fault detected')
axes[2].set_ylabel('Desired Crane angle [deg]')
axes[2].legend()

axes[3].plot(frame['time'], frame['sut.crane_angle'], label='Crane angle', color='brown')
axes[3].axvline(x=fault_occurred_at, color='red', linestyle='--', label='Fault occurred')
axes[3].axvline(x=fault_detected_at, color='green', linestyle='--', label='Fault detected')
axes[3].set_ylabel('Current crane angle [deg]')
axes[3].legend()

# Add a common x-label for all subplots
plt.xlabel('Time')

# Adjust layout for better spacing
plt.tight_layout()

# Display the plot
plt.show()
