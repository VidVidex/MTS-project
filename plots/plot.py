import matplotlib.pyplot as plt
import pandas as pd
import math
import glob

# Load the data
normal_file = '../data/01_ok.csv'
fault_file = glob.glob('../data/04_*.csv')[0]

normal_frame = pd.read_csv(normal_file)
fault_frame = pd.read_csv(fault_file)

fault_time = 30

# Convert angles to degrees
fault_frame['sut.desired_angle'] = fault_frame['sut.desired_angle'] * 180 / math.pi
fault_frame['sut.crane_angle'] = fault_frame['sut.crane_angle'] * 180 / math.pi
normal_frame['sut.desired_angle'] = normal_frame['sut.desired_angle'] * 180 / math.pi
normal_frame['sut.crane_angle'] = normal_frame['sut.crane_angle'] * 180 / math.pi

# Set global font size
plt.rcParams.update({'font.size': 14})  # Adjust font size as needed

# Create the subplots
fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

# Plot each variable on a separate subplot
# axes[0].plot(normal_frame['time'], normal_frame['sut.control_force'], label='Correct behaviour', color='blue')
axes[0].plot(fault_frame['time'], fault_frame['sut.control_force'], label='Faulty behaviour', color='red')
axes[0].axvline(x=fault_time, color='green', linestyle='--', label='Fault occurred')
axes[0].set_ylabel('Control force [N]')
axes[0].legend(fontsize=12)  # Optional: Adjust legend font size

# axes[1].plot(normal_frame['time'], normal_frame['sut.crane_wire_join.measured_weight'], label='Correct behaviour', color='blue')
axes[1].plot(fault_frame['time'], fault_frame['sut.crane_wire_join.measured_weight'], label='Faulty behaviour', color='red')
axes[1].axvline(x=fault_time, color='green', linestyle='--', label='Fault occurred')
axes[1].set_ylabel('Measured weight of load [N]')
axes[1].legend(fontsize=12)

# axes[2].plot(normal_frame['time'], normal_frame['sut.desired_angle'], label='Correct behaviour', color='blue')
axes[2].plot(fault_frame['time'], fault_frame['sut.desired_angle'], label='Faulty behaviour', color='red')
axes[2].axvline(x=fault_time, color='green', linestyle='--', label='Fault occurred')
axes[2].set_ylabel('Desired Crane angle [deg]')
axes[2].legend(fontsize=12)

# axes[3].plot(normal_frame['time'], normal_frame['sut.crane_angle'], label='Correct behaviour', color='blue')
axes[3].plot(fault_frame['time'], fault_frame['sut.crane_angle'], label='Faulty behaviour', color='red')
axes[3].axvline(x=fault_time, color='green', linestyle='--', label='Fault occurred')
axes[3].set_ylabel('Current crane angle [deg]')
axes[3].legend(fontsize=12)

# Add a common x-label for all subplots
plt.xlabel('Time', fontsize=16)

# Adjust layout for better spacing
plt.tight_layout()

# Display the plot
plt.show()

# Save the plot
fig.savefig('../plots/plot.png')
