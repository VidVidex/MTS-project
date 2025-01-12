import matplotlib.pyplot as plt
import pandas as pd
import math
import glob
import numpy as np

# Load the data
normal_file = '../data/01_ok.csv'
fault_file = glob.glob('../data/02_*.csv')[0]

normal_frame = pd.read_csv(normal_file)
fault_frame = pd.read_csv(fault_file)

# Detect spring breaking
# top spring: rapid increase in control force and decrease in angle when no angle change is requested
# bottom spring: slight increase in control force and decrease in angle when no angle change is requested
def detect_spring_break(normal_frame, fault_frame, threshold_multiplier=25):

    # Calculate force change when angle change is not requested
    normal_frame['control_force_change'] = np.where(normal_frame['angle_change'],  0, normal_frame['sut.control_force'].diff())
    fault_frame['control_force_change'] = np.where(fault_frame['angle_change'],  0, fault_frame['sut.control_force'].diff())

    # Set the threshold for maximum force change to be threshold_multiplier times the maximum force change in normal frame
    max_force_change_threshold = normal_frame['control_force_change'].max() * threshold_multiplier

    # Mark all rows where force change is greater than the threshold as fault
    fault_frame['fault'] = fault_frame['control_force_change'] > max_force_change_threshold 

    # Find faults
    faults = fault_frame[fault_frame['fault'] == True]

    # Check if we have any faults
    if faults.empty:
        return False, None
    
    # Return first fault
    return True, faults.iloc[0]

# Detect wire breaking
# top wire: desired angle in upward direction not achieved
def detect_top_wire_break(normal_frame, fault_frame):
    normal_frame['desired_current_angle_diff'] = normal_frame['sut.desired_angle'] - normal_frame['sut.crane_angle']
    fault_frame['desired_current_angle_diff'] = fault_frame['sut.desired_angle'] - fault_frame['sut.crane_angle']

    # Find the time when the desired angle is not achieved (with some threshold)
    normal_frame['desired_angle_not_achieved'] = normal_frame['desired_current_angle_diff'] > 10
    fault_frame['desired_angle_not_achieved'] = fault_frame['desired_current_angle_diff'] > 10

    # Find the longest sequence of desired angle not achieved
    normal_frame['desired_angle_not_achieved_count'] = normal_frame['desired_angle_not_achieved'].cumsum()
    fault_frame['desired_angle_not_achieved_count'] = fault_frame['desired_angle_not_achieved'].cumsum()

    # Set a threshold for the number of rows where the desired angle is achieved based on the normal frame
    threshold = normal_frame['desired_angle_not_achieved_count'].max() * 5

    # Find the time when the desired angle is not achieved for more than the threshold
    faults = fault_frame[fault_frame['desired_angle_not_achieved_count'] > threshold]

    # Check if we have any faults
    if faults.empty:
        return False, None

    return True, faults.iloc[0]

    
# Detect bottom wire breaking
# bottom wire: desired angle in downward direction not achieved
def detect_bottom_wire_break(normal_frame, fault_frame):
    normal_frame['desired_current_angle_diff'] = normal_frame['sut.desired_angle'] - normal_frame['sut.crane_angle']
    fault_frame['desired_current_angle_diff'] = fault_frame['sut.desired_angle'] - fault_frame['sut.crane_angle']

    # Find the time when the desired angle is not achieved (with some threshold)
    normal_frame['desired_angle_not_achieved'] = normal_frame['desired_current_angle_diff'] < -10
    fault_frame['desired_angle_not_achieved'] = fault_frame['desired_current_angle_diff'] < -10

    # Find the longest sequence of desired angle not achieved
    normal_frame['desired_angle_not_achieved_count'] = normal_frame['desired_angle_not_achieved'].cumsum()
    fault_frame['desired_angle_not_achieved_count'] = fault_frame['desired_angle_not_achieved'].cumsum()

    # Set a threshold for the number of rows where the desired angle is achieved based on the normal frame
    threshold = normal_frame['desired_angle_not_achieved_count'].max() * 5

    # Find the time when the desired angle is not achieved for more than the threshold
    faults = fault_frame[fault_frame['desired_angle_not_achieved_count'] > threshold]

    # Check if we have any faults
    if faults.empty:
        return False, None

    return True, faults.iloc[0]

# Convert angles to degrees
fault_frame['sut.desired_angle'] = fault_frame['sut.desired_angle'] * 180 / math.pi
fault_frame['sut.crane_angle'] = fault_frame['sut.crane_angle'] * 180 / math.pi
normal_frame['sut.desired_angle'] = normal_frame['sut.desired_angle'] * 180 / math.pi
normal_frame['sut.crane_angle'] = normal_frame['sut.crane_angle'] * 180 / math.pi

# Mark the row where the angle changes and the few rows before and after that (time to stabilize the angle)
before_rows = 100
after_rows = 600
normal_frame['angle_change'] = normal_frame['sut.desired_angle'].diff()
indices = normal_frame[normal_frame['angle_change'] != 0].index
for index in indices:
    normal_frame.loc[max(index-before_rows, 0):index+after_rows, 'angle_change'] = 1

fault_frame['angle_change'] = fault_frame['sut.desired_angle'].diff()
indices = fault_frame[fault_frame['angle_change'] != 0].index
for index in indices:
    fault_frame.loc[max(index-before_rows, 0):index+after_rows, 'angle_change'] = 1

# Detect top spring breaking
found, fault = detect_spring_break(normal_frame, fault_frame, 25)
if found:
    print(f'Top spring break fault detected at time: {fault["time"]}')
    exit()

# Detect bottom spring breaking
found, fault = detect_spring_break(normal_frame, fault_frame, 5)
if found:
    print(f'Bottom spring break fault detected at time: {fault["time"]}')
    exit()

# Detect top wire breaking
found, fault = detect_top_wire_break(normal_frame, fault_frame)
if found:
    print(f'Top wire break fault detected sometime before time: {fault["time"]}')
    exit()

# Detect bottom wire breaking
found, fault = detect_bottom_wire_break(normal_frame, fault_frame)
if found:
    print(f'Bottom wire break fault detected sometime before time: {fault["time"]}')
    exit()


print('No fault detected')

