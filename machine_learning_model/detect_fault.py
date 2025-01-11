import pandas as pd
from sklearn.ensemble import IsolationForest
from sklearn.preprocessing import StandardScaler

case = '02'

# Load data
normal_data = pd.read_csv(f'data/{case}_normal.csv')
faulty_data = pd.read_csv(f'data/{case}_faulty.csv')

# Extract features
features = ["sut.control_force", "sut.m_load", "sut.desired_angle", "sut.crane_angle"]
X_train = normal_data[features]
X_test = faulty_data[features]

# Scale data
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled = scaler.transform(X_test)

# Train Isolation Forest
iso_forest = IsolationForest(contamination=0.001, random_state=42)
iso_forest.fit(X_train_scaled)

# Predict on faulty data
anomaly_scores = iso_forest.decision_function(X_test_scaled)

# Add results to the faulty dataset
faulty_data['anomaly_score'] = anomaly_scores

# Create threshold for anomaly detection as the minimum anomaly score with a 100% margin. Do not use the first 10 seconds (time < 10) of data since the model is stabilizing
minimum_anomaly_score = faulty_data[faulty_data['time'] > 10]['anomaly_score'].min()
minimum_anomaly_score += abs(minimum_anomaly_score)
print(f'Minimum anomaly score: {minimum_anomaly_score}')

# Detect fault by finding enough consecutive anomalies
fault_count = 250
faulty_data['is_anomaly'] = faulty_data['anomaly_score'] < minimum_anomaly_score
faulty_data['is_anomaly'] = faulty_data['is_anomaly'].rolling(window=fault_count).sum() == fault_count

# Find the first anomaly index, we have to go 'fault_count' steps back to find the actual start of the fault
first_anomaly_index = faulty_data[faulty_data['is_anomaly']].index[0] - fault_count
first_fault = faulty_data.loc[first_anomaly_index]

# Set all rows after the first fault to be anomaly
faulty_data.loc[first_anomaly_index:, 'is_anomaly'] = True

print(f'Fault occurred at time: {first_fault["time"]}')

import matplotlib.pyplot as plt


plt.figure(figsize=(10, 6))
plt.plot(faulty_data['time'], faulty_data['anomaly_score'], label='Anomaly Score')
plt.scatter(faulty_data['time'][faulty_data['is_anomaly']], 
            faulty_data['anomaly_score'][faulty_data['is_anomaly']], 
            color='red', label='Anomaly')
plt.xlabel('Time')
plt.ylabel('Anomaly Score')
plt.legend()
plt.title(f'Anomaly Detection Results for case {case}')
plt.show()