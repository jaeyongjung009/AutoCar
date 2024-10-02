import pandas as pd
import matplotlib.pyplot as plt

# 데이터 로드
df = pd.read_csv('imu_data.csv')

# 열 데이터를 NumPy 배열로 변환
timestamps = df['Timestamp'].values
print(df['Timestamp'].describe())

linear_accelerations_x = df['Linear_Acceleration_x'].values
linear_accelerations_y = df['Linear_Acceleration_y'].values
linear_accelerations_z = df['Linear_Acceleration_z'].values
angular_velocities_x = df['Angular_Velocity_x'].values
angular_velocities_y = df['Angular_Velocity_y'].values
angular_velocities_z = df['Angular_Velocity_z'].values

# 타임스탬프를 1초씩 증가시키기
# 시작 타임스탬프
start_time = 0

# 타임스탬프 배열을 생성 (1초 간격으로 증가)
timestamps_simulated = [start_time + i for i in range(len(timestamps))]

# 시각화
plt.figure(figsize=(14, 10))

# Linear Acceleration 데이터 시각화
plt.subplot(2, 1, 1)
plt.plot(timestamps_simulated, linear_accelerations_x, label='X', color='r')
plt.plot(timestamps_simulated, linear_accelerations_y, label='Y', color='g')
plt.plot(timestamps_simulated, linear_accelerations_z, label='Z', color='b')
plt.title('Linear Acceleration')
plt.xlabel('Timestamp')
plt.ylabel('Value')
plt.legend()

# Angular Velocity 데이터 시각화
plt.subplot(2, 1, 2)
plt.plot(timestamps_simulated, angular_velocities_x, label='X', color='r')
plt.plot(timestamps_simulated, angular_velocities_y, label='Y', color='g')
plt.plot(timestamps_simulated, angular_velocities_z, label='Z', color='b')
plt.title('Angular Velocity')
plt.xlabel('Timestamp')
plt.ylabel('Value')
plt.legend()

plt.tight_layout()
plt.show()
