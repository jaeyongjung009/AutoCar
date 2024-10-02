import pandas as pd
import matplotlib.pyplot as plt
import os

# 파일 경로 지정
csv_file = '/home/adg/catkin_ws/imu_mag_data.csv'

# 파일이 존재하는지 확인
if not os.path.exists(csv_file):
    print(f"File not found: {csv_file}")
else:
    # 데이터 로드
    df = pd.read_csv(csv_file)

    # 열 데이터를 NumPy 배열로 변환
    timestamps = df['Timestamp'].values
    print(df['Timestamp'].describe())

    magnetic_field_x = df['MagneticField_x'].values
    magnetic_field_y = df['MagneticField_y'].values
    magnetic_field_z = df['MagneticField_z'].values

    # 타임스탬프를 1초씩 증가시키기
    start_time = 0
    timestamps_simulated = [start_time + i for i in range(len(timestamps))]

    # 시각화
    plt.figure(figsize=(14, 7))

    # Magnetic Field 데이터 시각화
    plt.plot(timestamps_simulated, magnetic_field_x, label='MagneticField_x', color='r')
    plt.plot(timestamps_simulated, magnetic_field_y, label='MagneticField_y', color='g')
    plt.plot(timestamps_simulated, magnetic_field_z, label='MagneticField_z', color='b')
    plt.title('Magnetic Field Data')
    plt.xlabel('Timestamp (simulated)')
    plt.ylabel('Magnetic Field (T)')
    plt.legend()

    plt.tight_layout()
    plt.show()
