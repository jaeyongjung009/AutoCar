import pandas as pd
import matplotlib.pyplot as plt
import os

# 파일 경로 지정
csv_file = '/home/adg/catkin_ws/imu_filtered_mag_data.csv'

# 파일이 존재하는지 확인
if not os.path.exists(csv_file):
    print(f"File not found: {csv_file}")
else:
    # 데이터 로드
    df = pd.read_csv(csv_file)

    # 데이터의 기본 통계 출력
    print(df.describe())

    # 열 데이터를 NumPy 배열로 변환
    timestamps = df['Timestamp'].values
    magnetic_field_x = df['MagneticField_x'].values
    magnetic_field_y = df['MagneticField_y'].values
    magnetic_field_z = df['MagneticField_z'].values

    # 시각화
    plt.figure(figsize=(14, 7))

    # Magnetic Field 데이터 시각화
    plt.plot(timestamps, magnetic_field_x, label='MagneticField_x', color='r')
    plt.plot(timestamps, magnetic_field_y, label='MagneticField_y', color='g')
    plt.plot(timestamps, magnetic_field_z, label='MagneticField_z', color='b')

    plt.title('Magnetic Field Data')
    plt.xlabel('Timestamp (s)')
    plt.ylabel('Magnetic Field (T)')
    plt.legend()

    plt.tight_layout()
    plt.show()
