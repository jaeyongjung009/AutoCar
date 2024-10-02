import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# 데이터 로드
file_path = '/home/adg/catkin_ws/imu_mag_data.csv'  # 적절한 파일 경로로 수정하세요
data = pd.read_csv(file_path)

# Yaw (Heading) 계산
data['Heading'] = (np.arctan2(data['MagneticField_y'], data['MagneticField_x']) * (180.0 / np.pi))

# Heading 값을 0-360도 범위로 정규화
data['Heading'] = (data['Heading'] + 360) % 360

# 타임스탬프를 더 읽기 쉬운 형식으로 변환 (필요한 경우)
data['Timestamp'] = pd.to_datetime(data['Timestamp'], unit='s')

# 'Timestamp'와 'Heading'을 numpy 배열로 변환
timestamps = data['Timestamp'].values
headings = data['Heading'].values

# Yaw 값을 시간에 따라 플로팅
plt.figure(figsize=(10, 6))
plt.plot(timestamps, headings, label='Yaw (Heading)')
plt.xlabel('Time')
plt.ylabel('Yaw (Degrees)')
plt.title('Yaw over Time')
plt.grid(True)
plt.legend()
plt.xticks(rotation=45)
plt.tight_layout()
plt.show()
