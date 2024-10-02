import pandas as pd
import matplotlib.pyplot as plt

# CSV 파일 읽기 (구분자가 쉼표)
df = pd.read_csv('encoder_log.csv', delimiter=',')

# 데이터프레임 구조 확인
print(df.head())  # 데이터가 올바르게 로드되었는지 확인

# 열 이름에서 공백 제거
df.columns = df.columns.str.strip()

# Timestamp를 datetime 형식으로 변환
df['Timestamp'] = pd.to_datetime(df['Timestamp'], unit='s')

# Numpy 배열로 변환
timestamps = df['Timestamp'].to_numpy()
encoder_counts = df['EncoderCount'].to_numpy()

# 그래프 그리기
plt.figure(figsize=(10, 6))
plt.plot(timestamps, encoder_counts, marker='o', linestyle='-')

# 그래프에 제목 및 레이블 추가
plt.title('Encoder Count Over Time')
plt.xlabel('Time')
plt.ylabel('Encoder Count')

# X축 타임스탬프 포맷 조정
plt.xticks(rotation=45, ha='right')

# 그리드 추가
plt.grid(True)

# 그래프 레이아웃 조정 및 표시
plt.tight_layout()
plt.show()
