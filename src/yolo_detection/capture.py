import cv2
import os

# 创建图像保存的文件夹
image_folder = 'images'
if not os.path.exists(image_folder):
    os.makedirs(image_folder)

# 获取文件夹内已有的照片编号
def get_next_image_number(folder):
    images = [f for f in os.listdir(folder) if f.startswith('image_') and f.endswith('.jpg')]
    if not images:
        return 1
    numbers = [int(img.split('_')[1].split('.')[0]) for img in images]
    return max(numbers) + 1

# 获取下一个照片编号
image_number = get_next_image_number(image_folder)

# 打开摄像头
cap = cv2.VideoCapture('/dev/video0')

if not cap.isOpened():
    print("Error: Could not open video device.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture image")
        break
    
    # 显示画面
    cv2.imshow('Camera', frame)
    
    # 监听键盘事件
    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):
        # 保存照片
        image_path = os.path.join(image_folder, f'image_{image_number}.jpg')
        cv2.imwrite(image_path, frame)
        print(f'Saved {image_path}')
        image_number += 1
    elif key == ord('q'):
        # 退出
        break

# 释放摄像头并关闭所有窗口
cap.release()
cv2.destroyAllWindows()
