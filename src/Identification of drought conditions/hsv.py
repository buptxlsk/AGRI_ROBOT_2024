import cv2
import numpy as np

def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # 获取点击点的BGR值
        bgr = image[y, x]
        # 转换为HSV
        hsv = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0][0]
        print(f'点击点坐标: ({x}, {y}), HSV值: {hsv}')

# 图片路径
image_path = "D:\\Code\\test.png"

# 读取图片
image = cv2.imread(image_path)
if image is None:
    print('无法读取图片，请检查文件路径和文件格式。')
else:
    cv2.imshow('Image', image)
    cv2.setMouseCallback('Image', click_event)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
