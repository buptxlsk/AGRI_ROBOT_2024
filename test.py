import cv2
from ultralytics import YOLO
from collections import Counter

# 初始化YOLOv8模型
model = YOLO('best.pt')  # 使用YOLOv8n的预训练模型，可以根据需要更换模型

def main():
    # 打开摄像头
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("无法打开摄像头")
        return

    frame_count = 0
    detections = []

    while True:
        # 读取帧
        ret, frame = cap.read()
        if not ret:
            print("无法接收帧，结束...")
            break

        frame_width = frame.shape[1]
        mid_line = frame_width / 2

        # 检测对象
        results = model.predict(frame)

        # 初始化最优的检测结果
        best_box = None
        best_label = None
        best_confidence = 0
        min_score = float('inf')

        for result in results:
            boxes = result.boxes
            for box in boxes:
                confidence = box.conf.item()
                coords = box.xyxy.cpu().numpy().astype(int).flatten()
                x1, y1, x2, y2 = coords

                # 计算框的中心点
                box_mid = (x1 + x2) / 2

                # 计算距离差值并归一化
                distance = abs(box_mid - mid_line) / mid_line

                # 计算加权得分
                score = 0.7 * (1 - confidence) + 0.3 * distance

                if score < min_score:
                    min_score = score
                    best_box = box
                    best_label = result.names[int(box.cls)]
                    best_confidence = confidence

        # 如果找到了最优的检测结果，绘制边界框和标签
        if best_box is not None:
            coords = best_box.xyxy.cpu().numpy().astype(int).flatten()
            x1, y1, x2, y2 = coords
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{best_label} {best_confidence:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            detections.append(best_label)

        # 每10帧打印一次出现次数最多的类型
        frame_count += 1
        if frame_count % 10 == 0:
            if detections:
                most_common_detection = Counter(detections).most_common(1)[0][0]
                print(f"I SEE {most_common_detection}!")
            detections = []  # 重置检测结果

        # 显示帧
        cv2.imshow('YOLOv8 Detection', frame)

        # 按下'q'键退出循环
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放摄像头并关闭所有窗口
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
