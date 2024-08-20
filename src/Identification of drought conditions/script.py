import cv2
import numpy as np
import argparse
import json
import os

# 全局变量
image = None
click_phase = 'red'
color_ranges_file = "color_ranges.json"
output_file = "conditions.txt"

# 读取颜色范围
def load_color_ranges():
    if os.path.exists(color_ranges_file):
        with open(color_ranges_file, "r") as file:
            data = json.load(file)
            return (
                np.array(data["red_lower"]),
                np.array(data["red_upper"]),
                np.array(data["green_lower"]),
                np.array(data["green_upper"]),
                np.array(data["blue_lower"]),
                np.array(data["blue_upper"]),
            )
    return (
        np.array([0, 0, 0]),
        np.array([0, 0, 0]),
        np.array([0, 0, 0]),
        np.array([0, 0, 0]),
        np.array([0, 0, 0]),
        np.array([0, 0, 0]),
    )

# 保存颜色范围
def save_color_ranges(red_lower, red_upper, green_lower, green_upper, blue_lower, blue_upper):
    data = {
        "red_lower": red_lower.tolist(),
        "red_upper": red_upper.tolist(),
        "green_lower": green_lower.tolist(),
        "green_upper": green_upper.tolist(),
        "blue_lower": blue_lower.tolist(),
        "blue_upper": blue_upper.tolist(),
    }
    with open(color_ranges_file, "w") as file:
        json.dump(data, file)

# 写入颜色矩阵到文件
def write_color_matrix_to_file(color_matrix):
    with open(output_file, "w") as file:
        for row in color_matrix:
            file.write(" ".join(map(str, row)) + "\n")

# 将颜色矩阵转换为字符串
def convert_color_matrix_to_string(color_matrix):
    col1 = ''.join(map(str, color_matrix[:, 0]))
    col2 = ''.join(map(str, color_matrix[:, 1]))
    col3 = ''.join(map(str, color_matrix[:, 2]))
    return col1, col2, col3

red_lower, red_upper, green_lower, green_upper, blue_lower, blue_upper = load_color_ranges()

def click_event(event, x, y, flags, param):
    global red_lower, red_upper, green_lower, green_upper, blue_lower, blue_upper, click_phase
    if event == cv2.EVENT_LBUTTONDOWN:
        # 获取点击点的BGR值
        bgr = image[y, x]
        # 转换为HSV
        hsv = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0][0]
        print(f'点击点坐标: ({x}, {y}), HSV值: {hsv}')

        # 根据选择更新相应颜色的范围
        if click_phase == 'red':
            red_lower = np.array([hsv[0] - 20, max(hsv[1] - 20, 0), max(hsv[2] - 20, 0)])
            red_upper = np.array([hsv[0] + 20, min(hsv[1] + 20, 255), min(hsv[2] + 20, 255)])
            print(f'红色范围更新为: lower={red_lower}, upper={red_upper}')
            click_phase = 'green'
            print("请点击绿色色块")
        elif click_phase == 'green':
            green_lower = np.array([hsv[0] - 20, max(hsv[1] - 20, 0), max(hsv[2] - 20, 0)])
            green_upper = np.array([hsv[0] + 20, min(hsv[1] + 20, 255), min(hsv[2] + 20, 255)])
            print(f'绿色范围更新为: lower={green_lower}, upper={green_upper}')
            click_phase = 'blue'
            print("请点击蓝色色块")
        elif click_phase == 'blue':
            blue_lower = np.array([hsv[0] - 20, max(hsv[1] - 20, 0), max(hsv[2] - 20, 0)])
            blue_upper = np.array([hsv[0] + 20, min(hsv[1] + 20, 255), min(hsv[2] + 20, 255)])
            print(f'蓝色范围更新为: lower={blue_lower}, upper={blue_upper}')
            click_phase = 'done'
            save_color_ranges(red_lower, red_upper, green_lower, green_upper, blue_lower, blue_upper)
            cv2.destroyAllWindows()

def get_contours(image, lower_bound, upper_bound):
    mask = cv2.inRange(image, lower_bound, upper_bound)
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=1)
    mask = cv2.erode(mask, kernel, iterations=1)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def calculate_centers(contours):
    centers = []
    for contour in contours:
        M = cv2.moments(contour)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            centers.append((cx, cy))
    return centers

def find_bounding_box(centers):
    if not centers:
        return None, None
    xs, ys = zip(*centers)
    top_left = (min(xs), min(ys))
    bottom_right = (max(xs), max(ys))
    return top_left, bottom_right

def get_grid_index(x, y, grid_width, grid_height, top_left):
    col = (x - top_left[0]) // grid_width
    row = (y - top_left[1]) // grid_height
    return row, col

def find_nearest_color(center, red_centers, green_centers, blue_centers):
    nearest_center = None
    nearest_color = 0
    nearest_distance = float('inf')

    for c in red_centers:
        distance = np.linalg.norm(np.array(center) - np.array(c))
        if distance < nearest_distance:
            nearest_distance = distance
            nearest_center = c
            nearest_color = 1

    for c in green_centers:
        distance = np.linalg.norm(np.array(center) - np.array(c))
        if distance < nearest_distance:
            nearest_distance = distance
            nearest_center = c
            nearest_color = 2

    for c in blue_centers:
        distance = np.linalg.norm(np.array(center) - np.array(c))
        if distance < nearest_distance:
            nearest_distance = distance
            nearest_center = c
            nearest_color = 3

    return nearest_color

def print_color_ranges():
    global red_lower, red_upper, green_lower, green_upper, blue_lower, blue_upper
    print("当前颜色的掩膜范围:")
    print(f"红色: lower={red_lower}, upper={red_upper}")
    print(f"绿色: lower={green_lower}, upper={green_upper}")
    print(f"蓝色: lower={blue_lower}, upper={blue_upper}")

def main():
    global image, red_lower, red_upper, green_lower, green_upper, blue_lower, blue_upper, click_phase

    parser = argparse.ArgumentParser(description="Python script for image processing")
    parser.add_argument('mode', type=int, choices=[1, 2, 3], help='Mode 1: Determine HSV, Mode 2: Detect Colors, Mode 3: Print Color Ranges')
    args = parser.parse_args()

    image_path = "D:\\Code\\AGRI_ROBOT_2024\\src\\Identification of drought conditions\\test.png"
    image = cv2.imread(image_path)
    if image is None:
        print('无法读取图片，请检查文件路径和文件格式。')
        return

    if args.mode == 1:
        click_phase = 'red'
        cv2.imshow('Image', image)
        print("请点击红色色块")
        cv2.setMouseCallback('Image', click_event)
        while click_phase != 'done':
            cv2.waitKey(1)
    elif args.mode == 2:
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        red_contours = get_contours(hsv_image, red_lower, red_upper)
        green_contours = get_contours(hsv_image, green_lower, green_upper)
        blue_contours = get_contours(hsv_image, blue_lower, blue_upper)

        red_centers = calculate_centers(red_contours)
        green_centers = calculate_centers(green_contours)
        blue_centers = calculate_centers(blue_contours)

        for i, center in enumerate(red_centers):
            cv2.circle(image, center, 5, (0, 0, 255), -1)
            cv2.putText(image, f'R{i+1}', (center[0]+10, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        for i, center in enumerate(green_centers):
            cv2.circle(image, center, 5, (0, 255, 0), -1)
            cv2.putText(image, f'G{i+1}', (center[0]+10, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        for i, center in enumerate(blue_centers):
            cv2.circle(image, center, 5, (255, 0, 0), -1)
            cv2.putText(image, f'B{i+1}', (center[0]+10, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        all_centers = red_centers + green_centers + blue_centers
        top_left, bottom_right = find_bounding_box(all_centers)

        if top_left and bottom_right:
            print(f'色块矩阵左上角坐标: {top_left}')
            print(f'色块矩阵右下角坐标: {bottom_right}')

            grid_rows = 6
            grid_cols = 3
            grid_width = (bottom_right[0] - top_left[0]) // grid_cols
            grid_height = (bottom_right[1] - top_left[1]) // grid_rows

            color_matrix = np.zeros((grid_rows, grid_cols), dtype=int)

            for center in red_centers:
                row, col = get_grid_index(center[0], center[1], grid_width, grid_height, top_left)
                if 0 <= row < grid_rows and 0 <= col < grid_cols:
                    color_matrix[row, col] = 1

            for center in green_centers:
                row, col = get_grid_index(center[0], center[1], grid_width, grid_height, top_left)
                if 0 <= row < grid_rows and 0 <= col < grid_cols:
                    color_matrix[row, col] = 2

            for center in blue_centers:
                row, col = get_grid_index(center[0], center[1], grid_width, grid_height, top_left)
                if 0 <= row < grid_rows and 0 <= col < grid_cols:
                    color_matrix[row, col] = 3

            for row in range(grid_rows):
                for col in range(grid_cols):
                    if color_matrix[row, col] == 0:
                        grid_center = (top_left[0] + col * grid_width + grid_width // 2, top_left[1] + row * grid_height + grid_height // 2)
                        color_matrix[row, col] = find_nearest_color(grid_center, red_centers, green_centers, blue_centers)

            print('色块颜色矩阵:')
            print(color_matrix)

            # 将颜色矩阵转换为字符串并写入文件
            col1, col2, col3 = convert_color_matrix_to_string(color_matrix)
            with open(output_file, "w") as file:
                file.write(col1 + "\n" + col2 + "\n" + col3 + "\n")

            # 绘制矩阵的格子
            for row in range(grid_rows + 1):
                start_point = (top_left[0], top_left[1] + row * grid_height)
                end_point = (bottom_right[0], top_left[1] + row * grid_height)
                cv2.line(image, start_point, end_point, (0, 0, 0), 2)

            for col in range(grid_cols + 1):
                start_point = (top_left[0] + col * grid_width, top_left[1])
                end_point = (top_left[0] + col * grid_width, bottom_right[1])
                cv2.line(image, start_point, end_point, (0, 0, 0), 2)

        cv2.imshow('Image with Grid', image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    elif args.mode == 3:
        print_color_ranges()

if __name__ == "__main__":
    main()
