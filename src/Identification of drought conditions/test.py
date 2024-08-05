import cv2
import numpy as np

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

# 图片路径
image_path = "D:\\Code\\test.png"
# 读取图片
image = cv2.imread(image_path)
if image is None:
    print('无法读取图片，请检查文件路径和文件格式。')
else:
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    red_lower = np.array([170, 200, 220])
    red_upper = np.array([190, 240, 240])
    green_lower = np.array([40, 190, 150])
    green_upper = np.array([70, 220, 190])
    blue_lower = np.array([100, 160, 200])
    blue_upper = np.array([120, 200, 255])

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
