import cv2
import numpy as np
import colorsys

def hex_to_hsv(hex_color):
    """Converts a hex color code to OpenCV-compatible HSV values."""
    hex_color = hex_color.lstrip('#')
    
    # Convert hex to RGB (0-1 range)
    r = int(hex_color[0:2], 16) / 255.0
    g = int(hex_color[2:4], 16) / 255.0
    b = int(hex_color[4:6], 16) / 255.0

    # Convert RGB to HSV (0-1 range)
    h, s, v = colorsys.rgb_to_hsv(r, g, b)

    # Scale to OpenCV HSV format
    h = h * 179  # OpenCV uses 0-179 for Hue
    s = s * 255  # Saturation: 0-255
    v = v * 255  # Value: 0-255

    return np.array([h, s, v], dtype=np.uint8)

def adjust_hsv_bound(hsv_color, h_shift=0, s_shift=0, v_shift=0):
    """Adjusts HSV values with boundary checks."""
    h, s, v = hsv_color
    h = np.clip(h + h_shift, 0, 179)  # Hue: 0-179
    s = np.clip(s + s_shift, 0, 255)  # Saturation: 0-255
    v = np.clip(v + v_shift, 0, 255)  # Value: 0-255
    return np.array([h, s, v], dtype=np.uint8)

hex_color1 = "#CB9EBB"  # Light pinkish
hex_color2 = "#A4344F"  # Darker reddish

# Convert hex to HSV
lower_red1 = hex_to_hsv(hex_color1)
upper_red1 = hex_to_hsv(hex_color2)

# Ensure correct lower and upper bounds
lower_bound = np.minimum(lower_red1, upper_red1)
upper_bound = np.maximum(lower_red1, upper_red1)

# Expand the color range slightly
lower_bound_adjusted = adjust_hsv_bound(lower_bound, h_shift=-30, s_shift=-30, v_shift=-60)
upper_bound_adjusted = adjust_hsv_bound(upper_bound, h_shift=30, s_shift=30, v_shift=60)

# Load the image
color_image = cv2.imread('/home/marcus/orchard_template_ws/src/apple-harvest/harvest_vision/data/prosser_a/color_raw.png')
depth_image = cv2.imread('/home/marcus/orchard_template_ws/src/apple-harvest/harvest_vision/data/prosser_a/depth_to_color.png', cv2.IMREAD_UNCHANGED)

if len(depth_image.shape) == 3:
    depth_image = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY)

# --- Apply Color Mask ---
hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
color_mask = cv2.inRange(hsv_image, lower_bound_adjusted, upper_bound_adjusted)

depth_in_meters = depth_image.astype(np.float32) / 1000.0  # Convert to meters

# Initialize the combined mask to be the color mask alone
combined_mask = color_mask

# Set this variable to True or False based on your requirement
depth_threshold_bool = True  # Change to False to disable depth thresholding

if depth_threshold_bool:
    # Create the depth mask (foreground within 1m)
    foreground_mask = np.where((depth_in_meters > 0.0) & (depth_in_meters <= 0.0), 255, 0).astype(np.uint8)

    # Widen the foreground mask to the right
    kernel = np.ones((1, 100), np.uint8)  # Adjust the kernel size as needed
    foreground_mask = cv2.dilate(foreground_mask, kernel, iterations=1)

    # Combine color and depth masks
    combined_mask = cv2.bitwise_and(color_mask, foreground_mask)

# Invert the combined mask for background masking
background_mask = cv2.bitwise_not(combined_mask)

# Apply mask: keep red regions within 1m (if depth thresholding is enabled), gray out the rest
gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
gray_bgr = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
result = np.where(combined_mask[:, :, None] == 255, color_image, gray_bgr)

# # --- Save and Display the Result ---
cv2.imwrite('greyed_apples.png', result)
cv2.imshow('Red in Foreground, Background Grayed Out', result)
cv2.waitKey(0)
cv2.destroyAllWindows()