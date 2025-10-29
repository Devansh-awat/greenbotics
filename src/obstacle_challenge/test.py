import cv2
import numpy as np
from src.sensors import camera
from src.obstacle_challenge.config import LOWER_GREEN, UPPER_GREEN, LOWER_RED_1, UPPER_RED_1, LOWER_RED_2, UPPER_RED_2

# Empty callback function required for trackbar creation
def empty(a):
    pass

# Initialize the camera using your custom module
camera.initialize()

# --- Create Trackbar Windows ---

# Window for GREEN trackbars
cv2.namedWindow("Green Trackbars")
cv2.resizeWindow("Green Trackbars", 640, 240)
cv2.createTrackbar("HUE MIN", "Green Trackbars", LOWER_GREEN[0], 179, empty)
cv2.createTrackbar("HUE MAX", "Green Trackbars", UPPER_GREEN[0], 179, empty)
cv2.createTrackbar("SAT MIN", "Green Trackbars", LOWER_GREEN[1], 255, empty)
cv2.createTrackbar("SAT MAX", "Green Trackbars", UPPER_GREEN[1], 255, empty)
cv2.createTrackbar("VAL MIN", "Green Trackbars", LOWER_GREEN[2], 255, empty)
cv2.createTrackbar("VAL MAX", "Green Trackbars", UPPER_GREEN[2], 255, empty)

# Window for RED trackbars (for two ranges)
cv2.namedWindow("Red Trackbars")
cv2.resizeWindow("Red Trackbars", 640, 480)
# Trackbars for the first red range
cv2.createTrackbar("HUE MIN 1", "Red Trackbars", LOWER_RED_1[0], 179, empty)
cv2.createTrackbar("HUE MAX 1", "Red Trackbars", UPPER_RED_1[0], 179, empty)
cv2.createTrackbar("SAT MIN 1", "Red Trackbars", LOWER_RED_1[1], 255, empty)
cv2.createTrackbar("SAT MAX 1", "Red Trackbars", UPPER_RED_1[1], 255, empty)
cv2.createTrackbar("VAL MIN 1", "Red Trackbars", LOWER_RED_1[2], 255, empty)
cv2.createTrackbar("VAL MAX 1", "Red Trackbars", UPPER_RED_1[2], 255, empty)
# Trackbars for the second red range
cv2.createTrackbar("HUE MIN 2", "Red Trackbars", LOWER_RED_2[0], 179, empty)
cv2.createTrackbar("HUE MAX 2", "Red Trackbars", UPPER_RED_2[0], 179, empty)
cv2.createTrackbar("SAT MIN 2", "Red Trackbars", LOWER_RED_2[1], 255, empty)
cv2.createTrackbar("SAT MAX 2", "Red Trackbars", UPPER_RED_2[1], 255, empty)
cv2.createTrackbar("VAL MIN 2", "Red Trackbars", LOWER_RED_2[2], 255, empty)
cv2.createTrackbar("VAL MAX 2", "Red Trackbars", UPPER_RED_2[2], 255, empty)


while True:
    # Capture a frame from the camera and convert it to HSV color space
    image = camera.capture_frame()
    imgHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # --- Green Color Detection ---
    # Read current positions of green trackbars
    g_h_min = cv2.getTrackbarPos("HUE MIN", "Green Trackbars")
    g_h_max = cv2.getTrackbarPos("HUE MAX", "Green Trackbars")
    g_s_min = cv2.getTrackbarPos("SAT MIN", "Green Trackbars")
    g_s_max = cv2.getTrackbarPos("SAT MAX", "Green Trackbars")
    g_v_min = cv2.getTrackbarPos("VAL MIN", "Green Trackbars")
    g_v_max = cv2.getTrackbarPos("VAL MAX", "Green Trackbars")

    # Define lower and upper HSV bounds for green
    lower_green = np.array([g_h_min, g_s_min, g_v_min])
    upper_green = np.array([g_h_max, g_s_max, g_v_max])
    
    # Create the green mask and apply it to the original image
    green_mask = cv2.inRange(imgHSV, lower_green, upper_green)
    green_result = cv2.bitwise_and(image, image, mask=green_mask)

    # --- Red Color Detection ---
    # Read current positions of the first set of red trackbars
    r1_h_min = cv2.getTrackbarPos("HUE MIN 1", "Red Trackbars")
    r1_h_max = cv2.getTrackbarPos("HUE MAX 1", "Red Trackbars")
    r1_s_min = cv2.getTrackbarPos("SAT MIN 1", "Red Trackbars")
    r1_s_max = cv2.getTrackbarPos("SAT MAX 1", "Red Trackbars")
    r1_v_min = cv2.getTrackbarPos("VAL MIN 1", "Red Trackbars")
    r1_v_max = cv2.getTrackbarPos("VAL MAX 1", "Red Trackbars")
    
    # Read current positions of the second set of red trackbars
    r2_h_min = cv2.getTrackbarPos("HUE MIN 2", "Red Trackbars")
    r2_h_max = cv2.getTrackbarPos("HUE MAX 2", "Red Trackbars")
    r2_s_min = cv2.getTrackbarPos("SAT MIN 2", "Red Trackbars")
    r2_s_max = cv2.getTrackbarPos("SAT MAX 2", "Red Trackbars")
    r2_v_min = cv2.getTrackbarPos("VAL MIN 2", "Red Trackbars")
    r2_v_max = cv2.getTrackbarPos("VAL MAX 2", "Red Trackbars")

    # Define lower and upper HSV bounds for both red ranges
    lower_red_1 = np.array([r1_h_min, r1_s_min, r1_v_min])
    upper_red_1 = np.array([r1_h_max, r1_s_max, r1_v_max])
    lower_red_2 = np.array([r2_h_min, r2_s_min, r2_v_min])
    upper_red_2 = np.array([r2_h_max, r2_s_max, r2_v_max])

    # Create masks for each red range and combine them
    red_mask_1 = cv2.inRange(imgHSV, lower_red_1, upper_red_1)
    red_mask_2 = cv2.inRange(imgHSV, lower_red_2, upper_red_2)
    red_mask = cv2.add(red_mask_1, red_mask_2)
    
    # Apply the combined red mask to the original image
    red_result = cv2.bitwise_and(image, image, mask=red_mask)

    # --- Display Final Output ---
    # Convert single-channel masks to 3-channel BGR to stack them with color images
    green_mask_bgr = cv2.cvtColor(green_mask, cv2.COLOR_GRAY2BGR)
    red_mask_bgr = cv2.cvtColor(red_mask, cv2.COLOR_GRAY2BGR)

    # Create the layout for the green side (masked image on top, result on bottom)
    left_side = np.vstack([green_mask_bgr, green_result])

    # Create the layout for the red side (masked image on top, result on bottom)
    right_side = np.vstack([red_mask_bgr, red_result])

    # Combine both sides horizontally into a single window
    combined_view = np.hstack([left_side, right_side])
    
    cv2.imshow("Color Tuning - Green (Left) & Red (Right)", combined_view)

    # Exit loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up resources
cv2.destroyAllWindows()
camera.cleanup()
