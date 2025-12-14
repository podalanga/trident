import cv2
import sys
import time

def main():
    # RealSense RGB is often at index 0, 2, or 4 on Linux
    indices_to_try = [0, 2, 4, 6, 1, 3, 5]
    cap = None
    working_index = -1

    print("Searching for available camera...")

    for index in indices_to_try:
        print(f"Checking /dev/video{index}...")
        temp_cap = cv2.VideoCapture(index)
        if temp_cap.isOpened():
            # Try to read a frame to make sure it's actually working
            ret, frame = temp_cap.read()
            if ret:
                print(f"Success! Camera found at index {index}")
                cap = temp_cap
                working_index = index
                break
            else:
                temp_cap.release()
        
    if cap is None or not cap.isOpened():
        print("Error: Could not open any camera. Please check connection.")
        sys.exit(1)

    print(f"Camera initialized at index {working_index}.")
    print("Capturing frames... (Headless mode - no display required)")

    # Warm up camera
    for i in range(10):
        ret, frame = cap.read()
        time.sleep(0.1)

    if ret:
        filename = "camera_test_image.jpg"
        cv2.imwrite(filename, frame)
        print(f"Successfully saved test image to: {filename}")
        print("Please check this image to verify the camera view.")
    else:
        print("Error: Failed to capture frame.")

    cap.release()

if __name__ == "__main__":
    main()
