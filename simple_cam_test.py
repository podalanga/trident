import cv2
import sys

def main():
    # RealSense RGB is often at index 0, 2, or 4 on Linux depending on what other devices are plugged in.
    # We will try a few common indices.
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

    print(f"Starting video feed from camera {working_index}. Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Can't receive frame.")
            break

        cv2.imshow(f'Camera Feed (Index {working_index})', frame)
        
        # Press 'q' to exit
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
