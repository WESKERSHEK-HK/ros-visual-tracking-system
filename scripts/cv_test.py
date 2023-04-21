#!/usr/bin/env python

import cv2
import numpy as np

def main():
    # Create a dummy image (a 400x400 black image)
    img = np.zeros((400, 400, 3), dtype=np.uint8)

    while True:
        # Display the image in a window named "Test"
        cv2.imshow("Test", img)

        # Wait for a key press and exit the loop if 'q' is pressed
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    # Destroy all OpenCV windows
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
