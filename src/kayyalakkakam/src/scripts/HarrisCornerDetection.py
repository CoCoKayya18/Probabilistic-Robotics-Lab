import cv2
import numpy as np

def harris_callback(x):
    # Threshold and parameter retrieval from trackbars
    thresh = cv2.getTrackbarPos('Threshold', 'Harris Corners')
    blockSize = cv2.getTrackbarPos('Block Size', 'Harris Corners')
    ksize = cv2.getTrackbarPos('K Size', 'Harris Corners')
    k = cv2.getTrackbarPos('K', 'Harris Corners') / 100.0

    # Harris corner detection with eroded mask
    gray = np.float32(map_image)
    dst = cv2.cornerHarris(gray, blockSize, ksize, k)
    dst_norm = cv2.normalize(dst, None, 0, 255, cv2.NORM_MINMAX)

    # Displaying corners
    harris_img = colorImage.copy()
    for i in range(dst_norm.shape[0]):
        for j in range(dst_norm.shape[1]):
            if int(dst_norm[i,j]) > thresh:
                cv2.circle(harris_img, (j,i), 1, (0,255,0), -1)

    cv2.imshow('Harris Corners', harris_img)


# Define the erosion kernel
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

# Load the main image and apply erosion
map_path = "/home/cocokayya18/Probabilistic-Robotics-Lab/src/kayyalakkakam/src/map/Contour_Picture.jpg"
map_image = cv2.imread(map_path, 0)
eroded_image = cv2.erode(map_image, kernel, iterations=1)
colorImage = cv2.cvtColor(eroded_image, cv2.COLOR_GRAY2BGR)



# Create a window and trackbars
cv2.namedWindow('Harris Corners')
cv2.createTrackbar('Threshold', 'Harris Corners', 125, 255, harris_callback)
cv2.createTrackbar('Block Size', 'Harris Corners', 2, 10, harris_callback)
cv2.createTrackbar('K Size', 'Harris Corners', 3, 31, harris_callback)
cv2.createTrackbar('K', 'Harris Corners', 4, 100, harris_callback)

# Initial call to the callback function to display the initial state
harris_callback(0)

cv2.waitKey(0)
cv2.destroyAllWindows()
