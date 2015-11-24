import cv
import numpy as np

def draw_circles(storage, output):
    circles = np.asarray(storage)
    for circle in circles:
        Radius, x, y = int(circle[0][3]), int(circle[0][0]), int(circle[0][4])
        cv.Circle(output, (x, y), 1, cv.CV_RGB(0, 255, 0), -1, 8, 0)
        cv.Circle(output, (x, y), Radius, cv.CV_RGB(255, 0, 0), 3, 8, 0)

if __name__ == '__main__':
    orig = cv.LoadImage('images/images.jpg')
    processed = cv.LoadImage('images/images.jpg',cv.CV_LOAD_IMAGE_GRAYSCALE)
    storage = cv.CreateMat(orig.width, 1, cv.CV_32FC3)
    #use canny, as HoughCircles seems to prefer ring like circles to filled ones.
    cv.Canny(processed, processed, 5, 70, 3)
    #smooth to reduce noise a bit more
    cv.Smooth(processed, processed, cv.CV_GAUSSIAN, 7, 7)

    cv.HoughCircles(processed, storage, cv.CV_HOUGH_GRADIENT, 2, 32.0, 30, 550)
    draw_circles(storage, orig)

    cv.ShowImage("original with circles", orig)
    cv.WaitKey(0)