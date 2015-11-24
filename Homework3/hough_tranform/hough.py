import cv2
import numpy as np
from matplotlib import pyplot as plt
from skimage.transform import (hough_line, hough_line_peaks,
                               probabilistic_hough_line)
from skimage.feature import canny
from skimage import data


def test_p():
    try:
        img = cv2.imread('../images/chk.png')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize = 3)

        minLineLength = 100
        maxLineGap = 1

        lines = cv2.HoughLinesP(edges, 1 , np.pi/180, 100, minLineLength, maxLineGap)

        for x1,y1,x2,y2 in lines[0]:
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

        cv2.imwrite('../images/houghlines5.jpg', img)

    except Exception as ex:
        print(ex.message)


def test():
    try:
        orig = cv2.imread('../images/empire.png')
        img = cv2.imread('../images/empire.png')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize = 3)

        lines = cv2.HoughLines(edges, 1, np.pi/180, 50)

        for rho, theta in lines[0]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))

            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

        plt.subplot(221), plt.imshow(orig, cmap = 'gray')
        plt.title('Original Image'), plt.xticks([]), plt.yticks([])

        plt.subplot(222), plt.imshow(gray, cmap = 'gray')
        plt.title('Gray Image'), plt.xticks([]), plt.yticks([])

        plt.subplot(223), plt.imshow(edges, cmap = 'gray')
        plt.title('Edge Image'), plt.xticks([]), plt.yticks([])

        plt.subplot(224), plt.imshow(img, cmap = 'gray')
        plt.title('Lines Image'), plt.xticks([]), plt.yticks([])


        plt.show()

        cv2.imwrite('../images/houghlines3.jpg',img)
    except Exception as ex:
        print(ex.message)

def test_circles():

    bgr_img = cv2.imread('b.jpg') # read as it is

    if bgr_img.shape[-1] == 3:           # color image
        b,g,r = cv2.split(bgr_img)       # get b,g,r
        rgb_img = cv2.merge([r,g,b])     # switch it to rgb
        gray_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
    else:
        gray_img = bgr_img

    img = cv2.medianBlur(gray_img, 5)
    cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

    circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,1,20,
                                param1=50,param2=30,minRadius=0,maxRadius=0)

    circles = np.uint16(np.around(circles))

    for i in circles[0,:]:
        # draw the outer circle
        cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
        cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

    plt.subplot(121),plt.imshow(rgb_img)
    plt.title('Input Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(cimg)
    plt.title('Hough Transform'), plt.xticks([]), plt.yticks([])
    plt.show()

def skimage_transform():
    image = np.zeros((100, 100))

    # Classic straight-line Hough transform

    idx = np.arange(25, 75)
    image[idx[::-1], idx] = 255
    image[idx, idx] = 255

    h, theta, d = hough_line(image)

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(8,4))

    ax1.imshow(image, cmap=plt.cm.gray)
    ax1.set_title('Input image')
    ax1.set_axis_off()

    ax2.imshow(np.log(1 + h),
               extent=[np.rad2deg(theta[-1]), np.rad2deg(theta[0]),
               d[-1], d[0]],
               cmap=plt.cm.gray,
               aspect=1/1.5)
    ax2.set_title('Hough transform')
    ax2.set_xlabel('Angles (degrees)')
    ax2.set_ylabel('Distance (pixels)')
    ax2.axis('image')

    ax3.imshow(image, cmap=plt.cm.gray)
    rows, cols = image.shape
    for _, angle, dist in zip(*hough_line_peaks(h, theta, d)):
        y0 = (dist - 0 * np.cos(angle)) / np.sin(angle)
        y1 = (dist - cols * np.cos(angle)) / np.sin(angle)
        ax3.plot((0, cols), (y0, y1), '-r')
    ax3.axis((0, cols, rows, 0))
    ax3.set_title('Detected lines')
    ax3.set_axis_off()

    # Line finding, using the Probabilistic Hough Transform

    image = data.camera()
    edges = canny(image, 2, 1, 25)
    lines = probabilistic_hough_line(edges, threshold=10, line_length=5,
                                     line_gap=3)

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(8,4), sharex=True, sharey=True)

    ax1.imshow(image, cmap=plt.cm.gray)
    ax1.set_title('Input image')
    ax1.set_axis_off()
    ax1.set_adjustable('box-forced')

    ax2.imshow(edges, cmap=plt.cm.gray)
    ax2.set_title('Canny edges')
    ax2.set_axis_off()
    ax2.set_adjustable('box-forced')

    ax3.imshow(edges * 0)

    for line in lines:
        p0, p1 = line
        ax3.plot((p0[0], p1[0]), (p0[1], p1[1]))

    ax3.set_title('Probabilistic Hough')
    ax3.set_axis_off()
    ax3.set_adjustable('box-forced')
    plt.show()

if __name__ == '__main__':
    test()
    test_p()