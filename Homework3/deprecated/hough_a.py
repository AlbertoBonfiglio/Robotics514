import matplotlib.pyplot as plt
import cv2
import math

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
def test():
    ############ make houghspace array ############
    houghspace = []
    c = 0
    height = 400
    while c <= height:
        houghspace.append([])
        cc = 0
        while cc <= 180:
            houghspace[c].append(0)
            cc += 1
        c+=1

    ############ do transform ############
    degree_tick = 1 #by how many degrees to check
    total_votes = 1 #votes counter
    highest_vote = 0 #highest vote in the array
    while total_votes < 50:
        img = cv2.imread('../images/empire.png')
        edges = cv2.Canny(img, 50, 150, apertureSize=3)
        lines = cv2.HoughLines(edges, 1, math.pi*degree_tick/180, total_votes)

        try:
            for rho, theta in lines[0]:
                a = math.cos(theta)
                b = math.sin(theta)
                x1 = int((a*rho) + 1000*(-b))
                y1 = int((b*rho) + 1000*(a))
                x2 = int((a*rho) - 1000*(-b))
                y2 = int((b*rho) - 1000*(a))
                cv2.line(img, (x1, y1), (x2, y2), (0, 200, 255), 2)

            #################add votes into the array################
            deradian = 180/math.pi #used to convert to degrees
            for rho, theta in lines[0]:
                degree = int(round(theta * deradian))
                rho_pos = int(rho - 200)
                houghspace[rho_pos][degree] += 1

        #when lines[0] has no votes, it throws an error which is caught here
        except:
            total_votes = 999 #exit loop

        highest_vote = total_votes
        total_votes += 1

        del lines
    cv2.imwrite('../images/houghlines3.jpg',img)

    ########### loop finished ###############################
    print highest_vote

    ################### plot the houghspace ###################
    maxy = 200 #used to offset the y-axis
    miny = -200 #used to offset the y-axis

    #the main graph

    fig = plt.figure(figsize=(10, 5))
    ax = fig.add_subplot(111)
    ax.set_title('Houghspace')
    plt.imshow(houghspace, cmap='gist_stern')
    ax.set_aspect('equal')
    plt.yticks([0, -miny, maxy-miny], [miny, 0, maxy])

    #the legend
    cax = fig.add_axes([0, 0.1, 0.78, 0.8])
    cax.get_xaxis().set_visible(False)
    cax.get_yaxis().set_visible(False)
    cax.patch.set_alpha(0)
    cax.set_frame_on(False)
    plt.colorbar(orientation='vertical')
    plt.show()


if __name__ == '__main__':
    test()