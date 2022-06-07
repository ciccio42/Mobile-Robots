import sys
import math
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

MAP_RESOLUTION = 0.05

def main(default_file = None):
    
    # Loads an image
    src = cv.imread(default_file, cv.IMREAD_GRAYSCALE)
    
    src_numpy = np.asarray(src)
    cv.imshow("Original map image", src)
    cv.waitKey(0)
    print(f"Image size {src_numpy.shape}")

    """ 
    # convert pixel into continues value
    obstacle_distance_list = []

    for h in range(src_numpy.shape[0]):
        for w in range(src_numpy.shape[1]):
            if src_numpy[h][w] == 0:
                obstacle_distance_list.append([round(w*MAP_RESOLUTION, 2), round(h*MAP_RESOLUTION, 2), 1])
                print(f"Original pixel [h][w] {h}-{w} - Continue value [x][y] {obstacle_distance_list[-1][0]}-{obstacle_distance_list[-1][1]}") 
    

    # rotate to plot correctly
    x_plot_frame_np = np.zeros(len(obstacle_distance_list))
    y_plot_frame_np = np.zeros(len(obstacle_distance_list))

    A_plot_frame = np.zeros((3,3))
    A_plot_frame[0][0] = 1
    A_plot_frame[1][1] = -1
    A_plot_frame[2][2] = 1
    A_plot_frame[1][2] = src_numpy.shape[0] * MAP_RESOLUTION
    print(f"Homogeneous plot_frame to image_frame\n{A_plot_frame}")
    for indx, point in enumerate(obstacle_distance_list):
        point_image_frame = np.array([obstacle_distance_list[indx]]).transpose()
        point_plot_frame = np.matmul(A_plot_frame, point_image_frame)
        print(point_plot_frame)
        x_plot_frame_np[indx] = point_plot_frame[0][0]
        y_plot_frame_np[indx] = point_plot_frame[1][0]
        
    
    plt.scatter(x_plot_frame_np, y_plot_frame_np, marker="o", s=0.01, color="black")
    plt.show()
    """
    
    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        print ('Usage: hough_lines.py [image_name -- default ' + default_file + '] \n')
        return -1
    
    src_gaussian = cv.GaussianBlur(src, (5, 5), 1)
    dst = cv.Canny(src_gaussian, threshold1=100, threshold2=200, apertureSize=3, L2gradient=True)
    # Copy edges to the images that will display the results in BGR
    cdst = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
    cdstP = np.copy(cdst)
        
    linesP = cv.HoughLinesP(image=dst,rho=0.5, theta=np.pi/360, threshold=10, lines=np.array([]), minLineLength=10, maxLineGap=10)
    print(f"Number of lines {len(linesP)}")
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            print(f"{i}-th line {l}")
            cv.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv.LINE_AA)
    
    # convert points from pixel to continuos space
    # each row  
    lines_continous = np.zeros((len(linesP), 4))
    for i in range(0, len(linesP)):
        pass

    cv.imshow("Source", src_gaussian)
    cv.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
    cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
    
    cv.waitKey()
    return 0
    
if __name__ == "__main__":
    default_file = '/home/ciccio/Desktop/catkin_ws/src/exam/maps/map6.pgm'
    main(default_file=default_file)