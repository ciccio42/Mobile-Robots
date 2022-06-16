from math import sqrt
import sys, os
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

MAP_RESOLUTION = 0.05
plot = True

# get the folder path 
dir_path = os.path.dirname(os.path.abspath(__file__))
conf_dir_path = os.path.join(dir_path, "../config")
map_dir_path = os.path.join(dir_path, "../maps")
rho_thrs = 15e-2 # 15 cm
alpha_thrs = 10e-2 # 0.1

def convert_cartesian_to_polar(p1, p2):
    # compute the line parameters a,b,c given the points p1 and p2
    # general line equation ax + by + c = 0
    # a = - (p_y2 - p_y1)
    # b = (p_x2 - p_x1)
    # c = -(p_x2*p_y1 - p_x1*p_y2)
    a = -(p2[1] - p1[1]) 
    b = (p2[0] - p1[0])
    c = -((p2[0]*p1[1])-(p1[0]*p2[1]))

    # compute the point distance
    r = abs(c) / sqrt(pow(a,2)+pow(b,2))
    r = r.real
    # compute theta
    # 1. compute the intersection point
    intersect_x = round(-(a*c)/(a**2+b**2),2)
    intersect_y = round(-(b*c)/(a**2+b**2),2)
    
    slope = - a/b
    intercept = - c/b
    print(f"Slope {slope} - Intercept {intercept}")
    print(f"Intersect {intersect_x},{intersect_y}")
    
    # check the sign
    # first quadrant
    #theta = 0.0
    #if (intersect_x >= 0.0 and intersect_y >= 0) or (intersect_x <= 0.0 and intersect_y >= 0.0):
    #    theta = np.arctan2(intersect_y, intersect_x)
    #elif ((intersect_x >= 0.0 and intersect_y <= 0) or (intersect_x <= 0.0 and intersect_y <= 0.0)):
    #    theta = 2*np.pi + np.arctan2(intersect_y, intersect_x)
    theta = np.arctan2(intersect_y, intersect_x)
    return r, theta

def filter_lines(polar_coordinates: np.array):
    # remove eventual
    polar_coordinates = polar_coordinates[~np.isnan(polar_coordinates).any(axis=1), :]
    # order coordinates by rows
    polar_coordinates.sort(axis=0)
    print(f"Ordered polar coordinates {polar_coordinates}")
    filtered_lines = []
    print("Filtering lines.....")
    similar_flag_indx = []
    for indx, line in enumerate(polar_coordinates):
        for remaining_line in polar_coordinates[indx+1:, :]:
            # chech over rho and alpha
            if (abs(line[0]-remaining_line[0]) <= rho_thrs) and (abs(line[1]-remaining_line[1]) <= alpha_thrs):
                print(f"Similar lines\n{line}-{remaining_line}\n")
                similar_flag_indx.append(indx)
                break
    print(len(similar_flag_indx))
    return np.delete(polar_coordinates, similar_flag_indx, 0)


def main(default_file = None):
    
    # Loads an image
    src = cv.imread(default_file, cv.IMREAD_GRAYSCALE)
    
    src_numpy = np.asarray(src)
    cv.imshow("Original map image", src)
    cv.waitKey(0)
    print(f"Image size {src_numpy.shape}")

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
            #print(f"{i}-th line {l}")
            cv.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv.LINE_AA)
    
    if plot:
        cv.imshow("Source", src_gaussian)
        cv.imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst)
        cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
        cv.waitKey(0)

    # convert points from pixel to continuos space
    # each row is a line, [x1, y1][x2,y2]  
    lines_continous_wrt_image_frame = np.zeros((len(linesP), 4))
    for i in range(0, len(linesP)):
        x1 = linesP[i][0][0] * MAP_RESOLUTION
        y1 = linesP[i][0][1] * MAP_RESOLUTION
        x2 = linesP[i][0][2] * MAP_RESOLUTION
        y2 = linesP[i][0][3] * MAP_RESOLUTION
        lines_continous_wrt_image_frame[i][0] = x1
        lines_continous_wrt_image_frame[i][1] = y1
        lines_continous_wrt_image_frame[i][2] = x2
        lines_continous_wrt_image_frame[i][3] = y2

    # convert from image frame to world frame 
    # origin frame: bottom-left corner (x-right, y-up)
    A_world_image = np.zeros((3,3))
    A_world_image[0][0] = 1
    A_world_image[1][1] = -1
    A_world_image[2][2] = 1
    A_world_image[1][2] = src_numpy.shape[0] * MAP_RESOLUTION

    lines_continous_wrt_world_frame = np.zeros((len(linesP), 4))
    for i in range(0, len(linesP)):
        p1_img_frame = np.array([[lines_continous_wrt_image_frame[i][0], lines_continous_wrt_image_frame[i][1], 1]]).transpose()
        p2_img_frame = np.array([[lines_continous_wrt_image_frame[i][2], lines_continous_wrt_image_frame[i][3], 1]]).transpose()
        p1_world_frame = np.matmul(A_world_image, p1_img_frame)
        p2_world_frame = np.matmul(A_world_image, p2_img_frame)       
        #if (p1_world_frame[0][0] > 10 and p1_world_frame[0][0] < 20) and (p1_world_frame[1][0] > 9 and p1_world_frame[1][0] < 16):  
        lines_continous_wrt_world_frame[i][0] = p1_world_frame[0][0]
        lines_continous_wrt_world_frame[i][1] = p1_world_frame[1][0]
        lines_continous_wrt_world_frame[i][2] = p2_world_frame[0][0]
        lines_continous_wrt_world_frame[i][3] = p2_world_frame[1][0]
    
    polar_coordiantes = np.zeros((len(linesP), 2))
    
    if plot:
        plt.scatter(np.concatenate((lines_continous_wrt_world_frame[:,0],lines_continous_wrt_world_frame[:,2]), axis=None), 
                    np.concatenate((lines_continous_wrt_world_frame[:,1],lines_continous_wrt_world_frame[:,3]), axis=None), 
                    marker="o", s=1, color="black")
        plt.show()
    
    # for each obained line, convert cartesian to polar
    for i in range(0, len(linesP)):
        print(f"P1 {lines_continous_wrt_world_frame[i,:2]} / P2 {lines_continous_wrt_world_frame[i,2:4]}")
        rho, theta = convert_cartesian_to_polar(p1 = lines_continous_wrt_world_frame[i,:2], p2 = lines_continous_wrt_world_frame[i,2:4])
        polar_coordiantes[i][0] = rho
        polar_coordiantes[i][1] = theta
        print(f"Rho {rho} - Theta {theta}\n")
    
    polar_coordiantes = filter_lines(polar_coordiantes)
    print(f"Filtered lines {polar_coordiantes}")
    print(f"Remaining lines {np.size(polar_coordiantes)}")
    np.save(os.path.join(conf_dir_path , "map_lines.npy"), polar_coordiantes)
    
    return 0
    
if __name__ == "__main__":
    default_file = os.path.join(map_dir_path, "map6.pgm")
    main(default_file=default_file)