import math
from math import sqrt
import sys, os
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

import rospy, rospkg
from visualization_msgs.msg import MarkerArray, Marker 
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

MAP_RESOLUTION = 0.05
plot = True

# get the folder path 
"""dir_path = os.path.dirname(os.path.abspath(__file__))
conf_dir_path = os.path.join(dir_path, "../config")
map_dir_path = os.path.join(dir_path, "../maps")"""
rospack = rospkg.RosPack()
pkg_path = rospack.get_path('exam')
map_dir_path= os.path.join(pkg_path, "maps")
conf_dir_path = os.path.join(pkg_path, "config")
rho_thrs = 10e-2 # 10 cm
alpha_thrs = 10e-2 # 0.1

def convert_cartesian_to_polar(p1, p2):
    """Remove similar lines

    Parameters
    ----------
        p1
        p2
    Returns
        ray
        theta
    -------
    """
    # compute the line parameters a,b,c given the points p1 and p2
    # general line equation ax + by + c = 0
    # a = - (p_y2 - p_y1)
    # b = (p_x2 - p_x1)
    # c = -(p_x2*p_y1 - p_x1*p_y2)
    rospy.logdebug(f"P1: {p1} - P2: {p2}")
    a = -(p2[1] - p1[1]) 
    b = (p2[0] - p1[0])
    c = -((p2[0]*p1[1])-(p1[0]*p2[1]))

    # compute the point distance
    r = abs(c) / sqrt(pow(a,2)+pow(b,2))
    # compute theta
    # 1. compute the intersection point
    intersect_x = -(a*c)/(a**2+b**2)
    intersect_y = -(b*c)/(a**2+b**2)
    

    """
    slope = - a/b
    intercept = - c/b
    rospy.logdebug(f"Slope {slope} - Intercept {intercept}")
    rospy.logdebug(f"Intercept x: {intersect_x}, y:{intersect_y}")
    """
    theta = np.arctan2(intersect_y, intersect_x)
    return r, theta

def filter_lines(polar_coordinates: np.array) -> np.array:
    """Remove similar lines

    Parameters
    ----------
        polar_coordinates: numpy.array
            Computed lines in polar coordinate, [rho, theta]
    Returns
        filtered_lines: numpy.array
            Filtered set of lines
    -------
    """
    # remove eventual
    polar_coordinates = polar_coordinates[~np.isnan(polar_coordinates).any(axis=1), :]
    # order coordinates by rows, in descending order
    polar_coordinates = polar_coordinates[polar_coordinates[:, 0].argsort()]
    rospy.logdebug(f"Ordered polar coordinates\n{polar_coordinates}")
    rospy.logdebug("Filtering lines.....")
    similar_flag_indx = []

    # for each line, find the similar ones
    for indx, line in enumerate(polar_coordinates):
        for remaining_line in polar_coordinates[indx+1:, :]:
            # chech over rho and alpha
            if (abs(line[0]-remaining_line[0]) <= rho_thrs) and (abs(line[1]-remaining_line[1]) <= alpha_thrs):
                rospy.logdebug(f"Similar lines\n{line}-{remaining_line}\n")
                similar_flag_indx.append(indx)
                break
    
    rospy.logdebug(len(similar_flag_indx))
    return np.delete(polar_coordinates, similar_flag_indx, 0)

def publish_points(x: np.array, y: np.array, reference_frame: str):
    """Publish the points (x,y) as Marker, with respect to reference_frame

    Parameters
    ----------
        x: np.array
            Numpy array of x-coordinate
        y: np.array
            Numpy array of y-coordinate
        reference_frame: str
            Name of the reference frame with respect to which plot the points
    Returns
    -------
    """
    # create Marker Array Publisher
    marker = Marker()
    # marker header 
    marker.header.frame_id = reference_frame
    # marker field
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    # pose
    marker.pose.orientation.w = 1

    marker.color.r, marker.color.g, marker.color.b = (0, 1, 0)
    marker.color.a = 1
    marker.scale.x, marker.scale.y, marker.scale.z = (0.2,0.2,0.2)
        
    # for each point create a marker
    for point in zip(x, y):
        marker.points.append(Point(point[0], point[1], 0.5))
        marker.colors.append(ColorRGBA(0,0,1,1))
    
    rate = rospy.Rate(2)
    for i in range(10):
        extracted_points_pub.publish(marker)
        rate.sleep()

def publish_lines(lines, reference_frame: str):
    """Publish the extracted lines as MarkerArray, with respect to reference_frame

    Parameters
    ----------
        lines: list
            list of extracted lines in polar coordinates
        reference_frame: str
            Name of the reference frame with respect to which plot the lines
    Returns
    -------
    """
    # create Marker Array Publisher
    marker_array = MarkerArray()
    marker_array.markers = []

    for line in lines:
        rospy.logdebug(f"Line {line}")
        # marker header 
        marker = Marker()
        marker.header.frame_id = reference_frame

        # marker field
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        # pose
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1

        marker.color.r, marker.color.g, marker.color.b = (0, 1, 0)
        marker.color.a = 0.5
        marker.scale.x, marker.scale.y, marker.scale.z = (0.06, 0.06, 0.06)
        
        # compute line points
        a = math.cos(line[1])
        b = math.sin(line[1])
        x0 = a * line[0]
        y0 = b * line[0]
        pt1 = ((x0 + 100*(-b)),(y0 + 100*(a)))
        pt2 = ((x0 - 100*(-b)),(y0 - 100*(a)))
        marker.points.append(Point(pt1[0], pt1[1], 0.5))
        marker.points.append(Point(pt2[0], pt2[1], 0.5))
        marker.colors.append(ColorRGBA(1,0,0,1))
        marker.colors.append(ColorRGBA(1,0,0,1))
        marker_array.markers.append(marker)

    # set markers id
    id = 0
    for m in marker_array.markers:
        m.id = id
        id += 1
    rate = rospy.Rate(2)
    for i in range(10):
        map_line_pub.publish(marker_array)
        rate.sleep()
        
def main(default_file = None):
    
    # Loads an image
    src = cv.imread(default_file, cv.IMREAD_GRAYSCALE)
    
    src_numpy = np.asarray(src)
    if plot:
        cv.imshow("Original map image", src)
        cv.waitKey(0)
        rospy.logdebug(f"Image size {src_numpy.shape}")

    # Check if image is loaded fine
    if src is None:
        rospy.logdebug ('Error opening image!')
        rospy.logdebug ('Usage: hough_lines.py [image_name -- default ' + default_file + '] \n')
        return -1

    src_gaussian = cv.GaussianBlur(src, (5, 5), 1)
    dst = cv.Canny(src_gaussian, threshold1=100, threshold2=200, apertureSize=3, L2gradient=True)
    # Copy edges to the images that will display the results in BGR
    cdst = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
    cdstP = np.copy(cdst)

    linesP = cv.HoughLinesP(image=dst,rho=0.5, theta=np.pi/360, threshold=10, lines=np.array([]), minLineLength=10, maxLineGap=10)
    rospy.logdebug(f"Number of lines {len(linesP)}")
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            #rospy.logdebug(f"{i}-th line {l}")
            cv.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv.LINE_AA)
    
    if plot:
        cv.imshow("Source", src_gaussian)
        cv.imwrite(os.path.join(map_dir_path, "gaussian_map.jpg"), src_gaussian)
        cv.imshow("Canny edge detection", cdst)
        cv.imwrite(os.path.join(map_dir_path, "canny_edge.jpg"), cdst)
        cv.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
        cv.imwrite(os.path.join(map_dir_path, "extracted_lines.jpg"), cdstP)
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
    
    publish_points(x = np.concatenate((lines_continous_wrt_world_frame[:10,0],lines_continous_wrt_world_frame[:10,2]), axis=None),
                   y = np.concatenate((lines_continous_wrt_world_frame[:10,1],lines_continous_wrt_world_frame[:10,3]), axis=None),
                   reference_frame = "world")

    # for each obained line, convert cartesian to polar
    for i in range(0, len(linesP)):
        rospy.logdebug(f"P1 {lines_continous_wrt_world_frame[i,:2]} / P2 {lines_continous_wrt_world_frame[i,2:4]}")
        rho, theta = convert_cartesian_to_polar(p1 = lines_continous_wrt_world_frame[i,:2], p2 = lines_continous_wrt_world_frame[i,2:4])
        polar_coordiantes[i][0] = rho
        polar_coordiantes[i][1] = theta
        rospy.logdebug(f"Rho {rho} - Theta {theta}\n")

    rospy.loginfo(f"Number of lines: {len(linesP)}")
    polar_coordiantes = filter_lines(polar_coordiantes)
    publish_lines(polar_coordiantes[:10], reference_frame = "world")
    rospy.logdebug(f"Filtered lines\n{polar_coordiantes}")
    rospy.loginfo(f"Remaining lines {np.size(polar_coordiantes)}")
    np.save(os.path.join(conf_dir_path , "map_lines.npy"), polar_coordiantes)
    
    return 0
    
if __name__ == "__main__":

    rospy.init_node("line_extractor_from_map")
    extracted_points_pub = rospy.Publisher("extracted_points", Marker, queue_size=100)
    map_line_pub = rospy.Publisher("map_lines", MarkerArray, queue_size=100)

    default_file = os.path.join(map_dir_path, "map6.pgm")
    rospy.logdebug(default_file)
    main(default_file=default_file)