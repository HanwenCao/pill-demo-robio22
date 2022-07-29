import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
from math import atan2, cos, sin, sqrt, pi, asin, acos
import random
from .plt_click import MouseControl


def drawAxis(img, p_, q_, colour, scale):
    p = list(p_)
    q = list(q_)
    ## [visualization1]
    angle = atan2(p[1] - q[1], p[0] - q[0])  # angle in radians
    hypotenuse = sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))
    # Here we lengthen the arrow by a factor of scale
    q[0] = p[0] - scale * hypotenuse * cos(angle)
    q[1] = p[1] - scale * hypotenuse * sin(angle)
    cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 5, cv.LINE_AA)
    # create the arrow hooks
    p[0] = q[0] + 9 * cos(angle + pi / 4)
    p[1] = q[1] + 9 * sin(angle + pi / 4)
    cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 5, cv.LINE_AA)
    p[0] = q[0] + 9 * cos(angle - pi / 4)
    p[1] = q[1] + 9 * sin(angle - pi / 4)
    cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 5, cv.LINE_AA)
    ## [visualization1]


def getOrientation(pts, img):
    ## [pca]
    # Construct a buffer used by the pca analysis
    sz = len(pts)
    data_pts = np.empty((sz, 2), dtype=np.float64)
    for i in range(data_pts.shape[0]):
        data_pts[i] = pts[i]
    # Perform PCA analysis
    mean = np.empty((0))
    mean, eigenvectors, eigenvalues = cv.PCACompute2(data_pts, mean)
    # Store the center of the object
    cntr = (int(mean[0, 0]), int(mean[0, 1]))  # u,v
    # Draw the principal components
    #cv.circle(img, cntr, 3, (255, 0, 255), 2)
    p1 = (
    cntr[0] + 0.02 * eigenvectors[0, 0] * eigenvalues[0, 0], cntr[1] + 0.02 * eigenvectors[0, 1] * eigenvalues[0, 0])
    p2 = (
    cntr[0] - 0.02 * eigenvectors[1, 0] * eigenvalues[1, 0], cntr[1] - 0.02 * eigenvectors[1, 1] * eigenvalues[1, 0])
    drawAxis(img, cntr, p1, 255, 1)
    #drawAxis(img, cntr, p2, (255, 255, 0), 5)
    angle = atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation of p1 in radians
    # angle = - atan2(-eigenvectors[1, 1], -eigenvectors[1, 0])  # orientation of p2 in radians
    grasp_axis = np.asarray(p2) - cntr
    grasp_axis = grasp_axis / np.linalg.norm(grasp_axis)
    # cv.circle(img, np.asarray(p2).astype(int), radius=10, color=255, thickness=-1)
    grasp_axis_2 = np.asarray(p1) - cntr
    grasp_axis_2 = grasp_axis_2 / np.linalg.norm(grasp_axis_2)
    return angle, cntr, grasp_axis, grasp_axis_2


def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx], idx


def collisionCheck(pts, cnt, grasp_axis, grasp_axis_2, img, pill_img):
    collision_mask = np.zeros(img.shape, np.uint8)
    pill_mask = np.copy(pill_img)

    # get grasp distance
    sz = len(pts)
    data_pts = np.empty((sz, 2), dtype=np.float64)
    theta_pts = np.empty(sz, dtype=np.float64)
    for i in range(data_pts.shape[0]):
        data_pts[i] = pts[i]
        vector = data_pts[i] - cnt
        theta_pts[i] = acos(np.dot(vector, grasp_axis) / (np.linalg.norm(vector) * np.linalg.norm(
            grasp_axis)))  # angle between p2 and each point along the contour
    _, idx_1 = find_nearest(theta_pts, 0)
    _, idx_2 = find_nearest(theta_pts, pi)
    fingercontact_1 = data_pts[idx_1].astype(int)
    fingercontact_2 = data_pts[idx_2].astype(int)
    #print('fingercontact_1',fingercontact_1)

    SCALE = 1/3
    finger_h = int(40 * SCALE)
    finger_w = int(120 * SCALE)
    finger_open_margin = 15 * SCALE
    grasp_axis_2 = grasp_axis_2 * finger_w / 2
    grasp_axis = grasp_axis * (finger_h + finger_open_margin) / 2

    # Create collision mask
    # Draw finger 1
    start_point_1 = (fingercontact_1 + grasp_axis_2 + grasp_axis).astype(int)
    end_point_1 = (fingercontact_1 - grasp_axis_2 + grasp_axis).astype(int)
    #print('start_point_1',start_point_1)
    cv.line(collision_mask, (start_point_1[0], start_point_1[1]), (end_point_1[0], end_point_1[1]), color=255,
            thickness=finger_h)
    # Draw finger 2
    start_point_2 = (fingercontact_2 + grasp_axis_2 - grasp_axis).astype(int)
    end_point_2 = (fingercontact_2 - grasp_axis_2 - grasp_axis).astype(int)
    cv.line(collision_mask, (start_point_2[0], start_point_2[1]), (end_point_2[0], end_point_2[1]), color=255,
            thickness=finger_h)
    # Draw closing area between two fingers
    pts = np.array([start_point_1, end_point_1, end_point_2, start_point_2], np.int32)
    cv.fillConvexPoly(collision_mask, pts, color=255)


    # Finger opening distance
    grasp_opening_dist = np.linalg.norm((start_point_2 + end_point_2) / 2 - (start_point_1 + end_point_1) / 2)

    # Create virtual border (box walls)
    cv.rectangle(pill_mask, (0, 0), (pill_mask.shape[1], pill_mask.shape[0]), color=255, thickness=10)

    # Collision check (between 'opening' and 'collision_mask')
    _, collision_mask = cv.threshold(collision_mask, 128, 1, cv.THRESH_BINARY | cv.THRESH_OTSU)
    _, pill_mask = cv.threshold(pill_mask, 128, 1, cv.THRESH_BINARY | cv.THRESH_OTSU)

    collision_result = cv.bitwise_and(pill_mask, collision_mask)

    num_points_in_collision = cv.sumElems(collision_result)
    # print('num_points_in_collision:', num_points_in_collision)

    # plt.subplot(1, 3, 1), plt.imshow(collision_mask)
    # plt.subplot(1, 3, 2), plt.imshow(pill_mask)
    # plt.subplot(1, 3, 3), plt.imshow(collision_result)
    # plt.show()

    # Visualization of contacts
    #cv.circle(img, (fingercontact_1[0], fingercontact_1[1]), radius=10, color=255, thickness=-1)
    #cv.circle(img, (fingercontact_2[0], fingercontact_2[1]), radius=10, color=255, thickness=-1)

    # if collision_result.max() == 0:
    if num_points_in_collision[0] < 30:  
        # collision free
        # Visualization (Only visualize collision free grasps)
        cv.line(img, (start_point_1[0], start_point_1[1]), (end_point_1[0], end_point_1[1]), color=125,
                thickness=finger_h)
        cv.line(img, (start_point_2[0], start_point_2[1]), (end_point_2[0], end_point_2[1]), color=125,
                thickness=finger_h)
        # cv.fillConvexPoly(img, pts, color=255)
        return True, cnt, grasp_opening_dist  # collision free
    else:
        return False, cnt, grasp_opening_dist


def grasp_detection(img, img_ins_seg, img_sem_seg):
    img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    '''
    img_gray = cv.imread('test2.jpg', 0)
    # Otsu's thresholding after Gaussian filtering
    blur = cv.GaussianBlur(img_gray, (5, 5), 0)
    ret3, th3 = cv.threshold(blur, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
    # # Opening
    kernel = np.ones((5, 5), np.uint8)
    opening = cv.morphologyEx(th3, cv.MORPH_OPEN, kernel)

    # Marker labelling
    ret, markers = cv.connectedComponents(opening)
    # Add one to all labels so that sure background is not 0, but 1
    markers = markers + 1

    # TODO
    # Complete watershed usage
    # img = cv.imread('test2.jpg')
    # markers = cv.watershed(img,markers)
    # img[markers == -1] = [255,0,0]

    markers_ctrp = np.copy(markers)
    '''
    markers = np.copy(img_ins_seg)
    markers_ctrp = np.copy(img_ins_seg)
    #_, opening = cv.threshold(img_ins_seg, 1, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
    _, opening = cv.threshold(img_ins_seg, 1, 255, cv.THRESH_BINARY)
    
    #plt.imshow(markers_ctrp), plt.show()
    
    collision_free_grasp_count = 0
    collision_free_grasp = {'grasp_coord': [], 'grasp_angle': [], 'grasp_opening':[], 'class':[]}

    for i in range(markers.max()):
        # itr on each pill
        cnt = i + 1
        if cnt == 1:
            continue  # cnt=1: background

        # Find main axis of each pill
        curr_pill_mask = (markers == cnt)  # true/false array
        curr_pill_mask = curr_pill_mask.astype(np.uint8)
        curr_pill_mask *= 255  # convert to 0/255 binary img
        # plt.imshow(opening - curr_pill_mask), plt.show()  # for collision check

        # Contour
        contours, _ = cv.findContours(curr_pill_mask, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
        largest_contour_area = 0
        largest_contour_idx = 0
        for i, c in enumerate(contours):
            area = cv.contourArea(c)
            
            # if area < 1e2 or 1e5 < area:
            #     continue
            if area > largest_contour_area:
                largest_contour_area = area
                largest_contour_idx = i
        #print('area=', cv.contourArea(contours[largest_contour_idx]), largest_contour_area)
        if largest_contour_area < 100:
            continue
            
        #print('largest_contour_idx',largest_contour_idx)
        #cv.drawContours(markers_ctrp, [contours[largest_contour_idx]], i, 255, 2)
        cv.drawContours(markers_ctrp, contours, largest_contour_idx, 255, 2)
        angle, cntr, grasp_axis, grasp_axis_2 = getOrientation(contours[largest_contour_idx], markers_ctrp)

        collision_free, grasp_coord, grasp_opening = collisionCheck(contours[largest_contour_idx], cntr, grasp_axis, grasp_axis_2,
                                                     markers_ctrp, opening - curr_pill_mask)
        collision_free_grasp_count += collision_free
        if collision_free:
            drug_class = img_sem_seg[grasp_coord[1], grasp_coord[0]]
            # print('drug_class:', drug_class, 'grasp_coord:', grasp_coord, 'angle(deg):', angle/pi*180, 'angle:', angle)
            # print('grasp_coord[0], grasp_coord[1]:', grasp_coord[0], grasp_coord[1])
            # plt.imshow(img_sem_seg)
            # plt.show()
            collision_free_grasp['grasp_angle'].append(angle)
            collision_free_grasp['grasp_coord'].append(grasp_coord)
            collision_free_grasp['grasp_opening'].append(grasp_opening)
            collision_free_grasp['class'].append(drug_class)

    # End for itr on each pill #
    # print("In total,", collision_free_grasp_count, "grasps is collision-free.")
    # print(collision_free_grasp)

    # Visualization
    # ax1 = plt.subplot(2, 3, 1)
    # ax1.set_title("img_gray")
    # plt.imshow(img_gray)
    # ax2 = plt.subplot(2, 3, 2), ax2.set_title("markers"),plt.imshow(markers)
    # ax3 = plt.subplot(2, 3, 3), ax3.set_title("opening"),plt.imshow(opening, 'gray')
    # ax4 = plt.subplot(2, 3, 4), ax4.set_title("markers_ctrp"),plt.imshow(markers_ctrp)
    # ax5 = plt.subplot(2, 3, 5), ax5.set_title("img_ins_seg"),plt.imshow(img_ins_seg)
    # ax6 = plt.subplot(2, 3, 6), ax6.set_title("img_sem_seg"),plt.imshow(img_sem_seg)
    # plt.show()

    
    fig, axs = plt.subplots(2, 3, constrained_layout=True)
    axs[0, 0].imshow(img[:,:,::-1])
    axs[0, 0].set_title('img_gray')
    axs[0, 1].imshow(markers)
    axs[0, 1].set_title('markers')    
    axs[0, 2].imshow(opening)
    axs[0, 2].set_title('opening') 
    axs[1, 0].imshow(markers_ctrp)
    axs[1, 0].set_title('markers_ctrp') 
    axs[1, 1].imshow(img_ins_seg)
    axs[1, 1].set_title('img_ins_seg') 
    axs[1, 2].imshow(img_sem_seg)
    axs[1, 2].set_title('img_sem_seg') 
    plt.show()
    

    return collision_free_grasp_count, collision_free_grasp


def push_motion_generation(img):
    # ms = MouseControl(plt.imread("test2.jpg"))
    ms = MouseControl(img)
    ms.click_detect()
    return ms.motion_start, ms.motion_end, ms.motion_type


def get_random_grasp(collision_free_grasp):
    # Get a random grasp
    grasp_i = random.choice(range(len(collision_free_grasp['grasp_angle'])))
    grasp_coord = collision_free_grasp['grasp_coord'][grasp_i]
    grasp_angle = collision_free_grasp['grasp_angle'][grasp_i]
    grasp_opening = collision_free_grasp['grasp_opening'][grasp_i]
    grasp_class = collision_free_grasp['class'][grasp_i]
    return grasp_coord, grasp_angle, grasp_opening, grasp_class


def think(img, img_ins_seg, img_sem_seg, depth):
    
    motion_command  = 0  # 0:pushing, 1:swiping, 2:picking
    push_start      = (0, 0)
    push_end        = (0, 0)
    grasp_coord     = (0, 0)
    grasp_angle     = 0
    grasp_opening   = 0
    
    collision_free_grasp_count, collision_free_grasp = grasp_detection(img, img_ins_seg, img_sem_seg)
    
    # collision_free_grasp_count = 0  # for debug only
    if collision_free_grasp_count == 0:
        print('No available picking target, do pushing!')
                            # TODO:
                            # Get a new image, and locate centers
                            # Find the closest 2 centers, 
                            # and the pushing-end-point should be the center of the line connecting two closest pills
                            # A line perpendicular to the line connecting two closest pills
                            # Two available pushing directions
                            # Determine the pushing distance (determining the pushing-starting-point)
        
        # Human-in-the-loop Manipulation
        push_start, push_end, motion_command = push_motion_generation(img)
        
        # return motion_command, push_start, push_end, grasp_coord, grasp_angle, grasp_opening
    
    else:
        print("In total,", collision_free_grasp_count, "grasps is collision-free.")
        # motion_command = 2
        #print(collision_free_grasp)
        # Get a random grasp (all in image coordinate)
        grasp_coord, grasp_angle, grasp_opening, grasp_class = get_random_grasp(collision_free_grasp)
        motion_command = grasp_class
        print("Get a random grasp:", grasp_coord, grasp_angle, grasp_opening, grasp_class)

    K = np.array([[914.5117797851562, 0.0, 645.9793701171875], 
                [0.0, 912.8472290039062, 341.26898193359375], 
                [0.0, 0.0, 1.0]])
    push_start_3d = np.dot(np.linalg.inv(K), depth*np.array([push_start[0], push_start[1], 1]).transpose())
    push_end_3d = np.dot(np.linalg.inv(K), depth*np.array([push_end[0], push_end[1], 1]).transpose())
    grasp_coord_3d = np.dot(np.linalg.inv(K), depth*np.array([grasp_coord[0], grasp_coord[1], 1]).transpose())
    return motion_command, push_start_3d[0:2], push_end_3d[0:2], grasp_coord_3d[0:2], grasp_angle, grasp_opening


if __name__ == "__main__":
    motion_command, push_start, push_end, grasp_coord, grasp_angle, grasp_opening = think(img, img_ins_seg, img_sem_seg, depth)
