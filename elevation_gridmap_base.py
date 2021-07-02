# -*- coding: utf-8 -*-
"""
@author: Lars Schilling

"""
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import matplotlib.pyplot as plt
import math
from scipy import signal
#add imports for the pathfinding package
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
def relu(rel):
	nrml_relu = np.maximum(rel, 0)
	return nrml_relu
def callback(msg):
        print('Recieved map data')
        bridge = CvBridge()
        data = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        height_map=np.array(data, dtype=np.uint)
        h = height_map.shape[0]
        w = height_map.shape[1]
        
        #Where there is 0 convert to 1 (better for A*)

        height_map[height_map==0] = 1
        height_costmap = height_map

        
        #create a distance cost map with the shape of height_costmap
        #pixels around the center should have low cost and get higher at the edges

        
        dist_map = np.zeros((h,w,3), dtype=np.uint8)
        imgsize = dist_map.shape[:2]
        innerColor = (0,0,0)
        outerColor = (100,100,100)
        for y in range(imgsize[1]):
        	for x in range(imgsize[0]):
        		distancetocentre= np.sqrt((x - imgsize[0]//2) ** 2 + (y - imgsize[1]//2) ** 2)
        		distancetocentre= distancetocentre/ (np.sqrt(2) * imgsize[0]/2)
        		r = outerColor[0] * distancetocentre + innerColor[0] + (1 - distancetocentre)
        		g = outerColor[1] * distancetocentre + innerColor[1] + (1 - distancetocentre)
        		b = outerColor[2] * distancetocentre + innerColor[2] + (1 - distancetocentre)

        		dist_map[y,x] = (int(r), int(g), int(b))

        #Obatined image is in RGB, convert it to Grayscale.
    
        dist_costmap = cv2.cvtColor(dist_map, cv2.COLOR_RGB2GRAY)
       	

        
        #define a combined cost map based height and distance cost map
        #this could be a sum or multiplication (we did sum)
        comb_costmap = height_costmap + dist_costmap
  

        #implement the AStarFinder from the pathfinding package to the combined cost map
        #to find a path from the center of the image (30,30) to the upper edge of the image (30,0)

        #Define a matrix for A* algorithm
        matrix = comb_costmap
        #Define Grid
        grid = Grid(matrix=matrix)
        start = grid.node(30,30)
        end = grid.node(30,0)
        #A* finder 
        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        #returns path and no of operations
        path, runs = finder.find_path(start, end, grid)
       
  
        #plot your height, distance and combined cost map, as well as the astar path
        #Operations and lenght of the path
        print('operations:', runs,'path length:', len(path))
        #prints path in terminal
        print (grid.grid_str(path=path, start=start, end=end))

        #for visualization draw path on the combined_costmap/data

        for points in path:
        	test = cv2.circle(data,tuple(points), radius = 0, color=(255,255,0), thickness=-1)
        
        #Plotting the maps and final result.

        fig = plt.figure(figsize=(15,5))
        rows = 1
        coloumns = 3
        fig.add_subplot(rows,coloumns,1)
        plt.imshow(height_costmap)
        plt.axis('off')
        plt.title("height_costmap")

        fig.add_subplot(rows,coloumns,2)
        plt.imshow(dist_costmap)
        plt.axis('off')
        plt.title("dist_costmap")

        fig.add_subplot(rows,coloumns,3)
        plt.imshow(test)
        plt.axis('off')
        plt.title("final_image")
        
        
      	
      	plt.show()

        rospy.sleep(0.5)

if __name__ == '__main__':
    try:

        rospy.init_node('elevation_path')
        sub=rospy.Subscriber("/grid_map_image", Image, callback, queue_size=1)
        rospy.spin()
    except KeyboardInterrupt or rospy.ROSInterruptException: pass
