import queue
import time
import matplotlib.pyplot as plt
import numpy as np
import math as mt
from collections import deque

start_node = [100.0,16.0,0.0]
goal_node = [384.0,234.0,0.0] 
cost_to_come = 0
visited_node = {}
Queue = queue.PriorityQueue()
visited_region = np.zeros((800,500,12))  # matrix to store the visited region. It is 3 dimensional matrix
        
##############################################################
#### Define obstacle zone considering clearance and radius ###
##############################################################

# Function to check whether any part of the robot is in the obstacle region or not
def isObstacle(coordinates,radius, clearance) : 
    #Center of the robot(node)
    x_c, y_c,_= coordinates
    # queue to store points along the circumference of given robot position
    cir=deque()
    # Dividing the circle into 36 equal points
    for i in range(36):
        t = (2*mt.pi)*(i) /36
        # Radius of circle = radius of robot + clearance 
        y = y_c + (radius+clearance)*mt.sin((t))
        x = x_c+ (radius+clearance)*mt.cos((t))
        x=round(x)
        y=round(y)
        cir.append((x,y))
    # Passing the coordinates of robot circle through each shape    
    bool_1 = circle(cir) 
    bool_2 = hexagon(cir) 
    bool_3 = quadrilateral(cir) 
    bool_4 = boundary(cir)
    # Returns true if any one of point along circumference lies in the obstacle
    if bool_1 or bool_2 or bool_3 or bool_4 :
        return True
    else :
        return False

    
def circle(arr):
    # Passing each value of queue through circle equation
    for i in range(36) :
        b = (arr[i][0] - 300)**2 + (arr[i][1] - 185)**2 < 40**2
        if b == True:
            return b
    return False

def hexagon(arr):
    for i in range(36) :
        # Using half-plane method for the 6 line equations 
        if (4.044*arr[i][0])+(7*arr[i][1])-1225.895 > 0 :
            if (-4.044*arr[i][0])+(7*arr[i][1])+391.705 > 0 :
                if (-1*arr[i][0])+235 > 0 :
                    if (-4.039*arr[i][0])+(-7*arr[i][1])+1790.705 > 0 :
                        if (4.039*arr[i][0])+(-7*arr[i][1])+175.105 > 0 :
                            if (arr[i][0])-165 > 0 :
                                return True                        

# Divided the quadrilateral into 2 triangles and evaluating half-plane for each triangle
def quadrilateral(arr): 
    for i in range(36) :
        b_1 = False
        b_2 = False
        if (85*arr[i][0])+(69*arr[i][1])-15825 > 0 :
            if (-16*arr[i][0])+(-5*arr[i][1])+2180 > 0 :
                if (-5*arr[i][0])+(-44*arr[i][1])+8320 > 0 :
            
                    b_1 = True
        if (-6*arr[i][0])+(7*arr[i][1])-780 > 0 :
            if (25*arr[i][0])+(-79*arr[i][1])+13715 > 0 :
                if (-5*arr[i][0])+(-44*arr[i][1])+8320 < 0 :
            
                    b_2 = True
        # Evaluates to true if point is present in either of the two triangles            
        if b_1 or b_2 :
            return True 

def boundary(arr):    
    for i in range(36) :
        # These equations signify the horizonal and vertical boundaries of the plane
        if arr[i][0]<(0) or arr[i][0]>(400) or arr[i][1]<(0) or arr[i][1]>(250) :
            return True


##################################
#### plot define obstacle zone ###
##################################

x_c =[]
y_c =[]
# Defined functions for the obstacle regions using half-plane method
def actual_circle(x,y):
    return ((x - 300)**2 + (y - 185)**2) < 40**2

def actual_hexagon(x,y):
    if (4.044*x)+(7*y)-1225.895 > 0 :
        if (-4.044*x)+(7*y)+391.705 > 0 :
            if (-1*x)+235 > 0 :
                if (-4.039*x)+(-7*y)+1790.705 > 0 :
                    if (4.039*x)+(-7*y)+175.105 > 0 :
                        if (x)-165 > 0 :
                            return True

def actual_quadrilateral(x,y): 
    b_1 = False
    b_2 = False
    if (85*x)+(69*y)-15825 > 0 :
        if (-16*x)+(-5*y)+2180 > 0 :
            if (-5*x)+(-44*y)+8320 > 0 :
                b_1 = True
    if (-6*x)+(7*y)-780 > 0 :
        if (25*x)+(-79*y)+13715 > 0 :
            if (-5*x)+(-44*y)+8320 < 0 :        
                b_2 = True
    if b_1 or b_2 :
        return True 

# Function to check whether a given point is in obstacle region or not         
def actual_obstacle(X,Y) : 
    bool_1 = actual_circle(X,Y) 
    bool_2 = actual_hexagon(X,Y) 
    bool_3 = actual_quadrilateral(X,Y)     
    if bool_1 or bool_2 or bool_3  :
        return True
    else :
        return False

# Passing all coordinates to find obstacle coordinates
for i in range(401) :
    for j in range(251) :
        if actual_obstacle(i,j) :
            x_c.append(i)
            y_c.append(j) 
# Plotting obstacles
plt.scatter(x_c , y_c , c='yellow' , s=5)

#######################################
#### Function for taking user input ###
#######################################

# Takes user input for 'robot radius' and 'clearance'
def takeRobParam():
    print("Please enter robot radius: ")
    radius = int(input())
    print("Please enter clearance: ")
    clearance = int(input())
    return radius, clearance

# Function to input step-size
def takeStepSize():
    print("Please enter step size of movement, it should be 0 <= step <=10: ")
    step = int(input())
    if step <= 0 or step > 10:
        print("step size should be between 0 and 10 only.")
        takeStepSize()
    return int(step)

# Takes 'x coordinate, y coordinate, orientation' of start position 
def takeStartInput():
    print("Enter x and y co-ordinate of start node. Please press 'Enter' key after adding each element: ")
    for i in range(3):
         # Inputing coordinates
        if i<2:
            start_node[i] = int(input())
        # Inputing orientation
        if i==2:
            print("please enter k (0 <= k <= 11) for orientation of start node of the robot. Orientation will be equal to 30*k: ")
            k = int(input())
            start_node[i] = 30*k
            if k < 0 or k > 11:
                print("k should be 0 <= k <= 11.")
                takeStartInput()
            
    # Checking for obstacle
    isTrue = isObstacle(start_node,radius,clearance)
    if isTrue:
        print("Start node is either in obstacle area or in clearnace area. Please enter start node again. ")
        takeStartInput()

# Takes 'x coordinate, y coordinate, orientation' of goal position    
def takeGoalInput():
    print("Enter x and y co-ordinate of goal node. Please press 'Enter' key after adding each element: ")
    for i in range(3):
        if i<2:
            goal_node[i] = int(input())
        if i==2:
            if i==2:
                print("please enter k (0 <= k <= 11) for orientation of goal node of the robot. Orientation will be equal to 30*k: ")
            k = int(input())
            goal_node[i] = 30*k
            if k < 0 or k > 11:
                print("k should be 0 <= k <= 11")
                takeGoalInput()
            
    # Checking for obstacle
    isTrue = isObstacle(goal_node,radius,clearance)
    if isTrue:
        print("Goal node is either in obstacle area or in clearnace area. Please enter goal node again. ")
        takeGoalInput()

    
############################
#### Function for action ###
############################

# Creation of new nodes
def exploreNode(current_node,cost,step):

    # calculate the new direction for exploration
    theta = current_node[2]
    theta1 = theta-60
    if theta1 < 0:
        theta1 = theta1 + 360
    theta2 = theta-30
    if theta2 < 0:
        theta2 = theta2 + 360
    theta3 = theta
    theta4 = theta + 30
    if theta4 >=360:
        theta4 = theta4 - 360
    theta5 = theta + 60
    if theta5 >= 360:
        theta5 = theta5 - 360 
    # Assigning new cost from start node     
    cost_to_come = cost + 1

    # generate node in theta1 direction
    node1 = (round((current_node[0]+step*mt.cos(theta1*mt.pi/180))*2)/2,round((current_node[1]+step*mt.sin(theta1*mt.pi/180))*2)/2,theta1)
    # check if the new node is falling in obstacle
    isTrue = isObstacle(node1, radius, clearance)
    if not isTrue:
        # check if the new node is already visited
        if not isVisited(node1):
            cost_to_go = costToGo(node1)
            # calculate new cost for the new node
            new_cost = cost_to_come + cost_to_go
            # append new node into priority queue
            Queue.put((new_cost,(node1[0], node1[1],theta1,cost_to_come,current_node[0], current_node[1],theta)))
            plt.arrow(current_node[0], current_node[1],node1[0]-current_node[0],node1[1]-current_node[1],fc="k", head_width=0.5, head_length=0.1)
            # plt.pause(0.005)
            
    # generate node in theta2 direction        
    node2 = (round((current_node[0]+step*mt.cos(theta2*mt.pi/180))*2)/2,(round(current_node[1]+step*mt.sin(theta2*mt.pi/180))*2)/2,theta2)
    # check if the new node is falling in obstacle
    isTrue = isObstacle(node2, radius, clearance)
    if not isTrue:
        # check if the new node is already visited
        if not isVisited(node2):
            cost_to_go = costToGo(node2)
            # calculate new cost for the new node
            new_cost = cost_to_come + cost_to_go
            # append new node into priority queue
            Queue.put((new_cost,(node2[0], node2[1],theta2,cost_to_come,current_node[0], current_node[1],theta)))
            plt.arrow(current_node[0], current_node[1],node2[0]-current_node[0],node2[1]-current_node[1],fc="k", head_width=0.5, head_length=0.1)
            # plt.pause(0.005)
     
    # generate node in theta3 direction        
    node3 = (round((current_node[0]+step*mt.cos(theta3*mt.pi/180))*2)/2,round((current_node[1]+step*mt.sin(theta3*mt.pi/180))*2)/2,theta3)
    # check if the new node is falling in obstacle
    isTrue = isObstacle(node3, radius, clearance)
    if not isTrue:
        # check if the new node is already visited
        if not isVisited(node3):
            cost_to_go = costToGo(node3)
            # calculate new cost for the new node
            new_cost = cost_to_come + cost_to_go
            # append new node into priority queue
            Queue.put((new_cost,(node3[0], node3[1],theta3,cost_to_come,current_node[0], current_node[1],theta)))
            plt.arrow(current_node[0], current_node[1],node3[0]-current_node[0],node3[1]-current_node[1],fc="k", head_width=0.5, head_length=0.1)
            # plt.pause(0.005)
    
    # generate node in theta4 direction    
    node4 = (round((current_node[0]+step*mt.cos(theta4*mt.pi/180))*2)/2,round((current_node[1]+step*mt.sin(theta4*mt.pi/180))*2)/2,theta4)
    # check if the new node is falling in obstacle
    isTrue = isObstacle(node4, radius, clearance)
    if not isTrue:
        # check if the new node is already visited
        if not isVisited(node4):
            cost_to_go = costToGo(node4)
            # calculate new cost for the new node
            new_cost = cost_to_come + cost_to_go
            # append new node into priority queue
            Queue.put((new_cost,(node4[0], node4[1],theta4,cost_to_come,current_node[0], current_node[1],theta)))
            plt.arrow(current_node[0], current_node[1],node4[0]-current_node[0],node4[1]-current_node[1],fc="k", head_width=0.5, head_length=0.1)
            # plt.pause(0.005)
    
    # generate node in theta5 direction        
    node5 = (round((current_node[0]+step*mt.cos(theta5*mt.pi/180))*2)/2,round((current_node[1]+step*mt.sin(theta5*mt.pi/180))*2)/2,theta5)
    # check if the new node is falling in obstacle
    isTrue = isObstacle(node5, radius, clearance)
    if not isTrue:
        # check if the new node is already visited
        if not isVisited(node5):
            cost_to_go = costToGo(node5)
            # calculate new cost for the new node
            new_cost = cost_to_come + cost_to_go
            # append new node into priority queue
            Queue.put((new_cost,(node5[0], node5[1],theta5,cost_to_come,current_node[0], current_node[1],theta)))
            plt.arrow(current_node[0], current_node[1],node5[0]-current_node[0],node5[1]-current_node[1],fc="k", head_width=0.5, head_length=0.1)
            # plt.pause(0.005)

    plt.pause(0.005)
    
# function to compute cost to reach goal node
# cost to reach goal node is euclidean distance between current node and goal node
def costToGo(X):
    X = np.array([X[0],X[1]])
    Y = np.array([goal_node[0],goal_node[1]])
    return np.sqrt(np.sum((X-Y)**2))

# function to check whether current node is goal node or not
# if the current node is within 1.5 unit euclidean distance of the goal node, it will be considered as goal node
def isGoalNode(X):
    rotation = X[2]
    X = np.array([X[0],X[1]])
    Y = np.array([goal_node[0],goal_node[1]])
    if np.sqrt(np.sum((X-Y)**2)) <= 1.5 and rotation==goal_node[2]:
        return True
    else:
        return False

# update visited region
def updateVisitedRegion(X):
    i = int(X[0]*2)
    j = int(X[1]*2)
    k1 = int(X[2]/30)
    k2 = k1 - 1
    if k2 < 0:
        k2 = 11
    k3 = k1 + 1
    if k3 > 11:
        k3 = 0
    visited_region[i][j][k1] = 1
    visited_region[i][j][k2] = 1
    visited_region[i][j][k3] = 1
    return None

# check whether the current node is visited or not
def isVisited(X):
    i = int(X[0]*2)
    j = int(X[1]*2)
    k = int(X[2]/30)
    if visited_region[i][j][k]:
        return True
    else:
        return False

##################
#### Main loop ###
##################

radius, clearance = takeRobParam()  # take radius and clearance input from user
takeStartInput()                    # take start position and orientation input from user
takeGoalInput()                     # take goal position and orientation input from user
start_node = tuple(start_node)      
goal_node = tuple(goal_node)
step = takeStepSize()               # take step size input from user
total_cost = cost_to_come + costToGo(start_node)

# Inserting first element into the queue
# The first value of each element is 'total cost'
# The second value has the following structure:
# (x position current, y position current , orientation current , cost to come, x parent , y parent , orientation parent)  
Queue.put((total_cost,(start_node[0],start_node[1],start_node[2],cost_to_come,start_node[0],start_node[1],start_node[2])))

start = time.time()
plt.plot()
plt.axis([0,400,0,250])
plt.title("exploring map to find goal node")

print("finding optimal path to reach goal....")
while True:
    # pop node from priority queue
    pop_node = Queue.get()
    current_node = (pop_node[1][0], pop_node[1][1], pop_node[1][2])
    cost_to_come = pop_node[1][3]
    parent_node = (pop_node[1][4], pop_node[1][5],pop_node[1][6])
    cost = pop_node[0]
    
    # check if the current node is goal node
    if isGoalNode(current_node):
        visited_node[current_node] = parent_node
        # redefine the goal to the nearest reachanble position to the actual goal position
        goal_node = (current_node)
        print('reached at goal node')
        break
    
    # if the current node is not goal node, explore the next visiting node from current node
    if  isVisited(current_node)== False:
        visited_node[current_node] = parent_node
        # update the visited note in visited region
        updateVisitedRegion(current_node)
        # explore the new node from current node
        exploreNode(current_node,cost_to_come,step)
        
        
######################
#### Back-tracking ###
######################

# Seperating key and values from the visited_node dictionary
# key is the node and value is its parents node
# key list contain all the node and value list contain all the its parents 
key_list = list(visited_node.keys())
value_list = list(visited_node.values())

# Storing nodes of optimal path
x_path = []
y_path = []
while True:

    # find the index in the key list corrosponding node
    # find the value in the value list corrospoding to the above index, it will be its parents node 
    position = key_list.index((goal_node))
    value = value_list[position]
    # check if the index is start node during back tracking
    if key_list[position]==(start_node):
        x_path.append(key_list[position][0])
        y_path.append(key_list[position][1])
        break
    else:
        # append the first elements of node to x_path and y element of node to y_path
        x_path.append(key_list[position][0])
        y_path.append(key_list[position][1])
        # parent become next node and while loop will continue to run to find its parents 
        goal_node = value

##############################
#### Plotting Optimal path ###
##############################

x = []
y = []

while True:
    if not x_path:
        break
    plt.title("Plotting optimal path to travel from start node to goal node using A* algorithm")
    x.append(x_path.pop())
    y.append(y_path.pop())
    plt.plot(x, y, c = 'red', linewidth=3)
    plt.pause(0.005)

plt.title("Optimal path to travel from start node to goal node using A* algorithm")
end = time.time()
print("time taken to run the code"+ " : " + str(end-start)+ " seconds ")       
plt.show()   




    
    
    
    




    


    

     












