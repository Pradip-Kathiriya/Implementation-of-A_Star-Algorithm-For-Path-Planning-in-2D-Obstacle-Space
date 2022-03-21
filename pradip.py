import queue
import time
from turtle import width
import matplotlib.pyplot as plt
import numpy as np
import math as mt
from collections import deque 

start_node = [0.0,0.0,0.0]
goal_node = [0.0,0.0,0.0] 
cost_to_come = 0
visited_node = {}
Queue = queue.PriorityQueue()
x_node = []
y_node = []
x_par = []
y_par = []
clearance = 0
radius = 0
step = 0
        
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
        if arr[i][0]<=(1) or arr[i][0]>=(399) or arr[i][1]<=(1) or arr[i][1]>=(249) :
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
    if step < 0 or step > 10:
        print("step size should be between 0 and 10 only.")
        takeStepSize()
    return step

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
            if k < 0 or k > 11:
                print("k should be 0 <= k <= 11.")
                takeStartInput()
            start_node[i] = 30*k
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
            if k < 0 or k > 11:
                print("k should be 0 <= k <= 11")
                takeGoalInput()
            goal_node[i] = 30*k
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
    cost_to_come = cost + step


    node1 = (current_node[0]+step*mt.cos(theta1*mt.pi/180),current_node[1]+step*mt.sin(theta1*mt.pi/180),theta1)
    isTrue = isObstacle(node1, radius, clearance)
    if not isTrue:
        if not isVisited(node1):
            cost_to_go = costToGo(node1)
            new_cost = cost_to_come + cost_to_go
            Queue.put((new_cost,(node1[0], node1[1],theta1,cost_to_come,current_node[0], current_node[1],theta)))
            
    node2 = (current_node[0]+step*mt.cos(theta2*mt.pi/180),current_node[1]+step*mt.sin(theta2*mt.pi/180),theta2)
    isTrue = isObstacle(node2, radius, clearance)
    if not isTrue:
        if not isVisited(node2):
            cost_to_go = costToGo(node2)
            new_cost = cost_to_come + cost_to_go
            Queue.put((new_cost,(node2[0], node2[1],theta2,cost_to_come,current_node[0], current_node[1],theta)))
            
    node3 = (current_node[0]+step*mt.cos(theta3*mt.pi/180),current_node[1]+step*mt.sin(theta3*mt.pi/180),theta3)
    isTrue = isObstacle(node3, radius, clearance)
    if not isTrue:
        if not isVisited(node3):
            cost_to_go = costToGo(node3)
            new_cost = cost_to_come + cost_to_go
            Queue.put((new_cost,(node3[0], node3[1],theta3,cost_to_come,current_node[0], current_node[1],theta)))
        
    node4 = (current_node[0]+step*mt.cos(theta4*mt.pi/180),current_node[1]+step*mt.sin(theta4*mt.pi/180),theta4)
    isTrue = isObstacle(node4, radius, clearance)
    if not isTrue:
        if not isVisited(node4):
            cost_to_go = costToGo(node4)
            new_cost = cost_to_come + cost_to_go
            Queue.put((new_cost,(node4[0], node4[1],theta4,cost_to_come,current_node[0], current_node[1],theta)))
            
    node5 = (current_node[0]+step*mt.cos(theta5*mt.pi/180),current_node[1]+step*mt.sin(theta5*mt.pi/180),theta5)
    isTrue = isObstacle(node5, radius, clearance)
    if not isTrue:
        if not isVisited(node5):
            cost_to_go = costToGo(node5)
            new_cost = cost_to_come + cost_to_go
            Queue.put((new_cost,(node5[0], node5[1],theta5,cost_to_come,current_node[0], current_node[1],theta)))

# function to compute cost to reach goal node
# cost to reach goal node is euclidean distance between current node and goal node
def costToGo(X):
    X = np.array([X[0],X[1]])
    Y = np.array([goal_node[0],goal_node[1]])
    return np.sqrt(np.sum((X-Y)**2))

# function to check whether current node is goal node or not
# if the current node is within 1.5 unit euclidean distance of the goal node, it will be considered as goal node
def isGoalNode(X):
    X = np.array([X[0],X[1]])
    Y = np.array([goal_node[0],goal_node[1]])
    if np.sqrt(np.sum((X-Y)**2)) <= 1.5:
        return True
    else:
        return False

# matrix to store the visited region. It is 3 dimensional matrix
visited_region = np.zeros((800,500,12))

# update visited region
def updateVisitedRegion(X):
    i = int(round(X[0]*2))
    j = int(round(X[1]*2))
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
    i = int(round(X[0]*2))
    j = int(round(X[1]*2))
    k = int(X[2]/30)
    if visited_region[i][j][k]:
        return True
    else:
        return False

##################
#### Main loop ###
##################

radius, clearance = takeRobParam()
takeStartInput()
takeGoalInput()
start_node = tuple(start_node)
goal_node = tuple(goal_node)
step = takeStepSize()
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
    # getting the element with lowest toatal cost
    pop_node = Queue.get()
    current_node = (pop_node[1][0], pop_node[1][1], pop_node[1][2])
    cost_to_come = pop_node[1][3]
    parent_node = (pop_node[1][4], pop_node[1][5],pop_node[1][6])
    cost = pop_node[0]
    # Only implemented when goal node is reached
    if isGoalNode(current_node):
        visited_node[current_node] = parent_node
        x_node.append(current_node[0])
        y_node.append(current_node[1])
        x_par.append(parent_node[0])
        y_par.append(parent_node[1])
        goal_node = (current_node)
        print('reached at goal node')
        break
    
    # When current node is not goal
    if  isVisited(current_node)== False:
        visited_node[current_node] = parent_node
        updateVisitedRegion(current_node)
        x_node.append(current_node[0])
        y_node.append(current_node[1])
        x_par.append(parent_node[0])
        y_par.append(parent_node[1])
        # Creating new node
        exploreNode(current_node,cost_to_come,step)

######################
#### Back-tracking ###
######################
# Seperating key and values from the visited_node dictionary
key_list = list(visited_node.keys())
value_list = list(visited_node.values())

# Storing nodes of optimal path
optimal_path = []
x_path = []
y_path = []
while True:

    position = key_list.index((goal_node))
    value = value_list[position]
    if key_list[position]==(start_node):
        optimal_path.append(key_list[position])
        x_path.append(key_list[position][0])
        y_path.append(key_list[position][1])
        break
    else:
        optimal_path.append(key_list[position])
        x_path.append(key_list[position][0])
        y_path.append(key_list[position][1])
        goal_node = value
        
# # ##################
# # #### Animation ###
# # ##################
    
i = 0
j = 0
# plotting vectors for explored nodes
while True:
    x_temp = x_node.pop(0)
    y_temp = y_node.pop(0)
    u_temp = x_par.pop(0)
    v_temp = y_par.pop(0)

    plt.arrow(u_temp, v_temp,x_temp-u_temp, y_temp-v_temp,fc="k", ec="k", head_width=0.05, head_length=0.1)
    i = i+1
    if len(x_node) > 10000:
        if i > 10000:
            plt.pause(0.005)
            i = 0
    elif len(x_node) > 5000:
        if i > 5000:
            plt.pause(0.005)
            i = 0
    elif len(x_node) > 1000:
        if i > 1000:
            plt.pause(0.005)
            i = 0
    elif len(x_node) > 500:
        if i > 500:
            plt.pause(0.005)
            i = 0
    elif len(x_node) > 100:
        if i > 100:
            plt.pause(0.005)
            i = 0
    elif len(x_node) > 10:
        if i > 10:
            plt.pause(0.005)
            i = 0
    else:
        plt.pause(0.005)         
    if not x_node:
        plt.pause(0.005)
        break
        
plt.plot(x_path, y_path, c = 'red', linewidth=3)
plt.title("Optimal path to travel from start node to goal node using A* algorithm")
end = time.time()
print("time taken to run the code"+ " : " + str(end-start)+ " seconds ")       
plt.show()   




    
    
    
    




    


    

     












