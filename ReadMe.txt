This is a readme file for ENPM 661 project 3 phase 1.

############
### Team ###
############

1. Pradipkumar Kathiriya (117678345)
2. Jeffin johny Kachapiilly (118293929)

##################
### About file ###
##################

This file contain implementation of A* Algorithm to find path in 2D obstacle space
obstacle space is defined used half plane method

Code is written in python 2.17.17 using visual studio

####################
### Library used ###
####################

queue, time, matplotlib, numpy, collections, math

#######################
### How to run code ###
#######################

1. Unzip the folder and save it into your local device

2. Run pradip.py file

3. It will ask user to input following data in the terminal:

   1. Robot radius: input the radius of the robot

   2. Clearance : input the clearance between obstacle and robot

   3. Start x and y position

   4. Start orientation of robot: Input digit between 0 and 11 (including 0 and 11). orientation will be equal to digit*30 degree.	

   5. Goal x and y position

   6. Goal orientation of robot: Input digit between 0 and 11 (including 0 and 11). orientation will be equal to digit*30 degree.
      # Depending on the goal orientation, program may take short or very long time to find the path. 
      # If the goal position is very near to wall or obstacle, goal position can't be reach from certain orientation. before entering goal
        orientation, please make sure it is reachable.

   7. Step size (0<=step<=10): size of step movement. Ideal step size input is 5.
4. Once you enter the data, the output will be animation showing exploration of the map to find goal node and optimal path to reach to goal.

5. Finding path takes less than 45 seconds for the largest distance.

6. Plotting of path can vary from 2-3minutes.

##################################
### Other file along with code ###
##################################

video.mkv : video showing path finding between random start and goal node
