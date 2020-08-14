#Hung-Hsiu Yen(yen.142)
#Real Time Robotics HW

import numpy as np 
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#Create a forward kinematics matrix
def forward_kinematics_matrix(a, alpha,di,thetai):
    return np.array([[math.cos(math.radians(thetai)), -1*math.sin(math.radians(thetai)), 0, a],
              [math.sin(math.radians(thetai))*math.cos(math.radians(alpha)), math.cos(math.radians(thetai))*math.cos(math.radians(alpha)), -1*math.sin(math.radians(alpha)),  -1*math.sin(math.radians(alpha))*di],
              [math.sin(math.radians(thetai))*math.sin(math.radians(alpha)), math.cos(math.radians(thetai))*math.sin(math.radians(alpha)),    math.cos(math.radians(alpha)),     math.cos(math.radians(alpha))*di],
              [0, 0, 0, 1]])

#Create a initial robot arm position
x_position=[0,0,5,15,15]
y_position=[0,0,0,0,0]
z_position=[0,10,10,10,5]

#Save joint position to draw dot
joints_x=[x_position[0],x_position[2],x_position[3],x_position[4]]
joints_y=[y_position[0],y_position[2],y_position[3],y_position[4]]
joints_z=[z_position[0],z_position[2],z_position[3],z_position[4]]

#Create matrix for calculation
first_joint_position= np.array([[x_position[2], y_position[2], z_position[2],1]])
second_jiont_position = np.array([[x_position[3]-5,y_position[3],z_position[3],1]])
end_effector = np.array([[x_position[4]-5,y_position[4],z_position[4],1]])



#<----- Run Simulation----->
while True:
      #Draw Robot Arm
      ax = Axes3D(plt.gcf())
      ax.set_zlim(0,10)
      ax.set_ylim(-10,10)
      ax.set_xlim(-10,10)
      ax.set_xticks(np.arange(-10, 10, 2))
      ax.set_yticks(np.arange(-10, 10, 2))
      ax.plot(x_position, y_position,z_position)
      ax.plot(joints_x,joints_y,joints_z,'ro')
      plt.title('Current Position of End Effector: X=%f Y=%f Z=%f'%(x_position[-1],y_position[-1],z_position[-1]))
      ax.set_xlabel("X")
      ax.set_ylabel("Y")
      ax.set_zlabel("Z")
      plt.pause(0.1)

      #Enter first angle
      angle_1 = raw_input('Please Enter the Angle of Joint 1 or Enter "end" to turn off the arm:')
      if angle_1 == 'end' or not angle_1:
        print "Thanks for Using"
        break
      else:
         T_1_to_0 = forward_kinematics_matrix(0,0,0,float(angle_1))
         new_position_point_prime = T_1_to_0.dot(first_joint_position.transpose())
         new_position_point = new_position_point_prime.tolist()
         x_position[-3] = joints_x[-3] = new_position_point[0][0]
         y_position[-3] = joints_y[-3] = new_position_point[1][0]
         z_position[-3] = joints_z[-3] = new_position_point[2][0]
         new_position_point_prime = T_1_to_0.dot(second_jiont_position.transpose())
         new_position_point = new_position_point_prime.tolist()
         x_position[-2] = joints_x[-2] = new_position_point[0][0]
         y_position[-2] = joints_y[-2] = new_position_point[1][0]
         z_position[-2] = joints_z[-2] = new_position_point[2][0]
         new_position_point_prime = T_1_to_0.dot(end_effector.transpose())
         new_position_point = new_position_point_prime.tolist()
         x_position[-1] = joints_x[-1]= new_position_point[0][0]
         y_position[-1] = joints_y[-1]= new_position_point[1][0]
         z_position[-1] = joints_z[-1]= new_position_point[2][0]

      #Enter second angle
      angle_2 = raw_input('Please Enter the Angle of Joint 2 or Enter "end" to turn off the arm:')
      if angle_2 == 'end' or not angle_2:
          print "Thanks for Using"
          break
      else: 
         T_2_to_1 = forward_kinematics_matrix(5,0,0,float(angle_2))
         T_2_to_0 = T_1_to_0.dot(T_2_to_1)
         new_position_point_prime = T_2_to_0.dot(second_jiont_position.transpose())
         new_position_point = new_position_point_prime.tolist()
         x_position[-2] = joints_x[-2] =  new_position_point[0][0]
         y_position[-2] = joints_y[-2] =  new_position_point[1][0]
         z_position[-2] = joints_z[-2] =  new_position_point[2][0]
         new_position_point_prime = T_2_to_0.dot(end_effector.transpose())         
         new_position_point = new_position_point_prime.tolist()
         x_position[-1] = joints_x[-1]= new_position_point[0][0]
         y_position[-1] = joints_y[-1]= new_position_point[1][0]
         z_position[-1] = joints_z[-1]= new_position_point[2][0]

      #Enter arm length      
      print "Please Enter the Length of the Arm or Enter end to turn off the arm"
      prismatic_length = raw_input('(For example -4 or 4 for reachong down or up):')
      if prismatic_length == 'end' or not prismatic_length:
          print "Thanks for Using"
          break
      else:
         T_3_to_2 = forward_kinematics_matrix(0,0,5+float(prismatic_length),0)
         T_3_to_0 = T_2_to_0.dot(T_3_to_2)
         new_position_point_prime = T_3_to_0.dot(end_effector.transpose())
         new_position_point = new_position_point_prime.tolist()         
         x_position[-1] = joints_x[-1]= new_position_point[0][0]
         y_position[-1] = joints_y[-1]= new_position_point[1][0]
         z_position[-1] = joints_z[-1]= new_position_point[2][0]

      print 'Current Position of End Effector: X=%f Y=%f Z=%f'%(x_position[-1],y_position[-1],z_position[-1])
