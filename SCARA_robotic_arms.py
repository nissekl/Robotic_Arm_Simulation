#Hung-Hsiu Yen(yen.142)
#Real Time Robotics HW
#Roborics Arm Simulation

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

#Matrix Calculation and display
def matrix_calculation(joint_number, homo_matrix_list, initial = False):
    tn_to_0=[]
    for i in range(len(homo_matrix_list)):
        if i == 0:
           tn_to_0.append(homo_matrix_list[i])
        else:
           tn_to_0.append(np.dot(tn_to_0[i-1],homo_matrix_list[i]))
    if not initial:
       print 'The Homogenous Matrix %d to 0 is \n' % (joint_number)
       print tn_to_0[joint_number-1]
       return tn_to_0
    else:
       return tn_to_0 


#refresh all points function
def refresh_all_points(x_coordinate, y_coordinate, z_coordinate, homo_matrix_list):
    for i in range(len(homo_matrix_list)):
        new_p = homo_matrix_list[i].tolist()
        x_coordinate[i+1] = new_p[0][3]
        y_coordinate[i+1] = new_p[1][3]
        z_coordinate[i+1] = new_p[2][3]
    return x_coordinate, y_coordinate, z_coordinate


#draw the graph function
def draw_robot(x_draw, y_draw,z_draw):
    draw_joint_x =[x_draw[0]]+x_draw[2:7]+[x_draw[8]]
    draw_joint_y =[y_draw[0]]+y_draw[2:7]+[y_draw[8]]
    draw_joint_z =[z_draw[0]]+z_draw[2:7]+[z_draw[8]]
    ax = Axes3D(plt.gcf())
    ax.set_zlim(0,14)
    ax.set_ylim(-12,12)
    ax.set_xlim(-12,12)
    ax.set_xticks(np.arange(-12, 12, 2))
    ax.set_yticks(np.arange(-12, 12, 2))
    ax.plot(x_draw, y_draw,z_draw)
    ax.plot(draw_joint_x,draw_joint_y,draw_joint_z,'ro')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.title('Current Position of End Effector: X=%f Y=%f Z=%f'%(x_draw[-1],y_draw[-1],z_draw[-1]))
    plt.pause(0.1)


#reset all the matrix
def reset_arms():
    global T1_to_0, T2_to_1, T3_to_2, T4_to_3, T5_to_4, T6_to_5, T7_to_6, T8_to_7
    T1_to_0 = forward_kinematics_matrix(0,0,5,0)
    T2_to_1 = forward_kinematics_matrix(0,-90,5,90)
    T3_to_2 = forward_kinematics_matrix(0,90,1+5,0)
    T4_to_3 = forward_kinematics_matrix(0,0,0,0)
    T5_to_4 = forward_kinematics_matrix(0,-90,0,90)
    T6_to_5 = forward_kinematics_matrix(0,90,0,0)
    T7_to_6 = forward_kinematics_matrix(0,0,3,180)
    T8_to_7 = forward_kinematics_matrix(3,0,0,0)



#Points for Drawing
x_draw=[0, 0, 0, 0, 0, 0, 0, 0, 0]
y_draw=[0, 0, 0, 0, 0, 0, 0, 0, 0]
z_draw=[0, 0, 0, 0, 0, 0, 0, 0, 0]


#Initail Condition Matrix
T1_to_0, T2_to_1, T3_to_2, T4_to_3, T5_to_4, T6_to_5, T7_to_6, T8_to_7 = [], [], [], [], [], [], [] ,[]
reset_arms()
relative_matrix_ls=[T1_to_0,T2_to_1,T3_to_2,T4_to_3,T5_to_4,T6_to_5,T7_to_6,T8_to_7]


#Initiate the arm
matrix_to_0 = matrix_calculation(1,relative_matrix_ls,True)
x_draw,y_draw,z_draw=refresh_all_points(x_draw,y_draw,z_draw,matrix_to_0,)
draw_robot(x_draw, y_draw,z_draw)


#<----- Run Simulation----->
while True:
    #Draw Robot Arm
    draw_robot(x_draw, y_draw,z_draw)
    
    #Enter Joint 1 
    print "<< Acceptable Parameter:-180<=Theta<=180 >>"
    print "<< Enter 'reset' to Reset the Arm >>"
    angle_1 = raw_input('Please Enter the Angle of Joint 1 or Enter "end" to turn off the arm:')
    if angle_1 == 'end' or not angle_1:
       print "Thanks for Using"
       break
    elif angle_1 == 'reset':
        reset_arms()
        relative_matrix_ls=[T1_to_0,T2_to_1,T3_to_2,T4_to_3,T5_to_4,T6_to_5,T7_to_6,T8_to_7]
        matrix_to_0 = matrix_calculation(1,relative_matrix_ls,True)
        x_draw,y_draw,z_draw=refresh_all_points(x_draw,y_draw,z_draw,matrix_to_0,)
    elif -180<=float(angle_1)<=180:
        T1_to_0 = forward_kinematics_matrix(0,0,5,float(angle_1))
        relative_matrix_ls[0] = T1_to_0 
        matrix_to_0=matrix_calculation(1,relative_matrix_ls)
        x_draw,y_draw,z_draw=refresh_all_points(x_draw,y_draw,z_draw,matrix_to_0)
    else:
        print "Please Enter Proper Value"
        break
    #Draw Robot Arm
    draw_robot(x_draw, y_draw,z_draw)
    

    #Enter Joint 2
    print "<< Acceptable Parameter:-90<=Theta<=90 >>"
    print "<< Enter 'reset' to Reset the Arm >>"
    angle_2 = raw_input('Please Enter the Angle of Joint 2 or Enter "end" to turn off the arm:')
    if angle_2 == 'end' or not angle_2:
       print "Thanks for Using"
       break
    elif angle_2 == 'reset':
        reset_arms()
        relative_matrix_ls=[T1_to_0,T2_to_1,T3_to_2,T4_to_3,T5_to_4,T6_to_5,T7_to_6,T8_to_7]
        matrix_to_0 = matrix_calculation(1,relative_matrix_ls,True)
        x_draw,y_draw,z_draw=refresh_all_points(x_draw,y_draw,z_draw,matrix_to_0,)
    elif -90<=float(angle_2)<=90:
        T2_to_1 = forward_kinematics_matrix(0,-90,5,float(angle_2)+90)
        relative_matrix_ls[1] = T2_to_1 
        matrix_to_0=matrix_calculation(2,relative_matrix_ls)
        x_draw,y_draw,z_draw=refresh_all_points(x_draw,y_draw,z_draw,matrix_to_0)
    else:
        print "Please Enter Proper Value"
        break
    #Draw Robot Arm
    draw_robot(x_draw, y_draw,z_draw) 
    

    #Enter Joint 3
    print "<< Acceptable Parameter:1<=Distance<=3 >>"
    print "<< Enter 'reset' to Reset the Arm >>"
    distance_3 = raw_input('Please Enter the Distance of Prisimatic Joint 3 or Enter "end" to turn off the arm:')
    if distance_3 == 'end' or not distance_3:
       print "Thanks for Using"
       break
    elif distance_3 == 'reset':
        reset_arms()
        relative_matrix_ls=[T1_to_0,T2_to_1,T3_to_2,T4_to_3,T5_to_4,T6_to_5,T7_to_6,T8_to_7]
        matrix_to_0 = matrix_calculation(1,relative_matrix_ls,True)
        x_draw,y_draw,z_draw=refresh_all_points(x_draw,y_draw,z_draw,matrix_to_0,)
    elif 1<=float(distance_3)<=3:
        T3_to_2 = forward_kinematics_matrix(0,+90,float(distance_3)+5,0)
        relative_matrix_ls[2] = T3_to_2 
        matrix_to_0=matrix_calculation(3,relative_matrix_ls)
        x_draw,y_draw,z_draw=refresh_all_points(x_draw,y_draw,z_draw,matrix_to_0)
    else:
        print "Please Enter Proper Value"
        break
    #Draw Robot Arm
    draw_robot(x_draw, y_draw,z_draw) 
    

    #Enter Joint 4
    print "<< Acceptable Parameter:-180<=Theta<=180 >>"
    print "<< Enter 'reset' to Reset the Arm >>"
    angle_4 = raw_input('Please Enter the Angle of Joint 4 or Enter "end" to turn off the arm:')
    if angle_4 == 'end' or not angle_4:
       print "Thanks for Using"
       break
    elif angle_4 == 'reset':
        reset_arms()
        relative_matrix_ls=[T1_to_0,T2_to_1,T3_to_2,T4_to_3,T5_to_4,T6_to_5,T7_to_6,T8_to_7]
        matrix_to_0 = matrix_calculation(1,relative_matrix_ls,True)
        x_draw,y_draw,z_draw=refresh_all_points(x_draw,y_draw,z_draw,matrix_to_0,)
    elif -180<=float(angle_4)<=180:
        T4_to_3 = forward_kinematics_matrix(0,0,0,float(angle_4))
        relative_matrix_ls[3] = T4_to_3 
        matrix_to_0=matrix_calculation(4,relative_matrix_ls)
        x_draw,y_draw,z_draw=refresh_all_points(x_draw,y_draw,z_draw,matrix_to_0)
    else:
        print "Please Enter Proper Value"
        break 
    #Draw Robot Arm
    draw_robot(x_draw, y_draw,z_draw)


    #Enter Joint 5
    print "<< Acceptable Parameter:-25<=Theta<=25 >>"
    print "<< Enter 'reset' to Reset the Arm >>"
    angle_5 = raw_input('Please Enter the Angle of Joint 5 or Enter "end" to turn off the arm:')
    if angle_5 == 'end' or not angle_5:
       print "Thanks for Using"
       break
    elif angle_5 == 'reset':
        reset_arms()
        relative_matrix_ls=[T1_to_0,T2_to_1,T3_to_2,T4_to_3,T5_to_4,T6_to_5,T7_to_6,T8_to_7]
        matrix_to_0 = matrix_calculation(1,relative_matrix_ls,True)
        x_draw,y_draw,z_draw=refresh_all_points(x_draw,y_draw,z_draw,matrix_to_0,)
    elif -25<=float(angle_5)<=25:
        T5_to_4 = forward_kinematics_matrix(0,-90,0,float(angle_5)+90)
        relative_matrix_ls[4] = T5_to_4 
        matrix_to_0=matrix_calculation(5,relative_matrix_ls)
        x_draw,y_draw,z_draw=refresh_all_points(x_draw,y_draw,z_draw,matrix_to_0)
    else:
        print "Please Enter Proper Value"
        break 
    #Draw Robot Arm
    draw_robot(x_draw, y_draw,z_draw)


    #Enter Joint 6
    print "<< Acceptable Parameter:-180<=Theta<=180 >>"
    print "<< Enter 'reset' to Reset the Arm >>"
    angle_6 = raw_input('Please Enter the Angle of Joint 6 or Enter "end" to turn off the arm:')
    if angle_6 == 'end' or not angle_6:
       print "Thanks for Using"
       break
    elif angle_6 == 'reset':
        reset_arms()
        relative_matrix_ls=[T1_to_0,T2_to_1,T3_to_2,T4_to_3,T5_to_4,T6_to_5,T7_to_6,T8_to_7]
        matrix_to_0 = matrix_calculation(1,relative_matrix_ls,True)
        x_draw,y_draw,z_draw=refresh_all_points(x_draw,y_draw,z_draw,matrix_to_0,)
    elif -180<=float(angle_6)<=180:
        T6_to_5 = forward_kinematics_matrix(0,+90,0,float(angle_6))
        relative_matrix_ls[5] = T6_to_5 
        matrix_to_0=matrix_calculation(6,relative_matrix_ls)
        x_draw,y_draw,z_draw=refresh_all_points(x_draw,y_draw,z_draw,matrix_to_0)
    else:
        print "Please Enter Proper Value"
        break 
