#!/usr/bin/python3
import rospy
import math
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

x_goal,y_goal,x_init,y_init,theta,theta1,distance ,sub, sumn , sumb , sumx , sumy= 0,0,0,0,0,0,0,0,0,0,0,0

def traversal_logic(dir_index ,diro_index, dir,dir_count,count,y,leny,Goal):  #dir_index shold be under 0 and leny left_index and right_index can less than 0 or more leny resp.
                    index_temp = dir_index 

                    while(count<9):

                        if(y[index_temp]<0.2 ):
                            count = count+1
                            dir_index = dir_index +(2*dir-1)
                            dir_count = dir_count + 1
                            index_temp = dir_index
                            if(dir_index==dir*leny + (dir-1)):  #if(left_index == -1 or right_index == leny)
                              dir_index = (-dir+1)*leny + (dir-1) #left_index  = leny-1 right_index= 0
                              index_temp = dir_index
                            if( dir_index==diro_index or dir_index == Goal):
                              if(count==9):
                                   return dir_index,dir_count,count
                              else:
                                   count = 0
                                   return -1,dir_count,count
                            
                        else:
                            count = 0
                            dir_index = dir_index +(2*dir-1)
                            dir_count = dir_count + 1
                            index_temp = dir_index
                            if(dir_index == dir*leny + (dir-1)):
                                 dir_index = (-dir+1)*leny + (dir-1)
                                 index_temp = dir_index
                            return dir_index,dir_count,count

                    return dir_index,dir_count,count
                    

def traversal(count,left_count,right_count,left_index,right_index,y,leny,Goal):
            while(count<9):
                if(Goal<abs(int(len(y)/2))):
                    # if(left_count<=right_count ):
                         count = 0
                         dir = 0
                         left_index ,left_count ,count = traversal_logic(left_index ,right_index, dir,left_count,count,y,leny,Goal)
                         if(left_index==-1):
                              return -1
                         
                else:    
                    # elif(left_count>right_count ):
                         count = 0
                         dir = 1
                         right_index ,right_count ,count = traversal_logic(right_index ,left_index, dir,right_count,count,y,leny,Goal)
                         if(right_index == -1):
                              return -1
                    
            if(dir==0 and count==9):
                if(left_index>=leny-4):
                     return left_index-leny+4
                else:
                    return left_index+4

            elif(dir==1 and count==9):
                if(right_index<=4):
                     return leny-4+right_index
                else:
                    return right_index-4
            else:
                 return -1
        
def callback(data):
    global x_init, y_init, distance, x_goal, y_goal, theta, theta1, sumn , sumb , sumx , sumy
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

  
    distance = math.sqrt((x_goal - x_init)**2+(y_goal-y_init)**2)
    y , leny=min_range_index(data.ranges)
    z = Twist()
    count  , sum  = 0 ,0 
    if (theta1 < 0):
        theta1 = theta1+2*math.pi
    elif (theta1 > 2*math.pi):
        theta1 = theta1-2*math.pi
    index_temp = math.floor(theta1*180/(5*math.pi)) #n=5
    Goal , left_index , right_index = index_temp ,index_temp , index_temp
    left_count = index_temp - left_index #initially zero
    right_count = right_index - index_temp #initially zero
    for i in range (1,4):
        index_temp1 , index_temp2 = Goal,Goal
        if (index_temp1 -i) < 0:
            index_temp1 = leny + (index_temp1 - i) + i  # if index_temp - i = -1 then index_temp -i = leny + (index_temp -i)
        elif (index_temp2 +i) > leny-1:
            index_temp2 = (index_temp2+i) - leny -i
        sumn = sumn + y[index_temp1-i]
        sumb = sumb + y[index_temp2+i]
    if(y[Goal]==0 and sumn ==0 and sumb ==0):

        index_final = Goal
        sumn = 0 
        sumb = 0

    else:
        index_final = traversal(count,left_count,right_count,left_index,right_index,y,leny,Goal)
        sumn = 0
        sumb = 0

    print(Goal,index_final)
    theta_movement = index_final*(5*math.pi)/180  #theta_movement is between 0-2pi and 0 is at vertical
    #print(theta_movement)

    if (theta_movement <=math.pi):
        angular_velocity_goal_obs = theta_movement/math.pi
    else:
        angular_velocity_goal_obs = -(2*math.pi-theta_movement)/math.pi
    for i in range(0,5):
        sumx=sumx+y[i]
        sumy = sumy +y[71-i]
    sum = (sumx + sumy)/11 #jiska sum jyada waha pe obstacle hoga
    
    linear_velocity = (2 - sum)/2.9
    try:
        k = angular_velocity_goal_obs/abs(angular_velocity_goal_obs)
    except ZeroDivisionError as e:
        k = 1
    angular_velocity = k*(abs(angular_velocity_goal_obs)/(2)+sum)
    # angular_velocity = 0.7*k*(1- pow(2.73, -(abs(angular_velocity_goal_obs)*0.5+sum*0.8))) 
    print(linear_velocity,angular_velocity,sum)
    sum , sumx , sumy = 0,0,0

    z = Twist()
    z.linear.x = linear_velocity
    z.angular.z = angular_velocity

    if (linear_velocity>= 0.20):
        z.linear.x = 0.20
    if (abs(angular_velocity)>= 0.7):
        try:
            k = angular_velocity/abs(angular_velocity)
        except ZeroDivisionError as e:
            k = 1
        z.angular.z = k*0.7

    if(distance<1 ):
        z.linear.x = distance/4
        z.angular.z = angular_velocity_goal_obs*0.4
    if(z.linear.x<0):
         z.linear.x = 0.1
    # print(index_final,index_temp)  
    #print(index_temp,index_final,theta1,distance,theta_movement,z.linear.x,z.angular.z)  
    pub.publish(z)

def min_range_index(ranges):
    ranges = [x for x in ranges if not (math.isnan(x) or x == 0)]
    n=5
    y = [ranges[i:i + n] for i in range(0, len(ranges), n)]
    b=35
    for i in range(len(y)):
        for j in range(len(y[i])):
            if (abs(y[i][j])>3.5):
                y[i][j]=3.5
                y[i][j]=3.5-y[i][j]
            elif(abs(y[i][j])<0.5):
                 y[i][j]=5
            else:
                y[i][j]=3.5-y[i][j]
        y[i]=sum(y[i])
        if (y[i])<b:
            y[i]=y[i]/b
        else:
            y[i]=1
    #print(y,len(y))
    return(y , len(y))

def poseCallback1(msg):
    global x_init, y_init, distance, x_goal, y_goal, theta, theta1
    x_init=msg.pose.pose.position.x
    y_init=msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, theta) = euler_from_quaternion(orientation_list)
    theta1 = (math.atan2(y_goal-y_init, x_goal-x_init)) - (theta)
    distance = math.sqrt((x_goal - x_init)**2+(y_goal-y_init)**2)
    if (theta1 <0):   #theta1 is between 0 - 2pi 0 is at vertical
        theta1 = theta1+2*math.pi
    elif (theta1 > 2*math.pi):
        theta1 = theta1-2*math.pi

def main():
    global distance,  sub,  x_init, y_init, theta, x_goal, y_goal, theta1
    x_goal = float((input('Enter the goal x:')))
    y_goal = float((input('Enter the goal y:')))
    sub = rospy.Subscriber('odom', Odometry, poseCallback1)

if __name__ == '__main__':
    rospy.init_node('scan_node', anonymous=True)
    main()
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()