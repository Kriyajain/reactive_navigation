#!/usr/bin/env python3
import rospy, math
from sensor_msgs.msg import LaserScan,Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
x_goal,y_goal,x_init,y_init,theta,theta1 ,sub, sumn , sumb , sumx , sumy, distance= 0,0,0,0,0,0,0,0,0,0,0,0
alpha1=0
def traversal_logic(dir_index ,diro_index, dir,dir_count,count,y,leny,Goal):  #dir_index shold be under 0 and leny left_index and right_index can less than 0 or more leny resp.
                    index_temp = dir_index 

                    while(count<6):

                        if(y[index_temp]<0.2 ):
                            count = count+1
                            dir_index = dir_index +(2*dir-1)
                            dir_count = dir_count + 1
                            index_temp = dir_index
                            if(dir_index==dir*leny + (dir-1)):  #if(left_index == -1 or right_index == leny)
                              dir_index = (-dir+1)*leny + (dir-1) #left_index  = leny-1 right_index= 0
                              index_temp = dir_index
                            if( dir_index==diro_index or dir_index == Goal):
                              if(count==7):
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
            while(count<6):
                if(Goal<abs(int(len(y)/2))):
                         count = 0
                         dir = 0
                         left_index ,left_count ,count = traversal_logic(left_index ,right_index, dir,left_count,count,y,leny,Goal)
                         if(left_index==-1):
                              return -1
                         
                else:    
                         count = 0
                         dir = 1
                         right_index ,right_count ,count = traversal_logic(right_index ,left_index, dir,right_count,count,y,leny,Goal)
                         if(right_index == -1):
                              return -1
                    
            if(dir==0 and count==6):
                if(left_index>=leny-3):
                     return left_index-leny+3
                else:
                    return left_index+3

            elif(dir==1 and count==6):
                if(right_index<=3):
                     return leny-3+right_index
                else:
                    return right_index-3
            else:
                 return -1
def callback(data):
    global x_init, y_init, distance, x_goal, y_goal, theta, theta1, sumn , sumb , sumx , sumy,alpha1 
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    y , leny=min_range_index(data.ranges)
    alpha1=leny
    z = Twist()
    count  , sum  = 0 ,0 
    theta1 = -(math.atan2(y_goal-y_init, x_goal-x_init)) - (theta)
    if (theta1 < 0):# ek elif ko hatana h
        theta1 = theta1+2*math.pi
    elif (theta1 > 2*math.pi):
        theta1 = theta1-2*math.pi
    #print(theta1)
    index_temp = math.floor(theta1*(270/360)/5) #n=5 (math.pi*135/180)
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
    #print(Goal,index_final)
    theta_movement = (index_final*5)*(360/270)*(math.pi)/180  #theta_movement is between 0-2pi and 0 is at vertical
    #print(theta_movement)
    if (theta_movement <=math.pi):#isko chhota karna h #shart #PARTY NHI CHALEGI
        angular_velocity_goal_obs = theta_movement/math.pi
    else:
        angular_velocity_goal_obs = -(2*math.pi-theta_movement)/math.pi
    for i in range(1,5):
        #print(alpha1)
        sumx=sumx+y[i]
        sumy = sumy +y[alpha1-i]
    sum = (sumx + sumy)/11 #jiska sum jyada waha pe obstacle hoga
    linear_velocity = (2 - sum)/2.9
    try:
        k = angular_velocity_goal_obs/abs(angular_velocity_goal_obs)
    except ZeroDivisionError as e:
        k = 1
    angular_velocity = k*(abs(angular_velocity_goal_obs)/2+sum)
    # angular_velocity = 0.7*k*(1- pow(2.73, -(abs(angular_velocity_goal_obs)*0.5+sum*0.8))) 
    #print(linear_velocity,angular_velocity,sum)
    sum , sumx , sumy = 0,0,0

    z = Twist()
    z.linear.x = linear_velocity
    z.angular.z = angular_velocity

    if (linear_velocity>= 0.10):
        z.linear.x = 0.10
    if (abs(angular_velocity)>= 0.4):
            k = angular_velocity/abs(angular_velocity)
            z.angular.z = k*0.4
    if(distance<1 ):
        z.linear.x = distance/4
        z.angular.z = angular_velocity_goal_obs*0.4
    if(z.linear.x<0):
         z.linear.x = 0.1
    # print(index_final,index_temp)  
    #print(index_temp,index_final,theta1,distance,theta_movement,z.linear.x,z.angular.z)  
    z.linear.z = 0
    z.linear.y= 0
    z.linear.z=(1800*z.linear.x+750*z.angular.z)
    z.linear.y=(1800*z.linear.x-750*z.angular.z)
    #rospy.loginfo(z)
    # z.linear.y = Goal
    # z.linear.z = index_final
    pub.publish(z)
    #rospy.loginfo(z)

def min_range_index(ranges):
    global alpha1
    ranges = [x for x in ranges]
    for i in range(len(ranges)):
            if (ranges[i]>3.5 or ranges[i]==0):
                ranges[i]=3.5
    n=5
    y = [ranges[i:i + n] for i in range(0, len(ranges), n)]
    b=3.5*n
    for i in range(len(y)):
        for j in range(len(y[i])):
            if (abs(y[i][j])>3.5):
                y[i][j]=3.5
                y[i][j]=3.5-y[i][j]
            elif(abs(y[i][j])<0.3):
                 y[i][j]=3.5
            else:
                y[i][j]=3.5-y[i][j]
        y[i]=sum(y[i])
        if (y[i])<b:
            y[i]=y[i]/b
        else:
            y[i]=1
    #print(y,len(y))
    return(y , len(y))


def odometryCb(msg):
    global x_init, y_init,distance
    x_init=msg.position.y
    y_init=msg.position.z
    distance=msg.position.x
def odometryCb2(msg):
    global yaw
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y,
                        orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    yaw=yaw
    


def main():
    global distance,  sub,  x_init, y_init, theta, x_goal, y_goal, theta1
    x_goal = float((input('Enter the goal x:')))
    y_goal = float((input('Enter the goal y:')))
    rospy.Subscriber('distance', Pose, odometryCb)
    rospy.Subscriber('/android/imu',Imu,odometryCb2)


if __name__ == '__main__':
    rospy.init_node('scan_node', anonymous=True)
    
    main()
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()