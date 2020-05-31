import rospy
import tf
from get_model_gazebo_pose import GazeboModel
import time
from pdb import set_trace
import pandas as pd

x=[]
y=[]

def publisher_of_tf():
    
    rospy.init_node('publisher_of_tf_node', anonymous=True)
    robot_name_list = ["dahei"]
    gazebo_model_object = GazeboModel(robot_name_list)
    
    for robot_name in robot_name_list:
        pose_now = gazebo_model_object.get_model_pose(robot_name)
    
    # Leave time enough to be sure the Gazebo Model data is initalised
    time.sleep(1)
    rospy.loginfo("Ready..Starting to record now...")
    
    rate = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        for robot_name in robot_name_list:
            pose_now = gazebo_model_object.get_model_pose(robot_name)
            if not pose_now:
                print("The Pose is not yet"+str(robot_name)+" available...Please try again later")
            else:
                x.append(round(pose_now.position.x,3))
                y.append(round(pose_now.position.y,3))
                print("recording")  
        rate.sleep()



if __name__ == '__main__':
    try:
        publisher_of_tf()
    except rospy.ROSInterruptException:
        print("hallo")  
        dataframe = pd.DataFrame({'x':x,'y':y})
        dataframe.to_csv("/home/jieming/simdata/pureCAM_Remote.csv",index=False,sep=',') 
        pass
