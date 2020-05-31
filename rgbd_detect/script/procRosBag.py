import rospy
from sensor_msgs.msg import LaserScan
import copy
import math

pub = rospy.Publisher('scanPeo', LaserScan, queue_size=10)


def callback(rawData, scans):

    cData = copy.deepcopy(rawData)
    cData.ranges = list(cData.ranges)

    if (scans["old"]==0):
        scans["old"] = rawData
    else:
        for idx,cRange in enumerate(cData.ranges):
            preScan = scans["old"]
            if abs(preScan.ranges[idx] - cRange)<0.02:
                cData.ranges[idx] = 0
        
        recordList=[]
        for idx,cRange in enumerate(cData.ranges):
            if cRange>0.03:
                recordList.append((cRange,idx))
        
        last = 0
        lastIdx = 0
        segmentList=[]
        # segment and delete noise point
        for idx,cRange in enumerate(recordList):
            if last ==0:
                last = cData.ranges[idx]
                lastIdx = idx
                continue
            else:
                dist = pow(last,0) + pow(cRange,2) -2*last*cRange*math.cos((idx-lastIdx)*rawData.angle_increment)
                dist = math.sqrt(dist)
                if dist>0.10:
                    segmentList.append(idx)
                last = cData.ranges[idx]
                lastIdx = idx
        segmentList.append(len(recordList)-1)

        for i,idx in enumerate(segmentList, 1):
            if idx-segmentList[i-1]<3:
                cRange[]



        pub.publish(cData)

        scans["old"] = rawData



def listener():
    rospy.init_node('listen_rosbag', anonymous=False)

    scans={"old": 0}
    rospy.Subscriber("/scan", LaserScan, callback, scans)

    rospy.spin()

if __name__ == '__main__':
    listener()




    #rospy.loginfo("data length %s", len(data.ranges))
    #rospy.loginfo("angle min  %s", data.angle_min)
    #rospy.loginfo("angle max  %s", data.angle_max)
    #rospy.loginfo("angle incre  %s", data.angle_increment)