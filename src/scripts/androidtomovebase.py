#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped



class AndroidConverter():

    def __init__(self):
        rospy.init_node("androidconverter", anonymous=True)
        self.nodename = rospy.get_name()
        self.seq = 0
        self.frame = "base_link"
        self.frames = ["map", "odom", "base_link", "laser"]


        
        
        rospy.loginfo("%s started" % self.nodename)

        self.pubgoal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.Subscriber("/androidgoal", Pose, self.poseCBPublish)
        rospy.Subscriber("/androidframe", Int16, self.setframe)

    def setframe(self, msg):
        self.frame = self.frames[msg.data]

    def poseCBPublish(self, msg):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.seq = self.seq
        pose.pose.position.x = msg.position.x
        pose.pose.position.y = msg.position.y
        pose.pose.position.z = msg.position.z
        pose.pose.orientation.x = msg.orientation.x
        pose.pose.orientation.y = msg.orientation.y
        pose.pose.orientation.z = msg.orientation.z
        pose.pose.orientation.w = msg.orientation.w
        pose.header.frame_id = self.frame
        self.pubgoal.publish(pose) 
        self.seq += 1
     
     
    def spin(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__':
    """ main """
    try:
        androidconverter = AndroidConverter()
        androidconverter.spin()
    except rospy.ROSInterruptException:
        pass



