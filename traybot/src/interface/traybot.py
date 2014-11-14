#!/usr/bin/env python

import roslib
import rospy
import actionlib
from geometry_msgs.msg import *
from std_msgs.msg import *
from visualization_msgs.msg import *
from move_base_msgs.msg import *
index = 0
markers = []
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
goal = MoveBaseGoal()
subscriber = None 
done = True

def createMarkers():
    createMarker(.5, 1.25, 0)
    createMarker(.5, -1.25, 1)
    createMarker(-1.25, 1.35, 2)
    createMarker(-1.25, -1.1, 3)
    createMarker(-3, 1.5, 4)
    createMarker(-3, -1, 5)
    print "published marker"

def createMarker(x, y, id):
    print "Creating marker and publisher"
    # create a grey box marker
    marker_publisher = rospy.Publisher('visualization_marker', Marker)
    marker = Marker(type=Marker.SPHERE, id=id,
                pose=Pose(Point(x, y, 0), Quaternion(0, 0, 0, 1)),
                scale=Vector3(1.6, 1.6, 1.6),
                header=Header(frame_id='map'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.6))
    markers.append(marker)
    rospy.sleep(.5)
    marker_publisher.publish(marker) 

def travelToRegion():
    global done
    if done == False:
        return
    global client
    global goal
    global subscriber
    global index
    print "index: ", index
    done = False
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = markers[index].pose
    print "client", client
    print "goal", goal
    client.wait_for_server()
    client.send_goal(goal)
    subscriber = rospy.Subscriber("/robot_pose", Pose, waitForResult)
    print "done with first region"
    # send to goal with this position
    # increment index
    # listen when goal is completed & when:
        # Run travelToRegion again

def waitForResult(data):
    currentPosition = data.position
    goalPosition = goal.target_pose.pose.position
    distance = ((currentPosition.x - goalPosition.x)**2 + (currentPosition.y - goalPosition.y)**2)**.5  
    print "distance = ", distance
    if (distance < 0.5):
        global index
        global done
        index = (index + 1) % 6
        print "reached region"
        client.cancel_goal()        
        subscriber.unregister()
        done = True

if __name__=="__main__":
    
    rospy.init_node("regions", anonymous=True)
    createMarkers()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        travelToRegion()
        r.sleep() 
    rospy.spin()

