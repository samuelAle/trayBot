#!/usr/bin/env python

import roslib
import time
import rospy
import actionlib
import tf
import math
import geometry_msgs.msg
from geometry_msgs.msg import *
from std_msgs.msg import *
from visualization_msgs.msg import *
from move_base_msgs.msg import *
from people_msgs.msg import *
from tf.transformations import *


# Global fields, the large amount of them due to lack of
# ability to put parameters in a callback function
print "in fields"
index = 0
markers = []
client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
goal = MoveBaseGoal()
robot_pose_subscriber = None 
leg_detector_subscriber = None
movingToRegion = False
movingToPerson = False
reachedRegion = False
minDistance = 0.5
scanTime = -1
human_pose_subscriber = None
humanPosition = None
maxReliability = .7
personCount = 6
goingHome = False
atHome = True
home_subscriber = None
is_stuck = False
f = open("/home/turtlebot/scale/scaleData.txt")
transform = None
# Method first called to create all regional markers for
# debugging purposes
def createMarkers():
    _createMarker(.5, 1.25, 0,0,1.6,0)
    _createMarker(.5, -1.25, 1,0,1.6,0)
    _createMarker(-1.25, 1.35, 2,0,1.6,0)
    _createMarker(-1.25, -1.1, 3,0,1.6,0)
    _createMarker(-3, 1.5, 4,0,1.6,0)
    _createMarker(-3, -1, 5,0,1.6,0)
    print "published marker"

def _createMarker(x, y, id,lifetime,scale,red):
    #print "Creating marker and publisher"
    # create a grey box marker
    marker_publisher = rospy.Publisher('visualization_marker', Marker)
    marker = Marker(type=Marker.SPHERE, id=id,
                pose=Pose(Point(x, y, 0), Quaternion(0, 0, 0, 1)),
                scale=Vector3(scale,scale,scale),
                header=Header(frame_id='map'),
                lifetime = rospy.Duration(lifetime),
                color=ColorRGBA(red, 0.6, 0.3, 0.6))
    markers.append(marker)
    rospy.sleep(.5)
    marker_publisher.publish(marker) 

"""
Method used for traveling to both regions and people
@param pose: the pose to travel to
@param frameid: the frame of reference that the goal is in terms of
"""
def _travelHere(pose,frameid):
    global goal
    global robot_pose_subscriber
    global client
    goal.target_pose.header.frame_id = frameid
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = pose
    client.wait_for_server()
    client.send_goal(goal)
    robot_pose_subscriber = rospy.Subscriber("/robot_pose", Pose, waitForResult) 
    if not reachedRegion and not goingHome:
        print "Starting drive by scan"
        # state going to a region
        global human_pose_subscriber
        global maxReliability
        global humanPosition
        maxReliability = 0.5
        humanPosition = None
        human_pose_subscriber = rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, driveByScan)
 
# Method for populating data on a human position
def driveByScan(data):
    #print "scanning on drive"
    global maxReliability
    global movingToPerson
    global reachedRegion
    for person in data.people:
        if movingToPerson or reachedRegion:
            return
        if person.reliability < maxReliability:
            continue
        global personCount    
        pos = person.pos
        pose = Pose(Point(pos.x, pos.y, pos.z),Quaternion(0, 0, 0, 1))
        transformCoordinates(pose)
        distance = calculateDistance(goal.target_pose.pose.position, pose.position)
        #print "distance: ", distance
        _createMarker(pose.position.x, pose.position.y, personCount, 2,.5,1)
        personCount += 1
        if distance < 2:
            global humanPosition
            maxReliability = person.reliability
            humanPosition = pose
            print "found this person's reliability: ", person.reliability

# Callback while traveling to a goal.  Listens until goal
# has reached it's minDistance away
def waitForResult(data):
    global minDistance
    global goal
    global movingToRegion
    global stuck_counter
    # Robot's position data depends on if we are looking for a human or region
    # Human topic for position data: /robot_pose_ekf/odom_combined
    # Region topic for position data: /robot_pose
    #currentPosition = None
    global reachedRegion
    global atHome
    global goingHome
    global movingToPerson
    currentPosition = data.position
    goalPosition = goal.target_pose.pose.position
    distance = calculateDistance(currentPosition,goalPosition) 
    global client
    print "distance: " , distance 
    if client.get_state() > 3:
        print "I'm stuck like a dumbass" 
        if stuck_counter >= 1:
            currentTime = time.time()
            r = rospy.Publisher('/is_stuck', String)
            str = "true"
            r.publish(str)
            while time.time() - currentTime < 0.5:
                continue
            r.publish(str)
            print "published stuck, changing to home state"
            atHome = True
            movingToRegion = False
            goingHome = False
            movingToPerson = False
            reachedRegion = False
            stuck_counter = 0
        else: 
            print "retrying, incrementing stuck count ", stuck_counter
            client.send_goal(goal)
            stuck_counter += 1

   # print "distance = ", distance, " ********** goal = (", goalPosition.x, ",", goalPosition.y, ",", goalPosition.z, ") ********* position = ", currentPosition.x, ",", currentPosition.y, ",", currentPosition.z, ")"
    if (distance < minDistance):
        # Robot has reached it's goal (under our rules of distance)
        global index
        global robot_pose_subscriber
        global reachedRegion
        global goingHome
        global home_subscriber
        global human_pose_subscriber
        global atHome
        client.cancel_goal()        
        robot_pose_subscriber.unregister()
        human_pose_subscriber.unregister()
        stuck_counter = 0
        updateTransform()
        # Set booleans depending on what state robot is in
        if goingHome: #Todo: Add listener to web interface to listen for "Send" command, change atHome to Flase when clicked
            print "reached home"
            atHome = True #Set when we have listener
            r = rospy.Publisher('/is_home', String)
            str = "true"
            currentTime = time.time()
            r.publish(str)
            while time.time() - currentTime < 0.5:
                continue
            r.publish(str)
            #home_subscriber = rospy.Subscriber("/is_home", String, waitForSend)
            goingHome = False	
        elif reachedRegion:
            # Person finding state
            global movingToPerson
            print "person reached"
            print "waiting"
            curTime = time.time()
            while time.time() - curTime < 5:
                # Waiting
                continue
            print "Done waiting, start scale measurements"
            getScaleData()
            if not goingHome:
                movingToPerson = False
                reachedRegion = False
        else:
            # Region finding state
            global index
            index = (index + 1) % 6
            print "region reached, drive-by complete"
            global humanPosition
            print "checking scale data"
            getScaleData()
            if humanPosition is not None:
                print "humanPosition: ", humanPosition
            if not goingHome:
                movingToRegion = False
                reachedRegion = True

def waitForSend(data):
    global home_subscriber
    global atHome
    print "Operator pressed sendTray button", data
    if data.data == "false":
        print "Sending robot out"
        atHome = False
        home_subscriber.unregister()
        home_subscriber = None

def updateTransform():
    global transform
    rate = rospy.Rate(1)
    listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            transform = listener.lookupTransform('map', 'odom', rospy.Time(0))
            #print "transformation is: ", trans
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

def transformCoordinates(pose):
    global transform
    #print "Started Transform!!" 
        #print "pose before transformation: ", pose
    #pose.position.x -= trans[0]
    #pose.position.y -= trans[1]
    #print "pose after transformation: ", pose
    euler = euler_from_quaternion(transform[1])
    #print "rotation: ", rot
    #print "roll: ", euler[0], "pitch: ", euler[1], "yaw: ", euler[2]
    newX = pose.position.x * math.cos(euler[2]) - pose.position.y * math.sin(euler[2])
    newY = pose.position.x * math.sin(euler[2]) + pose.position.y * math.cos(euler[2])
    pose.position.x = newX + transform[0][0]
    pose.position.y = newY + transform[0][1]
    #print "new x: ", newX , " new y: ", newY
    #print "pose after transformation: ", pose
    #print "Ended Transform!!"

def goToPerson():
    global humanPosition
    global movingToPerson
    global personCount
    global goal
    if humanPosition is not None:
        # Person has been found
        print "I Found a person, now traveling there"
        print "Before position is: ", humanPosition
        if humanPosition.position.x > goal.target_pose.pose.x:
            humanPosition.position.x -= 0.4
        else:
            humanPosition.position.x += 0.4
        if humanPosition.position.y > goal.target_pose.pose.y:
            humanPosition.position.y -= 0.4
        else:
            humanPosition.position.y += 0.4
        personCount += 1
        print "After position is: ", humanPosition
        _createMarker(humanPosition.position.x, humanPosition.position.y, personCount, 5,.5,.1)
        movingToPerson = True
        _travelHere(humanPosition, 'map')
    else:
        global reachedRegion
        movingToPerson = False
        reachedRegion = False

# Callback for subscription of current people positions
def scanForPeople(data):
    global scanTime
    global humanPosition
    curTime = time.time()
#   maxReliability = 0.7
#   pos = None
#   for person in data.people:    
#       if person.reliability > maxReliability:
#           # Done searching for people (new state)
#           maxReliability = person.reliability
#           pos = person.pos
    # 3 states:
    # 1) person has been found, begin travel
    # 2) Scan has gone on too long, move to new region 
    # 3) Person has not been found, update counter
    if humanPosition is not None:
        # Person has been found
        global leg_detector_suscriber
        global minDistance
        global notFoundCount
        leg_detector_subscriber.unregister()
        scanTime = -1
        print "Found a person, now traveling there"
        _travelHere(humanPosition, 'map')
    else:
        global reachedRegion
        global movingToPerson
        movingToPerson = False
        reachedRegion = False
#   elif (curTime - scanTime) > 10:
#       # At this point a human has not been found 10 seconds
#       # It is not time to continue to a new region
#       global leg_detector_suscriber
#       global movingToPerson
#       global reachedRegion
#       global index
#       scanTime = -1
#       leg_detector_subscriber.unregister()
#       movingToPerson = False
#       reachedRegion = False
#       print "no person was found in this region, going to next"


#Assumes same frame_id
def calculateDistance(pose1,pose2):
    return ((pose1.x - pose2.x)**2 + (pose1.y - pose2.y)**2)**.5

def getScaleData():
    incrementer = 0
    line = f.readline()
    previousLine = line
    while line != '' or previousLine == '':
        previousLine = line
        line = f.readline() 
    firstReading = float(previousLine)
    if firstReading <= 0.5:  #Can change to another threshold
        goHome()
        return
    r = rospy.Rate(5)
    while incrementer < 4: 
        reading = f.readline()
        #print "reading: ", reading
        if not reading == '':
            reading = float(reading)
            if abs(reading - firstReading) <= 0.5: #added threshold weight
                incrementer += 1
            else:
                firstReading = reading
                incrementer = 0
            print "weight, ", reading 
        r.sleep()
    if firstReading <= 0.5:
        goHome()
    
    
   
def goHome():
    #ToDo
    global goingHome
    global movingToPerson
    global reachedRegion
    global movingToRegion
    goingHome = True
    movingToPerson = False
    reachedRegion = False
    movingToRegion = False
    print "going home"
    _travelHere(markers[0].pose,'map')

if __name__=="__main__":    
    print "in main"
    rospy.init_node("regions", anonymous=True)
    createMarkers()
    updateTransform()
    # Rate is created in Hz for sleeping
    # TODO: update rate to a good value
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        if goingHome or atHome:
            if home_subscriber == None:
                print "Listening on is_home"
                home_subscriber = rospy.Subscriber("/is_home", String, waitForSend)
            continue
        if not (reachedRegion or movingToRegion):
            # begin moving to a region with distance threshold of 0.5 meters
            movingToRegion = True
            minDistance = 0.5
            print "Traveling to region at index", index
            _travelHere(markers[index].pose, 'map')

        elif not movingToPerson and reachedRegion:
            # begin moving to a person with distance threshold of 0.3 meters
            movingToPerson = True
            minDistance = 0.5
            scanTime = time.time()
            # Used if we want to scan once region has been reached :)
            # leg_detector_subscriber = rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, scanForPeople)
            goToPerson()    
        r.sleep() 
    rospy.spin()

