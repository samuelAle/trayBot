#!/usr/bin/env python

import roslib
import time
import rospy
import actionlib
import tf
import math
import random
import geometry_msgs.msg
import os, sys
from geometry_msgs.msg import *
from std_msgs.msg import *
from visualization_msgs.msg import *
from move_base_msgs.msg import *
from people_msgs.msg import *
from tf.transformations import *
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient


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
maxReliability = .8
personCount = 6
goingHome = False
atHome = True
home_subscriber = None
is_stuck = False
f = open("/home/turtlebot/scale/scaleData.txt")
transform = None
stuck_counter = 0
stuckDistance = 0
stuckTime = time.time()
soundHandler = None
jokes = []
isTwisting = False

# Method first called to create all regional markers for
# debugging purposes
def createMarkers():
    
    _createMarker(.5, 1.25, 0,0,3,0,True)
    _createMarker(-1.25, -1.1, 3,0,3,0,True)
    _createMarker(-3, 1.5, 4,0,3,0,True)
    _createMarker(-3, -1, 5,0,3,0,True)
    _createMarker(-1.25, 1.35, 2,0,3,0,True)
    _createMarker(.5, -1.25, 1,0,3,0,True)
    """
    ###############################################
    #Demo Markers
    ###############################################
    _createMarker(0.5,0,0,0,1.6,0,True)
    _createMarker(-1.15,1.3,1,0,1.6,0,True)
    _createMarker(-3,0.3,2,0,1.6,0,True)
    _createMarker(-1.4,-1,3,0,1.6,0,True)
    """
    print "published marker"

def _createMarker(x, y, id,lifetime,scale,red,shouldSleep):
    #print "Creating marker and publisher"
    # create a grey box marker 
    marker_publisher = rospy.Publisher('visualization_marker', Marker)
    marker = Marker(type=Marker.SPHERE, id=id,
                pose=Pose(Point(x, y, 0), Quaternion(0, 0, 0, 1)),
                scale=Vector3(scale,scale,scale),
                header=Header(frame_id='map',stamp = rospy.Time.now()),
                lifetime = rospy.Duration(lifetime),
                color=ColorRGBA(red, 0.6, 0.3, 0.6))
    markers.append(marker)
    if shouldSleep:
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
    global stuckTime
    global soundHandler
    stuckTime = time.time()
    goal.target_pose.header.frame_id = frameid
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = pose
    client.wait_for_server()
    client.send_goal(goal)
    robot_pose_subscriber = rospy.Subscriber("/robot_pose", Pose, waitForResult) 
    if not reachedRegion and not goingHome:
        # soundHandler.say("Scanning for people")
        print "Starting drive by scan"
        # state going to a region
        global human_pose_subscriber
        global maxReliability
        global humanPosition
        maxReliability = 0.8
        humanPosition = None
        human_pose_subscriber = rospy.Subscriber("/people_tracker_measurements", PositionMeasurementArray, driveByScan)
 
# Method for populating data on a human position
def driveByScan(data):
    #print "scanning on drive"
    global maxReliability
    global movingToPerson
    global reachedRegion
    global human_pose_subscriber
    global robot_post_subscriber
    global stuck_counter
    global client
    global humanPosition
    global movingToRegion
    global goingHome
    if goingHome:
        return
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
        _createMarker(pose.position.x, pose.position.y, personCount, 2,.5,1,False)
        personCount += 1
        if distance < 1.75: #TODO: Make sure jokes are good.
             
            maxReliability = person.reliability
            humanPosition = pose
            print "found this person's reliability: ", person.reliability
            
            client.cancel_goal()        
            robot_pose_subscriber.unregister()
            human_pose_subscriber.unregister()
            stuck_counter = 0
            updateTransform()
     
           
            print "humanPosition: ", humanPosition
            movingToRegion = False
            reachedRegion = True
            break

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
    global reachedRegion
    global atHome
    global goingHome
    global movingToPerson
    global stuckDistance
    global stuckTime
    global robot_pose_subscriber
    global humanPosition
    currentPosition = data.position
    global human_pose_subscriber
    global index
    goalPosition = goal.target_pose.pose.position
    distance = calculateDistance(currentPosition,goalPosition) 
    global client
    global soundHandler
    global jokes
    # Check if robot is stuck, looking at time passed w/o moving
    now = time.time()
    stuckDifference = abs(distance - stuckDistance)
    stuck = False
    if stuckDifference > 0.02:
        stuckDistance = distance
        stuckTime = time.time()
    else:
        if now - stuckTime > 15:
            # Robot is stuck
            print "haven't moved in 15 seconds, stuck :("
            stuck = True
    print "distance: " , distance 
    if client.get_state() > 3 or stuck:
        print "I'm stuck like a dumbass"
        if not movingToPerson: 
            index = (index + 1) % 6
        robot_pose_subscriber.unregister()
        movingToPerson = False
        reachedRegion = False
        if stuck_counter >= 1 or goingHome:
            robot_pose_subscriber.unregister()
            human_pose_subscriber.unregister()
            atHome = True
            movingToRegion = False
            goingHome = False
            client.cancel_goal()
            humanPosition = None
            currentTime = time.time()
            r = rospy.Publisher('/is_stuck', String)
            str = "true"
            r.publish(str)
            while time.time() - currentTime < 0.5:
                continue
            r.publish(str)
            print "published stuck, changing to home state"
            soundHandler.say("I'm stuck, come help me human")
            stuck_counter = 0
        else:
            print "sending to new region, incrementing stuck count ", stuck_counter
            movingToRegion = True
            goingHome = False
            atHome = False
            soundHandler.say("Please move aside, I am stuck")
            rospy.sleep(2)
            minDistance = 0.5
            stuckDistance = distance
            robot_pose_subscriber.unregister()
            _travelHere(markers[index].pose, 'map')
            stuck_counter += 1
            return
    if (distance < minDistance):
        # Robot has reached it's goal (under our rules of distance)
        global home_subscriber
        client.cancel_goal()        
        robot_pose_subscriber.unregister()
        human_pose_subscriber.unregister()
        stuck_counter = 0
        updateTransform()
        # Set booleans depending on what state robot is in
        if goingHome: #Todo: Add listener to web interface to listen for "Send" command, change atHome to Flase when clicked
            soundHandler.say("I need to be refilled")
            rospy.sleep(2)
            print "reached home"
            atHome = True #Set when we have listener
            r = rospy.Publisher('/is_home', String)
            str = "true"
            currentTime = time.time()
            r.publish(str)
            while time.time() - currentTime < 0.5:
                continue
            r.publish(str)
            goingHome = False	
        elif reachedRegion:
            soundHandler.say("Please help yourself while I tell you a joke")
            # Person finding state
            print "person reached"
            print "waiting"
            rospy.sleep(4)
            print "Done waiting, start scale measurements"
            joke = jokes.pop(0)
            soundHandler.say(joke[0])
            getScaleData(joke[1])
            jokes.append(joke)
            if not goingHome:
                print "now going to region finding state"
                rospy.sleep(3)
                soundHandler.say("ha ha ha, ha ha ha, ha ha ha, ha ha ha, ha ha ha")
                movingToPerson = False
                reachedRegion = False
        else:
            global isTwisting
            # Region finding state
            index = (index + 1) % 6
            print "region reached, drive-by complete"
            if humanPosition is not None:
                print "humanPosition: ", humanPosition
            """ 
            isTwisting = True
            r = rospy.Publisher('/mobile_base/commands/velocity', Twist)
            twist = Twist()
            if random.randint(0,1) == 0:
                twist.angular.z = .7
            else:
                twist.angular.z = -.7
            
            curTime = time.time()
            print "beginning twisting"
            while time.time() - curTime < 3:
                r.publish(twist)
            print "Done twisting"
            rospy.sleep(2)
            isTwisting = False
            """
            if not goingHome:
                movingToRegion = False
                reachedRegion = True

def waitForSend(data):
    global home_subscriber
    global atHome
    global soundHandler
    print "Operator pressed sendTray button", data
    if data.data == "false":
        print "Sending robot out"
        soundHandler.say("Beginning to serve")
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
    global soundHandler
    if humanPosition is not None:
        # Person has been found
        soundHandler.say("Going to a person now")
        print "I Found a person, now traveling there"
        print "Before position is: ", humanPosition
        if humanPosition.position.x > goal.target_pose.pose.position.x:
            humanPosition.position.x -= 0.45
        else:
            humanPosition.position.x += 0.45
        if humanPosition.position.y > goal.target_pose.pose.position.y:
            humanPosition.position.y -= 0.45
        else:
            humanPosition.position.y += 0.45
        personCount += 1
        print "After position is: ", humanPosition
        _createMarker(humanPosition.position.x, humanPosition.position.y, personCount, 5,.5,.1,True)
        movingToPerson = True
        _travelHere(humanPosition, 'map')
    else:
        global reachedRegion
        movingToPerson = False
        reachedRegion = False

#Assumes same frame_id
def calculateDistance(pose1,pose2):
    return ((pose1.x - pose2.x)**2 + (pose1.y - pose2.y)**2)**.5

def getScaleData(joke):
    global soundHandler
    start = time.time() 
    said = False
    incrementer = 0
    line = f.readline()
    previousLine = line
    while line != '' or previousLine == '':
        previousLine = line
        line = f.readline() 
    firstReading = float(previousLine)
    if firstReading <= 12:  #Can change to another threshold
        rospy.sleep(3)
        soundHandler.say(joke)
        goHome()
        return
    r = rospy.Rate(5)
    while incrementer < 3: 
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
#TODO: Make sure jokes are good
    soundHandler.say(joke)
    if firstReading <= 0.5:
        goHome()
    print "Done Reading Scale"
    return said
    
    
   
def goHome():
    global soundHandler
    global goingHome
    global movingToPerson
    global reachedRegion
    global movingToRegion
    global minDistance
    goingHome = True
    movingToPerson = False
    reachedRegion = False
    movingToRegion = False
    minDistance = 0.5
    rospy.sleep(3)
    soundHandler.say("Actually, I need to go home, excuse me party people")
    rospy.sleep(3)
    print "going home"
    _travelHere(markers[3].pose,'map')

def initializeJokes():
    global jokes
    jokes.append(("What is Bruce Lees favorite drink", "Wataaaaah"))
    jokes.append(("How does NASA organize their company parties", "they plan it"))
    jokes.append(("time flies like an arrow", "fruit flies like banana"))
    jokes.append(("why did the lifeguard not save the hippie", "because he was too far out man"))
    jokes.append(("i was wondering why the baseball was getting bigger", "then it hit me"))
    jokes.append(("How did the hipster burn his tongue", "he drank his coffee before it was cool"))
    jokes.append(("A sequel queer E goes in to a bar walks up to two tables an asks", "can I join you"))
    jokes.append(("how many programmers does it take to change a lightbulb", "none, that's a hardware problem"))


if __name__=="__main__":    
    print "in main"
    rospy.init_node("regions", anonymous=True)
    soundHandler = SoundClient()
    rospy.sleep(1)
    soundHandler.stopAll()
    createMarkers()
    initializeJokes()
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
        if not (reachedRegion or movingToRegion or isTwisting):
            # begin moving to a region with distance threshold of 0.5 meters
            movingToRegion = True
            minDistance = 0.5
            print "Traveling to region at index", index
            _travelHere(markers[index].pose, 'map')

        elif not movingToPerson and reachedRegion:
            # begin moving to a person with distance threshold of 0.3 meters
            movingToPerson = True
            minDistance = 0.25
            scanTime = time.time()
            # Used if we want to scan once region has been reached :)
            goToPerson()    
        r.sleep() 
    rospy.spin()

