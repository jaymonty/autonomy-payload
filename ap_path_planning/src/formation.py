#!/usr/bin/env python

#-----------------------------------------------
# Formation
# Stefan Jorgensen
# 
#-----------------------------------------------

import sys
import rospy
#Append libraries directory to path:
#from ap_lib.dubins import *
# Keeps track of agents in the networ
from ap_perception.swarm_tracker import *
from ap_lib import gps_utils
from argparse import ArgumentParser
from autopilot_bridge.msg import LLA
import math

STATE_TYPE =0
# These are globals set by command line arguments
 # Flock leader:
flockLeaderID = None
ownID = None
 # Base altitude
BASE_ALT = None
 # Altitude separation
ALT_SEP = None
 #Following distance from leader
Rfollow = -10 #units meters
Rovershoot = 60 #units meters
leaderUpdateCount =10 #how long before updating leader

#Home point, used for computing relative metric distance (dubins paths)
HOME_LAT = 35.7167
HOME_LON = -120.764508
dubins_rad = 20.0
dubins_step = 2.0

#meters_to_lla(home_lat, home_lon, target_mx, target_my, target_malt)
#Converts meters from home to lla values. 
#TODO: Incorporate into general GPS distance library
# @home_lat, home_lon: Home latitude, longidtude 
# @target_mx, target_my: meters from Home in x, y directions (+x is E, +y is N)
# @target_malt: altitude, passed straight through
def meters_to_lla(home_lat, home_lon, target_mx, target_my, target_malt):
    #Assume that the earth is perfectly round
    radius_of_earth = 6378100.0
    #Radius changes as you move north
    radius_at_lat = radius_of_earth*math.cos(math.radians(home_lat))

    lat = home_lat + math.degrees(target_my/radius_of_earth)
    lon = home_lon + math.degrees(target_mx/radius_at_lat)
    return (lat,lon,target_malt)


#Compute target Waypoints with overshoot
def compute_follow_wp(leaderID, tracker):
    global Rfollow, Rovershoot,STATE_TYPE

    #Test whether leaderID is in the swarm:
    if leaderID not in tracker.swarm:
        print "Can't see leader %u yet"%leaderID
        return None

    #Check contents
    leader = tracker.swarm[leaderID]
    l_lla = leader.getPosition(STATE_TYPE) #stores (lat, lon, alt)
    myself = tracker.swarm[ownID]    
    m_lla = myself.getPosition(STATE_TYPE)
    if leader is None or myself is None: 
        print "Missing data"
        return None

    #load current/past leader lat/lon into containers
    radius_of_earth = 6378100.0
    radius_at_lat = radius_of_earth*math.cos((math.radians(l_lla[0] + m_lla[0]))/2)

    #Compute the forward projection based on leader heading
    rpy = leader.getEulerAngles(STATE_TYPE)
    leader_angle = rpy[2]

    #Project a line back (Point A)
    a_lat = math.radians(l_lla[0]) - (Rfollow*math.cos(leader_angle))/radius_of_earth
    a_lon = math.radians(l_lla[1])  - (Rfollow*math.sin(leader_angle))/radius_at_lat
    
    #Project a line forward (Point B)
    a_angle = math.radians(gps_utils.gps_bearing(m_lla[0], m_lla[1], a_lat, a_lon))
    b_lat = math.degrees(a_lat + (Rovershoot*math.cos(a_angle))/radius_of_earth)
    b_lon = math.degrees(a_lon + (Rovershoot*math.sin(a_angle))/radius_at_lat)

    return (b_lat, b_lon)



def dubins_path_length(start_config, end_config, radius):
    #A config is a tuple (x,y,theta)
    #This is a hueristic for the actual dubins distance.
    
    #if within turning radius, then really hard to visit
    path_length = math.sqrt((start_config[0]-end_config[0])**2 + (start_config[1]-end_config[1])**2)
    if path_length < radius:
        path_length += 2*math.pi*radius
    path_length += radius*abs(start_config[2]-end_config[2])
    return path_length


#Perform algorithm for V formation
def find_local_leader(flockLeaderID, tracker):
    global STATE_TYPE
    # Find leader object
    if flockLeaderID not in tracker.swarm or ownID not in tracker.swarm:
       print "no data"
       return None
    
    leader = tracker.swarm[flockLeaderID]
    myself = tracker.swarm[ownID]

    #Here we convert yaw into coordinates which play nicely with the dubins generator (might not be necessary?)
    #Get self configuration for purposes of dubins path lengths
    m_lat,m_lon,m_alt = myself.getPosition(STATE_TYPE)
    m_roll,m_pitch,m_yaw = myself.getEulerAngles(STATE_TYPE)
    self_R = gps_utils.gps_distance(HOME_LAT, HOME_LON, m_lat, m_lon)
    self_bearing = math.radians(gps_utils.gps_bearing(HOME_LAT, HOME_LON, m_lat, m_lon))
    self_config = (self_R*math.cos(self_bearing),self_R*math.sin(self_bearing),math.pi/2-m_yaw)

    #Get leader configuration for purposes of dubins path lengths
    l_lat,l_lon,l_alt = leader.getPosition(STATE_TYPE)
    l_roll,l_pitch,l_yaw = leader.getEulerAngles(STATE_TYPE)
    leader_R = gps_utils.gps_distance(HOME_LAT, HOME_LON, l_lat, l_lon)
    leader_bearing = math.radians(gps_utils.gps_bearing(HOME_LAT, HOME_LON, l_lat, l_lon))
    leader_config = (leader_R*math.cos(leader_bearing), leader_R*math.sin(leader_bearing), math.pi/2-l_yaw)

    #Get distance to go straight to leader
    MAX_DISTANCE = dubins_path_length(self_config, leader_config, dubins_rad)

    #Initialize to flockLeader
    local_leader = flockLeaderID

    #Look for agents closer to leader than myself; 
    #  choose to follow the nearest of these
    MAX_NEARNESS = 100 #The furthest leader you are willing to follow (units vary depending on weighting. See below)

    closest = MAX_NEARNESS

    #Perform exhaustive search (will not scale)
    for agentID in tracker.swarm:
        #consider agents not self or leader:
        if agentID is not flockLeaderID and agentID is not ownID:

            agent = tracker.swarm[agentID]

            #Form configuration for agent
            a_lat,a_lon,a_alt = agent.getPosition(STATE_TYPE)
            a_roll,a_pitch,a_yaw = agent.getEulerAngles(STATE_TYPE)
            agent_R = gps_utils.gps_distance(HOME_LAT, HOME_LON, a_lat, a_lon)
            agent_bearing = math.radians(gps_utils.gps_bearing(HOME_LAT, HOME_LON, a_lat, a_lon)) 
            agent_config = (agent_R*math.cos(agent_bearing), agent_R*math.sin(agent_bearing), math.pi/2-a_yaw)
            #TODO: revisit this logic.

            #find minimum distance path from agent to leader
            distance_al = dubins_path_length(agent_config, leader_config, dubins_rad)

            #Don't consider if further from leader than you are.
            if distance_al > MAX_DISTANCE:
                continue

            #find minimum distance path from self to agent 
            distance_sa = dubins_path_length(self_config, agent_config, dubins_rad)
            #don't consider if further from leader
            if distance_sa > MAX_DISTANCE:
                continue 
            elif distance_sa < closest:
                closest = distance_sa
                local_leader = agentID

    if local_leader not in tracker.swarm:
        print "Found a bug! I thought %u was in the swarm:"%local_leader
        print swarm
        return flockLeaderID
    return local_leader

if __name__ == '__main__':
    #Parse arguments
    parser = ArgumentParser("rosrun ap_path_planning follow_us.py")
    parser.add_argument("-leaderID",help="ID of flock leader",dest="leaderID",default=None)
    parser.add_argument("-selfID",help="ID of self",dest="selfID",default=None)
    parser.add_argument("--use-base-alt",help="altitude to maintain during flight",dest="BASE_ALT",default=None)
    parser.add_argument("--use-alt-sep",help="altitude separation to maintain from leader",dest="ALT_SEP",default=None)
    parser.add_argument("--lookahead", dest="lookahead",default=-10,help="following distance to leader")
    parser.add_argument("--overshoot", dest="overshoot",default=40,help="Overshoot distance distance")
    args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])
    if args.BASE_ALT is not None:
        BASE_ALT = float(args.BASE_ALT)
        print "Using constant altitude %f" % (BASE_ALT)
    elif args.ALT_SEP is not None:
        ALT_SEP = float(args.ALT_SEP)
        if BASE_ALT is not None:
            print "Please use either the --use-base-alt or --use-alt-sep option"
            sys.exit(1)
        print "Using altitude separation %f" % (ALT_SEP)
    else:
        print "Please supply either base altitude or separation altitude (--use-base-alt (alt in meters) or --use-alt-sep (alt_sep in meters)"
        sys.exit(1)
    flockLeaderID = int(args.leaderID)
    if flockLeaderID is None:
        print "Please supply leader ID (--leaderID LEADERID)"
        sys.exit(1)

    ownID = int(args.selfID)
    if ownID is None:
        print "Please supply self ID (--selfID SELFID)"
        sys.exit(1)

    Rfollow = float(args.lookahead)
    Rovershoot = float(args.overshoot)

    #Initialize ROS nodes
    rospy.init_node("formation")
    #pub_pl_wp = rospy.Publisher("/autopilot/guided_goto",LLA)
    pub_pl_wp = rospy.Publisher("/autopilot/payload_waypoint",LLA)
    print "\n Agent %u flocking around aircraft %u....\n" % (ownID,flockLeaderID)
    tracker = SwarmTrackerSubscriber()
    leaderUpdateCounter=0
    leader_ID = flockLeaderID
    r = rospy.Rate(1.0)
    lastLeader = 0;
    while not rospy.is_shutdown():
       
        target_wp =  None
        #check for valid data (should be more thorough)
        if tracker is not None and tracker.swarm.get(ownID,None) is not None \
          and tracker.swarm.get(flockLeaderID,None) is not None:
            if leaderUpdateCounter == leaderUpdateCount:
                print "Re-evaluating leader"
                leaderUpdateCounter = 0
                leader_ID = find_local_leader(flockLeaderID, tracker)
            else:
                leaderUpdateCounter+=1
            leader_lat=None
            leader_lon=None
            leader_alt=None
            if leader_ID is not None:
                if leader_ID is not lastLeader:
                    print "Now following agent %u..." % leader_ID
                    lastLeader = leader_ID
#                target_wp = compute_follow_wp(leader_ID, tracker)
                if leader_ID in tracker.swarm:
                    leader_lat,leader_lon,leader_alt = tracker.swarm[leader_ID].getPosition(STATE_TYPE)
                    target_wp = (leader_lat,leader_lon)
                else:
                    print "Targeting leader %u, but can't find him!"%leader_ID
                    print tracker.swarm
            if target_wp is not None:
                target_lat, target_lon = target_wp
                #Build and send message
                lla = LLA()
                lla.lat, lla.lon = target_wp
                lla.alt = None
                if BASE_ALT is not None:
                    lla.alt = BASE_ALT
                elif ALT_SEP is not None:
                    print "Warning: this altitude implementation is not complete (does not guarantee airspace separation when leaders switch)"
                    lla.alt = leader_alt + ALT_SEP
                if lla.alt is not None:
                    pub_pl_wp.publish(lla)
                    print "Sent to (%0.06f, %0.06f, %0.03f)" % (lla.lat, lla.lon, lla.alt)
                else:
                    print "Invalid altitude command. No waypoint sent"
        r.sleep()
