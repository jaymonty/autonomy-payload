#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from argparse import ArgumentParser
import rospy
import math
import tf

from ap_perception.swarm_tracker import *
from ap_lib.gps_utils import *

import matplotlib
import matplotlib.pyplot as plt
import numpy



################SET THIS TO YOUR SITL INSTANCE NUMBER######################
SELFID = 100
###########################################################################

#matlibplot functions for displaying trajectories
plt.ion()
fig = plt.figure()
trajPlotter = fig.add_subplot(111)
trajPlotter.set_ylim([-500,500])
trajPlotter.set_xlim([-500,500])

def plot_path(path,colorOption,ID=0):
    global trajPlotter, fig
    pltPath = numpy.array(path)
    xs = pltPath[:,0]
    ys = pltPath[:,1]
    trajPlotter.plot(xs,ys,colorOption, label=str(ID))
    plt.annotate(str(ID),path[-1])

def clear_plot(plot):
    plot.cla()
    plot.set_ylim([-500,500])
    plot.set_xlim([-500,500])

#Return small-angle approximation based x,y, distance and angle relative to East,Up
def gps_distance2(lat1, lon1, lat2, lon2, isRadians = False):
    if isRadians:
        lat1 = math.degrees(lat1)
        lon1 = math.degrees(lon1)
        lat2 = math.degrees(lat2)
        lon2 = math.degrees(lon2)

    dist = gps_distance(lat1, lon1, lat2, lon2)
    angle = gps_bearing(lat1, lon1, lat2, lon2)
    dlat = dist * math.cos(angle)
    dlon = dist * math.sin(angle)
    return (dlat, dlon, angle)

#Fills common marker data (position, pose)
def fill_marker(plane, scale, offsets):
    global HOME_LAT, HOME_LON
    marker = Marker()
    marker.lifetime = rospy.Duration(0.3)
    marker.id = plane.ID
    marker.header.frame_id = "/world"
    marker.action = marker.ADD
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.pose.orientation.x = plane.state.pose.pose.orientation.x
    marker.pose.orientation.y = plane.state.pose.pose.orientation.y
    marker.pose.orientation.z = plane.state.pose.pose.orientation.z
    marker.pose.orientation.w = plane.state.pose.pose.orientation.w
    marker.pose.position.y, marker.pose.position.x, _ = \
        gps_distance2(HOME_LAT, HOME_LON, \
                      plane.state.pose.pose.position.lat, plane.state.pose.pose.position.lon, False)
    marker.pose.position.x = marker.pose.position.x + offsets[3]
    marker.pose.position.y = marker.pose.position.y + offsets[4]
    marker.pose.position.z = plane.state.pose.pose.position.alt + offsets[5]
    return marker

def get_marker_tie(plane):
    scale = 0.051
    offsets = (0,0,math.pi, 0.0,0.0,1.0) #roll, pitch, yaw, x, y, z
    marker = fill_marker(plane, scale, offsets)
    marker.type = marker.MESH_RESOURCE
    marker.mesh_resource = "package://ap_path_planning/meshes/TIE.stl"
    marker.color.a = 1.0
    marker.color.r = 0.45
    marker.color.g = 0.8
    marker.color.b = 0.24
    markerArray.markers.append(marker)
    return marker

def get_marker_corvette(plane):
    scale = 0.02
    offsets = (0, 0, -math.pi, 0, 0, 1) #roll, pitch, yaw, x, y, z    
    marker = fill_marker(plane,scale,offsets) 
    marker.type = marker.MESH_RESOURCE
    marker.mesh_resource = "package://ap_path_planning/meshes/corvette.stl"
    marker.color.a = 1.0
    marker.color.r = 0.45
    marker.color.g = 0.8
    marker.color.b = 0.24
    markerArray.markers.append(marker)
    return marker

def get_marker_xwing(plane):
    scale = 0.005
    offsets = (math.pi/2,0,math.pi,-0.5,-1.0,0.5)
    marker = fill_marker(plane, scale, offsets) 
    marker.type = marker.MESH_RESOURCE
    marker.mesh_resource = "package://ap_path_planning/meshes/xwing.stl"
    marker.color.a = 1.0
    marker.color.r = 0.45
    marker.color.g = 0.8
    marker.color.b = 0.25
    markerArray.markers.append(marker)
    return marker

def get_marker_5th_2ss(plane):
    scale = 0.2
    offsets = (0,0,math.pi, 0,0,1)
    marker = fill_marker(plane,scale,offsets)
    marker.type = marker.MESH_RESOURCE
    marker.mesh_resource = "package://ap_path_planning/meshes/5th_2ss.stl"
    marker.color.a = 1.0
    marker.color.r = 0.45
    marker.color.g = 0.8
    marker.color.b = 0.24
    markerArray.markers.append(marker)
    return marker

if __name__ == '__main__':
    parser = ArgumentParser("rosrun ap_perception swarm_tracker.py")
    parser.add_argument('-id', "--aircraft", dest="acft", \
                        help="ID (integer) of this aircraft", default=0)
    args = parser.parse_args(args=rospy.myargv(argv=sys.argv)[1:])
    print args.acft
#    if args.acft == 0 and rospy.has_param("aircraft_id"):
    if rospy.has_param("aircraft_id"):
        print "TRUE"
        args.acft = rospy.get_param("aircraft_id")

    rospy.init_node('Plane_Visualizer')
    marker_pub = rospy.Publisher("/plane_viz",MarkerArray)
    label_pub = rospy.Publisher("/label_viz",MarkerArray)
    br = tf.TransformBroadcaster()
    HOME_LAT = 35.7167
    HOME_LON = -120.764508

    SELFID = int(args.acft)
#    trackerBaseName = "/sitl%d/swarm_tracker"%SELFID
#    tracker = SwarmTrackerSubscriber(trackerBaseName)
    tracker = SwarmTrackerSubscriber()

    COLLISION_RADIUS = 10 #units meters
    tail_length = 5
    rate = rospy.Rate(10)
    trajectories = {}
    while not rospy.is_shutdown():
        clear_plot(trajPlotter)
        markerArray = MarkerArray()
        labelArray = MarkerArray()
        plane_list = ''
        sumDist = 0.0
        n = 0.0
        color = {}
        for plane in tracker.swarm:
            p = tracker.swarm[plane]
#            marker = get_marker_xwing(p)
#            br.sendTransform((marker.pose.position.x, marker.pose.position.y, marker.pose.position.y), \
#                             (p.state.pose.pose.orientation.x, p.state.pose.pose.orientation.y, \
#                              p.state.pose.pose.orientation.z, p.state.pose.pose.orientation.w), \
#                              rospy.Time.now(),"ac100","world")
#            color[p.ID] = "r-"
            if p.ID == SELFID: #your plane
                marker = get_marker_xwing(p)#5th_2ss(p)
		#marker = get_marker_tie(p)
                br.sendTransform((marker.pose.position.x, marker.pose.position.y, marker.pose.position.y), \
                                 (p.state.pose.pose.orientation.x, p.state.pose.pose.orientation.y, \
                                  p.state.pose.pose.orientation.z, p.state.pose.pose.orientation.w), \
                                  rospy.Time.now(),"ac100","world")
#                marker.color.r = 0.0
#                marker.color.b = 0.8
#                marker.color.a = 0.8
                color[p.ID] = "r-"
            else:
                marker = get_marker_xwing(p)
		#marker = get_marker_xwing(p)
#                marker.color.r = 0.5
#                marker.color.b = 0.5
#                marker.color.a = 1.3
                color[p.ID] = "k-"

            markerArray.markers.append(marker)
            if p.ID in trajectories:
                if len(trajectories[p.ID]) < tail_length:
                    trajectories[p.ID] = trajectories[p.ID] + \
                                         [(marker.pose.position.x,marker.pose.position.y)]
                else:
                    trajectories[p.ID] = trajectories[p.ID][1:tail_length-1] + \
                                         [(marker.pose.position.x, marker.pose.position.y)]
                plot_path(trajectories[p.ID], color[p.ID],p.ID)
            else:
                #sanity check for numbers:
                if abs(marker.pose.position.x) > 100000 or abs(marker.pose.position.y) > 100000:
                    _ = 0
                else:
                    trajectories[p.ID] = [(marker.pose.position.x, marker.pose.position.y)]
#Add labels
            offset = (0,0,0,6,6,15)
            scale = 5
            label = fill_marker(p, scale, offset)
            label.type = label.TEXT_VIEW_FACING
            label.text = str(p.ID)
#            label.color.a = 1.0
#            label.color.r = 0.2
#            label.color.g = 0.5
#            label.color.b = 0.3
            labelArray.markers.append(label)

            #Check for collisions (computationally intense way to do it):
            for plane in tracker.swarm:
                if tracker.swarm[plane].ID < p.ID:
                    dist = \
                        gps_distance(tracker.swarm[plane].state.pose.pose.position.lat, \
                                     tracker.swarm[plane].state.pose.pose.position.lon, \
                                     p.state.pose.pose.position.lat, \
                                     p.state.pose.pose.position.lon)
                    n = n + 1
                    sumDist = sumDist + dist
                    if dist < COLLISION_RADIUS:
                        print "Collision between agent %u and %u" % (p.ID, tracker.swarm[plane].ID)

        # Publish the MarkerArray
        marker_pub.publish(markerArray)
        label_pub.publish(labelArray)
        plt.legend()
        fig.canvas.draw()
        rate.sleep()
