#Simple visualizer node. 
#
# Run by calling:
# python visualizer.py
#
# Stefan Jorgensen


import sys
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import tf
import matplotlib
import matplotlib.pyplot as plt
import numpy
from ap_perception.swarm_tracker import *
from ap_lib import gps_utils

#Used to hilight your own aircraft and setup the subscriber.
SELFID = 22 

#matlibplot functions for displaying trajectories
plt.ion()
fig = plt.figure()
trajPlotter = fig.add_subplot(111)
trajPlotter.set_ylim([-500,500])
trajPlotter.set_xlim([-500,500])

def plot_path(path,colorOption,ID=0):
    global trajPlotter, fig
    if len(path) is 0:
        print "No data to plot"
        return
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
def gps_distance2(lat1, lon1, lat2, lon2, isRadians):
    radius_of_earth = 6378100.0 #meters
    #Operate in Radians
    if isRadians == False:
        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)
        lon1 = math.radians(lon1)
        lon2 = math.radians(lon2)
    #Compute radius of earth at longitude
    radius_at_lat = radius_of_earth*math.cos((lat1+lat2)/2)
    #Compute arclength as distance (precise implementation would incorporate altitude difference into radius)
    # arclength = (dtheta/2*Pi)*2*Pi*radius <= big*small, beware numerical instability.
    dlat = (lat2-lat1)*radius_of_earth
    dlon = (lon2-lon1)*radius_at_lat
#    if (lat2-lat1)/radius_of_earth <  10**-12:
#        print "Potential numerical instability in gps_distance2: 12 orders of magnitude difference in multiplication of two floats"
    #Compute heading relative to North-Down (atan2(lat/lon) is relative to East-Up)
    angle = math.atan2(dlat,dlon)
    return (dlat,dlon,angle)


#Fills common marker data (position, pose)
def fill_marker(plane, scale, offsets):
    HOME_LAT = 35.7167
    HOME_LON = -120.764508
    marker = Marker()
    marker.lifetime = rospy.Duration(0.3)
    marker.id = plane.ID
    marker.header.frame_id = "/world"
    marker.action = marker.ADD

    #use dead-reckoned values (arg = 1)
    roll,pitch,yaw = plane.getEulerAngles(0)
    lat, lon, alt = plane.getPosition(0)

    R = gps_utils.gps_distance(HOME_LAT, HOME_LON, lat, lon)
    T = math.radians(gps_utils.gps_bearing(HOME_LAT, HOME_LON, lat, lon))
    
    q = tf.transformations.quaternion_from_euler(roll + offsets[0], pitch + offsets[1], -yaw + offsets[2])
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]
    marker.pose.position.x, marker.pose.position.y,_=gps_distance2(HOME_LAT,HOME_LON, lat,lon,False)
    marker.pose.position.x += offsets[3]
    marker.pose.position.y += offsets[4]
#    marker.pose.position.x = R*math.cos(T) + offsets[3]
#    marker.pose.position.y = R*math.sin(T)+ offsets[4]
    marker.pose.position.z = alt + offsets[5] 

    return marker

def get_marker_tie(plane):
    scale = 0.051
    offsets = (0,0,math.pi, 0.0,0.0,1.0) #roll, pitch, yaw, x, y, z
    marker = fill_marker(plane, scale, offsets)
    marker.type = marker.MESH_RESOURCE
    marker.mesh_resource = "package://ap_lib/meshes/TIE.stl"
    marker.color.a = 1.0
    marker.color.r = 0.45
    marker.color.g = 0.8
    marker.color.b = 0.24
    markerArray.markers.append(marker)
    return marker

def get_marker_corvette(plane):
    scale = 0.02
    offsets = (0,0,-math.pi, 0,0,1) #roll, pitch, yaw, x, y, z    
    marker = fill_marker(plane,scale,offsets) 
    marker.type = marker.MESH_RESOURCE
    marker.mesh_resource = "package://ap_lib/meshes/corvette.stl"
    marker.color.a = 1.0
    marker.color.r = 0.45
    marker.color.g = 0.8
    marker.color.b = 0.24
    markerArray.markers.append(marker)
    return marker

def get_marker_xwing(plane):
    scale = 0.005
#    offsets = (3*math.pi/2, math.pi, 0, -0.5, -1, 0.5) #roll, pitch, yaw, x, y, z
    offsets = (math.pi/2,0,math.pi,-0.5,-1.0,0.5)
    marker = fill_marker(plane, scale, offsets) 
    marker.type = marker.MESH_RESOURCE
    marker.mesh_resource = "package://ap_lib/meshes/xwing.stl"
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
    marker.mesh_resource = "package://ap_lib/meshes/5th_2ss.stl"
    marker.color.a = 1.0
    marker.color.r = 0.45
    marker.color.g = 0.8
    marker.color.b = 0.24
    markerArray.markers.append(marker)
    return marker

if __name__ == '__main__':
    rospy.init_node('Plane_Visualizer')
    marker_pub = rospy.Publisher("/plane_viz",MarkerArray)
    label_pub = rospy.Publisher("/label_viz",MarkerArray)
    br = tf.TransformBroadcaster()

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
            
            if p.ID == SELFID: #your plane
                marker = get_marker_5th_2ss(p)
                q = marker.pose.orientation
       
                br.sendTransform((marker.pose.position.x, marker.pose.position.y, marker.pose.position.y), (q.x,q.y,q.z,q.w), rospy.Time.now(),"your_plane","world")
                color[p.ID] = "r-"
            elif p.ID == 1: #keep as a special plane
                marker = get_marker_corvette(p)
                #br.sendTransform((marker.pose.position.x, marker.pose.position.y, marker.pose.position.y), tf.transformations.quaternion_from_euler(p.pitch,p.roll,p.yaw), rospy.Time.now(),"1","world")
                
                color[p.ID] = "g-"
            else:
                marker = get_marker_5th_2ss(p)
                color[p.ID] = "k-"

            markerArray.markers.append(marker)

            if p.ID in trajectories:
                if len(trajectories[p.ID]) < tail_length:
                    trajectories[p.ID] = trajectories[p.ID] + [(marker.pose.position.x,marker.pose.position.y)]
                else:
                    trajectories[p.ID] = trajectories[p.ID][1:tail_length-1] + [(marker.pose.position.x, marker.pose.position.y)]
                plot_path(trajectories[p.ID], color[p.ID],p.ID)
            else:
                #sanity check for numbers:
                if abs(marker.pose.position.x) > 100000 or abs(marker.pose.position.y) > 100000:
                    _ = 0
                    print marker.pose.position.x
                    print marker.pose.position.y 
                else:
                    trajectories[p.ID] = [(marker.pose.position.x, marker.pose.position.y)]
#Add labels
            offset = (0,0,0,1,1,1)
            scale = 5
            label = fill_marker(p, scale, offset)
            label.type = label.TEXT_VIEW_FACING
            label.text = str(p.ID)
            label.color.a = 1.0
            label.color.r = 0.2
            label.color.g = 0.5
            label.color.b = 0.3
            labelArray.markers.append(label)

        if n > 0.0:
            print "Average separation: %0.6f" % (sumDist/n)
        # Publish the MarkerArray
        marker_pub.publish(markerArray)
        label_pub.publish(labelArray)
        plt.legend()
        fig.canvas.draw()
        rate.sleep()
