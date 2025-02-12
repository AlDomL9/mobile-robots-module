#!/usr/bin/env python
""" Node that generates a map of the environment based on the laser scan data.
and the odometry data.

Author: kjartan@tec.mx (Kjartan Halvorsen) with help from github copilot

Notes.
1) The scan data give information about free space as well as obstacles. Each ray in the scan will cover a number
of pixels in the map. The map should be updated by setting the pixels covered by the ray to 0 (free) and the last pixel
to occupied (100). The map should be updated only if the ray range is less than the max_range of the scan.
2) You should determine the number of points in each scan ray by multiplying the range of the ray by the map resolution.
Then you convert these points (each corresponding to a pixel) from a robot frame to a map frame using the odometry data.
3) The map should be updated only if the robot has moved a certain distance since the last update. This is to
avoid updating the map too often, since it is a somewhat expensive operation.
4) It can be more efficient to use numpy arrays for the rigid transformations needed to convert scans
to map coordinates. To make this work, you need to convert the geometry_msgs/TransformStamped to a numpy array.
See https://answers.ros.org/question/332407/transformstamped-to-transformation-matrix-python/
With a transform matrix T, you can transform a number of points at once by collecting the points in a numpy array
and multiplying the array with T.
To use numpify, you need to install the package ros-meldic-ros-numpy.


"""
import sys
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from sensor_msgs.msg import LaserScan
import math


class Mapper:
    def __init__(self, map_width, map_height, map_resolution):
        """
        Arguments
        ---------
        map_width : float
            Width of map in pixels (x-axis)
        map_height : float
            Height of map in pixels (y-axis)
        map_resolution : float
            Resolution of map in pixels/meter
        """
        self.scan_listener = rospy.Subscriber(
            "/laser/scan", LaserScan, self.scan_callback
        )
        self.odom_listener = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1)
        self.rate = rospy.Rate(5.0)
        self.map = OccupancyGrid()
        self.map.info = MapMetaData(
            rospy.Time.now(), map_resolution, map_width, map_height, 0, 0, 0
        )
        self.map.data = np.zeros(map_width * map_height, dtype=np.int8)
        self.map.data[:] = -1  # Unknown
        self.map2d = np.full((map_width, map_height), fill_value=-1, dtype=np.int8)  # For computation
        self.scan = None
        self.odom = None

        self.x_ma = int(self.map2d.shape[1]/2)
        self.y_ma = self.map2d.shape[0] - 1

    def scan_callback(self, msg):
        """Called when a new scan is available from the lidar."""
        self.scan = msg

    def odom_callback(self, msg):
        """Called when a new odometry message is available."""
        self.odom = msg

    def map(self):
        while not rospy.is_shutdown():
            if self.scan is not None and self.odom is not None:
                # --------------------------------------------------------------
                # Your code here
                # 1) For each ray in the scan, calculate the corresponding
                #    position in the map by transforming the ray from the robot
                #    frame to the map frame, using the odometry data. The map frame
                #    is defined as the frame of the first laser scan, when the robot
                #    is initialized.
                # 2) If the ray range is less than max_range, then set the map pixel
                #    corresponding to the end point of the ray to 100 (occupied).
                # 3) Set pixels along the ray to 0 (free).
                # --------------------------------------------------------------
                # --------------------------------------------------------------

                # Publish the map
                np.copyto(
                    self.map.data, self.map2d.reshape(-1)
                )  # Copy from map2d to 1d data, fastest way
                self.map.header.stamp = rospy.Time.now()
                self.map_pub.publish(self.map)
            self.rate.sleep()

def transform(xr, yr, x, y, x_ma, y_ma):
    ma = np.array([[1,  0,  0, x_ma], 
                    [0, -1,  0, y_ma], 
                    [0,  0, -1, 0   ], 
                    [0,  0,  0, 1   ]])
    
    pr_m = np.array([xr, yr, 0, 1])
    ps_m = np.array([ x,  y, 0, 1])

    pr_a = np.dot(ma, pr_m)
    ps_a = np.dot(ma, ps_m)

    return pr_a, ps_a

def ray_to_pixels(xr, yr, x, y, map_resolution, map, x_ma, y_ma):
    """Set the pixels along the ray with origin (xr,yr) and with range ending at (x,y) to 0 (free) and the end point to 100 (occupied).
    Arguments
    ---------
    xr : float
        x-coordinate of the robot in the map frame
    yr : float
        y-coordinate of the robot in the map frame
    x : ndarray
        x-coordinates of the scan in the map frame
    y : ndarray
        y-coordinates of the scan in the map frame
    map_resolution : float
        Resolution of map in pixels/meter
    map : ndarray
        The map as a 2d numpy array
    x_ma : int
        X in the map as seen in the array
    y_ma : int
        Y in the map as seen in the array
    """
    # --------------------------------------------------------------
    # Your code here
    pr_a, ps_a = transform(xr, yr, x, y, x_ma, y_ma)
    
    x_min = min(pr_a[0], ps_a[0])
    x_max = max(pr_a[0], ps_a[0])
    y_min = min(pr_a[1], ps_a[1])
    y_max = max(pr_a[1], ps_a[1])

    if x_min < 0:
        x_add = np.full((map.shape[0], -(x_min)), fill_value=-1, dtype=np.int8)
        map = np.concatenate((x_add, map), axis = 1)
        x_ma = x_ma - x_min

    if y_min < 0:
        y_add = np.full((-(y_min), map.shape[1]), fill_value=-1, dtype=np.int8)
        map = np.concatenate((y_add, map), axis = 0)
        y_ma = y_ma - y_min

    if x_max >= map.shape[1]:
        x_add = np.full((map.shape[0], x_max - map.shape[1]), fill_value=-1, dtype=np.int8)
        map = np.concatenate((map, x_add), axis = 1)

    if y_max >= map.shape[1]:
        y_add = np.full((y_max - map.shape[0], map.shape[1]), fill_value=-1, dtype=np.int8)
        map = np.concatenate((map, y_add), axis = 0)
    
    pr_a, ps_a = transform(xr, yr, x, y, x_ma, y_ma)

    m = (pr_a[1] - ps_a[1]) / (pr_a[0] - ps_a[0])

    x_i = pr_a[0]
    y_i = pr_a[0]

    i = 0

    map[ps_a[0]][ps_a[1]] = 100

    while x_i != ps_a[1]:
        x_i += i
        yc = math.ciel(x_i * m)
        yf = math.floor(x_i * m)

        map[x_i][yc] = 0
        map[x_i][yf] = 0


    # --------------------------------------------------------------
    # --------------------------------------------------------------


def scan_to_map_coordinates(scan, odom):
    """Convert a scan from the robot frame to the map frame.
    Arguments
    ---------
    scan : LaserScan
        The scan to convert
    odom : Odometry
        The odometry message providing the robot pose
    Returns
    -------
    (xr, yr) : tuple of floats
        The position of the robot in the map frame
    xy : list
        list of tuples (x,y) with the coordinates of the scan end points in the map frame

    Tests
    -----
    >>> scan = test_laser_scan()
    >>> odom = test_odometry()
    >>> orig, xy = scan_to_map_coordinates(scan, odom)
    >>> np.allclose(orig, (1.0, 2.0))
    True
    >>> np.allclose(xy,[(2.0, 2.0), (1.0, 3.0), (0.0, 2.0)])
    True
    """

    robot_origin = odom.pose.pose.position
    xr = robot_origin.x
    yr = robot_origin.y
    thr = 2 * np.arccos(odom.pose.pose.orientation.w)
    xy = []
    for i in range(len(scan.ranges)):
        th = scan.angle_min + i * scan.angle_increment + thr
        r = scan.ranges[i]
        x, y = polar_to_cartesian(r, th)

        x += xr
        y += yr

        xy.append((x, y))
    return (xr, yr), xy


def polar_to_cartesian(r, th):
    """Convert a polar coordinate to a cartesian coordinate.
    Arguments
    ---------
    r : float
        The radius
    th : float
        The angle
    Returns
    -------
    (x, y) : tuple of floats
        The cartesian coordinates
    Tests
    -----
    >>> polar_to_cartesian(1.0, 0.0)
    (1.0, 0.0)
    >>> polar_to_cartesian(1.0, np.pi/2)
    (0.0, 1.0)
    >>> polar_to_cartesian(1.0, np.pi)
    (-1.0, 0.0)
    >>> polar_to_cartesian(1.0, 3*np.pi/2)
    (0.0, -1.0)
    """
    # --------------------------------------------------------------
    # Your code here
    x = np.cos(th) * r
    y = np.sin(th) * r

    x = round(x, 10) + 0.0 #* To fix -0.0
    y = round(y, 10) + 0.0
    # --------------------------------------------------------------
    # --------------------------------------------------------------

    return (x, y)


def test_laser_scan():
    """Create a simple test LaserScan message.
    There are 3 rays, to the left, straight ahead and to the right.
    Each ray has range 1.0."""
    scan = LaserScan()
    scan.header.frame_id = "base_link"
    scan.angle_min = -np.pi / 2
    scan.angle_max = np.pi / 2
    scan.angle_increment = np.pi / 2
    scan.range_min = 0.0
    scan.range_max = 10.0
    scan.ranges = [1.0, 1.0, 1.0]
    return scan


def test_odometry():
    """Create a test Odometry message."""
    odom = Odometry()
    odom.header.frame_id = "odom"
    odom.pose.pose.position.x = 1.0
    odom.pose.pose.position.y = 2.0
    odom.pose.pose.position.z = 0.0
    # Quaternion for 90 degree rotation around z-axis. So the robot is facing in the y-direction.
    odom.pose.pose.orientation.x = 0.0
    odom.pose.pose.orientation.y = 0.0
    odom.pose.pose.orientation.z = np.sin(np.pi / 4)
    odom.pose.pose.orientation.w = np.cos(np.pi / 4)
    return odom


if __name__ == "__main__":
    if len(sys.argv) > 1:
        if sys.argv[1] == "--test":
            import doctest

            doctest.testmod()
            sys.exit(0)

    rospy.init_node("Mapper")
    Mapper().map()
