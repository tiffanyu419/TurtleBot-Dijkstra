#!/usr/bin/env python

import maze
import roslib; roslib.load_manifest('project4')
import rospy
import tf
import transform2d
import numpy 

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState

# set maze
maze1 = [
        '+-+-+-+-+',
        '| |     |',
        '+ + +-+ +',
        '| | |   |',
        '+ + +-+-+',
        '|       |',
        '+-+-+ +-+',
        '|       |',
        '+ + +-+-+',
        '| |     |',
        '+ +-+-+ +',
        '| |     |',
        '+-+-+-+-+',
    ]

# we will run our controller at 100Hz
CONTROL_PERIOD = rospy.Duration(0.01)

# one degree is pi/180 radians
DEG = numpy.pi / 180.0

# one foot is 0.3048 meters
FT = 0.3048

ANGULAR_TOL = 0.03 # rad 
ANGULAR_RAMP_TIME = 2.0 # s
ANGULAR_RAMP_SPEED = 0.8 # rad/s
ANGULAR_MIN_SPEED = 0.2 # rad/s
ANGULAR_GAIN = 3.0 # converts rad to rad/s

VELOCITY_TOL = 0.05
VELOCITY_RAMP_TIME = 2.0
VELOCITY_GAIN = 0.5
VELOCITY_RAMP_SPEED = 0.3
VELOCITY_MIN_SPEED = 0.1


# List of valid directions
VALID_COMMANDS = ['straight', 'turn_left', 'turn_right', 'nudge', 'forward']
VALID_DIRECTIONS = ['up','down','left','right']

# Controller for project 4
class Controller:

    # Initializer
    def __init__(self):

        # Setup this node
        rospy.init_node('Controller')

        # Create a TransformListener - we will use it both to fetch
        # odometry readings and to get the transform between the
        # Kinect's depth image and the base frame.
        self.tf_listener = tf.TransformListener()

        # Stash the 2D transformation from depth image to base frame.
        self.base_from_depth = None

        # Let's start out in the initializing state
        self.reset_state('initializing')

        # These member variables will be set by the laser scan
        # callback.
        self.angles = None
        self.ranges = None
        self.points = None

        # Create a publisher for commanded velocity
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity',
                                           Twist)

        # Subscribe to laser scan data
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Subscribe to string-based maze command
        rospy.Subscriber('/maze_command', String, self.command_callback)

        # Subscribe to bumpers/cliff sensors
        rospy.Subscriber('/mobile_base/sensors/core',
                         SensorState, self.sensor_callback)

	# Set up a Transformlistner to get odometry information
	self.odom_listener = tf.TransformListener()

        # Call the controll callback at 100 HZ
        rospy.Timer(CONTROL_PERIOD, self.control_callback)
	
    # Called whenever we get a command string message
    def command_callback(self, msg):
        if self.state != 'idle':
            # Don't allow commands unless currently idle
            rospy.loginfo('not idle, so ignoring command {}'.format(
                    msg.data))
            return

        #elif msg.data not in VALID_COMMANDS:
          #  rospy.logwarn('invalid command: {}'.format(msg.data))
          #  return

	#set up maze
	m = maze.Maze()
	m.load_from_array(maze1)
	
	h = m.height()
	w = m.width()

	#split string command into different elements
	x0, y0, dir0, x1, y1 = maze.split_command(msg.data)
	#check if inputs are valid
	if x0 > w-1 or x1 > w-1 or y0 > h-1 or y1 > h-1:
	  rospy.logwarn('invalid coordinate: {}'.format(msg.data))
	  return
	
	path = m.find_path(x0, y0, x1, y1)	
	self.path_commands = maze.path_to_command(m, path, dir0)
	next_state = self.path_commands.pop(0)
	print "path_commands after first pop", self.path_commands
	self.reset_state(next_state)

    # You can pass reset_time=False to disable resetting the
    # self.start_time variable. This is useful if you want to switch
    # from one behavior to another (i.e. "turnright" to "straighten")
    # without restricting speed.
    def reset_state(self, state, reset_time=True):

        self.state = state
        self.start_pose = None

        if reset_time:
            self.start_time = rospy.get_rostime()

        if state == 'initializing':
            rospy.loginfo('waiting for TF and laser scan...')
        elif state == 'idle':
            rospy.loginfo('waiting for commands...')
            
    # Called whenever sensor messages are received.
    def sensor_callback(self, msg):

        if ((msg.cliff or msg.bumper) and 
            self.state not in ['idle', 'initializing']):
            # picked up, just become idle
            rospy.loginfo('going to idle for safety stop')
            self.reset_state('idle')

    # Set up the transformation from depth camera to base frame.
    # We will need this later to deal with laser scanner.
    def setup_tf(self):

        # We might need to do this more than once because the
        # TransformListener might not be ready yet.
        try:

            ros_xform = self.tf_listener.lookupTransform(
                '/base_footprint', '/camera_depth_frame',
                rospy.Time(0))

        except tf.LookupException:

            return False

        self.base_from_depth = \
            transform2d.transform2d_from_ros_transform(ros_xform)

        return True

    # Called whenever we hear from laser scanner. This just sets up
    # the self.angles, self.ranges, and self.points member variables.
    def scan_callback(self, msg):

        # Don't do anything til we have the transform from depth
        # camera to base frame.
        if self.base_from_depth is None:
            if not self.setup_tf():
                return

        # Get # of range returns
        count = len(msg.ranges)

        # Create angle array of size N
        self.angles = (msg.angle_min +
                       numpy.arange(count, dtype=float) * msg.angle_increment)

        # Create numpy array from range returns (note many could be
        # NaN indicating no return for a given angle).
        self.ranges = numpy.array(msg.ranges)

        # Points is a 2xN array of cartesian points in depth camera frame
        pts = self.ranges * numpy.vstack( ( numpy.cos(self.angles),
                                            numpy.sin(self.angles) ) )

        # This converts points from depth camera frame to base frame
        # and reshapes into an Nx2 array so that self.points[i] is the
        # point corresponding to self.angles[i].
        self.points = self.base_from_depth.transform_fwd(pts).T

    # Given a list of desired angles (e.g. [-5*DEG, 5*DEG], look up
    # the indices of the closest valid ranges to those angles. If
    # there is no valid range within the cutoff angular distance,
    # returns None. 
    def lookup_angles(self, desired_angles, cutoff=3*DEG):

        # Don't return anything if no data.
        if self.angles is None:
            return None

        # Get indices of all non-NaN ranges
        ok_idx = numpy.nonzero(~numpy.isnan(self.ranges))[0]

        # Check all NaN
        if not len(ok_idx):
            return None

        # Build up array of indices to return
        indices = []

        # For each angle passed in
        for angle in desired_angles:

            # Find the closest index
            angle_err = numpy.abs(angle - self.angles[ok_idx])
            i = angle_err.argmin()

            # If too far away from desired, fail :(
            if angle_err[i] > cutoff:
                return None

            # Append index of closest
            indices.append(ok_idx[i])

        # Return the array we built up
        return indices

    # Look up points at the angles given (see lookup_angles for
    # interpretation of desired_angles, cutoff). 
    def points_at_angles(self, desired_angles, cutoff=3*DEG):

        indices = self.lookup_angles(desired_angles, cutoff)
        if indices is None:
            return None

        return self.points[indices]

    # Gets the current pose of the robot w.r.t. odometry frame.
    def get_current_pose(self):

        try:
            ros_xform = self.tf_listener.lookupTransform(
                '/odom', '/base_footprint',
                rospy.Time(0))

        except tf.LookupException:
            return None

        xform2d = transform2d.transform2d_from_ros_transform(ros_xform)

        return xform2d

    # Abstract out the control envelope function that was developed
    # for project 2. 
    def envelope(self, error, tol, gain,
                 min_speed, ramp_time, ramp_speed):

        time = (rospy.get_rostime() - self.start_time).to_sec()
        command = error * gain
        command = numpy.sign(command) * max(abs(command), min_speed)
        effective_time = min(time, ramp_time)
        max_speed = effective_time * ramp_speed/ramp_time
        command = numpy.clip(command, -max_speed, max_speed)
        done = abs(error) < tol

        return command, done

    # Just calls function above - might want to duplicate this for
    # linear_envelope.
    def angular_envelope(self, error):
        return self.envelope(error, ANGULAR_TOL, ANGULAR_GAIN,
                             ANGULAR_MIN_SPEED,
                             ANGULAR_RAMP_TIME, ANGULAR_RAMP_SPEED)

    # Calls envelope function with velocity constants
    def linear_envelope(self, error):
	return self.envelope(error, VELOCITY_TOL,  VELOCITY_GAIN,
                              VELOCITY_MIN_SPEED,
                              VELOCITY_RAMP_TIME,  VELOCITY_RAMP_SPEED)

    #called to make robot go straight for a desired distance
    def go_straight(self, rel_pose, desired_distance):
	cmd_vel = Twist()
	#Using the envelope to go straight
        distance_error = desired_distance - rel_pose.x
	command, done = self.linear_envelope(distance_error)

        cmd_vel.linear.x = command
	cmd_vel.angular.z = 0

	return cmd_vel, done


    # Called 100 times per second to control the robot.
    def control_callback(self, timer_event=None):
	print 'state: ',self.state
        # Velocity we will command (modifed below)
        cmd_vel = Twist()

        # Get current pose
        cur_pose = self.get_current_pose()

        # Try to get relative pose
        if cur_pose is not None:
            if self.start_pose is None:
                self.start_pose = cur_pose.copy()
            rel_pose = self.start_pose.inverse() * cur_pose 

        # Dispatch on state:
        if self.state == 'initializing':

            # Go from initializing to idle once TF is ready and we
            # have our first laser scan.
            if cur_pose is not None and self.angles is not None:
                self.reset_state('idle')
	
        elif self.state == 'straighten':

            # Register points at +/- 1 degrees, and examine the angle
            # between them.
            points = self.points_at_angles([1*DEG, -1*DEG])

            if points is None or points[0][1] > 6*FT or points[1][1] > 6*FT:
                # No points - backs up
                #rospy.logwarn('points was None in straighten state - '
                 #             'backing up')
	#	backup_distance = -0.25 - rel_pose.x
	#	command, done = self.linear_envelope(-0.25)
		#cmd_vel.linear.x = command

		next_state = self.path_commands.pop(0)	
                self.reset_state(next_state)

            else:

                # Subtract points
                delta = points[1]-points[0]

                # Get angle
                angular_error = numpy.arctan2(delta[0], -delta[1])

                # Get command and go to idle state if done
                command, done = self.angular_envelope(angular_error)

                cmd_vel.angular.z = command

                if done:
		    next_state = self.path_commands.pop(0)	
                    self.reset_state(next_state)

	elif self.state == 'turn_left':
	    angle_error = (numpy.pi)/2 - rel_pose.theta
	    command, done = self.angular_envelope(angle_error)
	    cmd_vel.angular.z = command	 
	    #transition to straighten 
	    if done:
                self.reset_state ('straighten', reset_time = False)

	elif self.state == 'turn_right':
	    angle_error = -(numpy.pi-0.1)/2 - rel_pose.theta
	    command, done = self.angular_envelope(angle_error)            
	    cmd_vel.angular.z = command

	    #transition to straighten
       	    if done:
               	self.reset_state ('straighten', reset_time = False)

	elif self.state == 'forward':
	    
	    # forward will always try to move a full cell (3 tiles) forward 
	    cmd_vel, done = self.go_straight(rel_pose, 0.756)
 
	    #transition to nudge when nearing center of cell
   	    if done:
          	self.reset_state('nudge', reset_time = False)

	elif self.state == 'nudge':
		#Register point at 0 degrees, directly in front of robot
		point_0 = self.points_at_angles([0])

 		if point_0 is None:

               		# No points - backs up
               		rospy.logwarn('points was None in nudge state - '
                              'backing up')
			cmd_vel, done = self.go_straight(rel_pose, -0.2)
		
		#find distance to nudge to center of nearest cell using lasers
           	else:
 			measured_dist = point_0[0][0]
			rounded_dist = numpy.floor(measured_dist/(3*FT))*3*FT + 1.5*FT
			difference_dist = measured_dist - rounded_dist
			
			# execute nudge movement
			command, done = self.linear_envelope(difference_dist)
			cmd_vel.linear.x = command
			
			if done:
				next_state = self.path_commands.pop(0)
				self.reset_state(next_state)	

        elif self.state != 'idle':
            # Die if invalid state :(
 
           rospy.logerr('invalid state {}'.format(self.state))
           rospy.signal_shutdown('invalid state')

        # Publish our velocity command
        self.cmd_vel_pub.publish(cmd_vel)

    # Running the controller is just rospy.spin()
    def run(self):
        rospy.spin()

# Really skinny main function
if __name__ == '__main__':
    try: 
        c = Controller()
        c.run()
    except rospy.ROSInterruptException:
        pass
