#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from sonar_data_aggregator import SonarDataAggregator
from laser_data_aggregator import LaserDataAggregator
from navigation import Navigation

# Class for assigning the robot speeds 
class RobotController:

    # Constructor
    def __init__(self):

        # Debugging purposes
        self.print_velocities = rospy.get_param('print_velocities')
    
        # Where and when should you use this?
        self.stop_robot = False
    
        # Create the needed objects
        self.sonar_aggregation = SonarDataAggregator()
        self.laser_aggregation = LaserDataAggregator()
        self.navigation  = Navigation()
    
        self.linear_velocity  = 0
        self.angular_velocity = 0
    
        # Check if the robot moves with target or just wanders
        self.move_with_target = rospy.get_param("calculate_target")
    
        # The timer produces events for sending the speeds every 110 ms
        rospy.Timer(rospy.Duration(0.11), self.publishSpeeds)
        self.velocity_publisher = rospy.Publisher(\
        rospy.get_param('speeds_pub_topic'), Twist,\
        queue_size = 10)
    
        # Read the velocities architecture
        self.velocity_arch = rospy.get_param("velocities_architecture")
        print "The selected velocities architecture is " + self.velocity_arch

    # This function publishes the speeds and moves the robot
    def publishSpeeds(self, event):

        # Choose architecture
        if self.velocity_arch == "subsumption":
            self.produceSpeedsSubsumption()
        else:
            self.produceSpeedsMotorSchema()
    
        # Create the commands message
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0 
        twist.angular.y = 0
        twist.angular.z = self.angular_velocity
    
        # Send the command
        self.velocity_publisher.publish(twist)
        # Print the speeds for debuggind purposes
        if self.print_velocities == True:
            print "[L,R] = [" + str(twist.linear.x) + " , " + \
            str(twist.angular.z) + "]"
    
    # Produce speeds from sonars
    def produceSpeedsSonars(self):
        # Get the sonars' measurements
        front   = self.sonar_aggregation.sonar_front_range
        left    = self.sonar_aggregation.sonar_left_range
        right   = self.sonar_aggregation.sonar_right_range
        r_left  = self.sonar_aggregation.sonar_rear_left_range
        r_right = self.sonar_aggregation.sonar_rear_right_range
    
        linear  = 0
        angular = 0
    
        # YOUR CODE HERE ------------------------------------------------------
        # Adjust the linear and angular velocitie s using the five sonars values 
        
        #Sonar's max distance
        max_dist = 3
        
        #Distance range 0 to 1
        linear = front / max_dist
        
        #Try to walk at the center
        angle = ( left - right ) / max_dist
        angular = angle / linear
        
        #90 degrees turn when it is very close to the obstacle
        if linear < 0.1:
            angular = 1
        
        # ---------------------------------------------------------------------
        
        return [linear, angular]

    # Produces speeds from the laser
    def produceSpeedsLaser(self):

        # Get the laser scan
        scan   = self.laser_aggregation.laser_scan
    
        linear  = 0
        angular = 0
    
        # YOUR CODE HERE ------------------------------------------------------
        # Adjust the linear and angular velocities using the laser scan
    
        # ---------------------------------------------------------------------
    
        return [linear, angular]

    # Combines the speeds into one output using a subsumption approach
    def produceSpeedsSubsumption(self):

        # Produce target if not existent
        if self.move_with_target == True and self.navigation.target_exists == False:
            self.navigation.selectTarget()
    
        # Get the submodules' speeds
        [l_sonar, a_sonar] = self.produceSpeedsSonars()
        [l_laser, a_laser] = self.produceSpeedsLaser()
        [l_goal, a_goal] = self.navigation.velocitiesToNextSubtarget()
    
        self.linear_velocity  = 0
        self.angular_velocity = 0
    
        # Combine the speeds following the subsumption architecture
        # YOUR CODE HERE ------------------------------------------------------
        # ---------------------------------------------------------------------

    # Combines the speeds into one output using a motor schema approach
    def produceSpeedsMotorSchema(self):

        # Produce target if not existent
        if self.move_with_target == True and self.navigation.target_exists == False:
            self.navigation.selectTarget()

        # Get the submodule's speeds
        [l_sonar, a_sonar] = self.produceSpeedsSonars()
        [l_laser, a_laser] = self.produceSpeedsLaser()
        [l_goal, a_goal] = self.navigation.velocitiesToNextSubtarget()

        self.linear_velocity  = 0
        self.angular_velocity = 0

        # Get the speeds using the motor schema approach
        # YOUR CODE HERE ------------------------------------------------------

        self.linear_velocity = l_sonar
        self.angular_velocity = a_sonar     

        # ---------------------------------------------------------------------

    # Assistive functions - Do you have to call them somewhere?
    def stopRobot(self):
        self.stop_robot = True

    def resumeRobot(self):
        self.stop_robot = False
