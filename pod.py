import math
from massmotion import *
import numpy as np

'''
Constants for pod and pedestrian agent.
'''
FRAME_RATE = 5
OFF_FLOOR_SEARCH_RAD = 55 #10
COS_30D = 0.866 # cos(30 degrees)
COS_45D = 0.707
COS_60D = 0.5
COS_90D = 0
COS_15D = 0.9659
COS_25D = 0.9063
COS_40D = 0.766
MINUS_COS_40D = -0.766
MINUS_COS_25D = 0.9063
MINUS_COS_15D = -0.9659
MINUS_COS_30D = -0.866
MINUS_COS_20D = -0.94
COS_20D = 0.94

'''
Constants defined for the pod agent.
'''
POD_SEARCH_CYLINDER_RAD = 22
POD_SEARCH_CYLINDER_HEIGHT = 5
POD_EMERGENCY_RADIUS = 2
GOAL_BOX_RAD = 1
SPACE_SEARCH_RADIUS = 5
AlIGNED_BOX_LENGTH = 20
ALIGNED_STOPPING_SAFETY_FACTOR = 1.5

'''
Converts an angle in radians to the index for the corresponding available space segment.
'''
def roundNearest(x):
    return round(x / 0.1963)

'''
Pod class definition.
'''
class pod:

    def __init__(self, agent, simulation):
        self.agent = agent
        self.simulation = simulation
        self.agent.set_radius(0.25)
        self.agent.assume_control()
        self.agent.disallow_adjustment()
        self.maxAcceleration = 1
        self.maxDeceleration = 1.6
        self.maxSpeed = 50/9
        self.minSpeed = 0.01
        self.turningRadius = 1.5
        self.avoidTime = 5
        self.avoidDistance = 7

    '''
    Constrains the pod's changes in heading to be within its turning radius allowance.
    '''
    def checkHeading(self, oldHeading, newHeading, speed):

        # this is required to allow pod's to rotate on the spot when spawned
        if speed < self.minSpeed:
            if self.agent.get_distance_traveled() == 0:
                return newHeading

        adjustedTurningRadius = self.turningRadius + (speed**2 * 0.4)
        travelDistance = speed / FRAME_RATE # distance the pod will travel in the next frame
        maxAllowedRotation = travelDistance / adjustedTurningRadius

        # ensure the dot product is valid (MassMotion can incur errors)
        dotProduct = newHeading.dot(oldHeading)
        if dotProduct > 1:
            dotProduct = 1
        if dotProduct < -1:
            dotProduct = -1
        proposedRotation = math.acos(dotProduct)

        if proposedRotation < maxAllowedRotation:
            return newHeading

        directionOfRotation = np.sign(newHeading.cross(oldHeading).get_y())
        return oldHeading.rotated_by(maxAllowedRotation*-1*directionOfRotation)

    '''
    Checks if the pod needs to slow down for aligned pedestrians.
    '''
    def slowDownAligned(self, distance, podSpeed, pedSpeed):

        podSpeed = podSpeed * 1.2
        if pedSpeed < 0:
            pedSpeed = pedSpeed*1.1
            podSpeed = podSpeed * 1.25
            offset = 2

        else:
            pedSpeed = pedSpeed*0.7
            podSpeed = podSpeed * 1.25
            offset = 1.5

        if (podSpeed - pedSpeed) ** 2 / (2 * self.maxDeceleration)\
                > distance - offset:
            return True
        return False

    def slowDownUnaligned(self, distance, podSpeed):

        podSpeed = podSpeed * 1.6
        offset = 2.6

        if ((podSpeed)** 2) / (2 * self.maxDeceleration) > distance - offset:
            return True
        return False

    '''
    Pod heads to its waypoint.
    '''
    def moveToWaypoint(self, podHeading, waypointHeading, podSpeed, podPosition):

        oldSpeed = podSpeed
        #self.agent.set_color(Color.GREEN)
        podSpeed = min(self.maxSpeed, podSpeed + self.maxAcceleration/FRAME_RATE)
        podHeading = self.checkHeading(podHeading, waypointHeading, podSpeed)
        velocity = podHeading * podSpeed
        newPosition = podPosition + velocity / FRAME_RATE

        # adjust the pod's new position to prevent it moving off the floor
        if self.agent.has_path_to_target_waypoint(newPosition) == False:
            newPosition = self.agent.get_open_point_closest_to(newPosition, OFF_FLOOR_SEARCH_RAD)

        # get the available space index from the heading angle
        headingAngle = math.atan2(podHeading.get_x(), podHeading.get_z()) # between -pi and pi, angle with positive 'x' axis, z=='x', x=='y'
        headingAngle = headingAngle + math.pi
        headingIndex = roundNearest(headingAngle)
        # if there is limited space to avoid the collision slow to stop
        availableSpace = self.agent.get_available_space(30)
        print(availableSpace.get_segment_direction(headingIndex))
        if availableSpace.get_segment_radius(headingIndex) - 0.5 < ((oldSpeed*1.2) ** 2) / (2 * self.maxDeceleration):
            self.slowToStop(podHeading, oldSpeed, podPosition)
            self.agent.set_color(Color.WHITE)
            return

        print((newPosition-podPosition).get_length())
        self.agent.move_to(newPosition, velocity)

    '''
    The pod slows down to its minimum speed.
    '''
    def slowToStop(self, podHeading, podSpeed, podPosition):

        podSpeed = max(self.minSpeed, podSpeed - self.maxDeceleration / FRAME_RATE)
        velocity = podHeading * podSpeed
        newPosition = podPosition + velocity / FRAME_RATE

        # adjust the pod's new position to prevent it moving off the floor
        if self.agent.has_path_to_target_waypoint(newPosition) == False:
            newPosition = self.agent.get_open_point_closest_to(newPosition, OFF_FLOOR_SEARCH_RAD)

        self.agent.move_to(newPosition, velocity)

    '''
    The pod navigates to avoid a collision with another agent.
    '''
    def avoidCollision(self, collisionPoint, podPosition, podHeading, podSpeed, waypointHeading):

        collisionVector = collisionPoint - podPosition
        sign = np.sign(collisionVector.cross(podHeading).get_y()) # direction for the edge vector
        edgeVector = collisionVector.rotated_by(sign*(math.pi / 2)).get_normalized() # vector to create the avoid point
        avoidPoint = collisionPoint + edgeVector* self.avoidDistance # new point for pod to head towards

        # if the avoid point is off the floor move it to the closest point that is on the floor
        if self.agent.has_path_to_target_waypoint(avoidPoint) == False:
            avoidPoint = self.agent.get_open_point_closest_to(avoidPoint, OFF_FLOOR_SEARCH_RAD)
            if avoidPoint.is_valid() == False:
                avoidPoint = podPosition + podHeading*(podSpeed+3) + edgeVector* self.avoidDistance
                self.agent.set_color(Color.WHITE)

        # check if the avoid point is near the edge of the floor
        if self.agent.get_closest_obstacle_point(0.5).is_valid() == True:
            # move it in 0.5m
            avoidPoint = avoidPoint + edgeVector.rotated_by(math.pi)*0.5

        newHeading = (avoidPoint - podPosition).get_normalized()
        newHeading = self.checkHeading(podHeading, newHeading, podSpeed)

        '''# if the proposed new heading pivots towards the collision vector then use 'move to waypoint'
        if np.sign(podHeading.cross(newHeading).get_y()) ==  np.sign(podHeading.cross(collisionVector).get_y()):
            self.moveToWaypoint(podHeading, waypointHeading, podSpeed, podPosition)
            self.agent.set_color(Color.BLACK)
            return'''

        newSpeed = min(self.maxSpeed, podSpeed + self.maxAcceleration / FRAME_RATE)
        podVelocity = newHeading * newSpeed
        newPosition = podPosition + podVelocity / FRAME_RATE

        # if proposed new position is off the floor shift it back on
        if self.agent.has_path_to_target_waypoint(newPosition) == False:
            newPosition = self.agent.get_point_with_path_to_target_waypoint_closest_to(newPosition, SPACE_SEARCH_RADIUS)

        # get the available space index from the heading angle
        headingAngle = math.atan2(newHeading.get_x(), newHeading.get_z())
        if headingAngle < 0:
            headingAngle = headingAngle + 2 * math.pi
        headingAngle = 2 * math.pi - headingAngle
        headingIndex = roundNearest(headingAngle)

        # if there is limited space to avoid the collision slow to stop
        availableSpace = self.agent.get_available_space(15)
        if availableSpace.get_segment_radius(headingIndex) < ((newSpeed*1.2)**2 / (2*self.maxDeceleration)) + 0.5:
            self.slowToStop(podHeading, podSpeed, podPosition)
            self.agent.set_color(Color.BLUE)
            return

        self.agent.set_color(Color.YELLOW)
        self.agent.move_to(newPosition, podVelocity)

    '''
    Determine if a non-aligned collision will occur and project the position of the pedestrian when the pod reaches the
    intersection of the agents' headings.
    '''
    def projectCollisionPoint(self, pedPosition, pedHeading, pedVelocity, pedSpeed, podPosition, podHeading, podSpeed):

        if podSpeed < 1.5:
            podSpeed = podSpeed + 0.5

        # no collision if pedestrian stationary
        if pedSpeed == 0:
            return False, False

        # calculate the intersection point using simultaneous equations
        A = np.array(
            [[podHeading.get_x(), -1 * pedHeading.get_x()], [podHeading.get_z(), -1 * pedHeading.get_z()]])
        B = np.array([pedPosition.get_x() - podPosition.get_x(), pedPosition.get_z() - podPosition.get_z()])
        C = np.linalg.solve(A, B)
        intersectionPoint = Vec3d(podPosition.get_x() + C[0] * podHeading.get_x(),
                                    podPosition.get_y(), podPosition.get_z() + C[0] * podHeading.get_z())
        podIntersectionVector = intersectionPoint - podPosition
        pedIntersectionVector = intersectionPoint - pedPosition
        distanceToCollision = podIntersectionVector.get_length()

        # exit if intersection is far away
        if distanceToCollision > 30:
            return False, False

        # no collision if either agent is heading away from the intersection point
        if podIntersectionVector.dot(podHeading) < 0 or pedIntersectionVector.dot(pedHeading) < 0:
            return False, False

        timeToCollision = distanceToCollision / podSpeed
        avoidTime = 1 + podSpeed*1.4 # 6 is good when far off

        '''# exit if collision is still a long time away
        if timeToCollision > avoidTime:
            return False, False'''

        collisionPoint = pedPosition + pedVelocity * timeToCollision
        print(collisionPoint, intersectionPoint)
        return collisionPoint, intersectionPoint

    '''
    Evaluates where the pod should move to in the next frame of the simulation.
    '''
    def step(self):

        self.agent.clear_color()
        self.agent.set_color(Color.GREEN)

        # exit if pod does not have a target (MassMotion will assign this)
        if not self.agent.has_target_waypoint():
            return

        # exit simulation if pod is near its goal (MassMotion not tuned to permit fast moving agents to exit)
        podPosition = self.agent.get_position()
        goalLine = self.agent.get_target_waypoint_goal_line()
        waypointHeading = (goalLine.get_midpoint() - podPosition).get_normalized()
        goalBox = goalLine.get_bounding_box()
        if goalBox.contains(podPosition, GOAL_BOX_RAD):
            self.agent.exit_simulation()

        # align recently spawned pod's heading with the waypoint heading
        distanceTravelled = self.agent.get_distance_traveled()
        if  distanceTravelled < 0.1:
            self.agent.move_to(podPosition + waypointHeading*0.1, waypointHeading)
            return

        # if pod has moved off the floor then shift it back on
        if self.agent.is_in_open_space() == False:
            self.agent.move_to(self.agent.get_closest_open_point(OFF_FLOOR_SEARCH_RAD), waypointHeading)
            return

        # ensure podSpeed and podHeading are valid
        podVelocity = self.agent.get_velocity()
        if podVelocity.is_valid() == True:
            podSpeed = podVelocity.get_length()
            podHeading = podVelocity.get_normalized()
        else:
            podSpeed = 0
            podHeading = waypointHeading

        # identify nearby pedestrians
        nearAgents = self.simulation.get_agents_near_point(podPosition, POD_SEARCH_CYLINDER_RAD,
                                                           POD_SEARCH_CYLINDER_HEIGHT)
        closestPedPosition = False
        nearPedestrians = []
        for nearAgent in nearAgents:

            # ignore itself
            if nearAgent.get_id() == self.agent.get_id():
                continue

            # ignore agents which have not yet been given a target
            if not nearAgent.has_target_waypoint():
                continue

            nearPedestrians.append(nearAgent.get_id())
            pedPosition = nearAgent.get_position()

            # ensure pedSpeed and pedHeading are valid
            pedVelocity = nearAgent.get_velocity()
            if pedVelocity.is_valid() == True:
                pedSpeed = pedVelocity.get_length()
                pedHeading = pedVelocity.get_normalized()
            else:
                pedSpeed = 0
                pedHeading = False

            pedVector = pedPosition - podPosition
            pedDistance = pedVector.get_length()

            # slow to stop if a pedestrian is in the pod's emergency zone
            vertex1 = podPosition + podHeading.rotated_by(math.pi / 2) * 1.3 + Vec3d(0, -1, 0) + (podHeading.rotated_by(math.pi) * 0.6)
            vertex2 = podPosition + podHeading.rotated_by((-1 * math.pi) / 2) * 1.3 + Vec3d(0, 1, 0) + (podHeading * (2.5 + podSpeed * 0.36))
            emergencyBox = vertex1.hull(vertex2)
            if emergencyBox.contains(pedPosition):
                self.slowToStop(podHeading, podSpeed, podPosition)
                self.agent.set_color(Color.RED)
                return nearPedestrians

            # check if the pedestrian is in the pod's aligned region
            vertex3 = podPosition + podHeading.rotated_by(math.pi / 2) * 1.35 + Vec3d(0, -1, 0) + podHeading*1.2
            vertex4 = podPosition + podHeading.rotated_by((-1*math.pi)/2)*1.4 + podHeading*AlIGNED_BOX_LENGTH + Vec3d(0,1,0)
            alignedBox = vertex3.hull(vertex4)
            if (pedVector.dot(podHeading) / pedDistance > COS_20D or alignedBox.contains(pedPosition)):

                # check if agent headings are aligned
                if (podHeading.dot(pedHeading) < MINUS_COS_20D or podHeading.dot(pedHeading) > COS_20D):

                    # check if the pod needs to slow down to a stop
                    if self.slowDownAligned(pedVector.get_length(), podVelocity.dot(pedVector) / pedVector.get_length(),
                                pedVelocity.dot(pedVector) / pedVector.get_length()):
                        self.slowToStop(podHeading, podSpeed, podPosition)
                        self.agent.set_color(Color.ORANGE)
                        return nearPedestrians

            # else check the unaligned region
            if pedVector.dot(podHeading) / pedDistance > COS_90D:
                self.agent.set_color(Color.GREEN)

                # check if there is an intersection of the pod's and pedestrian's heading vectors
                projectedPosition, intersectionPoint = self.projectCollisionPoint(pedPosition, pedHeading, pedVelocity, pedSpeed, podPosition, podHeading, podSpeed)

                if projectedPosition != False:

                    # check if pod needs to slow to a stop
                    avoidRadius = 3 + 0.5*podSpeed
                    if (projectedPosition - intersectionPoint).get_length() < avoidRadius:
                        self.agent.set_color(Color.WHITE)
                        if self.slowDownUnaligned((projectedPosition - podPosition).get_length(), podSpeed):
                            self.slowToStop(podHeading, podSpeed, podPosition)
                            self.agent.set_color(Color.BLUE)
                            return nearPedestrians

                    # check if this is the closest pedestrian to the pod so it knows which pedestrian to avoid
                    projectedUnalignedVector = projectedPosition - podPosition
                    projectedUnalignedDistance = projectedUnalignedVector.get_length()
                    if projectedUnalignedVector.dot(podHeading) / projectedUnalignedDistance > COS_45D or alignedBox.contains(projectedPosition):
                        if closestPedPosition == False or projectedUnalignedDistance < (closestPedPosition - podPosition).get_length():
                            closestPedPosition = projectedPosition

            # check if the pedestrian is in the pod's avoid region
            if pedVector.dot(podHeading) / pedDistance > COS_45D or alignedBox.contains(pedPosition):

                # check if agent headings are aligned
                if (podHeading.dot(pedHeading) < MINUS_COS_30D or podHeading.dot(pedHeading) > COS_30D):

                    # check if this is the closest pedestrian to the pod so it knows which pedestrian to avoid
                    alignedDistance = pedVector.get_length()
                    alignedPodSpeed = podVelocity.dot(pedVector) / alignedDistance
                    alignedPedSpeed = pedVelocity.dot(pedVector) / alignedDistance
                    alignedTime = alignedDistance / (alignedPodSpeed - alignedPedSpeed)

                    if alignedTime < 0:
                        self.moveToWaypoint(podHeading, waypointHeading, podSpeed, podPosition)

                    podVector = self.agent.get_position() - pedPosition
                    projectedAlignedPosition = pedPosition + alignedPedSpeed * podVector.get_normalized() * alignedTime
                    projectedAlignedDistance = projectedAlignedPosition.get_length()

                    if closestPedPosition == False or projectedAlignedDistance < (closestPedPosition - podPosition).get_length():
                        closestPedPosition = pedPosition #projectedAlignedPosition

        # if there is no close pedestrian to avoid: move to waypoint
        if closestPedPosition == False:
            self.moveToWaypoint(podHeading, waypointHeading, podSpeed, podPosition)

        # else avoid a collision with the closest pedestrian
        else:
            self.avoidCollision(closestPedPosition, podPosition, podHeading, podSpeed, waypointHeading)

        return nearPedestrians