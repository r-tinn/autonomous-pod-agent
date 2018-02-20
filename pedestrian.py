import math
import numpy as np
from massmotion import *
from pod import POD_SEARCH_CYLINDER_RAD, FRAME_RATE, OFF_FLOOR_SEARCH_RAD, COS_30D, COS_20D, COS_45D, MINUS_COS_20D, COS_60D, COS_25D, MINUS_COS_25D, MINUS_COS_30D

'''
Constants defined for the pedestrian agent.
'''
PED_SEARCH_CYLINDER_RAD = POD_SEARCH_CYLINDER_RAD + 3 # safety factor of 2 ensures the closest pod is always picked up
PED_SEARCH_CYLINDER_HEIGHT = 5 # height of the search cylinder
AVOID_TIME_SCALING_FACTOR = 4
AVOID_DISTANCE_SCALING_FACTOR = 3
AVOID_DISTANCE_OFFSET = 4
ALIGNED_AVOID_TIME_OFFSET = 2
STATIONARY_SAFETY_RADIUS = 3
PED_EMERGENCY_RADIUS = 3.5

def defaultBehaviour(agent, pedPosition, waypointHeading, pedSpeed):

    if pedSpeed < 1:
        pedSpeed = 1

    agent.set_color(Color.WHITE)
    agent.assume_control()
    agent.move_to(pedPosition + waypointHeading * (pedSpeed / FRAME_RATE))

def moveAside(agent, pedPosition, pedHeading, pedSpeed, podPosition, podVector, behaviour):

    sign = np.sign(podVector.cross(pedHeading).get_y()) * -1  # direction for the edge vector
    edgeVector = podVector.rotated_by(sign * math.pi / 2).get_normalized()  # vector used to create the avoid point
    avoidDistance = 3 #(behaviour * AVOID_DISTANCE_SCALING_FACTOR) + AVOID_DISTANCE_OFFSET # distance to avoid pod by
    avoidPoint = pedPosition + pedHeading*6 + avoidDistance*edgeVector  # new point for pedestrian to head towards

    # if the avoid point is off the floor move it to the closest point that is on the floor
    if agent.has_path_to_target_waypoint(avoidPoint) == False:
        avoidPoint = agent.get_open_point_closest_to(avoidPoint, OFF_FLOOR_SEARCH_RAD)

    newPedHeading = (avoidPoint - pedPosition).get_normalized()
    agent.assume_control()
    agent.move_to(pedPosition + newPedHeading * (pedSpeed / FRAME_RATE))

'''
Calculates the new heading for a pedestrian so that it avoids a collision with a pod.
'''
def calculateNewPedestrianHeading(agent, pedPosition, pedHeading, podPosition, podVector, behaviour, emergency):

    sign = np.sign(podVector.cross(pedHeading).get_y())  # direction for the edge vector
    edgeVector = podVector.rotated_by(sign * math.pi / 2).get_normalized()  # vector used to create the avoid point

    if emergency:
        avoidDistance = (behaviour * AVOID_DISTANCE_SCALING_FACTOR) + AVOID_DISTANCE_OFFSET # distance to avoid pod by

    else:
        avoidDistance = 3

    avoidPoint = podPosition + avoidDistance * edgeVector  # new point for pedestrian to head towards

    # if the avoid point is off the floor move it to the closest point that is on the floor
    if agent.has_path_to_target_waypoint(avoidPoint) == False:
        avoidPoint = agent.get_open_point_closest_to(avoidPoint, OFF_FLOOR_SEARCH_RAD)

    if (avoidPoint - podPosition).get_length() < 1:
        edgeVector = podVector.rotated_by(sign * -1 * math.pi / 2).get_normalized()  # vector used to create the avoid point
        avoidPoint = podPosition + avoidDistance * edgeVector  # new point for pedestrian to head towards

    newPedHeading = (avoidPoint - pedPosition).get_normalized()

    return newPedHeading

'''
Pedestrian to avoid pod using the 'aligned collision' algorithm.
'''
def avoidAlignedCollision(agent, pedPosition, pedHeading, pedSpeed, podPosition, podVector, behaviour, emergency):

    newPedHeading = calculateNewPedestrianHeading(agent, pedPosition, pedHeading, podPosition, podVector, behaviour, emergency)
    agent.assume_control()
    agent.set_color(Color.YELLOW)
    agent.move_to(pedPosition + newPedHeading * (pedSpeed / FRAME_RATE))

'''
Pedestrian to avoid pod using the 'non-aligned collision' algorithm.
'''
def avoidNonAlignedCollision(agent, pedPosition, pedHeading, pedSpeed, podPosition, podHeading, podVelocity, podSpeed,
                             behaviour, avoidTime, waypointHeading):

    if podSpeed == 0:
        projectedPodPosition = podPosition

    else:
        # find the intersection of the pedestrian and pods' headings using simultaneous equations
        A = np.array([[pedHeading.get_x(), -1*podHeading.get_x()], [pedHeading.get_z(), -1*podHeading.get_z()]])
        B = np.array([podPosition.get_x() - pedPosition.get_x(), podPosition.get_z() - pedPosition.get_z()])
        C = np.linalg.solve(A, B)
        headingIntersection = Vec3d(pedPosition.get_x() + C[0]*pedHeading.get_x(),
                                   pedPosition.get_y(),pedPosition.get_z() + C[0]*pedHeading.get_z())

        pedIntersectionVector = headingIntersection - pedPosition
        podIntersectionVector = headingIntersection - podPosition

        # exit if either agent is heading away from the heading intersection point
        if pedIntersectionVector.dot(pedHeading) < 0 or podIntersectionVector.dot(podHeading) < 0 :
            defaultBehaviour(agent, pedPosition, waypointHeading, pedSpeed)
            return

        pedTimeToCollision = pedIntersectionVector.get_length() / pedSpeed

        # exit if the time to collision is greater than the avoid time for the pedestrian
        if pedTimeToCollision > avoidTime:
            defaultBehaviour(agent, pedPosition, waypointHeading, pedSpeed)
            return

        projectedPodPosition = podPosition + podVelocity * pedTimeToCollision

    projectedPodVector = projectedPodPosition - pedPosition

    #return if not heading towards the pod
    if projectedPodVector.dot(pedHeading) < COS_30D:
        defaultBehaviour(agent, pedPosition, waypointHeading, pedSpeed)
        return

    newPedHeading = calculateNewPedestrianHeading(agent, pedPosition, pedHeading, projectedPodPosition,
                                                  projectedPodVector, behaviour, False)
    agent.assume_control()
    agent.move_to(pedPosition + newPedHeading * (pedSpeed / FRAME_RATE))

'''
Find the closest pod to the pedestrian in order to determine whether the pedestrian should use its default
behaviour or avoid a collision with a pod.
'''
def analysePodLocation(agent, simulation, behaviour, podProfile):

    # set the pedestrian to use its default behaviour
    agent.release_control()
    agent.clear_color()

    # exit if pedestrian does not have a target (MassMotion will assign this)
    if not agent.has_target_waypoint():
        agent.set_color(Color.BLACK)
        return

    pedPosition = agent.get_position()

    # ensure pedSpeed and pedHeading are valid
    pedVelocity = agent.get_velocity()

    goalLine = agent.get_target_waypoint_goal_line()
    waypointHeading = (goalLine.get_midpoint() - pedPosition).get_normalized()

    if pedVelocity.is_valid() == True:
        pedSpeed = pedVelocity.get_length()
        pedHeading = pedVelocity.get_normalized()
    else:
        pedSpeed = 0
        pedHeading = False

    # identify nearest pod
    closestPodDistance = PED_SEARCH_CYLINDER_RAD
    nearAgents = simulation.get_agents_near_point(pedPosition, PED_SEARCH_CYLINDER_RAD,
                                                  PED_SEARCH_CYLINDER_HEIGHT)
    for nearAgent in nearAgents:
        # check if the nearby agent is a pod
        if nearAgent.get_profile_id() == podProfile.get_id():
            if (nearAgent.get_position() - pedPosition).get_length() < closestPodDistance:
                podPosition = nearAgent.get_position()
                podVelocity = nearAgent.get_velocity()
                podSpeed = podVelocity.get_length()
                closestPodDistance = (nearAgent.get_position() - pedPosition).get_length()

    # exit if pedestrian is not on the floor
    if agent.get_direction_to_target_waypoint().is_valid() == False:
        agent.assume_control()
        agent.set_next_movement_type(Agent.STANDING)
        agent.set_color(Color.BLACK)
        agent.move_to(agent.get_closest_point_with_path_to_target_waypoint(OFF_FLOOR_SEARCH_RAD))
        return

    podHeading = podVelocity.get_normalized()
    podVector = podPosition - pedPosition
    pedVector = pedPosition - podPosition
    avoidTime = (behaviour * AVOID_TIME_SCALING_FACTOR)

    # check if a pod is within a pedestrian's emergency zone and its heading is within 60 degrees of the pod vector
    if closestPodDistance < PED_EMERGENCY_RADIUS and (podVector.dot(pedHeading) / podVector.get_length()) > COS_60D:
        avoidAlignedCollision(agent, pedPosition, pedHeading, pedSpeed, podPosition, podVector, behaviour, True)
        agent.set_color(Color.RED)
        return

    # check if a pod is in the pedestrian's front aligned region
    if (pedHeading.dot(podVector) / (podVector.get_length())) > COS_20D:

        # if pod heading towards pedestrian and collision is imminent then navigate to avoid point
        if(podHeading.dot(pedVector) / (pedVector.get_length())) > COS_20D:
            timeToCollision = podVector.get_length() / (
            pedVelocity.dot(podVector.get_normalized()) + podVelocity.dot(pedVector.get_normalized()))

            # return if collision point is remote
            if timeToCollision > avoidTime + ALIGNED_AVOID_TIME_OFFSET:
                agent.set_color(Color.BLUE)
                defaultBehaviour(agent, pedPosition, waypointHeading, pedSpeed)
                return

            avoidAlignedCollision(agent, pedPosition, pedHeading, pedSpeed, podPosition, podVector, behaviour, False)
            return

        # if pod heading away from pedestrian and collision is imminent then navigate to avoid point
        elif (podHeading.dot(pedVector) / (podHeading.get_length() * pedVector.get_length())) < MINUS_COS_20D:

            timeToCollision = podVector.get_length() / (
            pedVelocity.dot(podVector.get_normalized()) - podVelocity.dot(pedVector.get_normalized()))

            # return if pod is moving faster away
            if timeToCollision < 0:
                defaultBehaviour(agent, pedPosition, waypointHeading, pedSpeed)
                return

            # return if collision point is remote
            if timeToCollision > avoidTime:
                defaultBehaviour(agent, pedPosition, waypointHeading, pedSpeed)
                return

            avoidAlignedCollision(agent, pedPosition, pedHeading, pedSpeed, podPosition, podVector, behaviour, False)
            return

    # check if a pod is in the pedestrian's rear aligned region
    if (pedHeading.dot(podVector) / podVector.get_length()) < MINUS_COS_30D:
        agent.set_color(Color.GREEN)

        # 5% of pedestrians won't move aside
        '''if behaviour < 0.05:
            defaultBehaviour(agent, pedPosition, waypointHeading, pedSpeed)
            return'''

        # if pod heading towards pedestrian and collision is imminent then navigate to avoid point
        if (podHeading.dot(pedVector) / (pedVector.get_length())) > COS_30D:
            timeToCollision = podVector.get_length() / (pedVelocity.dot(podVector.get_normalized()) + podVelocity.dot(pedVector.get_normalized()))

            # return if collision point is remote
            if timeToCollision > avoidTime + ALIGNED_AVOID_TIME_OFFSET:
                defaultBehaviour(agent, pedPosition, waypointHeading, pedSpeed)
                return
            agent.set_color(Color.BLACK)
            moveAside(agent, pedPosition, waypointHeading, pedSpeed, podPosition, podVector, behaviour)
            return

    avoidNonAlignedCollision(agent, pedPosition, pedHeading, pedSpeed, podPosition, podHeading, podVelocity, podSpeed,
                             behaviour, avoidTime, waypointHeading)

