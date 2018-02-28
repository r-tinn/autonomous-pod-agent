from massmotion import *
from pod import pod
import random
from pedestrian import analyseNearestPod
from random import randint

def main():

    Sdk.init()
    Sdk.set_output_level(Sdk.VERBOSE)
    name = 'simpleTest2'
    project = Project.open(name +'.mm')
    podEntryTimes=[] # list to store times that pods enter the simulation

    portal1 = project.get_portal('Portal1')
    portal2 = project.get_portal('Portal2')
    portal3 = project.get_portal('Portal3')

    pod_profile = project.get_profile('PodProfile')

    pod_request1 = AgentRequest(portal1.get_id())
    pod_request1.set_profile(pod_profile.get_id())
    pod_request1.set_goal(portal2.get_id())

    pod_request2 = AgentRequest(portal2.get_id())
    pod_request2.set_profile(pod_profile.get_id())
    pod_request2.set_goal(portal1.get_id())

    pod_request3 = AgentRequest(portal3.get_id())
    pod_request3.set_profile(pod_profile.get_id())
    pod_request3.set_goal(portal2.get_id())

    pod_request4 = AgentRequest(portal2.get_id())
    pod_request4.set_profile(pod_profile.get_id())
    pod_request4.set_goal(portal3.get_id())

    pod_request = [pod_request1, pod_request2, pod_request3, pod_request4]

    pods = {}
    simulation = Simulation.create(project, name,  name + '.mmdb')
    clock = simulation.get_clock()
    podCount = 0
    oldPedestriansIDs = {}

    while not simulation.is_done():

        podCheck = False
        releasePedestrians = []
        pedestrianIDs = {}

        # call step method on each pod
        for agent in simulation.get_all_agents():
            if agent.get_profile_id() == pod_profile.get_id():
                podCheck = True
                try:
                    closePedestrians = pods[agent.get_id()].step()
                    if closePedestrians:
                        for ID in closePedestrians:
                            pedestrianIDs[ID] = random.random()
                            if ID not in oldPedestriansIDs:
                                oldPedestriansIDs[ID] = pedestrianIDs[ID]

                except KeyError:
                    pods[agent.get_id()] = pod(agent, simulation)
                    closePedestrians = pods[agent.get_id()].step()
                    if closePedestrians:
                        for ID in closePedestrians:
                            pedestrianIDs[ID] = random.random()
                            if ID not in oldPedestriansIDs:
                                oldPedestriansIDs[ID] = pedestrianIDs[ID]

        # determine which pedestrians are no longer close to pods that were in previous frames
        oldPedestriansIDsList = list(oldPedestriansIDs.keys())
        pedestrianIDsList = list(pedestrianIDs.keys())
        for ID in oldPedestriansIDsList:
            if ID not in pedestrianIDsList:
                releasePedestrians.append(ID)
                del oldPedestriansIDs[ID]

        # if there are no pods in the simulation
        if podCheck == False:
            rand = randint(0, 3)
            simulation.request_new_agent(pod_request[rand])
            podCount = podCount + 1
            podEntryTimes.append(clock.get_current_second())

        # take control over all pedestrians which are close to pods
        for agent in simulation.get_all_agents():
            if agent.get_id() in oldPedestriansIDs:
                analyseNearestPod(agent, simulation, oldPedestriansIDs[agent.get_id()], pod_profile)
                continue

            # release control of pedestrians which are no longer close to pods
            if agent.get_id() in releasePedestrians:
                agent.release_control()
                agent.clear_color()

        simulation.step()

    Sdk.fini()

    print("Number of pods that entered simulation: ", podCount)
    print("Times that pods entered simulation: ", podEntryTimes)
    delta = podEntryTimes[-1]-podEntryTimes[0] # when pods enter simulation
    distance = (portal1.get_goal_line().get_midpoint() - portal2.get_goal_line().get_midpoint()).get_length()
    print("Average pod speed: ", distance / (delta / (podCount-1)))

if __name__ == '__main__':
    main()