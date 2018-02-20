from massmotion import *
from pod import pod
import random
from pedestrian import analysePodLocation
from random import randint

def main():
    Sdk.init()
    Sdk.set_output_level(Sdk.VERBOSE)
    name = 'squareTest'

    project = Project.open(name + '.mm')
    t=[]

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
    simulation = Simulation.create(project, 'name', name + '.mmdb')
    clock = simulation.get_clock()
    podCount = 0

    oldPedestriansIDs = {}
    i =0.2
    while not simulation.is_done():
        print(i)
        i = i+0.2

        podCheck = False
        releasePedestrians = []
        pedestrianIDs = {}

        # step forward each pod
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

        oldPedestriansIDsList = list(oldPedestriansIDs.keys())
        pedestrianIDsList = list(pedestrianIDs.keys())
        for ID in oldPedestriansIDsList:
            if ID not in pedestrianIDsList:
                releasePedestrians.append(ID)
                del oldPedestriansIDs[ID]

        if podCheck == False:
            rand = randint(0, 3)
            simulation.request_new_agent(pod_request[rand])
            #print("pod")
            podCount = podCount + 1
            t.append(clock.get_current_second())

        for agent in simulation.get_all_agents():
            if agent.get_id() in oldPedestriansIDs:
                analysePodLocation(agent, simulation, oldPedestriansIDs[agent.get_id()], pod_profile)
                continue

            if agent.get_id() in releasePedestrians:
                agent.release_control()
                agent.clear_color()

        simulation.step()

    Sdk.fini()
    print(podCount)
    print(t)
    delta = t[-1]-t[1]
    print(delta)
    print(48.8 / (delta / (podCount-2)))

if __name__ == '__main__':
    main()