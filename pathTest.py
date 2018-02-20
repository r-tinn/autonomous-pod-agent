from massmotion import *
from pod import pod
import random
from pedestrian import analysePodLocation

def main():
    Sdk.init()
    Sdk.set_output_level(Sdk.VERBOSE)
    name = 'pathTest'

    project = Project.open('pathTest.mm')
    t=[]

    start_portal = project.get_portal('Portal1')
    end_portal = project.get_portal('Portal2')

    pod_profile = project.get_profile('PodProfile')
    pod_request = AgentRequest(start_portal.get_id())
    pod_request.set_profile(pod_profile.get_id())
    pod_request.set_goal(end_portal.get_id())

    pods = {}
    simulation = Simulation.create(project, 'pathTest',  'pathTest.mmdb')
    clock = simulation.get_clock()
    podCount = 0

    oldPedestriansIDs = {}
    i =0.2
    while not simulation.is_done():

        if podCount== 11:
            Sdk.fini()

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

        if podCheck == False and i > 60*5:
            simulation.request_new_agent(pod_request)
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
    delta = t[-1]-t[0] # when pods enter simulation
    print(delta)
    print(460 / (delta / (podCount-1)))

if __name__ == '__main__':
    main()