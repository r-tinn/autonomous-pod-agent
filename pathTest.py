from massmotion import *
from pod import pod
import random
from pedestrian import analyseNearestPod

def main():

    Sdk.init()
    Sdk.set_output_level(Sdk.VERBOSE)
    name = 'simpleTest'
    project = Project.open(name +'.mm')
    podEntryTimes=[] # list to store times that pods enter the simulation

    start_portal = project.get_portal('Portal1')
    end_portal = project.get_portal('Portal2')

    pod_profile = project.get_profile('PodProfile')
    pod_request = AgentRequest(start_portal.get_id())
    pod_request.set_profile(pod_profile.get_id())
    pod_request.set_goal(end_portal.get_id())

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
            simulation.request_new_agent(pod_request)
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
    distance = (start_portal.get_goal_line().get_midpoint() - end_portal.get_goal_line().get_midpoint()).get_length()
    print("Average pod speed: ", distance / (delta / (podCount-1)))

if __name__ == '__main__':
    main()