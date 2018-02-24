# An agent based simulation of the interactions between autonomous vehicles and pedestrians

Autonomous vehicles are increasingly becoming a viable transport solution. The goal of this project is to evaluate how
autonomous vehicles, which operate in pedestrianised areas, could be used within cities in the future.

### Getting Started (this requires the acquisition of a licence for MassMotion and its SDK)
1) Install MassMotion and its SDK (http://www.oasys-software.com/products/engineering/massmotion.html)
2) Clone this repo into MassMotion SDK's path, this contains the massmotion.py file.
This looks like 'C:\MassMotion SDK 9.0' in Windows
3) Run either pathTest.py or squareTest.py (note you must be using Python 3.4 to run MassMotion's SDK)

### Changing the tests
Changes to the environment can be made by editing the pathTest.mm and squareTest.mm files.
This includes altering the number of pedestrians travelling between goals and the positions of the goals themselves.
The pod's and pedestrian's attributes and parameters can be edited by altering the corresponding python files.
A class diagram has also illustrates how these extensions fit into the wider MassMotion environment.
The video exampleJourney.mp4 shows a larger (Sketchup) model imported into MassMotion which pods can travel through.

### Author
Robert Tinn

### Acknowledgments
The team at Oasys for their guidance and SDK