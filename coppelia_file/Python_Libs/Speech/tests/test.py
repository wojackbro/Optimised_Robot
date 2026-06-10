from time import sleep
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')
sim.setStepping(True)

sensorHandle = sim.getObject('/MiddleSensor')

sim.startSimulation()
while sim.getSimulationTime() < 10:
    image, resolution = sim.getVisionSensorImg(sensorHandle)
    print(f'image: {resolution}')
    sim.step()
    #sim.setVisionSensorImg(sensor2Handle, image)
sim.stopSimulation()
