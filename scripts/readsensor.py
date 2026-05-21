def sysCall_init():
    sim = require('sim')
    self.objHandle = sim.getObject('..')
    self.leftSensor = sim.getObject("../LeftSensor")
    self.middleSensor = sim.getObject("../MiddleSensor")
    self.rightSensor = sim.getObject("../RightSensor")
    self.intLeftSensor = sim.getObject("../InternalLeftSensor")
    self.intRightSensor = sim.getObject("../InternalRightSensor")

    self.centralProximitySensor = sim.getObject("../CentralProximitySensor")
    self.leftProximitySensor = sim.getObject("../LeftProximitySensor")
    self.rightProximitySensor = sim.getObject("../RightProximitySensor")
    self.hasProximitySensor = True
    self.proximity_threshold = [ 0.20, 0.20, 0.20 ]

    self.graph = sim.getObject('/graph')
    self.ls = sim.addGraphStream(self.graph, 'left sensor', 'bool', 0, [1.0, 0.0, 0.0])
    self.ms = sim.addGraphStream(self.graph, 'middle sensor', 'bool', 0, [0.0, 1.0, 0.0])
    self.rs = sim.addGraphStream(self.graph, 'right sensor', 'bool', 0, [0.0, 0.0, 1.0])
    self.ils = sim.addGraphStream(self.graph, 'internal left sensor', 'bool', 0, [1.0, 0.0, 0.0])
    self.irs = sim.addGraphStream(self.graph, 'internal right sensor', 'bool', 0, [0.0, 0.0, 1.0])

def sysCall_sensing():
    # Send 'Vision Sensor' message in broadcast
    lf_result, lf_pk1, lf_pk2 = sim.readVisionSensor(self.leftSensor)
    md_result, md_pk1, md_pk2 = sim.readVisionSensor(self.middleSensor)
    rt_result, rt_pk1, rt_pk2 = sim.readVisionSensor(self.rightSensor)
    ilf_result, ilf_pk1, ilf_pk2 = sim.readVisionSensor(self.intLeftSensor)
    irt_result, irt_pk1, irt_pk2 = sim.readVisionSensor(self.intRightSensor)

    sens_msg = {
        'id': 'sensor_reading',
        'data': [
            lf_pk1,
            md_pk1,
            rt_pk1,
            ilf_pk1,
            irt_pk1
        ]
    }
    sim.broadcastMsg(sens_msg)

    sim.setGraphStreamValue(self.graph, self.ls, (sens_msg['data'][0][10]> 0.5) + 0)
    sim.setGraphStreamValue(self.graph, self.ms, (sens_msg['data'][1][10]> 0.5) + 2)
    sim.setGraphStreamValue(self.graph, self.rs, (sens_msg['data'][2][10]> 0.5) + 4)
    sim.setGraphStreamValue(self.graph, self.ils, (sens_msg['data'][3][10]> 0.5) + 6)
    sim.setGraphStreamValue(self.graph, self.irs, (sens_msg['data'][4][10]> 0.5) + 8)

    # Send 'Proximity Sensor' message in broadcast
    ps_handle = [ self.centralProximitySensor, self.leftProximitySensor, self.rightProximitySensor ]
    detected = [ False, False, False ]
    distance = [ float('inf'), float('inf'), float('inf') ]

    for i in range(0,3):
        prox_result, prox_distance, prox_point, prox_objHandle, prox_normal = sim.readProximitySensor(ps_handle[i])
        if prox_result > 0:
            distance[i] = prox_distance
            detected[i] = (distance[i] < self.proximity_threshold[i])

    prox_msg = {
        'id': 'proximity',
        'data': [
            [ detected[0], distance[0], self.proximity_threshold[0] ],
            [ detected[1], distance[1], self.proximity_threshold[1] ],
            [ detected[2], distance[2], self.proximity_threshold[2] ]
        ]
    }
    sim.broadcastMsg(prox_msg)

def sysCall_cleanup():
    sim.resetVisionSensor(sim.handle_all_except_explicit)
