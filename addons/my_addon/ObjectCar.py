import numpy as np

class ObjectCar :

    def __init__(self, ob):
        self.ob=ob
        self.facingAngle =  ob.rotation_euler[2]
        self.positionXYZ = [ob.location[0],ob.location[1],ob.location[2]]
        self.lineSensor = []
        self.lineSensor.append([0,0])
        self.lineSensor.append([0,0])
        self.lineSensor.append([0,0])
        self.lineSensor.append([0,0])
        self.lineSensor.append([0,0])
        self.speed = 0
        
    def updateOb(self):
        self.ob.location[0] =self.positionXYZ[0] 
        self.ob.location[1] =self.positionXYZ[1] 
        self.ob.location[2] =self.positionXYZ[2]
        self.ob.rotation_euler[2] = self.facingAngle 
        self.updateSensorPos()
        
    def updateSensorPos(self):
        distanceX = 13.57
        distanceY = 1.8
        distanceXDist = 14.82
        # Sensor Ligne
        angle1 = np.tan(distanceY/distanceX)
        angle2 = np.tan(2 * distanceY/distanceX)
        self.lineSensor[0] = [(self.ob.location[0]+distanceX*np.cos(self.ob.rotation_euler[2]))*np.cos(angle2),
                            (self.ob.location[0]+distanceY*np.sin(self.ob.rotation_euler[2]))*np.sin(angle2)]
        self.lineSensor[1] = [(self.ob.location[0]+distanceX*np.cos(self.ob.rotation_euler[2]))*np.cos(angle1),
                            (self.ob.location[0]+distanceY*np.sin(self.ob.rotation_euler[2]))*np.sin(angle1)]
        self.lineSensor[2] = [self.ob.location[0]+distanceX*np.cos(self.ob.rotation_euler[2]),
                                self.ob.location[0]+distanceY*np.sin(self.ob.rotation_euler[2])]
        self.lineSensor[3] = [(self.ob.location[0]+distanceX*np.cos(self.ob.rotation_euler[2]))*np.cos(-angle1),
                            (self.ob.location[0]+distanceY*np.sin(self.ob.rotation_euler[2]))*np.sin(-angle1)]
        self.lineSensor[4] = [(self.ob.location[0]+distanceX*np.cos(self.ob.rotation_euler[2]))*np.cos(-angle2),
                            (self.ob.location[0]+distanceY*np.sin(self.ob.rotation_euler[2]))*np.sin(-angle2)]
        
        #Sensor distance
        self.distSensor = [self.ob.location[0]+distanceXDist*np.cos(self.ob.rotation_euler[2]),self.ob.location[0]+distanceXDist*np.sin(self.ob.rotation_euler[2])]
               