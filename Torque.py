import math

# angle_values = []
# torque_values = []

for i in range(0,31):
    
    # Iterating through angles 
    angle = 40-i

    maxTiltAngle = angle *(math.pi/180) #Have to convert to radians for python

    #Everything in SI Units
    diskMass = 0.2
    radiusDisk = 0.07
    RPM = 400
    #maxTiltAngle = 30 *(math.pi/180) #Have to convert to radians for python
    massSystem = 0.2704
    g = 9.81
    centreMassFromGround = 0.07

    angularSpeedDisk = RPM / (60/(2*math.pi))

    inertiaDisk = (diskMass*radiusDisk**2)/2

    highestPrecisionSpeed = (massSystem*g*centreMassFromGround*math.sin(maxTiltAngle)) / (inertiaDisk*angularSpeedDisk)

    torque = inertiaDisk*angularSpeedDisk*highestPrecisionSpeed

    # print("Inertia of Disk = ", inertiaDisk, "kg m2") # [N*m]
    # print("Wp = ", highestPrecisionSpeed, "rad/sec")
    # print("Torque = ", torque, "kg m2/sec2")
    
    print("Angle: ",angle," Torque:",torque)
    #print("Torque: \n",torque)

    # angle_values.append(angle)
    # torque_values.append(torque)



