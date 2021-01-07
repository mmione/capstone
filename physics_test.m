% Initializing arrays for calculations % 

angles = [10:1:40]';
masses = [];
torques = [];

for massSystem = [0.2704:0.025:1] % Loop from minimum (w/o battery) to 1kg 
    
    % Givens %

    diskMass = 0.2;
    radiusDisk = 0.07;
    RPM = 400;
    %massSystem = 0.2704;
    g=9.81;
    centerOfMassFromGround = 0.06;

    angularSpeedDisk = RPM*2*pi/60;
    
    % Calculations %
    
    inertiaDisk = (diskMass*radiusDisk^2)/2;
    
    highestPrecisionSpeed = (massSystem*g*centerOfMassFromGround*sind(angles))/(inertiaDisk*angularSpeedDisk);
    torque = (inertiaDisk*angularSpeedDisk*highestPrecisionSpeed)';
    
    torques = [torques torque]; % Changes size dynamically as the loop iterates. Bad practice?
    masses(end+1) = massSystem;
    
end