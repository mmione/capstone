% Initializing arrays for calculations % 

angles = [10:1:40]';
masses = [];
torques = [];

massBattery = 0.6332739; % Change these as needed
massNoBattery = 0.2864; 

for massSystem = [[massNoBattery:0.1:massBattery] massBattery] % Loop from minimum (w/o battery) to 1kg 
    
    % Givens %

    diskMass = 0.6;
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
    %test
end

% Plotting 

plot(angles, torques(1:31), angles, torques(1+1*31:31+1*31), angles, torques(1+2*31:31+2*31), angles, torques(1+3*31:31+3*31), angles, torques(1+4*31:31+4*31));
title("Max Angle vs Required Torque")
xlabel("Angle [deg]");
ylabel("Torque [N*m]");
% legend("Without battery","","","With Battery (approx)");
yline(0.0392266,'--',"Motor Threshold");
