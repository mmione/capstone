% Initializing arrays for calculations % 

angles = [10:1:40]'; % Use to test all angles within a reasonable range
masses = [];
torques = [];

massBattery = 0.6332739; % Change these as needed
massNoBattery = 0.2064*0.3; %  Assuming X infill

for massSystem = [[massNoBattery:0.1:massBattery] massBattery] % Loop from minimum (w/o battery) to 1kg 
    
    % Givens %

    diskMass = 0.2;
    radiusDisk = 0.05;
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

% Plotting 

plot(angles, torques(1:31), angles, torques(1+1*31:31+1*31), angles, torques(1+2*31:31+2*31), angles, torques(1+3*31:31+3*31), angles, torques(1+4*31:31+4*31));
title("Max Angle vs Required Torque")
yline(0.0392266,'--',"Motor Threshold"); % Creating line for motor threshold
xlabel("Angle [deg]");
ylabel("Torque [N*m]");
legend("Without battery m = "+string(masses(1)),"m = "+string(masses(2)),"m = "+string(masses(3)), "m = "+string(masses(3)), "m = "+string(masses(4)), "With Battery (approx) m = "+string(masses(5)));

