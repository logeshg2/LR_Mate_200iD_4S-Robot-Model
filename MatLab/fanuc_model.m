%{
    MatLab implementation of Fanuc LR Mate 200iD 4s robot model.
    For DH parameters refer this site: https://www.fanucamerica.com/cmsmedia/datasheets/LR%20Mate%20200iD%20Series_187.pdf and the below model also.
    Model developed by Logesh G
%}

% works to do: 1. qlim update

% dh parameters
%{
dh = [0      0      0       pi/2 ; 
      0      0      0.260   0    ; 
      0      0      0.020  -pi/2 ; 
      0     -0.290  0       pi/2 ; 
      0      0      0      -pi/2 ; 
      0     -0.070  0       pi   ];
%}

L(1) = Link([0, 0, 0, pi/2]);
L(2) = Link([0, 0, 260, 0]);
L(3) = Link([0, 0, 20, -pi/2]);
L(4) = Link([0,-290, 0, pi/2]);
L(5) = Link([0, 0, 0, -pi/2]);
L(6) = Link([0,-70, 0, pi]);

L(2).offset = pi/2;

% fanuc robot model
robot = SerialLink(L);
robot.name = "Lr-Mate-200id4s";

% set qlim
robot.qlim(1,:) = deg2rad([-170, 170]);
robot.qlim(2,:) = deg2rad([-100, 145]);
robot.qlim(3,:) = deg2rad([-140, 200]);
robot.qlim(4,:) = deg2rad([-180, 180]);
robot.qlim(5,:) = deg2rad([-120, 120]);
robot.qlim(1,:) = deg2rad([-360, 360]);

% home position joint angles 
qr = [0 0 0 0 0 0];
qz = [0 0 0 0 0 0];
weights = [1 1 1 1 1 1]; % [translation & rotation]         % for 6-dof no need to consider this

% plot robot
% robot.plot(qr);

% example tf
tf = transl(350, 0, 50) *  eul2tform([0,pi,0], "ZYX");          % eul => default order is "ZYX" 
disp("Original tf:")
disp(tf)

% example ikine (starting from all zero angles position)
config = robot.ikcon(tf);
disp("Computed config:")
disp(config)

% example fkine
disp("Computed tf:")
computed_tf = robot.fkine(config);
disp(computed_tf);

% example plotting
robot.plot(config);
% robot.teach(config)

% modify the joint angles to support the fanuc robotic arm (only for J2 & J3)
% config(2) = -config(2);
% config(3) = config(3) - config(2);
% now this config can be sent to the robot via plc and robot arm.