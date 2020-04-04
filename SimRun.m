 clear; clc; close all;

%% Controller Parameter Definitions
run Geometry.m; % creates a linkList in the workspace for you and for the simulation
load points3D; % loads the CSM trajectory points

%% Define Kp and Kd gains
Kp = 3200; 
Kd = 1750; 

th_last=[0;-pi/2;0;0;0;0];
t = [0 2 2.5:.25:7.5 15];
points = points3D;
transPercent = 0.01;
[n,~,~] = affine_fit(points);


% provides an initial target to see the simulation work
[t1,t2,t3,t4,t5,t6] = abbInvKine([eye(3) points3D(1,:)';0 0 0 1],[0;-pi/2;0;0;0;0]);
target = [t1,t2,t3,t4,t5,t6];

%% Choose Simulation System (perfect model or realistic model)
Sim_Exact = true; % sets if the simulated geometry is exact (true) or slightly different (false)

%% Enable/Disable Control
control_enable = true;

set_param('Project3_System_a/control_enable', 'sw', int2str(control_enable))

%% Run Simulation
simTime = 15;
simOut =  sim('Project3_System_a','SimulationMode','normal','AbsTol','1e-5','StopTime', int2str(simTime),...
    'SaveState','on','StateSaveName','xout',...
    'SaveOutput','on','OutputSaveName','yout',...
    'SaveFormat', 'array');


%% Extract Variables From Simulation
laser_tracking = simOut.get('laser_tracking');
theta_dot_actual = simOut.get('theta_actual');
theta_actual = simOut.get('theta_actual');
control_torque = simOut.get('control_torque');
act_error = simOut.get('act_error');

error=(act_error.signals.values(:,1:3))';
x_error = sqrt(((act_error.signals.values(:,1))).^2)';
y_error = sqrt(((act_error.signals.values(:,2))).^2)';
z_error = sqrt(((act_error.signals.values(:,3))).^2)';

pos_error = sqrt(mean(error.^2));

%% Plot theta as a function of time
figure(1)
for i=1:6
    subplot(3,2,i)
    plot(theta_actual.time,theta_actual.signals.values(:,i))
    title(['\theta_', int2str(i)])
    xlabel('time (s)')
    ylabel('angle (rad)')
    grid on;
end

%% Display Arm Motion Movie
figure(2)
plot(0,0); ah = gca; % just need a current axis handel
stepSize = fix((1/30)/theta_actual.time(2)); % 30 frames per second
for i=1:stepSize:length(theta_actual.time)
   plotArm(theta_actual.signals.values(i,:),Sim_Exact,ah);
   hold on;
   plot3(reshape(laser_tracking.signals.values(1,4,1:i),[i,1]),... % x pos
         reshape(laser_tracking.signals.values(2,4,1:i),[i,1]),... % y pos
         reshape(laser_tracking.signals.values(3,4,1:i),[i,1]),'r','Parent',ah); % z pos
   plot3(points3D(:,1),points3D(:,2),points3D(:,3),'m','Parent',ah);
   hold off;
   pause(1/30)
end


% figure
% plot(theta_actual.time,pos_error);
% title('RMS Position error')
% xlabel(['RMS_(error)', 's'])
% ylabel('time (t)')
% grid on;
% 
% 
% figure
% plot(theta_actual.time,pos_error);
% title('x')
% xlabel(['x_(error)', 's'])
% ylabel('time (t)')
% grid on;
% 
% figure
% plot(theta_actual.time,pos_error);
% title('y')
% xlabel(['y_(error)', 's'])
% ylabel('time (t)')
% grid on;
% 
% figure
% plot(theta_actual.time,pos_error);
% title('z')
% xlabel(['z_(error)', 's'])
% ylabel('time (t)')
% grid on;


