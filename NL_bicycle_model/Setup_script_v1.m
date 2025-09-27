clear, clc, close all
%this is the setup script for the NL bicycle model

%% INPUTS
%vehicle
m = 200; %[kg]
Izz = 1000; %[kg.m^3]
a = 0.8; %distance from front axle to CG [m]
b = 1; %distance from rear axle to CG [m]
r = 0.5; %radidus of wheel [m]
mu = 0.5; %coeff of friction of the road surface
c_alpha = 30000; %[N/rad]
c_sigma = 8000; %[N/rad]
I_wheel =  20; %[kg/m^3]
drivetrain_e = 0.8; %[drivetrain efficiency]
gr = 1; %gear ratio [-]

%powertrain
torque_curve = [200, 225, 300, 400, 500, 400, 350, 300, 250, 200];
torque_omega = linspace(0, 100, length(torque_curve));

brake_bias = 0.5; %front bias
brake_torque = [0, 25, 50, 100, 125, 150, 175, 200]; %[n/m]

%calcualte F_z
F_z_f = m*9.81*(b/(b+a)); %[N]
F_z_r = m*9.81*(a/(b+a));

%aero
C_l = -1.2;
C_d = 0.3;
A = 2; %[m^]
rho = 1.225; %[kg/m^3]
CoP = 0.4; %[%front]

%% inputs plot
figure('Position', [200, 200, 1200, 500])
t1 = tiledlayout(1,2);
nexttile
plot(out.time, out.inputs(:,1))
xlabel('Time [s]')
ylabel('Throttle Setpoint [-]')
ylim([min(out.inputs(:,1)) - 0.1*(max(out.inputs(:,1)) - min(out.inputs(:,1)))
      max(out.inputs(:,1)) + 0.1*(max(out.inputs(:,1)) - min(out.inputs(:,1)))]);

nexttile
plot(out.time, out.inputs(:,2))
xlabel('Time [s]')
ylabel('Brake Setpoint [-]')
ylim([min(out.inputs(:,2)) - 0.1*(max(out.inputs(:,2)) - min(out.inputs(:,2)))
      max(out.inputs(:,2)) + 0.1*(max(out.inputs(:,2)) - min(out.inputs(:,2)))]);

title(t1, 'System Inputs', 'FontName', 'Times New Roman', 'FontSize', 16);
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman');

%% Vehicle Velocities/Pos
figure('Position', [200, 200, 1200, 500])
t2 = tiledlayout(1,2);
nexttile
plot(out.time, out.velocities(:,1))
xlabel('Time [s]')
ylabel('$\dot{x}^t$ [m/s]', 'Interpreter', 'latex')
ylim([min(out.velocities(:,1)) - 0.1*(max(out.velocities(:,1)) - min(out.velocities(:,1)))
      max(out.velocities(:,1)) + 0.1*(max(out.velocities(:,1)) - min(out.velocities(:,1)))]);

nexttile

plot(out.time, out.global_XY(:,1))
xlabel('Time [s]')
ylabel('X-pos [m]')


title(t2, 'Vehicle X Velocity/Position', 'FontName', 'Times New Roman', 'FontSize', 16);
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman');





