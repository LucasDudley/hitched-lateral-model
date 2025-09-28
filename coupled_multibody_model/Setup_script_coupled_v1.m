clear, clc, close all
g=9.81;
%this script must be run before running the simulink model for the coupled
%vehicle model. Plots can be generated after the simulink file has been
%run. Note: system inputs are inside the simulink model.

%% INPUTS
%vehicle dimensions
a = 0.8; %distance from front axle to vehicle CG [m]
b = 1; %distance from rear axle to vehicle CG [m]
c = 0.8; %distance from hitch to vehicle CG [m]
d = 0.5; % distance from impliment CG to hitch [m]
e = 1; %distanc from impliment CG to rear tire [m]

%vehicle mass/interitas
m_t = 400; %[kg]
m_i = 100; %[kg]
Izz_t = 1000; %[kg.m^3]
Izz_i = 500; 
I_pin = Izz_i + m_i*(d^2); %parallel axis thm.


%tire model
r = 0.5; %radidus of wheel [m]
mu = 0.7; %coeff of friction of the road surface
c_alpha = 30000; %[N/rad]
c_sigma = 8000; %[N/rad]
peak_slip = 14*(pi/180);
slip_coeff = 0.3; %coeff of friction if sliding

%drivetrain
I_wheel =  20; %[kg/m^3]
drivetrain_e = 0.8; %[drivetrain efficiency]
gr = 1; %gear ratio [-]

%powertrain
torque_curve = [200, 225, 300, 400, 500, 400, 350, 300, 250, 200];
torque_omega = linspace(0, 100, length(torque_curve));

%brake
brake_bias = 0.5; %front bias (vehicle)
rear_braking = 0.3; %percent acting on rear axle
brake_torque = [0, 25, 50, 100, 125, 150, 175, 200]; %[n/m]

%aero
C_d = 0.3;
A = 2; %[m^]
rho = 1.225; %[kg/m^3]
CoP = 0.4; %[%front]

%Normal Load Calculations
F_h = (m_i*g*e)/(e+d);
F_t_f = (m_t*g*b - F_h*c)/(a+b);
F_t_r = (m_t*g*a + F_h*(b+a))/(a+b);
F_i_r = m_i*g-F_h;


%Plot Figures
%% inputs
figure('Position', [200, 200, 1400, 500])
t1 = tiledlayout(1,3);

labels = {'\delta_f [rad]', 'Throttle Setpoint [-]', 'Braking Setpoint [-]'};

for i = 1:3
    nexttile
    plot(out.time, out.inputs(:,i))
    xlabel('Time [s]')
    ylabel(labels{i})

    % Compute y-limits with 10% padding, handle constant signals
    ymin = min(out.inputs(:,i));
    ymax = max(out.inputs(:,i));

    if ymax == ymin
        pad = 0.1 * abs(ymax) + 1e-3; % give small pad even if value is 0
        ylim([ymin - pad, ymax + pad])
    else
        pad = 0.1 * (ymax - ymin);
        ylim([ymin - pad, ymax + pad])
    end
end

title(t1, 'System Inputs', 'FontName', 'Times New Roman', 'FontSize', 16);
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman');

%% Vehicle Velocities
figure('Position', [200, 200, 1400, 500])
t2 = tiledlayout(1,3);
nexttile
plot(out.time, out.velocities_t(:,1))
xlabel('Time [s]')
ylabel('$\dot{x}^t$ [m/s]', 'Interpreter', 'latex')
ylim([min(out.velocities_t(:,1)) - 0.1*(max(out.velocities_t(:,1)) - min(out.velocities_t(:,1)))
      max(out.velocities_t(:,1)) + 0.1*(max(out.velocities_t(:,1)) - min(out.velocities_t(:,1)))]);

nexttile
plot(out.time, out.velocities_t(:,2))
xlabel('Time [s]')
ylabel('$\dot{y}^t$ [m/s]', 'Interpreter', 'latex')
ylim([min(out.velocities_t(:,2)) - 0.1*(max(out.velocities_t(:,2)) - min(out.velocities_t(:,2)))
      max(out.velocities_t(:,2)) + 0.1*(max(out.velocities_t(:,2)) - min(out.velocities_t(:,2)))]);

nexttile
plot(out.time, out.velocities_t(:,3))
xlabel('Time [s]')
ylabel('$\dot{\varphi}^t$ [rad/s]', 'Interpreter', 'latex')
ylim([min(out.velocities_t(:,3)) - 0.1*(max(out.velocities_t(:,3)) - min(out.velocities_t(:,3)))
      max(out.velocities_t(:,3)) + 0.1*(max(out.velocities_t(:,3)) - min(out.velocities_t(:,3)))]);

title(t2, 'Vehicle Velocities', 'FontName', 'Times New Roman', 'FontSize', 16);
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman');


%% implement velocities
figure('Position', [200, 200, 1400, 500])
t3 = tiledlayout(1,3);
nexttile
plot(out.time, out.velocities_CG_i(:,1))
xlabel('Time [s]')
ylabel('$\dot{x}^i$ [m/s]', 'Interpreter', 'latex')
ylim([min(out.velocities_CG_i(:,1)) - 0.1*(max(out.velocities_CG_i(:,1)) - min(out.velocities_CG_i(:,1)))
      max(out.velocities_CG_i(:,1)) + 0.1*(max(out.velocities_CG_i(:,1)) - min(out.velocities_CG_i(:,1)))]);

nexttile
plot(out.time, out.velocities_CG_i(:,2))
xlabel('Time [s]')
ylabel('$\dot{y}^i$ [m/s]', 'Interpreter', 'latex')
ylim([min(out.velocities_CG_i(:,2)) - 0.1*(max(out.velocities_CG_i(:,2)) - min(out.velocities_CG_i(:,2)))
      max(out.velocities_CG_i(:,2)) + 0.1*(max(out.velocities_CG_i(:,2)) - min(out.velocities_CG_i(:,2)))]);

nexttile
plot(out.time, out.phi_i_dot(:,1))
xlabel('Time [s]')
ylabel('$\dot{\varphi}^i$ [rad/s]', 'Interpreter', 'latex')
ylim([min(out.phi_i_dot(:,1)) - 0.1 * (max(out.phi_i_dot(:,1)) - min(out.phi_i_dot(:,1))), ...
      max(out.phi_i_dot(:,1)) + 0.1 * (max(out.phi_i_dot(:,1)) - min(out.phi_i_dot(:,1)))]);


title(t3, 'Implement Velocities', 'FontName', 'Times New Roman', 'FontSize', 16);
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman');


%% wheel Velocities
figure('Position', [200, 200, 1400, 500])
t4 = tiledlayout(1,3);
nexttile
plot(out.time, out.wheel_vel(:,1))
xlabel('Time [s]')
ylabel('$\omega^t_f$ [rad/s]', 'Interpreter', 'latex')
ylim([min(out.wheel_vel(:,1)) - 0.1*(max(out.wheel_vel(:,1)) - min(out.wheel_vel(:,1)))
      max(out.wheel_vel(:,1)) + 0.1*(max(out.wheel_vel(:,1)) - min(out.wheel_vel(:,1)))]);

nexttile
plot(out.time, out.wheel_vel(:,2))
xlabel('Time [s]')
ylabel('$\omega^i_r$ [rad/s]', 'Interpreter', 'latex')
ylim([min(out.wheel_vel(:,2)) - 0.1*(max(out.wheel_vel(:,2)) - min(out.wheel_vel(:,2)))
      max(out.wheel_vel(:,2)) + 0.1*(max(out.wheel_vel(:,2)) - min(out.wheel_vel(:,2)))]);

nexttile
plot(out.time, out.omega_i(:,1))
xlabel('Time [s]')
ylabel('$\omega^i_r$ [rad/s]', 'Interpreter', 'latex')
ylim([min(out.omega_i(:,1)) - 0.1 * (max(out.omega_i(:,1)) - min(out.omega_i(:,1))), ...
      max(out.omega_i(:,1)) + 0.1 * (max(out.omega_i(:,1)) - min(out.omega_i(:,1)))]);


title(t4, 'Wheel Velocities', 'FontName', 'Times New Roman', 'FontSize', 16);
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman');

%% slip angles

% Define the threshold for unusually high values
threshold = 0.06;

% Replace values exceeding the threshold with NaN in the first column of slip_angles_t
out.slip_angles_t(out.slip_angles_t(:,1) > threshold, 1) = NaN;

figure('Position', [200, 200, 1400, 500])
t5 = tiledlayout(1,2);
nexttile
plot(out.time, out.slip_angles_t(:,1))
xlabel('Time [s]')
ylabel('$\alpha^t_f$ [rad]', 'Interpreter', 'latex')
ylim([min(out.slip_angles_t(:,1)) - 0.1*(max(out.slip_angles_t(:,1)) - min(out.slip_angles_t(:,1)))
      max(out.slip_angles_t(:,1)) + 0.1*(max(out.slip_angles_t(:,1)) - min(out.slip_angles_t(:,1)))]);

nexttile
plot(out.time, out.slip_angles_t(:,2))
xlabel('Time [s]')
ylabel('$\alpha^t_r$ [rad]', 'Interpreter', 'latex')
ylim([min(out.slip_angles_t(:,2)) - 0.1*(max(out.slip_angles_t(:,2)) - min(out.slip_angles_t(:,2)))
      max(out.slip_angles_t(:,2)) + 0.1*(max(out.slip_angles_t(:,2)) - min(out.slip_angles_t(:,2)))]);


title(t5, 'Slip Angles', 'FontName', 'Times New Roman', 'FontSize', 16);
set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman');



%% GLOBAL XY

figure()
plot(out.global_XY_t(:,1), out.global_XY_t(:,2))
title('Global XY Positions')
xlabel('X-Pos [m]')
ylabel('Y-Pos [m]')
hold on
plot(out.global_XY_i(:,1), out.global_XY_i(:,2))
legend('Vehicle CG', 'Implement CG')

set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman')



