clear, clc, close all
g=9.81;
s = settings;
s.matlab.appearance.figure.GraphicsTheme.TemporaryValue= 'light';

% INPUTS
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

%% Plots
%% Plots
figure()
plot(out.time, out.inputs(:,2), 'LineWidth', 1.5)
hold on
plot(out.time, out.inputs(:,3), 'LineWidth', 1.5)
xlabel('Time [s]', 'FontWeight', 'bold')
ylabel('Setpoint [-]', 'FontWeight', 'bold')
legend({'Throttle Setpoint', 'Braking Setpoint'}, 'Location', 'best')

% Compute y-limits with padding
ymin = min(min(out.inputs(:,2:3)));
ymax = max(max(out.inputs(:,2:3)));
if ymax == ymin
    pad = 0.1 * abs(ymax) + 1e-3;
else
    pad = 0.1 * (ymax - ymin);
end
ylim([ymin - pad, ymax + pad])

set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman');
grid on
box off


figure()
plot(out.time, out.wheel_vel(:,1), 'LineWidth', 1.5)
hold on
plot(out.time, out.wheel_vel(:,2), 'LineWidth', 1.5)
plot(out.time, out.omega_i(:,1), 'LineWidth', 1.5)
hold off

xlabel('Time [s]', 'FontWeight', 'bold')
ylabel('$\omega$ [rad/s]', 'Interpreter', 'latex', 'FontWeight', 'bold')
legend({'$\omega^t_f$', '$\omega^t_r$', '$\omega^i_r$'}, ...
       'Interpreter', 'latex', 'Location', 'best')

% Compute y-limits with padding, handle constant signals
ymin = min([out.wheel_vel(:,1); out.wheel_vel(:,2); out.omega_i(:,1)]);
ymax = max([out.wheel_vel(:,1); out.wheel_vel(:,2); out.omega_i(:,1)]);
if ymax == ymin
    pad = 0.1 * abs(ymax) + 1e-3;
else
    pad = 0.1 * (ymax - ymin);
end
ylim([ymin - pad, ymax + pad])

set(findall(gcf,'-property','FontName'), 'FontName', 'Times New Roman');
box off
grid on
