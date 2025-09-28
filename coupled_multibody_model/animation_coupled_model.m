clear, clc, close all
g=9.81;
%this script can be used to setup the coupled model simulink and generate
%an animation of the output

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
Izz_t = 1000; %[kg.m^2]
Izz_i = 500; 
I_pin = Izz_i + m_i*(d^2); %parallel axis thm.

%tire model
r = 0.25; %radidus of wheel [m]
mu = 0.7; %coeff of friction of the road surface
c_alpha = 30000; %[N/rad]
c_sigma = 8000; %[N/rad]
peak_slip = 14*(pi/180);
slip_coeff = 0.3; %coeff of friction if sliding

%drivetrain
I_wheel =  20; %[kg.m^2]
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

%% Animations
figure('Color', 'w')
axis equal;
hold on;

set(groot, 'defaultAxesFontName', 'Times New Roman')
set(groot, 'defaultTextFontName', 'Times New Roman')

time = out.time;
X_vehicle = out.global_XY_t(:,1);
Y_vehicle = out.global_XY_t(:,2);
phi_vehicle = out.phi(:,1);
X_implement = out.global_XY_i(:,1);
Y_implement = out.global_XY_i(:,2);
phi_implement = out.phi(:,2);
delta_f = out.inputs(:,1);

% Target frame rate
target_fps = 25;
dt = 1 / target_fps;

% Interpolate to constant frame rate
time_interp = min(time):dt:max(time);
X_vehicle_interp = interp1(time, X_vehicle, time_interp, 'linear');
Y_vehicle_interp = interp1(time, Y_vehicle, time_interp, 'linear');
phi_vehicle_interp = interp1(time, phi_vehicle, time_interp, 'linear');
X_implement_interp = interp1(time, X_implement, time_interp, 'linear');
Y_implement_interp = interp1(time, Y_implement, time_interp, 'linear');
phi_implement_interp = interp1(time, phi_implement, time_interp, 'linear');
delta_f_interp = interp1(time, delta_f, time_interp, 'linear');

% Axis limits
x_limits = [min([X_vehicle; X_implement]) - 3, max([X_vehicle; X_implement]) + 3];
y_limits = [min([Y_vehicle; Y_implement]) - 3, max([Y_vehicle; Y_implement]) + 3];
xlim(x_limits);
ylim(y_limits);
grid on
box off
xlabel('X-Pos [m]', 'FontWeight', 'bold')
ylabel('Y-Pos [m]', 'FontWeight', 'bold')

% Video setup (MP4)
v = VideoWriter('vehicle_animation.mp4','MPEG-4');
v.FrameRate = target_fps;
open(v)

for k = 1:length(time_interp)
    cla;
    % positions
    vehicle_x = [X_vehicle_interp(k) + a * cos(phi_vehicle_interp(k))
                 X_vehicle_interp(k) - b * cos(phi_vehicle_interp(k))];
    vehicle_y = [Y_vehicle_interp(k) + a * sin(phi_vehicle_interp(k))
                 Y_vehicle_interp(k) - b * sin(phi_vehicle_interp(k))];
    implement_x = [X_implement_interp(k) + d * cos(phi_implement_interp(k))
                   X_implement_interp(k) - e * cos(phi_implement_interp(k))];
    implement_y = [Y_implement_interp(k) + d * sin(phi_implement_interp(k))
                   Y_implement_interp(k) - e * sin(phi_implement_interp(k))];
    wheel_angle = phi_vehicle_interp(k) + delta_f_interp(k);
    wheel_front_x = vehicle_x(1) + r * cos(wheel_angle);
    wheel_front_y = vehicle_y(1) + r * sin(wheel_angle);
    wheel_rear_x = vehicle_x(1) - r * cos(wheel_angle);
    wheel_rear_y = vehicle_y(1) - r * sin(wheel_angle);

    % Plot
    plot(vehicle_x, vehicle_y, 'b', 'LineWidth', 2); % Vehicle plot
    plot(implement_x, implement_y, 'r', 'LineWidth', 2); % Implement plot
    plot([wheel_front_x, wheel_rear_x], [wheel_front_y, wheel_rear_y], ...
         'k', 'LineWidth', 2);
    title(sprintf('Time: %.2f s', time_interp(k)), ...
          'FontWeight', 'bold', 'FontName', 'Times New Roman');

    % Capture frame for video
    frame = getframe(gcf);
    writeVideo(v, frame);
    drawnow;
end

close(v);
hold off;





