% Estephany Barba Matta
% April 2024
% Lab 3: Aircraft Longitudinal Dynamics Simulation
% Script
close all
clc
clear

% Constants C152
g = 9.81;
m = 758;
density = 1.225;
AR = 6.8;
C_D_0 = 0.042696;
S = 14.9;
K = 0.068861;

% Formulas
W = m*g;
e = 1/(pi*AR*K);
E_max = 0.5*(sqrt((pi*AR*e)/(C_D_0)));
C_L_md = sqrt(pi*AR*e*C_D_0);
C_L = C_L_md;
fpangle_input = atan(-(1/E_max));
v_trim = sqrt((2*W)/(density*S*C_L));
q = 0.5*density*(v_trim^2);

%% Constant glide at L/D_max

tspan = (0:0.01:110);
% Initial conditions
IC = [v_trim ,fpangle_input, 0,1000/3.281]; 
[t,x] = ode45(@(t,x) lab3_function (t,x,C_L,g,density,E_max,S,m), tspan, IC);
% Distance, altitude, velocity and flight path angle
dist = x(:,3);
alt = x(:,4);
vel = x(:,1);
fp_angle = x(:,2);

%% Glide from horizontal trim position

tspan = (0:0.01:110);
trim_angle= 0; 
IC = [v_trim ,trim_angle, 0,1000/3.281]; %initial conditions, theta and theta dot
[t_trim,x_trim] = ode45(@(t_trim,x_trim) lab3_function_trim (t_trim,x_trim,C_L,g,density,S,m,E_max),tspan,IC);
vel_trim = x_trim(:,1);
fp_angletrim = x_trim(:,2);
disttrim = x_trim(:,3);
alttrim = x_trim(:,4);

%% Glide from horizontal trim position at trim airspeed + 25%

tspan = (0:0.01:110);
trim_angle= 0; 
IC = [v_trim *1.25,trim_angle, 0,1000/3.281]; %initial conditions, theta and theta dot
[t_trim25,x_trim25] = ode45(@(t_trim,x_trim) lab3_function_trim(t_trim,x_trim,C_L,g,density,S,m,E_max),tspan,IC);
vel_trim25 = x_trim25(:,1);
fp_angletrim25 = x_trim25(:,2);
disttrim25 = x_trim25(:,3);
alttrim25 = x_trim25(:,4);

%% Glide from horizontal trim position at trim airspeed + 50%

tspan = (0:0.01:110);
trim_angle= 0; 
IC = [v_trim*1.5 ,trim_angle, 0,1000/3.281]; %initial conditions, theta and theta dot
[t_trim50,x_trim50] = ode45(@(t_trim,x_trim) lab3_function_trim(t_trim,x_trim,C_L,g,density, S, m, E_max ), tspan, IC);
vel_trim50 = x_trim50(:,1);
fp_angletrim50 = x_trim50 (:,2);
disttrim50 = x_trim50(:,3);
alttrim50 = x_trim50(:,4);

%% Finding maximum values

max_array = (find(alt<0)-1);
ind_maxr = max_array(1,1);
maxr_const = dist(ind_maxr);
maxr_t = t(ind_maxr);

max_array_trim = (find(alttrim<0)- 1);
ind_maxr_trim = max_array_trim(1,1);

maxr_trim = disttrim(ind_maxr_trim);
maxr_trim_t = t_trim(ind_maxr_trim);


max_array_trim25 = (find(alttrim25<0)- 1);
ind_maxr_trim25 = max_array_trim25(1,1);

maxr_trim25 = disttrim25(ind_maxr_trim25);
maxr_trim25_t = t_trim25(ind_maxr_trim25);


max_array_trim50 = (find(alttrim50<0)- 1);
ind_maxr_trim50 = max_array_trim50(1,1);

maxr_trim50 = disttrim50(ind_maxr_trim50);
maxr_trim50_t = t_trim50(ind_maxr_trim50);

%% Deleting "under the ground" insignificant values

under_ground = find(alt<0);
alt(under_ground) = [];
dist(under_ground) = [];
vel(under_ground) = [];
fp_angle(under_ground) = [];
t(under_ground) = [];

under_ground = find(alttrim<0);
alttrim(under_ground) = [];
disttrim(under_ground) = [];
vel_trim(under_ground) = [];
fp_angletrim(under_ground) = [];
t_trim(under_ground) = [];

under_ground = find(alttrim25<0);
alttrim25(under_ground) = [];
disttrim25(under_ground) = [];
vel_trim25(under_ground) = [];
fp_angletrim25(under_ground) = [];
t_trim25(under_ground) = [];

under_ground = find(alttrim50<0);
alttrim50(under_ground) = [];
disttrim50(under_ground) = [];
vel_trim50(under_ground) = [];
fp_angletrim50(under_ground) = [];
t_trim50(under_ground) = [];

%% Figures

figure

plot(dist, alt ,'Color', [0 0 0], 'LineWidth', 1.5)

hold on
grid minor

plot(disttrim, alttrim, 'Color', [0.8 0 0.4], 'LineWidth', 1.5)
plot(disttrim25, alttrim25, 'Color', [0 0 1], 'LineWidth', 1.5)
plot(disttrim50, alttrim50, 'Color', [0 0.8 0.2], 'LineWidth', 1.5)

ylabel('Height [m]', 'FontSize', 12);
xlabel('Range [m]', 'FontSize', 12);
title('Flightpath in state space form', 'FontSize', 14);
legend('Constant angle glide at (L/D)max','Glide from horizontal trim position','Glide from horizontal trim position at trim airspeed + 25%','Glide from horizontal trim position at trim airspeed + 50%', 'FontSize', 12);

figure

subplot (2, 2, 1)
hold on
grid minor
plot(t, vel, 'Color', [0 0 0], 'LineWidth', 1.5)
plot(t_trim,vel_trim, 'Color', [0.8 0 0.4], 'LineWidth', 1.5)
plot(t_trim25,vel_trim25, 'Color', [0 0 1], 'LineWidth', 1.5)
plot(t_trim50,vel_trim50, 'Color', [0 0.8 0.2], 'LineWidth', 1.5)
ylabel('Velocity [m/s]', 'FontSize', 12);
xlabel('Time [s]', 'FontSize', 12);
title('Velocity time history', 'FontSize', 14);
legend('Constant angle glide at (L/D)max','Glide from horizontal trim position','Glide from horizontal trim position at trim airspeed + 25%','Glide from horizontal trim position at trim airspeed + 50%', 'FontSize', 10);

subplot (2, 2, 2)
hold on
grid minor
plot(t, fp_angle, 'Color', [0 0 0], 'LineWidth', 1.5)
plot(t_trim,fp_angletrim, 'Color', [0.8 0 0.4], 'LineWidth', 1.5)
plot(t_trim25,fp_angletrim25, 'Color', [0 0 1], 'LineWidth', 1.5)
plot(t_trim50,fp_angletrim50, 'Color', [0 0.8 0.2], 'LineWidth', 1.5)
ylabel('Flight Path Angle [rad]', 'FontSize', 12);
xlabel('Time [s]', 'FontSize', 12);
title('Flight path angle time history', 'FontSize', 14);
legend('Constant angle glide at (L/D)max','Glide from horizontal trim position','Glide from horizontal trim position at trim airspeed + 25%','Glide from horizontal trim position at trim airspeed + 50%', 'FontSize', 10);

subplot (2, 2, 3)
hold on
grid minor
plot(t,alt, 'Color', [0 0 0], 'LineWidth', 1.5)
plot(t_trim,alttrim, 'Color', [0.8 0 0.4], 'LineWidth', 1.5)
plot(t_trim25,alttrim25, 'Color', [0 0 1], 'LineWidth', 1.5)
plot(t_trim50,alttrim50, 'Color', [0 0.8 0.2], 'LineWidth', 1.5)
ylabel('Altitude [m]', 'FontSize', 12);
xlabel('Time [s]', 'FontSize', 12);
title('Altitude time history', 'FontSize', 14);
legend('Constant angle glide at (L/D)max','Glide from horizontal trim position','Glide from horizontal trim position at trim airspeed + 25%','Glide from horizontal trim position at trim airspeed + 50%', 'FontSize', 10);

subplot (2, 2, 4)
hold on
grid minor
plot(t, dist, 'Color', [0 0 0], 'LineWidth', 1.5)
plot(t_trim,disttrim, 'Color', [0.8 0 0.4], 'LineWidth', 1.5)
plot(t_trim25,disttrim25, 'Color', [0 0 1], 'LineWidth', 1.5)
plot(t_trim50,disttrim50, 'Color', [0 0.8 0.2], 'LineWidth', 1.5)
ylabel('Range [m]', 'FontSize', 12);
xlabel('Time [s]', 'FontSize', 12);
title('Range time history', 'FontSize', 14);
legend('Constant angle glide at (L/D)max','Glide from horizontal trim position','Glide from horizontal trim position at trim airspeed + 25%','Glide from horizontal trim position at trim airspeed + 50%', 'FontSize', 10, 'location', 'northwest');
