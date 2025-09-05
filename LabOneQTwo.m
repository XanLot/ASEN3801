import 'stdatmo.m.*'
clc;
clear;
close all


% Constants
m = 50 * 10^-3; % Mass in kg
Cd = 0.6; % Coefficient of Drag
d = 2 * 10^-2; % Diameter in meters
rho = stdatmo(1655); % Air Density in Boulder
A = ( pi / 4 ) * d^2; % Cross-sectional area
g = 9.81; % Gravity

wind_vel = [1; 0; 0]; % x-dir, y-dir, z-dir

% x - north
% y - east
% z - down

% Initial State Vector
x_0 = [0; 0; 0; 0; 20; -20];

tol = 1e-10;
options = odeset('RelTol',tol,'AbsTol',tol);

[t,x] = ode45(@(t,x) objectEOM(t,x,rho,Cd,A,m,g,wind_vel), [0 10],x_0,options);



% Plot the results in 3D (part c)
figure(1);
plot3(x(:,1), x(:,2), x(:,3), 'LineWidth', 2);
xlabel('North (x)');
ylabel('East (y)');
zlabel('Down (z)');
title('3D Object Trajectory');
grid on;
view(3); % Set the view to 3D

% Flip the z-axis
set(gca, 'ZDir', 'reverse');


% Find effect of wind velocities (part d)
% d1
vel_num = 100;
finalDisplacement = zeros(1,vel_num);
totalDistance = zeros(1,vel_num);
for l = 1:vel_num
    wind_vel = [l;0;0]; % Change wind velocities
    [t,x] = ode45(@(t,x) objectEOM(t,x,rho,Cd,A,m,g,wind_vel), [0 10],x_0,options);
    % Store the final x - displacement
    finalDisplacement(l) = x(end,1);
    totalDistance(l) = sqrt(x(end,1)^2 + x(end,2)^2);
end

figure(2)
plot(1:vel_num, finalDisplacement)
title('Displacement vs Wind Velocity');
xlabel('Wind Velocity (m/s)');
ylabel('Displacement (m)');
grid on;

% d2

figure(3)
plot(1:vel_num, totalDistance)
title('Ending Distance vs Wind Velocity');
xlabel('Wind Velocity (m/s)');
ylabel('Distance from Origin (m)');
grid on;


% Calculate the Distance vs Windspeed at different altitudes
% d3
altitudes = 0:1000:10000; % Altitudes in meters
num_altitudes = length(altitudes);
finalDistanceAlt = zeros(num_altitudes, vel_num);

figure(4);
hold on;

for k = 1:num_altitudes
    rho = stdatmo(altitudes(k)); % Get air density at the current altitude
    for l = 1:vel_num
        wind_vel = [l; 0; 0]; % Change wind velocities
        [t, x] = ode45(@(t, x) objectEOM(t, x, rho, Cd, A, m, g, wind_vel), [0 10], x_0, options);
        % Store the final x - displacement for the current altitude
        finalDistanceAlt(k, l) = sqrt(x(end, 1)^2 + x(end,2)^2);
    end
    plot(1:vel_num, finalDistanceAlt(k, :), 'DisplayName', sprintf('Altitude: %d m', altitudes(k)));
end


title('Distance vs Wind Velocity at Different Altitudes');
xlabel('Wind Velocity (m/s)');
ylabel('Distance from Origin (m)');
legend show;
grid on;
hold off;

function xdot = objectEOM(t,x,rho,Cd,A,m,g,wind_vel)
    xdot = zeros(6,1);
    xdot(1) = x(4);
    xdot(2) = x(5);
    xdot(3) = x(6);

    % Calculate Drag Force
    v = [x(4) - wind_vel(1); x(5) - wind_vel(2); x(6) - wind_vel(3)];
    v_mag = norm(v);
    D = 0.5 * rho * v_mag^2 * Cd * A;
    % Get direction of airflow
    dir_d = -v/v_mag;
    % Calculate acceleration due to drag
    a_drag = (D / m) * dir_d;
    xdot(4) = a_drag(1);
    xdot(5) = a_drag(2);
    xdot(6) = g + a_drag(3); % Update acceleration in z-direction

    if x(3) > 0
        xdot = zeros(6,1);
    end
    
end

