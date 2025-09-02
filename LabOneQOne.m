
% Contributors: Xander Lotito
% Course number: ASEN 3801
% Filename: LabOneQOne.m
% Created: 8/26/25


clc;
clear;
close all

v_0 = ones(4,1); % Initial State Vector
% v = [w,x,y,z]^T
tol = 1e-8;
options = odeset('RelTol', tol,'AbsTol',tol);
[tout,vout] = ode45(@ODEFUN,[0 20], v_0,options);

% Plot the results
figure;

subplot(4,1,1);
plot(tout, vout(:,1));
title('State Variable W vs Time');
xlabel('Time');
ylabel('w');
grid on


subplot(4,1,2);
plot(tout, vout(:,2));
title('State Variable X vs Time');
xlabel('Time');
ylabel('x');
grid on


subplot(4,1,3);
plot(tout, vout(:,3));
title('State Variable Y vs Time');
xlabel('Time');
ylabel('y');
grid on


subplot(4,1,4);
plot(tout, vout(:,4));
title('State Variable Z vs Time');
xlabel('Time');
ylabel('z');
grid on

% Rerun the code at different tolerances

% Get reference value with high tolerance 1e-12
tol = 1e-12;
options = odeset('RelTol', tol,'AbsTol',tol);
[tout,vout_ref] = ode45(@ODEFUN,[0 20], v_0,options);

% Store reference value at t = 20
refVal = vout_ref(end,:);

% Rerun code at lower tolerances
low_tol = [1e-2, 1e-4, 1e-6, 1e-8, 1e-10];

% Run ODE over larger tolerances
for i = 1:length(low_tol)
    options = odeset('RelTol', low_tol(i), 'AbsTol', low_tol(i));
    [~, vout_low{i}] = ode45(@ODEFUN, [0 20], v_0, options); % Save into table
end

% Save values
vout_two = vout_low{1};
vout_two = vout_two(end,:);

vout_four = vout_low{2};
vout_four = vout_four(end,:);

vout_six = vout_low{3};
vout_six = vout_six(end,:);

vout_eight = vout_low{4};
vout_eight = vout_eight(end,:);

vout_ten = vout_low{5};
vout_ten = vout_ten(end,:);

% Compare end values to reference values

endValues = [vout_two; vout_four; vout_six; vout_eight; vout_ten];
comparison = abs(endValues - refVal);
format shortE;
disp('Comparison of end values to reference values:');
disp(comparison);


function dvdt = ODEFUN(t,v)
% Pre-allocate derivative vector
dvdt = zeros(4,1);

dvdt(1) = -9 * v(1) + v(3); % w'
dvdt(2) = 4* v(1) * v(2) * v(3) - v(2).^2; % x'
dvdt(3) = 2 * v(1) - v(2) - 2 * v(4); % y'
dvdt(4) = v(2) * v(3) - v(3).^2 - 3 * v(4)^3; %z'

end