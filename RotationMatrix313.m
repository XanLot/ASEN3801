%Lab 2 ASEN 3801 Function 3

% Inputs: attitude 313 vector in the form [aplha, beta, gamma]^T
% Outputs: The rotation matrix calculated from the euler angles


clear;
clear workspace;
clc;

function RotMatx = rotx(beta)
        RotMatx = [1, 0, 0;
           0, cos(beta), sin(beta);
           0, -sin(beta), cos(beta)];
end

function RotMatz = rotz(alpha)
        RotMatz = [cos(alpha), sin(alpha), 0;
           -sin(alpha), cos(alpha), 0;
           0, 0, 1];
end

function R = RotationMatrix313(alpha, beta, gamma)
    % Rotation about z-axis by gamma
    R_z1 = rotz(gamma);

    % Rotation about new x-axis by beta
    R_x = rotx(beta);

    % Rotation about new z-axis by alpha
    R_z2 = rotz(alpha);

    % Final Direction Cosine Matrix (DCM)
    R = R_z2 * R_x * R_z1; % Note the order of multiplication
end