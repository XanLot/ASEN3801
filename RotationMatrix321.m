% ASEN 3801 Lab 2
% Contributor: Josh Bumann

% Inputs:
% • attitude321: 3 x 1 vector with the 3-2-1 Euler angles in the form attitude321 = [𝛼𝛼, 𝛽𝛽, 𝛾𝛾]T (In
% ASEN 3728 notation, this would be equivalent to [𝜙𝜙,𝜃𝜃,𝜓𝜓]T ).
% Outputs:
% • DCM: the rotation matrix calculated from the Euler angles.

function DCM = RotationMatrix321(attitude321)
    roll = [1 0 0; 0 cos(attitude321(1)) sin(attitude321(1)); 0 -sin(attitude321(1)) cos(attitude321(1))];
    pitch = [cos(attitude321(2)) 0 -sin(attitude321(2)); 0 1 0; sin(attitude321(2)) 0 cos(attitude321(2))];
    yaw = [cos(attitude321(3)) -sin(attitude321(3)) 0; sin(attitude321(3)) cos(attitude321(3)) 0; 0 0 1];

    DCM = roll * pitch * yaw;
end