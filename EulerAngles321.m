% ASEN 3801 Lab 2
% Contributor: Josh Bumann

% Inputs:
% • DCM: a rotation matrix
% Outputs:
% • attitude321: 3 x 1 vector with the 3-2-1 Euler angles in the form attitude321 = [𝛼𝛼, 𝛽𝛽, 𝛾𝛾]T (In
% ASEN 3728 notation, this would be equivalent to [𝜙𝜙,𝜃𝜃,𝜓𝜓]T ).

function attitude321 = EulerAngles321(DCM)
    attitude321(1) = atan2(DCM(2,3), DCM(3,3));
    attitude321(2) = -arcsin(DCM(1,3));
    attitude321(3) = atan2(DCM(1,2), DCM(1,1));
end