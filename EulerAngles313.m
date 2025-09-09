function attitude313 = EulerAngles313(DCM)
alpha = arctan(DCM(1,3) / DCM(2,3));
beta = arccos(DCM(3,3));
gamma = arctan(DCM(3,1) / - DCM(3,2));
attitude313 = [alpha, beta, gamma]';
end