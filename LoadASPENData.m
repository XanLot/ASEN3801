import 'ConvertASPENData.m'.*

function [t_vec, av_pos_inert, av_att, tar_pos_inert, tar_att] = LoadASPENData(filename)
table = readmatrix(filename); % Read in file
table(1:3,:) = []; % Remove headers
table(:,2) = []; % Remove subframe column
% Store Data for Conversion
pos_av_aspen = table(:,11:13)'; 
att_av_aspen = table(:,8:10)';
pos_tar_aspen = table(:,5:7)';
att_tar_aspen = table(:,2:4)';
% Convert Values
[pos_av_class, att_av_class, pos_tar_class, att_tar_class] = ConvertASPENData(pos_av_aspen, att_av_aspen, pos_tar_aspen, att_tar_aspen);
t_vec = table(:,1)' * (1/100); % Time in seconds.
av_pos_inert = pos_av_class / 1000; % Pos in m
av_att = att_tar_class;
tar_pos_inert = pos_tar_class;
tar_att = att_av_class;
end