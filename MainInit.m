clc;
clear all;
% Insert path that contain installation files of CPLEX
% example: 'C:\Program Files\IBM\ILOG\CPLEX_Studio1271\cplex\'
% It is important that the directories of Robot Motion Toolbox: 
% 'aux_toolboxes'
% 'environment' 
% are presents in the same directory of the file 'rmt.mat'

directoryCPLEX = 'C:\Program Files\IBM\ILOG\CPLEX_Studio1271\cplex';

addpath(genpath([directoryCPLEX '\matlab\x64_win64']));
addpath(genpath([directoryCPLEX '\examples\src\matlab']));
addpath(genpath('aux_toolboxes'));
addpath(genpath('environment'));

% Execute rmt
rmt
