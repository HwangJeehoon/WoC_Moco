close all;
clc;
pathCurrent= fileparts(mfilename('fullpath'));
pathMain = pathCurrent + "/../src/data_analysis";
addpath(pathMain);

% moco_git\result 폴더 위치 지정
pathResult = "E:\OneDrive-JJ\OneDrive\Biomechanical parameter optimization based wearable robot control\moco\moco_git\WoC_Moco\results";

get_metric(pathResult);