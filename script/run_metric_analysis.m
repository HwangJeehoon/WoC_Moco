close all;
clc;
pathCurrent= fileparts(mfilename('fullpath'));
pathMain = pathCurrent + "/../src/data_analysis";
addpath(pathMain);

% moco_git/results 폴더 위치 지정
pathResult = fullfile(pathCurrent, '..', 'results');

get_metric(pathResult);