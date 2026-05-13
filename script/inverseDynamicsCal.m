clc;
clear;

%% baseFolder
% if isempty(mfilename)
%     thisFile = matlab.desktop.editor.getActiveFilename;
% else
%     thisFile = mfilename("fullpath");
% end
% baseFolder = fileparts(thisFile);

%%
import org.opensim.modeling.*

modelName = '2D_gait_AFO_pc_50BW_3.osim';
solutionFile = '2D_gait_AFO_pc_Kinematics_q.sto';
xmlFile = 'test1.xml';

idTool = InverseDynamicsTool(xmlFile);
idTool.setModelFileName(modelName)
idTool.setCoordinatesFileName(solutionFile)
% idTool.setExcludedForces('');

idTool.setResultsDir('ID_Results')
idTool.run()