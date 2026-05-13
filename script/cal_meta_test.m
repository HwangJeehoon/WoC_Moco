clear

import org.opensim.modeling.*

%% 0) Paths
if isempty(mfilename)
    thisFile = matlab.desktop.editor.getActiveFilename;
else
    thisFile = mfilename('fullpath');
end
thisDir      = fileparts(thisFile);
modelPath    = fullfile(thisDir, '2D_gait.osim');
solutionPath = fullfile(thisDir, 'moco_WoC_Solution_iter03_kinematics.sto');

%% 1) Load model and Moco trajectory
model    = Model(modelPath);
solution = MocoTrajectory(solutionPath);

%% 2) Add metabolics component
metabolics = Bhargava2004SmoothedMuscleMetabolics();
metabolics.setName('metabolics');
metabolics.set_use_smoothing(true);
metabolics.set_include_negative_mechanical_work(true);
metabolics.set_forbid_negative_total_power(false);

%% 3) Collect muscles by iterating all components (getMuscles() fails when
%     muscles live inside sub-components)
muscleNames = {};
compList = model.getComponentsList();
it = compList.begin();
while ~it.equals(compList.end())
    comp = it.deref();
    mus  = Muscle.safeDownCast(comp);
    if ~isempty(mus)
        muscleNames{end+1} = char(mus.getName()); 
    end
    it.next();
end
fprintf('Found %d muscles via component iteration.\n', numel(muscleNames));

%% 4) Register muscles to metabolics
for i = 1:numel(muscleNames)
    musName = muscleNames{i};
    metabolics.addMuscle(musName, Muscle.safeDownCast(model.getComponent(musName)));
end

model.addComponent(metabolics);
model.finalizeConnections();
state = model.initSystem();

%% 5) Get metabolics output handles
metComp   = Bhargava2004SmoothedMuscleMetabolics.safeDownCast( ...
                model.getComponent('/metabolics'));
out_met   = metComp.getOutput('total_metabolic_rate');
out_act   = metComp.getOutput('total_activation_rate');
out_main  = metComp.getOutput('total_maintenance_rate');
out_short = metComp.getOutput('total_shortening_rate');
out_mech  = metComp.getOutput('total_mechanical_work_rate');

%% 6) Build states trajectory and evaluate outputs frame-by-frame
statesTable = solution.exportToStatesTable();
statesTraj  = StatesTrajectory.createFromStatesTable(model, statesTable, true, true);
nRow        = int32(statesTraj.getSize());

t            = zeros(nRow, 1);
met_rate_W   = zeros(nRow, 1);
act_rate_W   = zeros(nRow, 1);
maint_rate_W = zeros(nRow, 1);
short_rate_W = zeros(nRow, 1);
mech_rate_W  = zeros(nRow, 1);

for k = 0:nRow-1
    s = statesTraj.get(k);
    model.realizeDynamics(s);
    t(k+1)            = s.getTime();
    met_rate_W(k+1)   = str2double(char(out_met.getValueAsString(s)));
    act_rate_W(k+1)   = str2double(char(out_act.getValueAsString(s)));
    maint_rate_W(k+1) = str2double(char(out_main.getValueAsString(s)));
    short_rate_W(k+1) = str2double(char(out_short.getValueAsString(s)));
    mech_rate_W(k+1)  = str2double(char(out_mech.getValueAsString(s)));
end

%% 7) Integrate -> total energy and average rate
total_energy_J = trapz(t, met_rate_W);
avg_rate_W     = total_energy_J / (t(end) - t(1));

%% 8) Cost of transport
totalMass = model.getTotalMass(state);

% Find pelvis_tx column and compute displacement
txStateName = '/jointset/groundPelvis/pelvis_tx/value';
labels = statesTable.getColumnLabels();
txIdx  = -1;
for i = 0:labels.size()-1
    if strcmp(char(labels.get(i)), txStateName)
        txIdx = i;
        break;
    end
end
if txIdx < 0
    error('State not found: %s', txStateName);
end
col        = statesTable.getDependentColumnAtIndex(txIdx);
distance_m = col.get(col.size()-1) - col.get(0);

cot = total_energy_J / (totalMass * distance_m);

%% 9) Display results
fprintf('Total metabolic energy : %.2f J\n',           total_energy_J);
fprintf('Average metabolic rate : %.2f W\n',           avg_rate_W);
fprintf('Body mass              : %.2f kg\n',          totalMass);
fprintf('Distance               : %.3f m\n',           distance_m);
fprintf('Cost of transport      : %.4f J/(kg·m)\n',    cot);
