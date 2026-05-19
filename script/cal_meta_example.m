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

%% 6) Build states trajectory + controls trajectory and evaluate outputs frame-by-frame
statesTable   = solution.exportToStatesTable();
controlsTable = solution.exportToControlsTable();

statesTraj = StatesTrajectory.createFromStatesTable(model, statesTable, true, true);
nRow       = int32(statesTraj.getSize());

t            = zeros(nRow, 1);
met_rate_W   = zeros(nRow, 1);
act_rate_W   = zeros(nRow, 1);
maint_rate_W = zeros(nRow, 1);
short_rate_W = zeros(nRow, 1);
mech_rate_W  = zeros(nRow, 1);

%% 6-1) Read control labels
controlLabels = controlsTable.getColumnLabels();
nControlsCols = int32(controlLabels.size());

controlActs  = cell(nControlsCols, 1);
controlNames = cell(nControlsCols, 1);

fprintf('\nControl label mapping:\n');

nMatched = 0;

for j = 0:nControlsCols-1
    rawName = char(controlLabels.get(j));
    controlNames{j+1} = rawName;

    % Clean control label into candidate muscle/actuator names
    candidates = {};

    % Original label
    candidates{end+1} = rawName;

    % Remove common suffixes
    tmp = rawName;
    tmp = regexprep(tmp, '/control$', '');
    tmp = regexprep(tmp, '/excitation$', '');
    tmp = regexprep(tmp, '\|control$', '');
    tmp = regexprep(tmp, '\|excitation$', '');
    candidates{end+1} = tmp;

    % Remove leading slash
    if startsWith(tmp, '/')
        candidates{end+1} = tmp(2:end);
    end

    % Use last path element
    parts = split(string(tmp), "/");
    parts = parts(parts ~= "");
    if ~isempty(parts)
        lastName = char(parts(end));
        candidates{end+1} = lastName;
    end

    % Some Moco labels may look like /forceset/soleus_r
    % Last path element should become soleus_r.

    % Remove duplicates
    candidates = unique(candidates, 'stable');

    % Try to find matching Actuator/Muscle
    act = [];

    for c = 1:numel(candidates)
        cand = candidates{c};

        if isempty(cand)
            continue
        end

        try
            comp = model.getComponent(cand);
            tmpAct = Actuator.safeDownCast(comp);

            if ~isempty(tmpAct)
                act = tmpAct;
                break
            end
        catch
            % Try next candidate
        end
    end

    % If still not found, search through all model components by name
    if isempty(act)
        compList = model.getComponentsList();
        it = compList.begin();

        while ~it.equals(compList.end())
            comp = it.deref();
            tmpAct = Actuator.safeDownCast(comp);

            if ~isempty(tmpAct)
                actName = char(tmpAct.getName());

                for c = 1:numel(candidates)
                    cand = candidates{c};

                    if strcmp(actName, cand)
                        act = tmpAct;
                        break
                    end
                end
            end

            if ~isempty(act)
                break
            end

            it.next();
        end
    end

    % Store result
    if isempty(act)
        fprintf('  [%02d] %-50s -> NOT MATCHED\n', j, rawName);
    else
        fprintf('  [%02d] %-50s -> %s\n', ...
            j, rawName, char(act.getName()));
        nMatched = nMatched + 1;
    end

    controlActs{j+1} = act;
end

fprintf('Matched %d / %d control columns.\n', nMatched, nControlsCols);

if nMatched == 0
    error(['No controls matched to actuators/muscles. ', ...
           'Check printed control labels and model actuator names.']);
end

% 6-2) Optional debug muscle
debugMus = Muscle.safeDownCast(model.getComponent('soleus_r'));

% 6-3) Evaluate frame-by-frame with controls applied
for k = 0:nRow-1
    s = statesTraj.get(k);

    % Fresh model controls vector for this frame
    controls = Vector(model.getNumControls(), 0.0);

    % Directly fill controls vector from controlsTable
    for j = 0:nControlsCols-1
        col = controlsTable.getDependentColumnAtIndex(j);
        u   = col.get(k);

        controls.set(j, u);
    end

    % IMPORTANT:
    % setControls requires the State to be realized at least to Position.
    model.realizePosition(s);

    % Apply controls to State
    model.setControls(s, controls);

    % Now realize higher stage for metabolics outputs
    model.realizeDynamics(s);
    % If needed, use this instead:
    % model.realizeReport(s);

    t(k+1) = s.getTime();

    met_rate_W(k+1)   = metComp.getTotalMetabolicRate(s);
    act_rate_W(k+1)   = metComp.getTotalActivationRate(s);
    maint_rate_W(k+1) = metComp.getTotalMaintenanceRate(s);
    short_rate_W(k+1) = metComp.getTotalShorteningRate(s);
    mech_rate_W(k+1)  = metComp.getTotalMechanicalWorkRate(s);

    if k < 10 && ~isempty(debugMus)
        fprintf('t %.3f | soleus_r activation %.6f excitation %.6f | actRate %.6f maintRate %.6f\n', ...
            s.getTime(), ...
            debugMus.getActivation(s), ...
            debugMus.getExcitation(s), ...
            act_rate_W(k+1), ...
            maint_rate_W(k+1));
    end
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


%% debugging

labels = statesTable.getColumnLabels();
for i = 0:labels.size()-1
    name = char(labels.get(i));
    if contains(name, 'activation') || contains(name, 'fiber_length')
        fprintf('%s\n', name);
    end
end
for i = 1:numel(muscleNames)
    mus = Muscle.safeDownCast(model.getComponent(muscleNames{i}));
    fprintf('%s -> empty? %d\n', muscleNames{i}, isempty(mus));
end

labels = statesTable.getColumnLabels();
for i = 0:labels.size()-1
    name = char(labels.get(i));
    if contains(name, '/soleus_r/activation')
        col = statesTable.getDependentColumnAtIndex(i);
        vals = zeros(col.size(),1);
        for k = 0:col.size()-1
            vals(k+1) = col.get(k);
        end
        fprintf('soleus_r activation min %.6f max %.6f mean %.6f\n', ...
            min(vals), max(vals), mean(vals));
    end
end

mus = Muscle.safeDownCast(model.getComponent('soleus_r'));
for k = 0:min(10,nRow-1)
    s = statesTraj.get(k);
    model.realizeDynamics(s);
    fprintf('t %.3f activation %.6f\n', ...
        s.getTime(), mus.getActivation(s));
end

mus = Muscle.safeDownCast(model.getComponent('soleus_r'));
for k = 0:min(10,nRow-1)
    s = statesTraj.get(k);
    model.realizeDynamics(s);

    fprintf('t %.3f activation %.6f excitation %.6f\n', ...
        s.getTime(), mus.getActivation(s), mus.getExcitation(s));
end
%