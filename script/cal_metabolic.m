clear

import org.opensim.modeling.*

%% Parameters
subjectID  = 'SP097-3';
resultsDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results', subjectID);

resultFolders = dir(fullfile(resultsDir, 'result_*'));
resultFolders = resultFolders([resultFolders.isdir]);

for ri = 1:numel(resultFolders)
    resultPath = fullfile(resultsDir, resultFolders(ri).name);
    mocoDir    = fullfile(resultPath, 'moco_result');
    analyDir   = fullfile(resultPath, 'analy_result');

    %% Find kinematics .sto (full gait, not _half)
    stoFiles = dir(fullfile(mocoDir, '*kinematics.sto'));
    stoFiles = stoFiles(~contains({stoFiles.name}, 'half'));
    if isempty(stoFiles)
        warning('No kinematics .sto found in %s, skipping.', mocoDir);
        continue
    end
    solutionPath = fullfile(mocoDir, stoFiles(1).name);

    %% Extract iter tag (e.g. iter01, iter02, iter03)
    tokens = regexp(stoFiles(1).name, '(iter\d+)', 'tokens');
    if isempty(tokens)
        iterTag = 'iter00';
    else
        iterTag = tokens{1}{1};
    end

    %% Find model .osim
    osmFiles = dir(fullfile(analyDir, '*.osim'));
    if isempty(osmFiles)
        warning('No .osim found in %s, skipping.', analyDir);
        continue
    end
    modelPath = fullfile(analyDir, osmFiles(1).name);

    fprintf('\n[%s] model   : %s\n', resultFolders(ri).name, osmFiles(1).name);
    fprintf('[%s] solution: %s\n',  resultFolders(ri).name, stoFiles(1).name);

    %% Load model and solution
    model    = Model(modelPath);
    solution = MocoTrajectory(solutionPath);

    %% Add metabolics component
    metabolics = Bhargava2004SmoothedMuscleMetabolics();
    metabolics.setName('metabolics');
    metabolics.set_use_smoothing(true);
    metabolics.set_include_negative_mechanical_work(true); % eccentric contraction -> negative possible
    metabolics.set_forbid_negative_total_power(true); % total negative -> usually impossible

    %% Collect muscles via component iteration
    muscleNames = {};
    compList = model.getComponentsList();
    it = compList.begin();
    while ~it.equals(compList.end())
        mus = Muscle.safeDownCast(it.deref());
        if ~isempty(mus)
            muscleNames{end+1} = char(mus.getName());
        end
        it.next();
    end
    fprintf('[%s] Found %d muscles.\n', resultFolders(ri).name, numel(muscleNames));

    %% Register muscles to metabolics
    for i = 1:numel(muscleNames)
        metabolics.addMuscle(muscleNames{i}, ...
            Muscle.safeDownCast(model.getComponent(muscleNames{i})));
    end

    model.addComponent(metabolics);
    model.finalizeConnections();
    state = model.initSystem();

    %% Get metabolics component handle
    metComp = Bhargava2004SmoothedMuscleMetabolics.safeDownCast( ...
                  model.getComponent('/metabolics'));

    %% Export states and controls tables
    statesTable   = solution.exportToStatesTable();
    controlsTable = solution.exportToControlsTable();

    statesTraj    = StatesTrajectory.createFromStatesTable(model, statesTable, true, true);
    nRow          = int32(statesTraj.getSize());
    nCtrlCols     = int32(controlsTable.getColumnLabels().size());

    t            = zeros(nRow, 1);
    met_rate_W   = zeros(nRow, 1);
    act_rate_W   = zeros(nRow, 1);
    maint_rate_W = zeros(nRow, 1);
    short_rate_W = zeros(nRow, 1);
    mech_rate_W  = zeros(nRow, 1);

    %% Evaluate frame-by-frame with controls applied to state
    for k = 0:nRow-1
        s = statesTraj.get(k);

        % Build controls vector from controls table (direct index mapping)
        controls = Vector(model.getNumControls(), 0.0);
        for j = 0:nCtrlCols-1
            col = controlsTable.getDependentColumnAtIndex(j);
            controls.set(j, col.get(k));
        end

        % Apply controls before realizing dynamics
        model.realizePosition(s);
        model.setControls(s, controls);
        model.realizeDynamics(s);

        t(k+1)            = s.getTime();
        met_rate_W(k+1)   = metComp.getTotalMetabolicRate(s);
        act_rate_W(k+1)   = metComp.getTotalActivationRate(s);
        maint_rate_W(k+1) = metComp.getTotalMaintenanceRate(s);
        short_rate_W(k+1) = metComp.getTotalShorteningRate(s);
        mech_rate_W(k+1)  = metComp.getTotalMechanicalWorkRate(s);
    end

    %% Integrate -> summary statistics
    total_energy_J = trapz(t, met_rate_W);
    avg_rate_W     = total_energy_J / (t(end) - t(1));
    totalMass      = model.getTotalMass(state);

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

    fprintf('[%s] Body mass              : %.2f kg\n',       resultFolders(ri).name, totalMass);
    fprintf('[%s] Distance               : %.3f m\n',        resultFolders(ri).name, distance_m);
    fprintf('[%s] Total metabolic energy : %.2f J\n',        resultFolders(ri).name, total_energy_J);
    fprintf('[%s] Average metabolic rate : %.2f W\n',        resultFolders(ri).name, avg_rate_W);
    fprintf('[%s] Cost of transport      : %.4f J/(kg*m)\n', resultFolders(ri).name, ...
            total_energy_J / (totalMass * distance_m));

    %% Write .sto output
    outName = sprintf('moco_WoC_solution_%s_metabolic.sto', iterTag);
    outPath = fullfile(mocoDir, outName);

    fid = fopen(outPath, 'w');
    if fid < 0
        error('Cannot open output file: %s', outPath);
    end

    fprintf(fid, '%s\n',                                  outName(1:end-4));
    fprintf(fid, 'nRows=%d\n',                            nRow);
    fprintf(fid, 'nColumns=6\n');
    fprintf(fid, 'inDegrees=no\n');
    fprintf(fid, 'Body mass(kg)=%.4f\n',                  totalMass);
    fprintf(fid, 'Distance(m)=%.4f\n',                    distance_m);
    fprintf(fid, 'Total metabolic energy(J)=%.4f\n',      total_energy_J);
    fprintf(fid, 'Avg metabolic rate(W)=%.4f\n',          avg_rate_W);
    fprintf(fid, 'endheader\n');
    fprintf(fid, 'time\tmet_rate_W\tact_rate_W\tmaint_rate_W\tshort_rate_W\tmech_rate_W\n');
    for k = 1:nRow
        fprintf(fid, '%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\n', ...
            t(k), met_rate_W(k), act_rate_W(k), maint_rate_W(k), short_rate_W(k), mech_rate_W(k));
    end

    fclose(fid);
    fprintf('[%s] Saved: %s\n', resultFolders(ri).name, outPath);
end
