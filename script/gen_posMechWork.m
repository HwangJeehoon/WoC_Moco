% gen_posMechWork.m
%
% 각 results/<case>/result_#에 대해 다음을 계산한다.
%   1) AFO GeometryPath의 ankle moment arm [m]
%   2) input torque = moment arm * AFO input * optimal force [Nm]
%   3) positive input power = max(input torque * ankle speed, 0) [W]
%   4) positive mechanical work = integral(positive input power) [J]
%
% 실행 제어 환경변수 (run_posMechWork_batches.ps1가 설정):
%   POS_MECH_WORK_RESULTS_DIR : 처리할 results 폴더 절대경로
%   POS_MECH_WORK_BATCH_INDEX : 1-based batch 번호 (기본 1)
%   POS_MECH_WORK_BATCH_SIZE  : batch당 result 수 (기본 500)
%
% 출력:
%   results/<case>/result_#/analy_result/AFO_input_posMechWork.sto

clc;

import org.opensim.modeling.*

%% Path setup
if isempty(mfilename)
    thisFile = matlab.desktop.editor.getActiveFilename;
else
    thisFile = mfilename('fullpath');
end
rootDir    = fileparts(fileparts(thisFile));

[resultsDir, batchIndex, batchSize] = getBatchConfiguration(rootDir);

assert(exist(resultsDir, 'dir') == 7, 'results folder not found: %s', resultsDir);

%% Process every results/<case>/result_# folder
caseEntries = dir(resultsDir);
caseEntries = caseEntries([caseEntries.isdir]);
caseEntries = caseEntries(~ismember({caseEntries.name}, {'.', '..'}));

processed = 0;
skipped   = 0;
seen      = 0;
firstJob  = (batchIndex - 1) * batchSize + 1;
lastJob   = batchIndex * batchSize;
stopProcessing = false;

fprintf('gen_posMechWork batch %d: result %d through %d (batch size %d)\n', ...
    batchIndex, firstJob, lastJob, batchSize);

for caseIdx = 1:numel(caseEntries)
    caseDir = fullfile(resultsDir, caseEntries(caseIdx).name);
    resultEntries = dir(fullfile(caseDir, 'result_*'));
    resultEntries = resultEntries([resultEntries.isdir]);

    for resultIdx = 1:numel(resultEntries)
        resultDir = fullfile(caseDir, resultEntries(resultIdx).name);
        seen = seen + 1;

        if seen < firstJob
            continue;
        end
        if seen > lastJob
            stopProcessing = true;
            break;
        end

        try
            resultNum = getResultNumber(resultEntries(resultIdx).name);
            analyDir  = fullfile(resultDir, 'analy_result');
            mocoDir   = fullfile(resultDir, 'moco_result');
            controlDir = fullfile(resultDir, 'control_result');

            modelPath  = findSingleModel(analyDir);
            kinPath    = findKinematics(mocoDir, resultNum);
            controlPath = fullfile(controlDir, 'control.sto');
            speedPath   = fullfile(analyDir, '2D_gait_AFO_pc_Kinematics_u.sto');
            outputPath  = fullfile(analyDir, 'AFO_input_posMechWork.sto');

            assert(exist(controlPath, 'file') == 2, 'control file not found: %s', controlPath);
            assert(exist(speedPath, 'file') == 2, 'ankle-speed file not found: %s', speedPath);

            fprintf('[%s/%s]\n', caseEntries(caseIdx).name, resultEntries(resultIdx).name);
            [workR, workL] = extractPosMechWork( ...
                modelPath, kinPath, controlPath, speedPath, outputPath);
            fprintf('  wrote: %s  (right %.6g J, left %.6g J, total %.6g J)\n', ...
                outputPath, workR, workL, workR + workL);
            processed = processed + 1;

        catch ME
            warning('gen_posMechWork:SkippedResult', ...
                'Skipping %s: %s', resultDir, ME.message);
            skipped = skipped + 1;
        end
    end

    if stopProcessing
        break;
    end
end

fprintf('\ngen_posMechWork batch %d complete: %d processed, %d skipped.\n', ...
    batchIndex, processed, skipped);

% A direct MATLAB run benefits from a GC request. The batch launcher gives
% the stronger guarantee by terminating this MATLAB process after this script.
java.lang.System.gc();
java.lang.System.runFinalization();

function [workR, workL] = extractPosMechWork(modelPath, kinPath, controlPath, speedPath, outputPath)
    import org.opensim.modeling.*

    model = Model(modelPath);
    model.initSystem();

    % Moco solution에는 states와 controls가 함께 있으므로 MocoTrajectory로
    % 읽은 뒤 states table만 분리한다.
    trajectory  = MocoTrajectory(kinPath);
    statesTable = trajectory.exportToStatesTable();
    statesTraj  = StatesTrajectory.createFromStatesTable( ...
        model, statesTable, true, true);

    % These wrappers own OpenSim C++ memory. Capture their handles directly
    % in onCleanup so they are released on both success and error paths.
    resourceCleanup = onCleanup(@() releaseOpenSimObjects( ...
        statesTraj, statesTable, trajectory, model)); %#ok<NASGU>

    assert(model.hasComponent('/AFO_r') && model.hasComponent('/AFO_l'), ...
        'Model must contain /AFO_r and /AFO_l: %s', modelPath);
    afoR = PathActuator.safeDownCast(model.updComponent('/AFO_r'));
    afoL = PathActuator.safeDownCast(model.updComponent('/AFO_l'));
    assert(~isempty(afoR) && ~isempty(afoL), 'AFO components must be PathActuators.');

    ankleR = Coordinate.safeDownCast( ...
        model.updComponent('/jointset/ankle_r/ankle_angle_r'));
    ankleL = Coordinate.safeDownCast( ...
        model.updComponent('/jointset/ankle_l/ankle_angle_l'));

    nRows = statesTraj.getSize();
    time  = zeros(nRows, 1);
    armR  = zeros(nRows, 1);
    armL  = zeros(nRows, 1);
    pathR = afoR.getGeometryPath();
    pathL = afoL.getGeometryPath();

    for row = 0:nRows-1
        state = statesTraj.get(row);
        model.realizePosition(state);

        time(row + 1) = state.getTime();
        armR(row + 1) = pathR.computeMomentArm(state, ankleR);
        armL(row + 1) = pathL.computeMomentArm(state, ankleL);
    end

    [timeControl, inputR, inputL] = readStoColumns( ...
        controlPath, 'AFO_r', 'AFO_l');
    [timeSpeed, speedR, speedL] = readStoColumns( ...
        speedPath, 'ankle_angle_r', 'ankle_angle_l');

    % 각 source의 표본 시간이 다를 수 있으므로 kinematics time grid로 보간한다.
    % Control이 짧으면 남은 Moco time grid에는 AFO input = 0을 사용한다.
    % Control이 길면 Moco time grid 밖의 control 값은 출력하지 않는다.
    inputR = resampleControlToMocoTime(timeControl, inputR, time, controlPath, 'AFO_r');
    inputL = resampleControlToMocoTime(timeControl, inputL, time, controlPath, 'AFO_l');
    speedR = interpolateTo(timeSpeed, speedR, time, speedPath, 'ankle_angle_r');
    speedL = interpolateTo(timeSpeed, speedL, time, speedPath, 'ankle_angle_l');

    % PathActuator force = input control * optimal force.
    torqueR = armR .* inputR .* afoR.getOptimalForce();
    torqueL = armL .* inputL .* afoL.getOptimalForce();

    % Output power is clipped to the positive contribution used for work.
    powerR = max(torqueR .* speedR, 0);
    powerL = max(torqueL .* speedL, 0);
    workR  = trapz(time, powerR);
    workL  = trapz(time, powerL);

    writePosMechWorkSto(outputPath, time, armR, armL, torqueR, torqueL, ...
        powerR, powerL, workR, workL);
end

function valuesOnTarget = resampleControlToMocoTime(sourceTime, sourceValues, targetTime, sourcePath, label)
    assert(all(diff(sourceTime) > 0), ...
        'Time is not strictly increasing in %s (%s).', sourcePath, label);

    % `interp1(..., 0)` sets input to zero before/after the control range.
    % Thus the output always uses Moco kinematics time and never extrapolates
    % a nonzero control value outside the recorded control interval.
    valuesOnTarget = interp1(sourceTime, sourceValues, targetTime, 'linear', 0);
    assert(~any(isnan(valuesOnTarget)), ...
        'Interpolation produced NaN for %s (%s).', sourcePath, label);
end

function valuesOnTarget = interpolateTo(sourceTime, sourceValues, targetTime, sourcePath, label)
    timeTolerance = 1e-6; % AnalyzeTool STO time precision can truncate endpoints.
    assert(all(diff(sourceTime) > 0), ...
        'Time is not strictly increasing in %s (%s).', sourcePath, label);
    assert(targetTime(1) >= sourceTime(1) - timeTolerance && ...
           targetTime(end) <= sourceTime(end) + timeTolerance, ...
        'Target time range is outside %s (%s).', sourcePath, label);

    % Do not extrapolate: only roundoff-sized endpoint differences are clamped.
    targetTime = min(max(targetTime, sourceTime(1)), sourceTime(end));
    valuesOnTarget = interp1(sourceTime, sourceValues, targetTime, 'linear');
    assert(~any(isnan(valuesOnTarget)), ...
        'Interpolation produced NaN for %s (%s).', sourcePath, label);
end

function [time, value1, value2] = readStoColumns(stoPath, label1, label2)
    import org.opensim.modeling.*

    assert(exist(stoPath, 'file') == 2, 'STO file not found: %s', stoPath);
    storage = Storage(stoPath);
    nRows   = storage.getSize();
    assert(nRows >= 2, 'At least two rows are required in %s.', stoPath);

    timeArray = ArrayDouble();
    value1Array = ArrayDouble();
    value2Array = ArrayDouble();
    resourceCleanup = onCleanup(@() releaseStorageObjects( ...
        timeArray, value1Array, value2Array, storage)); %#ok<NASGU>
    storage.getTimeColumn(timeArray);
    storage.getDataColumn(label1, value1Array);
    storage.getDataColumn(label2, value2Array);

    time   = zeros(nRows, 1);
    value1 = zeros(nRows, 1);
    value2 = zeros(nRows, 1);
    for row = 0:nRows-1
        time(row + 1)   = timeArray.get(row);
        value1(row + 1) = value1Array.get(row);
        value2(row + 1) = value2Array.get(row);
    end
end

function releaseOpenSimObjects(statesTraj, statesTable, trajectory, model)
    % Delete children before their parent model. Components such as afoR and
    % pathR are borrowed from the model and must not be deleted separately.
    deleteOpenSimObject(statesTraj);
    deleteOpenSimObject(statesTable);
    deleteOpenSimObject(trajectory);
    deleteOpenSimObject(model);
end

function releaseStorageObjects(timeArray, value1Array, value2Array, storage)
    deleteOpenSimObject(timeArray);
    deleteOpenSimObject(value1Array);
    deleteOpenSimObject(value2Array);
    deleteOpenSimObject(storage);
end

function deleteOpenSimObject(object)
    if isempty(object)
        return;
    end
    try
        object.delete();
    catch cleanupError
        warning('gen_posMechWork:OpenSimCleanupFailed', ...
            'OpenSim cleanup failed: %s', cleanupError.message);
    end
end

function writePosMechWorkSto(outputPath, time, armR, armL, torqueR, torqueL, powerR, powerL, workR, workL)
    fid = fopen(outputPath, 'w');
    assert(fid >= 0, 'Could not create output file: %s', outputPath);
    cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>

    fprintf(fid, 'AFO_input_posMechWork\n');
    fprintf(fid, 'version=1\n');
    fprintf(fid, 'nRows=%d\n', numel(time));
    fprintf(fid, 'nColumns=7\n');
    fprintf(fid, 'inDegrees=no\n');
    fprintf(fid, 'posMechWork_tot=%.16g\n', workR + workL);
    fprintf(fid, 'posMechWork_r=%.16g\n', workR);
    fprintf(fid, 'posMechWork_l=%.16g\n', workL);
    fprintf(fid, 'endheader\n');
    fprintf(fid, ['time\tAFO_r_moment_arm\tAFO_l_moment_arm\t' ...
        'AFO_r_input_torque\tAFO_l_input_torque\t' ...
        'AFO_r_input_power\tAFO_l_input_power\n']);

    for row = 1:numel(time)
        fprintf(fid, '%.16g\t%.16g\t%.16g\t%.16g\t%.16g\t%.16g\t%.16g\n', ...
            time(row), armR(row), armL(row), torqueR(row), torqueL(row), ...
            powerR(row), powerL(row));
    end
end

function modelPath = findSingleModel(analyDir)
    assert(exist(analyDir, 'dir') == 7, 'analy_result folder not found: %s', analyDir);
    models = dir(fullfile(analyDir, '*.osim'));
    assert(numel(models) == 1, ...
        'Expected exactly one .osim in %s; found %d.', analyDir, numel(models));
    modelPath = fullfile(models(1).folder, models(1).name);
end

function kinPath = findKinematics(mocoDir, resultNum)
    assert(exist(mocoDir, 'dir') == 7, 'moco_result folder not found: %s', mocoDir);

    expectedName = sprintf('moco_WoC_Solution_iter%02d_kinematics.sto', resultNum);
    expectedPath = fullfile(mocoDir, expectedName);
    if exist(expectedPath, 'file') == 2
        kinPath = expectedPath;
        return;
    end

    candidates = dir(fullfile(mocoDir, 'moco_WoC_Solution_iter*_kinematics.sto'));
    assert(numel(candidates) == 1, ...
        ['Expected %s, or exactly one matching kinematics file in %s; ' ...
         'found %d.'], expectedName, mocoDir, numel(candidates));
    kinPath = fullfile(candidates(1).folder, candidates(1).name);
end

function resultNum = getResultNumber(folderName)
    tokens = regexp(folderName, '^result_(\d+)$', 'tokens', 'once');
    assert(~isempty(tokens), 'Unexpected result folder name: %s', folderName);
    resultNum = str2double(tokens{1});
end

function [resultsDir, batchIndex, batchSize] = getBatchConfiguration(rootDir)
    % Keep the current project-specific default for an interactive run.
    resultsDir = getenv('POS_MECH_WORK_RESULTS_DIR');
    if isempty(resultsDir)
        resultsDir = fullfile(rootDir, 'results_prime5');
    end

    batchIndex = getPositiveIntegerEnvironmentVariable( ...
        'POS_MECH_WORK_BATCH_INDEX', 1);
    batchSize = getPositiveIntegerEnvironmentVariable( ...
        'POS_MECH_WORK_BATCH_SIZE', 500);
end

function value = getPositiveIntegerEnvironmentVariable(name, defaultValue)
    rawValue = getenv(name);
    if isempty(rawValue)
        value = defaultValue;
        return;
    end

    value = str2double(rawValue);
    assert(isfinite(value) && value >= 1 && value == floor(value), ...
        '%s must be a positive integer; received "%s".', name, rawValue);
end
