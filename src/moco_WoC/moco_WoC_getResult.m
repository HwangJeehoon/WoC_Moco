function moco_WoC_getResult(moco_WoC_Solution, outputDir, opts)
% moco_WoC_getResult
%
%   moco_WoC_loop에서 얻은 MocoTrajectory를 받아
%   kinematics .sto 및 GRF .sto를 저장하는 함수.
%
% 입력:
%   moco_WoC_Solution : MocoTrajectory (moco_WoC_loop 결과)
%   outputDir         : 결과를 저장할 디렉터리 경로
%   opts              : (선택) struct
%       .modelPath : GRF 생성에 사용할 osim 모델 경로
%                    (default = '2D_gait_AFO.osim')
%       .prefix    : 출력 파일 이름 prefix
%                    (default = 'moco_WoC_Solution')
%       .gaitMode  : 'modeSym'  - 반 걸음 결과를 createPeriodicTrajectory로
%                                 확장 → full + half 파일 모두 저장 (default)
%                   'modeAsym' - 결과가 이미 full stride
%                                 → full 파일만 저장 (half 저장 없음)
%
% modeSym 저장 파일:
%   {prefix}_kinematics.sto      (full stride, createPeriodicTrajectory 결과)
%   {prefix}_kinematics_half.sto (half stride, 다음 iter의 initial guess용)
%   {prefix}_GRF.sto             (full stride 기준)
%   {prefix}_metabolic.sto       (Bhargava2004 대사 비용, full stride 기준)
%
% modeAsym 저장 파일:
%   {prefix}_kinematics.sto      (full stride = solve 결과 그대로)
%   {prefix}_GRF.sto             (full stride 기준)
%   {prefix}_metabolic.sto       (Bhargava2004 대사 비용, full stride 기준)

    import org.opensim.modeling.*

    if nargin < 2
        error('Usage: moco_WoC_getResult(moco_WoC_Solution, outputDir, [opts])');
    end
    if nargin < 3
        opts = struct();
    end

    if ~isfield(opts, 'modelPath') || isempty(opts.modelPath)
        error('moco_WoC_getResult: opts.modelPath 가 지정되지 않았습니다. GRF 추출에 사용할 .osim 절대경로를 반드시 전달하세요.');
    end
    modelPath = opts.modelPath;
    prefix    = getOpt(opts, 'prefix',    'moco_WoC_Solution');
    gaitMode  = getOpt(opts, 'gaitMode',  'modeSym');

    if ~exist(outputDir, 'dir')
        mkdir(outputDir);
    end

    %----------------------------------------------------------
    % 1. gaitMode에 따른 trajectory 처리 및 kinematics 저장
    %----------------------------------------------------------
    kinStoPath = fullfile(outputDir, [prefix '_kinematics.sto']);

    if strcmpi(gaitMode, 'modeSym')
        % 반 걸음 → createPeriodicTrajectory → full stride
        solution_full = opensimMoco.createPeriodicTrajectory(moco_WoC_Solution);
        solution_half = moco_WoC_Solution;

        solution_full.write(kinStoPath);
        solution_half.write(fullfile(outputDir, [prefix '_kinematics_half.sto']));

    elseif strcmpi(gaitMode, 'modeAsym')
        % solve 결과가 이미 full stride → 그대로 저장
        solution_full = moco_WoC_Solution;

        solution_full.write(kinStoPath);

    else
        error('moco_WoC_getResult: 알 수 없는 gaitMode = ''%s''. modeSym 또는 modeAsym 이어야 합니다.', gaitMode);
    end

    %----------------------------------------------------------
    % 2. GRF 추출 및 저장 (항상 full stride 기준)
    %----------------------------------------------------------
    model = Model(modelPath);
    model.initSystem();

    contact_r = StdVectorString();
    contact_l = StdVectorString();
    contact_r.add('contactHeel_r');
    contact_r.add('contactFront_r');
    contact_l.add('contactHeel_l');
    contact_l.add('contactFront_l');

    externalForcesTableFlat = opensimMoco.createExternalLoadsTableForGait( ...
        model, solution_full, contact_r, contact_l);

    STOFileAdapter.write(externalForcesTableFlat, fullfile(outputDir, [prefix '_GRF.sto']));

    %----------------------------------------------------------
    % 3. Metabolic cost (Bhargava2004)
    %----------------------------------------------------------
    metModel = Model(modelPath);

    metabolics = Bhargava2004SmoothedMuscleMetabolics();
    metabolics.setName('metabolics');
    metabolics.set_use_smoothing(true);
    metabolics.set_include_negative_mechanical_work(true);
    metabolics.set_forbid_negative_total_power(true);

    muscleNames = {};
    compList = metModel.getComponentsList();
    it = compList.begin();
    while ~it.equals(compList.end())
        mus = Muscle.safeDownCast(it.deref());
        if ~isempty(mus)
            muscleNames{end+1} = char(mus.getName());
        end
        it.next();
    end

    for i = 1:numel(muscleNames)
        metabolics.addMuscle(muscleNames{i}, ...
            Muscle.safeDownCast(metModel.getComponent(muscleNames{i})));
    end

    metModel.addComponent(metabolics);
    metModel.finalizeConnections();
    metState = metModel.initSystem();

    metComp = Bhargava2004SmoothedMuscleMetabolics.safeDownCast( ...
                  metModel.getComponent('/metabolics'));

    statesTable   = solution_full.exportToStatesTable();
    controlsTable = solution_full.exportToControlsTable();

    statesTraj = StatesTrajectory.createFromStatesTable(metModel, statesTable, true, true);
    nRow       = int32(statesTraj.getSize());
    nCtrlCols  = int32(controlsTable.getColumnLabels().size());

    t            = zeros(nRow, 1);
    met_rate_W   = zeros(nRow, 1);
    act_rate_W   = zeros(nRow, 1);
    maint_rate_W = zeros(nRow, 1);
    short_rate_W = zeros(nRow, 1);
    mech_rate_W  = zeros(nRow, 1);

    for k = 0:nRow-1
        s = statesTraj.get(k);

        controls = Vector(metModel.getNumControls(), 0.0);
        for j = 0:nCtrlCols-1
            col = controlsTable.getDependentColumnAtIndex(j);
            controls.set(j, col.get(k));
        end

        metModel.realizePosition(s);
        metModel.setControls(s, controls);
        metModel.realizeDynamics(s);

        t(k+1)            = s.getTime();
        met_rate_W(k+1)   = metComp.getTotalMetabolicRate(s);
        act_rate_W(k+1)   = metComp.getTotalActivationRate(s);
        maint_rate_W(k+1) = metComp.getTotalMaintenanceRate(s);
        short_rate_W(k+1) = metComp.getTotalShorteningRate(s);
        mech_rate_W(k+1)  = metComp.getTotalMechanicalWorkRate(s);
    end

    total_energy_J = trapz(t, met_rate_W);
    avg_rate_W     = total_energy_J / (t(end) - t(1));
    totalMass      = metModel.getTotalMass(metState);

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
        error('moco_WoC_getResult: 상태 열 없음: %s', txStateName);
    end
    txCol      = statesTable.getDependentColumnAtIndex(txIdx);
    distance_m = txCol.get(txCol.size()-1) - txCol.get(0);

    metOutName = [prefix '_metabolic.sto'];
    metOutPath = fullfile(outputDir, metOutName);
    fid = fopen(metOutPath, 'w');
    if fid < 0
        error('moco_WoC_getResult: metabolic .sto 파일을 열 수 없음: %s', metOutPath);
    end
    fprintf(fid, '%s\n',                                  metOutName(1:end-4));
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

    fprintf('moco_WoC_getResult: metabolic 저장 완료 → %s\n', metOutPath);
end


%% ---- 옵션 읽기용 헬퍼 ----
function val = getOpt(s, field, defaultVal)
    if isfield(s, field) && ~isempty(s.(field))
        val = s.(field);
    else
        val = defaultVal;
    end
end
