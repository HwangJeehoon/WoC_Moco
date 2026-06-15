function cal_metabolic(resultName, iterNums, resultsRootDir)
% cal_metabolic  Bhargava2004 대사 비용 계산 및 .sto 저장
%
%   cal_metabolic(resultName)
%   cal_metabolic(resultName, iterNums)
%   cal_metabolic(resultName, iterNums, resultsRootDir)
%
%   resultName    : results/ 하위 폴더명 (예: 'SP097-3')
%   iterNums      : 처리할 result 인덱스 벡터 (예: 1:3). 생략 시 result_* 전체 자동탐색
%   resultsRootDir: results 루트 경로. 생략 시 이 함수 파일 기준 ../results

import org.opensim.modeling.*

%% 경로 설정
if nargin < 3 || isempty(resultsRootDir)
    resultsRootDir = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
end
resultBaseDir = fullfile(resultsRootDir, resultName);

%% 처리할 result 폴더 목록 결정
if nargin < 2 || isempty(iterNums)
    found = dir(fullfile(resultBaseDir, 'result_*'));
    found = found([found.isdir]);
    nums = arrayfun(@(f) str2double(regexp(f.name, '\d+$', 'match', 'once')), found);
    [~, ord] = sort(nums);
    found = found(ord);
    iterNums = arrayfun(@(f) str2double(regexp(f.name, '\d+', 'match', 'once')), found);
    iterNums = iterNums(~isnan(iterNums));
end

for ri = iterNums(:)'
    resultPath = fullfile(resultBaseDir, sprintf('result_%d', ri));
    mocoDir    = fullfile(resultPath, 'moco_result');
    analyDir   = fullfile(resultPath, 'analy_result');

    %% kinematics .sto 탐색 (_half 제외)
    stoFiles = dir(fullfile(mocoDir, '*kinematics.sto'));
    stoFiles = stoFiles(~contains({stoFiles.name}, 'half'));
    if isempty(stoFiles)
        warning('[%s/result_%d] kinematics .sto 없음, 건너뜀.', resultName, ri);
        continue
    end
    solutionPath = fullfile(mocoDir, stoFiles(1).name);

    tokens = regexp(stoFiles(1).name, '(iter\d+)', 'tokens');
    iterTag = 'iter00';
    if ~isempty(tokens), iterTag = tokens{1}{1}; end

    %% 모델 .osim 탐색
    osmFiles = dir(fullfile(analyDir, '*.osim'));
    if isempty(osmFiles)
        warning('[%s/result_%d] .osim 없음, 건너뜀.', resultName, ri);
        continue
    end
    modelPath = fullfile(analyDir, osmFiles(1).name);

    fprintf('\n[%s/result_%d] model   : %s\n', resultName, ri, osmFiles(1).name);
    fprintf('[%s/result_%d] solution: %s\n',  resultName, ri, stoFiles(1).name);

    %% 모델 및 솔루션 로드
    model    = Model(modelPath);
    solution = MocoTrajectory(solutionPath);

    %% 대사 컴포넌트 추가
    metabolics = Bhargava2004SmoothedMuscleMetabolics();
    metabolics.setName('metabolics');
    metabolics.set_use_smoothing(true);
    metabolics.set_include_negative_mechanical_work(true);
    metabolics.set_forbid_negative_total_power(true);

    %% 근육 수집 및 등록
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
    fprintf('[%s/result_%d] 근육 %d개 발견\n', resultName, ri, numel(muscleNames));

    for i = 1:numel(muscleNames)
        metabolics.addMuscle(muscleNames{i}, ...
            Muscle.safeDownCast(model.getComponent(muscleNames{i})));
    end

    model.addComponent(metabolics);
    model.finalizeConnections();
    state = model.initSystem();

    metComp = Bhargava2004SmoothedMuscleMetabolics.safeDownCast( ...
                  model.getComponent('/metabolics'));

    %% 상태 및 제어 테이블 export
    statesTable   = solution.exportToStatesTable();
    controlsTable = solution.exportToControlsTable();

    statesTraj = StatesTrajectory.createFromStatesTable(model, statesTable, true, true);
    nRow       = int32(statesTraj.getSize());
    nCtrlCols  = int32(controlsTable.getColumnLabels().size());

    t            = zeros(nRow, 1);
    met_rate_W   = zeros(nRow, 1);
    act_rate_W   = zeros(nRow, 1);
    maint_rate_W = zeros(nRow, 1);
    short_rate_W = zeros(nRow, 1);
    mech_rate_W  = zeros(nRow, 1);

    %% 프레임별 계산 (controls → state 반영 후 realizeDynamics)
    for k = 0:nRow-1
        s = statesTraj.get(k);

        controls = Vector(model.getNumControls(), 0.0);
        for j = 0:nCtrlCols-1
            col = controlsTable.getDependentColumnAtIndex(j);
            controls.set(j, col.get(k));
        end

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

    %% 요약 통계
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
        error('[%s/result_%d] 상태 열 없음: %s', resultName, ri, txStateName);
    end
    col        = statesTable.getDependentColumnAtIndex(txIdx);
    distance_m = col.get(col.size()-1) - col.get(0);

    fprintf('[%s/result_%d] Body mass              : %.2f kg\n',       resultName, ri, totalMass);
    fprintf('[%s/result_%d] Distance               : %.3f m\n',        resultName, ri, distance_m);
    fprintf('[%s/result_%d] Total metabolic energy : %.2f J\n',        resultName, ri, total_energy_J);
    fprintf('[%s/result_%d] Average metabolic rate : %.2f W\n',        resultName, ri, avg_rate_W);
    fprintf('[%s/result_%d] Cost of transport      : %.4f J/(kg*m)\n', resultName, ri, ...
            total_energy_J / (totalMass * distance_m));

    %% .sto 저장
    outName = sprintf('moco_WoC_solution_%s_metabolic.sto', iterTag);
    outPath = fullfile(mocoDir, outName);

    fid = fopen(outPath, 'w');
    if fid < 0
        error('출력 파일을 열 수 없음: %s', outPath);
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
    fprintf('[%s/result_%d] 저장 완료: %s\n', resultName, ri, outPath);
end
end
