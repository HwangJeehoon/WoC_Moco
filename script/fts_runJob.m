function fts_runJob(cfg, inputPath, modelBasePath, AnalySetupPath, scriptDir)
% fts_runJob
%
%   fixed_time_spline.m 에서 단일 job 을 실행하는 헬퍼 함수.
%   직접 호출하지 말고 fixed_time_spline.m 을 통해 실행하세요.
%
%   cfg 필드 (fixed_time_spline.m 의 defaults/job 병합 결과):
%     .result_name   : 결과 폴더명 (필수)
%     .model         : 모델 파일명 (models/ 기준)
%     .iter          : 반복 횟수
%     .fixedInterval : [t_start, t_end] 초
%     .trigger/rise/flat/fall/maxVal : spline 파라미터
%     .mocoEffort / .mocoFinalTime   : Moco 비용 가중치
%     .mocoTimeBound / .mocoDistBound: Moco bounds
%     .gaitMode      : 'modeSym' | 'modeAsym'

import org.opensim.modeling.*

%% ── cfg 파싱 ──────────────────────────────────────────────────────
result_name   = cfg.result_name;
ModelNameOsim = cfg.model;
iterNum       = cfg.iter;
fixedInterval = cfg.fixedInterval;
gaitMode      = cfg.gaitMode;

splineParams.trigger = cfg.trigger;
splineParams.rise    = cfg.rise;
splineParams.flat    = cfg.flat;
splineParams.fall    = cfg.fall;
splineParams.maxVal  = cfg.maxVal;

MocoOpts.mocoEffort    = cfg.mocoEffort;
MocoOpts.mocoFinalTime = cfg.mocoFinalTime;
MocoOpts.gaitMode      = gaitMode;
MocoOpts.mocoTimeBound = cfg.mocoTimeBound;
MocoOpts.mocoDistBound = cfg.mocoDistBound;

%% ── 입력 검증 ─────────────────────────────────────────────────────
if numel(fixedInterval) ~= 2 || fixedInterval(1) >= fixedInterval(2)
    error('fixedInterval 은 [t_start, t_end] 이어야 하며 t_start < t_end 조건 필요. (%.4g, %.4g)', ...
        fixedInterval(1), fixedInterval(2));
end
if ~ismember(gaitMode, {'modeSym', 'modeAsym'})
    error('gaitMode 는 ''modeSym'' 또는 ''modeAsym'' 이어야 합니다.');
end

t_start = fixedInterval(1);
t_end   = fixedInterval(2);

%% ── 경로 설정 ─────────────────────────────────────────────────────
[~, modelName] = fileparts(ModelNameOsim);

OutputFolder = fullfile(scriptDir, '..', 'results', result_name);
if ~exist(OutputFolder, 'dir'), mkdir(OutputFolder); end

if strcmpi(gaitMode, 'modeAsym')
    guessInitSto = fullfile(inputPath, 'guess_init_full.sto');
else
    guessInitSto = fullfile(inputPath, 'guess_init_half.sto');
end

%% ── 파라미터 출력 ─────────────────────────────────────────────────
fprintf('  model         : %s\n', ModelNameOsim);
fprintf('  iter          : %d\n', iterNum);
fprintf('  fixedInterval : [%.4g, %.4g] s\n', t_start, t_end);
fprintf('  trigger/rise/flat/fall : %.4g / %.4g / %.4g / %.4g\n', ...
    splineParams.trigger, splineParams.rise, splineParams.flat, splineParams.fall);
fprintf('  maxVal        : %.4g\n', splineParams.maxVal);
fprintf('  mocoEffort    : %.4g\n', MocoOpts.mocoEffort);
fprintf('  mocoFinalTime : %.4g\n', MocoOpts.mocoFinalTime);
fprintf('  gaitMode      : %s\n',   gaitMode);
fprintf('  mocoTimeBound : %s\n',   mat2str(MocoOpts.mocoTimeBound));
fprintf('  mocoDistBound : %s\n',   mat2str(MocoOpts.mocoDistBound));

%% ── Baseline analysis ─────────────────────────────────────────────
baselineAnalyDir = fullfile(OutputFolder, 'baseline', 'analy_result');
if ~exist(baselineAnalyDir, 'dir'), mkdir(baselineAnalyDir); end

fprintf('[%.1f s] Baseline analysis 시작...\n', toc_global());
WoC_moco_analysis(AnalySetupPath, ...
    'modelPath',         fullfile(modelBasePath, ModelNameOsim), ...
    'kinematicsStoPath', guessInitSto, ...
    'resultsDir',        baselineAnalyDir);
fprintf('[%.1f s] Baseline analysis 완료.\n', toc_global());

%% ── 메인 루프 ─────────────────────────────────────────────────────
for i = 1:iterNum
    fprintf('\n[%.1f s] ===== Iter %d / %d =====\n', toc_global(), i, iterNum);

    iterRootDir      = fullfile(OutputFolder, sprintf('result_%d', i));
    AnalyResultDir   = fullfile(iterRootDir, 'analy_result');
    mocoResultDir    = fullfile(iterRootDir, 'moco_result');
    controlResultDir = fullfile(iterRootDir, 'control_result');

    if ~exist(iterRootDir,      'dir'), mkdir(iterRootDir);      end
    if ~exist(AnalyResultDir,   'dir'), mkdir(AnalyResultDir);   end
    if ~exist(mocoResultDir,    'dir'), mkdir(mocoResultDir);    end
    if ~exist(controlResultDir, 'dir'), mkdir(controlResultDir); end

    % ── (1) control.sto 생성 ──────────────────────────────────────
    fprintf('[%.1f s] (1) control.sto 생성...\n', toc_global());
    stanceTime = linspace(t_start, t_end, 101)';
    tau_R      = WoC_moco_buildSplineControl(splineParams, 101);

    t_cover  = max(MocoOpts.mocoTimeBound(end) * 2, t_end * 1.5);
    fullTime = linspace(0, t_cover, 101)';

    writeOpts.dataColName = 'spline_fixed';
    WoC_moco_writeControl(controlResultDir, ...
        fullTime, stanceTime, tau_R, zeros(101,1), ones(101,1), writeOpts);

    controlRefStoPath = fullfile(controlResultDir, 'control.sto');

    % ── (2) guess 경로 결정 ───────────────────────────────────────
    if i == 1
        guessStoPath = guessInitSto;
    else
        prevMocoDir = fullfile(OutputFolder, sprintf('result_%d', i-1), 'moco_result');
        if strcmpi(gaitMode, 'modeSym')
            guessStoPath = fullfile(prevMocoDir, ...
                sprintf('moco_WoC_Solution_iter%02d_kinematics_half.sto', i-1));
        else
            guessStoPath = fullfile(prevMocoDir, ...
                sprintf('moco_WoC_Solution_iter%02d_kinematics.sto', i-1));
        end
    end

    % ── (3) Moco 최적화 ───────────────────────────────────────────
    baseOsimPath = fullfile(modelBasePath, ModelNameOsim);
    fprintf('[%.1f s] (3) moco_WoC_loop 시작 (xmlread/xmlwrite/initSystem/CasADi)...\n', toc_global());
    sol = moco_WoC_loop(controlRefStoPath, guessStoPath, i, AnalyResultDir, baseOsimPath, MocoOpts);
    fprintf('[%.1f s] (3) moco_WoC_loop 완료.\n', toc_global());

    % ── (4) 결과 저장 ─────────────────────────────────────────────
    resOpts.modelPath = fullfile(AnalyResultDir, sprintf('%s_%d.osim', modelName, i));
    resOpts.prefix    = sprintf('moco_WoC_Solution_iter%02d', i);
    resOpts.gaitMode  = gaitMode;
    fprintf('[%.1f s] (4) moco_WoC_getResult (metabolic 계산)...\n', toc_global());
    moco_WoC_getResult(sol, mocoResultDir, resOpts);
    fprintf('[%.1f s] (4) moco_WoC_getResult 완료.\n', toc_global());

    % ── (5) Analysis ──────────────────────────────────────────────
    currKinPath = fullfile(mocoResultDir, sprintf('moco_WoC_Solution_iter%02d_kinematics.sto', i));
    currOsmPath = fullfile(AnalyResultDir, sprintf('%s_%d.osim', modelName, i));
    fprintf('[%.1f s] (5) WoC_moco_analysis...\n', toc_global());
    WoC_moco_analysis(AnalySetupPath, ...
        'modelPath',         currOsmPath, ...
        'kinematicsStoPath', currKinPath, ...
        'resultsDir',        AnalyResultDir);
    fprintf('[%.1f s] (5) WoC_moco_analysis 완료.\n', toc_global());

    % ── (6) Inverse Dynamics ──────────────────────────────────────
    idXmlPath      = fullfile(inputPath, 'id_setup.xml');
    kinQStoPath    = fullfile(AnalyResultDir, '2D_gait_AFO_pc_Kinematics_q.sto');
    currGrfRelPath = sprintf('../moco_result/moco_WoC_Solution_iter%02d_GRF.sto', i);

    grfXmlDoc = xmlread(fullfile(inputPath, 'GRF_setup.xml'));
    grfXmlDoc.getElementsByTagName('datafile').item(0).setTextContent(currGrfRelPath);
    tempGrfXml = fullfile(AnalyResultDir, 'GRF_setup_id.xml');
    xmlwrite(tempGrfXml, grfXmlDoc);

    fprintf('[%.1f s] (6) ID with assist...\n', toc_global());
    idTool = InverseDynamicsTool(idXmlPath);
    idTool.setModelFileName(currOsmPath);
    idTool.setCoordinatesFileName(kinQStoPath);
    idTool.setExternalLoadsFileName(tempGrfXml);
    idTool.setResultsDir(AnalyResultDir);
    idTool.setOutputGenForceFileName('id_withAssist.sto');
    idTool.run();

    fprintf('[%.1f s] (6) ID without assist...\n', toc_global());
    idTool2 = InverseDynamicsTool(idXmlPath);
    idTool2.setModelFileName(baseOsimPath);
    idTool2.setCoordinatesFileName(kinQStoPath);
    idTool2.setExternalLoadsFileName(tempGrfXml);
    idTool2.setResultsDir(AnalyResultDir);
    idTool2.setOutputGenForceFileName('id_withoutAssist.sto');
    idTool2.run();

    fprintf('[%.1f s] Iter %d done.\n', toc_global(), i);
end

fprintf('[%.1f s] Job "%s" finished.\n', toc_global(), result_name);
end


%% ── 전역 타이머 헬퍼 ──────────────────────────────────────────────
function t = toc_global()
% 스크립트 시작부터의 경과 시간(초)을 반환.
% fixed_time_spline.m 에서 global_tic 을 설정해야 함.
persistent t0
if isempty(t0)
    t0 = tic;
end
t = toc(t0);
end
