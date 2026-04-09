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
%
% modeAsym 저장 파일:
%   {prefix}_kinematics.sto      (full stride = solve 결과 그대로)
%   {prefix}_GRF.sto             (full stride 기준)

    import org.opensim.modeling.*

    if nargin < 2
        error('Usage: moco_WoC_getResult(moco_WoC_Solution, outputDir, [opts])');
    end
    if nargin < 3
        opts = struct();
    end

    modelPath = getOpt(opts, 'modelPath', '2D_gait_AFO.osim');
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
    % 3. TODO: kinematics / GRF 기반 그래프 그리기
    %----------------------------------------------------------
end


%% ---- 옵션 읽기용 헬퍼 ----
function val = getOpt(s, field, defaultVal)
    if isfield(s, field) && ~isempty(s.(field))
        val = s.(field);
    else
        val = defaultVal;
    end
end
