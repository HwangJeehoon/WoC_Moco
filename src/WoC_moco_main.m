function WoC_moco_main(model, iter, optMode, result_name, opts, optResume)
% WoC_moco_main
%
%   호출 예시:
%     % 일반 실행
%     WoC_moco_main(model, iter, 'modeWoC', 'my_result')
%     WoC_moco_main(model, iter, 'modeWoC', 'my_result', opts)
%
%     % Resume 실행 (optResume.resume_name 이 있으면 자동으로 resume mode)
%     optResume.resume_name = 'my_result/result_300';
%     WoC_moco_main(model, iter, 'modeWoC', 'my_result_continued', opts, optResume)
%
%   optMode (string):
%     'modeWoC'    : WoC QP 기반 최적 보조 토크 (기본 동작)
%     'modeOff'    : 보조력 없음 (AFO = 0, QP 생략)
%
%   optMode (struct) — modeSpline 전용:
%     .type    = 'modeSpline'
%     .trigger : control 시작 시각 (정규화 도메인 [0, 1) 기준)
%     .rise    : 상승 구간 길이
%     .flat    : 최대값 유지 구간 길이
%     .fall    : 하강 구간 길이 (trigger+rise+flat+fall > 1 이면 wrap-around)
%     .maxVal  : 최대 출력값 (0 ~ 1)
%     조건: trigger < 1
%
%   opts (선택 struct):
%     .QP_effort    : QP 비용 계수 (default = 0.01)
%     .QP_smooth    : QP 스무딩 계수 (default = 0)
%     .cost         : Main cost 종류 'et'|'etw'|'etv' (default = 'et')
%     .mocoEffort   : Moco effort goal weight (default = 1)
%     .mocoFinalTime: Moco final time goal weight (default = 0.03)
%     .mocoPropulsionWeight : 양의 apGRF impulse 최대화 goal weight
%                             (default = 0; 0이면 goal 비활성)
%     .mocoPropulsionSmoothing : 양의 apGRF smooth-max 계수 [0.2, 1]
%                                (default = 0.2)
%     .mocoPropulsionReferenceDuration : impulse 정규화 기준 시간(s).
%                                비우면 gaitMode/time bound에서 자동 설정
%     .mocoObjectiveMode : Moco 목적함수 (QP의 .cost와 별개)
%         'weighted'        : effort + final time (+ propulsion) 가중합 (default)
%         'minSpeedStateFixed' : 평균속도 v_avg = (pelvis_tx(tf)-pelvis_tx(t0))/(tf-t0)
%                             만 최소화하고 E = mocoEnergyTarget (지표는 mocoEnergyMetric),
%                             P = mocoPropulsionTarget 을 endpoint equality
%                             constraint로 강제한다. mocoEffort / mocoFinalTime /
%                             mocoPropulsionWeight 는 쓰지 않는다(0이 아니면 error).
%                             C++ 플러그인 필요: plugins/osimFixedEffortPropulsion/
%                             build_osimFixedEffortPropulsion.m 으로 1회 빌드.
%         'maxSpeedStateFixed' : minSpeedStateFixed와 같은 제약에서 v_avg를 최대화한다
%                             (Moco cost = -mocoSpeedWeight * v_avg).
%                             아래의 'minSpeedStateFixed' 옵션은 이 모드에도 같이 쓰인다.
%     .mocoEnergyMetric : min/maxSpeedStateFixed에서 E0로 고정하는 에너지 지표
%         'effort' (default)  : weighted 모드 effort goal과 같은 정의의 weight 곱하기 전 값
%                               E = (1/d_COM) * int sum_c u_c^2 dt [-]
%                               (AFO_r/AFO_l 제외, 근육 excitation과 lumbarAct, weight 1)
%         'metabolicCoT'      : metabolic cost of transport [J/kg/m]
%                               CoT = int P_met dt / (mass * d_COM),
%                               P_met = Bhargava2004SmoothedMuscleMetabolics의
%                               total_metabolic_rate [W] (basal 포함, use_smoothing,
%                               모든 Muscle component 등록: moco_WoC_addMetabolics).
%                               modeSym은 반주기 에너지 / 반주기 COM 변위.
%     .mocoEnergyTarget : E0 ('minSpeedStateFixed' 필수, 단위는 mocoEnergyMetric)
%     .mocoEffortTarget : mocoEnergyTarget의 이전 이름 (effort 지표에서만 허용)
%     .mocoLumbarControlBound : lumbarAct control 범위 [lower, upper] [N*m]
%                         (없거나 비어 있으면 모델의 control 범위를 그대로 쓴다)
%     .mocoPropulsionTarget : J [N*s] ('minSpeedStateFixed' 필수). 한 보행주기 동안
%                         양발 gated propulsive impulse의 합 (meanPropPerVel의 Iplus)
%                         P_gate = int [F_r g(F_r) + F_l g(F_l)] dt,
%                         g(F) = 1/(1+exp(-(F-F0)/sigma)), F_foot = heel + front x-force.
%                         modeSym은 반주기 적분의 2배를 J와 맞춘다.
%                         mocoPropulsionMetric = 'meanForce'이면 J [N]는 평균 추진력
%                         P_gate / Tplus, Tplus = int [g(F_r) + g(F_l)] dt
%                         (meanPropPerVel의 MeanPropulsiveForce, 반주기와 한 주기가 같다).
%     .mocoPropulsionMetric : 'impulse' (default, J = P_gate) | 'meanForce' (J = 평균 추진력)
%     .mocoPropulsionConstraintMode : J 제약 형태 (min/maxSpeedStateFixed)
%         'equality' (default) : J_actual = mocoPropulsionTarget (scalar)
%         'range'              : lower <= J_actual <= upper, mocoPropulsionBounds = [lower, upper]
%                                (닫힌 구간, 0 <= lower < upper, 단위는 mocoPropulsionTarget과 같음:
%                                 impulse는 한 보행주기 양발 합 N*s, meanForce는 N). range에서는
%                                 mocoPropulsionTarget을 지정하지 않는다 (구간 중심을 target으로 쓰지 않음).
%     .mocoPropulsionBounds : range 전용 [lower, upper]
%     .mocoPropulsionGateSigma     : 'minSpeedStateFixed' gate 폭 sigma [N] (default = 1)
%     .mocoPropulsionGateThreshold : 'minSpeedStateFixed' gate 중심 F0 [N] (default = 5)
%                         (mocoPropulsionSmoothing은 weighted 모드 propulsion 비용항의
%                          softplus 계수 [1/N]로, minSpeedStateFixed에서는 쓰지 않는다)
%     .mocoSpeedWeight : 'minSpeedStateFixed' 평균속도 cost의 weight 크기 (양수, default = 1;
%                        최소화/최대화 방향은 mocoObjectiveMode가 정한다)
%     .gaitMode      : 'modeSym' | 'modeAsym' (default = 'modeSym')
%     .mocoTimeBound : Moco 시간 상한 [lb, ub] 또는 scalar (default = [0.4, 0.8])
%     .mocoDistBound : pelvis_tx 최종 위치 bound [lb, ub] 또는 scalar (default = [0.4, 1.0])
%     .mocoMeshIntervals : CasADi mesh interval 수 (default = 300)
%     .mocoConvergenceTolerance : IPOPT convergence tolerance (default = 1e-3)
%     .mocoConstraintTolerance  : IPOPT constraint tolerance (default = 1e-3)
%     .mocoMaxIterations        : IPOPT 최대 반복 수 (default = 1000)
%     .guessInitSto  : Initial guess로 사용할 sto 위치
%     .resultsDir    : 결과 루트 폴더 (default = <repo>/results)
%
%   저장 구조:
%   (results = opts.resultsDir; resume_name도 이 폴더 기준)
%     results/<result_name>/baseline/analy_result/   ← 초기 guess kinematics 해석 결과
%     results/<result_name>/result_i/analy_result/   ← iter i moco 결과 해석
%     results/<result_name>/result_i/moco_result/    ← iter i 궤적/GRF/metabolic,
%                                                      사용한 parameter(*_parameters.csv)
%     results/<result_name>/result_i/control_result/ ← iter i 보조 제어력
%   같은 result_name으로 다시 실행하면 기존 폴더의 파일을 덮어쓴다.

close all;
import org.opensim.modeling.*

%% --------------------------------------------------
%  optMode 파싱 (string / struct 양쪽 허용)
% ---------------------------------------------------
if nargin < 3 || isempty(optMode)
    optMode = 'modeWoC';
end

if ischar(optMode) || isstring(optMode)
    modeType   = char(optMode);
    modeParams = struct();
elseif isstruct(optMode)
    if ~isfield(optMode, 'type')
        error('optMode가 struct인 경우 type 필드가 필요합니다.');
    end
    modeType   = optMode.type;
    modeParams = optMode;
else
    error('optMode는 string 또는 struct 이어야 합니다.');
end

validModes = {'modeWoC', 'modeOff', 'modeSpline', 'modeTorqAmp'};
if ~ismember(modeType, validModes)
    error('Unknown optMode: ''%s''. Valid options: modeWoC, modeOff, modeSpline, modeTorqAmp.', modeType);
end

% 모델별 설정 (경로, 접촉력, 분석 setup, 지원 mode). 2D 계열은 기존 동작 그대로다.
modelProfile = moco_WoC_modelProfile(model);
if ~ismember(modeType, modelProfile.supportedOptModes)
    error('WoC_moco_main: %s profile(%s)은 optMode %s 를 지원하지 않습니다 (지원: %s).', ...
        modelProfile.name, model, modeType, strjoin(modelProfile.supportedOptModes, ', '));
end

% modeSpline 필수 필드 검증 (루프 진입 전에 미리 확인)
if strcmpi(modeType, 'modeSpline')
    splineFields = {'trigger', 'rise', 'flat', 'fall', 'maxVal'};
    for sf = splineFields
        if ~isfield(modeParams, sf{1})
            error('modeSpline: optMode.%s 가 필요합니다.', sf{1});
        end
    end
end

% modeTorqAmp 필수 필드 검증
if strcmpi(modeType, 'modeTorqAmp')
    if ~isfield(modeParams, 'maxVal')
        error('modeTorqAmp: optMode.maxVal 가 필요합니다.');
    end
    if modeParams.maxVal < 0 || modeParams.maxVal > 1
        error('modeTorqAmp: maxVal 은 [0, 1] 범위여야 합니다 (%.4f).', modeParams.maxVal);
    end
    if isfield(modeParams, 'cutoffHz') && modeParams.cutoffHz <= 0
        error('modeTorqAmp: cutoffHz 는 양수여야 합니다 (%.4f). 필터 비활성화는 cutoffHz=0 대신 필드를 생략하세요.', modeParams.cutoffHz);
    end
end

%% --------------------------------------------------
%  result_name / opts / optResume 파싱
% ---------------------------------------------------
if nargin < 4 || isempty(result_name)
    error('result_name 을 지정해야 합니다.');
end

if nargin < 5 || isempty(opts)
    opts = struct();
end

if nargin < 6 || isempty(optResume)
    optResume = struct();
end

% ID / Date (run_queue 에서 전달되는 메타 정보, 선택적)
runID   = getOpt(opts, 'ID',   '');
runDate = getOpt(opts, 'Date', '');

% opts 기본값
% QP_effort / QP_smooth / cost 는 modeWoC 에서만 사용되므로,
% 다른 mode에서는 미지정이어도 경고를 띄우지 않는다.
isWoC = strcmpi(modeType, 'modeWoC');

if ~isfield(opts, 'QP_effort') || isempty(opts.QP_effort)
    QP_effort = 0.01;
    if isWoC, warning('WoC_moco_main: opts.QP_effort 가 지정되지 않아 default(0.01) 를 사용합니다.'); end
else
    QP_effort = opts.QP_effort;
end

if ~isfield(opts, 'QP_smooth') || isempty(opts.QP_smooth)
    QP_smooth = 0;
    if isWoC, warning('WoC_moco_main: opts.QP_smooth 가 지정되지 않아 default(0) 를 사용합니다.'); end
else
    QP_smooth = opts.QP_smooth;
end

if ~isfield(opts, 'cost') || isempty(opts.cost)
    cost = 'et';
    if isWoC, warning('WoC_moco_main: opts.cost 가 지정되지 않아 default(et) 를 사용합니다.'); end
else
    cost = opts.cost;
end

mocoObjectiveMode = getOpt(opts, 'mocoObjectiveMode', 'weighted');
if ~ismember(mocoObjectiveMode, {'weighted', 'minSpeedStateFixed', 'maxSpeedStateFixed'})
    error(['opts.mocoObjectiveMode 는 ''weighted'', ''minSpeedStateFixed'', ', ...
           '''maxSpeedStateFixed'' 중 하나여야 합니다.']);
end
% minSpeedStateFixed와 maxSpeedStateFixed는 평균속도 cost의 부호만 다르다.
isStateFixed = ismember(mocoObjectiveMode, {'minSpeedStateFixed', 'maxSpeedStateFixed'});
if isStateFixed
    % 목적함수는 평균속도 하나뿐이다. 기존 비용항 weight를 조용히 무시하지 않는다.
    for f = {'mocoEffort', 'mocoFinalTime', 'mocoPropulsionWeight'}
        if getOpt(opts, f{1}, 0) ~= 0
            error(['opts.mocoObjectiveMode = ''%s'' 에서는 opts.%s 를 ', ...
                   '지정하지 마세요 (목적함수는 평균속도뿐입니다).'], mocoObjectiveMode, f{1});
        end
    end
    opts.mocoEffort = 0;
    opts.mocoFinalTime = 0;
end

% minSpeedStateFixed/maxSpeedStateFixed의 equality target. 실제 값은 사용자가 지정해야 하며 기본값은 없다.
% E0가 고정하는 에너지 지표: effort (이전 실행과 같은 기본값) 또는 metabolic CoT [J/kg/m].
mocoEnergyMetric = getOpt(opts, 'mocoEnergyMetric', 'effort');
if ~ismember(mocoEnergyMetric, {'effort', 'metabolicCoT'})
    error('opts.mocoEnergyMetric 은 ''effort'' 또는 ''metabolicCoT'' 여야 합니다.');
end
mocoEnergyTarget = getOpt(opts, 'mocoEnergyTarget', []);
mocoEffortTarget = getOpt(opts, 'mocoEffortTarget', []);
if strcmp(mocoEnergyMetric, 'effort')
    % 이전 호출의 mocoEffortTarget을 그대로 받는다.
    if isempty(mocoEnergyTarget)
        mocoEnergyTarget = mocoEffortTarget;
    elseif ~isempty(mocoEffortTarget) && ~isequal(mocoEffortTarget, mocoEnergyTarget)
        error('opts.mocoEnergyTarget 과 opts.mocoEffortTarget 이 다릅니다.');
    end
    mocoEffortTarget = mocoEnergyTarget;
    energyUnit = '-';
    energyDefinition = ['effort: ' modelProfile.effortDefinition];
else
    % effort 목표값을 CoT 목표값으로 쓰지 않는다.
    if ~isempty(mocoEffortTarget)
        error(['opts.mocoEffortTarget 은 effort 지표의 목표값입니다. metabolicCoT 목표값은 ', ...
               'opts.mocoEnergyTarget [J/kg/m] 으로 지정하세요.']);
    end
    energyUnit = 'J/kg/m';
    energyDefinition = ['metabolicCoT: int P_met dt / (mass * d_COM), P_met = /metabolics|', ...
        'total_metabolic_rate [W] (basal included), mass = model total mass, d_COM = |r_COM(tf) - ', ...
        'r_COM(t0)| of the solved phase (modeSym: half stride energy / half stride displacement)'];
end
mocoPropulsionTarget = getOpt(opts, 'mocoPropulsionTarget', []);
% J가 고정하는 propulsion 지표: gated impulse P_gate [N*s] 또는 평균 추진력 [N].
mocoPropulsionMetric = getOpt(opts, 'mocoPropulsionMetric', 'impulse');
if ~ismember(mocoPropulsionMetric, {'impulse', 'meanForce'})
    error('opts.mocoPropulsionMetric 은 ''impulse'' 또는 ''meanForce'' 여야 합니다.');
end
propulsionUnit = 'N*s';
if strcmp(mocoPropulsionMetric, 'meanForce'), propulsionUnit = 'N'; end
% J 제약 모드: 'equality' (default, J_actual = mocoPropulsionTarget) |
% 'range' (mocoPropulsionBounds(1) <= J_actual <= mocoPropulsionBounds(2), 닫힌 구간).
% range에서는 scalar target을 쓰지 않으며 구간 중심을 target으로 고정하지 않는다.
mocoPropulsionConstraintMode = char(getOpt(opts, 'mocoPropulsionConstraintMode', 'equality'));
mocoPropulsionBounds = getOpt(opts, 'mocoPropulsionBounds', []);
if ~ismember(mocoPropulsionConstraintMode, {'equality', 'range'})
    error('opts.mocoPropulsionConstraintMode 는 ''equality'' 또는 ''range'' 여야 합니다 (입력: %s).', ...
        mocoPropulsionConstraintMode);
end
if isStateFixed
    if ~isscalar(mocoEnergyTarget) || ~isnumeric(mocoEnergyTarget) || ...
            ~isfinite(mocoEnergyTarget) || mocoEnergyTarget <= 0
        error('opts.mocoEnergyTarget (E0, %s [%s]) 가 입력되지 않았거나 양의 유한 scalar가 아닙니다.', ...
            mocoEnergyMetric, energyUnit);
    end
    if strcmp(mocoPropulsionConstraintMode, 'equality')
        if ~isempty(mocoPropulsionBounds)
            error(['opts.mocoPropulsionConstraintMode = ''equality'' 에서는 opts.mocoPropulsionBounds 를 ', ...
                   '지정하지 마세요 (equality: mocoPropulsionTarget, range: mocoPropulsionBounds).']);
        end
        if ~isscalar(mocoPropulsionTarget) || ~isnumeric(mocoPropulsionTarget) || ...
                ~isfinite(mocoPropulsionTarget) || mocoPropulsionTarget <= 0
            error('opts.mocoPropulsionTarget (J, %s) 가 입력되지 않았거나 양의 유한 scalar가 아닙니다.', ...
                propulsionUnit);
        end
        mocoPropulsionBounds = [mocoPropulsionTarget, mocoPropulsionTarget];   % 공통 판정용 정규화
    else
        if ~isempty(mocoPropulsionTarget)
            error(['opts.mocoPropulsionConstraintMode = ''range'' 에서는 opts.mocoPropulsionTarget 을 ', ...
                   '지정하지 마세요 (scalar target 없이 mocoPropulsionBounds = [lower, upper]만 씁니다).']);
        end
        if ~isnumeric(mocoPropulsionBounds) || ~isreal(mocoPropulsionBounds) || ...
                numel(mocoPropulsionBounds) ~= 2 || ~isvector(mocoPropulsionBounds) || ...
                any(~isfinite(mocoPropulsionBounds)) || mocoPropulsionBounds(1) < 0 || ...
                mocoPropulsionBounds(1) >= mocoPropulsionBounds(2)
            error(['opts.mocoPropulsionBounds (%s) 는 유한한 실수 [lower, upper], 0 <= lower < upper ', ...
                   '여야 합니다 (입력: %s).'], propulsionUnit, mat2str(mocoPropulsionBounds));
        end
        mocoPropulsionBounds = double(mocoPropulsionBounds(:)');
    end
elseif ~strcmp(mocoPropulsionConstraintMode, 'equality') || ~isempty(mocoPropulsionBounds)
    error(['opts.mocoPropulsionConstraintMode = ''range'' / opts.mocoPropulsionBounds 는 ', ...
           'min/maxSpeedStateFixed 모드에서만 씁니다.']);
elseif strcmp(mocoPropulsionMetric, 'meanForce')
    error('opts.mocoPropulsionMetric = ''meanForce'' 는 min/maxSpeedStateFixed 모드에서만 씁니다.');
elseif strcmp(mocoEnergyMetric, 'metabolicCoT')
    error('opts.mocoEnergyMetric = ''metabolicCoT'' 는 min/maxSpeedStateFixed 모드에서만 씁니다.');
end

% minSpeedStateFixed propulsion gate (meanPropPerVel MeanPropulsiveForce와 같은 기본값).
mocoPropulsionGateSigma = getOpt(opts, 'mocoPropulsionGateSigma', 1.0);
if ~isscalar(mocoPropulsionGateSigma) || ~isnumeric(mocoPropulsionGateSigma) || ...
        ~isfinite(mocoPropulsionGateSigma) || mocoPropulsionGateSigma <= 0
    error('opts.mocoPropulsionGateSigma 는 양의 유한 scalar여야 합니다 (N).');
end
mocoPropulsionGateThreshold = getOpt(opts, 'mocoPropulsionGateThreshold', 5.0);
if ~isscalar(mocoPropulsionGateThreshold) || ~isnumeric(mocoPropulsionGateThreshold) || ...
        ~isfinite(mocoPropulsionGateThreshold)
    error('opts.mocoPropulsionGateThreshold 는 유한 scalar여야 합니다 (N).');
end
% 평균속도 cost weight 크기. 방향(최소화/최대화)은 mocoObjectiveMode가 정하므로 양수만 허용한다.
mocoSpeedWeight = getOpt(opts, 'mocoSpeedWeight', 1.0);
if ~isscalar(mocoSpeedWeight) || ~isnumeric(mocoSpeedWeight) || ...
        ~isfinite(mocoSpeedWeight) || mocoSpeedWeight <= 0
    error('opts.mocoSpeedWeight 는 양의 유한 scalar여야 합니다.');
end
mocoLumbarControlBound = getOpt(opts, 'mocoLumbarControlBound', []);
if ~isempty(mocoLumbarControlBound) && ~strcmp(modelProfile.name, '2D')
    error(['opts.mocoLumbarControlBound 는 2D 모델의 /lumbarAct 전용입니다. %s profile의 ', ...
           '토크 구동기 bound는 moco_WoC_modelProfile의 torqueLimits로 정합니다.'], modelProfile.name);
end
if ~isempty(mocoLumbarControlBound) && (~isnumeric(mocoLumbarControlBound) || ...
        numel(mocoLumbarControlBound) ~= 2 || any(~isfinite(mocoLumbarControlBound)) || ...
        mocoLumbarControlBound(1) >= mocoLumbarControlBound(2))
    error('opts.mocoLumbarControlBound 는 비어 있거나 [lower, upper] (lower < upper, N*m) 여야 합니다.');
end

if ~isfield(opts, 'mocoEffort') || isempty(opts.mocoEffort)
    mocoEffort = 1;
    warning('WoC_moco_main: opts.mocoEffort 가 지정되지 않아 default(1) 를 사용합니다.');
else
    mocoEffort = opts.mocoEffort;
end

if ~isfield(opts, 'mocoFinalTime') || isempty(opts.mocoFinalTime)
    mocoFinalTime = 0.03;
    warning('WoC_moco_main: opts.mocoFinalTime 가 지정되지 않아 default(0.03) 를 사용합니다.');
else
    mocoFinalTime = opts.mocoFinalTime;
end

% Propulsion goal: 기본값 0은 기존 objective를 보존한다.
if ~isfield(opts, 'mocoPropulsionWeight') || isempty(opts.mocoPropulsionWeight)
    mocoPropulsionWeight = 0;
else
    mocoPropulsionWeight = opts.mocoPropulsionWeight;
end
if ~isscalar(mocoPropulsionWeight) || ~isnumeric(mocoPropulsionWeight) || ...
        ~isfinite(mocoPropulsionWeight) || mocoPropulsionWeight < 0
    error('opts.mocoPropulsionWeight 는 0 이상의 유한 scalar여야 합니다.');
end

if ~isfield(opts, 'mocoPropulsionSmoothing') || isempty(opts.mocoPropulsionSmoothing)
    mocoPropulsionSmoothing = 0.2;
else
    mocoPropulsionSmoothing = opts.mocoPropulsionSmoothing;
end
if ~isscalar(mocoPropulsionSmoothing) || ~isnumeric(mocoPropulsionSmoothing) || ...
        ~isfinite(mocoPropulsionSmoothing) || ...
        mocoPropulsionSmoothing < 0.2 || mocoPropulsionSmoothing > 1.0
    error('opts.mocoPropulsionSmoothing 은 [0.2, 1.0] 범위의 유한 scalar여야 합니다.');
end

if ~isfield(opts, 'mocoPropulsionReferenceDuration') || ...
        isempty(opts.mocoPropulsionReferenceDuration)
    mocoPropulsionReferenceDuration = [];
else
    mocoPropulsionReferenceDuration = opts.mocoPropulsionReferenceDuration;
    if ~isscalar(mocoPropulsionReferenceDuration) || ...
            ~isnumeric(mocoPropulsionReferenceDuration) || ...
            ~isfinite(mocoPropulsionReferenceDuration) || ...
            mocoPropulsionReferenceDuration <= 0
        error('opts.mocoPropulsionReferenceDuration 은 양의 유한 scalar여야 합니다.');
    end
end

if ~isfield(opts, 'gaitMode') || isempty(opts.gaitMode)
    gaitMode = 'modeSym';
    warning('WoC_moco_main: opts.gaitMode 가 지정되지 않아 default(''modeSym'') 를 사용합니다.');
else
    gaitMode = opts.gaitMode;
end

if ~ismember(gaitMode, {'modeSym', 'modeAsym'})
    error('opts.gaitMode 는 ''modeSym'' 또는 ''modeAsym'' 이어야 합니다.');
end
if ~ismember(gaitMode, modelProfile.supportedGaitModes)
    error('WoC_moco_main: %s profile(%s)은 gaitMode %s 를 지원하지 않습니다 (지원: %s).', ...
        modelProfile.name, model, gaitMode, strjoin(modelProfile.supportedGaitModes, ', '));
end

if ~isfield(opts, 'mocoTimeBound') || isempty(opts.mocoTimeBound)
    mocoTimeBound = [0.4, 0.8];
    warning('WoC_moco_main: opts.mocoTimeBound 가 지정되지 않아 default([0.4, 0.8]) 를 사용합니다.');
else
    mocoTimeBound = opts.mocoTimeBound;
end

if ~isfield(opts, 'mocoDistBound') || isempty(opts.mocoDistBound)
    mocoDistBound = [0.4 1.0];
    warning('WoC_moco_main: opts.mocoDistBound 가 지정되지 않아 default([0.4 1.0]) 를 사용합니다.');
else
    mocoDistBound = opts.mocoDistBound;
end

% E는 질량중심 변위로 나누므로 전진 변위가 양수여야 한다.
if isStateFixed && min(mocoDistBound) <= 0
    error('opts.mocoObjectiveMode = ''%s'' 이면 mocoDistBound 하한이 양수여야 합니다.', ...
        mocoObjectiveMode);
end

% Solver 설정: 기본값은 기존 고정값(mesh 300, tolerance 1e-3, 최대 반복 1000)과 같다.
mocoMeshIntervals = getOpt(opts, 'mocoMeshIntervals', 300);
if ~isscalar(mocoMeshIntervals) || ~isnumeric(mocoMeshIntervals) || ...
        ~isfinite(mocoMeshIntervals) || mocoMeshIntervals < 1 || ...
        mocoMeshIntervals ~= round(mocoMeshIntervals)
    error('opts.mocoMeshIntervals 는 1 이상의 정수 scalar여야 합니다.');
end
mocoConvergenceTolerance = getOpt(opts, 'mocoConvergenceTolerance', 1e-3);
mocoConstraintTolerance  = getOpt(opts, 'mocoConstraintTolerance', 1e-3);
if ~isscalar(mocoConvergenceTolerance) || ~isnumeric(mocoConvergenceTolerance) || ...
        ~isfinite(mocoConvergenceTolerance) || mocoConvergenceTolerance <= 0 || ...
        ~isscalar(mocoConstraintTolerance) || ~isnumeric(mocoConstraintTolerance) || ...
        ~isfinite(mocoConstraintTolerance) || mocoConstraintTolerance <= 0
    error('opts.mocoConvergenceTolerance / mocoConstraintTolerance 는 양의 유한 scalar여야 합니다.');
end
mocoMaxIterations = getOpt(opts, 'mocoMaxIterations', 1000);
if ~isscalar(mocoMaxIterations) || ~isnumeric(mocoMaxIterations) || ...
        ~isfinite(mocoMaxIterations) || mocoMaxIterations < 1 || ...
        mocoMaxIterations ~= round(mocoMaxIterations)
    error('opts.mocoMaxIterations 는 1 이상의 정수 scalar여야 합니다.');
end
% 설정 점검용: true면 Moco 문제/guess를 만들고 검사한 뒤 solve 대신 이름 기준으로
% 재구성한 initial guess를 결과 파일로 저장하고 후처리(GRF, 분석, ID)를 그대로 돌린다.
% parameters.csv의 mocoSkipSolve = 1로 남고 summary는 최적화 결과로 판정하지 않는다.
mocoSkipSolve = logical(getOpt(opts, 'mocoSkipSolve', false));

%% --------------------------------------------------
%  0. baseFolder = 이 파일이 있는 폴더
% ---------------------------------------------------
if isempty(mfilename)  % 스크립트처럼 실행하는 경우
    thisFile = matlab.desktop.editor.getActiveFilename;
else
    thisFile = mfilename("fullpath");
end
baseFolder = fileparts(thisFile);
resultsRoot = getOpt(opts, 'resultsDir', fullfile(baseFolder, '..', 'results'));

% moco_WoC_loop과 같은 규칙: modeAsym은 한 걸음 bound(x2)를 쓰고, 비어 있는
% propulsion reference duration은 mean(time bound)(modeAsym x2)로 정한다.
boundScale = 1 + strcmpi(gaitMode, 'modeAsym');
effPropulsionReferenceDuration = mocoPropulsionReferenceDuration;
if isempty(effPropulsionReferenceDuration)
    effPropulsionReferenceDuration = mean(mocoTimeBound) * boundScale;
end
if mocoPropulsionWeight > 0
    propulsionBodyWeight = modelBodyWeight(fullfile(baseFolder, '..', 'models', model));
end

%% --------------------------------------------------
%  파라미터 출력
% ---------------------------------------------------
fprintf('=== WoC_moco_main Parameters ===\n');
if ~isempty(runID),   fprintf('  ID         : %s\n', runID);   end
if ~isempty(runDate), fprintf('  Date       : %s\n', runDate); end
fprintf('  model      : %s\n', model);
fprintf('  iter       : %d\n', iter);
fprintf('  optMode    : %s\n', modeType);
fprintf('  result_name: %s\n', result_name);
fprintf('  --- opts ---\n');
if isWoC
    fprintf('  QP_effort    : %.4g\n', QP_effort);
    fprintf('  QP_smooth    : %.4g\n', QP_smooth);
    fprintf('  cost         : %s\n',   cost);
end
fprintf('  mocoEffort   : %.4g\n', mocoEffort);
fprintf('  mocoFinalTime: %.4g\n', mocoFinalTime);
fprintf('  mocoPropulsionWeight: %.4g\n', mocoPropulsionWeight);
if mocoPropulsionWeight > 0
    fprintf('  mocoPropulsionSmoothing: %.4g\n', mocoPropulsionSmoothing);
    if isempty(mocoPropulsionReferenceDuration)
        fprintf('  mocoPropulsionReferenceDuration: %.4g s (auto)\n', ...
            effPropulsionReferenceDuration);
    else
        fprintf('  mocoPropulsionReferenceDuration: %.4g s\n', ...
            effPropulsionReferenceDuration);
    end
    fprintf('  propulsion body weight: %.4g N\n', propulsionBodyWeight);
end
fprintf('  mocoObjectiveMode: %s\n', mocoObjectiveMode);
if isStateFixed
    speedVerb = 'minimize';
    if strcmp(mocoObjectiveMode, 'maxSpeedStateFixed'), speedVerb = 'maximize'; end
    fprintf('  objective: %s %.4g * v_avg, v_avg = (pelvis_tx(tf) - pelvis_tx(t0)) / (tf - t0)\n', ...
        speedVerb, mocoSpeedWeight);
    if strcmp(mocoEnergyMetric, 'metabolicCoT')
        fprintf('  mocoEnergyTarget E0: %.6g J/kg/m (metabolic CoT, Bhargava2004 incl. basal)\n', ...
            mocoEnergyTarget);
    else
        fprintf('  mocoEnergyTarget E0: %.6g (effort metric before weight)\n', mocoEnergyTarget);
    end
    if ~isempty(mocoLumbarControlBound)
        fprintf('  lumbarAct control bound: %s N*m\n', mat2str(mocoLumbarControlBound));
    end
    if strcmp(mocoPropulsionMetric, 'meanForce')
        jText = 'N (mean propulsive force, gated Iplus / Tplus)';
    else
        jText = 'N*s (full stride, both feet, gated Iplus)';
    end
    if strcmp(mocoPropulsionConstraintMode, 'range')
        fprintf('  propulsion constraint: range %.6g <= J <= %.6g %s (no scalar target)\n', ...
            mocoPropulsionBounds(1), mocoPropulsionBounds(2), jText);
    else
        fprintf('  propulsion constraint: equality, mocoPropulsionTarget J: %.6g %s\n', ...
            mocoPropulsionTarget, jText);
    end
    fprintf('  propulsion gate: sigma %.4g N, F0 %.4g N\n', ...
        mocoPropulsionGateSigma, mocoPropulsionGateThreshold);
end
fprintf('  gaitMode     : %s\n',   gaitMode);
fprintf('  model profile: %s (forward coordinate %s, %d+%d contact forces)\n', modelProfile.name, ...
    modelProfile.pelvisCoordinatePath, numel(modelProfile.contactPaths.r), numel(modelProfile.contactPaths.l));
fprintf('  mocoTimeBound: %s (input, half stride) -> applied final time %s s\n', ...
    mat2str(mocoTimeBound), mat2str(mocoTimeBound * boundScale));
fprintf('  mocoDistBound: %s (input, half stride) -> applied final pelvis_tx %s m\n', ...
    mat2str(mocoDistBound), mat2str(mocoDistBound * boundScale));
fprintf('  mocoMeshIntervals: %d, convergence tol: %.3g, constraint tol: %.3g, max iterations: %d\n', ...
    mocoMeshIntervals, mocoConvergenceTolerance, mocoConstraintTolerance, mocoMaxIterations);
fprintf('================================\n');

% resume_name 이 있으면 resume mode
resume_mode = isfield(optResume, 'resume_name') && ~isempty(optResume.resume_name);
if resume_mode
    resume_name = optResume.resume_name;
else
    resume_name = '';
end

%% --------------------------------------------------
%  1. 파라미터 설정 + Output 폴더명 지정
% ---------------------------------------------------

% 사용할 model명 지정
ModelNameOsim = model;
[~, modelName, ~] = fileparts(model);

% QP Parameter (modeWoC에서만 사용)
iterNum          = iter;
coeffi_cost      = QP_effort;
coeffi_smoothing = QP_smooth;

% Main Cost (modeWoC에서만 사용)
eta_tau   = false;
eta_tau_w = false;
eta_tau_v = false;
switch lower(cost)
    case 'et'
        eta_tau = true;
    case 'etw'
        eta_tau_w = true;
    case 'etv'
        eta_tau_v = true;
    otherwise
        if strcmpi(modeType, 'modeWoC')
            error("Unknown cost type: %s. Use 'et', 'etw', or 'etv'.", cost);
        end
end

% Moco Parameter
MocoOpts.mocoEffort       = mocoEffort;
MocoOpts.mocoFinalTime    = mocoFinalTime;
MocoOpts.mocoPropulsionWeight = mocoPropulsionWeight;
MocoOpts.mocoPropulsionSmoothing = mocoPropulsionSmoothing;
MocoOpts.mocoPropulsionReferenceDuration = mocoPropulsionReferenceDuration;
MocoOpts.mocoObjectiveMode = mocoObjectiveMode;
MocoOpts.mocoEffortTarget = mocoEffortTarget;   % effort 지표일 때만 (이전 결과와 같은 열)
MocoOpts.mocoEnergyMetric = mocoEnergyMetric;
MocoOpts.mocoEnergyTarget = mocoEnergyTarget;
MocoOpts.mocoLumbarControlBound = mocoLumbarControlBound;
MocoOpts.mocoPropulsionTarget = mocoPropulsionTarget;   % equality 전용 (range는 [])
MocoOpts.mocoPropulsionConstraintMode = mocoPropulsionConstraintMode;
MocoOpts.mocoPropulsionBounds = mocoPropulsionBounds;   % stateFixed: [lower, upper] (equality는 [J, J])
MocoOpts.mocoPropulsionMetric = mocoPropulsionMetric;
MocoOpts.mocoPropulsionGateSigma = mocoPropulsionGateSigma;
MocoOpts.mocoPropulsionGateThreshold = mocoPropulsionGateThreshold;
MocoOpts.mocoSpeedWeight = mocoSpeedWeight;
MocoOpts.gaitMode         = gaitMode;
MocoOpts.mocoTimeBound    = mocoTimeBound;
MocoOpts.mocoDistBound    = mocoDistBound;
MocoOpts.mocoMeshIntervals = mocoMeshIntervals;
MocoOpts.mocoConvergenceTolerance = mocoConvergenceTolerance;
MocoOpts.mocoConstraintTolerance  = mocoConstraintTolerance;
MocoOpts.mocoMaxIterations = mocoMaxIterations;
if mocoSkipSolve
    MocoOpts.mocoSkipSolve = true;   % 2D 기본 실행의 parameter 목록은 바꾸지 않는다
end

% result_i/moco_result/*_parameters.csv 로 기록할 parameter
% (iteration별 항목은 메인 루프에서 추가한다).
runParams = {'result_name', result_name; 'run_ID', runID; 'run_Date', runDate; ...
             'model', ModelNameOsim; 'optMode', modeType; ...
             'resume_name', resume_name; ...
             'opensim_version', char(opensimCommon.GetVersion())};
for f = setdiff(fieldnames(modeParams)', {'type'}, 'stable')
    runParams(end+1, :) = {['optMode_' f{1}], modeParams.(f{1})}; %#ok<AGROW>
end
if isWoC
    runParams = [runParams; {'QP_effort', QP_effort; 'QP_smooth', QP_smooth; ...
                             'cost', cost}];
end
for f = fieldnames(MocoOpts)'
    runParams(end+1, :) = {f{1}, MocoOpts.(f{1})}; %#ok<AGROW>
end
% boundScale / effPropulsionReferenceDuration 은 파라미터 출력 전에 계산함.
% mocoTimeBound / mocoDistBound (위 MocoOpts 항목)는 반주기 기준 입력값이고,
% effective_* 는 moco_WoC_loop가 실제로 적용하는 값이다 (modeAsym x2, 한 번만).
runParams = [runParams; ...
    {'model_profile', modelProfile.name; ...
     'input_time_bound_half_stride_s', mocoTimeBound; ...
     'input_pelvis_tx_bound_half_stride_m', mocoDistBound; ...
     'effective_final_time_bound_s', mocoTimeBound * boundScale; ...
     'effective_final_pelvis_tx_bound_m', mocoDistBound * boundScale; ...
     'forward_coordinate', modelProfile.pelvisCoordinatePath; ...
     'contact_forces_right', strjoin(modelProfile.contactPaths.r, ' '); ...
     'contact_forces_left', strjoin(modelProfile.contactPaths.l, ' '); ...
     'results_root', resultsRoot; ...
     'effort_definition', modelProfile.effortDefinition}];
if isStateFixed
    % E0가 고정한 에너지 지표. 이 항목이 없는 이전 결과는 effort 지표로 읽는다.
    runParams = [runParams; {'energy_metric', mocoEnergyMetric; 'energy_unit', energyUnit; ...
                             'energy_definition', energyDefinition}];
    if strcmp(mocoEnergyMetric, 'metabolicCoT')
        [~, metabolicsInfo] = moco_WoC_addMetabolics([]);
        runParams = [runParams; {'metabolics_settings', metabolicsInfo.settings; ...
            'cot_constraint', ['MocoFinalOutputGoal metabolic_cot_constraint: ', ...
            '/effortPropulsion|metabolic_energy, divide_by_mass, divide_by_displacement, endpoint_constraint']}];
    end
    % 결과를 다시 요약할 때 실행 당시 propulsion 정의를 구분하기 위한 메타데이터.
    if strcmp(mocoPropulsionMetric, 'meanForce')
        runParams(end+1, :) = {'propulsion_definition', ...
            ['gate_mean_force: m = Iplus / Tplus, Iplus = int [F_r g(F_r) + F_l g(F_l)] dt, ', ...
             'Tplus = int [g(F_r) + g(F_l)] dt, g = 1/(1+exp(-(F-F0)/sigma))']};
    else
        runParams(end+1, :) = {'propulsion_definition', ...
            'gate_Iplus: P = impulse_scale * int [F_r g(F_r) + F_l g(F_l)] dt, g = 1/(1+exp(-(F-F0)/sigma))'};
    end
    % J 제약 형태. 이 항목이 없는 이전 결과는 equality (lower = upper = mocoPropulsionTarget)로 읽는다.
    runParams = [runParams; {'propulsion_constraint_mode', mocoPropulsionConstraintMode; ...
        'propulsion_lower', mocoPropulsionBounds(1); 'propulsion_upper', mocoPropulsionBounds(2); ...
        'propulsion_unit', propulsionUnit}];
end
if mocoPropulsionWeight > 0 && isempty(mocoPropulsionReferenceDuration)
    runParams(end+1, :) = {'effective_mocoPropulsionReferenceDuration_s', ...
                           effPropulsionReferenceDuration};
end

% Output 폴더
OutputFolder = fullfile(resultsRoot, result_name);
if ~exist(OutputFolder, 'dir')
    mkdir(OutputFolder);
end

%% --------------------------------------------------
%  2. 초기 데이터 경로 설정
% ---------------------------------------------------
inputPath      = fullfile(baseFolder, '..','inputs');

% 모델명에서 height suffix 탐지 (예: _150cm, _160cm, ...)
heightTok = regexp(modelName, '_(\d+cm)', 'tokens', 'once');
if ~isempty(heightTok)
    heightSuffix = ['_' heightTok{1}];
else
    heightSuffix = '';
end

if ~isempty(modelProfile.defaultGuessFile)
    % profile 전용 기본 guess/GRF (예: rajagopal_AFO). stateFixed 실행은 opts.guessInitSto로
    % 정렬된 작업용 사본을 넘긴다.
    guessInitSto = fullfile(inputPath, modelProfile.defaultGuessFile);
    grfInitSto   = fullfile(inputPath, modelProfile.defaultGrfFile);
elseif strcmpi(gaitMode, 'modeAsym')
    guessInitSto = fullfile(inputPath, ['guess_init_full' heightSuffix '.sto']);
    grfInitSto   = fullfile(inputPath, 'GRF_init_full.sto');
else  % modeSym
    guessInitSto = fullfile(inputPath, ['guess_init_half' heightSuffix '.sto']);
    grfInitSto   = fullfile(inputPath, 'GRF_init_full.sto'); % GRF는 modeSym에서도 full 사용 -> 보조력 계산 시 full GRF traj가 필요
end

[~, guessInitName, guessInitExt] = fileparts(guessInitSto);
% 분석 setup과 출력 파일 접두어 (2D: analysis_setup.xml / 2D_gait_AFO_pc_*,
% Rajagopal: analysis_setup_rajagopal.xml / rajagopal_AFO_*, 전체 모델 COM)
AnalySetupPath = fullfile(inputPath, modelProfile.analysisSetupFile);
analysisPrefix = modelProfile.analysisPrefix;
modelPath      = fullfile(baseFolder, '..','models');

%% --------------------------------------------------
%  2.5 실행 범위 결정 (일반 / Resume)
% ---------------------------------------------------
if ~resume_mode
    baseIter  = 0;
    startIter = 1;
    endIter   = iterNum;
    fprintf('=== [%s] Normal mode: result_1 -> result_%d ===\n', modeType, endIter);
else
    % resume_name 가 baseFolder 기준 상대경로 (예: 'my_result/result_300')
    % '\' 와 '/' 구분자를 모두 허용 (Windows/Ubuntu 호환)
    resumeParts = strsplit(resume_name, {'\', '/'});
    resumeAbsDir = fullfile(resultsRoot, resumeParts{:});

    % result_XXX에서 XXX 파싱
    [~, resumeFolderName] = fileparts(resumeAbsDir);
    tok = regexp(resumeFolderName, '^result_(\d+)$', 'tokens', 'once');
    if isempty(tok)
        error('optResume.resume_name 마지막 폴더명이 result_### 형식이어야 합니다: %s', resumeFolderName);
    end
    baseIter  = str2double(tok{1});
    startIter = baseIter + 1;
    endIter   = baseIter + iterNum;

    fprintf('=== [%s] Resume mode: base = result_%d, run result_%d -> result_%d ===\n', ...
        modeType, baseIter, startIter, endIter);
end

%% --------------------------------------------------
%  Pre-loop: Baseline analysis (일반 모드에서만 실행)
%
%  opts.guessInitSto 가 있으면 (preceding modeOff trial 존재):
%    - preceding modeOff 의 GRF → baseline/moco_result/baseline_GRF.sto 복사
%    - preceding modeOff 의 id_withAssist.sto → baseline/analy_result/ 복사
%    - baseline analysis 입력 kinematics = opts.guessInitSto
%  없으면:
%    - baseline analysis 입력 kinematics = default guess_init
% ---------------------------------------------------
if ~resume_mode
    baselineAnalyDir = fullfile(OutputFolder, 'baseline', 'analy_result');
    baselineMocoDir  = fullfile(OutputFolder, 'baseline', 'moco_result');
    if ~exist(baselineAnalyDir, 'dir'), mkdir(baselineAnalyDir); end

    hasPrecedingOff = isfield(opts, 'guessInitSto') && ~isempty(opts.guessInitSto);

    if hasPrecedingOff
        mocoResDir_off  = fileparts(opts.guessInitSto);
        iterRootDir_off = fileparts(mocoResDir_off);
        analyResDir_off = fullfile(iterRootDir_off, 'analy_result');

        if ~exist(baselineMocoDir, 'dir'), mkdir(baselineMocoDir); end

        % GRF 복사 → baseline/moco_result/baseline_GRF.sto
        grfFiles = dir(fullfile(mocoResDir_off, '*_GRF.sto'));
        if ~isempty(grfFiles)
            copyfile(fullfile(mocoResDir_off, grfFiles(1).name), ...
                     fullfile(baselineMocoDir, 'baseline_GRF.sto'));
        else
            warning('[baseline] preceding modeOff moco_result 에서 *_GRF.sto 를 찾지 못했습니다.');
        end

        % kinematics 복사 → baseline/moco_result/ (파일명 그대로)
        kinFiles = dir(fullfile(mocoResDir_off, '*_kinematics*.sto'));
        for kf = kinFiles'
            copyfile(fullfile(mocoResDir_off, kf.name), fullfile(baselineMocoDir, kf.name));
        end
        if isempty(kinFiles)
            warning('[baseline] preceding modeOff moco_result 에서 *_kinematics*.sto 를 찾지 못했습니다.');
        end

        % id_withAssist.sto 복사 → baseline/analy_result/
        idStoSrc = fullfile(analyResDir_off, 'id_withAssist.sto');
        if isfile(idStoSrc)
            copyfile(idStoSrc, fullfile(baselineAnalyDir, 'id_withAssist.sto'));
        else
            warning('[baseline] preceding modeOff analy_result 에서 id_withAssist.sto 를 찾지 못했습니다: %s', analyResDir_off);
        end

        baselineKinPath = opts.guessInitSto;
        fprintf('[initial guess] preceding off trial 사용: %s\n', resultLabel(iterRootDir_off));
    else
        baselineKinPath = guessInitSto;
        fprintf('[initial guess] default 사용: %s%s\n', guessInitName, guessInitExt);
    end

    WoC_moco_analysis(AnalySetupPath, ...
        'modelPath',         fullfile(modelPath, ModelNameOsim), ...
        'kinematicsStoPath', baselineKinPath, ...
        'resultsDir',        baselineAnalyDir);
    fprintf('Baseline analysis done.\n');
end

%% --------------------------------------------------
%  3. 메인 루프
% ---------------------------------------------------
sealedOccurred = false;
sealedIter     = -1;

for i = startIter:endIter
    fprintf('===== Iteration %d / %d =====\n', i, iterNum+baseIter);

    %------------------------------------------------
    % 3-1. 이전 iter의 analysis 결과 경로 및 GRF 경로 결정
    %
    %  prevAnalyDir : control 생성의 입력 데이터 소스
    %    - i==1 (일반): baseline/analy_result/
    %    - i==startIter (resume): resumeAbsDir/analy_result/
    %    - 그 외: result_(i-1)/analy_result/
    %
    %  prevGrfPath : stance 검출용 GRF
    %    - i==1 (일반): inputs/GRF_init_*.sto
    %    - i==startIter (resume): resumeAbsDir/moco_result/...GRF.sto
    %    - 그 외: result_(i-1)/moco_result/...GRF.sto
    %------------------------------------------------
    if ~resume_mode && i == 1
        prevAnalyDir    = fullfile(OutputFolder, 'baseline', 'analy_result');
        baselineGrfPath = fullfile(OutputFolder, 'baseline', 'moco_result', 'baseline_GRF.sto');
        if isfield(opts, 'guessInitSto') && ~isempty(opts.guessInitSto) && isfile(baselineGrfPath)
            prevGrfPath = baselineGrfPath;
        else
            prevGrfPath = grfInitSto;
        end
    elseif resume_mode && i == startIter
        prevAnalyDir = fullfile(resumeAbsDir, 'analy_result');
        prevGrfPath  = fullfile(resumeAbsDir, 'moco_result', ...
                           sprintf('moco_WoC_Solution_iter%02d_GRF.sto', baseIter));
    else
        prevIdx      = i - 1;
        prevAnalyDir = fullfile(OutputFolder, sprintf('result_%d', prevIdx), 'analy_result');
        prevGrfPath  = fullfile(OutputFolder, sprintf('result_%d', prevIdx), 'moco_result', ...
                           sprintf('moco_WoC_Solution_iter%02d_GRF.sto', prevIdx));
    end

    % 이번 iteration의 result_i 폴더들
    iterRootDir      = fullfile(OutputFolder, sprintf('result_%d', i));
    AnalyResultDir   = fullfile(iterRootDir, 'analy_result');
    mocoResultDir    = fullfile(iterRootDir, 'moco_result');
    controlResultDir = fullfile(iterRootDir, 'control_result');

    if ~exist(iterRootDir, 'dir');       mkdir(iterRootDir);       end
    if ~exist(AnalyResultDir, 'dir');    mkdir(AnalyResultDir);    end
    if ~exist(mocoResultDir, 'dir');     mkdir(mocoResultDir);     end
    if ~exist(controlResultDir, 'dir');  mkdir(controlResultDir);  end

    %------------------------------------------------
    % 3-2 ~ 3-6. 모드별 control reference 생성
    %
    %  modeWoC    : QP 기반 WoC 제어기
    %  modeOff    : 보조력 없음 (영벡터 control.sto 생성)
    %  modeSpline : prevGrfPath에서 stance 추출 후 spline 생성
    %------------------------------------------------
    switch modeType

        case 'modeWoC'
            %--------------------------------------------
            % WoC mode: QP 기반 최적 보조 토크 계산
            % (analysis 결과는 prevAnalyDir에서 읽음 — 이전 iter 결과)
            %--------------------------------------------

            CoMPath   = fullfile(prevAnalyDir, [analysisPrefix '_PointKinematics_CoM_pos.sto']);
            CoP_RPath = fullfile(prevAnalyDir, [analysisPrefix '_PointKinematics_CoP_R_pos.sto']);

            % 3-3. GRF에서 오른발 stance time 추출
            [stanceTimeR_101, fullTime] = WoC_moco_detectStance(prevGrfPath);

            % 3-4. QP input (eta, w, t) 계산
            optsCalQP = struct();
            optsCalQP.CoMFields   = {'state_0','state_1','state_2'};
            optsCalQP.CoP_RFields = {'state_0','state_1','state_2'};
            optsCalQP.apAxis   = 'x';
            optsCalQP.vertAxis = 'y';
            optsCalQP.wPath  = fullfile(prevAnalyDir, [analysisPrefix '_Kinematics_u.sto']);
            optsCalQP.wField = 'ankle_angle_r';
            optsCalQP.vPath  = fullfile(prevAnalyDir, [analysisPrefix '_Kinematics_u.sto']);
            optsCalQP.vField = 'pelvis_tx';

            [etaR_101, wR_101, v_101, dt] = WoC_moco_cal_QP_input( ...
                CoMPath, CoP_RPath, stanceTimeR_101, optsCalQP);

            % 3-5. QP 풀어서 tau_R(stance 구간 control) 계산
            qpOpts             = struct();
            qpOpts.QP_effort   = coeffi_cost;
            qpOpts.QP_smooth   = coeffi_smoothing;
            qpOpts.tauDotMax   = 10;

            one_101 = ones(101,1);
            if eta_tau
                QP_in     = one_101;  qpColName = 'one_101';
            elseif eta_tau_w
                QP_in     = wR_101;   qpColName = 'wR_101';
            elseif eta_tau_v
                QP_in     = v_101;    qpColName = 'v_101';
            else
                QP_in     = one_101;  qpColName = 'one_101';
            end
            if (eta_tau + eta_tau_w + eta_tau_v) > 1
                error('Main Cost flag는 eta_tau / eta_tau_w / eta_tau_v 중 하나만 true여야 합니다.');
            end
            tau_R = WoC_moco_solveQP(etaR_101, QP_in, dt, qpOpts);

            % 3-6. 전체 시간축 control.sto + data.csv 출력
            writeOpts.dataColName = qpColName;
            WoC_moco_writeControl(controlResultDir, ...
                fullTime, stanceTimeR_101, tau_R, etaR_101, QP_in, writeOpts);

            controlRefStoPath = fullfile(controlResultDir, 'control.sto');

        case 'modeOff'
            %--------------------------------------------
            % Off mode: 보조력 없음 (AFO = 0 전구간)
            % Analysis / QP 생략. 영벡터 control.sto 생성.
            %--------------------------------------------
            controlRefStoPath = fullfile(controlResultDir, 'control.sto');
            fid = fopen(controlRefStoPath, 'w');
            if fid == -1
                error('control.sto 파일을 열 수 없습니다: %s', controlRefStoPath);
            end
            fprintf(fid, 'controls\n');
            fprintf(fid, 'nRows=2\n');
            fprintf(fid, 'nColumns=3\n');
            fprintf(fid, 'endheader\n');
            fprintf(fid, 'time\tAFO_r\tAFO_l\n');
            fprintf(fid, '0\t0\t0\n');
            fprintf(fid, '100\t0\t0\n');
            fclose(fid);

        case 'modeSpline'
            %--------------------------------------------
            % Spline mode: trapezoid cubic Hermite 보조력 프로파일
            %
            %   보행 주기 [0, 1] 정규화 도메인에서 spline 생성 후
            %   fullTime 전체 구간에 매핑하여 control.sto 출력.
            %   trigger+rise+flat+fall > 1 이면 fall이 주기 초반에 wrap.
            %   Analysis / QP 생략.
            %--------------------------------------------

            % fullTime 추출 (GRF 파일의 전체 시간 축)
            [~, fullTime] = WoC_moco_detectStance(prevGrfPath);

            % Spline control 생성 ([0, 1] 정규화 도메인 → fullTime 포인트 수 tau 벡터)
            tau_R = WoC_moco_buildSplineControl(modeParams, numel(fullTime));

            % Control.sto + data.csv 출력 (전체 보행 주기에 적용)
            % (eta, w 는 modeSpline에서 의미 없으므로 placeholder 사용)
            dummyEta             = zeros(numel(fullTime), 1);
            dummyW               = ones(numel(fullTime), 1);
            writeOpts_sp         = struct();
            writeOpts_sp.dataColName = 'spline';
            % modeSym: AFO_r 의 50~100% 구간을 AFO_l 의 0~50% 구간에 매핑
            % (왼발이 오른발과 50% 위상차를 가지므로, 반 주기 풀기에서 누락되는
            %  오른발 50%+ 보조력을 왼발 0%+ 에 주입)
            if strcmpi(gaitMode, 'modeSym')
                T_half = (fullTime(end) - fullTime(1)) / 2;
                writeOpts_sp.controlLeft = interp1(fullTime, tau_R, fullTime + T_half, 'linear', 0);
            end
            WoC_moco_writeControl(controlResultDir, ...
                fullTime, fullTime, tau_R, dummyEta, dummyW, writeOpts_sp);

            controlRefStoPath = fullfile(controlResultDir, 'control.sto');

        case 'modeTorqAmp'
            %--------------------------------------------
            % TorqAmp mode: 이전 iter 의 id_withAssist.sto 에서
            %   ankle_angle_r_moment 를 읽어 부호 반전(plantarflexion=양수),
            %   음수 클리핑 후 [0, maxVal] 로 스케일한 control.sto 출력.
            %   Analysis / QP 생략.
            %
            %   i==1        : baseline/analy_result/id_withAssist.sto  (preceding modeOff 복사본)
            %   i==startIter(resume): resumeAbsDir/analy_result/id_withAssist.sto
            %   i>1         : result_(i-1)/analy_result/id_withAssist.sto
            %--------------------------------------------

            if ~resume_mode && i == 1
                idStoPath = fullfile(OutputFolder, 'baseline', 'analy_result', 'id_withAssist.sto');
            elseif resume_mode && i == startIter
                idStoPath = fullfile(resumeAbsDir, 'analy_result', 'id_withAssist.sto');
            else
                idStoPath = fullfile(OutputFolder, sprintf('result_%d', i-1), 'analy_result', 'id_withAssist.sto');
            end

            if ~isfile(idStoPath)
                error('modeTorqAmp: id_withAssist.sto 를 찾을 수 없습니다: %s', idStoPath);
            end
            fprintf('[modeTorqAmp] 발목 토크 참조: %s\n', resultLabel(fileparts(idStoPath)));

            cutoffHz_ta = getOpt(modeParams, 'cutoffHz', 6);
            [tau_R_raw, t_id] = WoC_moco_buildTorqAmpControl(idStoPath, modeParams.maxVal, cutoffHz_ta);
            if strcmpi(gaitMode, 'modeSym')
                % i=1 에서 id_withAssist.sto 가 half-cycle만 커버할 수 있으므로
                % GRF 기반 fullTime 에 보간하여 time 축을 full gait cycle 로 통일
                [~, fullTime] = WoC_moco_detectStance(prevGrfPath);
                tau_R = interp1(t_id, tau_R_raw, fullTime, 'linear', 0);
            else  % modeAsym
                % id_withAssist.sto 가 항상 full gait cycle을 커버하므로 시간축 직접 사용
                fullTime = t_id;
                tau_R    = tau_R_raw;
            end

            dummyEta_ta          = zeros(numel(fullTime), 1);
            dummyW_ta            = ones(numel(fullTime), 1);
            writeOpts_ta         = struct();
            writeOpts_ta.dataColName = 'torqAmp';
            % modeSym: 오른발 50~100% plantarflexion torque 를 왼발 0~50% 에 주입
            if strcmpi(gaitMode, 'modeSym')
                T_half = (fullTime(end) - fullTime(1)) / 2;
                writeOpts_ta.controlLeft = interp1(fullTime, tau_R, fullTime + T_half, 'linear', 0);
            end
            WoC_moco_writeControl(controlResultDir, ...
                fullTime, fullTime, tau_R, dummyEta_ta, dummyW_ta, writeOpts_ta);

            controlRefStoPath = fullfile(controlResultDir, 'control.sto');

    end  % switch modeType

    %------------------------------------------------
    % 3-7. Moco loop에 사용할 guess 경로 결정
    %------------------------------------------------
    if ~resume_mode
        if i == 1
            if isfield(opts, 'guessInitSto') && ~isempty(opts.guessInitSto)
                guessStoPath = opts.guessInitSto;
            else
                guessStoPath = guessInitSto;
            end
        else
            prevMocoDir = fullfile(OutputFolder, sprintf('result_%d', i-1), 'moco_result');
            if strcmpi(gaitMode, 'modeSym')
                guessStoPath = fullfile(prevMocoDir, ...
                    sprintf('moco_WoC_Solution_iter%02d_kinematics_half.sto', i-1));
            else  % modeAsym
                guessStoPath = fullfile(prevMocoDir, ...
                    sprintf('moco_WoC_Solution_iter%02d_kinematics.sto', i-1));
            end
        end
    else
        if i == startIter
            prevMocoDir = fullfile(resumeAbsDir, 'moco_result');
        else
            prevMocoDir = fullfile(OutputFolder, sprintf('result_%d', i-1), 'moco_result');
        end
        prevIterIdx = i - 1;
        if i == startIter
            prevIterIdx = baseIter;
        end
        if strcmpi(gaitMode, 'modeSym')
            guessStoPath = fullfile(prevMocoDir, ...
                sprintf('moco_WoC_Solution_iter%02d_kinematics_half.sto', prevIterIdx));
        else  % modeAsym
            guessStoPath = fullfile(prevMocoDir, ...
                sprintf('moco_WoC_Solution_iter%02d_kinematics.sto', prevIterIdx));
        end
    end

    %------------------------------------------------
    % 3-8. Moco loop: control reference → 새로운 보행 궤적
    %       주입된 osim은 AnalyResultDir에 저장됨
    %------------------------------------------------
    baseOsimPath = fullfile(modelPath, ModelNameOsim);

    % 사용한 parameter 기록 (solve가 실패해도 남도록 solve 전에 저장)
    WoC_moco_saveParameters(fullfile(mocoResultDir, ...
        sprintf('moco_WoC_Solution_iter%02d_parameters.csv', i)), ...
        [runParams; {'iteration', i; 'guess_sto', guessStoPath; ...
                     'control_sto', controlRefStoPath; ...
                     'saved_at', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'))}]);

    [sol, loopSetup] = moco_WoC_loop(controlRefStoPath, guessStoPath, i, AnalyResultDir, baseOsimPath, MocoOpts);
    if mocoSkipSolve
        % 설정 점검: solve하지 않고 문제 순서로 재구성한 initial guess로 후처리를 점검한다.
        warning('WoC_moco_main:SkipSolve', ...
            '[Iter %d] mocoSkipSolve: solve 생략, initial guess를 결과 파일로 저장합니다 (설정 점검용).', i);
        sol = loopSetup.guess;
    end

    %------------------------------------------------
    % 3-8b. Sealed 검사: solver 실패(미수렴/max iter) 시
    %        unseal 후 _unseal 접미사로 저장하고 루프 종료.
    %        수렴하지 않은 궤적을 이후 analysis / ID에 넘기지 않는다.
    %------------------------------------------------
    if ~mocoSkipSolve && sol.isSealed()   % (설정 점검의 guess는 MocoSolution이 아니다)
        warning('[Iter %d] Solver did not converge (solution sealed). Saving partial result and stopping.', i);
        sol.unseal();
        resOpts_unseal           = struct();
        resOpts_unseal.modelPath = fullfile(AnalyResultDir, sprintf('%s_%d.osim', modelName, i));
        resOpts_unseal.prefix    = sprintf('moco_WoC_Solution_iter%02d_unseal', i);
        resOpts_unseal.gaitMode  = gaitMode;
        moco_WoC_getResult(sol, mocoResultDir, resOpts_unseal);
        sealedOccurred = true;
        sealedIter     = i;
        break;
    end

    %------------------------------------------------
    % 3-9. Moco 결과 저장 (kinematics, GRF, metabolic)
    %------------------------------------------------
    resOpts           = struct();
    resOpts.modelPath = fullfile(AnalyResultDir, sprintf('%s_%d.osim', modelName, i));
    resOpts.prefix    = sprintf('moco_WoC_Solution_iter%02d', i);
    resOpts.gaitMode  = gaitMode;
    moco_WoC_getResult(sol, mocoResultDir, resOpts);

    %------------------------------------------------
    % 3-10. 현재 iter 결과에 대한 analysis (전 모드 공통)
    %
    %  result_i/moco_result/kinematics.sto → result_i/analy_result/
    %  다음 iter의 control 생성 입력으로 사용된다.
    %------------------------------------------------
    currKinPath = fullfile(mocoResultDir, sprintf('moco_WoC_Solution_iter%02d_kinematics.sto', i));
    currOsmPath = fullfile(AnalyResultDir, sprintf('%s_%d.osim', modelName, i));
    WoC_moco_analysis(AnalySetupPath, ...
        'modelPath',         currOsmPath, ...
        'kinematicsStoPath', currKinPath, ...
        'resultsDir',        AnalyResultDir);

    %------------------------------------------------
    % 3-11. Inverse Dynamics (with / without assist)
    %
    %  id_withAssist.sto   : iter에서 사용한 model (analy_result에 저장된 model)
    %  id_withoutAssist.sto: origin model (입력 model)
    %  Results saved to analy_result/
    %------------------------------------------------

    idXmlPath      = fullfile(inputPath, 'id_setup.xml');
    kinQStoPath    = fullfile(AnalyResultDir, [analysisPrefix '_Kinematics_q.sto']);
    % GRF_setup_id.xml이 analy_result/에 저장되므로 moco_result/는 상대경로 ../moco_result/
    currGrfStoPath = sprintf('../moco_result/moco_WoC_Solution_iter%02d_GRF.sto', i);

    % GRF_setup.xml의 datafile을 현재 iter GRF.sto 경로로 업데이트 후 임시 저장
    grfXmlDoc = xmlread(fullfile(inputPath, 'GRF_setup.xml'));
    grfXmlDoc.getElementsByTagName('datafile').item(0).setTextContent(currGrfStoPath);
    tempGrfXml = fullfile(AnalyResultDir, 'GRF_setup_id.xml');
    xmlwrite(tempGrfXml, grfXmlDoc);

    % with assist (iter에서 사용한 model)
    idTool = InverseDynamicsTool(idXmlPath);
    idTool.setModelFileName(currOsmPath);
    idTool.setCoordinatesFileName(kinQStoPath);
    idTool.setExternalLoadsFileName(tempGrfXml);
    idTool.setResultsDir(AnalyResultDir);
    idTool.setOutputGenForceFileName('id_withAssist.sto');
    applyIdExcludedForces(idTool, modelProfile, idXmlPath);
    idTool.run();

    % without assist (origin model)
    idTool2 = InverseDynamicsTool(idXmlPath);
    idTool2.setModelFileName(baseOsimPath);
    idTool2.setCoordinatesFileName(kinQStoPath);
    idTool2.setExternalLoadsFileName(tempGrfXml);
    idTool2.setResultsDir(AnalyResultDir);
    idTool2.setOutputGenForceFileName('id_withoutAssist.sto');
    applyIdExcludedForces(idTool2, modelProfile, idXmlPath);
    idTool2.run();

    fprintf('Iteration %d done.\n', i);
end

if sealedOccurred
    error('WoC:sealedSolution', ...
        'Solver did not converge at iteration %d (solution sealed). Partial result saved with _unseal suffix.', ...
        sealedIter);
end

fprintf('All iterations finished.\n');
end


%% ---- ID 제외 force ----
function applyIdExcludedForces(idTool, modelProfile, idXmlPath)
% id_setup.xml의 forces_to_exclude(Muscles)에 profile의 접촉력을 더한다. GRF 파일이
% 같은 접촉력에서 만든 외력이므로, 모델 접촉력을 함께 적용하면 이중으로 들어간다.
% (2D profile은 추가 항목이 없어 기존 동작과 같다.)
    import org.opensim.modeling.*
    if isempty(modelProfile.idExtraExcludedForces), return; end
    doc = xmlread(idXmlPath);
    node = doc.getElementsByTagName('forces_to_exclude');
    existing = {};
    if node.getLength() > 0
        existing = strsplit(strtrim(char(node.item(0).getTextContent())));
        existing = existing(~cellfun(@isempty, existing));
    end
    names = ArrayStr();
    for name = [existing, modelProfile.idExtraExcludedForces]
        names.append(name{1});
    end
    idTool.setExcludedForces(names);
end

%% ---- 옵션 읽기용 헬퍼 ----
function val = getOpt(s, field, defaultVal)
    if isfield(s, field) && ~isempty(s.(field))
        val = s.(field);
    else
        val = defaultVal;
    end
end

%% ---- 로그 표시용 헬퍼 ----
function bw = modelBodyWeight(osimPath)
% osim의 Body 질량 합 x g [N] (moco_WoC_loop의 getTotalMass와 같은 값).
% OpenSim Model을 만들면 geometry 경고가 출력되므로 XML에서 직접 읽는다.
    doc = xmlread(osimPath);
    bodies = doc.getElementsByTagName('Body');
    mass = 0;
    for k = 0:bodies.getLength-1
        kids = bodies.item(k).getChildNodes();
        for c = 0:kids.getLength-1
            n = kids.item(c);
            if n.getNodeType() == n.ELEMENT_NODE && strcmp(char(n.getNodeName()), 'mass')
                mass = mass + str2double(char(n.getTextContent()));
            end
        end
    end
    bw = mass * 9.80665;
end

function lbl = resultLabel(iterRootDir)
% 절대경로 대신 'results/' 하위 상대경로(예: 'AF001-1/result_1')만 표시.
    parts = strsplit(iterRootDir, {'\\', '/'});
    idx = find(strcmpi(parts, 'results'), 1, 'last');
    if ~isempty(idx) && idx + 2 <= numel(parts)
        lbl = strjoin(parts(idx+1:idx+2), '/');
    else
        lbl = iterRootDir;
    end
end
