function P = moco_WoC_modelProfile(modelRef)
% moco_WoC_modelProfile
%
%   stateFixed 파이프라인(WoC_moco_main, moco_WoC_loop, moco_WoC_getResult,
%   moco_WoC_evalGaitMetrics, WoC_moco_runStateFixedCases)이 모델마다 다르게 쓰는
%   경로·접촉력·bound·분석 설정을 한 곳에 둔다. 최적화 적분기, 초기값 평가, GRF,
%   CoT/추진 재평가, summary가 모두 같은 접촉력 목록(P.contactPaths)을 쓴다.
%
%   입력:
%     modelRef : 모델 파일 이름 또는 경로 ('2D_gait_AFO_pc.osim', 'rajagopal_AFO.osim',
%                '<case>/result_1/analy_result/rajagopal_AFO_1.osim' 등) 또는 이 함수가
%                돌려준 profile struct (그대로 돌려준다).
%   출력 P (struct):
%     name                 : '2D' | 'rajagopal'
%     pelvisCoordinatePath : 전진 좌표 경로 (v_avg, 거리 bound)
%     contactPaths.r/.l    : 발마다 합산하는 SmoothSphereHalfSpaceForce 절대 경로
%     ...                  : 나머지 필드는 아래 정의 참고
%
%   모델 이름이 rajagopal_AFO로 시작하면 (주입 모델의 _<iter> 접미사 포함)
%   'rajagopal', 그 외에는 기존 동작인 '2D' profile이다.

    if isstruct(modelRef)
        P = modelRef;
        return;
    end
    [~, fileName] = fileparts(char(modelRef));
    baseName = regexprep(fileName, '_\d+$', '');   % 주입 모델 rajagopal_AFO_1 -> rajagopal_AFO

    if startsWith(baseName, 'rajagopal_AFO')
        P = rajagopalProfile();
    else
        P = twoDProfile();
    end
    P.modelName = baseName;
end


function P = twoDProfile()
% 기존 2D_gait_AFO_pc 계열 동작 (값은 이전 하드코딩과 같다).
    P.name = '2D';
    P.pelvisCoordinatePath = '/jointset/groundPelvis/pelvis_tx';
    P.contactPaths = struct('r', {{'/contactHeel_r', '/contactFront_r'}}, ...
                            'l', {{'/contactHeel_l', '/contactFront_l'}});
    P.contactDescription = 'contactHeel + contactFront';
    P.lumbarControlPaths = {'/lumbarAct'};
    % modeAsym control 주기성: '/lumbarAct'만 (기존 동작)
    P.periodicControlMode = 'lumbar';
    P.analysisSetupFile = 'analysis_setup.xml';
    P.analysisPrefix = '2D_gait_AFO_pc';            % analysis_setup.xml의 AnalyzeTool name
    P.comFileSuffix = '_PointKinematics_CoM_pos.sto';
    P.comColumns = {'state_0', 'state_1', 'state_2'};
    P.comDescription = 'pelvis fixed point (PointKinematics "CoM", 2D legacy)';
    P.idExtraExcludedForces = {};
    P.supportedGaitModes = {'modeSym', 'modeAsym'};
    P.supportedOptModes = {'modeWoC', 'modeOff', 'modeSpline', 'modeTorqAmp'};
    P.defaultGuessFile = '';                        % '' = WoC_moco_main의 기존 규칙
    P.defaultGrfFile = '';
    P.effortDefinition = 'int sum_c u_c^2 dt / d_COM; excludes AFO_r and AFO_l; includes lumbarAct';
    P.effortDefinitionShort = 'int sum_c u_c^2 dt / d_COM; excludes AFO_r/AFO_l; includes lumbarAct';
    P.effortControlsText = '18 muscle excitations and lumbarAct (CoordinateActuator, optimal force 1, control = N*m)';
    P.coordinateBoundsDeg = {};                     % 2D bound는 moco_WoC_loop의 기존 코드
    P.torqueLimits = {};
    P.frameDistance = [];
    P.excludeDependentCoordinatesFromPeriodicity = true;   % 2D에는 종속 좌표가 없다
    P.solver = struct('enforceConstraintDerivatives', [], 'kinematicConstraintMethod', '');
end


function P = rajagopalProfile()
% rajagopal_AFO (balance-exo-sim subject19, 68.5 kg, 3D). modeAsym + modeOff만 지원.
    P.name = 'rajagopal';
    P.pelvisCoordinatePath = '/jointset/ground_pelvis/pelvis_tx';
    contactNames = {'Heel', 'LateralRearfoot', 'LateralMidfoot', 'LateralToe', ...
                    'MedialToe', 'MedialMidfoot'};
    P.contactPaths = struct( ...
        'r', {cellfun(@(n) ['/forceset/contact' n '_r'], contactNames, 'UniformOutput', false)}, ...
        'l', {cellfun(@(n) ['/forceset/contact' n '_l'], contactNames, 'UniformOutput', false)});
    P.contactDescription = ['sum of 6 SmoothSphereHalfSpaceForce per foot (', ...
        strjoin(strcat('contact', contactNames), ', '), ')'];
    P.lumbarControlPaths = {'/forceset/torque_lumbar_extension', ...
        '/forceset/torque_lumbar_bending', '/forceset/torque_lumbar_rotation'};
    % modeAsym control 주기성: 모든 optimization control (근육 80 + 토크 구동기 13)
    P.periodicControlMode = 'all';
    P.analysisSetupFile = 'analysis_setup_rajagopal.xml';
    P.analysisPrefix = 'rajagopal_AFO';             % analysis_setup_rajagopal.xml의 AnalyzeTool name
    % 실제 전체 모델 질량중심 (BodyKinematics의 center_of_mass)
    P.comFileSuffix = '_BodyKinematics_pos_global.sto';
    P.comColumns = {'center_of_mass_X', 'center_of_mass_Y', 'center_of_mass_Z'};
    P.comDescription = 'whole-model center of mass (BodyKinematics center_of_mass)';
    % ID에서 모델 접촉력을 빼고 GRF 파일(같은 접촉력에서 만든 외력)만 적용한다.
    P.idExtraExcludedForces = [regexprep(P.contactPaths.r, '^.*/', ''), ...
                               regexprep(P.contactPaths.l, '^.*/', '')];
    P.supportedGaitModes = {'modeAsym'};
    P.supportedOptModes = {'modeOff'};
    P.defaultGuessFile = 'guess_init_rajagopal.sto';
    P.defaultGrfFile = 'GRF_init_rajagopal.sto';
    P.effortDefinition = ['int sum_c u_c^2 dt / d_COM; excludes AFO_r and AFO_l; ', ...
        'includes 80 muscle excitations and 13 torque actuator controls'];
    P.effortDefinitionShort = ['int sum_c u_c^2 dt / d_COM; excludes AFO_r/AFO_l; ', ...
        'includes 80 muscles and 13 torque actuators'];
    P.effortControlsText = ['80 muscle excitations and 13 ActivationCoordinateActuator controls ', ...
        '(torque = optimal_force * activation)'];

    % 관절각 bound [deg] (좌우 동일). 이름에 _r/_l이 없으면 같은 이름의 좌표 또는
    % <이름>_r, <이름>_l 좌표에 적용한다. knee beta는 coupler에 종속된 좌표다.
    P.coordinateBoundsDeg = {
        'pelvis_tilt',       [-15, 15]
        'pelvis_list',       [-10, 10]
        'pelvis_rotation',   [-15, 15]
        'hip_flexion',       [-30, 120]
        'hip_adduction',     [-50, 30]
        'hip_rotation',      [-40, 40]
        'knee_angle',        [0, 120]
        'ankle_angle',       [-40, 30]
        'subtalar_angle',    [-20, 20]
        'mtp_angle',         [-30, 30]
        'lumbar_extension',  [-30, 20]
        'lumbar_bending',    [-15, 15]
        'lumbar_rotation',   [-20, 20]
        'arm_flex',          [-45, 45]
        'arm_add',           [-25, 10]
        'arm_rot',           [-30, 30]
        'elbow_flex',        [0, 110]
        'pro_sup',           [0, 90]
        'knee_angle_r_beta', [-120, 120]
        'knee_angle_l_beta', [-120, 120]};

    % 이상적 토크 구동기 토크 범위 [N*m] (좌표 이름 기준, 좌우 동일).
    % ActivationCoordinateActuator는 torque = optimal_force * activation 이므로
    % control과 /activation에 [-b, b], b = limit / optimal_force 를 적용한다.
    P.torqueLimits = {
        'arm_flex',          10
        'arm_add',           5
        'arm_rot',           12
        'elbow_flex',        7.5
        'pro_sup',           0.5
        'lumbar_extension',  30
        'lumbar_bending',    40
        'lumbar_rotation',   12};

    % 발 사이·팔-몸통 간격 path constraint (balance-exo-sim tracking_problem.py와 같은 설정)
    pairs = {'/bodyset/calcn_l', '/bodyset/calcn_r'
             '/bodyset/toes_l',  '/bodyset/toes_r'
             '/bodyset/calcn_l', '/bodyset/toes_r'
             '/bodyset/toes_l',  '/bodyset/calcn_r'};
    for body = {'humerus', 'ulna', 'radius', 'hand'}
        for side = {'_l', '_r'}
            pairs(end+1, :) = {'/bodyset/torso', ['/bodyset/' body{1} side{1}]}; %#ok<AGROW>
        end
    end
    P.frameDistance = struct('name', 'distance_constraint', 'projection', 'vector', ...
        'projectionVector', [0, 0, 1], 'minimumDistance', 0.05, 'maximumDistance', Inf, ...
        'pairs', {pairs});

    P.excludeDependentCoordinatesFromPeriodicity = true;
    % 슬개골 CoordinateCouplerConstraint: 초기값(lambda/gamma 포함)과 같은 Posa2016 설정
    P.solver = struct('enforceConstraintDerivatives', true, ...
                      'kinematicConstraintMethod', 'Posa2016');
end
