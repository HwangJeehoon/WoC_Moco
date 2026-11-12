function m = moco_WoC_evalGaitMetrics(modelPath, trajPath, impulseScale, gateSigma, gateThreshold, opts)
% moco_WoC_evalGaitMetrics
%
%   저장된 Moco 궤적에서 effort E, metabolic CoT, gated propulsive impulse P_gate,
%   평균속도를 OpenSim 모델로 다시 계산한다. control, 상태, 접촉력에서 직접
%   적분하며(Moco goal, 플러그인 코드, 궤적의 적분 상태 열을 쓰지 않음), 적분
%   상태 열은 무시한다.
%
%   모델별 접촉력 목록, 전진 좌표 경로, lumbar 구동기는 moco_WoC_modelProfile(modelPath)
%   에서 읽는다 (최적화 적분기와 같은 목록).
%
%   E = (1/d) * int sum_c u_c^2 dt
%       c: AFO_r/AFO_l을 제외한 모델 control (2D: 근육 excitation 18개, lumbarAct;
%       Rajagopal: 근육 80개, 토크 구동기 13개). 근육/토크 구동기는 component class로
%       구분한다(moco_WoC_controlGroups). E_afo_simpson은 제외된 AFO control의 진단값이며
%       E에 합산하지 않는다. E_torque_simpson은 lumbar를 뺀 이상적 토크 구동기 몫.
%       d: 질량중심 변위 |r_com(tf) - r_com(t0)| (MocoGoal divide_by_displacement)
%   CoT = int P_met dt / (mass * d)   [J/kg/m]
%       P_met: moco_WoC_addMetabolics의 Bhargava2004SmoothedMuscleMetabolics
%       total_metabolic_rate [W] (basal 포함). 궤적의 상태(근육 activation 포함)와
%       control을 모델 control 이름에 맞춰 적용하고 Stage::Dynamics까지 realize해
%       점마다 계산한다. 궤적에 없는 control(PrescribedController의 AFO)은 모델의
%       controller 값을 쓴다. mass: 모델 총질량, d: E와 같은 COM 변위.
%       모든 점의 P_met이 basal 값과 같으면(근육 미등록) error.
%   P_gate = impulseScale * int [F_r g(F_r) + F_l g(F_l)] dt          [N*s]
%   Tplus  = impulseScale * int [g(F_r) + g(F_l)] dt                    [s]
%       F_foot = 그 발의 profile 접촉력 x-force 합 (2D: contactHeel + contactFront,
%       Rajagopal: 6개 sphere). 발마다 먼저 합산한 뒤 gate를 적용한다.
%       g(F) = 1 / (1 + exp(-(F - F0) / sigma))  (meanPropPerVel의 MeanPropulsiveForce)
%       mean_propulsive_force = P_gate / Tplus  (impulseScale은 약분됨)
%   P_physical = impulseScale * int sum_foot max(F_foot, 0) dt (비교용)
%   v_avg = (pelvis_tx(tf) - pelvis_tx(t0)) / (tf - t0)
%
%   입력:
%     modelPath     : 궤적을 푼 osim (AFO spline이 주입된 result_i/analy_result/*.osim)
%     trajPath      : MocoTrajectory .sto
%     impulseScale  : modeSym 반주기 궤적은 2 (한 주기로 환산), 한 주기 궤적은 1
%     gateSigma     : gate 폭 sigma [N]
%     gateThreshold : gate 중심 F0 [N]
%     opts          : (선택) struct. .metabolics = false면 대사율 계산을 건너뛴다
%                     (default true; 건너뛰면 대사 관련 필드는 NaN)
%
%   출력 m (struct): *_simpson은 Hermite-Simpson 격자(mesh+midpoint, 점 수
%   2N+1)의 Simpson 가중치(Moco 적분과 같은 quadrature), *_trapz는 모든 점에
%   대한 사다리꼴 적분이다. 반주기 궤적의 CoT는 반주기 에너지와 반주기 COM
%   변위로 구하며 impulseScale을 곱하지 않는다.

    import org.opensim.modeling.*
    if nargin < 6 || isempty(opts), opts = struct(); end
    computeMetabolics = ~isfield(opts, 'metabolics') || opts.metabolics;
    profile = moco_WoC_modelProfile(modelPath);

    model = Model(modelPath);
    if computeMetabolics
        [~, metInfo] = moco_WoC_addMetabolics(model);
    end
    model.initSystem();
    traj = MocoTrajectory(trajPath);
    t = traj.getTimeMat();
    n = numel(t);

    % 모델 control 순서로 궤적 control을 이름으로 배치하고, 궤적에 없는 control
    % (PrescribedController가 정하는 AFO)은 모델에서 계산한다.
    groupsInfo = moco_WoC_controlGroups(model, profile);
    ctrlNames = groupsInfo.names;
    nCtrl = numel(ctrlNames);
    trajNames = traj.getControlNames();
    trajCtrl = traj.getControlsTrajectoryMat();
    inTraj = false(nCtrl, 1);
    U = zeros(n, nCtrl);
    for k = 0:trajNames.size()-1
        idx = find(strcmp(ctrlNames, char(trajNames.get(k))));
        if numel(idx) ~= 1
            error('moco_WoC_evalGaitMetrics: 궤적 control %s 가 모델 control에 없습니다.', ...
                char(trajNames.get(k)));
        end
        U(:, idx) = trajCtrl(:, k+1);
        inTraj(idx) = true;
    end
    notPrescribed = setdiff(ctrlNames(~inTraj), ctrlNames(groupsInfo.isPrescribed));
    if ~isempty(notPrescribed)
        error(['moco_WoC_evalGaitMetrics: 궤적에 없고 controller도 정하지 않는 control이 ', ...
               '있습니다: %s'], strjoin(notPrescribed, ', '));
    end

    statesTraj = StatesTrajectory.createFromStatesTable(model, ...
        traj.exportToStatesTable(), false, true);
    contacts = {profile.contactPaths.r, profile.contactPaths.l};   % {right, left}
    forceComps = cell(1, 2);
    for foot = 1:2
        forceComps{foot} = cellfun(@(p) SmoothSphereHalfSpaceForce.safeDownCast( ...
            model.getComponent(p)), contacts{foot}, 'UniformOutput', false);
        if any(cellfun(@isempty, forceComps{foot}))
            error('moco_WoC_evalGaitMetrics: SmoothSphereHalfSpaceForce가 아닌 접촉력 경로가 있습니다: %s', ...
                strjoin(contacts{foot}, ', '));
        end
    end
    F = zeros(n, 2);       % [right, left] foot AP force [N]
    com = zeros(n, 3);
    P = NaN(n, 1);         % total metabolic rate [W]
    Pparts = NaN(n, 4);    % activation, maintenance, shortening, mechanical work [W]
    controlMismatch = 0;
    if computeMetabolics
        metabolics = Bhargava2004SmoothedMuscleMetabolics.safeDownCast( ...
            model.getComponent(metInfo.componentPath));
    end
    for k = 0:n-1
        s = statesTraj.get(k);
        model.realizeVelocity(s);
        modelCtrl = model.getControls(s);
        for c = find(~inTraj)'
            U(k+1, c) = modelCtrl.get(c-1);
        end
        for foot = 1:2
            for j = 1:numel(forceComps{foot})
                F(k+1, foot) = F(k+1, foot) + ...
                    forceComps{foot}{j}.getSphereForce(s).get(1).get(0);
            end
        end
        r = model.calcMassCenterPosition(s);
        com(k+1, :) = [r.get(0), r.get(1), r.get(2)];
        if computeMetabolics
            % 모델 control 순서의 전체 control(궤적 값 + controller 값)을 넣고
            % 대사율 Output의 stage(Dynamics)까지 realize한다.
            model.setControls(s, Vector.createFromMat(U(k+1, :)'));
            model.realizeDynamics(s);
            controlMismatch = max(controlMismatch, ...
                max(abs(model.getControls(s).getAsMat() - U(k+1, :)')));
            P(k+1) = metabolics.getTotalMetabolicRate(s);
            Pparts(k+1, :) = [metabolics.getTotalActivationRate(s), ...
                metabolics.getTotalMaintenanceRate(s), metabolics.getTotalShorteningRate(s), ...
                metabolics.getTotalMechanicalWorkRate(s)];
        end
    end
    if controlMismatch > 1e-12
        error('moco_WoC_evalGaitMetrics: 적용한 control이 궤적 값과 다릅니다 (max %.3g).', ...
            controlMismatch);
    end

    G = gate(F, gateSigma, gateThreshold);
    isAfo = groupsInfo.isAfo;
    groups = struct('muscle', groupsInfo.isMuscle, 'lumbar', groupsInfo.isLumbar, ...
                    'torque', groupsInfo.isTorque & ~groupsInfo.isLumbar, 'afo', isAfo);

    w = simpsonWeights(t);
    m.model_profile = profile.name;
    m.contact_paths = contacts;
    m.n_points = n;
    m.duration = t(end) - t(1);
    pelvisTx = traj.getStateMat([profile.pelvisCoordinatePath '/value']);
    m.distance = pelvisTx(end) - pelvisTx(1);
    m.v_avg = m.distance / m.duration;
    m.com_displacement = norm(com(end, :) - com(1, :));
    m.control_names = ctrlNames;

    effortRate = sum(U(:, ~isAfo).^2, 2);
    m.E_simpson = w' * effortRate / m.com_displacement;
    m.E_trapz = trapz(t, effortRate) / m.com_displacement;
    for g = fieldnames(groups)'
        m.(['E_' g{1} '_simpson']) = w' * sum(U(:, groups.(g{1})).^2, 2) / m.com_displacement;
    end

    m.impulse_scale = impulseScale;
    m.gate_sigma = gateSigma;
    m.gate_threshold = gateThreshold;
    m.P_gate_simpson = impulseScale * sum(w' * (F .* G));
    m.P_gate_trapz = impulseScale * sum(trapz(t, F .* G));
    m.Tplus_simpson = impulseScale * sum(w' * G);
    m.Tplus_trapz = impulseScale * sum(trapz(t, G));
    m.mean_propulsive_force = m.P_gate_simpson / m.Tplus_simpson;
    m.P_physical_simpson = impulseScale * sum(w' * max(F, 0));
    m.P_physical_trapz = impulseScale * sum(trapz(t, max(F, 0)));
    m.P_gate_foot_simpson = w' * (F .* G);          % [right, left], 궤적 구간 그대로
    m.time = t;
    m.F_ap = F;
    m.effort_rate = effortRate;                     % sum_c u_c^2, excluding AFO_r/AFO_l
    m.gate_impulse_rate = sum(F .* G, 2);           % sum_foot F g(F) [N]
    m.gate_time_rate = sum(G, 2);                   % sum_foot g(F)

    % Metabolic CoT: MocoFinalOutputGoal(divide_by_mass, divide_by_displacement)와
    % 같은 정규화 (모델 총질량, 같은 phase의 COM 변위). g나 duration으로 나누지 않는다.
    m.mass_kg = model.getTotalMass(statesTraj.get(0));
    m.metabolic_rate = P;                           % total_metabolic_rate [W]
    m.metabolic_parts = Pparts;                     % [activation, maintenance, shortening, mech. work] [W]
    m.metabolic_energy_simpson = w' * P;            % [J]
    m.metabolic_energy_trapz = trapz(t, P);         % [J]
    m.CoT_simpson = m.metabolic_energy_simpson / (m.mass_kg * m.com_displacement);
    m.CoT_trapz = m.metabolic_energy_trapz / (m.mass_kg * m.com_displacement);
    m.basal_rate_W = NaN;
    m.metabolics_settings = '';
    m.n_metabolic_muscles = 0;
    if computeMetabolics
        m.basal_rate_W = metInfo.basal_coefficient * m.mass_kg ^ metInfo.basal_exponent;
        m.metabolics_settings = metInfo.settings;
        m.n_metabolic_muscles = metInfo.numMuscles;
        % 근육이 대사 모델에 기여하지 않으면 대사율이 basal 상수뿐이다.
        if max(abs(P - m.basal_rate_W)) <= 1e-9 * m.basal_rate_W
            error(['moco_WoC_evalGaitMetrics: 대사율이 모든 점에서 basal 값(%.6g W)과 같습니다. ', ...
                   '근육이 대사 모델에 기여하지 않습니다.'], m.basal_rate_W);
        end
    end
end


function g = gate(F, sigma, F0)
% Logistic gate evaluated so that exp() never overflows (as MeanPropulsiveForce).
    x = (F - F0) / sigma;
    g = zeros(size(x));
    pos = x >= 0;
    g(pos) = 1 ./ (1 + exp(-x(pos)));
    e = exp(x(~pos));
    g(~pos) = e ./ (1 + e);
end


function w = simpsonWeights(t)
% Hermite-Simpson 궤적(mesh, midpoint 교대, 2N+1점)의 Simpson 가중치.
% 점 수가 짝수면 Simpson 격자가 아니므로 사다리꼴 가중치를 쓴다.
    n = numel(t);
    w = zeros(n, 1);
    if mod(n, 2) == 1 && n >= 3
        for k = 1:2:n-2
            h = t(k+2) - t(k);
            w(k:k+2) = w(k:k+2) + h / 6 * [1; 4; 1];
        end
    else
        dt = diff(t(:));
        w(1:end-1) = dt / 2;
        w(2:end) = w(2:end) + dt / 2;
    end
end
