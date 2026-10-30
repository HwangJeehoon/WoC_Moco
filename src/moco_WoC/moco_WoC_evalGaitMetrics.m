function m = moco_WoC_evalGaitMetrics(modelPath, trajPath, impulseScale, gateSigma, gateThreshold)
% moco_WoC_evalGaitMetrics
%
%   저장된 Moco 궤적에서 effort E, gated propulsive impulse P_gate, 평균속도를
%   OpenSim 모델로 다시 계산한다. control과 접촉력에서 직접 적분하며(Moco goal,
%   플러그인 코드, 궤적의 적분 상태 열을 쓰지 않음), 적분 상태 열은 무시한다.
%
%   E = (1/d) * int sum_c u_c^2 dt
%       c: 모델의 모든 control (근육 excitation 18개, lumbarAct, AFO_r/AFO_l).
%       AFO는 PrescribedController 값이며 MocoControlGoal의
%       ignore_controlled_actuators=false(기본값)이므로 포함된다.
%       d: 질량중심 변위 |r_com(tf) - r_com(t0)| (MocoGoal divide_by_displacement)
%   P_gate = impulseScale * int [F_r g(F_r) + F_l g(F_l)] dt          [N*s]
%   Tplus  = impulseScale * int [g(F_r) + g(F_l)] dt                    [s]
%       F_foot = contactHeel x-force + contactFront x-force (발에 작용하는 힘)
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
%
%   출력 m (struct): *_simpson은 Hermite-Simpson 격자(mesh+midpoint, 점 수
%   2N+1)의 Simpson 가중치(Moco 적분과 같은 quadrature), *_trapz는 모든 점에
%   대한 사다리꼴 적분이다.

    import org.opensim.modeling.*

    model = Model(modelPath);
    model.initSystem();
    traj = MocoTrajectory(trajPath);
    t = traj.getTimeMat();
    n = numel(t);

    % 모델 control 순서로 궤적 control을 배치하고, 궤적에 없는 control
    % (PrescribedController가 정하는 AFO)은 모델에서 계산한다.
    modelNames = opensimSimulation.createControlNamesFromModel(model);
    nCtrl = modelNames.size();
    ctrlNames = cell(nCtrl, 1);
    for k = 0:nCtrl-1
        ctrlNames{k+1} = char(modelNames.get(k));
    end
    trajNames = traj.getControlNames();
    trajCtrl = traj.getControlsTrajectoryMat();
    inTraj = false(nCtrl, 1);
    U = zeros(n, nCtrl);
    for k = 0:trajNames.size()-1
        idx = find(strcmp(ctrlNames, char(trajNames.get(k))));
        U(:, idx) = trajCtrl(:, k+1);
        inTraj(idx) = true;
    end

    statesTraj = StatesTrajectory.createFromStatesTable(model, ...
        traj.exportToStatesTable(), false, true);
    contacts = {'/contactHeel_r', '/contactFront_r'; '/contactHeel_l', '/contactFront_l'};
    forceComps = cellfun(@(p) SmoothSphereHalfSpaceForce.safeDownCast( ...
        model.getComponent(p)), contacts, 'UniformOutput', false);
    F = zeros(n, 2);       % [right, left] foot AP force [N]
    com = zeros(n, 3);
    for k = 0:n-1
        s = statesTraj.get(k);
        model.realizeVelocity(s);
        modelCtrl = model.getControls(s);
        for c = find(~inTraj)'
            U(k+1, c) = modelCtrl.get(c-1);
        end
        for foot = 1:2
            for j = 1:2
                F(k+1, foot) = F(k+1, foot) + ...
                    forceComps{foot, j}.getSphereForce(s).get(1).get(0);
            end
        end
        r = model.calcMassCenterPosition(s);
        com(k+1, :) = [r.get(0), r.get(1), r.get(2)];
    end

    G = gate(F, gateSigma, gateThreshold);
    isMuscle = endsWith(ctrlNames, {'_r', '_l'}) & ~startsWith(ctrlNames, '/AFO');
    groups = struct('muscle', isMuscle, 'lumbar', strcmp(ctrlNames, '/lumbarAct'), ...
                    'afo', startsWith(ctrlNames, '/AFO'));

    w = simpsonWeights(t);
    m.n_points = n;
    m.duration = t(end) - t(1);
    pelvisTx = traj.getStateMat('/jointset/groundPelvis/pelvis_tx/value');
    m.distance = pelvisTx(end) - pelvisTx(1);
    m.v_avg = m.distance / m.duration;
    m.com_displacement = norm(com(end, :) - com(1, :));
    m.control_names = ctrlNames;

    effortRate = sum(U.^2, 2);
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
    m.effort_rate = effortRate;                     % sum_c u_c^2
    m.gate_impulse_rate = sum(F .* G, 2);           % sum_foot F g(F) [N]
    m.gate_time_rate = sum(G, 2);                   % sum_foot g(F)
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
