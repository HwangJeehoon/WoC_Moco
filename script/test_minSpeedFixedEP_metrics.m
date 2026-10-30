% test_minSpeedFixedEP_metrics.m
%
% minSpeedFixedEP 지표 검증 (최적화 없이 기존 궤적만 사용).
%   B1. 수렴한 reference 궤적에서 EffortPropulsionIntegrator의 C++ rate 출력과
%       moco_WoC_evalGaitMetrics의 독립 MATLAB 계산(control, 접촉력)이 일치하는지
%   B2. 같은 궤적에서 meanPropPerVel 플러그인 MeanPropulsiveForce의 rate와 일치하는지
%   B3. 수렴한 meanPropPerVel 해의 mean propulsive force 상태 m이 P_gate / Tplus와
%       일치하는지 (m은 r(t0) = r(tf) = 0 제약으로 정해지므로 defect 허용오차 수준)
%   B4. 적분 상태가 있는 guess(이전 softplus 버전 해)를 minSpeedFixedEP와 weighted
%       모드 guess로 쓸 수 있는지 (2 iteration solve)
% 실패하면 error로 끝난다. meanPropPerVel 플러그인 DLL
% (plugins/osimMeanPropPerVel/build/Release)이 필요하다.
clc; clear; close all;
thisScriptDir = fileparts(mfilename('fullpath'));
repoDir = fullfile(thisScriptDir, '..');
addpath(genpath(fullfile(repoDir, 'src')));
import org.opensim.modeling.*
moco_WoC_loadFixedEffortPropulsionPlugin();
mppvLib = fullfile(repoDir, 'plugins', 'osimMeanPropPerVel', 'build', 'Release', 'osimMeanPropPerVel.dll');
assert(isfile(mppvLib), 'meanPropPerVel 플러그인 DLL이 없습니다: %s', mppvLib);
assert(opensimCommon.LoadOpenSimLibraryExact(mppvLib), 'meanPropPerVel 플러그인 로드 실패');

resultsDir = fullfile(repoDir, 'results');
gateSigma = 1.0;
gateThreshold = 5.0;

%% B1, B2: reference 궤적
refDir = fullfile(resultsDir, 'wfrac_s0p2_sym_ref_e1_t0p3_p0', 'result_1');
osimPath = fullfile(refDir, 'analy_result', '2D_gait_AFO_pc_1.osim');
trajPath = fullfile(refDir, 'moco_result', 'moco_WoC_Solution_iter01_kinematics_half.sto');
m = moco_WoC_evalGaitMetrics(osimPath, trajPath, 2, gateSigma, gateThreshold);

model = Model(osimPath);
ours = addComponent(model, 'EffortPropulsionIntegrator', 'effortPropulsion', ...
    {'gate_sigma', gateSigma; 'gate_threshold', gateThreshold; 'impulse_scale', 2});
mppv = addComponent(model, 'MeanPropulsiveForce', 'meanPropPerVel', ...
    {'force_smoothing', gateSigma; 'force_threshold', gateThreshold});
model.finalizeConnections();
model.initSystem();
outputs = struct('effort', ours.getOutput('effort_rate'), ...
    'impulse', ours.getOutput('propulsive_impulse_rate'), ...
    'time', ours.getOutput('propulsive_time_rate'), ...
    'mppvImpulse', mppv.getOutput('positive_impulse_rate'), ...
    'mppvTime', mppv.getOutput('positive_time_rate'));
names = fieldnames(outputs);
for k = 1:numel(names)
    outputs.(names{k}) = OutputDouble.safeDownCast(outputs.(names{k}));
end

traj = MocoTrajectory(trajPath);
statesTraj = StatesTrajectory.createFromStatesTable(model, ...
    traj.exportToStatesTable(), true, true);
controlNames = opensimSimulation.createControlNamesFromModel(model);
trajControlNames = traj.getControlNames();
trajControls = traj.getControlsTrajectoryMat();
controlIndex = zeros(trajControlNames.size(), 1);
for j = 0:trajControlNames.size()-1
    for c = 0:controlNames.size()-1
        if strcmp(char(controlNames.get(c)), char(trajControlNames.get(j)))
            controlIndex(j+1) = c;
        end
    end
end
n = statesTraj.getSize();
rates = zeros(n, numel(names));
for i = 0:n-1
    s = statesTraj.get(i);
    model.realizeVelocity(s);
    controls = model.getControls(s);         % AFO: PrescribedController 값
    for j = 1:numel(controlIndex)
        controls.set(controlIndex(j), trajControls(i+1, j));
    end
    model.setControls(s, controls);
    model.realizeDynamics(s);
    for k = 1:numel(names)
        rates(i+1, k) = outputs.(names{k}).getValue(s);
    end
end

relDiff = @(a, b) max(abs(a - b)) / max(abs(b));
checks = {
    'C++ effort_rate vs MATLAB sum u^2',           rates(:, 1), m.effort_rate
    'C++ propulsive_impulse_rate vs MATLAB F*g(F)', rates(:, 2), m.gate_impulse_rate
    'C++ propulsive_time_rate vs MATLAB g(F)',      rates(:, 3), m.gate_time_rate
    'MPPV positive_impulse_rate vs ours',           rates(:, 4), rates(:, 2)
    'MPPV positive_time_rate vs ours',              rates(:, 5), rates(:, 3)};
for k = 1:size(checks, 1)
    d = relDiff(checks{k, 2}, checks{k, 3});
    fprintf('%-46s max rel diff %.3e\n', checks{k, 1}, d);
    assert(d < 1e-9, '%s 불일치 (%.3e)', checks{k, 1}, d);
end
w = simpsonWeights(m.time);
fprintf(['reference (%s): E = %.8f, P_gate = %.6f N*s (C++ rate: %.6f), ', ...
         'Tplus = %.6f s, mean force = %.4f N, P_physical = %.6f N*s, ', ...
         'P_gate - P_physical = %.4f N*s\n'], 'wfrac_s0p2_sym_ref_e1_t0p3_p0', ...
         m.E_simpson, m.P_gate_simpson, 2 * w' * rates(:, 2), m.Tplus_simpson, ...
         m.mean_propulsive_force, m.P_physical_trapz, m.P_gate_trapz - m.P_physical_trapz);
fprintf('  quadrature: P_gate Simpson %.6f vs trapz %.6f; E Simpson %.8f vs trapz %.8f\n', ...
    m.P_gate_simpson, m.P_gate_trapz, m.E_simpson, m.E_trapz);

% 반주기 x2와 한 주기 궤적이 같은 물리량인지.
mFull = moco_WoC_evalGaitMetrics(osimPath, fullfile(refDir, 'moco_result', ...
    'moco_WoC_Solution_iter01_kinematics.sto'), 1, gateSigma, gateThreshold);
fprintf('  half x2 vs full stride: P_gate %.6f / %.6f, Tplus %.6f / %.6f, E %.8f / %.8f\n', ...
    m.P_gate_simpson, mFull.P_gate_simpson, m.Tplus_simpson, mFull.Tplus_simpson, ...
    m.E_simpson, mFull.E_simpson);
assert(abs(m.P_gate_simpson - mFull.P_gate_simpson) < 1e-6 * m.P_gate_simpson, ...
    'modeSym 반주기 x2와 한 주기 P_gate가 다릅니다.');

%% B3: 수렴한 meanPropPerVel 해의 m vs P_gate / Tplus
mppvDir = fullfile(resultsDir, ['smoke_mppv_sym_m300_ct0p0001_kt0p0001_it3000_', ...
    'tb0p3-0p8_db0p3-1_ws_mppv1_s1_F5'], 'result_1');
mppvTraj = fullfile(mppvDir, 'moco_result', 'moco_WoC_Solution_iter01_kinematics_half.sto');
mm = moco_WoC_evalGaitMetrics(fullfile(mppvDir, 'analy_result', '2D_gait_AFO_pc_1.osim'), ...
    mppvTraj, 2, gateSigma, gateThreshold);
mState = MocoTrajectory(mppvTraj).getStateMat('/meanPropPerVel/mean_propulsive_force');
d = abs(mm.mean_propulsive_force - mState(1)) / abs(mState(1));
fprintf('MPPV solution: m state = %.6f N, P_gate/Tplus = %.6f N (rel diff %.2e)\n', ...
    mState(1), mm.mean_propulsive_force, d);
assert(d < 1e-3, 'meanPropPerVel m과 P_gate/Tplus가 다릅니다 (%.2e).', d);

%% B4: 적분 상태가 있는 이전(softplus) 해를 guess로 사용
legacyDir = fullfile(resultsDir, 'smoke_minspeed_m50_sym_J2', 'result_1');
legacyGuess = fullfile(legacyDir, 'moco_result', ...
    'moco_WoC_Solution_iter01_unseal_kinematics_half.sto');
legacyStates = MocoTrajectory(legacyGuess).getStateNames();
assert(any(arrayfun(@(k) strcmp(char(legacyStates.get(k)), ...
    '/effortPropulsion/positive_impulse'), 0:legacyStates.size()-1)), ...
    'legacy guess에 softplus 적분 상태가 없습니다.');
tmpDir = tempname;
mkdir(tmpDir);
opts = struct('mocoEffort', 0, 'mocoFinalTime', 0, 'gaitMode', 'modeSym', ...
    'mocoTimeBound', [0.4 0.8], 'mocoDistBound', [0.4 1.0], 'mocoMeshIntervals', 10, ...
    'mocoMaxIterations', 2, 'mocoObjectiveMode', 'minSpeedFixedEP', ...
    'mocoEffortTarget', m.E_simpson, 'mocoPropulsionTarget', m.P_gate_simpson);
sol = moco_WoC_loop(fullfile(legacyDir, 'control_result', 'control.sto'), legacyGuess, 1, ...
    tmpDir, fullfile(repoDir, 'models', '2D_gait_AFO_pc.osim'), opts);
if sol.isSealed(), sol.unseal(); end
stateNames = sol.getStateNames();
stateNames = arrayfun(@(k) char(stateNames.get(k)), 0:stateNames.size()-1, 'UniformOutput', false);
assert(all(ismember({'/effortPropulsion/effort_integral', '/effortPropulsion/propulsive_impulse'}, ...
    stateNames)) && ~ismember('/effortPropulsion/positive_impulse', stateNames), ...
    'minSpeedFixedEP 해의 적분 상태가 예상과 다릅니다.');
opts.mocoObjectiveMode = 'weighted';
opts.mocoEffort = 1;
opts.mocoFinalTime = 0.3;
sol = moco_WoC_loop(fullfile(legacyDir, 'control_result', 'control.sto'), legacyGuess, 2, ...
    tmpDir, fullfile(repoDir, 'models', '2D_gait_AFO_pc.osim'), opts);
if sol.isSealed(), sol.unseal(); end
stateNames = sol.getStateNames();
assert(~any(arrayfun(@(k) startsWith(char(stateNames.get(k)), '/effortPropulsion/'), ...
    0:stateNames.size()-1)), 'weighted 해에 적분 상태가 남았습니다.');
rmdir(tmpDir, 's');
fprintf('PASS: C++ rates = MATLAB = meanPropPerVel rates; m = P_gate/Tplus; guess states matched.\n');


function component = addComponent(model, className, name, properties)
    import org.opensim.modeling.*
    component = Component.safeDownCast(OpenSimObject.newInstanceOfType(className));
    component.setName(name);
    for k = 1:size(properties, 1)
        PropertyHelper.setValueDouble(properties{k, 2}, ...
            component.updPropertyByName(properties{k, 1}));
    end
    sides = {'r', 'right_contact_forces'; 'l', 'left_contact_forces'};
    for k = 1:size(sides, 1)
        input = component.updInput(sides{k, 2});
        input.appendConnecteePath(['/contactHeel_' sides{k, 1} '|sphere_force']);
        input.appendConnecteePath(['/contactFront_' sides{k, 1} '|sphere_force']);
    end
    model.addComponent(component);
end


function w = simpsonWeights(t)
    n = numel(t);
    w = zeros(n, 1);
    for k = 1:2:n-2
        h = t(k+2) - t(k);
        w(k:k+2) = w(k:k+2) + h / 6 * [1; 4; 1];
    end
end
