% test_minSpeedFixedEP_endpointStage.m
%
% 회귀 테스트: endpoint constraint가 optimizer의 최종 적분 상태를 읽는지 확인한다.
% MocoCasADiSolver는 goal의 stage dependency가 Stage::Time 이상일 때만 optimizer의
% 상태값을 SimTK::State에 복사한다. 적분 상태 출력을 Stage::Model로 선언하면
% endpoint constraint가 이전 상태값을 읽어 아래 문제를 풀지 못한다.
%
% Sliding point mass, control u = 1로 고정, x(0) = v(0) = 0:
%   z_E' = u^2 = 1                        -> effort_integral(tf) = tf
%   z_P' = 2 * v * g(v), v = t, g ~ 1     -> scaled_propulsive_impulse(tf) = tf^2
%   (두 contact 입력을 body velocity 출력에 연결, F0 = -100 N이라 g = 1 - 1e-43)
%   minimize tf  s.t. effort_integral(tf) = 1, scaled_propulsive_impulse(tf) = 1
%   -> tf = 1
% 대조군: 같은 문제를 Stage::Model 출력인 Coordinate value로 x(tf) = 0.5 (tf = 1)를
% 걸면 같은 이유로 풀리지 않아야 한다. 대조군이 풀리면 이 테스트는 stage 버그를
% 검출할 수 없으므로 실패로 처리한다.
clc; clear; close all;
thisScriptDir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(thisScriptDir, '..', 'src')));
import org.opensim.modeling.*
moco_WoC_loadFixedEffortPropulsionPlugin();

% 1) 플러그인 적분 상태 출력 (Stage::Time)
[study, problem] = makeToyProblem(true);
rep = problem.createRep();
stageNames = {'Topology', 'Model (or lower)', 'Instance', 'Time', 'Position', 'Velocity', ...
              'Dynamics', 'Acceleration', 'Report'};
for name = {'effort_constraint', 'propulsion_constraint'}
    level = stageLevel(rep.getEndpointConstraint(name{1}).getStageDependency());
    fprintf('%s stage dependency: %s\n', name{1}, stageNames{level});
    assert(level >= 4, '%s의 stage dependency(%s)가 Stage::Time보다 낮습니다.', ...
        name{1}, stageNames{level});
end
sol = study.solve();
assert(sol.success(), '플러그인 출력 endpoint 문제가 수렴하지 않았습니다 (%s).', char(sol.getStatus()));
t = sol.getTimeMat();
zE = sol.getStateMat('/integrator/effort_integral');
zP = sol.getStateMat('/integrator/propulsive_impulse');
fprintf('plugin outputs: tf = %.8f, z_E(tf) = %.8f, z_P(tf) = %.8f\n', t(end), zE(end), zP(end));
assert(abs(t(end) - 1) < 1e-4, 'tf = %.8f (기대값 1)', t(end));
assert(abs(zE(end) - 1) < 1e-4 && abs(zP(end) - 1) < 1e-4, ...
    'endpoint 적분 상태가 target 1과 다릅니다.');

% 2) 대조군: Stage::Model 출력 (Coordinate value)
[study, problem] = makeToyProblem(false);
level = stageLevel(problem.createRep().getEndpointConstraint('position_constraint') ...
    .getStageDependency());
fprintf('control (Coordinate value) stage dependency: %s\n', stageNames{level});
sol = study.solve();
controlSolved = sol.success();
if ~controlSolved, sol.unseal(); end
t = sol.getTimeMat();
x = sol.getStateMat('/slider/position/value');
fprintf('control: success = %d, tf = %.6f, x(tf) = %.6f\n', controlSolved, t(end), x(end));
assert(~(controlSolved && abs(t(end) - 1) < 1e-4 && abs(x(end) - 0.5) < 1e-4), ...
    ['대조군이 풀렸습니다: Stage::Model 출력도 최신 상태를 읽으므로 ', ...
     '이 테스트로는 stage 버그를 검출할 수 없습니다.']);
fprintf('PASS: endpoint constraints read the final integral states (tf = 1).\n');


function [study, problem] = makeToyProblem(usePlugin)
    import org.opensim.modeling.*
    model = ModelFactory.createSlidingPointMass();
    model.finalizeConnections();
    if usePlugin
        body = model.getBodyList().begin().getAbsolutePathString();
        component = Component.safeDownCast( ...
            OpenSimObject.newInstanceOfType('EffortPropulsionIntegrator'));
        component.setName('integrator');
        PropertyHelper.setValueDouble(-100, component.updPropertyByName('gate_threshold'));
        for input = {'right_contact_forces', 'left_contact_forces'}
            component.updInput(input{1}).appendConnecteePath([char(body) '|velocity']);
        end
        model.addComponent(component);
        model.finalizeConnections();
    end

    study = MocoStudy();
    problem = study.updProblem();
    problem.setModelAsCopy(model);
    problem.setTimeBounds(0, [0.1, 3]);
    problem.setStateInfo('/slider/position/value', [-10, 10], 0);
    problem.setStateInfo('/slider/position/speed', [-10, 10], 0);
    problem.setControlInfo('/forceset/actuator', [1, 1]);
    problem.addGoal(MocoFinalTimeGoal('final_time'));
    if usePlugin
        problem.setStateInfo('/integrator/effort_integral', [-Inf, Inf], 0);
        problem.setStateInfo('/integrator/propulsive_impulse', [-Inf, Inf], 0);
        addEquality(problem, 'effort_constraint', '/integrator|effort_integral', 1);
        addEquality(problem, 'propulsion_constraint', ...
            '/integrator|scaled_propulsive_impulse', 1);
    else
        addEquality(problem, 'position_constraint', '/slider/position|value', 0.5);
    end
    solver = study.initCasADiSolver();
    solver.set_num_mesh_intervals(20);
    solver.set_optim_convergence_tolerance(1e-6);
    solver.set_optim_constraint_tolerance(1e-8);
    solver.set_optim_max_iterations(200);
end


function level = stageLevel(stage)
% SimTK::Stage level (Topology = 1, Model = 2, Instance = 3, Time = 4, ...).
% The MATLAB binding has no level accessor, so count the steps down through
% the runtime stages (Instance ... Report); Model or lower returns 2.
    level = 2;
    while stage.isInRuntimeRange()
        level = level + 1;
        stage = stage.prev();
    end
end


function addEquality(problem, name, outputPath, value)
    import org.opensim.modeling.*
    goal = MocoFinalOutputGoal(name);
    goal.setOutputPath(outputPath);
    goal.setMode('endpoint_constraint');
    bounds = StdVectorMocoBounds();
    bounds.add(MocoBounds(value));
    goal.setEndpointConstraintBounds(bounds);
    problem.addGoal(goal);
end
