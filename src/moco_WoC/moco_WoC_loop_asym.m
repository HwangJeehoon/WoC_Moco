function moco_WoC_Solution = moco_WoC_loop_asym(guessStoPath, model ,opts)
% moco_WoC_loop
%
%   오른발 AFO reference control(.sto)와
%   tracking solution 기반 initial guess(.sto)를 입력으로 받아
%   moco_WoC gait prediction 문제를 세팅하고 solve한 뒤
%   MocoTrajectory를 반환하는 함수.
%
%   입력:
%     controlInitStoPath : AFO reference control .sto 경로
%     guessStoPath       : initial guess trajectory .sto 경로
%
%   출력:
%     moco_WoC_Solution_fullstride  : MocoTrajectory(study.solve() 결과를 createPeriodicTrajectory로 늘림)
%
%   나머지 설정(모델, goal, bound, solver option 등)은
%   함수 내부 fixed 값으로 구성.

    %% 
    import org.opensim.modeling.*

    if nargin < 3
        opts = struct();
    end

    % --- goal weight defaults ---
    weight_effort     = getOpt(opts, 'weight_effort',     1.0);
    weight_finalTime  = getOpt(opts, 'weight_finalTime',  0.03);

    %--------------------------------------------------------------
    % 1. MocoStudy 및 문제 정의
    %--------------------------------------------------------------
    study = MocoStudy();
    study.setName('moco_WoC');
    problem = study.updProblem();

    baseOsimPath = model;

    modelProcessor = ModelProcessor(baseOsimPath);
    problem.setModelProcessor(modelProcessor);
    model = modelProcessor.process();
    model.initSystem();

    %--------------------------------------------------------------
    % 2. Goal 설정
    %--------------------------------------------------------------

    % 2-1) Periodic goal
    periodicGoal = MocoPeriodicityGoal('periodicGoal');
    problem.addGoal(periodicGoal);

    % 대칭 좌표/속도/Muscle (pelvis_tx 제외)
    numStates = model.getNumStateVariables();
    stateNames = model.getStateVariableNames();  % StdVectorString

    for i = 0:numStates-1
        currentStateName = char(stateNames.get(i));  % char로 받아서 string 처리
        sName = string(currentStateName);
        if contains(sName, 'pelvis_tx/value')
            continue;
        end
        periodicGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName));
    end

    % 대칭 coordinate actuator controls
    periodicGoal.addControlPair(MocoPeriodicityGoalPair('/lumbarAct'));

    % 2-2) Effort over distance goal
    effortGoal = MocoControlGoal('effort', weight_effort);
    effortGoal.setExponent(3);
    effortGoal.setDivideByDisplacement(true);
    problem.addGoal(effortGoal);

    % 2-3) Final time goal
    finalTimeGoal = MocoFinalTimeGoal('final_time', weight_finalTime);
    finalTimeGoal.setDivideByDisplacement(true);
    problem.addGoal(finalTimeGoal);

    %--------------------------------------------------------------
    % 3. Bounds 설정
    %--------------------------------------------------------------
    % setStateInfo(state_name, [lower_bound, upper_bound], initial_value, final_value), 이때 final value는 범위가 될 수 있음
    problem.setTimeBounds(0, [0.8, 1.6]); % 한 걸음을 풀어야 함
    problem.setStateInfo('/jointset/groundPelvis/pelvis_tilt/value', [-20*pi/180, -10*pi/180]);
    problem.setStateInfo('/jointset/groundPelvis/pelvis_tx/value', [0, 3], 0.0, [0.8 2.0]); % set final tx bound
    % problem.setStateInfo('/jointset/groundPelvis/pelvis_tx/value', [0, 2]);
    problem.setStateInfo('/jointset/groundPelvis/pelvis_ty/value', [0.75, 1.25]);
    problem.setStateInfo('/jointset/hip_l/hip_flexion_l/value', [-10*pi/180, 60*pi/180]);
    problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value', [-10*pi/180, 60*pi/180]);
    problem.setStateInfo('/jointset/knee_l/knee_angle_l/value', [-50*pi/180, 0]);
    problem.setStateInfo('/jointset/knee_r/knee_angle_r/value', [-50*pi/180, 0]);
    problem.setStateInfo('/jointset/ankle_l/ankle_angle_l/value', [-15*pi/180, 25*pi/180]);
    problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value', [-15*pi/180, 25*pi/180]);
    problem.setStateInfo('/jointset/lumbar/lumbar/value', [0, 20*pi/180]);

    %--------------------------------------------------------------
    % 4. Solver 설정
    %--------------------------------------------------------------
    solver = study.initCasADiSolver();
    solver.set_num_mesh_intervals(50);
    solver.set_verbosity(2);
    solver.set_optim_solver('ipopt');
    solver.set_optim_convergence_tolerance(1e-4);
    solver.set_optim_constraint_tolerance(1e-4);
    solver.set_optim_max_iterations(10000);

    % 초기 guess: 입력으로 받은 tracking solution
    guessTraj = MocoTrajectory(guessStoPath);
    solver.setGuess(guessTraj);

    %--------------------------------------------------------------
    % 5. 문제 풀기
    %--------------------------------------------------------------
    moco_WoC_Solution = study.solve();

end

function val = getOpt(s, field, defaultVal)
    if isfield(s, field) && ~isempty(s.(field))
        val = s.(field);
    else
        val = defaultVal;
    end
end