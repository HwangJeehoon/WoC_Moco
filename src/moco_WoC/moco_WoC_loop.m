function moco_WoC_Solution = moco_WoC_loop(controlInitStoPath, guessStoPath, i, resultsDir, modelPath, opts)
% moco_WoC_loop
%
%   AFO reference control(.sto)와 initial guess(.sto)를 받아
%   moco_WoC gait prediction 문제를 세팅하고 solve한 뒤
%   MocoTrajectory를 반환하는 함수.
%
%   입력:
%     controlInitStoPath : AFO reference control .sto 경로
%     guessStoPath       : initial guess trajectory .sto 경로
%     i                  : iteration 번호 (주입 osim 파일명에 사용)
%     resultsDir         : 주입된 osim을 저장할 디렉터리
%     modelPath          : base .osim 경로
%     opts               : (선택) struct
%       .mocoEffort      : effort goal weight (default = 1.0)
%       .mocoFinalTime   : final time goal weight (default = 0.03)
%       .mocoPropulsionWeight : 양의 apGRF impulse 최대화 weight
%                               (default = 0; 0이면 비활성)
%       .mocoPropulsionSmoothing : smooth positive-force 계수 [0.2, 1]
%                                  (default = 0.2)
%       .mocoPropulsionReferenceDuration : impulse 정규화 기준 시간(s)
%       .mocoObjectiveMode : 'weighted' (default) | 'minSpeedFixedEP'
%                            'minSpeedFixedEP'은 평균속도만 최소화하고 effort와
%                            propulsion을 endpoint equality로 고정한다
%                            (플러그인 osimFixedEffortPropulsion 필요)
%       .mocoEffortTarget : 'minSpeedFixedEP'의 E0
%       .mocoPropulsionTarget : 'minSpeedFixedEP'의 J [N*s, 한 보행주기 gated Iplus]
%       .mocoPropulsionGateSigma / .mocoPropulsionGateThreshold :
%                            'minSpeedFixedEP' gate g(F) = 1/(1+exp(-(F-F0)/sigma))의
%                            sigma, F0 [N] (default = 1, 5)
%       .mocoSpeedWeight : 'minSpeedFixedEP' 평균속도 cost weight (default = 1)
%       .mocoMeshIntervals : CasADi mesh interval 수 (default = 300)
%       .mocoConvergenceTolerance : IPOPT convergence tolerance (default = 1e-3)
%       .mocoConstraintTolerance  : IPOPT constraint tolerance (default = 1e-3)
%       .mocoMaxIterations        : IPOPT 최대 반복 수 (default = 1000)
%       .gaitMode        : 'modeSym'  - 반 걸음 풀고 좌우 대칭 복사 (default)
%                          'modeAsym' - 한 걸음 전체를 풀고 주기성만 강제

    import org.opensim.modeling.*

    weight_effort    = opts.mocoEffort;
    weight_finalTime = opts.mocoFinalTime;
    weight_propulsion = getOpt(opts, 'mocoPropulsionWeight', 0);
    propulsionSmoothing = getOpt(opts, 'mocoPropulsionSmoothing', 0.2);
    propulsionReferenceDuration = getOpt(opts, ...
        'mocoPropulsionReferenceDuration', []);
    useMinSpeed = strcmp(getOpt(opts, 'mocoObjectiveMode', 'weighted'), 'minSpeedFixedEP');
    gateSigma = getOpt(opts, 'mocoPropulsionGateSigma', 1.0);
    gateThreshold = getOpt(opts, 'mocoPropulsionGateThreshold', 5.0);
    integratorPath = '/effortPropulsion';
    gaitMode         = opts.gaitMode;
    % modeSym은 반 걸음을 풀므로 한 보행주기 propulsion은 반주기 적분의 2배다.
    impulseScale = 1 + strcmpi(gaitMode, 'modeSym');
    mocoTimeBound    = opts.mocoTimeBound;
    mocoDistBound    = opts.mocoDistBound;

    %--------------------------------------------------------------
    % 1. MocoStudy 및 문제 정의
    %--------------------------------------------------------------
    study = MocoStudy();
    study.setName('moco_WoC');
    problem = study.updProblem();

    %--------------------------------------------------------------
    % 1-2. Prescribed controller 설정 (control.sto → GCVSpline 주입)
    %--------------------------------------------------------------
    half_order     = 1;
    error_variance = 0;
    col_r = 'AFO_r';
    col_l = 'AFO_l';

    baseOsimPath = modelPath;
    [~, modelName, ~] = fileparts(baseOsimPath);

    % STO 읽기
    sto = Storage(controlInitStoPath);
    tArr = ArrayDouble();
    sto.getTimeColumn(tArr);
    N = tArr.getSize();

    labels = sto.getColumnLabels();
    nCol   = labels.getSize();

    idx_r = -1; idx_l = -1;
    for k0 = 0:nCol-1
        nm = char(labels.get(k0));
        if strcmp(nm, col_r), idx_r = k0; end
        if strcmp(nm, col_l), idx_l = k0; end
    end
    if idx_r < 0 || idx_l < 0
        error('control.sto에 %s 또는 %s 컬럼이 없음', col_r, col_l);
    end

    yArr_r = ArrayDouble(); sto.getDataColumn(col_r, yArr_r);
    yArr_l = ArrayDouble(); sto.getDataColumn(col_l, yArr_l);

    x = zeros(1, N); yr = zeros(1, N); yl = zeros(1, N);
    for ii = 0:N-1
        x(ii+1)  = tArr.get(ii);
        yr(ii+1) = yArr_r.get(ii);
        yl(ii+1) = yArr_l.get(ii);
    end

    x_str  = strtrim(sprintf(' %.17g', x));
    yr_str = strtrim(sprintf(' %.17g', yr));
    yl_str = strtrim(sprintf(' %.17g', yl));
    w_str  = strtrim(sprintf(' %.17g', ones(1,N)));
    c_str  = strtrim(sprintf(' %.17g', zeros(1,N)));

    % base osim XML 로드 → GCVSpline 주입
    doc = xmlread(baseOsimPath);

    pcNode = [];
    allPC  = doc.getElementsByTagName('PrescribedController');
    for p = 0:allPC.getLength-1
        n = allPC.item(p);
        a = n.getAttributes().getNamedItem('name');
        if ~isempty(a) && strcmp(char(a.getValue()), 'AFO_controller')
            pcNode = n; break;
        end
    end
    if isempty(pcNode)
        error('osim에서 PrescribedController name="AFO_controller"를 못 찾음');
    end

    fsNode = [];
    kids = pcNode.getChildNodes();
    for kk = 0:kids.getLength-1
        n = kids.item(kk);
        if n.getNodeType() == n.ELEMENT_NODE && strcmp(char(n.getNodeName()), 'FunctionSet')
            a = n.getAttributes().getNamedItem('name');
            if ~isempty(a) && strcmp(char(a.getValue()), 'ControlFunctions')
                fsNode = n; break;
            end
        end
    end
    if isempty(fsNode)
        error('AFO_controller 아래 FunctionSet name="ControlFunctions"를 못 찾음');
    end

    objectsNode = [];
    kids = fsNode.getChildNodes();
    for kk = 0:kids.getLength-1
        n = kids.item(kk);
        if n.getNodeType() == n.ELEMENT_NODE && strcmp(char(n.getNodeName()), 'objects')
            objectsNode = n; break;
        end
    end
    if isempty(objectsNode)
        error('ControlFunctions 아래 <objects>를 못 찾음');
    end

    while objectsNode.hasChildNodes()
        objectsNode.removeChild(objectsNode.getFirstChild());
    end

    g1 = doc.createElement('GCVSpline'); g1.setAttribute('name', col_r);
    e = doc.createElement('half_order');     e.appendChild(doc.createTextNode(num2str(half_order))); g1.appendChild(e);
    e = doc.createElement('error_variance'); e.appendChild(doc.createTextNode(num2str(error_variance))); g1.appendChild(e);
    e = doc.createElement('x');             e.appendChild(doc.createTextNode(x_str));  g1.appendChild(e);
    e = doc.createElement('y');             e.appendChild(doc.createTextNode(yr_str)); g1.appendChild(e);
    e = doc.createElement('weights');       e.appendChild(doc.createTextNode(w_str));  g1.appendChild(e);
    e = doc.createElement('coefficients');  e.appendChild(doc.createTextNode(c_str));  g1.appendChild(e);
    objectsNode.appendChild(g1);

    g2 = doc.createElement('GCVSpline'); g2.setAttribute('name', col_l);
    e = doc.createElement('half_order');     e.appendChild(doc.createTextNode(num2str(half_order))); g2.appendChild(e);
    e = doc.createElement('error_variance'); e.appendChild(doc.createTextNode(num2str(error_variance))); g2.appendChild(e);
    e = doc.createElement('x');             e.appendChild(doc.createTextNode(x_str));  g2.appendChild(e);
    e = doc.createElement('y');             e.appendChild(doc.createTextNode(yl_str)); g2.appendChild(e);
    e = doc.createElement('weights');       e.appendChild(doc.createTextNode(w_str));  g2.appendChild(e);
    e = doc.createElement('coefficients');  e.appendChild(doc.createTextNode(c_str));  g2.appendChild(e);
    objectsNode.appendChild(g2);

    injectedOsimPath = fullfile(resultsDir, sprintf('%s_%d.osim', modelName, i));
    xmlwrite(injectedOsimPath, doc);

    modelProcessor = ModelProcessor(injectedOsimPath);
    problem.setModelProcessor(modelProcessor);
    model = modelProcessor.process();
    if useMinSpeed
        % 적분 상태 component는 Moco 문제의 모델에만 추가한다.
        % 저장되는 injected osim은 플러그인 없이도 읽을 수 있게 유지된다.
        moco_WoC_loadFixedEffortPropulsionPlugin();
        addEffortPropulsionIntegrator(model, integratorPath, ...
            gateSigma, gateThreshold, impulseScale);
        problem.setModelProcessor(ModelProcessor(model));
    end
    modelState = model.initSystem();

    %--------------------------------------------------------------
    % 2. Goal 설정
    %--------------------------------------------------------------
    numStates  = model.getNumStateVariables();
    stateNames = model.getStateVariableNames();

    if strcmpi(gaitMode, 'modeSym')
        %----------------------------------------------------------
        % modeSym: 좌우 대칭 goal (반 걸음 → createPeriodicTrajectory)
        %   - jointset 좌표, Activation : _r ↔ _l 교차 페어
        %   - pelvis_tx 제외
        %----------------------------------------------------------
        symmetryGoal = MocoPeriodicityGoal('symmetryGoal');
        problem.addGoal(symmetryGoal);

        for i = 0:numStates-1
            currentStateName = char(stateNames.get(i));
            sName = string(currentStateName);
            if startsWith(sName, "/jointset")
                if contains(sName, "_r")
                    symmetryGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName, ...
                        char(regexprep(sName, "_r", "_l"))));
                elseif contains(sName, "_l")
                    symmetryGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName, ...
                        char(regexprep(sName, "_l", "_r"))));
                else
                    if ~contains(sName, "pelvis_tx/value") && ~contains(sName, "/activation")
                        symmetryGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName));
                    end
                end
            end
        end

        for i = 0:numStates-1
            currentStateName = char(stateNames.get(i));
            sName = string(currentStateName);
            if endsWith(sName, "/activation")
                if contains(sName, "_r")
                    symmetryGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName, ...
                        char(regexprep(sName, "_r", "_l"))));
                elseif contains(sName, "_l")
                    symmetryGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName, ...
                        char(regexprep(sName, "_l", "_r"))));
                end
            end
        end

        symmetryGoal.addControlPair(MocoPeriodicityGoalPair('/lumbarAct'));

    elseif strcmpi(gaitMode, 'modeAsym')
        %----------------------------------------------------------
        % modeAsym: 주기성 goal (한 걸음 전체)
        %   - 모든 state에 동일 side 페어 (끝 = 시작)
        %   - pelvis_tx 제외
        %----------------------------------------------------------
        periodicGoal = MocoPeriodicityGoal('periodicGoal');
        problem.addGoal(periodicGoal);

        for i = 0:numStates-1
            currentStateName = char(stateNames.get(i));
            sName = string(currentStateName);
            if contains(sName, 'pelvis_tx/value')
                continue;
            end
            % 적분 상태는 0에서 시작해 증가하므로 주기성을 걸지 않는다.
            if startsWith(sName, [integratorPath '/'])
                continue;
            end
            periodicGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName));
        end

        periodicGoal.addControlPair(MocoPeriodicityGoalPair('/lumbarAct'));

    else
        error('moco_WoC_loop: 알 수 없는 gaitMode = ''%s''. modeSym 또는 modeAsym 이어야 합니다.', gaitMode);
    end

    if useMinSpeed
        % Minimum average speed at fixed effort and propulsion.
        %
        %   minimize   v_avg = (pelvis_tx(tf) - pelvis_tx(t0)) / (tf - t0)
        %   subject to E = mocoEffortTarget, P = mocoPropulsionTarget
        %
        % E is the effort goal of the weighted mode before its weight. P is
        % the gated propulsive impulse Iplus of MeanPropulsiveForce
        % (meanPropPerVel_goal): each foot's heel + front AP force F is
        % gated by g(F) = 1/(1+exp(-(F-F0)/sigma)) and F*g(F) is summed over
        % the feet. Both integrals are states of the
        % EffortPropulsionIntegrator component (plugin
        % osimFixedEffortPropulsion), zero at t0:
        %   z_E' = sum_c u_c^2,   z_P' = F_r g(F_r) + F_l g(F_l),
        % and the equalities are MocoFinalOutputGoal endpoint constraints
        %   z_E(tf) / d_COM = E0,   impulseScale * z_P(tf) = J.
        % (MocoControlGoal does not support endpoint constraint mode, and an
        % integral endpoint constraint goal makes the constraint Jacobian
        % about 27 times slower to evaluate.) The effort, final time and
        % propulsion costs are not added, so v_avg is the only cost.
        % modeSym solves a half stride whose right and left halves together
        % form one foot's full cycle, so the full stride P is twice the
        % half-stride integral. E (per COM displacement) and v_avg are equal
        % for the half and full stride.
        speedWeight = getOpt(opts, 'mocoSpeedWeight', 1.0);
        addMinSpeedFixedEPGoals(problem, integratorPath, ...
            opts.mocoEffortTarget, opts.mocoPropulsionTarget, speedWeight);
        fprintf(['Moco minSpeedFixedEP: speed weight=%.4g, E0=%.6g, J=%.6g N*s ', ...
                 '(impulse scale %d, gate sigma %.4g N, F0 %.4g N)\n'], speedWeight, ...
                 opts.mocoEffortTarget, opts.mocoPropulsionTarget, impulseScale, ...
                 gateSigma, gateThreshold);
    else
        % Effort goal (공통)
        effortGoal = MocoControlGoal('effort', weight_effort);
        effortGoal.setExponent(2); % 2로 사용
        effortGoal.setDivideByDisplacement(true);
        problem.addGoal(effortGoal);

        % Final time goal (공통)
        finalTimeGoal = MocoFinalTimeGoal('final_time', weight_finalTime);
        finalTimeGoal.setDivideByDisplacement(true);
        problem.addGoal(finalTimeGoal);
    end

    % Positive anteroposterior GRF impulse goal (optional).
    %
    % The generated GRF columns are the sum of each foot's heel/front
    % sphere-force x-components. MocoOutputExtremumGoal with the 'maximum'
    % mode provides a smooth approximation of max(F_AP, 0). The negative
    % goal weight rewards propulsion because Moco minimizes the objective.
    % Do not divide by the optimized phase duration: this goal must remain
    % proportional to the physical positive impulse (N*s), not mean force.
    % (WoC_moco_main keeps the weight 0 in the minSpeedFixedEP mode.)
    if weight_propulsion > 0
        bodyWeight = model.getTotalMass(modelState) * 9.80665;
        if isempty(propulsionReferenceDuration)
            propulsionReferenceDuration = mean(mocoTimeBound);
            if strcmpi(gaitMode, 'modeAsym')
                propulsionReferenceDuration = 2 * propulsionReferenceDuration;
            end
        end

        if ~isscalar(propulsionReferenceDuration) || ...
                ~isfinite(propulsionReferenceDuration) || ...
                propulsionReferenceDuration <= 0
            error(['moco_WoC_loop: propulsion reference duration must be ', ...
                   'a positive finite scalar.']);
        end

        % Dividing by BW * fixed reference duration makes the user-facing
        % weight dimensionless while preserving the impulse objective.
        weightPerSide = -weight_propulsion / ...
            (2 * bodyWeight * propulsionReferenceDuration);
        addPropulsionGoal(problem, 'r', weightPerSide, propulsionSmoothing);
        addPropulsionGoal(problem, 'l', weightPerSide, propulsionSmoothing);
        % 설정값 로그는 WoC_moco_main 파라미터 블록에서 출력한다.
    end

    %--------------------------------------------------------------
    % 3. Bounds 설정
    %--------------------------------------------------------------
    if strcmpi(gaitMode, 'modeSym')
        % 반 걸음 bounds
        problem.setTimeBounds(0, mocoTimeBound);
        problem.setStateInfo('/jointset/groundPelvis/pelvis_tx/value', [0, 10], 0.0, mocoDistBound);

    elseif strcmpi(gaitMode, 'modeAsym')
        % 한 걸음 bounds (modeSym 대비 시간 및 이동 거리 ~2배)
        problem.setTimeBounds(0, mocoTimeBound*2);
        problem.setStateInfo('/jointset/groundPelvis/pelvis_tx/value', [0, 20], 0.0, mocoDistBound*2);
    end

    % 나머지 bounds (공통)
    % problem.setStateInfo('/jointset/groundPelvis/pelvis_tilt/value', [-20*pi/180, -10*pi/180]);
    % problem.setStateInfo('/jointset/groundPelvis/pelvis_ty/value',   [0.75, 1.25]);
    % problem.setStateInfo('/jointset/hip_l/hip_flexion_l/value',      [-10*pi/180, 60*pi/180]);
    % problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value',      [-10*pi/180, 60*pi/180]);
    % problem.setStateInfo('/jointset/knee_l/knee_angle_l/value',      [-50*pi/180, 0]);
    % problem.setStateInfo('/jointset/knee_r/knee_angle_r/value',      [-50*pi/180, 0]);
    % problem.setStateInfo('/jointset/ankle_l/ankle_angle_l/value',    [-15*pi/180, 25*pi/180]);
    % problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value',    [-15*pi/180, 25*pi/180]);
    % problem.setStateInfo('/jointset/lumbar/lumbar/value',            [0, 20*pi/180]);

    problem.setStateInfo('/jointset/groundPelvis/pelvis_tilt/value', [-20*pi/180, 10*pi/180]);
    problem.setStateInfo('/jointset/groundPelvis/pelvis_ty/value',   [0.75, 1.25]);
    problem.setStateInfo('/jointset/hip_l/hip_flexion_l/value',      [-35*pi/180, 75*pi/180]);
    problem.setStateInfo('/jointset/hip_r/hip_flexion_r/value',      [-35*pi/180, 75*pi/180]);
    problem.setStateInfo('/jointset/knee_l/knee_angle_l/value',      [-90*pi/180, 10*pi/180]);
    problem.setStateInfo('/jointset/knee_r/knee_angle_r/value',      [-90*pi/180, 10*pi/180]);
    problem.setStateInfo('/jointset/ankle_l/ankle_angle_l/value',    [-40*pi/180, 30*pi/180]);
    problem.setStateInfo('/jointset/ankle_r/ankle_angle_r/value',    [-40*pi/180, 30*pi/180]);
    problem.setStateInfo('/jointset/lumbar/lumbar/value',            [-20*pi/180, 20*pi/180]);
    %--------------------------------------------------------------
    % 4. Solver 설정
    %--------------------------------------------------------------
    solver = study.initCasADiSolver();
    solver.set_num_mesh_intervals(getOpt(opts, 'mocoMeshIntervals', 300));
    solver.set_verbosity(2);
    solver.set_optim_solver('ipopt');
    solver.set_optim_convergence_tolerance(getOpt(opts, 'mocoConvergenceTolerance', 1e-3));
    solver.set_optim_constraint_tolerance(getOpt(opts, 'mocoConstraintTolerance', 1e-3));
    solver.set_optim_max_iterations(getOpt(opts, 'mocoMaxIterations', 1000));

    guessTraj = MocoTrajectory(guessStoPath);
    guessTraj = matchIntegratorGuess(guessTraj, model, problem, integratorPath, ...
        useMinSpeed, injectedOsimPath, guessStoPath);
    solver.setGuess(guessTraj);

    %--------------------------------------------------------------
    % 5. 문제 풀기
    %--------------------------------------------------------------
    if useMinSpeed
        checkMinSpeedGoals(problem);
    end
    moco_WoC_Solution = study.solve();

end


function addPropulsionGoal(problem, side, weight, smoothing)
% Add one foot's positive AP-GRF impulse reward.
% SpatialVec index 3 is the translational x component (force-x).
    import org.opensim.modeling.*

    goal = MocoOutputExtremumGoal(['propulsion_' side], weight);
    goal.setOutputPath(['/contactHeel_' side '|sphere_force']);
    goal.setSecondOutputPath(['/contactFront_' side '|sphere_force']);
    goal.setOperation('addition');
    goal.setOutputIndex(3);
    goal.setExtremumType('maximum');
    goal.setSmoothingFactor(smoothing);
    goal.setExponent(1);
    goal.setDivideByDisplacement(false);
    goal.setDivideByDuration(false);
    problem.addGoal(goal);
end


function addEffortPropulsionIntegrator(model, componentPath, gateSigma, ...
        gateThreshold, impulseScale)
% Add the plugin component that integrates the effort rate and the gated
% propulsive impulse rate of each foot (sum of its heel/front sphere_force
% x-components, as in addPropulsionGoal and MeanPropulsiveForce).
    import org.opensim.modeling.*

    component = Component.safeDownCast( ...
        OpenSimObject.newInstanceOfType('EffortPropulsionIntegrator'));
    component.setName(componentPath(2:end));
    PropertyHelper.setValueDouble(gateSigma, component.updPropertyByName('gate_sigma'));
    PropertyHelper.setValueDouble(gateThreshold, ...
        component.updPropertyByName('gate_threshold'));
    PropertyHelper.setValueDouble(impulseScale, ...
        component.updPropertyByName('impulse_scale'));
    sides = {'r', 'right_contact_forces'; 'l', 'left_contact_forces'};
    for k = 1:size(sides, 1)
        input = component.updInput(sides{k, 2});
        input.appendConnecteePath(['/contactHeel_' sides{k, 1} '|sphere_force']);
        input.appendConnecteePath(['/contactFront_' sides{k, 1} '|sphere_force']);
    end
    model.addComponent(component);
    model.finalizeConnections();
end


function addMinSpeedFixedEPGoals(problem, componentPath, effortTarget, ...
        propulsionTarget, speedWeight)
% Add the v_avg cost (weight speedWeight) and the E and P endpoint equality
% constraints.
    import org.opensim.modeling.*

    speedGoal = MocoGoal.safeDownCast( ...
        OpenSimObject.newInstanceOfType('MocoCoordinateAverageSpeedGoal'));
    speedGoal.setName('average_speed');
    speedGoal.setWeight(speedWeight);
    PropertyHelper.setValueString('/jointset/groundPelvis/pelvis_tx', ...
        speedGoal.updPropertyByName('coordinate_path'));
    problem.addGoal(speedGoal);

    % E = z_E(tf) / d_COM: same displacement normalization as the effort goal.
    effortGoal = MocoFinalOutputGoal('effort_constraint');
    effortGoal.setOutputPath([componentPath '|effort_integral']);
    effortGoal.setDivideByDisplacement(true);
    effortGoal.setMode('endpoint_constraint');
    effortGoal.setEndpointConstraintBounds(equalityBounds(effortTarget));
    problem.addGoal(effortGoal);

    % P_gate = impulse_scale * z_P(tf) [N*s, one stride].
    propulsionGoal = MocoFinalOutputGoal('propulsion_constraint');
    propulsionGoal.setOutputPath([componentPath '|scaled_propulsive_impulse']);
    propulsionGoal.setMode('endpoint_constraint');
    propulsionGoal.setEndpointConstraintBounds(equalityBounds(propulsionTarget));
    problem.addGoal(propulsionGoal);

    % Both integrals start at 0. Moco omits states without a state info.
    problem.setStateInfo([componentPath '/effort_integral'], [-Inf, Inf], 0);
    problem.setStateInfo([componentPath '/propulsive_impulse'], [-Inf, Inf], 0);
end


function bounds = equalityBounds(value)
    import org.opensim.modeling.*
    bounds = StdVectorMocoBounds();
    bounds.add(MocoBounds(value));
end


function guessTraj = matchIntegratorGuess(guessTraj, model, problem, ...
        componentPath, useIntegrator, modelPath, guessPath)
% Make the guess match the integral states of the problem. Integral states
% already in the guess (from a minSpeedFixedEP solution, including the
% earlier softplus version) are dropped; when the mode is used, the states
% are rebuilt as running integrals along the guess itself.
    import org.opensim.modeling.*

    statesTable = guessTraj.exportToStatesTable();
    labels = statesTable.getColumnLabels();
    oldStates = {};
    for k = 0:labels.size()-1
        if startsWith(char(labels.get(k)), [componentPath '/'])
            oldStates{end+1} = char(labels.get(k)); %#ok<AGROW>
        end
    end
    if ~useIntegrator && isempty(oldStates)
        return;
    end
    for k = 1:numel(oldStates)
        statesTable.removeColumn(oldStates{k});
    end

    if useIntegrator
        component = model.getComponent(componentPath);
        m = moco_WoC_evalGaitMetrics(modelPath, guessPath, 1, ...
            PropertyHelper.getValueDouble(component.getPropertyByName('gate_sigma')), ...
            PropertyHelper.getValueDouble(component.getPropertyByName('gate_threshold')));
        statePaths = {[componentPath '/effort_integral'], [componentPath '/propulsive_impulse']};
        values = [cumtrapz(m.time, m.effort_rate), ...
                  cumtrapz(m.time, m.gate_impulse_rate)];

        % Moco 4.5 maps guess states to the problem by position. The guess
        % columns are in system order and the component's states come last,
        % so append them in their system order.
        systemOrder = opensimSimulation.createStateVariableNamesInSystemOrder(model);
        systemNames = cell(systemOrder.size(), 1);
        for k = 0:systemOrder.size()-1
            systemNames{k+1} = char(systemOrder.get(k));
        end
        [position, order] = sort(cellfun(@(p) find(strcmp(systemNames, p)), statePaths));
        if ~isequal(position, numel(systemNames) - [1, 0])
            error('moco_WoC_loop: 적분 상태가 system order의 마지막 두 상태가 아닙니다.');
        end
        for k = order
            statesTable.appendColumn(statePaths{k}, Vector.createFromMat(values(:, k)));
        end
    end
    guessTraj = MocoTrajectory.createFromStatesControlsTables( ...
        problem.createRep(), statesTable, guessTraj.exportToControlsTable());
end


function checkMinSpeedGoals(problem)
% Check on the problem Moco solves that v_avg is the only cost, that E and
% P are endpoint constraints (createRep initializes the goals, so an
% unsupported endpoint constraint mode would also throw here) and that both
% read the optimizer's states: MocoCasADiSolver copies them into the
% SimTK::State only for goals that depend on Stage::Time or later.
    names = @(v) arrayfun(@(k) char(v.get(k)), 0:v.size()-1, 'UniformOutput', false);
    rep = problem.createRep();
    costs = names(rep.createCostNames());
    constraints = names(rep.createEndpointConstraintNames());
    fprintf('Moco costs: %s | endpoint constraints: %s\n', ...
        strjoin(costs, ', '), strjoin(constraints, ', '));
    if ~isequal(costs, {'average_speed'}) || ...
            ~all(ismember({'effort_constraint', 'propulsion_constraint'}, constraints))
        error('moco_WoC_loop: minSpeedFixedEP 문제의 cost/endpoint constraint 구성이 예상과 다릅니다.');
    end
    stageNames = {'Topology', 'Model (or lower)', 'Instance', 'Time', 'Position', 'Velocity', ...
                  'Dynamics', 'Acceleration', 'Report'};
    for name = {'effort_constraint', 'propulsion_constraint'}
        level = stageLevel(rep.getEndpointConstraint(name{1}).getStageDependency());
        fprintf('  %s stage dependency: %s\n', name{1}, stageNames{level});
        if level < 4
            error('moco_WoC_loop: %s 의 stage dependency(%s)가 Stage::Time보다 낮습니다.', ...
                name{1}, stageNames{level});
        end
    end
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


function val = getOpt(s, field, defaultVal)
    if isfield(s, field) && ~isempty(s.(field))
        val = s.(field);
    else
        val = defaultVal;
    end
end
