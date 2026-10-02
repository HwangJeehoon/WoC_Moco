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
%                                  (default = 1.0)
%       .mocoPropulsionReferenceDuration : impulse 정규화 기준 시간(s)
%       .gaitMode        : 'modeSym'  - 반 걸음 풀고 좌우 대칭 복사 (default)
%                          'modeAsym' - 한 걸음 전체를 풀고 주기성만 강제

    import org.opensim.modeling.*

    weight_effort    = opts.mocoEffort;
    weight_finalTime = opts.mocoFinalTime;
    weight_propulsion = getOpt(opts, 'mocoPropulsionWeight', 0);
    propulsionSmoothing = getOpt(opts, 'mocoPropulsionSmoothing', 1.0);
    propulsionReferenceDuration = getOpt(opts, ...
        'mocoPropulsionReferenceDuration', []);
    gaitMode         = opts.gaitMode;
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
            periodicGoal.addStatePair(MocoPeriodicityGoalPair(currentStateName));
        end

        periodicGoal.addControlPair(MocoPeriodicityGoalPair('/lumbarAct'));

    else
        error('moco_WoC_loop: 알 수 없는 gaitMode = ''%s''. modeSym 또는 modeAsym 이어야 합니다.', gaitMode);
    end

    % Effort goal (공통)
    effortGoal = MocoControlGoal('effort', weight_effort);
    effortGoal.setExponent(2); % 2로 사용
    effortGoal.setDivideByDisplacement(true);
    problem.addGoal(effortGoal);

    % Final time goal (공통)
    finalTimeGoal = MocoFinalTimeGoal('final_time', weight_finalTime);
    finalTimeGoal.setDivideByDisplacement(true);
    problem.addGoal(finalTimeGoal);

    % Positive anteroposterior GRF impulse goal (optional).
    %
    % The generated GRF columns are the sum of each foot's heel/front
    % sphere-force x-components. MocoOutputExtremumGoal with the 'maximum'
    % mode provides a smooth approximation of max(F_AP, 0). The negative
    % goal weight rewards propulsion because Moco minimizes the objective.
    % Do not divide by the optimized phase duration: this goal must remain
    % proportional to the physical positive impulse (N*s), not mean force.
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

        fprintf(['Moco propulsion goal enabled: weight=%.4g, smoothing=%.4g, ', ...
                 'body weight=%.4g N, reference duration=%.4g s\n'], ...
                 weight_propulsion, propulsionSmoothing, bodyWeight, ...
                 propulsionReferenceDuration);
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
    solver.set_num_mesh_intervals(300); % mesh num = 300
    solver.set_verbosity(2);
    solver.set_optim_solver('ipopt');
    solver.set_optim_convergence_tolerance(1e-3);
    solver.set_optim_constraint_tolerance(1e-3);
    solver.set_optim_max_iterations(1000);

    guessTraj = MocoTrajectory(guessStoPath);
    solver.setGuess(guessTraj);

    %--------------------------------------------------------------
    % 5. 문제 풀기
    %--------------------------------------------------------------
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


function val = getOpt(s, field, defaultVal)
    if isfield(s, field) && ~isempty(s.(field))
        val = s.(field);
    else
        val = defaultVal;
    end
end
