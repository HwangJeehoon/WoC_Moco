function moco_WoC_Solution = moco_WoC_loop(controlInitStoPath, guessStoPath, i, resultsDir, modelPath ,opts)
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
    weight_finalTime  = getOpt(opts, 'weight_finalTime',  0.01);

    %--------------------------------------------------------------
    % 1. MocoStudy 및 문제 정의
    %--------------------------------------------------------------
    study = MocoStudy();
    study.setName('moco_WoC');
    problem = study.updProblem();

    %--------------------------------------------------------------
    % 1-2. Prescribed controller 설정
    %--------------------------------------------------------------

    half_order = 1;
    error_variance = 0;
    col_r = 'AFO_r';
    col_l = 'AFO_l';

    baseOsimPath = modelPath;
    [~, modelName, ~] = fileparts(baseOsimPath);

    % ---- 1) STO 읽기 (Storage + ArrayDouble) ----
    sto = Storage(controlInitStoPath);

    tArr = ArrayDouble(); 
    sto.getTimeColumn(tArr);
    N = tArr.getSize();

    labels = sto.getColumnLabels();
    nCol = labels.getSize();

    % AFO_r / AFO_l 찾기
    idx_r = -1; 
    idx_l = -1;
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

    % ---- 2) XML에 넣을 문자열 생성 (space-separated) ----
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

    % ---- 3) base osim XML 로드 ----
    doc = xmlread(baseOsimPath);

    % PrescribedController name="AFO_controller" 찾기
    pcNode = [];
    allPC = doc.getElementsByTagName('PrescribedController');
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

    % FunctionSet name="ControlFunctions" 찾기 (pcNode 직계 자식)
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

    % <objects> 노드 찾기
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

    % ---- 4) objects 내용 비우고 GCVSpline 두 개 삽입 ----
    while objectsNode.hasChildNodes()
        objectsNode.removeChild(objectsNode.getFirstChild());
    end

    % GCVSpline 노드 만들기
    % --- GCVSpline AFO_r ---
    g1 = doc.createElement('GCVSpline'); g1.setAttribute('name', col_r);
    e = doc.createElement('half_order');     e.appendChild(doc.createTextNode(num2str(half_order))); g1.appendChild(e);
    e = doc.createElement('error_variance'); e.appendChild(doc.createTextNode(num2str(error_variance))); g1.appendChild(e);
    e = doc.createElement('x');             e.appendChild(doc.createTextNode(x_str));  g1.appendChild(e);
    e = doc.createElement('y');             e.appendChild(doc.createTextNode(yr_str)); g1.appendChild(e);
    e = doc.createElement('weights');       e.appendChild(doc.createTextNode(w_str));  g1.appendChild(e);
    e = doc.createElement('coefficients');  e.appendChild(doc.createTextNode(c_str));  g1.appendChild(e);
    objectsNode.appendChild(g1);

    % --- GCVSpline AFO_l ---
    g2 = doc.createElement('GCVSpline'); g2.setAttribute('name', col_l);
    e = doc.createElement('half_order');     e.appendChild(doc.createTextNode(num2str(half_order))); g2.appendChild(e);
    e = doc.createElement('error_variance'); e.appendChild(doc.createTextNode(num2str(error_variance))); g2.appendChild(e);
    e = doc.createElement('x');             e.appendChild(doc.createTextNode(x_str));  g2.appendChild(e);
    e = doc.createElement('y');             e.appendChild(doc.createTextNode(yl_str)); g2.appendChild(e);
    e = doc.createElement('weights');       e.appendChild(doc.createTextNode(w_str));  g2.appendChild(e);
    e = doc.createElement('coefficients');  e.appendChild(doc.createTextNode(c_str));  g2.appendChild(e);
    objectsNode.appendChild(g2);

    % ---- 5) 수정된 osim 저장 후 그걸 ModelProcessor로 사용 ----
    injectedOsimPath = fullfile(resultsDir, sprintf('%s_%d.osim', modelName, i));
    xmlwrite(injectedOsimPath, doc);

    modelProcessor = ModelProcessor(injectedOsimPath);
    problem.setModelProcessor(modelProcessor);
    model = modelProcessor.process();
    model.initSystem();

    %--------------------------------------------------------------
    % 2. Goal 설정
    %--------------------------------------------------------------

    % 2-1) Periodicity (symmetry) goal
    symmetryGoal = MocoPeriodicityGoal('symmetryGoal');
    problem.addGoal(symmetryGoal);

    % 대칭 좌표/속도 (pelvis_tx 제외, activation 제외)
    numStates = model.getNumStateVariables();
    stateNames = model.getStateVariableNames();  % StdVectorString

    for i = 0:numStates-1
        currentStateName = char(stateNames.get(i));  % char로 받아서 string 처리
        sName = string(currentStateName);

        if startsWith(sName, "/jointset")
            if contains(sName, "_r")
                pair = MocoPeriodicityGoalPair(currentStateName, ...
                    char(regexprep(sName, "_r", "_l")));
                symmetryGoal.addStatePair(pair);
            elseif contains(sName, "_l")
                pair = MocoPeriodicityGoalPair(currentStateName, ...
                    char(regexprep(sName, "_l", "_r")));
                symmetryGoal.addStatePair(pair);
            else
                if ~contains(sName, "pelvis_tx/value") && ...
                   ~contains(sName, "/activation")
                    symmetryGoal.addStatePair( ...
                        MocoPeriodicityGoalPair(currentStateName));
                end
            end
        end
    end

    % 대칭 muscle activation
    for i = 0:numStates-1
        currentStateName = char(stateNames.get(i));
        sName = string(currentStateName);

        if endsWith(sName, "/activation")
            if contains(sName, "_r")
                pair = MocoPeriodicityGoalPair(currentStateName, ...
                    char(regexprep(sName, "_r", "_l")));
                symmetryGoal.addStatePair(pair);
            elseif contains(sName, "_l")
                pair = MocoPeriodicityGoalPair(currentStateName, ...
                    char(regexprep(sName, "_l", "_r")));
                symmetryGoal.addStatePair(pair);
            end
        end
    end

    % 대칭 coordinate actuator controls
    symmetryGoal.addControlPair(MocoPeriodicityGoalPair('/lumbarAct'));

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
    problem.setTimeBounds(0, [0.4, 0.8]);
    problem.setStateInfo('/jointset/groundPelvis/pelvis_tilt/value', [-20*pi/180, -10*pi/180]);
    problem.setStateInfo('/jointset/groundPelvis/pelvis_tx/value', [0, 1.5], 0.0, [0.4 1.0]); % set final tx bound
    % problem.setStateInfo('/jointset/groundPelvis/pelvis_tx/value', [0, 1]);
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