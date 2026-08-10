function WoC_moco_analysis(AnalySetupPath, opts)
% WoC_moco_PK
%   PointKinematics가 들어 있는 AnalyzeTool setup XML(pkSetupPath)을 실행.
%   opts에 주어지면 model, kinematics, time, resultsDir를 XML 설정 위에 덮어씀.

    arguments
        AnalySetupPath (1,:) char
        opts.modelPath (1,:) char          = ''   % 옵션: 모델 파일 경로
        opts.kinematicsStoPath (1,:) char  = ''   % 옵션: 좌표/kinematics .sto/.mot
        opts.t0 (1,1) double               = -inf % 옵션: 초기 시간 (미지정 시 XML 값 사용)
        opts.tf (1,1) double               =  inf % 옵션: 최종 시간 (미지정 시 XML 값 사용)
        opts.resultsDir (1,:) char         = ''   % 옵션: 결과 폴더
    end

    import org.opensim.modeling.*

    % 1. setup.xml로 AnalyzeTool 생성
    analyzeTool = AnalyzeTool(AnalySetupPath);

    % 2. modelPath가 들어왔으면 XML에 있는 모델 대신 override
    if ~isempty(opts.modelPath)
        model = Model(opts.modelPath);
        model.initSystem();
        analyzeTool.setModel(model);
    end

    % 3. kinematics 파일 override
    if ~isempty(opts.kinematicsStoPath)
        analyzeTool.setCoordinatesFileName(opts.kinematicsStoPath);
    end

    % 4. 시간 구간 override (입력 값이 -inf/inf가 아닐 때만)
    if ~isinf(opts.t0)
        analyzeTool.setInitialTime(opts.t0);
    end
    if ~isinf(opts.tf)
        analyzeTool.setFinalTime(opts.tf);
    end

    % 5. 결과 디렉토리 override
    if ~isempty(opts.resultsDir)
        analyzeTool.setResultsDir(opts.resultsDir);
    end

    % 6. 실행
    analyzeTool.run();
end
