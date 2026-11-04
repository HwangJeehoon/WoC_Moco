function build_osimFixedEffortPropulsion(openSimDir)
% build_osimFixedEffortPropulsion
%
%   opts.mocoObjectiveMode = 'minSpeedStateFixed' 에 필요한 C++ 플러그인
%   (EffortPropulsionIntegrator, MocoCoordinateAverageSpeedGoal)을 CMake로 빌드한다.
%   결과: build/Release/osimFixedEffortPropulsion.dll (Windows)
%         build/libosimFixedEffortPropulsion.so       (Linux)
%   moco_WoC_loadFixedEffortPropulsionPlugin 이 이 경로에서 라이브러리를 로드한다.
%
%   입력:
%     openSimDir : (선택) OpenSim 설치 폴더 (예: 'C:\OpenSim 4.5.2').
%                  생략하면 MATLAB javaclasspath의 org-opensim-modeling.jar
%                  위치에서 찾는다. 반드시 MATLAB이 사용하는 OpenSim과 같아야 한다.
%
%   요구 사항: CMake, OpenSim 빌드와 같은 C++ 컴파일러
%              (OpenSim 4.5.2 Windows 배포판: Visual Studio 2022, x64).

if nargin < 1 || isempty(openSimDir)
    jcp = javaclasspath('-all');
    jar = jcp(endsWith(jcp, 'org-opensim-modeling.jar'));
    if isempty(jar)
        error('javaclasspath에서 org-opensim-modeling.jar를 찾지 못했습니다. openSimDir를 지정하세요.');
    end
    % <openSimDir>/sdk/Java/org-opensim-modeling.jar
    openSimDir = fileparts(fileparts(fileparts(jar{1})));
end
openSimCMakeDir = fullfile(openSimDir, 'cmake');
if ~isfile(fullfile(openSimCMakeDir, 'OpenSimConfig.cmake'))
    error('OpenSimConfig.cmake가 없습니다: %s', openSimCMakeDir);
end

srcDir   = fileparts(mfilename('fullpath'));
buildDir = fullfile(srcDir, 'build');

runCmd(sprintf('cmake -S "%s" -B "%s" -DOpenSim_DIR="%s"', ...
    srcDir, buildDir, strrep(openSimCMakeDir, '\', '/')));
runCmd(sprintf('cmake --build "%s" --config Release', buildDir));
fprintf('osimFixedEffortPropulsion 빌드 완료: %s\n', buildDir);
end


function runCmd(cmd)
    fprintf('> %s\n', cmd);
    status = system(cmd);
    if status ~= 0
        error('명령 실패 (exit %d): %s', status, cmd);
    end
end
