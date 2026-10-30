function moco_WoC_loadFixedEffortPropulsionPlugin()
% moco_WoC_loadFixedEffortPropulsionPlugin
%
%   EffortPropulsionIntegrator / MocoCoordinateAverageSpeedGoal 을 정의한 C++ 플러그인
%   (plugins/osimFixedEffortPropulsion)을 현재 MATLAB 세션의 OpenSim에 로드한다.
%   타입 등록은 라이브러리 로드 시 이루어지므로, component/goal을 만들기
%   전에 호출해야 한다. 같은 세션에서 반복 호출해도 한 번만 로드한다.
%
%   라이브러리가 없으면 먼저 빌드한다:
%     run(fullfile(<repo>, 'plugins', 'osimFixedEffortPropulsion', 'build_osimFixedEffortPropulsion.m'))

    persistent loaded
    if ~isempty(loaded)
        return;
    end

    import org.opensim.modeling.*

    pluginDir = fullfile(fileparts(mfilename('fullpath')), '..', '..', ...
                         'plugins', 'osimFixedEffortPropulsion');
    if ispc
        libPath = fullfile(pluginDir, 'build', 'Release', 'osimFixedEffortPropulsion.dll');
    else
        libPath = fullfile(pluginDir, 'build', 'libosimFixedEffortPropulsion.so');
    end
    if ~isfile(libPath)
        error(['osimFixedEffortPropulsion 플러그인이 빌드되지 않았습니다: %s\n', ...
               '빌드: run(''%s'')'], libPath, ...
               fullfile(pluginDir, 'build_osimFixedEffortPropulsion.m'));
    end

    if ~opensimCommon.LoadOpenSimLibraryExact(libPath)
        error('osimFixedEffortPropulsion 플러그인 로드 실패: %s', libPath);
    end
    loaded = true;
end
