function moco_WoC_getResult(moco_WoC_Solution, outputDir, opts)
% moco_WoC_getResult
%
%   moco_WoC_loop에서 얻은 MocoTrajectory를 받아
%   1) periodic full stride trajectory 생성
%   2) kinematics(Full and half stride)를 .sto로 저장
%   3) GRF를 .sto로 저장
%   4) 나중에 그래프를 그릴 수 있도록 skeleton(TODO) 위치를 확보하는 함수.
%
% 입력:
%   moco_WoC_Solution : MocoTrajectory (moco_WoC_loop 결과)
%   outputDir         : 결과를 저장할 디렉터리 경로 (루프마다 서로 다르게 지정)
%   opts              : (선택) struct
%       .modelPath : GRF 생성에 사용할 osim 모델 경로
%                    (default = '2D_gait_AFO.osim')
%       .prefix    : 출력 파일 이름 prefix
%                    (default = 'moco_WoC_Solution')
%
% 출력:
%               : kinematics(Full and half stride)를 .sto로 저장
%               : GRF를 .sto로 저장
%               (그래프 관련 결과는 TODO 영역에서 추후 확장 가능)

    import org.opensim.modeling.*

    if nargin < 2
        error('Usage: moco_WoC_getResult(moco_WoC_Solution, outputDir, [opts])');
    end
    if nargin < 3
        opts = struct();
    end

    % 옵션 기본값
    modelPath = getOpt(opts, 'modelPath', '2D_gait_AFO.osim');
    prefix    = getOpt(opts, 'prefix',    'moco_WoC_Solution');

    % 출력 디렉터리 생성
    if ~exist(outputDir, 'dir')
        mkdir(outputDir);
    end

    %----------------------------------------------------------
    % 1. 결과 저장
    %----------------------------------------------------------
    moco_WoC_Solution_fullstride = opensimMoco.createPeriodicTrajectory(moco_WoC_Solution);
    moco_WoC_Solution_halfstride = moco_WoC_Solution;

    % .sto 경로 설정
    kinStoName = [prefix '_kinematics.sto'];
    kinStoPath = fullfile(outputDir, kinStoName);

    kinHalfStoName = [prefix '_kinematics_half.sto'];
    kinHalfStoPath = fullfile(outputDir, kinHalfStoName);

    % Write
    moco_WoC_Solution_fullstride.write(kinStoPath); % Full stride
    moco_WoC_Solution_halfstride.write(kinHalfStoPath); % Half stride for initial guess

    %----------------------------------------------------------
    % 2. GRF 추출 및 .sto 저장
    %----------------------------------------------------------

    % 모델 로드
    model = Model(modelPath);
    model.initSystem();

    % contact set 정의 (오른발/왼발)
    contact_r = StdVectorString();
    contact_l = StdVectorString();
    contact_r.add('contactHeel_r');
    contact_r.add('contactFront_r');
    contact_l.add('contactHeel_l');
    contact_l.add('contactFront_l');

    % fullStride & model 기반 GRF 테이블 생성
    externalForcesTableFlat = opensimMoco.createExternalLoadsTableForGait( ...
        model, moco_WoC_Solution_fullstride, contact_r, contact_l);

    % GRF .sto 경로 설정
    grfStoName = [prefix '_GRF.sto'];
    grfStoPath = fullfile(outputDir, grfStoName);

    % write GRF sto
    STOFileAdapter.write(externalForcesTableFlat, grfStoPath);

    %----------------------------------------------------------
    % 3. TODO: kinematics / GRF 기반 그래프 그리기
    %----------------------------------------------------------
    % 여기서부터는 나중에 원하는 그래프들을 추가하는 영역.
    % 예시 아이디어 (현재는 TODO로 남김):
    %
    %   - 오른발/왼발 vertical GRF vs time
    %   - AFO_right control vs time (solution에서 직접 추출 or control sto를 읽어와서)
    %   - COM trajectory (pelvis_tx, ty 등) vs time
    %
    % 아래는 skeleton 형태로만 남겨둠:
    %
    % try
    %     % TODO: kinStoPath, grfStoPath를 읽어서
    %     %       특정 변수(time, GRF_y, AFO torque 등)를 plot하는 코드 작성
    %     % 예:
    %     %   kin = WoC_moco_readSTO(kinStoPath);
    %     %   grf = WoC_moco_readSTO(grfStoPath);
    %     %
    %     %   figure; plot(kin.time, kin.pelvis_tx); title('Pelvis TX');
    %     %   figure; plot(grf.time, grf.ground_force_vy_r); title('vGRF Right');
    % catch ME
    %     warning('Plotting section (TODO) raised an error: %s', ME.message);
    % end
end


%% ---- 옵션 읽기용 헬퍼 ----
function val = getOpt(s, field, defaultVal)
    if isfield(s, field) && ~isempty(s.(field))
        val = s.(field);
    else
        val = defaultVal;
    end
end
