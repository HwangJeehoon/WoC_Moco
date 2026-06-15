%%  Metric extraction program
% Sim_run 데이터로부터 metric 데이터를 모아서 결과 파일 
% 만드는 프로그램

%% Main function

function get_metric(pathResult)
    clc;
    close all;
       
    % Import data
    pathCurrent = fileparts(mfilename('fullpath'));
    pathCurrent_ = pathCurrent + "/../../script";
    pathResultRun = pathCurrent_ + "/simulation_queue.xlsx";
    pathMetric = pathCurrent_ + "/../data/metric_run.xlsx";

    if ~canWriteFile(pathMetric)
        msgbox("Excel 파일이 열려 있어서 저장할 수 없습니다. 파일을 닫고 다시 실행하세요.", ...
               "File Permission Error", "error");
        return;
    end
    % 
    % addpath(pathCurrent);
    metricRegistry = makeMetricRegistry();
    
    
    resultTable = readtable(pathResultRun, "Sheet", "completed_queue");
    metricListTable = filterEnabledMetrics(...
        readtable(pathMetric, "Sheet", "metric"));
    metricDataTable = readtable(pathMetric, "Sheet", "data");

    completed_run_list = resultTable(:,1);
    analyzed_run_list = metricDataTable{:,1};

    % 임시로 test할 때, 몇개까지 할건지 결정하는 meta parameter.
    calculate_run_num = 10;   % enable
    % calculate_run_num = 999999999; % disable

    % Update column(metric) list
    % metric sheet의 output_type에 따라
    % scalar -> metric_name
    % vector -> metric_name_left, metric_name_right
    curMetricList = string(metricDataTable.Properties.VariableNames);
    
    for i = 1:height(metricListTable)
    
        nameBaseMetric = getTableString(metricListTable, i, 1);
        outputType = getTableString(metricListTable, i, "output_type");
    
        metricColumns = getMetricColumnNames(nameBaseMetric, outputType);
    
        for j = 1:numel(metricColumns)
            curCol = metricColumns(j);
    
            if ~ismember(curCol, curMetricList)
                metricDataTable = addvars(metricDataTable, NaN(height(metricDataTable), 1), ...
                    'NewVariableNames', char(curCol));
            end
        end
    
        curMetricList = string(metricDataTable.Properties.VariableNames);
    
    end
    
    % Update new run
    % 아예 처음 보는 Run이면 더미데이터 추가
    for i = 1:min(height(completed_run_list), calculate_run_num)
        curID = completed_run_list{i, 1}{1};
        curID_split = split(curID, "-");
        curID_noIter = curID_split{1};
        curID_numIter = curID_split{2};
        
        if ~ismember(curID, analyzed_run_list)
            for idx_iter = 1:str2double(curID_numIter)
                metricDataTable(end+1,  :) = metricDataTable(1, :);
                metricDataTable{end, 'ID'} = {curID};
                metricDataTable(end, 2) = {idx_iter};
                metricDataTable{end, 3} = "2000-01-01";
            end
        end 
    end
    
    % Update metric
    date_today = string(datetime("now", "Format", "yyyy-MM-dd"));

    totalRun = height(completed_run_list);
    lastPrintedID = "";

    fprintf("\n=== Metric update start ===\n");
    fprintf("Total completed runs: %d\n\n", totalRun);

    for i = 1:min(height(metricDataTable), calculate_run_num)
        curID = metricDataTable{i, 1}{1};
        curIter = metricDataTable{i, 2};

        runIdx = find(strcmp(completed_run_list{:,1}, curID), 1);
        if ~strcmp(lastPrintedID, curID)

            if isempty(runIdx)
                fprintf("[?/ %d] Processing ID: %s\n", totalRun, curID);
            else
                fprintf("[%d / %d] Processing ID: %s\n", runIdx, totalRun, curID);
            end

            lastPrintedID = curID;
            drawnow;
        end

        date_curID = metricDataTable{i, 3};
        if date_curID == ""
            date_curID = datetime("2000-01-01");
        end

        for idxMetric = 1:height(metricListTable)
            try
                nameCurMetric = getTableString(metricListTable, idxMetric, 1);
                outputType = getTableString(metricListTable, idxMetric, "output_type");

                metricColumns = getMetricColumnNames(nameCurMetric, outputType);

                date_cur_metric = metricListTable{idxMetric, 'function_date'};

                isMetricMissing = false;
                for idxCol = 1:numel(metricColumns)
                    curCol = metricColumns(idxCol);

                    if isnan(metricDataTable.(char(curCol))(i))
                        isMetricMissing = true;
                        break;
                    end
                end

                if date_cur_metric < date_curID && ~isMetricMissing
                    continue;
                end

                pathCurrentResult = pathResult + "/" + curID + "/result_" + curIter;

                if ~isKey(metricRegistry, char(nameCurMetric))
                    warning("등록되지 않은 metric입니다: %s", nameCurMetric);
                    continue;
                end

                metricFunc = metricRegistry(char(nameCurMetric));
                metric = metricFunc(pathCurrentResult);

                metric = formatMetricOutput(metric, outputType, nameCurMetric);

                metricDataTable.Date(i) = date_today;

                for idxCol = 1:numel(metricColumns)
                    curCol = metricColumns(idxCol);
                    metricDataTable.(char(curCol))(i) = metric(idxCol);
                end

            catch ME
                warning("Metric 계산 중 오류 발생: ID=%s, Iter=%d, Metric=%s\n%s", ...
                    curID, curIter, nameCurMetric, ME.message);
                keyboard
            end

        end
        
    end
    
    % Excel sheet update
    writetable(metricDataTable, pathMetric, "Sheet", "data");
    
    
end

%% Sub-functions
function registry = makeMetricRegistry()

    registry = containers.Map();

    % Clinical metrics.
    registry("velocity_average") = @metrics.velocity_average;
    registry("elapsed_time") = @metrics.elapsed_time;
    registry("stride_length") = @metrics.stride_length;

    % GRF related metrics.
    registry("apGRF_max") = @metrics.apGRF_max;
    registry("propulsion") = @metrics.propulsion;
    registry("propulsion_PT") = @metrics.propulsion_PT;
    registry("propulsion_PD") = @metrics.propulsion_PD;

    % Muscle activation related metrics.
    registry("CMA_gs_PD") = @metrics.CMA_gs_PD;
    registry("CMA_shank_PD") = @metrics.CMA_shank_PD;
    registry("CMA_4set_PD") = @metrics.CMA_4set_PD;
    registry("CMA_limb_PD") = @metrics.CMA_limb_PD;
    
    % Energy related metrics.
    registry("effort") = @metrics.effort;
    registry("work_COM") = @metrics.work_COM;
    registry("work_ankle") = @metrics.work_ankle;


    % Non-dimensional numbers.
    registry("Froude_number") = @metrics.Froude_number;
    registry("Normalized_step") = @metrics.Normalized_step;

    

end

function tf = canWriteFile(filename)
% 파일에 쓸 수 있으면 true, 아니면 false

    tf = true;

    if isfile(filename)
        [fid, msg] = fopen(filename, 'a');

        if fid == -1
            tf = false;
            return;
        else
            fclose(fid);
        end
    else
        % 파일이 아직 없으면, 폴더에 쓸 수 있는지 확인
        folder = fileparts(filename);

        if folder == ""
            folder = pwd;
        end

        testFile = fullfile(folder, "__write_test__.tmp");

        [fid, msg] = fopen(testFile, 'w');

        if fid == -1
            tf = false;
            return;
        else
            fclose(fid);
            delete(testFile);
        end
    end
end

function metricColumns = getMetricColumnNames(nameBaseMetric, outputType)
    
    nameBaseMetric = string(nameBaseMetric);
    outputType = lower(string(outputType));
    
    switch outputType
        case "scalar"
            metricColumns = nameBaseMetric;
    
        case "vector"
            metricColumns = [nameBaseMetric + "_left", ...
                nameBaseMetric + "_right"];
    
        otherwise
            error("알 수 없는 output_type입니다: %s. scalar 또는 vector를 사용하세요.", outputType);
    end

end

function metric = formatMetricOutput(metric, outputType, nameCurMetric)
    
    outputType = lower(string(outputType));
    metric = metric(:).';   % row vector로 정리
    
    switch outputType
    
        case "scalar"
            if isscalar(metric)
                % 그대로 사용
                return;
            end
    
            % 전환 과정에서 [x, x]가 들어오는 경우 허용
            if numel(metric) == 2 && metric(1) == metric(2)
                metric = metric(1);
                return;
            end
    
            error("Metric %s는 scalar로 등록되어 있으므로 scalar 값을 반환해야 합니다. 현재 반환 크기: %s", ...
                nameCurMetric, mat2str(size(metric)));
    
        case "vector"
            if numel(metric) == 2
                return;
            end
    
            % 기존 scalar 함수가 아직 남아있는 경우 임시 호환
            if isscalar(metric)
                metric = [metric, metric];
                return;
            end
    
            error("Metric %s는 vector로 등록되어 있으므로 [left, right] 2개 값을 반환해야 합니다. 현재 반환 크기: %s", ...
                nameCurMetric, mat2str(size(metric)));
    
        otherwise
            error("알 수 없는 output_type입니다: %s. scalar 또는 vector를 사용하세요.", outputType);
    end

end

function value = getTableString(T, rowIdx, col)
    
    raw = T{rowIdx, col};
    
    if iscell(raw)
        value = string(raw{1});
    else
        value = string(raw);
    end

end

function metricListTable = filterEnabledMetrics(metricListTable)
    % metric sheet의 enable column을 기준으로 사용할 metric만 남김
    %
    % enable == 1 : 사용
    % enable == 0 : 무시
    %
    % enable column이 없으면 모든 metric을 enable로 간주
    
    if ~ismember("enable", string(metricListTable.Properties.VariableNames))
        warning("metric sheet에 enable column이 없습니다. 모든 metric을 enable로 간주합니다.");
        return;
    end
    
    enableRaw = metricListTable.enable;
    
    if iscell(enableRaw)
        enableValue = zeros(height(metricListTable), 1);
    
        for i = 1:height(metricListTable)
            curVal = enableRaw{i};
    
            if isnumeric(curVal) || islogical(curVal)
                enableValue(i) = double(curVal);
            else
                enableValue(i) = str2double(string(curVal));
            end
        end
    
    elseif isnumeric(enableRaw) || islogical(enableRaw)
        enableValue = double(enableRaw);
    
    else
        enableValue = str2double(string(enableRaw));
    end
    
    % NaN은 disable로 처리
    enableValue(isnan(enableValue)) = 0;
    
    metricListTable = metricListTable(enableValue == 1, :);

end