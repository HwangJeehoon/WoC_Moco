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
    metricListTable = readtable(pathMetric, "Sheet", "metric");
    metricDataTable = readtable(pathMetric, "Sheet", "data");
    
    completed_run_list = resultTable(:,1);
    analyzed_run_list = metricDataTable{:,1};
    
    % Update column(metric) list
    curMetricList = metricDataTable.Properties.VariableNames;
    
    for i=1:height(metricListTable)
        curMetric = metricListTable{i, 1}{1};
        
        if ismember(curMetric, curMetricList)
            continue;
        end
        
        metricDataTable = addvars(metricDataTable, NaN(height(metricDataTable), 1), 'NewVariableNames', curMetric);
    end    
    
    % Update new run
    % 아예 처음 보는 Run이면 더미데이터 추가
    for i = 1:1%height(completed_run_list)
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
    
    for i = 1:height(metricDataTable)
        curID = metricDataTable{i, 1}{1};
        curIter = metricDataTable{i, 2};
        date_curID = metricDataTable{i, 3};
        if date_curID == ""
            date_curID = datetime("2000-01-01");
        end
        for idxMetric = 1:height(metricListTable)
            try
                date_cur_metric = metricListTable{idxMetric, 'function_date'};
    
                if date_cur_metric < date_curID
                    continue;
                end
                
                nameCurMetric = metricListTable{idxMetric, 1}{1};
                pathCurrentResult = pathResult + "/" + curID + "/result_" + curIter;
                
                % pathCurrentResult를 전달 -> full인지 half인지 알아야 할지도.
                if ~isKey(metricRegistry, nameCurMetric)
                    warning("등록되지 않은 metric입니다: %s", nameCurMetric);
                    continue;
                end
                metricFunc = metricRegistry(nameCurMetric);
                metric = metricFunc(pathCurrentResult);
                
                metricDataTable.Date(i) = date_today;
                metricDataTable(i, nameCurMetric) = {metric};
            catch
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

