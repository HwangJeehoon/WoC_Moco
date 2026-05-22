function m = Froude_number(path)
    %% Leg length Reader
    % path로부터 ID 가져오는 부분 (복사 사용 가능)
    parts_path = strsplit(path, '/');
    parts_path = parts_path(~cellfun('isempty', parts_path));
    run_id = parts_path{end - 1};
    
    % Run 정보 가져오는 부분
    runInfo = readtable("simulation_queue.xlsx", "Sheet", 'completed_queue');
    idCol = string(runInfo.ID);
    rowIdx = (idCol == run_id);
    rowTable = runInfo(rowIdx, :);
    
    model_name = rowTable{1, 3};

    model_path= "../models/" + model_name{1};
    
    % model 정보로부터 leg length 추출
    import matlab.io.xml.xpath.*
    e = Evaluator();

    xml_femur_l = "//Body[@name='femur_l']/mass_center";
    xml_tibia_l = "//Body[@name='tibia_l']/mass_center";
    
    nodes = evaluate(e, xml_femur_l, model_path, EvalResultType.NodeSet);
    values = string({nodes.TextContent});
    nums = sscanf(char(values), '%f')';
    length_femur = 2 * abs(nums(2));
    
    nodes = evaluate(e, xml_tibia_l, model_path, EvalResultType.NodeSet);
    values = string({nodes.TextContent});
    nums = sscanf(char(values), '%f')';
    length_tibia = 2 * abs(nums(2));
    
    length_leg = length_femur + length_tibia;

    %% Run data reader
    path_moco_result = path + "/moco_result";
    d = dir(path_moco_result);
    d = d(~[d.isdir]);
    names = string({d.name});

    tf = contains(names, "kinematics", "IgnoreCase", true);
    matches = fullfile(path_moco_result, cellstr(names(tf))); % full paths as cell array

    [tmp, t] = readSTO_auto(matches{1});

    time = t.time;
    elapsed_time = time(end) - time(1);   

    distance = t.x_jointset_groundPelvis_pelvis_tx_value;
    vel_average = (distance(end) - distance(1)) / elapsed_time;
    
    m = vel_average^2 / 9.8 / length_leg;


end

function [headerLines, dataTbl] = readSTO_auto(filename)
% readSTO_auto Read .sto file; avoid NaN in first row by auto-detecting
% whether the first non-header line is variable names or numeric data.
% Returns headerLines (cell) and dataTbl (table).

if nargin<1 || isempty(filename)
    error("Filename required");
end

fid = fopen(filename,"r");
if fid==-1
    error("Cannot open file: %s", filename);
end

% read header until 'endheader'
headerLines = {};
lineCount = 0;
found = false;
while ~feof(fid)
    tline = fgetl(fid);
    lineCount = lineCount + 1;
    headerLines{end+1,1} = tline; %#ok<AGROW>
    if contains(tline,"endheader","IgnoreCase",true)
        found = true;
        break;
    end
end

if ~found
    fclose(fid);
    error("No 'endheader' line found in file: %s", filename);
end

% peek the first data line (without moving file pointer permanently)
% reopen file and skip header lines then read one line
fclose(fid);
fid = fopen(filename,"r");
for k = 1:lineCount
    fgetl(fid);
end
firstDataLine = fgetl(fid);
fclose(fid);

if isempty(firstDataLine)
    % no data
    dataTbl = table();
    return
end

% Normalize delimiters to space for quick token check
% Treat comma, tab as delimiter too
tokens = regexp(firstDataLine, '[^\s,]+', 'match'); %#ok<REGEXP>
isNumericToken = false(1,numel(tokens));
for i=1:numel(tokens)
    % consider numeric if str2double gives non-NaN
    v = str2double(tokens{i});
    if ~isnan(v)
        isNumericToken(i) = true;
    end
end

% helper: sanitize variable names in table (replace '/' -> '_' and make valid & unique)
    function T = sanitizeVarNames(T)
        if isempty(T) || isempty(T.Properties.VariableNames)
            return
        end
        vn = T.Properties.VariableNames;
        % replace slashes with underscores
        vn = strrep(vn, '/', '_');
        % make valid MATLAB names (handles spaces, leading digits, etc.)
        vn = matlab.lang.makeValidName(vn);
        % ensure uniqueness (R2020b+). If unavailable, fallback to simple uniquefy.
        try
            vn = matlab.lang.makeUniqueStrings(vn);
        catch
            % basic uniqueness fallback
            [uniqueNames, ~, idx] = unique(vn, 'stable');
            counts = accumarray(idx, 1);
            vnNew = vn;
            for ii = 1:numel(uniqueNames)
                dupIdx = find(strcmp(vn, uniqueNames{ii}));
                if numel(dupIdx) > 1
                    for kdup = 1:numel(dupIdx)
                        if kdup == 1
                            vnNew{dupIdx(kdup)} = uniqueNames{ii};
                        else
                            vnNew{dupIdx(kdup)} = sprintf('%s_%d', uniqueNames{ii}, kdup);
                        end
                    end
                end
            end
            vn = vnNew;
        end
        T.Properties.VariableNames = vn;
    end

% If most tokens are numeric -> use readmatrix; otherwise assume header names and use readtable
if mean(isNumericToken) > 0.5
    % numeric data
    M = readmatrix(filename, "NumHeaderLines", lineCount);
    if isempty(M)
        dataTbl = table();
    else
        dataTbl = array2table(M);
        % Optionally sanitize variable names created by array2table (Var1, Var2, ...)
        dataTbl = sanitizeVarNames(dataTbl);
    end
else
    % first non-header line is variable names: use detectImportOptions and set DataLines
    opts = detectImportOptions(filename, 'FileType', 'text');
    % preserve original variable name strings so we can sanitize them ourselves
    opts.VariableNamingRule = 'preserve';
    % set VariableNamesLine and DataLines appropriately
    opts.VariableNamesLine = lineCount + 1;
    opts.DataLines = [lineCount+2 Inf];
    dataTbl = readtable(filename, opts);
    % sanitize variable names (replace '/' -> '_' etc.)
    dataTbl = sanitizeVarNames(dataTbl);
end
end
