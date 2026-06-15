function m = positive_apGRF_impulse(path)
    % GRF file 가져오는 함수 (범용적으로 사용 가능)
    path_moco_result = path + "/moco_result";
    d = dir(path_moco_result);
    d = d(~[d.isdir]);
    names = string({d.name});

    tf = contains(names, "GRF", "IgnoreCase", true);
    matches = fullfile(path_moco_result, cellstr(names(tf))); % full paths as cell array

    [tmp, t] = readSTO_auto(matches{1});

    apGRF_left = t.ground_force_l_vx;
    apGRF_right = t.ground_force_r_vx;
    time = t.time;
    
    apGRF = apGRF_left;
    apGRF_positive = max(apGRF, 0);
    integrated_apGRF_positive = cumtrapz(time, apGRF_positive);
    m_left = integrated_apGRF_positive(end);

    apGRF = apGRF_right;
    apGRF_positive = max(apGRF_right, 0);
    integrated_apGRF_positive = cumtrapz(time, apGRF_positive);
    m_right = integrated_apGRF_positive(end);

    m = [m_left m_right];

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

% If most tokens are numeric -> use readmatrix; otherwise assume header names and use readtable
if mean(isNumericToken) > 0.5
    % numeric data
    M = readmatrix(filename, "NumHeaderLines", lineCount);
    if isempty(M)
        dataTbl = table();
    else
        dataTbl = array2table(M);
    end
else
    % first non-header line is variable names: use detectImportOptions and set DataLines
    opts = detectImportOptions(filename, 'FileType', 'text');
    % set DataLines to start at the first data row (header lines + 1 is varnames)
    % VariableNamesLine is the line that has variable names
    opts.VariableNamesLine = lineCount + 1;
    opts.DataLines = [lineCount+2 Inf];
    dataTbl = readtable(filename, opts);
end
end
