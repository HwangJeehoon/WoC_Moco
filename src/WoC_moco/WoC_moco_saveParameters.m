function WoC_moco_saveParameters(filePath, params)
% WoC_moco_saveParameters
%   실행에 사용한 parameter를 Parameter, Value 두 열의 CSV로 저장한다.
%   값은 모두 텍스트로 기록되므로 형식을 지정해 읽는다(자동 추정하면 문자열이
%   NaN이 된다). 벡터는 '[0.4 0.8]' 형태라 str2num으로 되돌릴 수 있다.
%     T = readtable(filePath, 'Format', '%s%s', 'TextType', 'string', 'Delimiter', ',');
%
%   입력:
%     filePath : 저장할 .csv 경로
%     params   : N x 2 cell {이름, 값; ...}. 값은 숫자(스칼라/벡터), 논리값,
%                문자열이며 빈 값은 빈 칸으로 기록한다.

    names = string(params(:, 1));
    values = strings(size(names));
    for k = 1:numel(names)
        values(k) = formatValue(names(k), params{k, 2});
    end
    writetable(table(names, values, 'VariableNames', {'Parameter', 'Value'}), ...
        filePath);
end


function s = formatValue(name, v)
    if isempty(v)
        s = "";
    elseif ischar(v) || isstring(v)
        s = string(v);
    elseif (isnumeric(v) || islogical(v)) && isscalar(v)
        s = string(num2str(v, 10));
    elseif isnumeric(v) || islogical(v)
        s = string(mat2str(v, 10));
    else
        error('WoC_moco_saveParameters: %s 의 값 형식(%s)을 기록할 수 없습니다.', ...
            name, class(v));
    end
end
