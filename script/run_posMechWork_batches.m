function run_posMechWork_batches(resultsDir, batchSize, startBatch, endBatch, matlabExecutable)
% run_posMechWork_batches
%
% gen_posMechWork.m을 독립 MATLAB process 단위로 실행한다. 각 process는
% 최대 batchSize개의 result를 처리한 뒤 종료되므로, OpenSim native memory는
% batch 사이에 Windows가 확실히 회수한다.
%
% 사용법:
%   run_posMechWork_batches
%   run_posMechWork_batches('C:\path\to\results_prime5')
%   run_posMechWork_batches('C:\path\to\results', 500, 4)
%   run_posMechWork_batches('C:\path\to\results', 500, 4, 4)
%
% 입력:
%   resultsDir      : results/<case>/result_# 구조의 최상위 폴더
%   batchSize       : process 하나가 처리할 최대 result 수 (기본 500)
%   startBatch      : 재시작할 1-based batch 번호 (기본 1)
%   endBatch        : 마지막 batch 번호 (기본 마지막 batch)
%   matlabExecutable: MATLAB 실행 파일 또는 PATH상의 'matlab' (기본 'matlab')

    arguments
        resultsDir (1,1) string = ""
        batchSize (1,1) double {mustBePositive, mustBeInteger} = 500
        startBatch (1,1) double {mustBePositive, mustBeInteger} = 1
        endBatch (1,1) double {mustBePositive} = inf
        matlabExecutable (1,1) string = "matlab"
    end

    scriptDir = fileparts(mfilename('fullpath'));
    rootDir   = fileparts(scriptDir);
    if strlength(resultsDir) == 0
        resultsDir = fullfile(rootDir, 'results');
    end
    resultsDir = string(char(java.io.File(char(resultsDir)).getCanonicalPath()));

    assert(isfolder(resultsDir), 'results folder not found: %s', resultsDir);
    generatorPath = fullfile(scriptDir, 'gen_posMechWork.m');
    assert(isfile(generatorPath), 'generator not found: %s', generatorPath);

    resultCount = countResultFolders(resultsDir);
    assert(resultCount > 0, 'No result_* folders found below %s', resultsDir);

    batchCount = ceil(resultCount / batchSize);
    assert(startBatch <= batchCount, ...
        'startBatch (%d) exceeds the final batch (%d).', startBatch, batchCount);
    endBatch = min(endBatch, batchCount);
    assert(endBatch == floor(endBatch) && endBatch >= startBatch, ...
        'endBatch must be an integer from startBatch (%d) through %d.', ...
        startBatch, batchCount);

    fprintf('Processing batches %d through %d of %d (%d result folders; at most %d per batch).\n', ...
        startBatch, endBatch, batchCount, resultCount, batchSize);

    for batchIndex = startBatch:endBatch
        fprintf('\n=== Batch %d / %d ===\n', batchIndex, batchCount);

        command = buildBatchCommand(matlabExecutable, resultsDir, generatorPath, ...
            batchIndex, batchSize);
        [exitCode, commandOutput] = system(command, '-echo');
        if exitCode ~= 0
            error('run_posMechWork_batches:BatchFailed', ...
                ['MATLAB batch %d failed (exit code %d). Resume after resolving ' ...
                 'the error with startBatch=%d.\n%s'], ...
                batchIndex, exitCode, batchIndex, commandOutput);
        end
    end
end

function resultCount = countResultFolders(resultsDir)
    caseEntries = dir(resultsDir);
    caseEntries = caseEntries([caseEntries.isdir]);
    caseEntries = caseEntries(~ismember({caseEntries.name}, {'.', '..'}));

    resultCount = 0;
    for caseIndex = 1:numel(caseEntries)
        caseDir = fullfile(resultsDir, caseEntries(caseIndex).name);
        resultEntries = dir(fullfile(caseDir, 'result_*'));
        resultCount = resultCount + sum([resultEntries.isdir]);
    end
end

function command = buildBatchCommand(matlabExecutable, resultsDir, generatorPath, batchIndex, batchSize)
    % `system` uses cmd.exe on Windows. Environment variables are scoped to
    % this child command, then the child MATLAB exits after one batch.
    resultsDir = replace(resultsDir, '"', '');
    generatorPath = replace(generatorPath, '\', '/');
    matlabExecutable = replace(matlabExecutable, '"', '');

    command = sprintf([ ...
        'set "POS_MECH_WORK_RESULTS_DIR=%s" && ' ...
        'set "POS_MECH_WORK_BATCH_INDEX=%d" && ' ...
        'set "POS_MECH_WORK_BATCH_SIZE=%d" && ' ...
        '"%s" -batch "run(''%s'');"'], ...
        resultsDir, batchIndex, batchSize, matlabExecutable, generatorPath);
end
