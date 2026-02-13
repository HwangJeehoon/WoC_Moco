% 일반 : WoC_moco_main(iter, alpha, beta, main cost, p, q, result_dir)
% 이어서 돌리기 : WoC_moco_main(iter, a, b, cost, p, q, result_dir, resume_mode, resume_dir)
% (resume_dir 예: 'et_a001b0_iter300\result_300' -> baseFolder 상대 경로로 넣어야 함)

clc; clear; close all;
WoC_moco_main(300, 0.01, 1,'et',1,0.03,'et_a001b1_iter300') 

% clc; clear; close all;
% WoC_moco_main(300, 0.5,0,'et',1,0.03,'et_a05b0_iter300')
% 
% clc; clear; close all;
% WoC_moco_main(300, 0.1,0,'et',1,0.03,'et_a01b0_iter300')
% 
% clc; clear; close all;
% WoC_moco_main(300, 0.05,0,'et',1,0.03,'et_a005b0_iter300')
% 

clc; clear; close all;
WoC_moco_main(300, 0.01,0,'et',1,0.03,'et_a001b0_iter300to600',true,'et_a001b0_iter300\result_300')
