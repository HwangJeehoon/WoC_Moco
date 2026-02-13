% 일반 : WoC_moco_main(model, iter, alpha, beta, main cost, p, q, result_dir)
% 이어서 돌리기 : WoC_moco_main(model, iter, a, b, cost, p, q, result_dir, resume_mode, resume_dir)
% (resume_dir 예: 'et_a001b0_iter300\result_300' -> baseFolder 상대 경로로 넣어야 함)

clc; clear; close all;
WoC_moco_main('2D_gait_AFO_pc',400, 0.01, 0,'et',1,0.03,'et_a001b0_iter600to1000', true, 'et_a001b0_iter600\result_600') 

clc; clear; close all;
WoC_moco_main('2D_gait_AFO_pc',300, 0.3, 0,'et',1,0.03,'et_a03b0_iter300', false) 


% clc; clear; close all;
% WoC_moco_main(300, 0.01,0,'et',1,0.03,'et_a001b0_iter300to600',true,'et_a001b0_iter300\result_300')
