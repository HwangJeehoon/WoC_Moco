## WoC_Moco

# Overview
WoC에서 나온 최적 토크를 OpenSim Moco에 쏴서 보행 데이터를 얻는 Pipeline을 Matlab에서 구현함

# Structure
- src/        : core MATLAB functions
- scripts/    : Pipeline을 돌리는데 사용되는 코드(main_loop, plotting 등)
- models/     : OpenSim models (.osim)
- inputs/     : initial guess(.sto) 등등 코드가 돌아가는데 필요한 input들