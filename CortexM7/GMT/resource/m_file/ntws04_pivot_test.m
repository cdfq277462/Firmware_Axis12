clear;clc;
clear N_ToolWS04_Su

Phome = [0;0;66.4236;0;0;0];
pivot=[4.0;0.0;2.0];
alpha = pi/2;
beta = 0.0 ;  % depends on real situation; 
% ToolPoseWorking=[0.0;0.0;0.0;0.0;0.0;0.0];
base = 0;

ToolPoseWorking=[0.0;0.0;0;0.0;0.1;0.0]
disp("resl0")
[ToolWS_MinMax, Tstart] = N_ToolWS04_Su(Phome,pivot,alpha,beta,ToolPoseWorking,base)

Tf = 1.5;
Ts = 0.5;
TimeScaling = '3';
[traj,ToolangleWS] = N_ToolPivotTraj04_Su(pivot,alpha, beta, Tstart,Tf,Ts, TimeScaling);

trajXp = traj(:,:,1)
trajXn = traj(:,:,2)
trajYp = traj(:,:,3)
trajYn = traj(:,:,4)
trajZp = traj(:,:,5)
trajZn = traj(:,:,6)