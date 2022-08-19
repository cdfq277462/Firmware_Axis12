clear;clc;
clear F_ToolWS04
Phome = [0;0;155.085;0;0;0];
pivot=[0.0;0;1];
alpha = 0.5*pi;
beta = 0.0 ;  % depends on real situation; 
% ToolPoseWorking=[0.0;0.0;0.0;0.0;0.0;0.0];
base = 0;

ToolPoseWorking=[0.0;2;0;0.0;0.00;0.0]
disp("resl0")
[ToolWS_MinMax, TMtool, Tstart] = F_ToolWS04(Phome,pivot,alpha,beta,ToolPoseWorking,base)

velocity = 1.1;
Ts= 0.5;
TimeScaling = '3';
LineSpace = 0.3;
StepRange = 3;ScanRange = 4;
Id = 'V';
%r = rand(2,1);
r = [0.655098003973841;0.162611735194631];
u = [0.3;0.5];
Thd0 = 0.75;
Thd1 = 0.95;
[trajM, LdataM,traj,Ldata,flag1] = F_SinTrack004_DataPin2_Su(ToolWS_MinMax,TMtool,Tstart,velocity,Ts,TimeScaling,LineSpace,StepRange,ScanRange,Id,r,u,Thd0,Thd1);

%[traj,XYZ] = N_SinTrack04_Su(pivot,alpha,beta,Tstart,v,Ts,LineSpace,StepRange,ScanRange);

% aaa=[1;-2;3;4;-5;-3.3]
% norm(aaa,inf)

