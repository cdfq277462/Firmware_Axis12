clear;clc;
clear N_ToolWS04_Su

Phome = [0;0;66.4236;0;0;0];
pivot=[0.0;0;0.0];
alpha = 0;
beta = 0.0 ;  % depends on real situation; 
% ToolPoseWorking=[0.0;0.0;0.0;0.0;0.0;0.0];
base = 0;

ToolPoseWorking=[0.0;0.0;0;0.0;0.00;0.0]
disp("resl0")
[ToolWS_MinMax, Tstart, TMtool] = N_ToolWS04_Su(Phome,pivot,alpha,beta,ToolPoseWorking,base)

Ts = 0.1;
velocity = 1;
LineSpace = 0.2;
StepRange = 2.217;ScanRange = 2.339;

Id='H';
TimeScaling='3';
traj = N_SinTrack04_Su(TMtool,Tstart,velocity,Ts,TimeScaling,LineSpace,StepRange,ScanRange,Id);
%[traj,XYZ] = N_SinTrack04_Su(pivot,alpha,beta,Tstart,v,Ts,LineSpace,StepRange,ScanRange);

% aaa=[1;-2;3;4;-5;-3.3]
% norm(aaa,inf)

