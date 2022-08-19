clear;clc;
clear F_ToolWS04

Phome = [0;0;155.085;0;0;0];
pivot=[1;0.0;0];
alpha = 0.5*pi;
beta = 0.0 ;  % depends on real situation; 
% ToolPoseWorking=[0.0;0.0;0.0;0.0;0.0;0.0];
base = 0;

ToolPoseWorking=[0.0;0.1;0;0.0;0;0.0]
disp("resl0")
[ToolWS_MinMax, TMtool,Tstart] = F_ToolWS04(Phome,pivot,alpha,beta,ToolPoseWorking,base)

velocity = 1.1;
Ts= 3;
TimeScaling = '3';
ScanRange = 1.5;  % ScanRange(max) = Zmax;
LineSpace = 1;    % ScanRange*2/#(circles)
%r = rand(2,1);
r = [0.276025076998578; 0.679702676853675];
u = [0.3;0.5];
Thd0 = 0.75;
Thd1 = 0.95;
[trajM,LdataM,traj,Ldata] = F_ToolSpiralTrack004_DataPin2_Su(ToolWS_MinMax,TMtool,Tstart,ScanRange,LineSpace, TimeScaling,velocity,Ts,r,u,Thd0,Thd1);

% ToolPoseWorking=[0.0;0.0;-0.1;0.0;0.0;0.0]
% disp("resl1")
% [ToolWS_MinMax, Tstart] = N_ToolWS02(ToolPoseWorking)
% 
% ToolPoseWorking=[0.0;0.0;0;0.0;0.0;0.0]
% disp("resl2")
% [ToolWS_MinMax, Tstart] = N_ToolWS02(ToolPoseWorking)
% 
% ToolPoseWorking=[0.0;0.0;-0.1;0.0;0.0;0.0]
% disp("resl3")
% [ToolWS_MinMax, Tstart] = N_ToolWS02(ToolPoseWorking)
% 
% ToolPoseWorking=[0.0;0.0;0;0.0;0.0;-0.1]
% disp("resl4")
% [ToolWS_MinMax, Tstart] = N_ToolWS02(ToolPoseWorking)