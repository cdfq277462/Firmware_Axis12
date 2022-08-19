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

ScanRange = 4.09;  % ScanRange(max) = Zmax;
LineSpacing = 1.36;    % ScanRange*2/#(circles)
velocity=500;   % 1<=v<=10;
[traj,XYZ] = N_ToolSpiralTrack04_Su(pivot,alpha,beta,Tstart,ScanRange,LineSpacing,velocity)

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