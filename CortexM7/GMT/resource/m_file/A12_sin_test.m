clear;clc;
clear gmt12WS

ToolPoseWorking=[0.0;0.0;0;0.0;0.01;0.0]
disp("resl0")
 [ToolWS, Tstart,TPhome] = gmt12WS(ToolPoseWorking)

% ToolPoseWorking=[0.0;0.0;-0.1;0.0;0;0.0]
% disp("resl1")
%  [ToolWS, Tstart,TPhome] = gmt12WS(ToolPoseWorking)

Ts = 1.1;
v = 3;
LineSpace = 2;
StepRange = 3;ScanRange = 4;
timeScaling = '3';
traj = gmt12SineTrack(Tstart,v,Ts,timeScaling,LineSpace,StepRange,ScanRange,'V');

traj2 = gmt12SineTrack(Tstart,v,Ts,timeScaling,LineSpace,StepRange,ScanRange,'H');

