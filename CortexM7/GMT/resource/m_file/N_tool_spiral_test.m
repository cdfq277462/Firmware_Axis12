clear;clc;
 clear gmt12WS


ToolPoseWorking=[0.0;0.0;0;0.0;0;0.0]
disp("resl0")
 [ToolWS, Tstart,TPhome] = gmt12WS(ToolPoseWorking)
% 
% ToolPoseWorking=[0.0;0.0;0;0.0;0.1;0.0]
% disp("resl1")
%  [ToolWS, Tstart,TPhome] = gmt12WS(ToolPoseWorking)
% 
% ToolPoseWorking=[0.0;0.0;-0.1;0.0;0;0.0]
% disp("resl2")
%  [ToolWS, Tstart,TPhome] = gmt12WS(ToolPoseWorking)

ScanRange = 1;  % ScanRange(max) = Zmax;
LineSpacing = 10;    % ScanRange*2/#(circles)velocity=500;   
velocity = 1% 1<=v<=10;
disp("resl3")
traj = gmt12SpiralTrack(Tstart,ScanRange,LineSpacing,velocity);
