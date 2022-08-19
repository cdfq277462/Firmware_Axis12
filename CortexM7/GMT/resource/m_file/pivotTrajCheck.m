%function Ptraj = pivotTraj(Pstart,pivot, w, theta, Tf,Ts, TimeScaling)
function trajEND= pivotTrajCheck(Pstart,pivot, w, theta)
% Takes Xstart: The initial PoseRPY of the platform;
%       pivot: The reference point of rotation w.r.t the upper platform frame,
%       w: Axis of rotation at pivot point,
%       r: Vector of a point at the screw axis w.r.t the reference frame,
%       theta: Angle of rotation of the upper frame w.r.t the screw axis.
%       Tf: Total time of the motion in seconds from rest to rest,
%       N: Number of discrete-time points in [0,Tf],
%  TimeScaling: The time-scaling method, where 3 indicates cubic
%               (third-order polynomial) time scaling and 5 indicates 
%               quintic (fifth-order polynomial) time scaling.
% Returns traj: The discretized trajectory as a list of N poses in Z-Y-Z
% parametrization separated in time by Tf/(N-1). 
% This function calculates a trajectory corresponding to the screw motion 
% about a space screw axis.

% Example Input:-----------------------------------
% 
% clear; clc;
% pivot=[5;5;10];
% % Pwork=[20;20;175;pi/2;0;0];
% Pstart=[0;0;155.085;0;0;0];
% w = [0;0;1];
% theta = pi/2;
% Tf = 15;
% Ts = 0.01
% TimeScaling = 3;
%--------------------------------
% N=ceil(Tf/Ts)+1;
% k=0.5; % k=0-->ith=1, k=1--->ith=N;
% ith = 1+k*(N-1)=1+k*ceil(Tf/Ts) ; % ith is any point between 1 and N; 
% Ptraji = pivotTraj(Pstart,pivot, w, theta, Tf,Ts, TimeScaling,ith)
% 
% Output:
% 
% Xwork = poseRPY2SE3(Pwork);

% N = ceil(Tf/Ts) + 1;


Xstart = poseRPY2SE3(Pstart);
r = pivot;

w = w/norm(w);
wx = [0 -w(3) w(2);...
      w(3) 0 -w(1);...
      -w(2) w(1) 0];
rx = [0 -r(3) r(2);...
      r(3) 0 -r(1);...
      -r(2) r(1) 0];
S = [wx rx*w;0 0 0 0];
XstarToend = MatrixExp6(S*theta);
TtrajEND = Xstart*XstarToend;
trajEND=SE3ToPoseRPY(TtrajEND);
end

