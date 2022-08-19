function Ptraj = pivotTraj(Pstart,pivot, w, theta, Tf,Ts, TimeScaling)
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
% Ptraj = pivotTraj(Pstart,pivot, w, theta, Tf,Ts, TimeScaling)
% 
% Output:
% 
% Xwork = poseRPY2SE3(Pwork);

N = ceil(Tf/Ts) + 1;
Xstart = poseRPY2SE3(Pstart);
Xs = eye(4);
r = pivot;

w = w/norm(w);
wx = [0 -w(3) w(2);...
      w(3) 0 -w(1);...
      -w(2) w(1) 0];
rx = [0 -r(3) r(2);...
      r(3) 0 -r(1);...
      -r(2) r(1) 0];
S = [wx rx*w;0 0 0 0];
Xend = MatrixExp6(S*theta);

timegap = Ts;
traj = zeros(4,4, 2*N);
for i = 1: N
    if TimeScaling == '3'
        s = CubicTimeScaling(Tf, timegap * (i - 1));
    else
        s = QuinticTimeScaling(Tf, timegap * (i - 1));
    end
    trajF(:,:,i) = Xstart*(Xs * MatrixExp6(MatrixLog6(TransInv(Xs) * Xend) * s)); 
    PtrajF(:,i) = SE3ToPoseRPY(trajF(:,:,i));
end
for j = 1: N
    trajB(:,:,j) = trajF(:,:,N-j+1);
    PtrajB(:,j) = SE3ToPoseRPY(trajB(:,:,j));
end
Ptraj = [PtrajF PtrajB];
% for k=1:2*N
%     traj(:,:,k) = poseRPY2SE3(Ptraj(:,k));
% end
% 
% % plot Pivot and Rotation Axes------
% t=0:0.01:1;
% u = (1/3)*norm(r)*cos(2*pi*t);
% v = (1/3)*norm(r)*sin(2*pi*t);
% q = zeros(1,length(t));
% 
% % Transformation of Xstart-------
% c = [u;v;q;ones(1,length(t))];
% R = Xstart(1:3,1:3);
% pivotP = Xstart*[pivot;1];
% H = [R pivotP(1:3);0 0 0 1];
% % H = Xstart*H;
% Zcircle=H*c;
% Ycircle=H*[Rotx(pi/2) zeros(3,1);0 0 0 1]*c;
% Xcircle=H*[Roty(pi/2) zeros(3,1);0 0 0 1]*c;

% plot3(pivotP(1),pivotP(2),pivotP(3),'ko','MarkerSize',10,'MarkerFaceColor','c');
% hold on;
% plot3(Zcircle(1,:),Zcircle(2,:),Zcircle(3,:),'b');
% plot3(Ycircle(1,:),Ycircle(2,:),Ycircle(3,:),'g');
% plot3(Xcircle(1,:),Xcircle(2,:),Xcircle(3,:),'r');

% Plot Trajectories ------
% s=10;
% % plotH(Xwork,0.5*s);hold on;
% for k=1:2*N
%     plotH(traj(:,:,k),s);
%     pivotH(:,:,k) = [traj(1:3,1:3,k) pivotP(1:3);0 0 0 1];
%     plotH(pivotH(:,:,k),0.3*s);
% end
% xlabel('x-axis');ylabel('y-axis');zlabel('z-axis');
% axis equal;
end

