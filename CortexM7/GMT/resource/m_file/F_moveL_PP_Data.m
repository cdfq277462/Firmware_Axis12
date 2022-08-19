
function [traj,Ldata] = F_moveL_PP_Data(ToolWS_MinMax,TMtool,Tstart, Tend, TimeScaling,velocity, Ts, r,u)
%%  Move Point-to-Point via Cartesian Straight Line Path 
%
%---J-P SU 03/24/2022
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%*******************
%  Inputs
%*******************
% clear;clc;close all;
%% Tstart;   % from N_ToolWS04;
% % alpha;
% % beta;
% % pivot;
% TimeScaling = '3'; 
% thetadot = 2*pi/3; 
% Ts = 0.01;   % sample time
% r = rand(2,1);
% u = rand(2,1);
% Compute the corresponding appropriate joint angles via IK:

%----------------------------------

%bbi fixed platform global coordinates
format long

b1=[97.25217186425962; -100.70757204741115; 0];
b2=[38.58922981437988; -134.57663743136464; 0];
b3=[-135.8414016786395; -33.86906538395348; 0];
b4=[-135.8414016786395; 33.86906538395348; 0];
b5=[38.58922981437988; 134.57663743136464; 0];
b6=[97.25217186425962; 100.70757204741115; 0];
bb0=[b1 b2 b3 b4 b5 b6];

p1=[83.40418343188286;  -26.4573654406638; 0];
p2=[-18.78934112711802; -85.4588243542396; 0];
p3=[-64.6148423047648; -59.00145891357579; 0];
p4=[-64.6148423047648;  59.00145891357579; 0];
p5=[-18.78934112711802; 85.4588243542396; 0];
p6=[83.40418343188286; 26.4573654406638; 0];
pp0=[p1 p2 p3 p4 p5 p6];

Lupp=181;
Llow=164;

bb=bb0;
pp=pp0;

%------------------------
TtoolStart = Tstart*TMtool;

ToolTranslx = TtoolStart(1:3,1);
ToolTransly = TtoolStart(1:3,2);
ToolTranslz = TtoolStart(1:3,3);

Pend = SE3ToPoseRPY(Tend);
Pstart = SE3ToPoseRPY(Tstart);
ax = Pstart(1);
ay = Pstart(2);
az = Pstart(3);
%------------------------
y1=ToolWS_MinMax(2,1);
y2=ToolWS_MinMax(2,2);
z1=ToolWS_MinMax(3,1);
z2=ToolWS_MinMax(3,2);
c1=r(1)*y1+(1-r(1))*y2;
c2=r(2)*z1+(1-r(2))*z2;
C = [ax;ay;az]+c1*ToolTransly + c2*ToolTranslz;
cy = C(2);
cz = C(3);
s1=u(1);
s2=u(2);
f=@(y,z) exp(-((y-cy)^2/s1^2+(z-cz)^2/s2^2));
%----------------------------------------------

Tend = poseRPY2SE3(Pend);

Lstart = F_Stewart_IK(Pstart,pp,bb); 
if (min(Lstart))<Llow||(max(Lstart))>Lupp
    error('Tstart is out of working space');
end

Lend = F_Stewart_IK(Pend,pp,bb);   
if (min(Lend))<Llow||(max(Lend))>Lupp
    error('Tend is out of working space');
end
% -----------------------------------
Rstart = Tstart(1:3,1:3);
pstart = Tstart(1:3,4);
Rend = Tend(1:3,1:3);
pend = Tend(1:3,4);

% Compute T,N

if TimeScaling == '3'
    T = (3/2)*(norm((Lstart-Lend),inf)/velocity);
    N = ceil(T/Ts) +1;
    %   T2 = sqrt((6).*(abs(thetastart-thetaend)./thetaddot_limit));    
elseif TimeScaling == '5'
    T = (15/8)*(norm((Lstart-Lend),inf)/velocity);
    N = ceil(T/Ts) +1;
     %  T2 = sqrt((10/sqrt(3)).*(abs(thetastart-thetaend)./thetaddot_limit));
end 

% Construct the trajectory of Line-Segments 
 trajT = zeros(4,4,N);
 traj = zeros(6,N);
 Ldata = zeros(1,N);
for i = 1: N
    if TimeScaling == '3'
        s = CubicTimeScaling(T, Ts * (i - 1));
    elseif TimeScaling == '5'
        s = QuinticTimeScaling(T, Ts * (i - 1));  
    end
    trajT(:,:,i)...
    = [Rstart * MatrixExp3(MatrixLog3(Rstart' * Rend) * s), ...
       pstart + s * (pend - pstart); 0, 0, 0, 1];
   traj(:,i) = SE3ToPoseRPY(trajT(:,:,i));
   yy=traj(2,i);
   zz=traj(3,i);
   Ldata(i) = f(yy,zz);
end
end

