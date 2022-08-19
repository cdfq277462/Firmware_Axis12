
function traj = N_moveL_PP(Pstart, Pend, TimeScaling,velocity, Ts)
%%  Move Point-to-Point via Cartesian Straight Line Path 
%
%---J-P SU 03/24/2022
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%*******************
%  Inputs
%*******************
% clear;clc;close all;
%% Tstart;   % from N_ToolWS04;
% alpha;
% beta;
% pivot;
% TimeScaling = '3'; 
% thetadot = 2*pi/3; 
% Ts = 0.01;   % sample time
% Compute the corresponding appropriate joint angles via IK:

%----------------------------------

%bbi fixed platform global coordinates
b1=[27.79095418441866; -25.6223118691413; 0];
b2=[8.294095890154637; -36.87882825368669; 0];
b3=[-36.0850500745733; -11.25651638454532; 0];
b4=[-36.0850500745733; 11.25651638454532; 0];
b5=[8.294095890154637; 36.87882825368669; 0];
b6=[27.79095418441866; 25.6223118691413; 0];

bb0=[b1 b2 b3 b4 b5 b6];

%ppi moving platform coordinates
p1=[25.20360654775701; -8.3474976481494; 0];
p2=[-5.372658252550269; -26.00071236142009; 0];
p3=[-19.83094829520674; -17.65321471327068; 0];
p4=[-19.83094829520674; 17.65321471327068; 0];
p5=[-5.372658252550269; 26.00071236142009; 0];
p6=[25.20360654775701; 8.3474976481494; 0];

pp0=[p1 p2 p3 p4 p5 p6];

Lupp=58.5;
Llow=50.5;
% h0
h0 = 21.2;
% g0
g0 = 49.41;
bb=bb0;
pp=pp0;

%-------------------
Tstart = poseRPY2SE3(Pstart);
Tend = poseRPY2SE3(Pend);

[L1,alpha] = N_Stewart_IK(Pstart,pp,bb);    
    if any((alpha.^2 + h0^2 - L1.^2)<0) 
        error('Tstart is out of working space');
    else
        g1 = alpha-sqrt(alpha.^2 + h0^2 - L1.^2);
        Lstart = g1;
    end
[L2,beta] = N_Stewart_IK(Pend,pp,bb);    
    if any((beta.^2 + h0^2 - L2.^2)<0) 
        error('Tend is out of working space');
    else
        g2 = beta-sqrt(beta.^2 + h0^2 - L2.^2);
        Lend = g2;
    end
 %-----------------------------------
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
% [Rstart, pstart] = (Tstart); 
% [Rend, pend] = TransToRp(Tend);

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
   
end
% Plot the trajectory traj
 
% for k = 1:size(trajT,3)    
%     plotH(trajT(:,:,k));
% end  
% xlabel('x');ylabel('y');zlabel('z');
end

