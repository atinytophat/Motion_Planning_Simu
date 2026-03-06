%% Project | Task B: Analytical Inverse Kinematics (IK)

% The robot I selected is a KUKA KR10 R1100 2
% The DH table I got from the Github repository was in modified convention

%%
clc
clear

%% Input: Target End Effector T matrix wanted

T_target = [ -0.34405      0.88533      0.31276       316.99
     0.047126     -0.31639      0.94746       878.87
      0.93777      0.34071     0.067133       756.33
            0            0            0            1];

%% Symbolic Computation of each T i-1 -> i

%DHM Parameters
DHM = [0        0       400;
       -90      25      0;
       0        560     0;
       -90      25      515;
       90       0       0;
       -90      0       90];


%Separating DH parameters
alpha = DHM(:,1);
a = DHM(:,2);
d = DHM(:,3);

%Defining Th     
th_offset = [0; 0; -90; 0; 0; 180];
th_inv = [-1; 1; 1; -1; 1; -1];

%Joint limits
lim = [-170  170;
       -190   45;
       -120  156;
       -185  185;
       -120  120;
       -350  350];

%% G matrix
G = eye(4);

%% H matrix
H = modDH(0,0,d(6),0);

%% IK Solver

%Get Wrist Center position
T_w = T_target*(H^-1);

x_w = T_w(1,4);
y_w = T_w(2,4);
z_w = T_w(3,4);

%To solve for Th1:
th1 = atan2d(y_w, x_w);          % base angle
th1_sols = [th1, th1+180];           % left/right shoulder candidates

th1_sols = mod(th1_sols + 180, 360) - 180;      % wrap to [-180,180)
th1_sols = th1_sols(th1_sols >= -170 & th1_sols <= 170);    % apply joint limits [-170,170]
th1_sols = round(th1_sols,3);
disp(th1_sols)

%To solve for Th2:
%Relevant DH Parameters
d1 = d(1);
a2 = a(2);
a3 = a(3);
d4 = d(4);
a4 = a(4);

th1_used = th1_sols(2);         %Th1 being used to find Th2

x1 = cosd(th1_used)*x_w + sind(th1_used)*y_w;
z1 = z_w;

X = x1 - a2;          
Z = z1 - d1;

R_abs = hypot(X,Z);
R_th = atan2d(-Z,X);

L1 = a3;
L2 = sqrt((a4^2)+(d4^2));

alpha = (((L1^2)+(R_abs^2)-(L2^2))/(2*L1*R_abs));
alpha = acosd(max(min(alpha,1),-1));

th2_a = R_th+alpha;
th2_b = R_th-alpha;

th2_sols = [th2_a, th2_b];
th2_sols = th2_sols(th2_sols >= -190 & th2_sols <= 45);    % apply joint limits [-190,45]
th2_sols = round(th2_sols,3);
disp(th2_sols)

%To solve for Th3:

gamma = (((L1^2)+(L2^2)-(R_abs^2))/(2*L1*L2));
gamma = acosd(max(min(gamma,1),-1));

delta = atan2d(d4,a4);

th3_down   = (180 - gamma) - delta;
th3_up = -(180 - gamma)  -delta;

th3_up   = mod(th3_up + 180, 360) - 180;
th3_down = mod(th3_down + 180, 360) - 180;

th3_sols = [th3_up+90, th3_down+90];

disp(th3_sols)