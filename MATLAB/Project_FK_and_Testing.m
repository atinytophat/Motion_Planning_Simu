%% Project | Task A: Forward Kinematics (FK)

% The robot I selected is a KUKA KR10 R1100 2
% The DH table I got from the Github repository was in modified convention

%% T Matrix Computation for Given Thetas
clc
clear

%Variable - Thetas as seen in RobotDK
th1 = 0;        %[-170, 170]
th2 = -10;        %[-190,  45]
th3 = -60.37;        %[-120, 156]
th4 = -143.46;        %[-185, 185]
th5 = 0;        %[-120, 120]
th6 = 330;        %[-350, 350]

%DH parameters
inv_joint = [-1, 1, 1, -1, 1, -1];

%DH Table from repository with th offsets
DHM = [0        0       400     inv_joint(1)*th1;
       -90    25      0       inv_joint(2)*th2;
       0        560     0       inv_joint(3)*th3-90;
       -90    25      515     inv_joint(4)*th4;
       90     0       0       inv_joint(5)*th5;
       -90    0       90     inv_joint(6)*th6+180];
 
alpha = DHM(:,1);
a = DHM(:,2);
d = DHM(:,3);
theta = DHM(:,4);

% Matrix Computation using modified DH
T = modDH(a,alpha,d,theta);
fprintf("T Matrix Computation:\nTheta Set (Degrees):\n      th1 = %d,       th2 = %d,       th3 = %d\n      th4 = %d,       th5 = %d,       th6 = %d\n\n",th1,th2,th3,th4,th5,th6)
disp(T)

% %% Symbolic Computation of each T i-1 -> i
% 
% 
% %Setting up TH symbolically
% syms th1_sym th2_sym th3_sym th4_sym th5_sym th6_sym; 
% 
% %Drawing DH parameters
% alpha = DHM(:,1);
% a = DHM(:,2);
% d = DHM(:,3);
% 
% %Defining Th symbols
% th_sym = [th1_sym; th2_sym; th3_sym; th4_sym; th5_sym; th6_sym];        
% th_offset = [0; 0; -90; 0; 0; 180];
% th_inv = [-1; 1; 1; -1; 1; -1];
% 
% th_sym1 = th_sym.*th_inv+th_offset;
% 
% for i = 1:length(alpha)
%     T_i = modDH(a(i),alpha(i),d(i),th_sym1(i));
%     fprintf("T Matrix '%d':\n\n", i)
%     disp(T_i)
% end

