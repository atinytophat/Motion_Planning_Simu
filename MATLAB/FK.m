%% Task A -- Forward Kinematics (FK)

% The robot I selected is a KUKA KR10 R1100 2
% The DH table I got from the Github repository was in modified DH convention


%% T Matrix Computation for Given Thetas
clc
clear

%Variable - Thetas as seen in RobotDK
th1 = 12;        %[-170, 170]
th2 = -50;        %[-190,  45]
th3 = 75;        %[-120, 156]
th4 = -170;        %[-185, 185]
th5 = 30;        %[-120, 120]
th6 = 25;        %[-350, 350]

%Th limits
lim = [-170  170;
       -190   45;
       -120  156;
       -185  185;
       -120  120;
       -350  350];

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

%% 3. Computing overall end-effector transformation
T = modDH(a,alpha,d,theta);
fprintf("T Matrix Computation:\nTheta Set (Degrees):\n      th1 = %d,       th2 = %d,       th3 = %d\n      th4 = %d,       th5 = %d,       th6 = %d\n\n",th1,th2,th3,th4,th5,th6)
disp(T)

%% 2. Deriving individual homogeneous transformation matrices T i-1 -> i

%Setting up Th symbolically
syms th1_sym th2_sym th3_sym th4_sym th5_sym th6_sym; 

%Drawing DH parameters
alpha = DHM(:,1);
a = DHM(:,2);
d = DHM(:,3);

%Defining Th symbols
th_sym = [th1_sym; th2_sym; th3_sym; th4_sym; th5_sym; th6_sym];        
th_offset = [0; 0; -90; 0; 0; 180];
th_inv = [-1; 1; 1; -1; 1; -1];

th_sym1 = th_sym.*th_inv+th_offset;
T_full = eye(4);
for i = 1:length(alpha)
    T_i = modDH(a(i),alpha(i),d(i),th_sym1(i));
    fprintf("T Matrix '%d':\n\n", i)
    disp(T_i)
    T_full = T_full*T_i;
end

%% 4. Validate Against RoboDK

%Set up thetas
% th1 = 91;        %[-170, 170]
% th2 = -51;        %[-190,  45]
% th3 = 106;        %[-120, 156]
% th4 = -171;        %[-185, 185]
% th5 = -61;        %[-120, 120]
% th6 = 257;        %[-350, 350]
% 
% i=1;         %index for th
% 
% ths = [th1,th2,th3,th4,th5,th6];
% 
% for th = lim(i,1):5:lim(i,2)        %Iterates through several thetas withing limit bounds of thi
% 
%     ths(i) = th;
% 
%     alpha = DHM(:,1);
%     a = DHM(:,2);
%     d = DHM(:,3);
%     theta = [inv_joint(1)*ths(1); 
%              inv_joint(2)*ths(2); 
%              inv_joint(3)*ths(3)-90; 
%              inv_joint(4)*ths(4); 
%              inv_joint(5)*ths(5); 
%              inv_joint(6)*ths(6)+180];
% 
%     T = modDH(a,alpha,d,theta);
%     fprintf("T Matrix Computation:\nTheta Set (Degrees):\n      th1 = %d,       th2 = %d,       th3 = %d\n      th4 = %d,       th5 = %d,       th6 = %d\n\n",ths(1),ths(2),ths(3),ths(4),ths(5),ths(6))
%     disp(T)
% 
% end

%Essentially, for any given set of ths and a variable thi withing it, this loop iterates through several values withing thi's bounds and computes the T matrix
%This matrix can then be checked against RoboDK