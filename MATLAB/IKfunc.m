function [ths] = IKfunc(T_target)
DHM = [  0    0   400;
       -90   25     0;
         0  560     0;
       -90   25   515;
        90    0     0;
       -90    0    90];

alpha = DHM(:,1);
a     = DHM(:,2);
d     = DHM(:,3);

% Offsets & Inverses (calculation theta = inv*q + offset)
th_offset = [0; 0; -90; 0; 0; 180];
th_inv    = [-1; 1; 1; -1; 1; -1];

% Joint limits (deg)
lim = [-170  170;
       -190   45;
       -120  156;
       -185  185;
       -120  120;
       -350  350];

%% G & H matrix
G = eye(4);
H = modDH(0,0,d(6),0);

%% To Find Th1,2,3

% Locate Wrist center
T_w = T_target / H;     
x_w = T_w(1,4);
y_w = T_w(2,4);
z_w = T_w(3,4);

% Constants for Arm Geometry
d1 = d(1);
a2 = a(2);
a3 = a(3);
a4 = a(4);
d4 = d(4);

L1 = a3;        % with L0 corresponding to the link from the base to joint 1
L2 = hypot(a4, d4);

delta = atan2d(d4, a4);         %Angle that forms link 3 at joint location

%% Solve for Arm Joints (Thetas 1, 2, & 3)

th1_raw = atan2d(y_w, x_w);
th1_used_sols = [th1_raw, th1_raw+180];         %There's two solutions due to shoulder-left/right

sols123 = [];  % store RoboDK joint angles [q1 q2 q3]

for th1_used = th1_used_sols

    x1 = cosd(th1_used)*x_w + sind(th1_used)*y_w;
    z1 = z_w;

    X = x1 - a2;
    Z = z1 - d1;

    R_abs = hypot(X,Z);
    if R_abs < 1e-9
        continue;
    end

    R_th = atan2d(-Z, X);

    c_alpha = (L1^2 + R_abs^2 - L2^2) / (2*L1*R_abs);           %elbow-up/down angle difference for theta 2
    c_alpha = max(min(c_alpha,1),-1);
    alpha_tri = acosd(c_alpha);

    th2_used_list = [R_th+alpha_tri, R_th-alpha_tri];           %Resulting angles for theta 2

    c_gamma = (L1^2 + L2^2 - R_abs^2) / (2*L1*L2);              %elbow-up/down angle difference for theta 3
    c_gamma = max(min(c_gamma,1),-1);
    gamma = acosd(c_gamma);

    th3_down = (180 - gamma) - delta;
    th3_up   = -(180 - gamma) - delta;
    th3_used_list = [th3_up, th3_down];                         %Resulting angles for theta 3


    sets = [th1_used, th2_used_list(1), th3_used_list(1);       %Final sets of thetas 1, 2, & 3
            th1_used, th2_used_list(2), th3_used_list(2)];


    for k = 1:2
        th1u_k = sets(k,1);
        th2u_k = sets(k,2);
        th3u_k = sets(k,3);

        q1_raw = round((th1u_k - th_offset(1)) / th_inv(1),3);
        q2_raw = round((th2u_k - th_offset(2)) / th_inv(2),3);
        q3_raw = round((th3u_k - th_offset(3)) / th_inv(3),3);

        % Keep values that can be shifted into joint windows
        q1_list = localInWindow(q1_raw, lim(1,1), lim(1,2));
        q2_list = localInWindow(q2_raw, lim(2,1), lim(2,2));
        q3_list = localInWindow(q3_raw, lim(3,1), lim(3,2));
  

        if isempty(q1_list) || isempty(q2_list) || isempty(q3_list)
            continue;
        end

        % Usually one value survives; take the first
        q1 = q1_list(1);
        q2 = q2_list(1);
        q3 = q3_list(1);

        sols123 = [sols123; q1 q2 q3]; %#ok<AGROW>
        end
    end

%% Solve for Wrist Joints (Thetas 4, 5, & 6)


Thfinal = [];   % store RoboDK joint angles [q1 q2 q3 q4 q5 q6]

for i = 1:size(sols123,1)

    q1 = sols123(i,1);
    q2 = sols123(i,2);
    q3 = sols123(i,3);

    th1 = th_inv(1)*q1 + th_offset(1);
    th2 = th_inv(2)*q2 + th_offset(2);
    th3 = th_inv(3)*q3 + th_offset(3);


    T03 = modDH(a(1:3), alpha(1:3), d(1:3), [th1 th2 th3]); %Computing Arm matrix
    T03 = T03*modDH(a(4), alpha(4), d(4), 0);               %Accounting for additional shifts from alpha4 and a4
    T_orient = inv(T03)*(G^-1)*T_target/H;                  %Computing Wrist matrix

    c5 = T_orient(3,3);
    th5_p = acosd(c5);
    th5_n = -acosd(c5);                                     

    th5u_list = [th5_p,th5_n];                              %Resulting angles for theta 5
    
 for th5u = th5u_list
        s5 = sind(th5u);
            
        if abs(s5) < 1e-7                                     %When th5 is close to 0, th 4 and 6 essentially overlap creating a singularity.
                                                              %Making th4 = 0
                                                               %and solving
                                                               %for th6
            th4u = 0;
            th6u = atan2d(T_orient(1,1), T_orient(1,2));
        else
            
            th4u = atan2d(-T_orient(2,3)/s5, (-T_orient(1,3))/s5);          %No singularity occurs, solve for corresponding th4 and th6        
            th6u = atan2d((-T_orient(3,2))/s5, ( T_orient(3,1))/s5);
        end

        q4_raw = (th4u - th_offset(4)) / th_inv(4);
        q5_raw = (th5u - th_offset(5)) / th_inv(5);
        q6_raw = (th6u - th_offset(6)) / th_inv(6);



q5_list = fitToWindow(q5_raw, lim(5,1), lim(5,2));
if isempty(q5_list)
    continue;
end
q5 = q5_list(1);

% --- allow multiple valid theta 4 representations (like theta 6) ---
q4_cands = [q4_raw, q4_raw+360, q4_raw-360];
q4_list = unique(q4_cands(q4_cands >= lim(4,1) & q4_cands <= lim(4,2)));
if isempty(q4_list)
    continue;
end

% --- allow multiple valid theta 6 representations (your existing logic) ---
q6_cands = [q6_raw, q6_raw+360, q6_raw-360];
q6_list = unique(q6_cands(q6_cands >= lim(6,1) & q6_cands <= lim(6,2)));
if isempty(q6_list)
    continue;
end

% Add a row for every (q4, q6) combo that is valid
for q4 = q4_list
    for q6 = q6_list
        Thfinal = [Thfinal; q1 q2 q3 q4 q5 q6]; %#ok<AGROW>
    end
end

    end
end


%% Display All Valid Thetas!
Thfinal = unique(round(Thfinal,3),'rows');
ths = Thfinal;
end