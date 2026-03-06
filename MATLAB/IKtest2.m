%% Project | Task B: Analytical Inverse Kinematics (IK)
% KUKA KR10 R1100-2
% Full script for th1-th6 with singular wrist handling
clc; clear

%% Input: Target End Effector T matrix wanted
T_target = [-0.260074157340216   0.649448048330184   0.714547874675932   898.107526887407;
             0.222124314572465   0.760405965600031  -0.610281538599684  -767.056292183895;
            -0.939692620785908   0                  -0.342020143325669   304.409569427526;
             0                   0                   0                    1];

%% DHM Parameters (Modified DH): [alpha(deg), a(mm), d(mm)]
DHM = [  0    0   400;
       -90   25     0;
         0  560     0;
       -90   25   515;
        90    0     0;
       -90    0    90];

alpha = DHM(:,1);
a     = DHM(:,2);
d     = DHM(:,3);

% Offsets & Inverses (calculation theta_used = inv*q + offset)
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
H = modDH(0,0,d(6),0);   % tool translation only

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

L1 = a3;
L2 = hypot(a4, d4);
delta = atan2d(d4, a4);

%% Solve for Arm Joints (Thetas 1, 2, & 3)

th1_raw = atan2d(y_w, x_w);
th1_used_sols = [th1_raw, th1_raw+180];   % shoulder left/right

sols123 = [];   % store RoboDK joint angles [q1 q2 q3]

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

    if R_abs > (L1+L2)
continue;
    end
    % theta 2
    c_alpha = (L1^2 + R_abs^2 - L2^2) / (2*L1*R_abs);
    c_alpha = max(min(c_alpha,1),-1);
    alpha_tri = acosd(c_alpha);
    th2_used_list = [R_th+alpha_tri, R_th-alpha_tri];

    % theta 3
    c_gamma = (L1^2 + L2^2 - R_abs^2) / (2*L1*L2);
    c_gamma = max(min(c_gamma,1),-1);
    gamma = acosd(c_gamma);

    th3_down = (180 - gamma) - delta;
    th3_up   = -(180 - gamma) - delta;
    th3_used_list = [th3_up, th3_down];
  
    % Pair corresponding branches
    sets = [th1_used, th2_used_list(1), th3_used_list(1);
            th1_used, th2_used_list(2), th3_used_list(2)];

    for k = 1:2
        th1u_k = sets(k,1);
        th2u_k = sets(k,2);
        th3u_k = sets(k,3);

        q1_raw = (th1u_k - th_offset(1)) / th_inv(1);
        q2_raw = (th2u_k - th_offset(2)) / th_inv(2);
        q3_raw = (th3u_k - th_offset(3)) / th_inv(3);

        q1_list = localInWindow(q1_raw, lim(1,1), lim(1,2));
        q2_list = localInWindow(q2_raw, lim(2,1), lim(2,2));
        q3_list = localInWindow(q3_raw, lim(3,1), lim(3,2));

        if isempty(q1_list) || isempty(q2_list) || isempty(q3_list)
            continue;
        end

        q1 = q1_list(1);
        q2 = q2_list(1);
        q3 = q3_list(1);

        sols123 = [sols123; q1 q2 q3]; %#ok<AGROW>
    end
end

sols123 = unique(round(sols123,3),'rows');
disp('Valid [th1 th2 th3] in RoboDK joint angles (deg):');
disp(sols123);

%% Solve for Wrist Joints (Thetas 4, 5, & 6)

Thfinal = [];   % store RoboDK joint angles [q1 q2 q3 q4 q5 q6]

for i = 1:size(sols123,1)

    q1 = sols123(i,1);
    q2 = sols123(i,2);
    q3 = sols123(i,3);

    % Convert q1..q3 -> used theta
    th1 = th_inv(1)*q1 + th_offset(1);
    th2 = th_inv(2)*q2 + th_offset(2);
    th3 = th_inv(3)*q3 + th_offset(3);

    % Arm matrix to wrist-center frame
    T03 = modDH(a(1:3), alpha(1:3), d(1:3), [th1 th2 th3]);
    T03 = T03 * modDH(a(4), alpha(4), d(4), 0);   % include frame-4 shift
    T_orient = inv(T03) * (G^-1) * T_target / H;  % wrist transform

    % theta 5 from your convention
    c5 = max(min(T_orient(3,3),1),-1);
    th5_p = acosd(c5);
    th5_n = -acosd(c5);
    th5u_list = [th5_p, th5_n];

    for th5u = th5u_list
        s5 = sind(th5u);

        % q5 first
        q5_raw = (th5u - th_offset(5)) / th_inv(5);
        q5_list = fitToWindow(q5_raw, lim(5,1), lim(5,2));
        if isempty(q5_list)
            continue;
        end
        q5 = q5_list(1);

        if abs(s5) < 1e-10
            % -------------------------------------------------
            % WRIST SINGULARITY:
            % only phi = th4u + th6u is observable for your convention
            % choose the valid pair with the SMALLEST |q4|,
            % and if tied, the smallest |q6|
            % -------------------------------------------------
            phi = atan2d(-T_orient(1,2), T_orient(1,1));

            validPairs = [];

            % search q4 across its valid window
            for q4 = lim(4,1):0.1:lim(4,2)
                th4u = th_inv(4)*q4 + th_offset(4);
                th6u = phi - th4u;

                q6_raw = (th6u - th_offset(6)) / th_inv(6);
                q6_cands = [q6_raw, q6_raw+360, q6_raw-360];
                q6_list = unique(q6_cands(q6_cands >= lim(6,1) & q6_cands <= lim(6,2)));

                for q6 = q6_list
                    validPairs = [validPairs; q4 q6]; %#ok<AGROW>
                end
            end

            if isempty(validPairs)
                continue;
            end

            % pick smallest |q4|, then smallest |q6|
            score = [abs(validPairs(:,1)), abs(validPairs(:,2))];
            idx = minrows(score);
            q4 = validPairs(idx,1);
            q6 = validPairs(idx,2);

            Thfinal = [Thfinal; q1 q2 q3 q4 q5 q6]; %#ok<AGROW>

        else
            % -------------------------------------------------
            % NON-SINGULAR CASE: your working formulas
            % -------------------------------------------------
            th4u = atan2d(-T_orient(2,3)/s5, (-T_orient(1,3))/s5);
            th6u = atan2d((-T_orient(3,2))/s5, ( T_orient(3,1))/s5);

            q4_raw = (th4u - th_offset(4)) / th_inv(4);
            q6_raw = (th6u - th_offset(6)) / th_inv(6);

            % allow multiple valid theta 4 representations
            q4_cands = [q4_raw, q4_raw+360, q4_raw-360];
            q4_list = unique(q4_cands(q4_cands >= lim(4,1) & q4_cands <= lim(4,2)));
            if isempty(q4_list)
                continue;
            end

            % allow multiple valid theta 6 representations
            q6_cands = [q6_raw, q6_raw+360, q6_raw-360];
            q6_list = unique(q6_cands(q6_cands >= lim(6,1) & q6_cands <= lim(6,2)));
            if isempty(q6_list)
                continue;
            end

            for q4 = q4_list
                for q6 = q6_list
                    Thfinal = [Thfinal; q1 q2 q3 q4 q5 q6]; %#ok<AGROW>
                end
            end
        end
    end
end

%% Display All Valid Thetas
Thfinal = unique(round(Thfinal,3),'rows');
disp('All IK solutions [th1 th2 th3 th4 th5 th6] (RoboDK deg):');
disp(Thfinal);

%% Unique physical IK solutions (ignore ±360 duplicates)
% Use wrap-to-180 ONLY for comparison, not for filtering or solving
Th_compare = mod(Thfinal + 180, 360) - 180;

% Round for numerical stability before unique
Th_compare = round(Th_compare, 3);

% Find unique physical rows
[~, idx_unique] = unique(Th_compare, 'rows', 'stable');
Thfinal_unique = Thfinal(idx_unique,:);

disp('Unique physical IK solutions [th1 th2 th3 th4 th5 th6] (RoboDK deg):');
disp(Thfinal_unique);

fprintf('Total valid numeric solution rows found: %d\n', size(Thfinal,1));
fprintf('Total unique physical solutions found: %d\n', size(Thfinal_unique,1));

%% -------- helper functions --------
function vals = fitToWindow(ang, lo, hi)
    cands = [ang, ang+360, ang-360];
    vals = cands(cands >= lo & cands <= hi);
end

function vals = localInWindow(ang, lo, hi)
    cands = [ang, ang+360, ang-360];
    mask  = (cands >= lo) & (cands <= hi);
    vals  = cands(mask);
end

function idx = minrows(A)
    [~, order] = sortrows(A);
    idx = order(1);
end