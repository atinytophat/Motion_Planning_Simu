function [T] = FKfunc(th)
    %% T Matrix Computation for Given Thetas
    %Variable - Thetas as seen in RobotDK
    
    
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
    DHM = [0        0       400     inv_joint(1)*th(1);
           -90    25      0       inv_joint(2)*th(2);
           0        560     0       inv_joint(3)*th(3)-90;
           -90    25      515     inv_joint(4)*th(4);
           90     0       0       inv_joint(5)*th(5);
           -90    0       90     inv_joint(6)*th(6)+180];
     
    alpha = DHM(:,1);
    a = DHM(:,2);
    d = DHM(:,3);
    theta = DHM(:,4);
    
    %% 3. Computing overall end-effector transformation
    T = modDH(a,alpha,d,theta);

end

