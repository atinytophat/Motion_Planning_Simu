%% Thorough Cross-Validation

%Th limits
lim = [-170  170;
       -190   45;
       -120  156;
       -185  185;
       -120  120;
       -350  350];


%DH parameters
inv_joint = [-1, 1, 1, -1, 1, -1];



%%
th1 = 40.5;        %[-170, 170]
th2 = 0;        %[-190,  45]
th3 = 10;        %[-120, 156]
th4 = 0;        %[-185, 185]
th5 = 10;        %[-120, 120]
th6 = 0;        %[-350, 350]

ths = [th1;th2;th3;th4;th5;th6];

i=1;         %index for th

%%
for th = lim(i,1):5:lim(i,2)        %Iterates through several thetas withing limit bounds of thi
    ths(i) = th;

    T_test = FKfunc(ths)
    th_test = IKfunc(T_test)



end



T = FKfunc(th)