function [finalT] = modDH(a,alpha,d,theta)
%This function computes transformation matrices in modified DH:

%       Rx(alpha)*Tx(a)*Rz(theta)*Tz(d)


T = [cosd(theta(1)) -sind(theta(1)) 0 a(1);
        sind(theta(1))*cosd(alpha(1)) cosd(theta(1))*cosd(alpha(1)) -sind(alpha(1)) d(1)*-sind(alpha(1));
        sind(theta(1))*sind(alpha(1)) cosd(theta(1))*sind(alpha(1)) cosd(alpha(1)) d(1)*cosd(alpha(1));
        0 0 0 1];
if length(a) >= 2
    for i = 2:1:length(a)
    
        Ti = [cosd(theta(i)) -sind(theta(i)) 0 a(i);
            sind(theta(i))*cosd(alpha(i)) cosd(theta(i))*cosd(alpha(i)) -sind(alpha(i)) d(i)*-sind(alpha(i));
            sind(theta(i))*sind(alpha(i)) cosd(theta(i))*sind(alpha(i)) cosd(alpha(i)) d(i)*cosd(alpha(i));
            0 0 0 1];
    
        T = T*Ti;
    end
end

finalT=T;
end

