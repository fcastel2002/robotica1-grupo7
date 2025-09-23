function Ti = invHomog(T)
% Inversa eficiente para matrices homogéneas [R p; 0 0 0 1]
    R = T(1:3,1:3); p = T(1:3,4);
    Rt = R.'; 
    Ti = [Rt, -Rt*p; 0 0 0 1];
end
