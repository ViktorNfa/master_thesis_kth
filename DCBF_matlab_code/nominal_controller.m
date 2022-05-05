function udi = nominal_controller(x,xd,i,N_set)
%NOMINAL_CONTROLLER Summary of this function goes here
xi=x(2*i-1:2*i,1);
xdi=xd(2*i-1:2*i,1);
udi = zeros(2,1);
for iter = 1:length(N_set{i})
    j = N_set{i}(iter);
    xj =x(2*j-1:2*j,1);
    xdj=xd(2*j-1:2*j,1);
    udi = udi+xj-xi-xdj+xdi;
end

end

