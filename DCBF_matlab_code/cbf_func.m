function h = cbf_func(x_i, x_j, d)
    %CBF_FUNC Summary of this function goes here
    %   Communication maintenance
    h = (d^2 - norm(x_i - x_j)^2);
end

