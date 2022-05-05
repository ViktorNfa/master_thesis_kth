function delta_h = cbf_grad(x_i, x_j, d)
    %CBF_FUNC Summary of this function goes here
    %   Communication maintenance
    delta_h = -2*[x_i(1)-x_j(1), x_i(2)-x_j(2)]';
end