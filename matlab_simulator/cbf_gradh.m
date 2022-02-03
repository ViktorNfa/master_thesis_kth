function grad_h = cbf_gradh(p_i, p_j, dir)

    % Function to calculate the gradient of the CBF safety function
    % Dir 1 corresponds to CM and -1 to OA
    grad_h = dir*(-2*[p_i(1)-p_j(1), p_i(2)-p_j(2)]);
    
end