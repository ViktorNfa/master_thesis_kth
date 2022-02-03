function h = cbf_h(p_i, p_j, d, dir)

    % Function to calculate the CBF safety function
    % Dir 1 corresponds to CM and -1 to OA
    h = dir*(d^2 - norm(p_i - p_j)^2);

end