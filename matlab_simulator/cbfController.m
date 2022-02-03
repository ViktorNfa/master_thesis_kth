function u_out = cbfController(p, u_nom, cm, oa, d_cm, d_oa, edges)

    % CBF parameters
    alpha = 1;

    % Create CBF constraint matrices
    A_cm = zeros(size(edges,1), length(u_nom));
    b_cm = zeros(size(edges,1),1);
    A_oa = zeros(size(edges,1), length(u_nom));
    b_oa = zeros(size(edges,1),1);
    for i=1:size(edges,1)
        aux_i = edges(i,1);
        aux_j = edges(i,2);

        b_cm(i) = alpha.*cbf_h([p(2*aux_i-1) p(2*aux_i)], [p(2*aux_j-1) p(2*aux_j)], d_cm, 1);
        b_oa(i) = alpha.*cbf_h([p(2*aux_i-1) p(2*aux_i)], [p(2*aux_j-1) p(2*aux_j)], d_oa, -1);

        grad_h_value_cm = transpose(cbf_gradh([p(2*aux_i-1) p(2*aux_i)], [p(2*aux_j-1) p(2*aux_j)], 1));
        grad_h_value_oa = transpose(cbf_gradh([p(2*aux_i-1) p(2*aux_i)], [p(2*aux_j-1) p(2*aux_j)], -1));

        A_cm(i, 2*aux_i-1:2*aux_i) = grad_h_value_cm;
        A_cm(i, 2*aux_j-1:2*aux_j) = -grad_h_value_cm;
        A_oa(i, 2*aux_i-1:2*aux_i) = grad_h_value_oa;
        A_oa(i, 2*aux_j-1:2*aux_j) = -grad_h_value_oa;
    end
       
    % Number of variables
    n = length(u_nom);
    % Create optimization variable
    u = optimvar('u',n);
    % Create problem with objective function
    obj = norm(u-u_nom)^2;
    qp = optimproblem("Objective",obj);
    % Create constraints
    qp.Constraints.cm = A_cm*u.*cm >= -b_cm.*cm;
    qp.Constraints.oa = A_oa*u.*oa >= -b_oa.*oa;
    % Create initial condition
    u0 = zeros(length(u_nom),1);
    u00 = struct('u',u0);
    [u_sol,~] = solve(qp,u00);
    u_out = u_sol.u;

end