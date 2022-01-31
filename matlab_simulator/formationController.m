function u = formationController(L, p, pd)

    % Create extended laplacian
    I = eye(2);
    L_ext = kron(L,I);

    % Compute formation controller
    u = -L*(p-pd);

end