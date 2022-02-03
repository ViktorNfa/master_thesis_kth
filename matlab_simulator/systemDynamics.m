function xdot = systemDynamics(x, u)

    % System dynamics parameters
    f = zeros(length(x),1);
    g = eye(length(u));

    % Update state vector derivative
    xdot = f+g*u;

end