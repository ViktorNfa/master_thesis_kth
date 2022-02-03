function u = huilController(u_nom, huil, human_robot,  i, max_T)

    u = u_nom;
    
    division = 6;
    % Leave some time at the start to allow the robots to form
    max_T = max_T - max_T/division;
    i = i - max_T/division;

    % HuIL parameters
    v_huil = 3;

    if huil > 0
        if i < max_T/4 && i > 0
            u(2*human_robot-1) = 0;
            u(2*human_robot) = -v_huil;
        elseif i > max_T/4 && i < max_T/2
            u(2*human_robot-1) = v_huil;
            u(2*human_robot) = 0;
        elseif i > max_T/2 && i < 3*max_T/4
            u(2*human_robot-1) = 0;
            u(2*human_robot) = v_huil;
        elseif i > 3*max_T/4
            u(2*human_robot-1) = -v_huil;
            u(2*human_robot) = 0;
        end
    end

end