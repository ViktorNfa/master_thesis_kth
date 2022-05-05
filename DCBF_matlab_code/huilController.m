function u_nom = huilController(u_nom, huil, human_robot, i, max_time_size, v_huil, division)
    % Leave some time at the start and the end to allow the robots to form
    max_time = max_time_size*(1-2/division);
    i = i - max_time_size/division;

    u_huil_x = 0;
    u_huil_y = 0;
    % Simulated HuIL input to move in  a rectangle manner
    if huil > 0
        if i < max_time/4 && i > 0
            u_huil_x = 0;
            u_huil_y = -v_huil;
        elseif i > max_time/4 && i < max_time/2
            u_huil_x = v_huil;
            u_huil_y = 0;
        elseif i > max_time/2 && i < 3*max_time/4
            u_huil_x = 0;
            u_huil_y = v_huil;
        elseif i > 3*max_time/4 && i <= max_time
            u_huil_x = -v_huil;
            u_huil_y = 0;
        else
            u_huil_x = 0;
            u_huil_y = 0;
        end
    end

    u_nom(2*human_robot-1) = u_nom(2*human_robot-1) + u_huil_x;
    u_nom(2*human_robot) = u_nom(2*human_robot) + u_huil_y;

end