function [CBF_cond_vec, CBF_value_vec, CBF_value_each] = CBF_vec(control_scheme,k0,appr_rang,alpha,p,L,N,N_set,safe_d,total_iter,t_vec,xd,dt,x_vec_1)
    x_vec = zeros(2*N,total_iter);   
    x_vec(:,1) = x_vec_1;
    u_vec = zeros(2*N,total_iter);
    
    % Implementation 1: stacked state boundedness
    y_vec = zeros(N,total_iter);
    c_vec = zeros(N,total_iter);
    a_vec = zeros(2*N,total_iter);
    b_vec = zeros(N,total_iter);
    
    % Implementation 2: stacked state boundedness
    z1_vec = zeros(N,total_iter);
    z2_vec = zeros(N,total_iter);
    p_vec = zeros(N,total_iter);
    q_vec = zeros(N,total_iter);
    
    
    for iter = 1:length(t_vec)-1
        u = zeros(2*N,1);
        ud = zeros(2*N,1);
        u_impl1 = zeros(2*N,1);
        u_impl2 = zeros(2*N,1);
        u_cen = zeros(2*N,1);
        % calculate local control
        for i = 1:N
            udi = nominal_controller(x_vec(:,iter),xd,i,N_set);
            ud(2*i-1:2*i,1) = udi;
        end
        
        % implementation 1
        a = zeros(2*N,1);
        b = zeros(N,1);
        c= zeros(N,1);
        for i = 1:N
            ai = 0;
            bi = 0;
            for j = 1:length(N_set{i})
                x_i = x_vec(2*i-1:2*i,iter);
                idx_j = N_set{i}(j);
                x_j = x_vec(2*idx_j-1:2*idx_j,iter);
                ai = ai - p*exp(-p*cbf_func(x_i, x_j, safe_d))*cbf_grad(x_i, x_j, safe_d);
                bi = bi - alpha/2*(1/4-exp(-p*cbf_func(x_i, x_j, safe_d)));
            end
            ci = (L(i,:)*y_vec(:,iter)+ai'*ud(2*i-1:2*i,1)+bi)/(ai'*ai);
            a(2*i-1:2*i,1) = ai;
            b(i,1) = bi;
            c(i,1) = ci;
            u_impl1_i = ud(2*i-1:2*i,1) - max(0,ci)*ai;
            u_impl1(2*i-1:2*i,1)=  u_impl1_i;
        end
        % update y
        y_vec(:,iter+1) = y_vec(:,iter)-k0*sign(L*c)*dt;
        % record a, b, c
        a_vec(:,iter) = a; b_vec(:,iter) = b; c_vec(:,iter) = c;
        
        
        % implementation 2
        d = zeros(N,1);
        e = zeros(N,1);
        for i = 1:N
            di = a(2*i-1:2*i,1)'*ud(2*i-1:2*i,1)+b(i,1);
            ei = a(2*i-1:2*i,1)'*a(2*i-1:2*i,1);
            pi = z1_vec(i,iter)+di;
            qi = z2_vec(i,iter)+ei;
            d(i,1) = di;
            e(i,1) = ei;
            p_vec(i,iter) = pi;
            q_vec(i,iter) = qi;
            u_impl2(2*i-1:2*i,1) = ud(2*i-1:2*i,1) - max(0,pi/qi)*a(2*i-1:2*i,1);
        end
        % update z1,z2
        dz1 = zeros(N,1);
        dz2 = zeros(N,1);
        for i = 1:N
            pi = p_vec(i,iter);
            qi = q_vec(i,iter);
            for N_iter = 1:length(N_set{i})
                j = N_set{i}(N_iter);
                pj = p_vec(j,iter);
                qj = q_vec(j,iter);
                
                dz1(i,1) = dz1(i,1)+alpha*sign(pj-pi);
                dz2(i,1) = dz2(i,1)+alpha*sign(qj-qi);
            end
        end
        z1_vec(:,iter+1) = z1_vec(:,iter)+dz1*dt;
        z2_vec(:,iter+1) = z2_vec(:,iter)+dz2*dt;
            
        % centralized CBF
        % Equation (6)
        portion = max(0,sum(d)/sum(e));
        u_cen = ud - portion*a;
        
        % choose one input
        switch control_scheme
            case 0
                u = ud;
            case 1
                u = u_impl1;
            case 2
                u = u_impl2;
            case 3
                u = u_cen;
            otherwise
                disp('error value in control_scheme')
        end
        x_vec(:,iter+1) = x_vec(:,iter)+dt*u;
        u_vec(:,iter) = u;
    end

    CBF_cond_vec = zeros(total_iter,1);
    CBF_value_vec = zeros(total_iter,1);
    CBF_value_each = zeros(total_iter,N-1);
    for i = 1:total_iter
        CBF_cond_vec(i) = a_vec(:,i)'*u_vec(:,i)+sum(b_vec(:,i));
        CBF_value_vec(i) = -1/alpha*sum(b_vec(:,i));
        for j = 1:N-1
            x_i = x_vec(3:4,i);
            idx_j = N_set{2}(j);
            x_j = x_vec(2*idx_j-1:2*idx_j,i);
            CBF_value_each(i,j) = cbf_func(x_i, x_j, safe_d);
        end
    end
end