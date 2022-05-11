clc; close all; clear;
%% initialization
N = 5;
% communication topology
L = [1 -1 0 0 0;
    -1 4 -1 -1 -1;
    0 -1 1 0 0;
    0 -1 0 1 0;
    0 -1 0 0 1];
N_set = {[2],[1,3,4,5],[2],[2],[2]};
% desired formation
%form_rad = 1;
%theta_vec = linspace(0,2*pi,N+1);
%xd1 = [form_rad.*cos(theta_vec(1,2:end))]; 
%xd2 = [form_rad.*sin(theta_vec(1,2:end))];
%xd = zeros(2*N,1); xd(1:2:end) = xd1; xd(2:2:end) = xd2;
xd = [0,2,0,0,0,-2,2,2,2,-2]';

% simulator
dt = 0.001;
T = 30;
t_vec = 0:dt:T;
total_iter = length(t_vec);
x_vec = zeros(2*N,total_iter);

%x_vec(:,1) = rand(2*N,1)+0.5;
%x_vec(:,1) = 0.7*ones(2*N,1);
x_vec(:,1) = 0.7+0.5*rand(2*N,1);
%x_vec(:,1) = 0.1*xd+0.1*rand(2*N,1);

% CBF parameters
alpha = 1;
p = 1;
safe_d = 2;

% HuIL parameters
% Variable to determine if HuIL is active or not 
% (1 is activated/0 is deactivated) as well as the robot it affects
huil = 1;
human_robot = 5;

% HuIL parameters
v_huil = 2;
division = 6;

u_vec = zeros(2*N,total_iter);

% 0 -- nominal controller
% 1 -- implementation 1 using an auxiliary variable
% 2 -- implementation 2 using average tracking
% 3 -- centralized CBF
control_scheme = 1;

% Implementation 1: stacked state boundedness
y_vec = zeros(N,total_iter);
c_vec = zeros(N,total_iter);
k0 = 1;
appr_rang = 0.1;
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
    ud = huilController(ud, huil, human_robot, iter, total_iter, v_huil, division);
    
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
        % To fix ai = 0 error
        if isnan(ci) || isinf(ci)
            ci = 0;
        end
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

% plot the trajectory
figure(1)
set(gcf,'position',[10,10,400,400])
hold on;
colors = [[0, 0.4470, 0.7410];[0.8500, 0.3250, 0.0980];[0.9290, 0.6940, 0.1250];[0.4940, 0.1840, 0.5560];[0.4660, 0.6740, 0.1880]];
for i=1:N
    plot(x_vec(2*i-1,:),x_vec(2*i,:),'Color',colors(i,:),'linewidth',2);
end
% Add initial and final point
for i=1:N
    plot(x_vec(2*i-1,end),x_vec(2*i,end),'-o','MarkerSize',10,'MarkerEdgeColor',colors(i,:));
    plot(x_vec(2*i-1,1),x_vec(2*i,1),'-*','MarkerSize',10,'MarkerEdgeColor',colors(i,:));
end

for i = 1:N
    for iter = 1:length(N_set{i})
        xi_fi = x_vec(2*i-1:2*i,end);
        j = N_set{i}(iter);
        xj_fi = x_vec(2*j-1:2*j,end);
        plot([xi_fi(1) xj_fi(1)],[xi_fi(2) xj_fi(2)],'--k','linewidth',2);
        hold on;
    end
end
legend('Agent 1','Agent 2', 'Agent 3', 'Agent 4', 'Agent 5');
axis equal
box on
xlim([-1.0,4.0])
ylim([-2.0,3.0])


set(gcf,'Units','normalized');
%t_inst = 0.4;
%for i = 1:N
%    [xfig_start,yfig_start] = axescoord2figurecoord(x_vec(2*i-1,t_inst/dt),x_vec(2*i,t_inst/dt));
%    [xfig_fin,yfig_fin] = axescoord2figurecoord(x_vec(2*i-1,t_inst/dt+2),x_vec(2*i,t_inst/dt+2));
%    annotation('arrow',[xfig_start,xfig_fin],[yfig_start,yfig_fin])
%end

% figure(2)
% for time_iter = 1:100:total_iter
%     clf;
%     for i = 1:N
%         for iter = 1:length(N_set{i})
%             xi_fi = x_vec(2*i-1:2*i,time_iter);
%             j = N_set{i}(iter);
%             xj_fi = x_vec(2*j-1:2*j,time_iter);
%             plot([xi_fi(1) xj_fi(1)],[xi_fi(2) xj_fi(2)],'k','linewidth',2);
%             hold on;
%         end
%     end
%     drawnow();
%     xlim([0,10])
%     ylim([0,10])
%     axis equal
% end
% plot CBF condition and CBF evolution
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
figure(3)
plot(t_vec(1:end-1),CBF_cond_vec(1:end-1)')
xlabel('t (sec)')
ylabel('$\dot{h}+\alpha(h)$', 'Interpreter','latex')
figure(4)
set(gcf,'position',[10,10,800,400])
plot(t_vec(1:end-1),CBF_value_vec(1:end-1),'b','linewidth',2)
xlabel('t (sec)')
ylabel('h(x)')
figure(5)
for j = 1:N-1
    plot(t_vec(1:end-1),CBF_value_each(1:end-1,j),'linewidth',2)
    hold on
end
xlabel('t (sec)')
ylabel('h_l(x)')
legend('Edge (1,2)','Edge (2,3)', 'Edge (2,4)', 'Edge (2,5)');

if max(CBF_cond_vec)>=0.01
    disp('Not a working scenario')
else
    disp('Found a working scenario')
end

%% plot CBF condition and CBF value

CBF_cond_vec_cell = cell(4,1);
CBF_value_vec_cell = cell(4,1);
CBF_value_each = cell(4,1);
for i = 1:4
    [CBF_cond_vec_cell{i}, CBF_value_vec_cell{i}, CBF_value_each{i}] = CBF_vec(i-1,k0,appr_rang,alpha,p,L,N,N_set,safe_d,total_iter,t_vec,xd,dt,x_vec(:,1));
end


figure(6)
set(gcf,'position',[100,100,400,400])
plot(t_vec(1:end-1),CBF_value_vec_cell{4}(1:end-1),'linewidth',2)
hold on;
plot(t_vec(1:end-1),CBF_value_vec_cell{2}(1:end-1),'--','linewidth',2)
plot(t_vec(1:end-1),CBF_value_vec_cell{3}(1:end-1),'-.','linewidth',2)
plot(t_vec(1:end-1),CBF_value_vec_cell{1}(1:end-1),':','linewidth',2)


box on;
xlabel('t (sec)')
ylabel('h(t)')
legend('Centralized CBF case', 'Distributed CBF case 1', 'Distributed CBF case 2','Nominal case')

figure(7)
set(gcf,'position',[100,100,400,400])
plot(t_vec(1:end-1),CBF_cond_vec_cell{4}(1:end-1),'linewidth',2)
hold on;
plot(t_vec(1:end-1),CBF_cond_vec_cell{2}(1:end-1),'--','linewidth',2)
plot(t_vec(1:end-1),CBF_cond_vec_cell{3}(1:end-1),'-.','linewidth',2)

box on;
xlabel('t (sec)')
ylabel('$\bar{a}^\top u + \bar{b}$','interpreter','latex')
legend('Centralized CBF case', 'Distributed CBF case 1', 'Distributed CBF case 2')

axes('position',[.3 .3 .4 .4])
plot(t_vec(1:end-1),CBF_cond_vec_cell{4}(1:end-1),'linewidth',2)
hold on;
plot(t_vec(1:end-1),CBF_cond_vec_cell{2}(1:end-1),'--','linewidth',2)
plot(t_vec(1:end-1),CBF_cond_vec_cell{3}(1:end-1),'-.','linewidth',2)
xlim([0,3])
ylim([-3,0.5])