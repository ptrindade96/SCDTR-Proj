function [d, cost] = primal_solve(node, rho)
    d_best = [-1,-1]';
    cost_best = 1000000; %large number
    sol_unconstrained = 1;
    sol_boundary_linear = 1;
    sol_boundary_0 = 1;
    sol_boundary_100 = 1;
    sol_linear_0 = 1;
    sol_linear_100 = 1;
    
    if(node.index == 1)
        index = 1;
        n_index = 2;
    else
        index = 2;
        n_index = 1;
    end
    
    B = rho*eye(2);
    B(index,index) = B(index,index) + node.Q;
    z = -(rho*node.d_av - node.y);
    z(index) = z(index) + node.c;
    
    
    %unconstrained minimum
    d_u = -inv(B)*z
    sol_unconstrained = check_feasibility(node,d_u);
    if sol_unconstrained
        cost_unconstrained = evaluate_cost(node, d_u,rho)
        if cost_unconstrained < cost_best
           d = d_u;
           cost = cost_unconstrained;
           return  %IF UNCONSTRAINED SOLUTION EXISTS, THEN IT IS OPTIMAL
                   %NO NEED TO COMPUTE THE OTHER
        end;
    end;
    
    disp(node.y)
    %compute minimum constrained to linear boundary
    d_bl(index) = -(node.c*node.k(2)^2 + node.k(2)^2*node.y(node.index) - node.d_av(index)*node.k(2)^2*rho - node.L*node.k(1)*rho + node.k(1)*node.o*rho - node.k(1)*node.k(2)*node.y(n_index) + node.d_av(n_index)*node.k(1)*node.k(2)*rho)/(node.Q*node.k(2)^2 + node.k(1)^2*rho + node.k(2)^2*rho);
    d_bl(n_index) = (node.k(2)*(node.L - node.o + (node.k(2)*(node.y(n_index) - node.d_av(n_index)*rho))/rho + (node.k(1)*(node.c + node.y(index) - node.d_av(index)*rho))/(node.Q + rho)))/(rho*(node.k(1)^2/(node.Q + rho) + node.k(2)^2/rho)) - (node.y(n_index) - node.d_av(n_index)*rho)/rho;
    %check feasibility of minimum constrained to linear boundary
    d_bl = d_bl'
    disp(node.index)
    sol_boundary_linear = check_feasibility(node, d_bl);
    % compute cost and if best store new optimum
    if sol_boundary_linear 
        cost_boundary_linear = evaluate_cost(node, d_bl,rho)
        if cost_boundary_linear < cost_best
           d_best = d_bl;
           cost_best = cost_boundary_linear;
        end;
    end;
    
    
    %compute minimum constrained to 0 boundary
    d_b0(n_index) = -(node.y(2) - node.d_av(n_index)*rho)/rho;
    d_b0(node.index) = 0;
    d_b0 = d_b0'
    %check feasibility of minimum constrained to 0 boundary
    sol_boundary_0 = check_feasibility(node, d_b0);
    % compute cost and if best store new optimum
    if sol_boundary_0 
        cost_boundary_0 = evaluate_cost(node, d_b0,rho);
        if cost_boundary_0 < cost_best
           d_best = d_b0;
           cost_best = cost_boundary_0;
        end;
    end;
    
    
    %compute minimum constrained to 100 boundary
    d_b1(n_index) = -(node.y(2) - node.d_av(n_index)*rho)/rho;
    d_b1(node.index) = 100;
    d_b1 = d_b1'
    %check feasibility of minimum constrained to 100 boundary
    sol_boundary_100 = check_feasibility(node, d_b1);
    % compute cost and if best store new optimum
    if sol_boundary_100 
        cost_boundary_100 = evaluate_cost(node, d_b1,rho);
        if cost_boundary_100 < cost_best
           d_best = d_b1;
           cost_best = cost_boundary_100;
        end;
    end;
    
    
    % compute minimum constrained to linear and 0 boundary
    d_l0(n_index) = (node.L-node.o)/node.k(2);
    d_l0(node.index) = 0;
    d_l0 = d_l0'
    %check feasibility of minimum constrained to linear and 0 boundary
    sol_linear_0 = check_feasibility(node, d_l0);
    % compute cost and if best store new optimum
    if sol_linear_0 
        cost_linear_0 = evaluate_cost(node, d_l0,rho);
        if cost_linear_0 < cost_best
           d_best = d_l0;
           cost_best = cost_linear_0;
        end;
    end;
    
    
    % compute minimum constrained to linear and 100 boundary
    d_l1(n_index) = -(100*node.k(1) - node.L + node.o)/node.k(2);
    d_l1(node.index) = 100;
    d_l1 = d_l1'
    %check feasibility of minimum constrained to linear and 0 boundary
    sol_linear_1 = check_feasibility(node, d_l1);
    % compute cost and if best store new optimum
    if sol_linear_1 
        cost_linear_1 = evaluate_cost(node, d_l1,rho);
        if cost_linear_1 < cost_best
           d_best = d_l1;
           cost_best = cost_linear_1;
        end;
    end;
    d = d_best;
    cost = cost_best;
end