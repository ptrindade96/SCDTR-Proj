function cost = evaluate_cost(node,d,rho)
    Q = zeros(2,2);
    Q(node.index,node.index) = node.Q;
    cost =  node.c*d(node.index) + node.y'*(d-node.d_av) + ...
            rho/2*norm(d-node.d_av)^2 + d'*Q*d/2;
end