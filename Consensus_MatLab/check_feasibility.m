function check = check_feasibility(node, d)
    if(node.index == 1)
        index = 1;
        n_index = 2;
    else
        index = 2;
        n_index = 1;
    end
    
    tol = 0.001; %%tolerance for rounding errors
    if (d(node.index) < 0-tol), check = 0; return; end;
    
    if (d(node.index) > 100+tol), check = 0; return; end;
    
    if (d(node.index)*node.k(1)+ d(n_index)*node.k(2)< node.L-node.o-tol), check = 0; return; end;
    
    check = 1;
end