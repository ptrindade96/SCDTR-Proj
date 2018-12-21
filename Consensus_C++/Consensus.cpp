#include "Consensus.hpp"

///////////////////////////////////////////////////////////////////////////////
//  Constructor - Initializes class internal variables.
///////////////////////////////////////////////////////////////////////////////
Consensus::Consensus(float C_,float Q_,float max_d_){
    C = C_;
    Q = Q_;
    max_d = max_d_;
}

///////////////////////////////////////////////////////////////////////////////
//  get_average() - returns (pass by reference), the current computed average
//  for the led dimming.
///////////////////////////////////////////////////////////////////////////////
void Consensus::get_average(float av[]){
    av[0] = d_av[0];
    av[1] = d_av[1];
}

///////////////////////////////////////////////////////////////////////////////
//  get_d() - returns (pass by reference), the current computed value for
//  the led dimming in this node.
///////////////////////////////////////////////////////////////////////////////
void Consensus::get_d(float di[]){
    di[0] = d[0];
    di[1] = d[1];
}

///////////////////////////////////////////////////////////////////////////////
//  update_k() - receives an array with new values for the node gains.
///////////////////////////////////////////////////////////////////////////////
void Consensus::update_k(float k_new[]){
    k[0] = k_new[0];
    k[1] = k_new[1];
}

///////////////////////////////////////////////////////////////////////////////
//  update_L() - receives new lower bound for node iluminance.
///////////////////////////////////////////////////////////////////////////////
void Consensus::update_L(float L_new){
    L = L_new;
}

///////////////////////////////////////////////////////////////////////////////
//  update_L() - receives new value for node's external disturbance.
///////////////////////////////////////////////////////////////////////////////
void Consensus::update_o(float o_new){
    o = o_new;
}

///////////////////////////////////////////////////////////////////////////////
//  update_average() - receives an array with the computed dimmings at the
//  other node and updates the average.
///////////////////////////////////////////////////////////////////////////////
void Consensus::update_average(float dj[]){
    d_av[0] = (d[0]+dj[1])/2;
    d_av[1] = (d[1]+dj[0])/2;
    y[0] += RHO*(d[0]-d_av[0]);
    y[1] += RHO*(d[1]-d_av[1]);
}

///////////////////////////////////////////////////////////////////////////////
//  is_feasible() - receives an array with the computed dimmings and checks
//  the fesibility of this solution for this node
///////////////////////////////////////////////////////////////////////////////
bool Consensus::is_feasible(float di[]){
    if(di[0] < -TOLERANCE)
        return false;
    if(di[0] > max_d + TOLERANCE)
        return false;
    if(di[0]*k[0]+di[1]*k[1]<L-o-TOLERANCE)
        return false;

    return true;
}

///////////////////////////////////////////////////////////////////////////////
//  compute_cost() - receives an array with the computed dimmings and evaluates
//  the cost function
///////////////////////////////////////////////////////////////////////////////
float Consensus::compute_cost(float di[]){
    float error[] = {di[0]-d_av[0],di[1]-d_av[1]};
    float cost = C*di[0] + y[0]*error[0] + y[1]*error[1];
    cost += RHO/2*(error[0]*error[0]+error[1]*error[1]) + di[0]*di[0]*Q/2;
    return cost;
}

///////////////////////////////////////////////////////////////////////////////
//  primal_solver() - Computes the new value for dimming, acording to the
//  objects internal values (y,C,Q,L,d_av)
///////////////////////////////////////////////////////////////////////////////
void Consensus::primal_solver(){
    float cost_best = 10000000;

    //              Unconstrained minimum               //
    d[0] = -(C+y[0]-d_av[0]*RHO)/(RHO+Q);
    d[1] = -(y[1]-d_av[1]*RHO)/RHO;
    if(Consensus::is_feasible(d))
        return;

    //          Minimum constrained to boundaries      //
    cost_best = Consensus::linear_boundary(d,cost_best);
    cost_best = Consensus::upper_boundary(d,cost_best);
    cost_best = Consensus::lower_boundary(d,cost_best);
    cost_best = Consensus::linear_and_upper_boundary(d,cost_best);
    cost_best = Consensus::linear_and_lower_boundary(d,cost_best);

    return;
}

///////////////////////////////////////////////////////////////////////////////
//  linear_boundary() - Computes the dimming associated with the linear
//  boundary, and updates if cost is best.
///////////////////////////////////////////////////////////////////////////////
float Consensus::linear_boundary(float di[],float cost_best){
    float d_aux[2];

    d_aux[0] = -(C*k[1]*k[1] + k[1]*k[1]*y[0] - d_av[0]*k[1]*k[1]*RHO - L*k[0]*RHO + k[0]*o*RHO - k[0]*k[1]*y[1] + d_av[1]*k[0]*k[1]*RHO);
    d_aux[0] = d_aux[0]/(Q*k[1]*k[1] + k[0]*k[0]*RHO + k[1]*k[1]*RHO);
    d_aux[1] = (k[1]*(L - o + (k[1]*(y[1] - d_av[1]*RHO))/RHO + (k[0]*(C + y[0] - d_av[0]*RHO))/(Q + RHO)));
    d_aux[1] = d_aux[1]/(RHO*(k[0]*k[0]/(Q + RHO) + k[1]*k[1]/RHO)) - (y[1] - d_av[1]*RHO)/RHO;

    if(Consensus::is_feasible(d_aux)){
        float cost = Consensus::compute_cost(d_aux);
        if(cost<cost_best){
            di[0] = d_aux[0];
            di[1] = d_aux[1];
            return cost;
        }
    }
    return cost_best;
}

///////////////////////////////////////////////////////////////////////////////
//  lower_boundary() - Computes the dimming associated with the lower
//  boundary, and updates if cost is best.
///////////////////////////////////////////////////////////////////////////////
float Consensus::lower_boundary(float di[],float cost_best){
    float d_aux[2];

    d_aux[0] = 0;
    d_aux[1] = -(y[1] - d_av[1]*RHO)/RHO;

    if(Consensus::is_feasible(d_aux)){
        float cost = Consensus::compute_cost(d_aux);
        if(cost<cost_best){
            di[0] = d_aux[0];
            di[1] = d_aux[1];
            return cost;
        }
    }
    return cost_best;
}

///////////////////////////////////////////////////////////////////////////////
//  upper_boundary() - Computes the dimming associated with the upper
//  boundary, and updates if cost is best.
///////////////////////////////////////////////////////////////////////////////
float Consensus::upper_boundary(float di[],float cost_best){
    float d_aux[2];

    d_aux[0] = max_d;
    d_aux[1] = -(y[1] - d_av[1]*RHO)/RHO;

    if(Consensus::is_feasible(d_aux)){
        float cost = Consensus::compute_cost(d_aux);
        if(cost<cost_best){
            di[0] = d_aux[0];
            di[1] = d_aux[1];
            return cost;
        }
    }
    return cost_best;
}

///////////////////////////////////////////////////////////////////////////////
//  linear_and_lower_boundary() - Computes the dimming associated with the
//  intersection between the linear and lower boundary, and updates if cost
//  is best.
///////////////////////////////////////////////////////////////////////////////
float Consensus::linear_and_lower_boundary(float di[],float cost_best){
    float d_aux[2];

    d_aux[0] = 0;
    d_aux[1] = (L - o)/k[1];

    if(Consensus::is_feasible(d_aux)){
        float cost = Consensus::compute_cost(d_aux);
        if(cost<cost_best){
            di[0] = d_aux[0];
            di[1] = d_aux[1];
            return cost;
        }
    }
    return cost_best;
}

///////////////////////////////////////////////////////////////////////////////
//  linear_and_upper_boundary() - Computes the dimming associated with the
//  intersection between the linear and upper boundary, and updates if cost
//  is best.
///////////////////////////////////////////////////////////////////////////////
float Consensus::linear_and_upper_boundary(float di[],float cost_best){
    float d_aux[2];

    d_aux[0] = max_d;
    d_aux[1] = (L - o - max_d*k[0])/k[1];

    if(Consensus::is_feasible(d_aux)){
        float cost = Consensus::compute_cost(d_aux);
        if(cost<cost_best){
            di[0] = d_aux[0];
            di[1] = d_aux[1];
            return cost;
        }
    }
    return cost_best;
}
