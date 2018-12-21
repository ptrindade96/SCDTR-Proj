///////////////////////////////////////////////////////////////////////////////
//  Implement consensus computations on Arduino
///////////////////////////////////////////////////////////////////////////////

// Some numbers (These may already be on Arduino).
#define MAX_D       100
#define TOLERANCE   0.001
#define RHO         0.07
#define Q           0.1
#define C           1


class Consensus{
private:
    float y[2] = {0,0};
    float d[2] = {0,0};

    float L;
    float o;
    float d_av[2] = {0,0};
    float k[2];

    bool is_feasible(float di[]);
    float compute_cost(float di[]);
    float linear_boundary(float di[],float cost_best);
    float lower_boundary(float di[],float cost_best);
    float upper_boundary(float di[],float cost_best);
    float linear_and_lower_boundary(float di[],float cost_best);
    float linear_and_upper_boundary(float di[],float cost_best);

public:
    Consensus() = delete;
    Consensus(float L_,float o_,float k_[]);
    ~Consensus() = default;

    void update_average(float dj[]);
    void primal_solver();
    void update_L(float L_new);
    void update_o(float o_new);
    void update_k(float k_new[]);
    void get_average(float av[]);
    void get_d(float di[]);
};


Consensus::Consensus(float L_,float o_,float k_[]){
    L = L_;
    o = o_;
    k[0] = k_[0];
    k[1] = k_[1];
}

void Consensus::get_average(float av[]){
    av[0] = d_av[0];
    av[1] = d_av[1];
}

void Consensus::get_d(float di[]){
    di[0] = d[0];
    di[1] = d[1];
}



void Consensus::update_k(float k_new[]){
    k[0] = k_new[0];
    k[1] = k_new[1];
}

void Consensus::update_L(float L_new){
    L = L_new;
}

void Consensus::update_o(float o_new){
    o = o_new;
}

void Consensus::update_average(float dj[]){
    d_av[0] = (d[0]+dj[1])/2;
    d_av[1] = (d[1]+dj[0])/2;
    y[0] += RHO*(d[0]-d_av[0]);
    y[1] += RHO*(d[1]-d_av[1]);
}

bool Consensus::is_feasible(float di[]){
    if(di[0] < -TOLERANCE)
        return false;
    if(di[0] > MAX_D + TOLERANCE)
        return false;
    if(di[0]*k[0]+di[1]*k[1]<L-o-TOLERANCE)
        return false;

    return true;
}

float Consensus::compute_cost(float di[]){
    float error[] = {di[0]-d_av[0],di[1]-d_av[1]};
    float cost = C*di[0] + y[0]*error[0] + y[1]*error[1];
    cost += RHO/2*(error[0]*error[0]+error[1]*error[1]) + di[0]*di[0]*Q;
    return cost;
}

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

float Consensus::linear_boundary(float di[],float cost_best){
    float d_aux[2];

    d_aux[0] = k[1]*k[1]*(C+y[0]-RHO*d_av[0])+k[0]*RHO*(o-L)+k[0]*k[1]*(d_av[1]*RHO-y[1]);
    d_aux[0] = -d_aux[0]/(k[1]*k[1]*(Q+RHO)+k[0]*k[0]*RHO);

    d_aux[1] = k[1]*(L-o+(k[1]*(y[1]-d_av[1]*RHO))/RHO + (k[1]*(C+y[0]-d_av[0]*RHO))/(Q+RHO));
    d_aux[1] = d_aux[1]/(RHO*(k[0]*k[0]/(Q+RHO)+k[1]*k[1]/RHO)) - (y[1]-d_av[1]*RHO)/RHO;

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

float Consensus::upper_boundary(float di[],float cost_best){
    float d_aux[2];

    d_aux[0] = MAX_D;
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

float Consensus::linear_and_upper_boundary(float di[],float cost_best){
    float d_aux[2];

    d_aux[0] = MAX_D;
    d_aux[1] = (L - o - MAX_D*k[0])/k[1];

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
