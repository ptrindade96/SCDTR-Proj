///////////////////////////////////////////////////////////////////////////////
//  Implement consensus computations on Arduino
///////////////////////////////////////////////////////////////////////////////

// Some numbers (These may already be on Arduino).
#define MAX_D       255
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
