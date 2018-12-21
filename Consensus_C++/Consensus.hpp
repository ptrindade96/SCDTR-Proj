///////////////////////////////////////////////////////////////////////////////
//  Implement consensus computations on Arduino
///////////////////////////////////////////////////////////////////////////////

// Some class definitions.
#define TOLERANCE   0.001
#define RHO         0.07
#define DEFAULT_LB  100
#define DEFAULT_K   1
#define DEFAULT_O   0

class Consensus{
private:
    float y[2] = {0,0};
    float d[2] = {0,0};

    float L = DEFAULT_LB;
    float o = DEFAULT_O;
    float d_av[2] = {0,0};
    float k[2] = {DEFAULT_K,DEFAULT_K};
    float C;
    float Q;
    float max_d;

    bool is_feasible(float di[]);
    float compute_cost(float di[]);
    float linear_boundary(float di[],float cost_best);
    float lower_boundary(float di[],float cost_best);
    float upper_boundary(float di[],float cost_best);
    float linear_and_lower_boundary(float di[],float cost_best);
    float linear_and_upper_boundary(float di[],float cost_best);

public:
    Consensus() = delete;
    Consensus(float C_,float Q_,float max_d_);
    ~Consensus() = default;

    void update_average(float dj[]);
    void primal_solver();
    void update_L(float L_new);
    void update_o(float o_new);
    void update_k(float k_new[]);
    void get_average(float av[]);
    void get_d(float di[]);
};
