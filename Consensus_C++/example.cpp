///////////////////////////////////////////////////////////////////////////////
//  Example Usage - Simulating to nodes
//  Compile with -std=c++11
///////////////////////////////////////////////////////////////////////////////
#include "Consensus.hpp"
#include <stdio.h>

#define LINEAR_COST_1   1
#define LINEAR_COST_2   1

#define QUAD_COST_1     0.1
#define QUAD_COST_2     0.1

#define LOW_BOUND_1     100
#define LOW_BOUND_2     20

#define O_1             10
#define O_2             2

#define K_11            0.56
#define K_12            0.06
#define K_21            0.04
#define K_22            0.87

#define MAX_D           255


int main(int argc, char const *argv[]){
    float k1[2] = {K_11,K_12};
    float k2[2] = {K_22,K_21};
    float av[2];
    float d1[2];
    float d2[2];

    // Create consensus objects
    Consensus consensus_1(LINEAR_COST_1,QUAD_COST_1,MAX_D);
    Consensus consensus_2(LINEAR_COST_2,QUAD_COST_2,MAX_D);

    // Define some values for node 1 (these had defaults)
    consensus_1.update_L(LOW_BOUND_1);
    consensus_1.update_o(O_1);
    consensus_1.update_k(k1);
    // Define some values for node 2 (these had defaults)
    consensus_2.update_L(LOW_BOUND_2);
    consensus_2.update_o(O_2);
    consensus_2.update_k(k2);

    // Loop for solving
    for(int i=1;i<=50;i++){
        consensus_1.primal_solver();
        consensus_2.primal_solver();

        // Get last dimming arrays
        consensus_1.get_d(d1);
        consensus_2.get_d(d2);

        // Rounding for transmission (simulating)
        d1[0] = (int) (d1[0]+0.5);
        d1[1] = (int) (d1[1]+0.5);
        d2[0] = (int) (d2[0]+0.5);
        d2[1] = (int) (d2[1]+0.5);

        // Print some values (After rounding...)
        printf("d11=%f\td21=%f\td12=%f\td22=%f\n",d1[0],d2[1],d1[1],d2[0]);

        // Update consensus values
        consensus_1.update_average(d2);
        consensus_2.update_average(d1);
    }
    return 0;
}
