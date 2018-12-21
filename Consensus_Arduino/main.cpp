#include "Lib_consensus.h"
#include <stdio.h>

int main(int argc, char const *argv[]) {
    float k1[2] = {0.56,0.06};
    float k2[2] = {0.87,0.04};
    Consensus consensus_1(100,0.02,k1);
    Consensus consensus_2(100,0.0,k2);
    float av[2];
    float d1[2];
    float d2[2];

    for(int i=1;i<=50000;i++){
        consensus_1.primal_solver();
        consensus_2.primal_solver();

        consensus_1.get_d(d1);
        consensus_2.get_d(d2);

        // Rounding for transmission
        d1[0] = (int) (d1[0]+0.5);
        d1[1] = (int) (d1[1]+0.5);
        d2[0] = (int) (d2[0]+0.5);
        d2[1] = (int) (d2[1]+0.5);
        printf("%f\n",d2[0]);


        consensus_1.update_average(d2);
        consensus_2.update_average(d1);

        consensus_1.get_average(av);
        printf("%d - av[0] = %f - av[1] = %f\n",i,av[0],av[1]);
    }

    return 0;
}
