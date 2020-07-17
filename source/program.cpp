#include "paging-simulation.h"

int main(int argc, char const *argv[])
{

    PagingSimulation simulation(argc, argv);
    simulation.simulate();

    /* comment them out for the test programs mentioned in part 3*/
    /* note: even though the arguments are  ignored for part3's programs. make sure you provide a argument list to this program */
    /* I just want to avoid creating seperate executables  */
    // simulation.findOptimalSize();
    // simulation.findOptimalAlgorithm();
    // simulation.workingSetData();
    return 0;
}
