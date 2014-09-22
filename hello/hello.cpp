#include <stdlib.h>
#include <stdio.h>

#include <vector>
#include <iostream>

extern "C" {
#include "include/mcbsp.h"
}

void spmd() {
    bsp_begin( 4 );
    /// Init
    std::vector<unsigned int> a;
    a.resize(3);
    bsp_push_reg(a.data(), a.capacity()*sizeof(unsigned int));

    if(0 == bsp_pid() % 2) {
        a = {bsp_pid()*5+1, bsp_pid()*10+1, bsp_pid()*15+1};
    }

    bsp_sync();
    // getting values of even into odd
    if(1 == bsp_pid() % 2) {
        bsp_get(bsp_pid()-1, a.data(), 0,
                a.data(), 3*sizeof(unsigned int));
    }
    
    bsp_sync();
    // print values
    for(int i = 0; i < 4; ++i) {
        if(i == bsp_pid()) {
            std::cout << "Proc {" << bsp_pid() << "} contains"
                      << std::endl << std::flush;
            for(std::vector<unsigned int>::const_iterator it = a.cbegin();
                it != a.cend(); ++it)
                std::cout << *it << " "; 
            std::cout << std::endl << std::flush;
        }
        bsp_sync();
    }
    bsp_pop_reg(a.data());
    bsp_end();
}

int main( int argc, char ** argv ) {
    bsp_init( &spmd, argc, argv );
    spmd();
    return EXIT_SUCCESS;
}
