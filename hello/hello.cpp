#include <stdlib.h>
#include <stdio.h>

#include <vector>
#include <map>
#include <iostream>

extern "C" {
#include "include/mcbsp.h"
}

void withMaps() {
    bsp_begin( 4 );
    /// Init
    std::map<char, std::vector<unsigned int> > m;
    std::vector<unsigned int> a;
    a.resize(3);
    m['m'] = a;
    bsp_push_reg(m['m'].data(), m['m'].capacity()*sizeof(unsigned int));

    if(0 == bsp_pid() % 2) {
        m['m'] = {bsp_pid()*5+1, bsp_pid()*10+1, bsp_pid()*15+1};
    }

    bsp_sync();
    // getting values of even into odd
    if(1 == bsp_pid() % 2) {
        bsp_get(bsp_pid() - 1, m['m'].data(), 0,
                m['m'].data(), 3 * sizeof(unsigned int));
    }
    
    bsp_sync();
    // print values
    for(int i = 0; i < 4; ++i) {
        if(i == bsp_pid()) {
            std::cout << "Proc {" << bsp_pid() << "} contains"
                      << std::endl << std::flush;
            for(std::vector<unsigned int>::const_iterator it = m['m'].cbegin();
                it != m['m'].cend(); ++it)
                std::cout << *it << " "; 
            std::cout << std::endl << std::flush;
        }
        bsp_sync();
    }
    bsp_pop_reg(m['m'].data());
    bsp_end();
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
        bsp_get(bsp_pid() - 1, a.data(), 0,
                a.data(), 3 * sizeof(unsigned int));
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
    bsp_init( &withMaps, argc, argv );
    spmd();
    return EXIT_SUCCESS;
}
