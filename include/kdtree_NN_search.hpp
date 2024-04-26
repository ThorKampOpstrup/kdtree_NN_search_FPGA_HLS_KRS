// Copyright (c) 2021, Xilinx®.
// All rights reserved
//
// Author: Víctor Mayoral Vilches <v.mayoralv@gmail.com>

#ifndef XILINX_EXAMPLES_KDTREE_NN_SEARCH_PUBLISHER_INCLUDE_KDTREE_NN_SEARCH_HPP_
#define XILINX_EXAMPLES_KDTREE_NN_SEARCH_PUBLISHER_INCLUDE_KDTREE_NN_SEARCH_HPP_

extern "C" {
    void kdtree_NN_search(
            const unsigned int *in1,  // Read-Only Vector 1
            const unsigned int *in2,  // Read-Only Vector 2
            unsigned int *out,        // Output Result
            int size                  // Size in integer
    );
}  // extern "C"

#endif  // XILINX_EXAMPLES_VADD_PUBLISHER_INCLUDE_VADD_HPP_
