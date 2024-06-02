// Copyright (c) 2021, Xilinx®.
// All rights reserved
//
// Author: Víctor Mayoral Vilches <v.mayoralv@gmail.com>

#ifndef XILINX_EXAMPLES_KDTREE_NN_SEARCH_PUBLISHER_INCLUDE_KDTREE_NN_SEARCH_HPP_
#define XILINX_EXAMPLES_KDTREE_NN_SEARCH_PUBLISHER_INCLUDE_KDTREE_NN_SEARCH_HPP_

extern "C"
{
    void kdtree_NN_search(int *tree_data_in,
                          uint *tree_index_in,
                          int *query_points_in,
                          uint *found_query_point_index,
                          bool rld_tree,
                          uint n_kdtree_nodes,
                          uint n_query_points);
} // extern "C"

#endif // XILINX_EXAMPLES_VADD_PUBLISHER_INCLUDE_VADD_HPP_
