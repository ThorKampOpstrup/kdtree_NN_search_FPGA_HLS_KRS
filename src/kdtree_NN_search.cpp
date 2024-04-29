/*
      ____  ____
     /   /\/   /
    /___/  \  /   Copyright (c) 2021, Xilinx®.
    \   \   \/    Author: Víctor Mayoral Vilches <victorma@xilinx.com>
     \   \
     /   /
    /___/   /\
    \   \  /  \
     \___\/\___\

Inspired by the Vector-Add example.
See https://github.com/Xilinx/Vitis-Tutorials/blob/master/Getting_Started/Vitis

*/
#include "ap_fixed.h"
#include "ap_int.h"

#define DATA_DIMENSION 3
#define INDEX_DIMENSION 3

#define LEFT_INDEX 0
#define RIGHT_INDEX 1
#define PARENT_INDEX 2
// #define ORIGINAL_INDEX 3

#define X_index 0
#define Y_index 1
#define Z_index 2

#define N_NODES 30000
#define N_QUERY_POINTS 1000

// define uint as unsigned int
typedef unsigned int uint;

// typedef ap_fixed<15,8, AP_RND, AP_WRAP_SM> fixed_float; // set a fixed type with 15 bits, 8 integer bits, and 7 fractional bits (8 describes the integer bits)
// typedef ap_uint<16> costum_uint16;
// typedef ap_uint<3> costum_uint3;
// typedef ap_uint<1> my_true_bool;

// const int c_size = DATA_SIZE;

float kdtree_nodes_data_FPGA[N_NODES * DATA_DIMENSION];
uint kdtree_nodes_indexes_FPGA[N_NODES * 3]; /// left, right, parent
float query_points_FPGA[N_QUERY_POINTS * INDEX_DIMENSION];
uint found_index_FPGA[N_QUERY_POINTS];

// void load_kdtree_node(kdtree_node *node, kdtree_node *dst)
// {
//     for (uint j = 0; j < DATA_DIMENSION; ++j)
//     {
//         dst->data[j] = node->data[j];
//     }
//     dst->left_index = node->left_index;
//     dst->right_index = node->right_index;
//     dst->parent_index = node->parent_index;
// }

void distance_squared(float point1[DATA_DIMENSION], float point2[DATA_DIMENSION], float &distance)
{
    float sum = 0;
    for (uint i = 0; i < DATA_DIMENSION; ++i)
    {
#pragma HLS UNROLL factor = 1
        sum += (point1[i] - point2[i]) * (point1[i] - point2[i]);
    }
    distance = sum;
}

void kdtree_NN_non_recursive(float *data_arr, uint *index_arr, float target[DATA_DIMENSION], uint &best_index)
{
    //   std::cout << "running kdtree_NN_non_recursive_test" << std::endl;
// #pragma HLS pipeline
    uint stack_index[N_NODES];
    char stack_dimension[N_NODES];

    uint stack_ptr = 0;
    stack_index[stack_ptr] = 0;
    stack_dimension[stack_ptr++] = 0;

    best_index = 0;
    float best_dist = 255;
    bool first_run = true;

    while (stack_ptr > 0)
    {
// #pragma HLS pipeline
        uint index = stack_index[--stack_ptr];
        uint dimension = stack_dimension[stack_ptr];

        if (index == 0 && !first_run)
        {
            continue;
        }

        uint current_index = index;
        float current_dist;
        distance_squared(&data_arr[current_index * DATA_DIMENSION], target, current_dist);

        if (current_dist < best_dist)
        {
// #pragma HLS pipeline
            best_dist = current_dist;
            best_index = current_index;
        }

        uint next_dimension = (dimension + 1) % DATA_DIMENSION;
        uint index_next, index_other;

        if (target[dimension] < data_arr[current_index * DATA_DIMENSION + dimension])
        {
// #pragma HLS pipeline
            index_next = index_arr[current_index * INDEX_DIMENSION + LEFT_INDEX];
            index_other = index_arr[current_index * INDEX_DIMENSION + RIGHT_INDEX];
        }
        else
        {
// #pragma HLS pipeline
            index_next = index_arr[current_index * INDEX_DIMENSION + RIGHT_INDEX];
            index_other = index_arr[current_index * INDEX_DIMENSION + LEFT_INDEX];
        }

        float plane_dist = target[dimension] - data_arr[current_index * DATA_DIMENSION + dimension];
        plane_dist *= plane_dist;

        if (plane_dist < best_dist && index_other != 0)
        {
            if (stack_ptr < N_NODES)
            {
// #pragma HLS pipeline
                stack_index[stack_ptr] = index_other;
                stack_dimension[stack_ptr++] = next_dimension;
            }
        }
        if (index_next != 0)
        {
            if (stack_ptr < N_NODES)
            {
// #pragma HLS pipeline
                stack_index[stack_ptr] = index_next;
                stack_dimension[stack_ptr++] = next_dimension;
            }
        }
        first_run = false;
    }
}

void load_kdtree(float *point_in, uint *index_in, uint n_nodes)
{
    for (uint i = 0; i < n_nodes; ++i)
    {
// #pragma HLS pipeline
#pragma HLS unroll factor = 16
        for (uint j = 0; j < DATA_DIMENSION; ++j)
        {
#pragma HLS unroll factor = 3
            kdtree_nodes_data_FPGA[i * DATA_DIMENSION + j] = (float)point_in[i * DATA_DIMENSION + j];
        }
        for (uint j = 0; j < INDEX_DIMENSION; ++j)
        {
#pragma HLS unroll factor = 3
            kdtree_nodes_indexes_FPGA[i * INDEX_DIMENSION + j] = index_in[i * INDEX_DIMENSION + j];
        }
    }
}

void load_query_points(float *query_points_in, uint n_query_points)
{
    for (uint i = 0; i < n_query_points; ++i)
    {
// #pragma HLS pipeline
#pragma HLS unroll factor = 16
        for (uint j = 0; j < DATA_DIMENSION; ++j)
        {
#pragma HLS unroll factor = 3
            query_points_FPGA[i * DATA_DIMENSION + j] = query_points_in[i * DATA_DIMENSION + j];
        }
    }
}

// {
//     for (uint i = 0; i < n_nodes; ++i)
//     {
//         load_kdtree_node(&nodes[i], &kdtree_nodes_FPGA[i]);
//     }
// }

// void calc_distance_sqrt(float point1[DATA_DIMENSION], float point2[DATA_DIMENSION], float &distance)
// {
//     float sum = 0;
//     for (uint i = 0; i < DATA_DIMENSION; ++i)
//     {
//         sum += (point1[i] - point2[i]) * (point1[i] - point2[i]);
//     }
//     distance = sum;
// }

//! only look for an exact match
// void find_point_index(float query_point[DATA_DIMENSION], uint n_nodes, uint &found_index)
// {
//     for (uint i = 0; i < n_nodes; ++i)
//     {

// #pragma HLS UNROLL factor = 1
//         if (query_point[X_index] == kdtree_nodes_data_FPGA[i * DATA_DIMENSION + X_index] &&
//             query_point[Y_index] == kdtree_nodes_data_FPGA[i * DATA_DIMENSION + Y_index] &&
//             query_point[Z_index] == kdtree_nodes_data_FPGA[i * DATA_DIMENSION + Z_index])
//         {
//             found_index = kdtree_nodes_indexes_FPGA[i * INDEX_DIMENSION + ORIGINAL_INDEX];
//             break;
//         }
//     }
// }
// void find_point_index(kdtree_node point, uint n_query_points, uint found_index)
// {
//     for (uint i = 0; i < n_query_points; ++i)
//     {
//         if (query_points_FPGA[i].x == point.data[X_index] &&
//             query_points_FPGA[i].y == point.data[Y_index] &&
//             query_points_FPGA[i].z == point.data[Z_index])
//         {
//             found_index =  i;
//             break;
//         }
//     }
// }

// void memcpy(float *dest, float *src, int size)
// {
//     // for (int i = 0; i < (size / 16) * 16; ++i)
//     for (int i = 0; i < size; ++i)
//     {
// #pragma HLS UNROLL factor = 16
//         // #pragma HLS loop_tripcount min = c_size max = c_size
//         dest[i] = src[i];
//     }
// }

void memcpy(uint *dest, uint *src, int size)
{
    // for (int i = 0; i < (size / 16) * 16; ++i)
    for (int i = 0; i < size; ++i)
    {
#pragma HLS UNROLL factor = 16
        // #pragma HLS loop_tripcount min = c_size max = c_size
        dest[i] = src[i];
    }
}

// uint in1_arr[DATA_SIZE];
// uint out_arr[DATA_SIZE];
// uint in2_arr[DATA_SIZE];

// Returns the index of the closest point not the original index
void find_NNs(uint n_query_points)
{
    for (uint i = 0; i < n_query_points; ++i)
    {
#pragma HLS UNROLL factor = 20
        uint tmp_index = 0;
        kdtree_NN_non_recursive(kdtree_nodes_data_FPGA, kdtree_nodes_indexes_FPGA, &query_points_FPGA[i * DATA_DIMENSION], tmp_index);
        found_index_FPGA[i] = tmp_index;
    }
}

extern "C"
{
    void kdtree_NN_search(
        // uint *in1, // Read-Only Vector 1
        // uint *in2, // Read-Only Vector 2
        float *tree_data_in,
        uint *tree_index_in,
        float *query_points_in,
        uint *found_query_point_index,
        // uint *out, // Output Result
        // float *cpy_arr,
        bool rld_tree,
        uint n_kdtree_nodes,
        uint n_query_points
        // int size // Size in integer
    )
    {
// #pragma HLS INTERFACE m_axi port = in1 offset = slave bundle = gmem0
// #pragma HLS INTERFACE m_axi port = in2 offset = slave bundle = gmem1
#pragma HLS INTERFACE m_axi port = tree_data_in offset = slave bundle = gmem0
#pragma HLS INTERFACE m_axi port = tree_index_in offset = slave bundle = gmem1
#pragma HLS INTERFACE m_axi port = query_points_in offset = slave bundle = gmem0
#pragma HLS INTERFACE m_axi port = found_query_point_index offset = slave bundle = gmem1
        // #pragma HLS INTERFACE m_axi port = out offset = slave bundle = gmem0
        // #pragma HLS INTERFACE m_axi port = cpy_arr offset = slave bundle = gmem1

#pragma HLS RESOURCE variable = kdtree_nodes_data_FPGA core = RAM_2P_URAM
#pragma HLS RESOURCE variable = kdtree_nodes_indexes_FPGA core = RAM_2P_URAM
#pragma HLS RESOURCE variable = query_points_FPGA core = RAM_2P_BRAM
#pragma HLS RESOURCE variable = found_index_FPGA core = RAM_2P_BRAM

        // memcpy(in1_arr, in1, size);
        // memcpy(in2_arr, in2, size);
        if (rld_tree)
        {
            load_kdtree(tree_data_in, tree_index_in, n_kdtree_nodes);
        }

        //         for (int j = 0; j < size; ++j)
        //         { // stupidly iterate over
        //           // it to generate load
        // #pragma HLS loop_tripcount min = c_size max = c_size
        //             for (int i = 0; i < size; ++i)
        //             {
        // #pragma HLS loop_tripcount min = c_size max = c_size
        // #pragma HLS UNROLL factor = 16
        //                 out_arr[i] = in1_arr[i] + 2 * in2_arr[i];
        //             }
        //         }

        load_query_points(query_points_in, n_query_points);
        // kd_tree_node_point_to_find = load_kdtree_node(kd_tree_node_point_in);

        // uint tmp_index = find_point_index(kd_tree_node_point_to_find, N_QUERY_POINTS);
        // uint tmp_index;
        // float query_point[DATA_DIMENSION];

        // for (int i = 0; i < DATA_DIMENSION; ++i)
        // {
        //     query_point[i] = query_points_FPGA[i];
        // }

        // find_point_index(query_points_FPGA, n_kdtree_nodes, tmp_index);
        // // tmp_index = 8; //!update this
        // memcpy(found_query_point_index, &tmp_index, 1);

        find_NNs(n_query_points);
        memcpy(found_query_point_index, found_index_FPGA, n_query_points);
        // memcpy(out, out_arr, size);
        // memcpy(cpy_arr, kdtree_nodes_data_FPGA, n_kdtree_nodes * DATA_DIMENSION);
    }
}
