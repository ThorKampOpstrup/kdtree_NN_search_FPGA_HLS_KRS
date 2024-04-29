#include <iostream>

#define CPU_DATA_DIMENSION 3
#define CPU_INDEX_DIMENSION 3

#define LEFT_INDEX 0
#define RIGHT_INDEX 1
#define PARENT_INDEX 2
// #define ORIGINAL_INDEX 3

#define X_index 0
#define Y_index 1
#define Z_index 2

// #define N_NODES 30000
// #define N_QUERY_POINTS 1000

void distance_squared_CPU(float point1[CPU_DATA_DIMENSION], float point2[CPU_DATA_DIMENSION], float &distance)
{
    float sum = 0;
    for (uint i = 0; i < CPU_DATA_DIMENSION; ++i)
    {
#pragma HLS UNROLL factor = 1
        sum += (point1[i] - point2[i]) * (point1[i] - point2[i]);
    }
    distance = sum;
}

void kdtree_NN_non_recursive_CPU(float *data_arr, uint *index_arr, float target[CPU_DATA_DIMENSION], uint &best_index, uint n_nodes)
{
    //   std::cout << "running kdtree_NN_non_recursive_test" << std::endl;
// #pragma HLS pipeline
    uint stack_index[n_nodes];
    char stack_dimension[n_nodes];

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
        distance_squared_CPU(&data_arr[current_index * CPU_DATA_DIMENSION], target, current_dist);

        if (current_dist < best_dist)
        {
// #pragma HLS pipeline
            best_dist = current_dist;
            best_index = current_index;
        }

        uint next_dimension = (dimension + 1) % CPU_DATA_DIMENSION;
        uint index_next, index_other;

        if (target[dimension] < data_arr[current_index * CPU_DATA_DIMENSION + dimension])
        {
// #pragma HLS pipeline
            index_next = index_arr[current_index * CPU_INDEX_DIMENSION + LEFT_INDEX];
            index_other = index_arr[current_index * CPU_INDEX_DIMENSION + RIGHT_INDEX];
        }
        else
        {
// #pragma HLS pipeline
            index_next = index_arr[current_index * CPU_INDEX_DIMENSION + RIGHT_INDEX];
            index_other = index_arr[current_index * CPU_INDEX_DIMENSION + LEFT_INDEX];
        }

        float plane_dist = target[dimension] - data_arr[current_index * CPU_DATA_DIMENSION + dimension];
        plane_dist *= plane_dist;

        if (plane_dist < best_dist && index_other != 0)
        {
            if (stack_ptr < n_nodes)
            {
// #pragma HLS pipeline
                stack_index[stack_ptr] = index_other;
                stack_dimension[stack_ptr++] = next_dimension;
            }
        }
        if (index_next != 0)
        {
            if (stack_ptr < n_nodes)
            {
// #pragma HLS pipeline
                stack_index[stack_ptr] = index_next;
                stack_dimension[stack_ptr++] = next_dimension;
            }
        }
        first_run = false;
    }
}

void find_NNs_CPU_FPGA_imitator(float *data_arr, uint *index_arr, float *query_points, uint *found_index, uint n_kdtree_nodes, uint n_query_points)
{
    for (uint i = 0; i < n_query_points; ++i)
    {
// #pragma HLS UNROLL factor = 20
        uint tmp_index = 0;
        kdtree_NN_non_recursive_CPU(data_arr, index_arr, &query_points[i * CPU_DATA_DIMENSION], tmp_index, n_kdtree_nodes);
        found_index[i] = tmp_index;
    }
}