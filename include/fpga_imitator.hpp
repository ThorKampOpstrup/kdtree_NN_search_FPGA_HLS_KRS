#include <cstdint>
#include <cstring>
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

#define N_NODES 21000
#define N_QUERY_POINTS 1000
#define N_NODES_in_TOP_URAM 1333 // 32*CPU_INDEX_DIMENSION*3000=280 000 (amount of bits in an ultraram)
// #define N_NODES_in_TOP_URAM 5
#define N_TOPS 10

int top_kdtree_data1[N_NODES_in_TOP_URAM * CPU_DATA_DIMENSION];
int top_kdtree_data2[N_NODES_in_TOP_URAM * CPU_DATA_DIMENSION];
int top_kdtree_data3[N_NODES_in_TOP_URAM * CPU_DATA_DIMENSION];
int top_kdtree_data4[N_NODES_in_TOP_URAM * CPU_DATA_DIMENSION];
int top_kdtree_data5[N_NODES_in_TOP_URAM * CPU_DATA_DIMENSION];
int top_kdtree_data6[N_NODES_in_TOP_URAM * CPU_DATA_DIMENSION];
int top_kdtree_data7[N_NODES_in_TOP_URAM * CPU_DATA_DIMENSION];
int top_kdtree_data8[N_NODES_in_TOP_URAM * CPU_DATA_DIMENSION];
int top_kdtree_data9[N_NODES_in_TOP_URAM * CPU_DATA_DIMENSION];
int top_kdtree_data10[N_NODES_in_TOP_URAM * CPU_DATA_DIMENSION];

int kd_tree_data_rest[(N_NODES - N_NODES_in_TOP_URAM) * CPU_DATA_DIMENSION];

uint top_kdtree_index1[N_NODES_in_TOP_URAM * CPU_INDEX_DIMENSION];
uint top_kdtree_index2[N_NODES_in_TOP_URAM * CPU_INDEX_DIMENSION];
uint top_kdtree_index3[N_NODES_in_TOP_URAM * CPU_INDEX_DIMENSION];
uint top_kdtree_index4[N_NODES_in_TOP_URAM * CPU_INDEX_DIMENSION];
uint top_kdtree_index5[N_NODES_in_TOP_URAM * CPU_INDEX_DIMENSION];
uint top_kdtree_index6[N_NODES_in_TOP_URAM * CPU_INDEX_DIMENSION];
uint top_kdtree_index7[N_NODES_in_TOP_URAM * CPU_INDEX_DIMENSION];
uint top_kdtree_index8[N_NODES_in_TOP_URAM * CPU_INDEX_DIMENSION];
uint top_kdtree_index9[N_NODES_in_TOP_URAM * CPU_INDEX_DIMENSION];
uint top_kdtree_index10[N_NODES_in_TOP_URAM * CPU_INDEX_DIMENSION];

uint kd_tree_index_rest[(N_NODES - N_NODES_in_TOP_URAM) * CPU_INDEX_DIMENSION];
int query_points[N_QUERY_POINTS * CPU_DATA_DIMENSION];

void distance_squared_CPU(int point1[CPU_DATA_DIMENSION], int point2[CPU_DATA_DIMENSION], int &distance)
{
    int sum = 0;
    for (uint i = 0; i < CPU_DATA_DIMENSION; ++i)
    {
#pragma HLS UNROLL factor = 1
        sum += (point1[i] - point2[i]) * (point1[i] - point2[i]);
    }
    distance = sum;
}

void kdtree_NN_non_recursive_CPU(int *data_arr, uint *index_arr, int *targets_in, uint *best_index, uint n_targets, uint n_nodes)
{
    //   std::cout << "running kdtree_NN_non_recursive_test" << std::endl;
    // #pragma HLS pipeline
    uint stack_index[n_nodes];
    char stack_dimension[n_nodes];
    int target[CPU_DATA_DIMENSION];

    for (uint i = 0; i < n_targets; i++)
    {
        for (uint j = 0; j < CPU_DATA_DIMENSION; j++)
        {
            target[j] = targets_in[i * CPU_DATA_DIMENSION + j];
        }

        uint stack_ptr = 0;
        stack_index[stack_ptr] = 0;
        stack_dimension[stack_ptr++] = 0;

        best_index[i] = 0;
        int best_dist = 2147483647 - 1; // 2^32/2-1
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
            int current_dist;
            distance_squared_CPU(&data_arr[current_index * CPU_DATA_DIMENSION], target, current_dist);

            if (current_dist < best_dist)
            {
                // #pragma HLS pipeline
                best_dist = current_dist;
                best_index[i] = current_index;
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

            int plane_dist = target[dimension] - data_arr[current_index * CPU_DATA_DIMENSION + dimension];
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
}

void kdtree_NN_non_recursive_CPU_separate_start(int *data_arr_start, uint *index_arr_start, int *target_in, uint *best_index, uint n_targets)
{
    //   std::cout << "running kdtree_NN_non_recursive_test" << std::endl;
    // #pragma HLS pipeline
    uint stack_index[N_NODES];
    char stack_dimension[N_NODES];
    int target[CPU_DATA_DIMENSION];

    for (uint i = 0; i < n_targets; i++)
    {
        for (uint j = 0; j < CPU_DATA_DIMENSION; j++)
        {
            target[j] = target_in[j + i * CPU_DATA_DIMENSION];
        }

        uint stack_ptr = 0;
        stack_index[stack_ptr] = 0;
        stack_dimension[stack_ptr++] = 0;

        best_index[i] = 0;
        int best_dist = 2147483647 - 1; // 2^32/2-1
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
            int current_dist;
            if (current_index < N_NODES_in_TOP_URAM)
            {
                distance_squared_CPU(&data_arr_start[current_index * CPU_DATA_DIMENSION], target, current_dist);
            }
            else
            {
                distance_squared_CPU(&kd_tree_data_rest[(current_index - N_NODES_in_TOP_URAM) * CPU_DATA_DIMENSION], target, current_dist);
            }

            if (current_dist < best_dist)
            {
                // #pragma HLS pipeline
                best_dist = current_dist;
                best_index[i] = current_index;
            }

            uint next_dimension = (dimension + 1) % CPU_DATA_DIMENSION;
            uint index_next, index_other;

            int tmp_data = 0;
            if (current_index < N_NODES_in_TOP_URAM)
            {
                tmp_data = data_arr_start[current_index * CPU_DATA_DIMENSION + dimension];
            }
            else
            {
                tmp_data = kd_tree_data_rest[(current_index - N_NODES_in_TOP_URAM) * CPU_DATA_DIMENSION + dimension];
            }

            if (target[dimension] < tmp_data)
            {
                // #pragma HLS pipeline
                if (current_index < N_NODES_in_TOP_URAM)
                {
                    index_next = index_arr_start[current_index * CPU_INDEX_DIMENSION + LEFT_INDEX];
                    index_other = index_arr_start[current_index * CPU_INDEX_DIMENSION + RIGHT_INDEX];
                }
                else
                {
                    index_next = kd_tree_index_rest[(current_index - N_NODES_in_TOP_URAM) * CPU_INDEX_DIMENSION + LEFT_INDEX];
                    index_other = kd_tree_index_rest[(current_index - N_NODES_in_TOP_URAM) * CPU_INDEX_DIMENSION + RIGHT_INDEX];
                }
            }
            else
            {
                // #pragma HLS pipeline
                if (current_index < N_NODES_in_TOP_URAM)
                {
                    index_next = index_arr_start[current_index * CPU_INDEX_DIMENSION + RIGHT_INDEX];
                    index_other = index_arr_start[current_index * CPU_INDEX_DIMENSION + LEFT_INDEX];
                }
                else
                {
                    index_next = kd_tree_index_rest[(current_index - N_NODES_in_TOP_URAM) * CPU_INDEX_DIMENSION + RIGHT_INDEX];
                    index_other = kd_tree_index_rest[(current_index - N_NODES_in_TOP_URAM) * CPU_INDEX_DIMENSION + LEFT_INDEX];
                }
            }

            int plane_dist = target[dimension] - tmp_data;
            plane_dist *= plane_dist;
            // std::cout << "plane_dist: " << plane_dist << std::endl;

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
}

void find_NNs_CPU_FPGA_imitator(int *data_arr, uint *index_arr, int *query_points_in, uint *found_index, uint n_kdtree_nodes, uint n_query_points)
{
    uint tmp_size = n_kdtree_nodes * CPU_DATA_DIMENSION * sizeof(int);
    uint tmp_size_top = (tmp_size > N_NODES_in_TOP_URAM * CPU_DATA_DIMENSION * sizeof(int)) ? N_NODES_in_TOP_URAM * CPU_INDEX_DIMENSION * sizeof(int) : tmp_size;

    std::memcpy(top_kdtree_data1, data_arr, tmp_size_top);
    std::memcpy(top_kdtree_data2, data_arr, tmp_size_top);
    std::memcpy(top_kdtree_data3, data_arr, tmp_size_top);
    std::memcpy(top_kdtree_data4, data_arr, tmp_size_top);
    std::memcpy(top_kdtree_data5, data_arr, tmp_size_top);
    std::memcpy(top_kdtree_data6, data_arr, tmp_size_top);
    std::memcpy(top_kdtree_data7, data_arr, tmp_size_top);
    std::memcpy(top_kdtree_data8, data_arr, tmp_size_top);
    std::memcpy(top_kdtree_data9, data_arr, tmp_size_top);
    std::memcpy(top_kdtree_data10, data_arr, tmp_size_top);

    std::memcpy(top_kdtree_index1, index_arr, tmp_size_top);
    std::memcpy(top_kdtree_index2, index_arr, tmp_size_top);
    std::memcpy(top_kdtree_index3, index_arr, tmp_size_top);
    std::memcpy(top_kdtree_index4, index_arr, tmp_size_top);
    std::memcpy(top_kdtree_index5, index_arr, tmp_size_top);
    std::memcpy(top_kdtree_index6, index_arr, tmp_size_top);
    std::memcpy(top_kdtree_index7, index_arr, tmp_size_top);
    std::memcpy(top_kdtree_index8, index_arr, tmp_size_top);
    std::memcpy(top_kdtree_index9, index_arr, tmp_size_top);
    std::memcpy(top_kdtree_index10, index_arr, tmp_size_top);

    if (0 < tmp_size - tmp_size_top)
    {
        std::memcpy(kd_tree_data_rest, &data_arr[N_NODES_in_TOP_URAM * CPU_DATA_DIMENSION], tmp_size - tmp_size_top);
        std::memcpy(kd_tree_index_rest, &index_arr[N_NODES_in_TOP_URAM * CPU_INDEX_DIMENSION], tmp_size - tmp_size_top);
    }
    std::memcpy(query_points, query_points_in, n_query_points * CPU_DATA_DIMENSION * sizeof(int));

    uint n_per_impl = n_query_points / N_TOPS;

    kdtree_NN_non_recursive_CPU_separate_start(top_kdtree_data1, top_kdtree_index1, &query_points[0], &found_index[0], n_per_impl);
    kdtree_NN_non_recursive_CPU_separate_start(top_kdtree_data2, top_kdtree_index2, &query_points[n_per_impl * CPU_DATA_DIMENSION], &found_index[n_per_impl], n_per_impl);
    kdtree_NN_non_recursive_CPU_separate_start(top_kdtree_data3, top_kdtree_index3, &query_points[2 * n_per_impl * CPU_DATA_DIMENSION], &found_index[2 * n_per_impl], n_per_impl);
    kdtree_NN_non_recursive_CPU_separate_start(top_kdtree_data4, top_kdtree_index4, &query_points[3 * n_per_impl * CPU_DATA_DIMENSION], &found_index[3 * n_per_impl], n_per_impl);
    kdtree_NN_non_recursive_CPU_separate_start(top_kdtree_data5, top_kdtree_index5, &query_points[4 * n_per_impl * CPU_DATA_DIMENSION], &found_index[4 * n_per_impl], n_per_impl);
    kdtree_NN_non_recursive_CPU_separate_start(top_kdtree_data6, top_kdtree_index6, &query_points[5 * n_per_impl * CPU_DATA_DIMENSION], &found_index[5 * n_per_impl], n_per_impl);
    kdtree_NN_non_recursive_CPU_separate_start(top_kdtree_data7, top_kdtree_index7, &query_points[6 * n_per_impl * CPU_DATA_DIMENSION], &found_index[6 * n_per_impl], n_per_impl);
    kdtree_NN_non_recursive_CPU_separate_start(top_kdtree_data8, top_kdtree_index8, &query_points[7 * n_per_impl * CPU_DATA_DIMENSION], &found_index[7 * n_per_impl], n_per_impl);
    kdtree_NN_non_recursive_CPU_separate_start(top_kdtree_data9, top_kdtree_index9, &query_points[8 * n_per_impl * CPU_DATA_DIMENSION], &found_index[8 * n_per_impl], n_per_impl);
    kdtree_NN_non_recursive_CPU_separate_start(top_kdtree_data10, top_kdtree_index10, &query_points[9 * n_per_impl * CPU_DATA_DIMENSION], &found_index[9 * n_per_impl], n_per_impl);

    uint rest = n_query_points - n_per_impl * N_TOPS;
    if (rest > 0)
    {
        kdtree_NN_non_recursive_CPU_separate_start(top_kdtree_data1, top_kdtree_index1, &query_points[10 * n_per_impl * CPU_DATA_DIMENSION], &found_index[10 * n_per_impl], rest);
    }
}