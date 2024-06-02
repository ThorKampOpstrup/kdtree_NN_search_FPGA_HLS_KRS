// Copyright (c) 2021, Xilinx®.
// All rights reserved
//
// Inspired by the Vector-Add example.
// See https://github.com/Xilinx/Vitis-Tutorials/blob/master/Getting_Started/Vitis
//
// Author: Víctor Mayoral Vilches <v.mayoralv@gmail.com>

#define DATA_SIZE 100 // 2**12

#define N_KDTREE 21000
#define KD_DIM 3
#define N_QUERY_POINTS 1000
#define FPGA_MAKS_N_QUERY_POINTS 1000

#define N_runs 100

#include <chrono> // NOLINT
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "kdtree_NN_search.hpp"
#include "test_kdtree.h"
#include "fpga_imitator.hpp"

#include <vitis_common/common/ros_opencl_120.hpp>
#include <vitis_common/common/utilities.hpp>

using namespace std::chrono_literals;
/

    void check_find_point_index(float *query_point, float *kd_tree_arr, uint *index_arr, uint &found_index)
{
  for (uint i = 0; i < N_KDTREE; ++i)
  {
    if (query_point[X_index] == kd_tree_arr[i * KD_DIM + X_index] &&
        query_point[Y_index] == kd_tree_arr[i * KD_DIM + Y_index] &&
        query_point[Z_index] == kd_tree_arr[i * KD_DIM + Z_index])
    {
      found_index = index_arr[i * N_INDEXES + ORIGINAL_INDEX];
      break;
    }
  }
}

float point_distance(int *point1, int *point2)
{
  float dist = 0.0;
  for (uint i = 0; i < KD_DIM; i++)
  {
    dist += (point1[i] - point2[i]) * (point1[i] - point2[i]);
  }
  return dist;
}

void find_NN_linear(int *query_points, uint *found_index, int *data_arr, uint *index_arr)
{
  for (uint i = 0; i < N_QUERY_POINTS; i++)
  {
    int query_point[KD_DIM];
    float closest_dist = 1000000.0;
    uint best_index = 0;

    for (uint j = 0; j < KD_DIM; j++)
    {
      query_point[j] = query_points[i * KD_DIM + j];
    }

    for (uint j = 0; j < N_KDTREE; j++)
    {
      float dist = point_distance(query_point, &data_arr[j * KD_DIM]);
      if (dist < closest_dist)
      {
        closest_dist = dist;
        best_index = j;
      }
    }
    found_index[i] = index_arr[best_index * CPU_INDEX_DIMENSION + 2];
  }
}

void print_points(int *points, uint n_points)
{
  for (uint i = 0; i < n_points; i++)
  {
    std::cout << "Point " << i << ": ";
    for (uint j = 0; j < KD_DIM; j++)
    {
      std::cout << points[i * KD_DIM + j] << " ";
    }
    std::cout << std::endl;
  }
}

void print_FPGA_found_NN_info(float *data_arr, uint *index_arr, uint *found_index)
{
  for (uint i = 0; i < N_QUERY_POINTS; i++)
  {
    std::cout << "FPGA-- Found node as NN: " << found_index[i] << std::endl;
    std::cout << "Data: " << data_arr[found_index[i] * KD_DIM + X_index] << ", " << data_arr[found_index[i] * KD_DIM + Y_index] << ", " << data_arr[found_index[i] * KD_DIM + Z_index] << std::endl;
    // std::cout << "Original index: " << index_arr[found_index[i] * N_INDEXES + ORIGINAL_INDEX] << std::endl;
    std::cout << "Parent index: " << index_arr[found_index[i] * N_INDEXES + PARENT_INDEX] << std::endl;
    std::cout << "Left index: " << index_arr[found_index[i] * N_INDEXES + LEFT_INDEX] << std::endl;
    std::cout << "Right index: " << index_arr[found_index[i] * N_INDEXES + RIGHT_INDEX] << std::endl;
  }
}

uint check_match_found_index(kdtree *tree, uint *cpu_found_index, uint *fpga_found_index)
{
  uint counter = 0;
  kdtree_node *fpga_found_node, *cpu_found_node;
  uint fpga_found_org_index, cpu_found_org_index;
  for (uint i = 0; i < N_QUERY_POINTS; i++)
  {

    if (cpu_found_index[i] != fpga_found_index[i])
    {
      std::cout << "Error: Result mismatch" << std::endl;
      std::cout << "i = " << i << " CPU result = "
                << cpu_found_index[i] << " FPGA result = " << fpga_found_index[i] << std::endl;

      counter++;
    }
  }
  return counter;
}

int main(int argc, char *argv[])
{
  std::cout << "Starting kdtree_NN_search_publisher" << std::endl;
  // ROS 2 abstractions
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("kdtree_NN_search_publisher");
  auto publisher = node->create_publisher<std_msgs::msg::String>("Nearest_neighbor", 10);
  auto publish_count = 0;
  std_msgs::msg::String message;
  rclcpp::WallRate loop_rate(500ms);

  // ------------------------------------------------------------------------
  // Step 1: Initialize the OpenCL environment for acceleration
  // ------------------------------------------------------------------------
  cl_int err;
  std::string binaryFile = (argc != 2) ? "kdtree_NN_search.xclbin" : argv[1];
  unsigned fileBufSize;
  std::vector<cl::Device> devices = get_xilinx_devices();
  devices.resize(1);
  cl::Device device = devices[0];
  cl::Context context(device, NULL, NULL, NULL, &err);
  char *fileBuf = read_binary_file(binaryFile, fileBufSize);
  cl::Program::Binaries bins{{fileBuf, fileBufSize}};
  cl::Program program(context, devices, bins, NULL, &err);
  cl::CommandQueue q(context, device, CL_QUEUE_PROFILING_ENABLE, &err);
  cl::Kernel krnl_nn_finder(program, "kdtree_NN_search", &err);

  // ------------------------------------------------------------------------
  // Step 2: Create buffers, map memory
  // ------------------------------------------------------------------------
  // Create the buffers and allocate memory
  cl::Buffer tree_data_buf(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_ONLY, sizeof(int) * N_KDTREE * KD_DIM, NULL, &err);
  cl::Buffer tree_index_buf(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_ONLY, sizeof(uint) * N_KDTREE * N_INDEXES, NULL, &err);
  cl::Buffer query_points_buf(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_ONLY, sizeof(int) * N_QUERY_POINTS * KD_DIM, NULL, &err);
  cl::Buffer kdtree_found_index(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_WRITE_ONLY, sizeof(uint) * N_QUERY_POINTS, NULL, &err);

  // Set the kernel arguments
  krnl_nn_finder.setArg(0, tree_data_buf);
  krnl_nn_finder.setArg(1, tree_index_buf);
  krnl_nn_finder.setArg(2, query_points_buf);
  krnl_nn_finder.setArg(4, kdtree_found_index);

  // Map buffers to host pointers
  int *tree_data = (int *)q.enqueueMapBuffer(tree_data_buf, CL_TRUE, CL_MAP_WRITE, 0, sizeof(int) * N_KDTREE * KD_DIM);             // NOLINT
  uint *tree_index = (uint *)q.enqueueMapBuffer(tree_index_buf, CL_TRUE, CL_MAP_WRITE, 0, sizeof(uint) * N_KDTREE * N_INDEXES);     // NOLINT
  int *query_points = (int *)q.enqueueMapBuffer(query_points_buf, CL_TRUE, CL_MAP_WRITE, 0, sizeof(int) * N_QUERY_POINTS * KD_DIM); // NOLINT
  uint *kdtree_found = (uint *)q.enqueueMapBuffer(kdtree_found_index, CL_TRUE, CL_MAP_READ, 0, sizeof(uint) * N_QUERY_POINTS);      // NOLINT

  // ------------------------------------------------------------------------
  // Step 2.2: Initialize the kdtree
  kdtree tree;
  kdtree_allocate(&tree, KD_DIM, N_KDTREE);
  float data_list[N_KDTREE][DATA_DIMENSION];

  for (uint i = 0; i < N_KDTREE; i++)
  {
    for (uint j = 0; j < DATA_DIMENSION; j++)
    {
      data_list[i][j] = (((float)(rand() % 20000)) / 10000) - 1;
    }
  }

  kdtree_insert(tree, data_list, N_KDTREE);

  // ------------------------------------------------------------------------
  // Step 2.3: convert tree to arrays
  load_tree_to_arrays(tree, tree_data, tree_index);

  // ------------------------------------------------------------------------
  // Step 3: Main loop, set kernel arguments, schedule transfer
  //  of memory to kernel, run kernel and transfer memory back from it
  // ------------------------------------------------------------------------
  bool rld_tree = true;

  auto start = std::chrono::high_resolution_clock::now();
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  while (rclcpp::ok())
  {

    std::cout << ":::::::::::NEW ITERATION::::::::::::" << std::endl;
    for (uint i = 0; i < N_QUERY_POINTS; i++)
    {
      for (uint j = 0; j < KD_DIM; j++)
      {
        // generate a random integer between 100 and -100
        query_points[i * KD_DIM + j] = (int)(1000.0 * (((float)(rand() % 20000)) / 10000) - 1);
      }
    }

    //--------------------------------------------------------------------------------
    // FPGA implementation with reloading the tree
    //--------------------------------------------------------------------------------
    krnl_nn_finder.setArg(0, tree_data_buf);
    krnl_nn_finder.setArg(1, tree_index_buf);
    krnl_nn_finder.setArg(2, query_points_buf);
    krnl_nn_finder.setArg(3, kdtree_found_index);

    rld_tree = true;
    krnl_nn_finder.setArg(4, rld_tree);

    krnl_nn_finder.setArg(5, N_KDTREE);
    krnl_nn_finder.setArg(6, N_QUERY_POINTS);

    start = std::chrono::high_resolution_clock::now();
    for (uint i = 0; i < N_runs; i++)
    {
      std::cout << "running FPGA rld" << std::endl;
      q.enqueueMigrateMemObjects({tree_data_buf, tree_index_buf, query_points_buf}, 0 /* 0 means from host*/);

      // execution of kernel
      q.enqueueTask(krnl_nn_finder);

      // transfer of outputs back to host memory
      q.enqueueMigrateMemObjects({kdtree_found_index}, CL_MIGRATE_MEM_OBJECT_HOST);

      // Wait for all scheduled operations to finish
      q.finish();
    }
    // end timer
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << duration.count() << std::endl;

    std::cout << "Tree was " << (rld_tree ? "reloaded" : "NOT reloaded") << std::endl;
    std::cout << "Time taken by FPGA: "
              << duration.count() / 1000 << " ms" << std::endl;
    std::cout << "Time taken by FPGA: "
              << duration.count() << " microseconds" << std::endl;

    //--------------------------------------------------------------------------------
    // FPGA implementation without reloading the tree
    //--------------------------------------------------------------------------------
    start = std::chrono::high_resolution_clock::now();
    rld_tree = true;
    krnl_nn_finder.setArg(4, rld_tree);

    start = std::chrono::high_resolution_clock::now();
    for (uint i = 0; i < N_runs; i++)
    {
      std::cout << "running FPGA no rld" << std::endl;
      q.enqueueMigrateMemObjects({tree_data_buf, tree_index_buf, query_points_buf}, 0 /* 0 means from host*/);

      // execution of kernel
      q.enqueueTask(krnl_nn_finder);

      // transfer of outputs back to host memory
      q.enqueueMigrateMemObjects({kdtree_found_index}, CL_MIGRATE_MEM_OBJECT_HOST);

      // Wait for all scheduled operations to finish
      q.finish();
    }
    // end timer
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << duration.count() << std::endl;

    std::cout << "Tree was " << (rld_tree ? "reloaded" : "NOT reloaded") << std::endl;
    std::cout << "Time taken by FPGA: "
              << duration.count() / 1000 << " ms" << std::endl;
    std::cout << "Time taken by FPGA: "
              << duration.count() << " microseconds" << std::endl;

    //--------------------------------------------------------------------------------
    // CPU implementation with FPGA imitator
    //--------------------------------------------------------------------------------
    start = std::chrono::high_resolution_clock::now();
    uint cpu_found_index[N_QUERY_POINTS];
    for (uint i = 0; i < N_runs; i++)
    {
      find_NNs_CPU_FPGA_imitator(tree_data, tree_index, query_points, cpu_found_index, N_KDTREE, N_QUERY_POINTS);
    }
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    std::cout << "<<FPGA----------CPU>>" << std::endl;
    std::cout << "Time taken by CPU: "
              << duration.count() / 1000 << " ms" << std::endl;
    std::cout << "Time taken by CPU: "
              << duration.count() << " microseconds" << std::endl;

    //--------------------------------------------------------------------------------
    // CPU implementation with linear search
    //--------------------------------------------------------------------------------
    uint cpu_found_org_index_lin[N_QUERY_POINTS];
    start = std::chrono::high_resolution_clock::now();
    for (uint i = 0; i < N_runs / 10; i++)
    {
      find_NN_linear(query_points, cpu_found_org_index_lin, tree_data, tree_index);
    }
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

    std::cout << "<<CPU linear search>>" << std::endl;
    std::cout << "Time taken by CPU linear search: "
              << duration.count() / 1000 << " ms" << std::endl;
    std::cout << "Time taken by CPU linear search: "
              << duration.count() << " microseconds" << std::endl;

    uint mismatch_counter = check_match_found_index(&tree, cpu_found_index, kdtree_found);
    std::cout << "Mismatch counter: " << mismatch_counter << std::endl;

    // Publish publish result
    message.data = "kdtree_NN_search, iteration: " +
                   std::to_string(publish_count++);
    try
    {
      publisher->publish(message);
      rclcpp::spin_some(node);
    }
    catch (const rclcpp::exceptions::RCLError &e)
    {
      RCLCPP_ERROR(
          node->get_logger(),
          "unexpectedly failed with %s",
          e.what());
    }
    std::cout << ":::::::::::END OF ITERATION::::::::::::" << std::endl;
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  delete[] fileBuf; // release memory from the acceleration kernel

  return 0;
}
