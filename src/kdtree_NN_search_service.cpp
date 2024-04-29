// Copyright (c) 2021, Xilinx®.
// All rights reserved
//
// Inspired by the Vector-Add example.
// See https://github.com/Xilinx/Vitis-Tutorials/blob/master/Getting_Started/Vitis
//
// Author: Víctor Mayoral Vilches <v.mayoralv@gmail.com>

#define DATA_SIZE 100 // 2**12
// #define DATA_SIZE 16384  // 2**14
// #define DATA_SIZE 65536  // 2**16
// #define DATA_SIZE 262144  // 2**18

// #define DATA_DIMENSION 5
#define N_KDTREE 6
#define KD_DIM 3
#define N_QUERY_POINTS 3

#include <chrono> // NOLINT
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "kdtree_NN_search.hpp"
#include "test_kdtree.h"

#include <vitis_common/common/ros_opencl_120.hpp>
#include <vitis_common/common/utilities.hpp>

using namespace std::chrono_literals; // NOLINT

// /**
//  * @brief Check that the vector add operation
//  *      was successfully computed, either in PL or PS.
//  *
//  * @param in1 summatory operand 1
//  * @param in2 summatory operand 1
//  * @param out result of the summatory
//  * @return true if successful
//  * @return false if failed
//  */
// bool check_kdtree_NN_search(
//     const int *in1, // Read-Only Vector 1
//     const int *in2, // Read-Only Vector 2
//     const int *out  // Read-Only Result
// )
// {
//   bool match = true;
//   for (int i = 0; i < DATA_SIZE; i++)
//   {
//     int expected = in1[i] + 2 * in2[i];
//     if (out[i] != expected)
//     {
//       std::cout << "Error: Result mismatch" << std::endl;
//       std::cout << "i = " << i << " CPU result = "
//                 << expected << " Device result = " << out[i] << std::endl;
//       match = false;
//       break;
//     }
//   }
//   return match;
// }

void check_find_point_index(float *query_point, float *kd_tree_arr, uint *index_arr, uint &found_index)
{
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "query_point: %f, %f, %f", query_point[X_index], query_point[Y_index], query_point[Z_index]);
  for (uint i = 0; i < N_KDTREE; ++i)
  {
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "kdtree_node: %f, %f, %f", kd_tree_arr[i * DATA_DIMENSION + X_index], kd_tree_arr[i * DATA_DIMENSION + Y_index], kd_tree_arr[i * DATA_DIMENSION + Z_index]);
    // #pragma HLS UNROLL factor = 1
    if (query_point[X_index] == kd_tree_arr[i * KD_DIM + X_index] &&
        query_point[Y_index] == kd_tree_arr[i * KD_DIM + Y_index] &&
        query_point[Z_index] == kd_tree_arr[i * KD_DIM + Z_index])
    {
      // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "inside_ Found index: %d", i);
      found_index = index_arr[i * N_INDEXES + ORIGINAL_INDEX];
      break;
    }
  }
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "*******RAN THROUGH ALL********");
}

void print_points(float *points, uint n_points)
{
  for (uint i = 0; i < n_points; i++)
  {
    std::cout << "Point " << i << ": ";
    for (uint j = 0; j < DATA_DIMENSION; j++)
    {
      std::cout << points[i * DATA_DIMENSION + j] << " ";
    }
    std::cout << std::endl;
  }
}

void print_FPGA_found_NN_info(float *data_arr, uint *index_arr, uint *found_index)
{
  for (uint i = 0; i < N_QUERY_POINTS; i++)
  {
    std::cout << "Found index: " << found_index[i] << std::endl;
    std::cout << "Data: " << data_arr[found_index[i] * KD_DIM + X_index] << ", " << data_arr[found_index[i] * KD_DIM + Y_index] << ", " << data_arr[found_index[i] * KD_DIM + Z_index] << std::endl;
    std::cout << "Original index: " << index_arr[found_index[i] * N_INDEXES + ORIGINAL_INDEX] << std::endl;
    std::cout << "Parent index: " << index_arr[found_index[i] * N_INDEXES + PARENT_INDEX] << std::endl;
    std::cout << "Left index: " << index_arr[found_index[i] * N_INDEXES + LEFT_INDEX] << std::endl;
    std::cout << "Right index: " << index_arr[found_index[i] * N_INDEXES + RIGHT_INDEX] << std::endl;
  }
}

int main(int argc, char *argv[])
{
  // ROS 2 abstractions
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("kdtree_NN_search_publisher");
  auto publisher = node->create_publisher<std_msgs::msg::String>("vector_acceleration", 10);
  auto publish_count = 0;
  std_msgs::msg::String message;
  rclcpp::WallRate loop_rate(100ms);

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
  cl::Kernel krnl_vector_add(program, "kdtree_NN_search", &err);

  // ------------------------------------------------------------------------
  // Step 1.1: Initialize the kdtree
  // kdtree tree;
  // kdtree_allocate(&tree, KD_DIM, N_KDTREE);
  // float data_list[6][DATA_DIMENSION] = {{-1.0, -1.0, -1.0, 0.0, 0.0}, {1.0, 1.0, 1.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0, 0.0}, {0.5, 0.5, 0.5, 0.0, 0.0}, {0.25, 0.25, 0.25, 0.0, 0.0}, {0.0, 0.75, 0.75, 0.0, 0.0}};

  // kdtree_insert(tree, data_list, 6);
  // //make test node to find {0.5, 0.5, 0.5, 0.0, 0.0}
  // kdtree_node *test_node_to_find = new kdtree_node;
  // test_node_to_find->data[X_index] = 0.5;
  // test_node_to_find->data[Y_index] = 0.5;
  // test_node_to_find->data[Z_index] = 0.5;
  // test_node_to_find->data[INTENSITY_index] = 0.0;
  // test_node_to_find->data[ORIGINAL_INDEX_index] = 0.0;

  // ------------------------------------------------------------------------
  // Step 2: Create buffers, map memory
  // ------------------------------------------------------------------------
  // Create the buffers and allocate memory
  // cl::Buffer in1_buf(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_ONLY, sizeof(int) * DATA_SIZE, NULL, &err);
  // cl::Buffer in2_buf(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_ONLY, sizeof(int) * DATA_SIZE, NULL, &err);
  cl::Buffer tree_data_buf(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_ONLY, sizeof(float) * N_KDTREE * KD_DIM, NULL, &err);
  cl::Buffer tree_index_buf(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_ONLY, sizeof(uint) * N_KDTREE * N_INDEXES, NULL, &err);
  cl::Buffer query_points_buf(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_ONLY, sizeof(float) * N_QUERY_POINTS * KD_DIM, NULL, &err);
  cl::Buffer kdtree_found_index(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_WRITE_ONLY, sizeof(uint)*N_QUERY_POINTS, NULL, &err);
  // cl::Buffer out_buf(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_WRITE_ONLY, sizeof(int) * DATA_SIZE, NULL, &err);
  // cl::Buffer cpy_arr_buf(context, CL_MEM_ALLOC_HOST_PTR | CL_MEM_WRITE_ONLY, sizeof(float) * N_KDTREE * KD_DIM, NULL, &err);
  // Map buffers to kernel arguments, thereby assigning
  //  them to specific device memory banks
  // krnl_vector_add.setArg(0, in1_buf);
  // krnl_vector_add.setArg(1, in2_buf);
  krnl_vector_add.setArg(0, tree_data_buf);
  krnl_vector_add.setArg(1, tree_index_buf);
  krnl_vector_add.setArg(2, query_points_buf);
  krnl_vector_add.setArg(3, kdtree_found_index);
  // krnl_vector_add.setArg(6, out_buf);
  // krnl_vector_add.setArg(7, cpy_arr_buf);

  // Map host-side buffer memory to user-space pointers
  // int *in1 = (int *)q.enqueueMapBuffer(in1_buf, CL_TRUE, CL_MAP_WRITE, 0, sizeof(int) * DATA_SIZE);                                       // NOLINT
  // int *in2 = (int *)q.enqueueMapBuffer(in2_buf, CL_TRUE, CL_MAP_WRITE, 0, sizeof(int) * DATA_SIZE);                                       // NOLINT
  float *tree_data = (float *)q.enqueueMapBuffer(tree_data_buf, CL_TRUE, CL_MAP_WRITE, 0, sizeof(float) * N_KDTREE * KD_DIM);             // NOLINT
  uint *tree_index = (uint *)q.enqueueMapBuffer(tree_index_buf, CL_TRUE, CL_MAP_WRITE, 0, sizeof(uint) * N_KDTREE * N_INDEXES);           // NOLINT
  float *query_points = (float *)q.enqueueMapBuffer(query_points_buf, CL_TRUE, CL_MAP_WRITE, 0, sizeof(float) * N_QUERY_POINTS * KD_DIM); // NOLINT
  uint *kdtree_found = (uint *)q.enqueueMapBuffer(kdtree_found_index, CL_TRUE, CL_MAP_READ, 0, sizeof(uint)*N_QUERY_POINTS);              // NOLINT
  // int *out = (int *)q.enqueueMapBuffer(out_buf, CL_TRUE, CL_MAP_WRITE | CL_MAP_READ, 0, sizeof(int) * DATA_SIZE);                         // NOLINT
  // float *cpy_arr = (float *)q.enqueueMapBuffer(cpy_arr_buf, CL_TRUE, CL_MAP_WRITE | CL_MAP_READ, 0, sizeof(float) * N_KDTREE * KD_DIM);   // NOLINT
  // ------------------------------------------------------------------------
  // Step 1.2: Initialize the kdtree
  kdtree tree;
  kdtree_allocate(&tree, KD_DIM, N_KDTREE);
  float data_list[N_KDTREE][DATA_DIMENSION] = {
      {-1.0, -1.0, -1.0, 0.1},
      {1.0, 1.0, 1.0, 0.2},
      {0.0, 0.0, 0.0, 0.3},
      {0.5, 0.5, 0.5, 0.4},
      {0.25, 0.25, 0.25, 0.55},
      {0.0, 0.75, 0.75, 0.6}};

  kdtree_insert(tree, data_list, N_KDTREE);
  // make test node to find {0.5, 0.5, 0.5, 0.0, 0.0}
  // float test_node_to_find[KD_DIM];
  // only look for the first point
  query_points[X_index] = 0.0;
  query_points[Y_index] = 0.25;
  query_points[Z_index] = 0.8;

  query_points[1 * KD_DIM + X_index] = 10.0;
  query_points[1 * KD_DIM + Y_index] = 0.25;
  query_points[1 * KD_DIM + Z_index] = 0.8;

  query_points[2 * KD_DIM + X_index] = 0.0;
  query_points[2 * KD_DIM + Y_index] = -0.25;
  query_points[2 * KD_DIM + Z_index] = 0.3;
  // alocate memory for test data
  // float *test_data_arr = (float *)calloc(N_KDTREE * KD_DIM, sizeof(float));
  // uint *test_index_arr = (uint *)calloc(N_KDTREE * N_CHILDS, sizeof(uint));

  // ------------------------------------------------------------------------
  // Step 2.1: convert tree to arrays
  std::cout << "Here 0" << std::endl;
  // kdtree_print_all_nodes(tree);
  load_tree_to_arrays(tree, tree_data, tree_index);
  std::cout << "Here 1" << std::endl;

  // for (uint i = 0; i < N_KDTREE; i++)
  // {
  //   std::cout << i << "indexes: left: " << tree_index[i * N_INDEXES + LEFT_INDEX] << ", right: " << tree_index[i * N_INDEXES + RIGHT_INDEX] << ", parent: " << tree_index[i * N_INDEXES + PARENT_INDEX] << ", original: " << tree_index[i * N_INDEXES + ORIGINAL_INDEX] << std::endl;
  // }
  float tmp_query_points[N_QUERY_POINTS][KD_DIM];
  for (uint i = 0; i < N_QUERY_POINTS; i++)
  {
    for (uint j = 0; j < KD_DIM; j++)
    {
      tmp_query_points[i][j] = query_points[i * KD_DIM + j];
    }
  }
  for (uint i = 0; i < N_QUERY_POINTS; i++)
  {
    kdtree_node *found_node = kdtree_NN(tree, tmp_query_points[i]); //! will break if N_QUERY_POINTS > 1
    std::cout << "Found node as NN:" << std::endl;
    kdtree_print_node_info(tree, found_node);
  }

  // HLS_equivalent_NN(query_points, 0, 0, kdtree_found[0], tree_data, tree_index);
  // std::cout << "HLS equivalent NN: " << kdtree_found[0] << std::endl;

  // kdtree_NN_non_recursive(tree_data, tree_index, N_KDTREE, query_points, kdtree_found);
  // std::cout << "Non recursive NN: " << kdtree_found[0] << std::endl;

  // found_node = kdtree_NN_non_recursive_test(tree, query_points);
  // uint best_index = 20;
  // kdtree_NN_non_recursive_test(tree_data, tree_index, query_points, best_index);
  // std::cout << "Non recursive NN: " << best_index << std::endl;
  // std::cout << "Data: " << tree_data[best_index * KD_DIM + X_index] << ", " << tree_data[best_index * KD_DIM + Y_index] << ", " << tree_data[best_index * KD_DIM + Z_index] << std::endl;
  // std::cout << "Original index: " << tree_index[best_index * N_INDEXES + ORIGINAL_INDEX] << std::endl;
  // std::cout << "Parent index: " << tree_index[best_index * N_INDEXES + PARENT_INDEX] << std::endl;
  // std::cout << "Left index: " << tree_index[best_index * N_INDEXES + LEFT_INDEX] << std::endl;
  // std::cout << "Right index: " << tree_index[best_index * N_INDEXES + RIGHT_INDEX] << std::endl;
  // std::cout << "Found node as NN:" << std::endl;
  // print original index of node
  // std::cout << "Original index: " << found_node->orginal_index << std::endl;
  // kdtree_print_node_info(tree, found_node);
  // return 0;
  // ------------------------------------------------------------------------
  // Step 3: Main loop, set kernel arguments, schedule transfer
  //  of memory to kernel, run kernel and transfer memory back from it
  // ------------------------------------------------------------------------
  while (rclcpp::ok())
  {
    std::cout << "here 1.1" << std::endl;
    // kdtree_found[0] = 0;
    // randomize the vectors used
    // for (int i = 0; i < DATA_SIZE; i++)
    // {
    //   in1[i] = rand() % DATA_SIZE; // NOLINT
    //   in2[i] = rand() % DATA_SIZE; // NOLINT
    //                                // out[i] = 0;  // writing into a CL_MEM_WRITE_ONLY
    //                                //              // buffer from the host-code is ambiguous and leads to the kernel
    //                                //              // not behaving as expected.
    // }

    // Set kernel arguments
    std::cout << "Here 2" << std::endl;
    // krnl_vector_add.setArg(0, in1_buf);
    // krnl_vector_add.setArg(1, in2_buf);
    krnl_vector_add.setArg(0, tree_data_buf);
    krnl_vector_add.setArg(1, tree_index_buf);
    krnl_vector_add.setArg(2, query_points_buf);
    krnl_vector_add.setArg(3, kdtree_found_index);
    // krnl_vector_add.setArg(6, out_buf);
    // krnl_vector_add.setArg(7, cpy_arr_buf);
    krnl_vector_add.setArg(4, N_KDTREE);
    krnl_vector_add.setArg(5, N_QUERY_POINTS);
    // krnl_vector_add.setArg(10, DATA_SIZE);
    std::cout << "Here 3" << std::endl;

    // Schedule transfer of inputs to device memory
    q.enqueueMigrateMemObjects({tree_data_buf, tree_index_buf, query_points_buf}, 0 /* 0 means from host*/);
    // execution of kernel
    std::cout << "Here 4" << std::endl;
    q.enqueueTask(krnl_vector_add);
    std::cout << "Here 5" << std::endl;
    // transfer of outputs back to host memory
    q.enqueueMigrateMemObjects({kdtree_found_index}, CL_MIGRATE_MEM_OBJECT_HOST);
    // Wait for all scheduled operations to finish
    std::cout << "Here 6" << std::endl;
    q.finish();
    std::cout << "Here 7" << std::endl;
    // best_index = *kdtree_found;

    print_FPGA_found_NN_info(tree_data, tree_index, kdtree_found);
    // std::cout << "FPGA non recursive NN: " << best_index << std::endl;
    // std::cout << "Data: " << tree_data[best_index * KD_DIM + X_index] << ", " << tree_data[best_index * KD_DIM + Y_index] << ", " << tree_data[best_index * KD_DIM + Z_index] << std::endl;
    // std::cout << "Original index: " << tree_index[best_index * N_INDEXES + ORIGINAL_INDEX] << std::endl;
    // std::cout << "Parent index: " << tree_index[best_index * N_INDEXES + PARENT_INDEX] << std::endl;
    // std::cout << "Left index: " << tree_index[best_index * N_INDEXES + LEFT_INDEX] << std::endl;
    // std::cout << "Right index: " << tree_index[best_index * N_INDEXES + RIGHT_INDEX] << std::endl;
    std::cout << "*********************************************************************" << std::endl;

    for (uint i = 0; i < N_QUERY_POINTS; i++)
    {
      kdtree_node *found_node = kdtree_NN(tree, tmp_query_points[i]);
      std::cout << "Found node as NN:" << std::endl;
      kdtree_print_node_info(tree, found_node);
    }
    // kdtree_node *found_node = kdtree_NN(tree, query_points); //! will break if N_QUERY_POINTS > 1
    // std::cout << "Found node as NN:" << std::endl;
    // kdtree_print_node_info(tree, found_node);

    // RCLCPP_INFO(node->get_logger(), "Original index: %d", index);

    // check_find_point_index(query_points, tree_data, tree_index, index);
    // RCLCPP_INFO(node->get_logger(), "Test original index: %d", index);
    // print_points(cpy_arr, N_KDTREE, KD_DIM);
    // print_points(tree_data, N_KDTREE, KD_DIM);
    std::cout << "Here 8" << std::endl;
    // Validate operation in the PS
    // check_kdtree_NN_search(in1, in2, out);

    // Publish publish result
    message.data = "kdtree_NN_search, iteration: " +
                   std::to_string(publish_count++);
    RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", message.data.c_str());

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
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  delete[] fileBuf; // release memory from the acceleration kernel

  return 0;
}
