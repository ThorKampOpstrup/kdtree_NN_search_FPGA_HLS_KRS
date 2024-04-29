// this file will generate a basic kd tree for the point cloud based on the xyz positions of the points
#include <test_kdtree.h>

#include <stdio.h>
#include <stdlib.h>

//! added after problems
#include <iostream>
#include <cstring>
#include <cmath>

#include <vector>
#include <limits>

// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>

// #define DATA_DIMENSION 5

// #define X_index 0
// #define Y_index 1
// #define Z_index 2
// #define INTENSITY_index 3
// #define ORIGINAL_INDEX_index 4

// typedef struct Data {
//   float x, y, z, intensity;
//   uint original_index;
// } Data;

// typedef struct kdtree_node {
//   // struct kdtree_node *left, *right, *parent;
//   uint left_index = 0, right_index = 0, parent_index = 0;
//   float data[DATA_DIMENSION]; //Should be changed to Data type
// } kdtree_node;

// // ex: DATA = {x,y,z,intensity, orginal_index}

// typedef struct kdtree {
//   kdtree_node *root;
//   uint dim;
//   uint number_of_nodes = 0;  // number of nodes inserted in the tree
// } kdtree;

// take a number of expected nodes and return 1 if succes and 0 if fail.
// dim is the number of dimensions of each point, that they shout be sorted for
// n is the number of nodes that the tree should be able to hold
int kdtree_allocate(kdtree *tree, uint dim_in, uint n)
{
  tree->dim = dim_in;
  tree->root = (kdtree_node *)calloc(n, sizeof(kdtree_node));
  std::cout << "kdtree_node size: " << sizeof(kdtree_node) << std::endl;
  uint bytes = sizeof(kdtree_node) * n;
  std::cout << "allocated memory: " << bytes << "bytes" << std::endl;
  float mb = (float(bytes)) / 1024.0 / 1024.0;
  std::cout << "alocated memory: " << mb << "mb" << std::endl;
  if (tree->root == nullptr)
  {
    std::cerr << "failed to allocate memory for kdtree" << std::endl;
    return 0;
  }
  return 1;
}

// Insert a point into the tree, a point is a 3d point(can contain intensities or other data)
void add_node(kdtree &tree, float data[DATA_DIMENSION], uint original_index)
{
  // if it is the first point
  if (tree.number_of_nodes == 0)
  {
    kdtree_node *new_node = tree.root;
    new_node->left_index = 0;
    new_node->right_index = 0;
    new_node->parent_index = 0;
    new_node->orginal_index = original_index;
    memcpy(new_node->data, data, sizeof(float) * DATA_DIMENSION);
    tree.number_of_nodes++;
    return;
  }

  uint current_index = 0;
  uint parent_index = 0;
  // insert into the kd tree
  bool correctly_inserted = false, left_branch = false;
  uint dimension_to_compare = 0;
  kdtree_node *parent_ptr;

  while (!correctly_inserted)
  {
    parent_ptr = tree.root + current_index;
    parent_index = current_index;
    if (parent_ptr == nullptr)
    {
      std::cout << "parent_ptr is null" << std::endl;
    }
    if (data[dimension_to_compare] < parent_ptr->data[dimension_to_compare])
    {
      current_index = parent_ptr->left_index;
      left_branch = true;
    }
    else
    {
      current_index = parent_ptr->right_index;
      left_branch = false;
    }

    if (current_index == 0)
    {
      kdtree_node *new_node = tree.root + tree.number_of_nodes;
      new_node->left_index = 0;
      new_node->right_index = 0;
      new_node->parent_index = parent_index;
      new_node->orginal_index = original_index;
      memcpy(new_node->data, data, sizeof(float) * DATA_DIMENSION);
      if (left_branch)
      {
        parent_ptr->left_index = tree.number_of_nodes;
      }
      else
      {
        parent_ptr->right_index = tree.number_of_nodes;
      }
      tree.number_of_nodes++;
      correctly_inserted = true;
    }

    dimension_to_compare = (dimension_to_compare + 1) % tree.dim;
  }
}

// Insert an array of points into the tree, an array of points is a set of 3d points(can contain intensities or other data)
// Points = [[],[],...,[]] where each [] is a 3d point
// n is the number of points
// The point will be added from the middle of the points array
void kdtree_insert(kdtree &tree, float points[][DATA_DIMENSION], uint n)
{
  int middle_index = n / 2;
  int offset, index;

  add_node(tree, points[middle_index], middle_index);

  for (uint i = 0; i < n - 1; i++)
  {
    offset = std::pow(-1, i + 1) * ((i / 2) + 1);
    index = middle_index + offset;
    add_node(tree, points[index], index);
    // print index
    // print point data
    //  std::cout << "point: " << points[index][0] << ", " << points[index][1] << ", " << points[index][2] << std::endl;
    //  std::cout << "index: " << index << std::endl;
    //  std::cout << "\r\n";
  }
}

// void kdtree_insert(kdtree &tree, pcl::PointCloud<pcl::PointXYZI>::Ptr pc) {
//   uint n = pc->width * pc->height;
//   uint middle_index = n / 2;
//   uint offset, index;
//   float data[DATA_DIMENSION] = {pc->points[middle_index].x, pc->points[middle_index].y, pc->points[middle_index].z, pc->points[middle_index].intensity};
//   add_node(tree, data);

//   for(uint i = 0; i < n - 1; i++) {
//     offset = std::pow(-1, i + 1) * ((i / 2) + 1);
//     // Calculate the actual index to access
//     index = middle_index + offset;
//     data[X_index] = pc->points[index].x;
//     data[Y_index] = pc->points[index].y;
//     data[Z_index] = pc->points[index].z;
//     data[INTENSITY_index] = pc->points[index].intensity;
//     data[ORIGINAL_INDEX_index] = index;
//     add_node(tree, data);
//   }
// }

// void kdtree_insert(kdtree &tree, pcl::PointCloud<pcl::PointXYZ>::Ptr pc) {
//   uint n = pc->width * pc->height;
//   uint middle_index = n / 2;
//   uint offset, index;
//   float data[DATA_DIMENSION] = {pc->points[middle_index].x, pc->points[middle_index].y, pc->points[middle_index].z, 0};
//   add_node(tree, data);
//   for(uint i = 1; i < n - 1; i++) {
//     offset = std::pow(-1, i + 1) * ((i / 2) + 1);
//     index = middle_index + offset;

//     data[X_index] = pc->points[index].x;
//     data[Y_index] = pc->points[index].y;
//     data[Z_index] = pc->points[index].z;
//     data[ORIGINAL_INDEX_index] = index;
//     add_node(tree, data);
//   }
// }

float distance_squared(kdtree &tree, float a_data[DATA_DIMENSION], float b_data[DATA_DIMENSION])
{
  float sum = 0;
  for (uint i = 0; i < tree.dim; i++)
  {
    sum += (a_data[i] - b_data[i]) * (a_data[i] - b_data[i]);
  }
  return sum;
}

float kdtree_distance(kdtree &tree, float a_data[DATA_DIMENSION], float b_data[DATA_DIMENSION])
{
  return sqrt(distance_squared(tree, a_data, b_data));
}

// For the user to use, as the start index should not be inserted by the user
kdtree_node *kdtree_NN(kdtree &tree, float target[DATA_DIMENSION])
{
  return kdtree_NN_intern(tree, target, 0, 0);
}

kdtree_node *kdtree_NN_intern(kdtree &tree, float target[DATA_DIMENSION], uint starting_index, uint dimension_to_compare)
{
  // std::cout << "starting_index: " << starting_index << std::endl;
  kdtree_node *current_node = tree.root + starting_index;
  if (current_node->left_index == 0 && current_node->right_index == 0)
  {
    return current_node;
  }

  uint index_next, index_other;
  if (target[dimension_to_compare] < current_node->data[dimension_to_compare])
  {
    index_next = current_node->left_index;
    index_other = current_node->right_index;
  }
  else
  {
    index_next = current_node->right_index;
    index_other = current_node->left_index;
  }

  if (index_next == 0)
  {
    return current_node;
  }

  kdtree_node *tmp = kdtree_NN_intern(tree, target, index_next, (dimension_to_compare + 1) % tree.dim);
  kdtree_node *best = (distance_squared(tree, tmp->data, target) < distance_squared(tree, current_node->data, target)) ? tmp : current_node;

  float r = distance_squared(tree, best->data, target);
  float r_mark = (target[dimension_to_compare] - current_node->data[dimension_to_compare]);

  if (r_mark * r_mark < r && index_other != 0)
  {
    tmp = kdtree_NN_intern(tree, target, index_other, (dimension_to_compare + 1) % tree.dim);
    best = (distance_squared(tree, tmp->data, target) < distance_squared(tree, best->data, target)) ? tmp : best;
  }

  return best;
}

void HLS_distance_squared(float *point1, float *point2, float &distance)
{
  float sum = 0;
  for (uint i = 0; i < 3; ++i) //! replace 3 with DATA_DIMENSION, when going to hls
  {
    // #pragma HLS UNROLL factor = 1
    sum += (point1[i] - point2[i]) * (point1[i] - point2[i]);
  }
  distance = sum;
}

constexpr size_t MAX_STACK_SIZE = 128;

void kdtree_NN_non_recursive_test(float *data_arr, uint *index_arr, float target[DATA_DIMENSION], uint &best_index)
{
  // std::cout << "running kdtree_NN_non_recursive_test" << std::endl;
  uint stack_index[MAX_STACK_SIZE];
  char stack_dimension[MAX_STACK_SIZE];

  int stack_ptr = 0;
  stack_index[stack_ptr] = 0;
  stack_dimension[stack_ptr++] = 0;

  best_index = 0;
  float best_dist = 1e38f;
  bool first_run = true;

  while (stack_ptr > 0)
  {
    uint index = stack_index[--stack_ptr];
    uint dimension = stack_dimension[stack_ptr];

    if (index == 0 && !first_run)
    {
      continue;
    }

    uint current_index = index;
    float current_dist;
    HLS_distance_squared(&data_arr[current_index * TMP_DIM], target, current_dist);

    if (current_dist < best_dist)
    {
      best_dist = current_dist;
      best_index = current_index;
    }

    uint next_dimension = (dimension + 1) % TMP_DIM;
    uint index_next, index_other;

    if (target[dimension] < data_arr[current_index * TMP_DIM + dimension])
    {
      index_next = index_arr[current_index * INDEX_DIMENSION_FPGA + LEFT_INDEX];
      index_other = index_arr[current_index * INDEX_DIMENSION_FPGA + RIGHT_INDEX];
    }
    else
    {
      index_next = index_arr[current_index * INDEX_DIMENSION_FPGA + RIGHT_INDEX];
      index_other = index_arr[current_index * INDEX_DIMENSION_FPGA + LEFT_INDEX];
    }

    float plane_dist = target[dimension] - data_arr[current_index * TMP_DIM + dimension];
    plane_dist *= plane_dist;

    if (plane_dist < best_dist && index_other != 0)
    {
      if (stack_ptr < MAX_STACK_SIZE)
      {
        stack_index[stack_ptr] = index_other;
        stack_dimension[stack_ptr++] = next_dimension;
      }
    }
    if (index_next != 0)
    {
      if (stack_ptr < MAX_STACK_SIZE)
      {
        stack_index[stack_ptr] = index_next;
        stack_dimension[stack_ptr++] = next_dimension;
      }
    }
    first_run = false;
  }

  // return best_index;
}

// kdtree_node *kdtree_NN_non_recursive_test(kdtree &tree, float target[DATA_DIMENSION])
// {
//   if (tree.root == nullptr)
//   {
//     return nullptr;
//   }

//   std::vector<std::pair<uint, uint>> stack;
//   stack.emplace_back(0, 0); // start with root index and dimension 0 //Appends a new element to the end of the container.

//   kdtree_node *best = nullptr;
//   float best_dist = std::numeric_limits<float>::max();

//   bool first_run = true;
//   while (!stack.empty())
//   {

//     auto [index, dimension] = stack.back();
//     stack.pop_back(); // Removes the last element of the container.

//     if (index == 0 && !first_run)
//     {
//       continue;
//     }

//     kdtree_node *current_node = tree.root + index;
//     float current_dist = distance_squared(tree, current_node->data, target);
//     if (current_dist < best_dist)
//     {
//       best_dist = current_dist;
//       best = current_node;
//     }
//     uint next_dimension = (dimension + 1) % tree.dim;
//     uint index_next, index_other;

//     if (target[dimension] < current_node->data[dimension])
//     {
//       index_next = current_node->left_index;
//       index_other = current_node->right_index;
//     }
//     else
//     {
//       index_next = current_node->right_index;
//       index_other = current_node->left_index;
//     }

//     // Check if the other side should also be searched
//     float plane_dist = target[dimension] - current_node->data[dimension];
//     plane_dist *= plane_dist;

//     if (plane_dist < best_dist && index_other != 0)
//     {
//       stack.emplace_back(index_other, next_dimension);
//     }
//     if (index_next != 0)
//     {
//       stack.emplace_back(index_next, next_dimension);
//     }
//     first_run = false;
//   }

//   return best;
// }

void HLS_equivalent_NN(float *query_point, uint starting_index, uint dimension_to_compare, uint &closest_index, float *kdtree_nodes_data_FPGA, uint *kdtree_nodes_indexes_FPGA)
{
  // closest_index = staring_index;
  // return;
  std::cout << "starting_index: " << starting_index << std::endl;
  if (kdtree_nodes_indexes_FPGA[starting_index * INDEX_DIMENSION_FPGA + LEFT_INDEX] == 0 &&
      kdtree_nodes_indexes_FPGA[starting_index * INDEX_DIMENSION_FPGA + RIGHT_INDEX] == 0)
  {
    closest_index = starting_index;
    return;
  }

  uint index_next, index_other;
  if (query_point[dimension_to_compare] < kdtree_nodes_data_FPGA[starting_index * TMP_DIM + dimension_to_compare])
  {
    index_next = kdtree_nodes_indexes_FPGA[starting_index * INDEX_DIMENSION_FPGA + LEFT_INDEX];
    index_other = kdtree_nodes_indexes_FPGA[starting_index * INDEX_DIMENSION_FPGA + RIGHT_INDEX];
  }
  else
  {
    index_next = kdtree_nodes_indexes_FPGA[starting_index * INDEX_DIMENSION_FPGA + RIGHT_INDEX];
    index_other = kdtree_nodes_indexes_FPGA[starting_index * INDEX_DIMENSION_FPGA + LEFT_INDEX];
  }

  if (index_next == 0)
  {
    closest_index = starting_index;
    return;
  }

  uint tmp_index = 0;
  HLS_equivalent_NN(query_point, index_next, (dimension_to_compare + 1) % 3, tmp_index, kdtree_nodes_data_FPGA, kdtree_nodes_indexes_FPGA);

  uint best = 0;
  float dist_tmp = 0;
  float dist_current = 0;
  HLS_distance_squared(query_point, &kdtree_nodes_data_FPGA[tmp_index * TMP_DIM], dist_tmp);
  HLS_distance_squared(query_point, &kdtree_nodes_data_FPGA[starting_index * TMP_DIM], dist_current);
  if (dist_tmp < dist_current)
  {
    best = tmp_index;
  }
  else
  {
    best = starting_index;
  }

  float r = 0;
  HLS_distance_squared(query_point, &kdtree_nodes_data_FPGA[best * TMP_DIM], r);

  float r_mark = (query_point[dimension_to_compare] - kdtree_nodes_data_FPGA[starting_index * TMP_DIM + dimension_to_compare]); //! should maybe include following *(query_point[dimension_to_compare]-kdtree_nodes_FPGA[staring_index * DATA_DIMENSION + dimension_to_compare]);

  if (r_mark * r_mark < r && index_other != 0)
  {
    HLS_equivalent_NN(query_point, index_other, (dimension_to_compare + 1) % TMP_DIM, tmp_index, kdtree_nodes_data_FPGA, kdtree_nodes_indexes_FPGA);
    float tmp_dist;
    HLS_distance_squared(query_point, &kdtree_nodes_data_FPGA[tmp_index * TMP_DIM], tmp_dist);
    if (tmp_dist < r)
    {
      closest_index = tmp_index;
    }
  }
  return;
}

void HLS_get_next_index(float *data_arr, uint *index_arr, float *query_point, uint current_index, uint dimension_to_compare, uint &next_index, uint &other_index)
{
  // check if both left and right are 0
  if (index_arr[current_index * INDEX_DIMENSION_FPGA + LEFT_INDEX] == 0 && index_arr[current_index * INDEX_DIMENSION_FPGA + RIGHT_INDEX] == 0)
  {
    std::cout << "both left and right are 0" << std::endl;
    next_index = current_index;
    return;
  }

  if (data_arr[current_index * TMP_DIM + dimension_to_compare] > query_point[dimension_to_compare])
  {
    next_index = index_arr[current_index * INDEX_DIMENSION_FPGA + LEFT_INDEX];
    other_index = index_arr[current_index * INDEX_DIMENSION_FPGA + RIGHT_INDEX];
  }
  else
  {
    next_index = index_arr[current_index * INDEX_DIMENSION_FPGA + RIGHT_INDEX];
    other_index = index_arr[current_index * INDEX_DIMENSION_FPGA + LEFT_INDEX];
  }

  if (next_index == 0)
  {
    std::cout << "next_index is 0" << std::endl;
    next_index = current_index;
    return;
  }
  std::cout << "next_index: " << next_index << std::endl;
  std::cout << "other_index: " << other_index << std::endl;
}

// index is not the original index
void kdtree_NN_non_recursive(float *data_arr, uint *index_arr, uint n_points, float *target, uint *result_index)
{
  std::cout << "running kdtree_NN_non_recursive" << std::endl;
  uint indexes_to_check[n_points];
  uint n_indexes_to_check = 0;
  uint current_index = 0;
  uint dimension_to_compare = 0;
  uint next_index = 0, other_index = 0;
  bool end_found = false; // without going back and checking if the other branch could contain a closer point

  // seach to the end of the tree
  while (!end_found)
  {
    current_index = next_index;
    indexes_to_check[n_indexes_to_check] = current_index;
    n_indexes_to_check++;

    std::cout << "current_index: " << current_index << std::endl;
    HLS_get_next_index(data_arr, index_arr, target, current_index, dimension_to_compare, next_index, other_index);
    if (current_index == next_index) // no new closer point were found
    {
      std::cout << "no new closer point were found" << std::endl;
      end_found = true;
      break;
    }

    dimension_to_compare = (dimension_to_compare + 1) % TMP_DIM;
  }

  // find closest point in the indexes_to_check
  float sqrt_dist_to_best = 0;
  HLS_distance_squared(target, &data_arr[indexes_to_check[0] * TMP_DIM], sqrt_dist_to_best);
  uint best_index = 0;

  float sqrt_dist_to_tmp = 0;

  for (uint i = 1; i < n_indexes_to_check; i++)
  {
    HLS_distance_squared(target, &data_arr[indexes_to_check[i] * TMP_DIM], sqrt_dist_to_tmp);
    if (sqrt_dist_to_tmp < sqrt_dist_to_best)
    {
      best_index = indexes_to_check[i];
      sqrt_dist_to_best = sqrt_dist_to_tmp;
    }
  }

  // get all nodes in the other branch
  other_index = index_arr[best_index * INDEX_DIMENSION_FPGA + LEFT_INDEX];
  indexes_to_check[0] = best_index; // replace the first index with the best index and use from 1 and down to check the other branch
  end_found = false;
  next_index = other_index;
  while (!end_found)
  {
    current_index = next_index;
  }

  // std::cout << "closest_index(current_index): " << current_index << std::endl;
  // result_index[0] = current_index;
  // std::cout << "result_index: " << result_index[0] << std::endl;
  // // check other branch after the closest point is found

  // // check if distance to other branch is closer than distance to current closest point
  // uint closest_index = current_index;
  // if (other_index != 0)
  // {
  //   float sqrt_dist_to_best = 0;
  //   HLS_distance_squared(target, &data_arr[closest_index * TMP_DIM], sqrt_dist_to_best);
  //   float tmp_dist_to_other_branch = target[dimension_to_compare] - data_arr[other_index * TMP_DIM + dimension_to_compare];
  //   float sqrt_dist_to_other = tmp_dist_to_other_branch * tmp_dist_to_other_branch;
  //   std::cout << "sqrt_dist_to_best: " << sqrt_dist_to_best << std::endl;
  //   std::cout << "sqrt_dist_to_other: " << sqrt_dist_to_other << std::endl;

  //   if (sqrt_dist_to_other < sqrt_dist_to_best)
  //   {
  //     std::cout << "checking other branch" << std::endl;
  //     current_index = other_index;

  //   }
  // }
}

void load_tree_to_arrays(kdtree &tree, float *point_arr, uint *index_arr)
{
  // kd_tree_node *node;
  kdtree_node *node = tree.root;
  for (uint i = 0; i < tree.number_of_nodes; i++)
  {
    node = tree.root + i;
    // if (i != 0)
    // {
    //   std::cout << "tmp_node2 arr out: " << i - 1 << std::endl;
    //   std::cout << "point: " << point_arr[(i - 1) * tree.dim] << ", " << point_arr[(i - 1) * tree.dim + 1] << ", " << point_arr[(i - 1) * tree.dim + 2] << std::endl;
    //   std::cout << "indexes: " << index_arr[(i - 1) * N_CHILDS] << ", " << index_arr[(i - 1) * N_CHILDS + 1] << ", " << index_arr[(i - 1) * N_CHILDS + 2] << std::endl;
    //   std::cout << std::endl;
    // }

    point_arr[i * tree.dim + X_index] = node->data[X_index];
    point_arr[i * tree.dim + Y_index] = node->data[Y_index];
    point_arr[i * tree.dim + Z_index] = node->data[Z_index];
    index_arr[i * INDEX_DIMENSION_FPGA + LEFT_INDEX] = node->left_index;
    index_arr[i * INDEX_DIMENSION_FPGA + RIGHT_INDEX] = node->right_index;
    // index_arr[i * N_INDEXES + LEFT_INDEX] = 420+i;
    // index_arr[i * N_INDEXES + RIGHT_INDEX] = 6969+i;
    index_arr[i * INDEX_DIMENSION_FPGA + PARENT_INDEX] = node->parent_index;
    // index_arr[i * N_INDEXES + ORIGINAL_INDEX] = node->orginal_index;
    // std::cout << i << "indexes: left: " << index_arr[i * N_INDEXES + LEFT_INDEX] << ", right: " << index_arr[i * N_INDEXES + RIGHT_INDEX]
    // << ", parent: " << index_arr[i * N_INDEXES + PARENT_INDEX] << ", original: " << index_arr[i * N_INDEXES + ORIGINAL_INDEX] << std::endl;
    // std::cout << "node in: " << i << std::endl;
    // std::cout << "point: " << node->data[X_index] << ", " << node->data[Y_index] << ", " << node->data[Z_index] << std::endl;
    // std::cout << "indexes: " << node->left_index << ", " << node->right_index << ", " << node->parent_index << std::endl;
    // std::cout << "node arr out: " << i << std::endl;
    // std::cout << "point: " << point_arr[i * tree.dim] << ", " << point_arr[i * tree.dim + 1] << ", " << point_arr[i * tree.dim + 2] << std::endl;
    // std::cout << "indexes: " << index_arr[i * N_CHILDS] << ", " << index_arr[i * N_CHILDS + 1] << ", " << index_arr[i * N_CHILDS + 2] << std::endl;

    // std::cout << "node arr out: " << i << std::endl;
    // std::cout << "point: " << point_arr[i * tree.dim] << ", " << point_arr[i * tree.dim + 1] << ", " << point_arr[i * tree.dim + 2] << std::endl;
    // std::cout << "indexes: " << index_arr[i * N_CHILDS] << ", " << index_arr[i * N_CHILDS + 1] << ", " << index_arr[i * N_CHILDS + 2] << std::endl;
    // std::cout << std::endl;
  }
  // std::cout << "print after" << std::endl;
  // std::cout << "size of uint: " << sizeof(uint) << std::endl;
  // for (uint i = 0; i < tree.number_of_nodes; i++)
  // {
  //   std::cout << i << "indexes: left: " << index_arr[i * N_INDEXES + LEFT_INDEX] << ", right: " << index_arr[i * N_INDEXES + RIGHT_INDEX] << ", parent: " << index_arr[i * N_INDEXES + PARENT_INDEX] << ", original: " << index_arr[i * N_INDEXES + ORIGINAL_INDEX] << std::endl;
  // }
}

// uint get_org_index_from_index(kdtree &tree, uint index)
// {
//   return tree.root[index].data[ORIGINAL_INDEX_index];
// }

void kdtree_print_node_info(kdtree &tree, uint index)
{
  kdtree_node *node = tree.root + index;
  std::cout << "data: " << node->data[0] << ", " << node->data[1] << ", " << node->data[2] << std::endl;
  std::cout << "original_index: " << node->orginal_index << std::endl;
  std::cout << "parent_index: " << node->parent_index << std::endl;
  std::cout << "left_index: " << node->left_index << std::endl;
  std::cout << "right_index: " << node->right_index << std::endl;
  std::cout << "index: " << index << std::endl;
}

void kdtree_print_node_info(kdtree &tree, kdtree_node *node)
{
  std::cout << "data: " << node->data[0] << ", " << node->data[1] << ", " << node->data[2] << std::endl;
  std::cout << "original_index: " << node->orginal_index << std::endl;
  std::cout << "parent_index: " << node->parent_index << std::endl;
  std::cout << "left_index: " << node->left_index << std::endl;
  std::cout << "right_index: " << node->right_index << std::endl;
}

// print the depth of all end nodes
void kdtree_print_depth(kdtree &tree)
{
  for (uint i = 0; i < tree.number_of_nodes; i++)
  {
    kdtree_node *node = tree.root + i;
    if (node->left_index == 0 && node->right_index == 0)
    {
      int depth = 0;
      while (node->parent_index != 0)
      {
        depth++;
        node = tree.root + node->parent_index;
      }
      std::cout << "Node " << i << " is at depth " << depth << std::endl;
    }
  }
}

void kdtree_print_all_nodes(kdtree &tree)
{
  for (uint i = 0; i < tree.number_of_nodes; i++)
  {
    kdtree_print_node_info(tree, i);
    std::cout << std::endl;
  }
}

// deletes the tree
void kdtree_free(kdtree *tree)
{
  free(tree->root);
}