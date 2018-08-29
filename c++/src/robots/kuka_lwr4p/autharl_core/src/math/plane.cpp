/*******************************************************************************
 * Copyright (c) 2016-2017 Automation and Robotics Lab, AUTh
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 ******************************************************************************/


#include <autharl_core/math/plane.h>
#include <vector>

namespace arl
{
namespace math
{
bool areCoplanar(const std::vector<KDL::Vector>& point, double epsilon)
{
  // Three and less points are always "coplanar"
  if (point.size() < 4)
  {
    return true;
  }

  // Find the normal vector of the plane defined by the first two vectors
  // (cross product)
  KDL::Vector ab = point.at(1) - point.at(0);
  ab.Normalize();
  KDL::Vector ac = point.at(2) - point.at(0);
  ac.Normalize();
  KDL::Vector ai;
  KDL::Vector normal = ab * ac;

  // Check if the rest of the vectors has zero dot product with the normal
  for (int i = 3; i < point.size(); i++)
  {
    ai = point.at(i) - point.at(0);
    ai.Normalize();
    if (fabs(dot(normal, ai)) > epsilon)
    {
      return false;
    }
  }
  return true;
}

void sortCircularOrder(const std::vector<KDL::Vector>& input, std::vector<KDL::Vector>* output)
{
  // Calculate the centroid point of the inputs
  KDL::Vector centroid(0, 0, 0);
  for (int i = 0; i < input.size(); i++)
  {
    centroid = input.at(i) + centroid;
  }
  centroid = centroid / static_cast<double>(input.size());

  // Calculate the unit direction vector from each point to the centroid
  std::vector<KDL::Vector> centroid_direction;
  for (int i = 0; i < input.size(); i++)
  {
    centroid_direction.push_back(centroid - input.at(i));
    centroid_direction.at(i).Normalize();
  }

  // Sort the centroid directions vectors based on the dot product between
  // them. Begin with the first vector. The next vector is the vector that with
  // the minimum angle (max dot product) with the first vector. The third
  // vector is the one with the min angle with the second etc.
  // Along with the centroid directions sort the actual points related to these
  // directions

  // Begin with the first centroid direction by adding it to the sorted ones
  // and removing it from the unsorted
  std::vector<KDL::Vector> sorted_centroid_direction;
  sorted_centroid_direction.push_back(*centroid_direction.begin());
  centroid_direction.erase(centroid_direction.begin());

  // Do the same with the points.
  std::vector<KDL::Vector> point = input;
  std::vector<KDL::Vector>::iterator input_temp_ptr;
  std::vector<KDL::Vector> sorted_point;
  sorted_point.push_back(*point.begin());
  point.erase(point.begin());

  // Move the centroid directions and points from the unsorted lists to the
  // sorted ones. The loop terminates when all the elements have been sorted
  // (the unsorted list is empty)
  while (!centroid_direction.empty())
  {
    double max_dot = -2;
    std::vector<KDL::Vector>::iterator sorted_point_ptr;
    std::vector<KDL::Vector>::iterator sorted_centroid_direction_ptr;
    std::vector<KDL::Vector>::iterator last_sorted_ptr = sorted_centroid_direction.end();
    last_sorted_ptr--;
    input_temp_ptr = point.begin();

    // Find the vector (and the point) in the unsorted list with the maximum
    // dot product with the last sorted element
    for (std::vector<KDL::Vector>::iterator remain = centroid_direction.begin();
         remain != centroid_direction.end();
         remain++)
    {
      double dot_product = dot(*last_sorted_ptr, *remain);
      if (dot_product > max_dot)
      {
        max_dot = dot_product;
        sorted_point_ptr = input_temp_ptr;
        sorted_centroid_direction_ptr = remain;
      }
      input_temp_ptr++;
    }
    sorted_point.push_back(*sorted_point_ptr);
    sorted_centroid_direction.push_back(*sorted_centroid_direction_ptr);
    centroid_direction.erase(sorted_centroid_direction_ptr);
    point.erase(sorted_point_ptr);
  }
  *output = sorted_point;
}
}  // namespace math
}  // namespace arl
