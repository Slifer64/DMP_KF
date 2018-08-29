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

#include <autharl_core/math/rotation.h>

namespace arl
{
namespace math
{
bool rotationHasUnitColumns(const KDL::Rotation &rot, double eps)
{
  if (fabs(rot.UnitX().Norm() - 1) > eps)
  {
    return false;
  }
  if (fabs(rot.UnitY().Norm() - 1) > eps)
  {
    return false;
  }
  if (fabs(rot.UnitZ().Norm() - 1) > eps)
  {
    return false;
  }
  return true;
}

bool rotationHasOrthogonalColumns(const KDL::Rotation &rot, double eps)
{
  if (fabs(dot(rot.UnitX(), rot.UnitY()) - 0) > eps)
  {
    return false;
  }
  if (fabs(dot(rot.UnitX(), rot.UnitZ()) - 0) > eps)
  {
    return false;
  }
  if (fabs(dot(rot.UnitZ(), rot.UnitY()) - 0) > eps)
  {
    return false;
  }
  return true;
}

bool rotationIsRightHanded(const KDL::Rotation &rot, double eps)
{
  KDL::Vector cross;
  cross = rot.UnitX() * rot.UnitY();
  for (int i = 0; i < 3; i++)
  {
    if (fabs(cross.data[i] - rot.UnitZ().data[i]) > eps)
    {
      return false;
    }
  }
  cross = rot.UnitZ() * rot.UnitX();
  for (int i = 0; i < 3; i++)
  {
    if (fabs(cross.data[i] - rot.UnitY().data[i]) > eps)
    {
      return false;
    }
  }
  cross = rot.UnitY() * rot.UnitZ();
  for (int i = 0; i < 3; i++)
  {
    if (fabs(cross.data[i] - rot.UnitX().data[i]) > eps)
    {
      return false;
    }
  }
  return true;
}

bool rotationIsOk(const KDL::Rotation &rot, double eps)
{
  if (rotationHasUnitColumns(rot, eps) &&
      rotationHasOrthogonalColumns(rot, eps) &&
      rotationIsRightHanded(rot, eps))
  {
    return true;
  }
  return false;
}
}  // namespace math
}  // namespace arl
