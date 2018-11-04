#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <functional>
#include <armadillo>

using namespace std::placeholders;

typedef arma::vec vec;

class EKF
{
public:
  EKF() {}

  EKF(vec (*fn)(const vec &theta, void *cookie))
  {
    fb = std::bind(fn, _1, _2);
  }

  template<class T>
  EKF(vec (T::*fn)(const vec &theta, void *cookie), T *obj_ptr)
  {
    fb = std::bind(fn, obj_ptr, _1, _2);
  }

  void correct(const vec &z, void *cookie=NULL)
  {
    vec z_hat = fb(this->theta, cookie);
    theta = theta + 0.15*(z-z_hat);
  }

  void setMsrFunPtr(vec (*fn)(const vec &theta, void *cookie))
  {
    fb = std::bind(fn, _1, _2);
  }

  template<class T>
  void setMsrFunPtr(vec (T::*fn)(const vec &theta, void *cookie), T *obj_ptr)
  {
    fb = std::bind(fn, *obj_ptr, _1, _2);
  }

  vec theta;
  std::function<vec(const vec &theta, void *cookie)> fb;
};

class A
{
public:
  int a;
  A()
  {
    fprintf(stdout, "Constructor: this: %p\n", this);
  }
  vec msrFun(const vec &theta, void *cookie=NULL)
  {
    fprintf(stdout, "msrFun: this: %p\n", this);
    std::cout << "a = " << a << "\n";
    return (2*a*theta-1);
  }
};

int main(int argc, char** argv)
{
  A a;
  a.a = 1;

  vec theta = {10, 10, 10};
  std::shared_ptr<EKF> ekf;
  ekf.reset(new EKF(&A::msrFun, &a));
  // EKF ekf();
  // ekf.setMsrFunPtr(&A::msrFun, &a);
  ekf->theta = {20, 20, 20};

  void *cookie;
  for (int k=1;k<=5;k++)
  {
    vec z = a.msrFun(theta);
    ekf->correct(z);

    vec z_hat = a.msrFun(ekf->theta);

    std::cout << "theta = " << ekf->theta.t() << "\n";
  }

  return 0;
}
