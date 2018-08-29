#include <dmp_kf/utils.h>
#include <cmath>

#include <math_lib/math_lib.h>

arma::vec getRobotState(const arma::vec &p, const arma::vec &Q)
{
  // arma::mat Htrans_r = robot->getTaskPose();
  // arma::mat R_r = Htrans_r.submat(0,0,2,2);
  // arma::vec p_r = Htrans_r.submat(0,3,1,3);
  // arma::vec Q_r = as64_::math_::rotm2quat(R_r);
  arma::vec S_r(3);
  S_r(0) = p(0);
  S_r(1) = p(2);

  arma::mat R = as64_::math_::quat2rotm(Q);
  arma::vec z = R.submat(0,2,2,2);
  arma::vec z_base = {0,0,1};
  double cos_theta = arma::dot(z_base,z);
  double theta = (cos_theta>=0?1:-1)*std::fabs(std::acos(cos_theta));
  S_r(2) = theta;
  // S_r(2) = Q(3);

  return S_r;
}

void PRINT_INFO_MSG(const std::string &msg, std::ostream &out)
{
  out << "\033[1m" << "\033[34m" << "[INFO]: " << msg << "\033[0m";
}

void PRINT_CONFIRM_MSG(const std::string &msg, std::ostream &out)
{
  std::cout << "\033[1m" << "\033[32m" << "[INFO]: " << msg << "\033[0m";
}

void PRINT_WARNING_MSG(const std::string &msg, std::ostream &out)
{
  std::cout << "\033[1m" << "\033[33m" << "[WARNING]: " << msg << "\033[0m";
}

void PRINT_ERROR_MSG(const std::string &msg, std::ostream &out)
{
  std::cout << "\033[1m" << "\033[31m" << "[ERROR]: " << msg << "\033[0m";
}
