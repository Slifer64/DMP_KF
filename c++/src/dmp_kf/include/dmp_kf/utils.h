#ifndef DMP_KF_GLOBAL_DEFS_H
#define DMP_KF_GLOBAL_DEFS_H

#define PACKAGE_NAME "dmp_kf"
#define N_DOFS 3
#define grav 9.81
// #define CATCH_EXCEPTIONS

#include <iostream>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <fstream>
#include <string>
#include <iomanip>

#include <armadillo>

void PRINT_INFO_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_CONFIRM_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_WARNING_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_ERROR_MSG(const std::string &msg, std::ostream &out = std::cout);

template <typename T>
void write_scalar(T scalar, std::ostream &out = std::cout, bool binary = false, int precision = 6)
{
  if (binary) out.write((const char *)(&scalar), sizeof(scalar));
  else out << std::setprecision(precision) << scalar;
}

void write_mat(const arma::mat &m, int n_rows, int n_cols, std::ostream &out, bool binary, int precision);
void write_mat(const arma::mat &m, std::ostream &out, bool binary = false, int precision = 6);

template <typename T>
void read_scalar(T &scalar, std::istream &in = std::cin, bool binary = false)
{
  if (binary) in.read((char *)(&scalar), sizeof(scalar));
  else in >> scalar;
}

void read_mat(arma::mat &m, long n_rows, long n_cols, std::istream &in = std::cin, bool binary = false);
void read_mat(arma::mat &m, std::istream &in = std::cin, bool binary = false);

#endif // DMP_KF_GLOBAL_DEFS_H
