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

#include <armadillo>

void PRINT_INFO_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_CONFIRM_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_WARNING_MSG(const std::string &msg, std::ostream &out = std::cout);
void PRINT_ERROR_MSG(const std::string &msg, std::ostream &out = std::cout);

#endif // DMP_KF_GLOBAL_DEFS_H
