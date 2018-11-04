#include <dmp_kf/LogData.h>
#include <dmp_kf/utils.h>
#include <iomanip>
#include <ros/package.h>

// =======================================
// ===========  KinematicData  ===========
// =======================================

bool KinematicData::save(const std::string &file_name, std::string &err_msg)
{
  std::string f_name = ros::package::getPath(PACKAGE_NAME)+ "/data/" + file_name;
  bool binary = true;

  std::ofstream out(f_name.c_str(), std::ios::binary);
  if (!out)
  {
    err_msg = std::string("Error saving training model data:\nCouldn't create file: \"" + f_name + "\"");
    return false;
  }

  write_mat(Time, out, binary);
  write_mat(Y_data, out, binary);
  write_mat(dY_data, out, binary);
  write_mat(ddY_data, out, binary);

  out.close();

  return true;
}

void KinematicData::log(double t, const arma::vec &Y, const arma::vec &dY, const arma::vec &ddY)
{
	Time = arma::join_horiz(Time, arma::mat({t}));
  Y_data = arma::join_horiz(Y_data, Y);
  dY_data = arma::join_horiz(dY_data, dY);
  ddY_data = arma::join_horiz(ddY_data, ddY);
}

void KinematicData::clear()
{
  Time.clear();
  Y_data.clear();
  dY_data.clear();
  ddY_data.clear();
}

void KinematicData::trim(double v_start_thres, double v_end_thres)
{
  int i_start=0;
  int i_end=dY_data.n_cols-1;
  for (int i=0; i<dY_data.n_cols; i++)
  {
    if (arma::norm(dY_data.col(i)) >= v_start_thres)
    {
      i_start = i;
      break;
    }
  }

  for (int i=dY_data.n_cols-1; i>-1; i--)
  {
    if (arma::norm(dY_data.col(i)) >= v_end_thres)
    {
      i_end = i;
      break;
    }
  }

  Time = Time.cols(i_start, i_end);
  Time -= Time(0);
  Y_data = Y_data.cols(i_start, i_end);
  dY_data = dY_data.cols(i_start, i_end);
  ddY_data = ddY_data.cols(i_start, i_end);
}

// ===========================================================
// ===========================================================

// =======================================
// ===========  ExecutionData  ===========
// =======================================

ExecutionData::ExecutionData()
{
  y_g = arma::vec().zeros(3);
}

bool ExecutionData::save(const std::string &file_name, std::string &err_msg)
{
	std::string f_name = ros::package::getPath(PACKAGE_NAME)+ "/data/" + file_name;
  bool binary = true;

  std::ofstream out(f_name.c_str(), std::ios::binary);
  if (!out)
  {
    err_msg = std::string("Error saving execution data:\nCouldn't create file: \"" + f_name + "\"");
    return false;
  }

	write_mat(Time, out, binary);

	write_mat(Y_data, out, binary);
	write_mat(dY_data, out, binary);
	write_mat(ddY_data, out, binary);

  write_mat(Y_ref_data, out, binary);
	write_mat(dY_ref_data, out, binary);
	write_mat(ddY_ref_data, out, binary);

	write_mat(Fext_data, out, binary);
	write_mat(Fext_filt_data, out, binary);

	write_mat(theta_data, out, binary);
	write_mat(Sigma_theta_data, out, binary);

  write_mat(y_g, out, binary);

	out.close();

	return true;
}

void ExecutionData::log(double t, const arma::vec &Y, const arma::vec &dY, const arma::vec &ddY,
                        const arma::vec &Y_ref, const arma::vec &dY_ref, const arma::vec &ddY_ref,
                        const arma::vec &Fext, const arma::vec &Fext_filt,
												const arma::vec &theta, const arma::mat &P_theta)
{
	Time = arma::join_horiz(Time, arma::mat({t}));

	Y_ref_data = arma::join_horiz(Y_ref_data, Y_ref);
	dY_ref_data = arma::join_horiz(dY_ref_data, dY_ref);
	ddY_ref_data = arma::join_horiz(ddY_ref_data, ddY_ref);

  Y_data = arma::join_horiz(Y_data, Y);
	dY_data = arma::join_horiz(dY_data, dY);
	ddY_data = arma::join_horiz(ddY_data, ddY);

	Fext_data = arma::join_horiz(Fext_data, Fext);
	Fext_filt_data = arma::join_horiz(Fext_filt_data, Fext_filt);

	theta_data = arma::join_horiz(theta_data, theta);
	Sigma_theta_data = arma::join_horiz(Sigma_theta_data, arma::sqrt(arma::diagvec(P_theta)));
}

void ExecutionData::clear()
{
	Time.clear();

  Y_data.clear();
  dY_data.clear();
  ddY_data.clear();

  Fext_data.clear();
  Fext_filt_data.clear();

  theta_data.clear();
  Sigma_theta_data.clear();
}


// std::string getTimeStamp()
// {
// 	std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
// 	std::ostringstream out_s;
// 	out_s << std::ctime(&t);
// 	std::string time_stamp = out_s.str();
// 	time_stamp.pop_back();
// 	for (int i=0;i<time_stamp.size();i++)
// 	{
// 		if (time_stamp[i]==' ') time_stamp[i] = '_';
// 		else if (time_stamp[i]==':') time_stamp[i] = '-';
// 	}
//
// 	return time_stamp;
// }
