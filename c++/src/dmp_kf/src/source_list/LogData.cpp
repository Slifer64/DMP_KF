#include <dmp_kf/LogData.h>
#include <dmp_kf/utils.h>
#include <iomanip>
#include <ros/package.h>

std::string getTimeStamp()
{
	std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	std::ostringstream out_s;
	out_s << std::ctime(&t);
	std::string time_stamp = out_s.str();
	time_stamp.pop_back();
	for (int i=0;i<time_stamp.size();i++)
	{
		if (time_stamp[i]==' ') time_stamp[i] = '_';
		else if (time_stamp[i]==':') time_stamp[i] = '-';
	}

	return time_stamp;
}

template <typename T>
void write_scalar(T scalar, std::ostream &out = std::cout, bool binary = false, int precision = 6)
{
  if (binary) out.write((const char *)(&scalar), sizeof(scalar));
  else out << std::setprecision(precision) << scalar;
}

void write_mat(const arma::mat &m, int n_rows, int n_cols, std::ostream &out, bool binary, int precision)
{
  if (binary)
  {
    double *buff = new double[n_rows*n_cols];

     int k=0;
     for (int i=0;i<n_rows;i++){
       for (int j=0;j<n_cols;j++) buff[k++] = m(i,j);
     }
     out.write((const char *)(buff), n_rows*n_cols*sizeof(double));

     delete []buff;
  }
  else
  {
    for (int i=0;i<n_rows;i++)
    {
      for (int j=0;j<n_cols;j++) out << std::setprecision(precision) << m(i,j) << " ";
      out << "\n";
    }
  }

}

void write_mat(const arma::mat &m, std::ostream &out, bool binary, int precision)
{
  long n_rows = m.n_rows;
  long n_cols = m.n_cols;

  write_scalar(n_rows, out, binary);
  if (!binary) out << "\n";
  write_scalar(n_cols, out, binary);
  if (!binary) out << "\n";

  write_mat(m, n_rows, n_cols, out, binary, precision);
}

LogData::LogData(const std::shared_ptr<Robot> &r, const std::shared_ptr<Controller> &c):robot(r),controller(c)
{}

void LogData::init()
{
  std::string path_to_data = ros::package::getPath(PACKAGE_NAME)+ "/data/";

  binary = true;
  data_input_path = path_to_data;
  data_output_path = path_to_data;
  in_data_filename = data_input_path + "in_data";
  out_data_filename = data_output_path + "out_data";;

  clear();
}

void LogData::log()
{
  Time_data = arma::join_horiz( Time_data, arma::mat({controller->t}) );

  Y_data = arma::join_horiz(Y_data, controller->Y);
  dY_data = arma::join_horiz(dY_data, controller->dY);
  ddY_data = arma::join_horiz(ddY_data, controller->ddY);

  Fext_data = arma::join_horiz(Fext_data, (robot->getTaskWrench()).subvec(0,2));
  Fext_filt_data = arma::join_horiz(Fext_filt_data, controller->f_ext);

	arma::vec Sigma_theta = arma::sqrt(arma::diagvec(controller->P_theta));

	g_hat_data = arma::join_horiz(g_hat_data, controller->g_hat);
	Sigma_g_hat_data = arma::join_horiz(Sigma_g_hat_data, Sigma_theta.subvec(0,2));
	tau_hat_data = arma::join_horiz(tau_hat_data, arma::mat({controller->tau_hat}));
	sigma_tau_hat_data = arma::join_horiz(sigma_tau_hat_data, arma::mat({Sigma_theta(3)}) );
	mf_data = arma::join_horiz(mf_data, arma::mat({controller->mf}));
}

void LogData::clear()
{
  Time_data.clear();

  Y_data.clear();
  dY_data.clear();
  ddY_data.clear();

  Fext_data.clear();
  Fext_filt_data.clear();

	g_hat_data.clear();
  Sigma_g_hat_data.clear();
  tau_hat_data.clear();
  sigma_tau_hat_data.clear();
	mf_data.clear();
}

void LogData::save()
{
  static long counter = 0;
  int precision = 7;

  std::ostringstream o_str;
  o_str << counter;
  std::string suffix = "_" + getTimeStamp();
  std::string filename = out_data_filename + suffix;
  counter++;

  std::string file_ext = binary?".bin":".txt";
  std::string file = filename + file_ext;

  std::cout << "file_ext = " << file_ext << "\n";
  std::cout << "filename = " << filename << "\n";
  std::cout << "file = " << file << "\n";

  std::ofstream out;
  if (binary) out.open(file, std::ios::binary);
  else out.open(file);
  if (!out) throw std::ios_base::failure(std::string("Couldn't create file: \"") + file_ext + "\"");

  write_mat(Time_data, out, binary, precision);
  write_mat(Y_data, out, binary, precision);
  write_mat(dY_data, out, binary, precision);
  write_mat(ddY_data, out, binary, precision);

  write_mat(Fext_data, out, binary, precision);
  write_mat(Fext_filt_data, out, binary, precision);

	write_mat(g_hat_data, out, binary, precision);
	write_mat(Sigma_g_hat_data, out, binary, precision);
	write_mat(tau_hat_data, out, binary, precision);
	write_mat(sigma_tau_hat_data, out, binary, precision);
	write_mat(mf_data, out, binary, precision);

	write_mat(controller->Timed, out, binary, precision);
  write_mat(controller->Yd_data, out, binary, precision);
  write_mat(controller->dYd_data, out, binary, precision);
  write_mat(controller->ddYd_data, out, binary, precision);

  out.close();
}