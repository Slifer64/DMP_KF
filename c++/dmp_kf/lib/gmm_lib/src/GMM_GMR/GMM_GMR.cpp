/**
 * Copyright (C) 2017 GMM_GMR
 */

#include <GMM_lib/GMM_GMR/GMM_GMR.h>

namespace as64_
{

GMM_GMR::GMM_GMR():realmin(1e-300)
{}

bool GMM_GMR::set_params(const GMM_GMR_PARAMS *params)
{
	const GMM_GMR_PARAMS *p = dynamic_cast<const GMM_GMR_PARAMS *>(params);
	if (p == 0) return false;
	
	GMR_params.Priors = p->Priors;
	GMR_params.Mu = p->Mu;
	GMR_params.Sigma = p->Sigma;
	
	calc_meta_params();
	
	return true;
}

bool GMM_GMR::get_params(GMM_GMR_PARAMS *params) const
{
	GMM_GMR_PARAMS *p = dynamic_cast<GMM_GMR_PARAMS *>(params);
	if (p == 0) return false;
	
	p->Priors = GMR_params.Priors;
	p->Mu = GMR_params.Mu;
	p->Sigma = GMR_params.Sigma;
	
	return true;
}
  
bool GMM_GMR::load_params(const std::string &filename)
{
	std::ifstream in(filename.c_str(), std::ios::in);
	if (!in) return false;

	int d, K;

	in >> d >> K;
	d = d/2;

	GMR_params.Priors.resize(K);
	for (int k=0;k<K;k++) in >> GMR_params.Priors(k);

	GMR_params.Mu.resize(K);
	for (int k=0;k<K;k++){
		GMR_params.Mu[k].resize(2*d);
		for (int i=0;i<2*d;i++) in >> GMR_params.Mu[k](i);
	}

	GMR_params.Sigma.resize(K);
	for (int k=0;k<K;k++){
		GMR_params.Sigma[k].resize(2*d,2*d);
		for (int j=0;j<2*d;j++){
			for (int i=0;i<2*d;i++) in >> GMR_params.Sigma[k](i,j);
		}
	}

	calc_meta_params();
	
	return true;
}

bool GMM_GMR::save_params(const std::string &filename, int  precision) const
{
	std::ofstream out(filename.c_str());
	if (!out) return false;
	out.precision(precision);


	int n = GMR_params.Mu[0].size();
	int K = GMR_params.Priors.size();

	out << n << " " << K << std::endl;

	for (int k=0;k<K;k++) out << GMR_params.Priors(k) << std::endl;
	for (int k=0;k<K;k++){
		for (int i=0;i<n;i++) out << GMR_params.Mu[k](i) << " ";
		out << std::endl;
	}
	for (int k=0;k<K;k++){
		for (int j=0;j<n;j++){
			for (int i=0;i<n;i++) out << GMR_params.Sigma[k](i,j) << " ";
		}
		out << std::endl;
	}

	return true;
}

void GMM_GMR::calc_meta_params()
{
	int K = GMR_params.Priors.size();
	int d = GMR_params.Mu[0].size()/2;
	
	inv_Sigma_in.resize(K);
	det_Sigma_in.resize(K);
	A.resize(K);

	for (int k=0;k<K;k++){
		Eigen::MatrixXd Sigma_in_k = GMR_params.Sigma[k].block(0,0,d,d);
		double det_Sigma_in_k = Sigma_in_k.determinant();
		if (det_Sigma_in_k == 0) std::cerr << "** WARNING **\n det(Sigma[" << k << "] = " << det_Sigma_in_k << "\n";
		inv_Sigma_in[k] = Sigma_in_k.inverse();
		det_Sigma_in[k] = det_Sigma_in_k;
		A[k] = GMR_params.Sigma[k].block(d,0,d,d) * inv_Sigma_in[k];
	}
}

void GMM_GMR::get_output(const Eigen::MatrixXd &X_in, Eigen::MatrixXd *X_out) const
{
	int nData = X_in.cols();
	int nVar = X_in.rows();
	int nStates = GMR_params.Mu.size();
	
	*X_out = Eigen::MatrixXd::Zero(nVar, nData);
	
	Eigen::MatrixXd h(nData,nStates);
	for (int k=0;k<nStates;k++){
		h.col(k) = GMR_params.Priors(k) * gaussPDF(X_in, GMR_params.Mu[k].segment(0,nVar), inv_Sigma_in[k], det_Sigma_in[k]);
	}
	//h = h.cwiseQuotient( (h.rowwise().sum().array() + realmin).matrix().replicate(1, nStates) );
	Eigen::Matrix<double, Eigen::Dynamic, 1> h_sum = h.rowwise().sum().array() + realmin;

	for (int i=0;i<nData;i++){
		for (int k=0;k<nStates;k++)
			X_out->col(i) += (h(k)/h_sum(i)) * ( A[k]*(X_in.col(i)-GMR_params.Mu[k].segment(0,nVar)) + GMR_params.Mu[k].segment(nVar,nVar) );
	}
}

Eigen::VectorXd GMM_GMR::gaussPDF(const Eigen::MatrixXd &X, const Eigen::VectorXd &Mu, const Eigen::MatrixXd &Sigma) const
{
	Eigen::MatrixXd inv_Sigma = Sigma.inverse();
	double det_Sigma = Sigma.determinant();
	
	return gaussPDF(X, Mu, inv_Sigma, det_Sigma);
}

Eigen::VectorXd GMM_GMR::gaussPDF(const Eigen::MatrixXd &X, const Eigen::VectorXd &Mu, const Eigen::MatrixXd &inv_Sigma, double det_Sigma) const
{
	int nVar = X.rows();
	int nData = X.cols();

	Eigen::MatrixXd X_temp = X.transpose() - Mu.transpose().replicate(nData, 1);
	Eigen::VectorXd prob = ( -0.5 * ( (X_temp*inv_Sigma).array() * X_temp.array() ).rowwise().sum()).exp();

	return prob/std::sqrt( std::pow(2*M_PI,nVar)*(std::fabs(det_Sigma) + realmin));

}

  
}  // namespace as64_
