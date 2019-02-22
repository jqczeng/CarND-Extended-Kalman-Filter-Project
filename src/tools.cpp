#include "tools.h"
#include <iostream>
#include <cmath>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */

	VectorXd rmse(4);
	rmse << 0,0,0,0;

	  // TODO: YOUR CODE HERE
	  // check the validity of the following inputs:
	  //  * the estimation vector size should not be zero
	  //  * the estimation vector size should equal ground truth vector size
	  if (estimations.size() == 0 || estimations.size() != ground_truth.size()){
	    cout << "Error: Estimations Size Wrong or Ground Truth Size Wrong" << endl;
	  }

	  // Accumulate squared residuals
	  for (int i=0; i < estimations.size(); ++i) {
	    VectorXd residual= estimations[i] - ground_truth[i];
	    residual = residual.array().pow(2);
	    rmse += residual;
	  }

	  // Calculate the mean
	  rmse = rmse / estimations.size();

	  // Calculate the squared root
	  rmse = rmse.array().sqrt();


	  // return the result
	  return rmse;


}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	MatrixXd Hj(3,4);
	// recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// TODO: YOUR CODE HERE

	// check division by zero
	if (fabs(px*px + py+py) < 0.0001) {
	    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
	    return Hj;
	} else {
		// compute the Jacobian matrix
		float Hj_00 = px / sqrt(px*px + py*py);
		float Hj_01 = py / sqrt(px*px + py*py);
		float Hj_02 = 0;
		float Hj_03 = 0;
		float Hj_10 = -py / (px*px + py*py);
		float Hj_11 = px / (px*px + py*py);
		float Hj_12 = 0;
		float Hj_13 = 0;
		float Hj_20 = py * (vx*py - vy*px) / pow(px*px+py*py, 3/2);
		float Hj_21 = px * (vy*px - vx*py) / pow(px*px+py*py, 3/2);
		float Hj_22 = px / sqrt(px*px + py*py);
		float Hj_23 = py / sqrt(px*px + py*py);

	Hj << Hj_00, Hj_01, Hj_02, Hj_03,
		  Hj_10, Hj_11, Hj_12, Hj_13,
		  Hj_20, Hj_21, Hj_22, Hj_23;
	}

	return Hj;
}

