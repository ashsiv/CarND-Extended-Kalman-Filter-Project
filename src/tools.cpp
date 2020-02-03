#include "tools.h"
#include <iostream>

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

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size

  if(!(estimations.size() == ground_truth.size()) || estimations.size()==0 || ground_truth.size()==0)
  {
      return rmse;
  }
  // TODO: accumulate squared residuals
  
  for (int i=0; i < estimations.size(); ++i) {
    // ... your code here
    VectorXd diff,diffsq;
    diff = estimations[i]-ground_truth[i];
    diffsq=diff.array()*diff.array();
    rmse += diffsq ;
  }

  // TODO: calculate the mean
  rmse =rmse/estimations.size();

  // TODO: calculate the squared root
  rmse =rmse.array().sqrt();
  // return the result
  return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  // TODO: YOUR CODE HERE 

  // check division by zero
  float vp;
  vp = ((px*px) + (py*py));
  if(vp==0)
  {
      cout<<"Divide by zero error";
      MatrixXd f(3,4);
                     f<< 0,0,0,0,
                        0,0,0,0,
                        0,0,0,0;
      return f;
  }
  
  
  // compute the Jacobian matrix
  
  Hj(0,0) = px/sqrt(vp);
  Hj(0,1) = py/sqrt(vp);
  Hj(0,2) = 0;
  Hj(0,3) = 0;
  
  Hj(1,0)=-py/(vp);
  Hj(1,1)=px/(vp);
  Hj(1,2) = 0;
  Hj(1,3) = 0;
  
  Hj(2,0) = (py*((vx*py) - (vy*px))/pow(vp,3/2));
  Hj(2,1) = (px*((vy*px) - (vx*py))/pow(vp,3/2));
  Hj(2,2) =px/sqrt(vp);
  Hj(2,3) = py/sqrt(vp);
  
  return Hj;
 
}
