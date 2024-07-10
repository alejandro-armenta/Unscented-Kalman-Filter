#ifndef UKF_SENSOR_CORRECTION_H
#define UKF_SENSOR_CORRECTION_H

#include "Eigen/Dense"
#include <iostream>

using namespace Eigen;
using namespace std;

class ukf_sensor_correction
{
  public:

  enum sensor_type
  {
    SENSOR_RADAR,
    SENSOR_LIDAR,
  };

  //Tengo dos estados a la vez y eso esta mal!
  
  void CorrectState(const MatrixXd & Xsig_pred);

  void Initialize(int n_x, const MatrixXd & R_, ukf_sensor_correction::sensor_type SensorType, int SigmaPointCount);

  ukf_sensor_correction & operator=(const ukf_sensor_correction & other);

  int n_z;

  sensor_type Type;

  MatrixXd Zsig;

  //Predicted Measurement Mean!
  VectorXd z_pred;

  //Predicted Measurement Covariance;
  MatrixXd S;

    
   MatrixXd R;

  VectorXd z;

  int sigmaPointCount;
  
  MatrixXd Tc;

};

#endif