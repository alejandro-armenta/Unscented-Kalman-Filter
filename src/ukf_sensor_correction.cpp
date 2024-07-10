#include "ukf_sensor_correction.h"


void ukf_sensor_correction::Initialize(int n_x, const MatrixXd & R_, ukf_sensor_correction::sensor_type SensorType, int SigmaPointCount)
{

  sigmaPointCount = SigmaPointCount;

  n_z = R_.rows();

  Type = SensorType;
  
  Zsig = MatrixXd(n_z, SigmaPointCount);
  Zsig.fill(0.0);

  z_pred = VectorXd(n_z);
  z_pred.fill(0.0);

  S = MatrixXd(n_z, n_z);
  S.fill(0.0);

  R = R_;

  z = VectorXd(n_z);
  z.fill(0.0);

  Tc = MatrixXd(n_x, n_z);
  Tc.fill(0.0);

  

  //cout << "Initialize ukf_sensor_correction" << endl;
  //cout << "dimensions for measurement: " << n_z << endl;
}

ukf_sensor_correction & ukf_sensor_correction::operator=(const ukf_sensor_correction & other)
{
  sigmaPointCount = other.sigmaPointCount;

  n_z = other.n_z;

  Type = other.Type;
  
  Zsig = other.Zsig;

  z_pred = other.z_pred;

  S = other.S;

  R = other.R;

  z = other.z;

  Tc = other.Tc;

  //std::cout << "operator = for sensor correction" << std::endl;

  //cout << "z: " << z << endl;

}






