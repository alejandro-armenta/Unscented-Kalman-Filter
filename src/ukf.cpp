#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*

Initialize everything to zero!

Check RMSE convergence!

CTRV Motion Model

RMSE error for each car!

X,
Y,
Vx,
Vy

*/

/**
 * Initializes Unscented Kalman filter
 */

/*
It is missing operator asignment overload!
*/
UKF::UKF() {

  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  //x_ = VectorXd(5);

  // initial covariance matrix
  //P_ = MatrixXd(5, 5);

  n_x_ = 5;

  n_z_radar = 3;

  n_z_lidar = 2;

  // Augmented state dimension
  n_aug_ = 7;

  lambda_ = 3 - n_x_;

  sigmaPointCount = 2 * n_aug_ + 1;

  



  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 5.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 5.0;

  VectorXd noise(2);
  noise << std_a_ * std_a_, std_yawdd_ * std_yawdd_;
  
  //Q_.fill(0.0);
  Q_ = noise.asDiagonal();

  x_ = VectorXd(n_x_);
  x_.fill(0.0);

  P_ = MatrixXd(n_x_, n_x_);
  P_.fill(0.0);

  P_.setIdentity();

  // create augmented mean vect
  x_aug = VectorXd(n_aug_);
  x_aug.fill(0.0);

  // create augmented state covariance
  P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);

  Xsig_aug_ = MatrixXd(n_aug_, sigmaPointCount);
  Xsig_aug_.fill(0.0);

  Xsig_pred_ = MatrixXd(n_x_, sigmaPointCount);
  Xsig_pred_.fill(0.0);

  weights_ = VectorXd(sigmaPointCount);
  weights_.fill(0.0);  

  weights_[0] = lambda_ / (lambda_ + n_aug_);
  
  for(int i = 1; i < sigmaPointCount; ++i)
  {
      weights_[i] = 1.0/(2*(lambda_ + n_aug_));
  }
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */

  R_radar = MatrixXd::Identity(n_z_radar, n_z_radar);
  R_radar(0,0) = std_radr_ * std_radr_;
  R_radar(1,1) = std_radphi_ * std_radphi_;
  R_radar(2,2) = std_radrd_ * std_radrd_;

  R_lidar = MatrixXd::Identity(n_z_lidar, n_z_lidar);
  R_lidar(0,0) = std_laspx_ * std_laspx_;
  R_lidar(1,1) = std_laspy_ * std_laspy_;
  

  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  
  lidarCorrection.Initialize(n_x_, R_lidar, ukf_sensor_correction::sensor_type::SENSOR_LIDAR, sigmaPointCount);

  radarCorrection.Initialize(n_x_, R_radar, ukf_sensor_correction::sensor_type::SENSOR_RADAR, sigmaPointCount);

}

UKF & UKF::operator=(const UKF & other)
{
  is_initialized_ = other.is_initialized_;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = other.use_laser_;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = other.use_radar_;

  // initial state vector
  //x_ = VectorXd(5);

  // initial covariance matrix
  //P_ = MatrixXd(5, 5);

  n_x_ = other.n_x_;

  n_z_radar = other.n_z_radar;

  n_z_lidar = other.n_z_lidar;

  // Augmented state dimension
  n_aug_ = other.n_aug_;

  lambda_ = other.lambda_;

  sigmaPointCount = other.sigmaPointCount;

  



  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = other.std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = other.std_yawdd_;
    
  Q_ = other.Q_;

  x_ = other.x_;

  P_ = other.P_;

  // create augmented mean vector
  x_aug = other.x_aug;

  // create augmented state covariance
  P_aug = other.P_aug;

  Xsig_aug_ = other.Xsig_aug_;

  Xsig_pred_ = other.Xsig_pred_;

  weights_ = other.weights_;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = other.std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = other.std_laspy_;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = other.std_radr_;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = other.std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = other.std_radrd_;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */

  /*
  R_radar = MatrixXd::Identity(n_z_radar, n_z_radar);
  R_radar(0,0) = std_radr_ * std_radr_;
  R_radar(1,1) = std_radphi_ * std_radphi_;
  R_radar(2,2) = std_radrd_ * std_radrd_;

  R_lidar = MatrixXd::Identity(n_z_lidar, n_z_lidar);
  R_lidar(0,0) = std_laspx_ * std_laspx_;
  R_lidar(1,1) = std_laspy_ * std_laspy_;
  */

  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  
  lidarCorrection = other.lidarCorrection;

  radarCorrection = other.radarCorrection;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  /*
  
  Aqui va todo 
  Prediction
  Update

  */

  if(!is_initialized_)
  {
    if(meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER)
    {
        x_ << meas_package.raw_measurements_[0],
          meas_package.raw_measurements_[1],
          0,
          0,
          0;

        P_.setIdentity();
        P_(2,2) = 10.0;
        P_(3,3) = 10.0;
        P_(4,4) = 1.0;

        previous_timestamp_ = meas_package.timestamp_;
      //cout << x_ << endl;
        is_initialized_ = true;
    }

  }
  else
  {
      //esta cosa es zero para el segundo sensor!

      //tecnicamente el process noise se borra

		//float dt = (meas_package.timestamp_ - time_us_) / 1000000.0
          
    	dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0f;
    	previous_timestamp_ = meas_package.timestamp_;

      //cout << "dt: " << dt << endl;

      //float dt = (meas_package.timestamp_ - previous_timestamp_) / ;
      //Prediction();
      //cout << Xsig_pred_ << endl;

      //if(fabs(dt) > 0.001f)
      
      AugmentedSigmaPoints();

      //cout << "P_aug : " << endl << P_aug << endl;

      SigmaPointPrediction();

      PredictMeanAndCovariance();

      //cout << "x : " << endl << x_ << endl;

      //cout << "P : " << endl << P_ << endl;

      

      if(meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER)
      {
        //cout << "Lidar predicted" << endl;

        lidarCorrection.z << meas_package.raw_measurements_[0],
                            meas_package.raw_measurements_[1];


        PredictMeasurement(Xsig_pred_, lidarCorrection);


        CorrectState(Xsig_pred_, lidarCorrection);

      }
      else if(meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR)
      {
        //cout << "Radar predicted" << endl;
        //cout << radarCorrection.z.size() << endl;

        radarCorrection.z << meas_package.raw_measurements_[0],
                              meas_package.raw_measurements_[1],
                              meas_package.raw_measurements_[2];

        
        PredictMeasurement(Xsig_pred_, radarCorrection);


        CorrectState(Xsig_pred_, radarCorrection);

      }








      //UpdateLidar(meas_package);
      //UpdateRadar(meas_package);
  }
}


void UKF::SetCorrelationMatrix(const MatrixXd & Xsig_pred, ukf_sensor_correction & sensorCorrection, VectorXd & x)
{

  sensorCorrection.Tc.fill(0.0);

  for(int i = 0; i < sigmaPointCount; ++i)
  {
      VectorXd xDiff = Xsig_pred.col(i) - x;
      VectorXd zDiff = sensorCorrection.Zsig.col(i) - sensorCorrection.z_pred;
      
      //while      
      
      
      while(xDiff[3] > M_PI) 
      {
          xDiff[3] -= 2.0 * M_PI;
      }
      
      while(xDiff[3] < -M_PI)
      {
          xDiff[3] += 2.0 * M_PI;
      }
      
      if(sensorCorrection.Type == ukf_sensor_correction::sensor_type::SENSOR_RADAR)
      {
        while(zDiff[1] > M_PI) 
        {
          zDiff[1] -= 2.0 * M_PI;
        }
      
        while(zDiff[1] < -M_PI)
        {
          zDiff[1] += 2.0 * M_PI;
        }
      }
      
      
    sensorCorrection.Tc = sensorCorrection.Tc + weights_[i] * (xDiff)*(zDiff).transpose();
      
      
  }
}

void UKF::PredictMeasurement(const Eigen::MatrixXd & Xsig_pred, ukf_sensor_correction & sensorCorrection)
{

  sensorCorrection.Zsig.fill(0.0);
  sensorCorrection.z_pred.fill(0.0);
  sensorCorrection.S.fill(0.0);

  for(int i = 0; i < sigmaPointCount; ++i)
  {
    VectorXd xsig = Xsig_pred.col(i);
    
    //cout << "Xsig_pred: " << Xsig_pred << endl;

    VectorXd z(sensorCorrection.n_z);
    z.fill(0.0);
    
    if(sensorCorrection.Type == ukf_sensor_correction::sensor_type::SENSOR_RADAR)
    {
      z[0] = sqrt(xsig[0] * xsig[0] + xsig[1] * xsig[1]);
      z[1] = atan2(xsig[1], xsig[0]);
      z[2] = (xsig[0] * cos(xsig[3])*xsig[2] + xsig[1] * sin(xsig[3])*xsig[2]) / z[0];
    }
    else
    { 
      //NOTE: We only take position from each sigma point and that's it!
      z[0] = xsig[0];
      z[1] = xsig[1];
    }
    
    sensorCorrection.Zsig.col(i) = z;
  }

  //cout << sensorCorrection.Zsig << endl;

  sensorCorrection.z_pred = sensorCorrection.Zsig * weights_;

  for(int i = 0; i < sigmaPointCount; ++i)
  {
      VectorXd zDiff = sensorCorrection.Zsig.col(i) - sensorCorrection.z_pred;
      
      if(sensorCorrection.Type == ukf_sensor_correction::sensor_type::SENSOR_RADAR)
      {
        while(zDiff[1] > M_PI) 
        {
          zDiff[1] -= 2.0 * M_PI;
        }
      
        while(zDiff[1] < -M_PI)
        {
          zDiff[1] += 2.0 * M_PI;
        }
      }
      
      sensorCorrection.S = sensorCorrection.S + weights_[i] * zDiff*zDiff.transpose();
  }
  
  sensorCorrection.S = sensorCorrection.S + sensorCorrection.R;
}

void UKF::CorrectState(const MatrixXd & Xsig_pred, ukf_sensor_correction & sensorCorrection)
{
    // calculate Kalman gain K = 5x3
  // 3x3

    SetCorrelationMatrix(Xsig_pred,sensorCorrection, x_);

    MatrixXd K = sensorCorrection.Tc * sensorCorrection.S.inverse();

    VectorXd residual = sensorCorrection.z - sensorCorrection.z_pred;
  
    if(sensorCorrection.Type == ukf_sensor_correction::sensor_type::SENSOR_RADAR)
    {
      while(residual[1] > M_PI) 
      {
          residual[1] -= 2.0 * M_PI;
      }
      
      while(residual[1] < -M_PI)
      {
          residual[1] += 2.0 * M_PI;
      }
    }  
  

  x_ = x_ + K*residual;
  
  P_ = P_ - K*sensorCorrection.S*K.transpose();

}

void UKF::AugmentedSigmaPoints() {

  // create augmented mean state
  
  x_aug.head(n_x_) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug.bottomRightCorner(2,2) = Q_;

  // create square root matrix

  MatrixXd sqrt_P_aug = P_aug.llt().matrixL();
    
  // create augmented sigma points
  
  Xsig_aug_.fill(0.0);

  Xsig_aug_.col(0) = x_aug;
  
  
  //5 X 5
  
  for(int i = 0; i < n_aug_; ++i)
  {
    Xsig_aug_.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_)*sqrt_P_aug.col(i);
    Xsig_aug_.col(i + n_aug_ + 1) = x_aug - sqrt(lambda_ + n_aug_)*sqrt_P_aug.col(i);
  }
  
}


void UKF::SigmaPointPrediction() {

  // predict sigma points
  Xsig_pred_.fill(0.0);

  for(int i = 0; i < sigmaPointCount; ++i)
  {
    VectorXd xk = Xsig_aug_.col(i);
    
    VectorXd dt_k(n_x_);
    dt_k.fill(0.0);

    VectorXd noise_k(n_x_);
    noise_k.fill(0.0);
    
    noise_k[0] = 0.5 * pow(dt, 2) * cos(xk[3]) * xk[5];
    noise_k[1] = 0.5 * pow(dt, 2) * sin(xk[3]) * xk[5];
    noise_k[2] = dt * xk[5];
    noise_k[3] = 0.5 * pow(dt, 2) * xk[6];
    noise_k[4] = dt * xk[6];
    
    if(fabs(xk[4]) > 0.001)
    {
        dt_k[0] = (xk[2] / xk[4]) * (sin(xk[3] + xk[4] * dt) - sin(xk[3]));
        dt_k[1] = (xk[2] / xk[4]) * (-cos(xk[3] + xk[4] * dt) + cos(xk[3]));
        dt_k[3] = xk[4] * dt;
    }
    else
    {
        //Here we dont multiply by xk[4]
        dt_k[0] = xk[2]*cos(xk[3])*dt;
        dt_k[1] = xk[2]*sin(xk[3])*dt;
    }
    
    VectorXd xk_5 = xk.head(5);
    //std::cout << xk_5 << std::endl;
    
    VectorXd xk_1 = xk_5 + dt_k + noise_k;
    
    Xsig_pred_.col(i) = xk_1;
  }

  // avoid division by zero

  // write predicted sigma points into right column
}


void UKF::PredictMeanAndCovariance() {

  // predict state mean
    //x.fill(0.0);

  //es por los sigma points, 
  // los sigma points tienen el acumulado en si dentro!

  x_.fill(0.0);
  P_.fill(0.0);

  x_ = Xsig_pred_ * weights_;
    
  // predict state covariance matrix
  
  for(int i = 0; i < sigmaPointCount; ++i)
  {
      VectorXd x_diff = Xsig_pred_.col(i) - x_;
      
        //Normalize angle!
        
        while (x_diff(3) > M_PI)
        {
            x_diff(3) -= 2.0 * M_PI;
        }
        
        while (x_diff(3) < -M_PI)
        {
            x_diff(3) += 2.0 * M_PI;
        }
      
     P_ = P_ + weights_[i] * x_diff*x_diff.transpose();
  }
}


void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */



}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  //PredictMeasurement(Xsig_pred, lidarCorrection);
  //lidarCorrection.CorrectState(Xsig_pred);
}



void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  //radarCorrection.PredictMeasurement();
  //radarCorrection.CorrectState(Xsig_pred);
}