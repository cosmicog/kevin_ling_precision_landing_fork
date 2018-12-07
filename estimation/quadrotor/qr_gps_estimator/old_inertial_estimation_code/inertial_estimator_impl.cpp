
/**
 * Source of the InertialKF class.
 *
 * Author: A.Das
 * Dec 2013
 *
 * WAVELab, University of Waterloo
 */

#include "inertial_estimator_impl.h"
#include <math.h>
#include <ros/ros.h>
#include <iostream>

using namespace Eigen;
using namespace std;

#define NSTATES 7

//states
/*
 * ax (acceleration in body x)
 * ay (acceleration in body y)
 * vx (vel in body x)
 * vy (vel in body y)
 * psi (heading , front of qr facing inertial X(East) is zero)
 * X (inertial X position in ENU frame
 * Y (inertial Y position in ENU frame
 * 
*/

InertialKF::InertialKF() : KalmanFilterBase(NSTATES) //7 state filter
{
  /*sonar_matr_C = MatrixXd(1, NSTATES);
  sonar_matr_C << 1, 0, 0;
  sonar_meas_cov = MatrixXd(1,1);
  sonar_meas_cov <<  0.0025;

  // [barom, climb rate, z_accel]
  barom_matr_C = MatrixXd(2,3);
  barom_matr_C << 1, 0, 1,
                  0, 1, 0;
  barom_meas_cov = MatrixXd(2,2);
  barom_meas_cov <<  0.5, 0,
                     0, 9.5;

  // Real full measurement update
  // [sonar, barom, climb rate, z_accel]
  full_matr_C = MatrixXd(3,3);
  full_matr_C << 1, 0, 0,
                 1, 0, 1,
                 0, 1, 0;
                   
  full_meas_cov = MatrixXd(3,3);
  full_meas_cov <<  0.0025, 0, 0,
                    0, 0.5, 0,
                    0, 0, 9.5;*/

  pred_cov = MatrixXd(NSTATES,NSTATES);
  pred_cov << 0.2, 0, 0, 0, 0, 0, 0,
              0.0, 0.2, 0, 0, 0, 0, 0,
              0.0, 0, 0.2, 0, 0, 0, 0,
              0.0, 0, 0, 0.2, 0, 0, 0,
              0.0, 0, 0, 0, 0.2, 0, 0,
              0.0, 0, 0, 0, 0, 0.2, 0,
              0.0, 0, 0, 0, 0, 0, 0.2;
}


void InertialKF::gpsMeasurementUpdate(double x_pos,double y_pos,double x_vel,double y_vel)
{
 
}


void InertialKF::magnetometerMeasurementUpdate(double heading)
{
 
}


void InertialKF::fullMeasurementUpdate(double x_pos,double y_pos,double x_vel,double y_vel, double heading)
{

}


void InertialKF::predictionUpdate(double f_thrust, double roll, double pitch, double yaw_rate, double qr_mass, double qr_drag, double dt)
{
	//precompute angles
	
	double c_roll = cos(roll); double s_roll = sin(roll);
	double c_pitch = cos(pitch); double s_pitch = sin(pitch);
		
	//get the best state so far
	
	VectorXd mean = getMeanStates();
	//fill in the states
	
	double ax = mean(0);
	double ay = mean(1);
	double vx = mean(2);
	double vy = mean(3);
	double psi = mean(4);
	double X = mean(5);
	double Y = mean(6);
	
	//propogate motion
  
	double ax_p =  1*((c_roll*s_pitch)*f_thrust - qr_drag*vx )/qr_mass;
	double ay_p = 1*((-s_roll)*f_thrust - qr_drag*vy)/qr_mass;
	double vx_p = ax_p*dt;
	double vy_p = ay_p*dt;
	double psi_p = psi + yaw_rate*dt;
	double X_p = X + (cos(psi)*vx_p*dt - sin(psi)*vy_p*dt);
	double Y_p = Y + (sin(psi)*vx_p*dt + cos(psi)*vy_p*dt);
	
	//save the new state
	
	VectorXd pred_state(7);
	pred_state<< ax_p,ay_p,vx_p,vy_p,psi_p,X_p,Y_p;
	
	
	//compute the jacobian
	
	 MatrixXd J(7,7);
	 

	J<< 0, 0, -1.0*(qr_drag*1)/qr_mass, 0, 0, 0, 0,
		0, 0, 0,-1.0*(qr_drag*1)/qr_mass, 0, 0, 0,
		dt, 0, 0, 0, 0, 0, 0,
		0, dt, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 1, 0, 0,
		0, 0, dt*cos(psi), -1.0*dt*sin(psi), -1.0*dt*vy*cos(psi)-dt*vx*sin(psi), 1, 0,
		0, 0, dt*sin(psi), dt*cos(psi), dt*vx*cos(psi)-dt*vy*sin(psi), 0, 1;
		
	//perform prediction update
	
	performPredictionUpdate(J, pred_state, pred_cov);
  
}


void InertialKF::getStateEstimate(double &X, double &Y)
{
  VectorXd mean = getMeanStates();
  X = mean(5);
  Y = mean(6);
}


