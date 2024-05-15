// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
#include <vector>
#include <math.h>

using namespace std;

double t;
vector<double> t_list;

void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
  // INSERT YOUR CODE HERE
    //Declare variables
    double Kh, Kp, Pout, e, altitude, speed;
    static double Delta;
    vector<double> e_list, speed_list;
    altitude = (position.abs()-MARS_RADIUS);
    speed = velocity*(position.norm());
    
    // Initialization!!!!!!!!!!!!!!!
    Kh = 1.8e-2;
    Kp = 1.0;
    Delta = ((UNLOADED_LANDER_MASS + fuel*FUEL_CAPACITY*FUEL_DENSITY)*GRAVITY*MARS_MASS)/ (MARS_RADIUS*MARS_RADIUS*MAX_THRUST);
    
    //Autopilot setting
    e = - (0.5 + Kh* altitude + speed);
    Pout = Kp * e;
    
//    e_list.push_back(position.abs());
//    speed_list.push_back(Delta);
    
    if(Pout < -Delta or Pout == -Delta){
        throttle = 0;
    }else if (Pout > -Delta and Pout < 1-Delta){
        throttle = Delta + Pout;
    }else{
        throttle = 1;
    }

//    //Write the trajectories to file
//    ofstream fout;
//    fout.open("trajectories.txt");
//    if (fout) {                                         //file opened successfully
//        double tmp = t_list.size();
//        for (int i=0; i<tmp; i=i+1) {
////            cout << e_list[i] <<" "<< speed_list[i] << endl;
//        }
//    }else {                                             //file did not open successfully
//        cout << "Could not open trajectory file for writing" << endl;
//    }
}

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).
{
  // INSERT YOUR CODE HERE
    //Declare variables
    double m;
    vector<double> a_list, h_list, v_list;
    vector3d a;
    
    //Qualify constants
    m = UNLOADED_LANDER_MASS + fuel*FUEL_CAPACITY*FUEL_DENSITY;
    
    //Numerial integration
        
        //Declare the variables required for acceleration
        double density, Cd_lander, Cd_para, A_lander, A_para;
        vector3d a_g, a_thrust, a_drag_lander, a_drag_para;
    
        // Useful parameters for acceleration
        density = atmospheric_density(position);
        Cd_lander = DRAG_COEF_LANDER;
        Cd_para = DRAG_COEF_CHUTE;
        A_lander = M_PI * (LANDER_SIZE)*(LANDER_SIZE);
        A_para = (4.0 *LANDER_SIZE*LANDER_SIZE)*5;
        
        //Calculate the related components of acceleration
        a_g = -((GRAVITY*MARS_MASS)/position.abs2())*position.norm();
        a_thrust = (thrust_wrt_world())/m;
        a_drag_lander = -(density * Cd_lander * A_lander * velocity.abs2() * velocity.norm())/(2 * m);
        a_drag_para = -(density * Cd_para * A_para * velocity.abs2() * velocity.norm())/(2 * m);
        
        //Discussing the case with respect to the state of the parachute
        if (parachute_status == DEPLOYED){
            a = a_g + a_thrust + a_drag_lander + a_drag_para;
        }else{
            a = a_g + a_thrust + a_drag_lander;
        }
        
        // Euler + Verlet Integration
        static vector3d previous_position;
        vector3d new_position;
        
        if (simulation_time == 0.0){                    /*Define the intial condition*/
            new_position = position + delta_t * velocity;
            velocity = velocity + delta_t * a;
        } else {                                       /*Verlet integation to update data*/
            new_position = 2*position - previous_position + delta_t*delta_t*a;
            velocity = (new_position - position)/delta_t;
        }
    
        //Recording instantanous data for output purpose
        t_list.push_back(simulation_time);
        a_list.push_back(a.abs());
        h_list.push_back(position.y);
        v_list.push_back(velocity.abs());
        
        //Updating the current state
        previous_position = position;
        position = new_position;

    
  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
    break;

  case 7:
    break;

  case 8:
    break;

  case 9:
    break;

  }
}
