#ifndef COSTS
#define COSTS

#include <iostream>
#include <cassert>
#include <vector>
#include <algorithm>
#include <cmath>
#include "constants.h"

using namespace std;

// UTILITY FUNCTIONS

double logistic(double x){
  return 2.0 / (1 + exp(-x)) - 1.0;
}

double nearest_approach(vector<double> s_traj, vector<double> d_traj, vector<vector<double>> prediction) {
  double closest = MAX_DOUBLE;
  for (int i = 0; i < N_SAMPLES; i++) {
    double current_dist = sqrt(pow(s_traj[i] - prediction[i][0], 2) + pow(d_traj[i] - prediction[i][1], 2));
    if (current_dist < closest) {
      closest = current_dist;
    }
  }
  return closest;
}

double nearest_approach_to_any_vehicle(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  // Determines the nearest the vehicle comes to any other vehicle throughout a trajectory
  double closest = MAX_DOUBLE;
  for (auto prediction : predictions) {
    double current_dist = nearest_approach(s_traj, d_traj, prediction.second);
    if (current_dist < closest) {
      closest = current_dist;
    }
  }
  return closest;
}

double nearest_approach_to_any_vehicle_in_lane(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  // Determines the nearest the vehicle comes to any other vehicle throughout a trajectory
  double closest = MAX_DOUBLE;
  for (auto prediction : predictions) {
    double my_final_d = d_traj[d_traj.size() - 1];
    int my_lane = my_final_d / 4;
    vector<vector<double>> pred_traj = prediction.second;
    double pred_final_d = pred_traj[pred_traj.size() - 1][1];
    int pred_lane = pred_final_d / 4;
    if (my_lane == pred_lane) {
      double current_dist = nearest_approach(s_traj, d_traj, prediction.second);
      if (current_dist < closest && current_dist < 120) {
        closest = current_dist;
      }
    }
  }
  return closest;
}

vector<double> velocities_for_trajectory(vector<double> traj) {
  // average velocity along a trajectory.
  vector<double> velocities;
  for (int i = 1; i < traj.size(); i++) {
    velocities.push_back((traj[i] - traj[i-1]) / DT);
  }
  return velocities;
}


// diverse cost

//cost on time overhead
double time_diff_cost(double target_time, double actual_time) {
  return logistic(fabs(actual_time - target_time) / target_time);
}

//cost on trajectory shift
double traj_diff_cost(vector<double> s_traj, vector<double> target_s) {
  int s_end = s_traj.size();
  double s1, s2, s3, s_dot1, s_dot2, s_ddot, cost = 0;
  s1 = s_traj[s_end - 1];
  s2 = s_traj[s_end - 2];
  s3 = s_traj[s_end - 3];
  s_dot1 = (s1 - s2) / DT;
  s_dot2 = (s2 - s3) / DT;
  s_ddot = (s_dot1 - s_dot2) / DT;
  cost += fabs(s1 - target_s[0]) / SIGMA_S;
  cost += fabs(s_dot1 - target_s[1]) / SIGMA_S_DOT;
  cost += fabs(s_ddot - target_s[2]) / SIGMA_S_DDOT;
  return logistic(cost);
}

//cost on car collision;binary due to importance;
double collision_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  double nearest = nearest_approach_to_any_vehicle(s_traj, d_traj, predictions);
  if (nearest < 2 * VEHICLE_RADIUS) {
    return 1;
  } else { 
    return 0;
  }
}


//cost on distance to other cars.
double buffer_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  double nearest = nearest_approach_to_any_vehicle(s_traj, d_traj, predictions);
  return logistic(2 * VEHICLE_RADIUS / nearest);
}

//cost on distance to others cars in same lane.
double in_lane_buffer_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
  double nearest = nearest_approach_to_any_vehicle_in_lane(s_traj, d_traj, predictions);
  return logistic(2 * VEHICLE_RADIUS / nearest);
}

//cost on overspeed
double exceeds_speed_limit_cost(vector<double> s_traj) {
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  for (double s_dot : s_dot_traj) {
    if (s_dot > SPEED_LIMIT) {
      return 1;
    }
  }
  return 0;
}

//cost on slowness
double efficiency_cost(vector<double> s_traj) {
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  double final_s_dot, total = 0;

  final_s_dot = s_dot_traj[s_dot_traj.size() - 1];
  return logistic((SPEED_LIMIT - final_s_dot) / SPEED_LIMIT);
} 

//cost on over acceleration.
double max_accel_cost(vector<double> s_traj) {
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  vector<double> s_ddot_traj = velocities_for_trajectory(s_dot_traj);
  for (double s_ddot : s_ddot_traj) {
    if (s_ddot > MAX_INSTANTANEOUS_ACCEL) {
      return 1;
    }
  }
  return 0;
}

//cost on high acceleration
double avg_accel_cost(vector<double> s_traj) {
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  vector<double> s_ddot_traj = velocities_for_trajectory(s_dot_traj);
  double total = 0;
  for (double s_ddot: s_ddot_traj) {
    total += s_ddot;
  }
  double avg_accel = total / s_ddot_traj.size();
  return logistic(avg_accel / EXPECTED_ACC_IN_ONE_SEC );
}

//cost on over jerk
double max_jerk_cost(vector<double> s_traj) {
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  vector<double> s_ddot_traj = velocities_for_trajectory(s_dot_traj);
  vector<double> s_dddot_traj = velocities_for_trajectory(s_ddot_traj);
  for (double s_dddot : s_dddot_traj) {
    if (s_dddot > MAX_INSTANTANEOUS_JERK) {
      return 1;
    }
  }
  return 0;
}


//cost on high jerk
double avg_jerk_cost(vector<double> s_traj) {
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  vector<double> s_ddot_traj = velocities_for_trajectory(s_dot_traj);
  vector<double> s_dddot_traj = velocities_for_trajectory(s_ddot_traj);
  double total = 0;
  for (double s_dddot: s_dddot_traj) {
    total += s_dddot;
  }
  double avg_jerk = total / s_dddot_traj.size();
  return logistic(avg_jerk / EXPECTED_JERK_IN_ONE_SEC );
}

//cost on shift from middle lane.
double not_middle_lane_cost(vector<double> d_traj) {
  double end_d = d_traj[d_traj.size()-1];
  return logistic(pow(end_d-6, 2));
}

//total cost
double calculate_total_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {

  double total_cost = 0;
  double col = collision_cost(s_traj, d_traj, predictions) * COLLISION_COST_WEIGHT;
  double buf = buffer_cost(s_traj, d_traj, predictions) * BUFFER_COST_WEIGHT;
  double ilb = in_lane_buffer_cost(s_traj, d_traj, predictions) * IN_LANE_BUFFER_COST_WEIGHT;
  double eff = efficiency_cost(s_traj) * EFFICIENCY_COST_WEIGHT;
  double nml = not_middle_lane_cost(d_traj) * NOT_MIDDLE_LANE_COST_WEIGHT;
  double aad = avg_accel_cost(d_traj) * AVG_ACCEL_COST_WEIGHT;
  double ajs = avg_jerk_cost(s_traj) * AVG_JERK_COST_WEIGHT;

  total_cost += col + buf + ilb + eff + nml + aad + ajs;
  return total_cost;
}

#endif
