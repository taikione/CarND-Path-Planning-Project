#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include "string"
#include <vector>

#include "spline.h"
#include "utility.h"

static const double VEHICLE_WIDTH = 2.0;

using namespace std;

bool isClose(double car_s, double car_d, double acc, int prev_size, int lane,
             const vector<vector<double>> &sensor_fusion) {

  bool too_close = false;

  for (int i = 0; i < sensor_fusion.size(); i++) {
    float d = sensor_fusion[i][6];

    if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_car_s = sensor_fusion[i][5];

      check_car_s += ((double) prev_size * 0.02 * check_speed);

      if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
        too_close = true;
      }
    }
  }

  return too_close;
}

vector<vector<double>> getTrajectory(double car_x, double car_y, double car_yaw, double car_s, double ref_val, int lane, int prev_size,
                                     const vector<double> &previous_path_x, const vector<double> &previous_path_y,
                                     const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x,
                                     const vector<double> &map_waypoints_y) {

  // cout << "lane number is " << lane << endl;
  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  if(prev_size < 2) {
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);

  } else {
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];

    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  // 現在時刻の座標を0として, ptsx, ptsyを書き換え(spline.set_points の為?)
  for(int i=0; i<ptsx.size(); i++){
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
  }

  tk::spline s;
  s.set_points(ptsx, ptsy);

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  for(int i=0; i<previous_path_x.size(); i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));

  // ref_val/2.24 is to convert mi/h to m/s: 1/(1.61*1000/3600) = 2.24
  double N = (target_dist / (0.02*ref_val / 2.24));

  double x_add_on = 0;

  for(int i=1; i <= 50-previous_path_x.size(); i++) {
    double x_point = x_add_on + (target_x)/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // 現在地点を基準に, trajectoryの相対座標を作る
    x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
    y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

    // grobal coordinateへ
    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  return {next_x_vals, next_y_vals};
}


int isCollision(const vector<double> &trajectory_s, const vector<double> &trajectory_d,
                 const vector<double> &target_trajectory_s, const vector<double> &target_trajectory_d) {

  for(int i=0; i<trajectory_s.size(); i++) {
    if (abs(trajectory_d[i] - target_trajectory_d[i]) < VEHICLE_WIDTH) {

      if ((target_trajectory_s[i] >= trajectory_s[i]) && (target_trajectory_s[i-1] <= trajectory_s[i])){
        cout << "is collison " << i << endl;
        return i+1;

      } else if ((target_trajectory_s[i-1] >= trajectory_s[i]) && (target_trajectory_s[i] <= trajectory_s[i])){
        cout << "is collison " << i << endl;
        return i+1;

      }
    }
  }

  return 0;
}


vector<double> getClosestForwardVehicles(double car_s,
                                         int lane,
                                         const vector<vector<double>> &sensor_fusion) {

  vector<double> closest_forward_vehicle;

  double distance_to_forward_vehicle = 30;

  for (int i = 0; i < sensor_fusion.size(); i++) {
    float d = sensor_fusion[i][6];

    // check the vehicle is in the lane or not.
    if (d < (2 + 4*lane + 2) && d > (2 + 4*lane - 2)) {
      double s = sensor_fusion[i][5];

      if (s > car_s && abs(car_s - s) < distance_to_forward_vehicle) {
        // cout << "closest forward !!!" << endl;
        closest_forward_vehicle = sensor_fusion[i];
        distance_to_forward_vehicle = abs(car_s - s);
      }
    }
  }

  return closest_forward_vehicle;
}


vector<double> getClosestBackwardVehicles(double car_s,
                                          int lane,
                                          const vector<vector<double>> &sensor_fusion) {

  vector<double> closest_backward_vehicle;

  double distance_to_backward_vehicle = 30;

  for (int i = 0; i < sensor_fusion.size(); i++) {
    float d = sensor_fusion[i][6];

    // check the vehicle is in the lane or not.
    if (d < (2 + 4*lane + 2) && d > (2 + 4*lane - 2)) {
      double s = sensor_fusion[i][5];

      if (s < car_s && abs(car_s - s) < distance_to_backward_vehicle) {
        // cout << "closest backward !!!" << endl;
        closest_backward_vehicle = sensor_fusion[i];
        distance_to_backward_vehicle = abs(car_s - s);
      }
    }
  }

  return closest_backward_vehicle;
}


vector<vector<double>> getOtherVehicleTrajectory(vector<double> &vehicle, int trajectory_size) {

  vector<double> cv_trajectory_x;
  vector<double> cv_trajectory_y;

  double closest_vehicle_x = vehicle[1];
  double closest_vehicle_y = vehicle[2];

  // trajectory内のway pointは0.2秒ごと, vx, vyは1秒毎なのでvx, vyを5で割る
  double vx = vehicle[3] / 5;
  double vy = vehicle[4] / 5;

  for(int i=0; i<trajectory_size; i++) {
    closest_vehicle_x += vx;
    closest_vehicle_y += vy;

    cv_trajectory_x.push_back(closest_vehicle_x);
    cv_trajectory_y.push_back(closest_vehicle_y);
  }

  return {cv_trajectory_x, cv_trajectory_y};

}


double getCollisionCost(int lane, double car_s, double car_yaw,
                        const vector<vector<double>> &trajectory,
                        const vector<vector<double>> &sensor_fusion,
                        const vector<double> &maps_x,
                        const vector<double> &maps_y){

  double cost = 0.0;
  double diff_x;
  double diff_y;
  double vehicle_yaw;
  int forward_collision_at = 0;
  int backward_collision_at = 0;

  // create trajectory with sd coordinate
  vector<vector<double>> sd_trajectory = getSDTrajectory(trajectory[0], trajectory[1], car_yaw, maps_x, maps_y);
  vector<double> trajectory_s = sd_trajectory[0];
  vector<double> trajectory_d = sd_trajectory[1];

  // get closest vehicle in changed lane
  vector<double> forward_vehicle = getClosestForwardVehicles(car_s, lane, sensor_fusion);

  if (forward_vehicle.size() == 7) {
    // create closest forward vehicle trajectory
    // cout << "closed vehicle is here" << endl;
    vector<vector<double>> forward_vehicle_trajectory = getOtherVehicleTrajectory(forward_vehicle, trajectory[0].size());

    diff_x = forward_vehicle_trajectory[0].back() - forward_vehicle_trajectory[0].front();
    diff_y = forward_vehicle_trajectory[1].back() - forward_vehicle_trajectory[1].front();
    vehicle_yaw = atan2(diff_y, diff_x);

    vector<vector<double>> forward_vehicle_sd_trajectory = getSDTrajectory(forward_vehicle_trajectory[0], forward_vehicle_trajectory[1], vehicle_yaw, maps_x, maps_y);

    forward_collision_at = isCollision(trajectory_s, trajectory_d,
                                       forward_vehicle_sd_trajectory[0], forward_vehicle_sd_trajectory[1]);

    if (forward_collision_at != 0) {
      // cost += exp(-pow(forward_collision_at, 2));
      cost += exp(3 / forward_collision_at);
    }
  }

  // create closest backward vehicle trajectory
  vector<double> backward_vehicle = getClosestBackwardVehicles(car_s, lane, sensor_fusion);

  if (backward_vehicle.size() == 7) {
    // cout << "closed vehicle is here" << endl;

    vector<vector<double>> backward_vehicle_trajectory = getOtherVehicleTrajectory(backward_vehicle, trajectory[0].size());

    diff_x = backward_vehicle_trajectory[0].back() - backward_vehicle_trajectory[0].front();
    diff_y = backward_vehicle_trajectory[1].back() - backward_vehicle_trajectory[1].front();
    vehicle_yaw = atan2(diff_y, diff_x);

    vector<vector<double>> backward_vehicle_sd_trajectory = getSDTrajectory(backward_vehicle_trajectory[0], backward_vehicle_trajectory[1], vehicle_yaw, maps_x, maps_y);

    backward_collision_at = isCollision(trajectory_s, trajectory_d,
                                        backward_vehicle_sd_trajectory[0], backward_vehicle_sd_trajectory[1]);

    if (backward_collision_at != 0) {
      // cost += exp(-pow(backward_collision_at, 2));
      cost += exp(3 / backward_collision_at);
    }
  }

  return cost * 1000;
}

#endif //PATH_PLANNING_VEHICLE_H
