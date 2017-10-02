#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include "string"
#include <vector>
#include "math.h"

#include "spline.h"
#include "utility.h"

static const double VEHICLE_WIDTH = 6.0;
static const double COLLISION_PENALTY = 6.5;

using namespace std;


double getClosestVehicleDistance(double car_s, int lane,
                                 const vector<vector<double>> &sensor_fusion) {

  double distance = 80.0 ;

  for (int i = 0; i < sensor_fusion.size(); i++) {
    double d = sensor_fusion[i][6];

    if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)) {
      double check_car_s = sensor_fusion[i][5];

      if ((check_car_s > car_s) && ((check_car_s - car_s) <= distance)) {
        distance = check_car_s - car_s;
      }
    }
  }

  return distance;
}


vector<vector<double>> getTrajectory(double car_x, double car_y, double car_yaw, double car_s, double ref_val, int lane, int prev_size,
                                     const vector<double> &previous_path_x, const vector<double> &previous_path_y,
                                     const vector<double> &map_waypoints_s, const vector<double> &map_waypoints_x,
                                     const vector<double> &map_waypoints_y) {

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


double isCollision(int lane, const vector<double> &trajectory_s, const vector<double> &trajectory_d,
                   const vector<double> &target_trajectory_s, const vector<double> &target_trajectory_d) {

  int collision_at = 0;
  cout << "iC ";

  for(int i=trajectory_s.size(); i>0; i--) {
    if (abs(target_trajectory_s[i] - trajectory_s[i]) <= 20){
      collision_at = i+1;
    }
  }

  cout << collision_at << ", ";
  return collision_at;
}


vector<double> getClosestForwardVehicles(double car_s,
                                         int lane,
                                         const vector<vector<double>> &sensor_fusion) {

  vector<double> closest_forward_vehicle;
  double distance_to_forward_vehicle = 50;

  for (int i = 0; i < sensor_fusion.size(); i++) {
    double d = sensor_fusion[i][6];

    // check the vehicle is in the lane or not.
    if (d < (2 + 4*lane + 2) && d > (2 + 4*lane - 2)) {
      double s = sensor_fusion[i][5];

      if ((s >= car_s) && ((s - car_s) <= distance_to_forward_vehicle)) {
        closest_forward_vehicle = sensor_fusion[i];
        distance_to_forward_vehicle = s - car_s;
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
    double d = sensor_fusion[i][6];

    // check the vehicle is in the lane or not.
    if (d < (2 + 4*lane + 2) && d > (2 + 4*lane - 2)) {
      double s = sensor_fusion[i][5];

      if ((s <= car_s) && ((car_s - s) <= distance_to_backward_vehicle)) {
        // cout << "closest backward !!!" << endl;
        closest_backward_vehicle = sensor_fusion[i];
        distance_to_backward_vehicle = car_s - s;
      }
    }
  }

  return closest_backward_vehicle;
}


vector<vector<double>> getOtherVehicleTrajectory(vector<double> &vehicle, int trajectory_size) {

  vector<double> cv_trajectory_s;
  vector<double> cv_trajectory_d;

  double closest_vehicle_s = vehicle[5];
  double closest_vehicle_d = vehicle[6];

  cv_trajectory_s.push_back(closest_vehicle_s);
  cv_trajectory_d.push_back(closest_vehicle_d);

  double vx = vehicle[3];
  double vy = vehicle[4];
  double speed = sqrt(vx * vx + vy * vy);
  double yaw = atan2(vy, vx);

  // trajectory内のway pointは0.02秒ごと, vx, vyは1秒毎なのでvx, vyを5で割る
  for(int i=0; i<trajectory_size; i++) {
    closest_vehicle_s += speed * 0.02 * cos(yaw);
    closest_vehicle_d -= speed * 0.02 * sin(yaw);

    cv_trajectory_s.push_back(closest_vehicle_s);
    cv_trajectory_d.push_back(closest_vehicle_d);
  }

  return {cv_trajectory_s, cv_trajectory_d};

}


double getForwardCollisionCost(int lane, double car_s,
                        const vector<vector<double>> &sd_trajectory,
                        const vector<vector<double>> &sensor_fusion){

  double cost = 0.0;
  double forward_collision_at = 0.0;
  double center_forward_collision_at = 0.0;

  // create trajectory with sd coordinate
  vector<double> trajectory_s = sd_trajectory[0];
  vector<double> trajectory_d = sd_trajectory[1];

  // get closest vehicle in changed lane
  vector<double> forward_vehicle = getClosestForwardVehicles(trajectory_s[0], lane, sensor_fusion);

  if (forward_vehicle.size() == 7) {
    // create closest forward vehicle trajectory
    cout << "F_id: " << forward_vehicle[0] << "; ";

    vector<vector<double>> forward_vehicle_trajectory = getOtherVehicleTrajectory(forward_vehicle, trajectory_s.size());
    cout << "F_Dist: " << forward_vehicle_trajectory[0][0] - trajectory_s[0] << "; ";

    forward_collision_at = isCollision(lane, trajectory_s, trajectory_d,
                                       forward_vehicle_trajectory[0], forward_vehicle_trajectory[1]);

    if (forward_collision_at != 0.0) {
      // cost += exp(3.0 / forward_collision_at);
      cost += COLLISION_PENALTY / forward_collision_at;
    }
  }

  return cost * 1000;
}


double getBackwardCollisionCost(int lane, double car_s,
                        const vector<vector<double>> &sd_trajectory,
                        const vector<vector<double>> &sensor_fusion){

  double cost = 0.0;
  double backward_collision_at = 0.0;
  double center_backward_collision_at = 0.0;

  // create trajectory with sd coordinate
  vector<double> trajectory_s = sd_trajectory[0];
  vector<double> trajectory_d = sd_trajectory[1];

  // create closest backward vehicle trajectory
  vector<double> backward_vehicle = getClosestBackwardVehicles(trajectory_s[0], lane, sensor_fusion);

  if (backward_vehicle.size() == 7) {
    cout << "B_id: " << backward_vehicle[0] << "; ";

    vector<vector<double>> backward_vehicle_trajectory = getOtherVehicleTrajectory(backward_vehicle, trajectory_s.size());
    cout << "B_Dist: " << trajectory_s[0] - backward_vehicle_trajectory[0][0] << "; ";

    backward_collision_at = isCollision(lane, trajectory_s, trajectory_d,
                                        backward_vehicle_trajectory[0], backward_vehicle_trajectory[1]);

    if (backward_collision_at != 0.0) {
      cost += COLLISION_PENALTY / backward_collision_at;
    }
  }

  return cost * 1000;
}

#endif //PATH_PLANNING_VEHICLE_H
