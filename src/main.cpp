#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

//costom package
#include "smoother.h"
#include "constants.h"
#include "vehicle.h"
#include "costs.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

        double closestLen = MAX_DOUBLE; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
          angle = min(2*pi() - angle, angle);

          if(angle > pi()/4)
          {
            closestWaypoint++;
          if (closestWaypoint == maps_x.size())
          {
            closestWaypoint = 0;
          }
          }

          return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y, vector<double> maps_s)
{
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size()-1;
    }
    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];
    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;
    double frenet_d = distance(x_x,x_y,proj_x,proj_y);
    //see if d value is positive or negative by comparing it to a center point
    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);
    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }
    // calculate s value
    double frenet_s = maps_s[0];
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }
    frenet_s += distance(0,0,proj_x,proj_y);
    return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

// Get nearby waypoints, given the numbers of the preceeding waypoints and the following waypoints.
vector<vector<double>> getNearbyWaypoints(int size_waypoints,int num_waypoints_behind, int num_waypoints_ahead, int id_next_waypoint,
                                          vector<double> map_x,vector<double> map_y,vector<double> map_s,
                                          vector<double> map_dx,vector<double> map_dy){
    vector<vector<double>> nearby_waypoints;
    vector<double> nearby_x,nearby_y,nearby_s,nearby_dx,nearby_dy;
    for (int i = -num_waypoints_behind; i < num_waypoints_ahead; i++) {
        //take some neawrby waypoints
        int idx = (id_next_waypoint+i) % size_waypoints;
        if (idx < 0) {//in case of negative value
            idx += size_waypoints;
        }
        //cout<<"idx:"<<idx<<endl;
        double current_s = map_s[idx];
        double base_s = map_s[id_next_waypoint];
        if (i < 0 && current_s > base_s) {
            current_s -= TRACK_LENGTH;
        }
        if (i > 0 && current_s < base_s) {
            current_s += TRACK_LENGTH;
        }
        nearby_x.push_back(map_x[idx]);
        nearby_y.push_back(map_y[idx]);
        nearby_s.push_back(current_s);
        nearby_dx.push_back(map_dx[idx]);
        nearby_dy.push_back(map_dy[idx]);
    }
    nearby_waypoints.push_back(nearby_x);
    nearby_waypoints.push_back(nearby_y);
    nearby_waypoints.push_back(nearby_s);
    nearby_waypoints.push_back(nearby_dx);
    nearby_waypoints.push_back(nearby_dy);
    //cout<<"nearby size:"<<nearby_waypoints.size()<<endl;
    return nearby_waypoints;
}

void getEgoCarState(Vehicle &car, vector<vector<double>> nearby_inter_waypoints,int prev_path_size,
                    vector<double> previous_path_x,vector<double> previous_path_y){
    double pos_s, vel_s, acc_s, pos_d, vel_d, acc_d;
    // Other values necessary for determining these based on future points in previous path
    double pos_x, pos_y, pos_angle1, vel_x1, vel_y1;
    double pos_x2, pos_y2, vel_x2, vel_y2;
    double pos_x3, pos_y3;
    double acc_x, acc_y;

    vector<double> nearby_inter_x=nearby_inter_waypoints[0];
    vector<double> nearby_inter_y=nearby_inter_waypoints[1];
    vector<double> nearby_inter_s=nearby_inter_waypoints[2];
    vector<double> nearby_inter_dx=nearby_inter_waypoints[3];
    vector<double> nearby_inter_dy=nearby_inter_waypoints[4];

    int subpath_size = min(PREVIOUS_PATH_POINTS_TO_KEEP,prev_path_size);
    double start_time = subpath_size * PATH_DT;

    //continue previous path
    pos_x = previous_path_x[subpath_size-1];
    pos_y = previous_path_y[subpath_size-1];
    pos_x2 = previous_path_x[subpath_size-2];
    pos_y2 = previous_path_y[subpath_size-2];
    pos_angle1 = atan2(pos_y-pos_y2,pos_x-pos_x2);
    vector<double> frenet = getFrenet(pos_x, pos_y, pos_angle1, nearby_inter_x, nearby_inter_y, nearby_inter_s);
    pos_s = frenet[0]; pos_d = frenet[1];

    // determine dx, dy vector from set of interpoated waypoints, with pos_x,pos_y as reference point;
    // since interpolated waypoints are ~1m apart and path points tend to be <0.5m apart, these
    // values can be reused for previous two points (and using the previous waypoint data may be
    // more accurate) to calculate vel_s (s_dot), vel_d (d_dot), acc_s (s_ddot), and acc_d (d_ddot)

    // find dx,dy of the next waypoints in the set of interpolated waypoints.
    int id_next_waypoint = NextWaypoint(pos_x, pos_y, pos_angle1, nearby_inter_x,nearby_inter_y);
    double dx = nearby_inter_dx[id_next_waypoint - 1];
    double dy = nearby_inter_dy[id_next_waypoint - 1];
    // sx as big as dy, sy as big as dx.
    double sx = -dy; double sy = dx;

    // calculate s_dot & d_dot
    vel_x1 = (pos_x - pos_x2) / PATH_DT;
    vel_y1 = (pos_y - pos_y2) / PATH_DT;
    // want projection of xy velocity vector (V) onto S (sx,sy) and D (dx,dy) vectors, and since S
    // and D are unit vectors this is simply the dot products of V with S and V with D
    vel_s = vel_x1 * sx + vel_y1 * sy;
    vel_d = vel_x1 * dx + vel_y1 * dy;

    // acceleration in s and d direction.
    pos_x3 = previous_path_x[subpath_size-3];
    pos_y3 = previous_path_y[subpath_size-3];
    vel_x2 = (pos_x2 - pos_x3) / PATH_DT;
    vel_y2 = (pos_y2 - pos_y3) / PATH_DT;
    acc_x = (vel_x1 - vel_x2) / PATH_DT;
    acc_y = (vel_y1 - vel_y2) / PATH_DT;
    acc_s = acc_x * sx + acc_y * sy;
    acc_d = acc_x * dx + acc_y * dy;

    car.s    = pos_s;
    car.s_d  = vel_s;
    car.s_dd = acc_s;
    car.d    = pos_d;
    car.d_d  = vel_d;
    car.d_dd = acc_d;
    car.pos_x= pos_x;
    car.pos_y= pos_y;
    car.angle= pos_angle1;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
    
  Vehicle ego_car = Vehicle();

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);
    
  ofstream log_file;
  log_file.open("path_planning_log.csv");

  string line;


  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &ego_car, &log_file](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object

        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;



            // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            ofstream single_iteration_log;
            single_iteration_log.open("path_planning_log-single_iteration.csv");
            

            //PART1: get nearby waypoints
            int next_waypoint_index = NextWaypoint(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);
            vector<vector<double>> nearby_waypoints;
            vector<double> nearby_x,nearby_y,nearby_s,nearby_dx,nearby_dy;
            nearby_waypoints=getNearbyWaypoints(map_waypoints_x.size(),NUM_WAYPOINTS_BEHIND, NUM_WAYPOINTS_AHEAD, next_waypoint_index,
                                                      map_waypoints_x, map_waypoints_y,map_waypoints_s,
                                                      map_waypoints_dx,map_waypoints_dy);

            nearby_x=nearby_waypoints[0];nearby_y=nearby_waypoints[1];nearby_s=nearby_waypoints[2];
            nearby_dx=nearby_waypoints[3];nearby_dy=nearby_waypoints[4];
            if(debug==true){
                cout<<"nearby waypoints size:"<<nearby_waypoints.size()<<endl;
                cout<<"nearby s size:"<<nearby_x.size()<<endl;
                cout<<"coarse s:"<<nearby_s[0]<<" "<<nearby_s[8]<<endl;
            }
            
            // PART2: interpolation parameters
            double dist_interpolation = 0.2;
            int num_interpolation = (nearby_s[nearby_s.size()-1] - nearby_s[0]) / dist_interpolation;
            vector<double> nearby_inter_x, nearby_inter_y, nearby_inter_s,nearby_inter_dx, nearby_inter_dy;
            nearby_inter_s.push_back(nearby_s[0]);
            for (int i = 1; i < num_interpolation; i++) {
                nearby_inter_s.push_back(nearby_s[0] + i * dist_interpolation);
            }
            nearby_inter_x = interpolate_points(nearby_s, nearby_x, dist_interpolation, num_interpolation);
            nearby_inter_y = interpolate_points(nearby_s, nearby_y, dist_interpolation, num_interpolation);
            nearby_inter_dx = interpolate_points(nearby_s, nearby_dx, dist_interpolation, num_interpolation);
            nearby_inter_dy = interpolate_points(nearby_s, nearby_dy, dist_interpolation, num_interpolation);
            vector<vector<double>> nearby_inter_waypoints;
            nearby_inter_waypoints.push_back(nearby_inter_x);
            nearby_inter_waypoints.push_back(nearby_inter_y);
            nearby_inter_waypoints.push_back(nearby_inter_s);
            nearby_inter_waypoints.push_back(nearby_inter_dx);
            nearby_inter_waypoints.push_back(nearby_inter_dy);
            if(debug==true){
                cout<<"nearby interpolation waypoints size:"<<nearby_inter_waypoints.size()<<endl;
                cout<<"nearby inter s size:"<<nearby_inter_s.size()<<endl;
                cout<<"nearby inter s:"<<nearby_inter_s[0]<<" "<<nearby_inter_s[8]<<endl;
            }


            //PART3: get ego car state
            // if not enough previous path points, use the current state
            int subpath_size = min(PREVIOUS_PATH_POINTS_TO_KEEP, (int)previous_path_x.size());
            if(debug) cout<<"subpath_size:"<<subpath_size<<endl;
            if (subpath_size < MIN_LEN_PREVIOUS_PATH) {

                ego_car.s    = car_s;           // s position
                ego_car.s_d  = car_speed;           // velocity in s
                ego_car.s_dd = 0;          // acceleration in s
                ego_car.d    = car_d;           // d position
                ego_car.d_d  = 0;           // velocity in d
                ego_car.d_dd = 0;          // acceleration in d
                ego_car.pos_x=car_x;        // x position
                ego_car.pos_y=car_y;        // y position
                ego_car.angle=car_yaw;      // yaw
            } else {
                //continue previous path
                getEgoCarState(ego_car, nearby_inter_waypoints,(int)previous_path_x.size(),previous_path_x,previous_path_y);
            }
            if(debug){
                cout<<"ego_car s:"<<ego_car.s<<" "<<ego_car.s_d<<" "<<ego_car.s_dd<<endl;
                cout<<"ego_car d:"<<ego_car.d<<" "<<ego_car.d_d<<" "<<ego_car.d_dd<<endl;
                cout<<"ego_car xy:"<<ego_car.pos_x<<" "<<ego_car.pos_y<<" "<<ego_car.angle<<endl;
            }
            
            //PART4: predict other cars' state [ id, x, y, vx, vy, s, d]
            double duration = ( N_SAMPLES*DT  - subpath_size * PATH_DT);
            if(debug) cout<<"prediction duration:"<<duration<<endl;
            vector<Vehicle> other_cars;//clear when a new message event starts
            map<int, vector<vector<double>>> predictions;
            for (auto sf: sensor_fusion) {
                Vehicle other_car = Vehicle(sf[5], sqrt(pow((double)sf[3], 2) + pow((double)sf[4], 2)), 0, sf[6], 0, 0);
                other_cars.push_back(other_car);
                int v_id = sf[0];
                //cout<<v_id<<endl;
                vector<vector<double>> preds = other_car.generate_predictions(subpath_size * PATH_DT, duration);
                predictions[v_id] = preds;
                if(debug){
                    cout<<"id:"<<v_id<<" predictions size:"<<preds.size()<<" one length:"<<preds[0].size()<<endl;
                }
            }
            
            
            //PART5: check whether some other car is left or right to our car.
            bool car_to_left = false, car_to_right = false, car_just_ahead = false;
            for (Vehicle other_car: other_cars) {
                double s_diff = fabs(other_car.s - car_s);
                if (s_diff < FOLLOW_DISTANCE) {
                    double d_diff = other_car.d - car_d;
                    if (d_diff > 2 && d_diff < 6) {
                        car_to_right = true;
                    } else if (d_diff < -2 && d_diff > -6) {
                        car_to_left = true;
                    } else if (d_diff > -2 && d_diff < 2) {
                        car_just_ahead = true;
                    }
                }
            }
            ego_car.update_available_states(car_to_left, car_to_right);
            if(debug) {
                cout<<car_to_left<<car_to_right<<car_just_ahead<<endl;
                cout<<ego_car.state<<endl;
                for(auto anystate:ego_car.available_states) cout<<anystate<<endl;
            }
            
            
            //PART6: find the best trajectory through which the ego car should move
            vector<vector<double>> best_frenet_traj, best_target;
            double best_cost = MAX_DOUBLE;
            string best_traj_state = "";
            for (string state: ego_car.available_states) {
                vector<vector<double>> target_s_and_d = ego_car.get_target_for_state(state, predictions, duration, car_just_ahead);
                vector<vector<double>> possible_traj = ego_car.generate_traj_for_target(target_s_and_d, duration);
                double current_cost = calculate_total_cost(possible_traj[0], possible_traj[1], predictions);
                if (current_cost < best_cost) {
                    best_cost = current_cost;
                    best_frenet_traj = possible_traj;
                    best_traj_state = state;
                    best_target = target_s_and_d;
                }
                if(debug) cout<<"best_cost:"<<best_cost<<endl;
            }
            
            
            // PART7: make a new path for the ego car
            vector<double> traj_x, traj_y, traj_s;
            vector<double>traj_inter_x, traj_inter_y, traj_inter_s;
            double prev_s = ego_car.s - ego_car.s_d * PATH_DT;
            // first two points of coarse trajectory, to ensure spline begins smoothly
            if (subpath_size >= 2) {
                traj_s.push_back(prev_s);
                traj_x.push_back(previous_path_x[subpath_size-2]);
                traj_y.push_back(previous_path_y[subpath_size-2]);
                traj_s.push_back(ego_car.s);
                traj_x.push_back(previous_path_x[subpath_size-1]);
                traj_y.push_back(previous_path_y[subpath_size-1]);
            } else {
                double prev_s = ego_car.s - 1;
                double prev_x = ego_car.pos_x - cos(deg2rad(ego_car.angle));
                double prev_y = ego_car.pos_y - sin(deg2rad(ego_car.angle));
                traj_s.push_back(prev_s);
                traj_x.push_back(prev_x);
                traj_y.push_back(prev_y);
                traj_s.push_back(ego_car.s);
                traj_x.push_back(ego_car.pos_x);
                traj_y.push_back(ego_car.pos_y);
            }
            // spline interpolation requires >2 points.
            // we assume in 25m, 50, speed stays almost same.
            double target_s1 = ego_car.s + 25, target_s2=ego_car.s+50;
            double target_d1 = best_target[1][0], target_d2=best_target[1][0];
            vector<double> target_xy1 = getXY(target_s1, target_d1, nearby_inter_s, nearby_inter_x, nearby_inter_y);
            vector<double> target_xy2 = getXY(target_s2, target_d2, nearby_inter_s, nearby_inter_x, nearby_inter_y);
            double target_x1 = target_xy1[0];double target_y1 = target_xy1[1];
            double target_x2 = target_xy2[0];double target_y2 = target_xy2[1];
            traj_s.push_back(target_s1);traj_s.push_back(target_s2);
            traj_x.push_back(target_x1);traj_x.push_back(target_x2);
            traj_y.push_back(target_y1);traj_y.push_back(target_y2);
            if(debug){
                cout<<"s";for(auto s:traj_s) cout<<" "<<s;cout<<endl;
                cout<<"x";for(auto x:traj_x) cout<<" "<<x;cout<<endl;
                cout<<"y";for(auto y:traj_s) cout<<" "<<y;cout<<endl;
            }
            
            // PART8: get next s values
            double target_s_dot = best_target[0][1];
            double current_s = ego_car.s;
            double current_v = ego_car.s_d;
            double current_a = ego_car.s_dd;
            for (int i = 0; i < (NUM_PATH_POINTS - subpath_size); i++) {
                double v_incr;
                double dist_v=(target_s_dot - current_v);
                double abs_dist_v=fabs(dist_v);
                v_incr=(abs_dist_v < VELOCITY_INCREMENT_LIMIT)? 0:dist_v/abs_dist_v*VELOCITY_INCREMENT_LIMIT;
                current_v += v_incr;
                current_s += current_v * PATH_DT;
                traj_inter_s.push_back(current_s);
            }
            if(debug){
                cout<<"current_s:"<<current_s<<" current_v:"<<current_v<<" current_a:"<<current_a<<endl;
                cout<<"target_s_dot:"<<target_s_dot<<endl;
            }
            
            //PART9: interpolate x,y given s for ego car trajectory
            traj_inter_x = interpolate_points(traj_s, traj_x, traj_inter_s);
            traj_inter_y = interpolate_points(traj_s, traj_y, traj_inter_s);
            // use previous path
            for(int i = 0; i < subpath_size; i++) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            } 
            // and use new path
            for (int i = 0; i < traj_inter_x.size(); i++) {
                next_x_vals.push_back(traj_inter_x[i]);
                next_y_vals.push_back(traj_inter_y[i]);
            }
               
            // TODO done.
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
