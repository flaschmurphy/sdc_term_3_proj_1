#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

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
string hasData(string s) 
{
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
	double closestLen = 100000; //large number
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

int NextWaypoint(double x, double y, double theta, const vector<double>
        &maps_x, const vector<double> &maps_y)
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
        closestWaypoint = 0;
    }
    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double>
        &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x, maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
		prev_wp  = maps_x.size()-1;

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
		frenet_d *= -1;

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const
        vector<double> &maps_x, const vector<double> &maps_y)
{
    // Find the nearest map_s that is less than car_s
	int prev_wp = -1;
	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
		prev_wp++;

    // Wraps back to zero at the end of the list
	int wp2 = (prev_wp+1)%maps_x.size();

    // The heading is the angle between the next wp and the current one
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

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";

  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

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

  // Start in lane 1
  int lane = 1;

  // Have a reference velocity (mph) to target. This is set to zero initially
  // and incrementally increased at the beginning of the simulation
  double ref_vel = 0;

  // This is used to stabilize the perceived state of the lanes (occupied or free)
  deque<int> left_lane_occupancy = {0, 0, 0, 0, 0};
  deque<int> middle_lane_occupancy = {0, 0, 0, 0, 0};
  deque<int> right_lane_occupancy = {0, 0, 0, 0, 0};

  h.onMessage([&map_waypoints_x,
               &map_waypoints_y,
               &map_waypoints_s,
               &map_waypoints_dx,
               &map_waypoints_dy,
               &lane,
               &left_lane_occupancy,
               &middle_lane_occupancy,
               &right_lane_occupancy,
               &ref_vel]
               (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

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

            int lane_width = 4;

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
          	vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];


            // TODO: define a path made up of (x,y) points that the car will
            // visit sequentially every .02 seconds

            int prev_size = previous_path_x.size();

            //////////////////////////////////////////////////////////
            // Begin fusion data processing and lane change handling
            //////////////////////////////////////////////////////////

            // Consider the sensor fusion data to see if there are vehicles in
            // our path. Decide on speed and lane accordingly, before passing
            // the values to the path generation code.
            if (prev_size > 0)
                car_s = end_path_s;

            // Store the indices of any cars that are close to the ego car
            vector<int> close_cars;

            // Scalar to convert miles per hour to meters per sec
            float mph2mps= 0.44704; 

            // the "front_car" is the car that's directly in front of our car in the same lane 
            int front_car_idx = 999; // just some large number
            int front_car_distance = 999; 
            int front_car_speed = 999;

            double free_space_lane0 = 999;
            double free_space_lane1 = 999;
            double free_space_lane2 = 999;
            int best_of_lane_0_or_2 = 0; // arbitrarily prefer lane 0 over lane 2

            bool lane0_free = true;
            bool lane1_free = true;
            bool lane2_free = true;

            // Define the min safe distance in front and behind the ego car
            int min_rear_clearance = 20;
            int min_forward_clearance = 15;

            // Minimum safe distance to the car in front in the same lane
            int min_samelane_clearance = min_forward_clearance+10;

            // Max acceleration and deceleration. Both are allowed to be as much as 10m/s^2
            float max_acceleration = 1.7;
            float max_deceleration = 1.9;
            float actual_deceleration = 0;

            // Difference in velocity between ego and vehicle in front, actual and normalized
            double vel_diff = 0;
            double vel_diff_norm = 0;

            // Examine all the sensor fusion data (i.e. all the other cars on the road)
            for (int i=0; i<sensor_fusion.size(); i++)
            {
                int check_car_lane;
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_car_v = sqrt(vx*vx+vy*vy);
                double check_car_s = sensor_fusion[i][5];
                float check_car_d = sensor_fusion[i][6];

                // The project walk through included the line below which takes
                // into account the fact that we need to consider the future
                // state of all vehicles not just their instantaneous state.
                // However during my testing I found this code actually makes
                // the performance worse not better
                //check_car_s += ((double)prev_size*0.2*check_car_v);

                // Left lane
                if (check_car_d < 4.0) // 4 is the lane width, so <4 means lane 0
                {
                    check_car_lane = 0;
                    if ((check_car_s >= car_s) && ((check_car_s - car_s) < min_forward_clearance))
                        lane0_free = false;
                    else if ((check_car_s < car_s) && ((car_s - check_car_s) < min_rear_clearance))
                        lane0_free = false;
                    if (abs(check_car_s - car_s) < free_space_lane0)
                        free_space_lane0 = abs(check_car_s - car_s);

                }
                // Middle lane
                else if (check_car_d >= 4.0  && check_car_d <= 8.0)
                {
                    // Check state of middle lane
                    check_car_lane = 1;
                    if ((check_car_s >= car_s) && ((check_car_s - car_s) < min_forward_clearance))
                        lane1_free = false;
                    else if ((check_car_s < car_s) && ((car_s - check_car_s) < min_rear_clearance))
                        lane1_free = false;

                    // Update the free space value
                    if (abs(check_car_s - car_s) < free_space_lane1)
                        free_space_lane1 = abs(check_car_s - car_s);
                }
                // Right lane
                else if (check_car_d > 8.0)
                {
                    check_car_lane = 2;
                    if ((check_car_s >= car_s) && ((check_car_s - car_s) < min_forward_clearance))
                        lane2_free = false;
                    else if ((check_car_s < car_s) && ((car_s - check_car_s) < min_rear_clearance))
                        lane2_free = false;
                    if (abs(check_car_s - car_s) < free_space_lane2)
                        free_space_lane2 = abs(check_car_s - car_s);
                }

                // Deal with any car that is in our current lane
                if (check_car_lane == lane)
                {
                    // If the car is behind us, just ignore it. This is ok for
                    // any car that's in the same lane as the ego vehicle
                    if (check_car_s < car_s)
                        continue;

                    if ((check_car_s > car_s) && ((check_car_s - car_s) < min_samelane_clearance))
                    {
                        close_cars.push_back(i);
                        if ((check_car_s - car_s) < front_car_distance)
                        {
                            front_car_idx = i;
                            front_car_speed = check_car_v;
                            front_car_distance = check_car_s - car_s;
                        }
                    }
                }
            }

            // Update the occupancy deques 
            left_lane_occupancy.pop_front();
            middle_lane_occupancy.pop_front();
            right_lane_occupancy.pop_front();

            if (lane0_free)
                left_lane_occupancy.push_back(0);
            else
                left_lane_occupancy.push_back(1);

            if (lane1_free)
                middle_lane_occupancy.push_back(0);
            else
                middle_lane_occupancy.push_back(1);

            if (lane2_free)
                right_lane_occupancy.push_back(0);
            else
                right_lane_occupancy.push_back(1);

            // By default best_of_lane_0_or_2 is already set to 0
            if (free_space_lane2 > free_space_lane0)
                best_of_lane_0_or_2 = 2;

            // Calculate the sliding window state of all lanes
            int lane0_sum = 0;
            for (int n : left_lane_occupancy)
                lane0_sum += n;

            int lane1_sum = 0;
            for (int n : middle_lane_occupancy)
                lane1_sum += n;

            int lane2_sum = 0;
            for (int n : right_lane_occupancy)
                lane2_sum += n;

            // Prefer the middle lane when reasonable
            if ((lane == 0 || lane == 2) && (lane1_sum == 0))
            {
                // If there is a lane change already in progress don't initiate another one
                if (((car_d >= 1) && (car_d <= 3)) || ((car_d >= 9) && (car_d <= 11)))
                {
                    if ((lane == 0) && ((free_space_lane0 < free_space_lane1) || (free_space_lane1 > 200)))
                        lane = 1;
                    else if ((lane == 2) && ((free_space_lane2 < free_space_lane1) || (free_space_lane1 > 200)))
                        lane = 1;
                }
            }

            // Decide whether or not to change lanes, change speed or maintain current state
            if (close_cars.size() > 0)
            {
                bool must_decelerate = true;
                // We only need to check what to do if in lane 1 below since
                // later on, if we are in any other lane we try to get back to
                // lane 1 asap anyway. So all we need to do here is make sure
                // we don't stay trapped in lane 1 unnecessarily.
                if (lane == 1)
                {
                    if ((best_of_lane_0_or_2 == 0) && (lane0_sum == 0))
                    {
                        // If there is a lane change already in progress don't initiate another one
                        if ((car_d >= 5) && (car_d <= 7))
                        {
                            lane = 0;
                            must_decelerate = false;
                        }
                    }
                    else if ((best_of_lane_0_or_2 == 2) && (lane2_sum == 0))
                    {
                        if ((car_d >= 5) && (car_d <= 7))
                        {
                            lane = 2;
                            must_decelerate = false;
                        }
                    }
                }

                // The case where we are in lane 0 or 2 is that earlier in the code we will
                // have already initiated a lane change back to the middle lane if it were
                // possible. So there's no need to check if the middle lane is free here again
                if (must_decelerate) 
                {
                    // When it's necessary to decelerate, chose a rate of
                    // deceleration that is equal to the max allowed scaled
                    // down by a factor proportional to the difference in
                    // velocity of the two cars
                    vel_diff = abs(front_car_speed - (car_speed*mph2mps));
                    vel_diff_norm = vel_diff / front_car_speed;
                    actual_deceleration = abs((vel_diff_norm * max_deceleration));
                    ref_vel -= actual_deceleration;
                }
            }
            // There is no car close in front, so accelerate to max velocity,
            // which is 50, but we need to stay a bit below to be on the safe side.
            else if (ref_vel < 48.0)
                ref_vel += max_acceleration;

            //////////////////////////////////////////////////////////////////
            //  Print some status info.
            //////////////////////////////////////////////////////////////////

            // Status of each lane, 1 means occupied and a * represents preference between lanes 0 and 2
            cout << !lane0_free; if (best_of_lane_0_or_2 == 0) cout << "* "; else cout << "  ";
            cout << !lane1_free << "  ";
            cout << !lane2_free; if (best_of_lane_0_or_2 == 2) cout << "*| "; else cout << " | ";

            // Occupancy history for the middle lane
            for (int n : middle_lane_occupancy) cout << n << " "; cout << "| ";

            // Ego d
            cout << car_d << " | ";

            // Speed of ego and car in front
            cout << car_speed*mph2mps << " " << front_car_speed << " | ";

            // Amount of free space in each of the 3 lanes
            //cout << (int)free_space_lane0 << " " << (int)free_space_lane1 <<
            //    " " << (int)free_space_lane2 << " | ";

            // Distance to car in front, diff in velocity, normalized diff, planned deceleration
            //cout << front_car_distance << " " << vel_diff << " " <<
            //    vel_diff_norm << " " << actual_deceleration << " | ";

            cout << endl << flush;

            //////////////////////////////////////////////////////////
            // End fusion data processing and lane change handling
            //////////////////////////////////////////////////////////

            vector<double> ptsx;
            vector<double> ptsy;

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = car_yaw;

            if (prev_size < 2)
            {
                // Use two points that make the path tangent to the car
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

            vector<double> next_wp0 = getXY(car_s+30,
                    (2+lane_width*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+60,
                    (2+lane_width*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+90,
                    (2+lane_width*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            for (int i=0; i<ptsx.size(); i++)
            {
                double shift_x = ptsx[i] - ref_x;
                double shift_y = ptsy[i] - ref_y;

                // Convert to car coords. The origin is the last point of the
                // previous path and the car reference angle is 0 degrees
                ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
                ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
            }

            tk::spline s;
            s.set_points(ptsx, ptsy);

            vector<double> next_x_vals;
            vector<double> next_y_vals;

            for (int i=0; i<previous_path_x.size(); i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));

            double x_add_on = 0;

            for (int i=1; i<=50-previous_path_x.size(); i++)
            {
                double N = (target_dist/(0.02*ref_vel/2.24));
                double x_point = x_add_on+(target_x)/N;
                double y_point = s(x_point);

                x_add_on = x_point;

                double x_ref = x_point;
                double y_ref = y_point;

                x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
                y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

                x_point += ref_x;
                y_point += ref_y;

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
            }

            // END

          	json msgJson;
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
