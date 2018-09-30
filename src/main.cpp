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

const double MAX_ACCELERATION = .224; // mph per 20 ms
const double MAX_VELOCITY = 49.5; // mph
const double CNV_MS_TO_MPH = 2.2369; // 1 m/s = 2.2369 mph
const double COST_HYST = 0.1; // Hysteresis for cost comparisons
const double LANE_CHANGE_HYST = 150; // Minimum number of cycles a 20ms between lane changes
const double MIN_DISTANCE = 30; // Safety distance for lane change manouvers
const double DISTANCE_NORMALIZER = 150.0; // Max distance difference used in cost function
const double VELOCITY_NORMALIZER = 30.0; // Maximum velocity difference used in cost function
//#define DEBUG_OUTPUT 1

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
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
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
	double frenet_s = 0;
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

  int lane = 1; // Starting in middle lane
  double ref_vel = 0; // Starting velocity mph
  double num_cycles = 0; // Measure of time, 1 cycle = 0.2 seconds
  double last_lane_change = 0; // Take note of time of last lane change

  h.onMessage([&last_lane_change, &num_cycles, &ref_vel, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
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

// Start Code Niels
            int prev_size = previous_path_x.size();

            if(prev_size > 0)
            {
              car_s = end_path_s; // Ensure continuity between paths
            }

            // The following variables are used in the cost calculations
            vector<vector<double>> closest; // closest car in front and behind our car, per lane
            vector<double> tmp_min = {-99999, -99999}; // s value, velocity (mph)
            vector<double> tmp_max = {99999, -99999};
            closest.push_back(tmp_min);
            closest.push_back(tmp_max);
            vector<vector<vector<double>>> lanes; // For each lane we store closest car in front and behind
            lanes.push_back(closest);
            lanes.push_back(closest);
            lanes.push_back(closest);  

            num_cycles++; // Keep track of "time" - 1 cycle = 20 ms           

            // Part 1: Analyze fustion data
            // Try to determine if there are cars in our lane 
            // or in the relevant lanes next to us
            // The following variables are used to determine when a lane change is impossible
            bool car_in_front = false, car_to_left = false, car_to_right = false;


            // Iterate through all the fusion measurements
            for(int i = 0; i< sensor_fusion.size(); i++)
            {
              // We want to determine lane and position of current car relative to our car
              // First determine lane of current car
              float curr_d = sensor_fusion[i][6]; // d measurement for current car

              // This variable marks in which lane the current car is
              // It is possible that a car can be in 2 lanes simultaneously, depending on its d value
              vector<bool> in_lane = {false, false, false};

              if( curr_d > 0 && curr_d < 5 ) in_lane[0] = true;
              if( curr_d > 3 && curr_d < 9 ) in_lane[1] = true;
              if( curr_d > 7 && curr_d < 12 ) in_lane[2] = true;

              // Now determine the s position of the current car
              // Current car velocity and s position
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];
              // Future s position for current car using number of points from previous trajectory
              check_car_s += (double)prev_size * .02 * check_speed;

              // Now update the "closest car" variable
              if( check_car_s < car_s ) // Current car is behind us
              {
                for( int j=0; j<3; j++ ) // Check all lanes in which our car could be
                {
                  if( in_lane[j] == true )
                  {
                    if( lanes[j][0][0] < check_car_s ) // Closer behind that present measurement
                    {
                      lanes[j][0][0] = check_car_s;
                      lanes[j][0][1] = check_speed * CNV_MS_TO_MPH;
                    }
                  }
                }
              }
              else // Current car is in front of us
              {
                for( int j=0; j<3; j++ ) // Check all lanes in which our car could be
                {
                  if( in_lane[j] == true )
                  {
                    if( lanes[j][1][0] >= check_car_s ) // Closer in front than present measurement
                    {
                      lanes[j][1][0] = check_car_s;
                      lanes[j][1][1] = check_speed * CNV_MS_TO_MPH;                  
                    }
                  }
                }
              }

              // Collate lane and position of current car to determine whether
              // it is relevant for lane changes of our car
              for(int j=0; j<3; j++)
              {
                if( in_lane[j] == true )
                {
                  if( j == lane ) // Current car in same lane as our car
                  {
                    if( (check_car_s > car_s) && ((check_car_s - car_s) < MIN_DISTANCE) )
                    // The car is within 30 meters in front of us
                    {
                      car_in_front = true;
                    }
                  }
                  
                  if( j - lane == -1 ) // Current car in the lane to the left of our car
                  {
                    if( (car_s - MIN_DISTANCE < check_car_s) && (car_s + MIN_DISTANCE > check_car_s) )
                    // Current car is with +/- 45 meters of our car
                    {
                      car_to_left = true;
                    } 
                  }
                  
                  if( j - lane == 1 ) // Current car in the lane to the right of our car
                  {
                    if( (car_s - MIN_DISTANCE < check_car_s) && (car_s + MIN_DISTANCE > check_car_s) )
                    // Current car is with +/- 45 meters of our car
                    {
                      car_to_right = true;
                    } 
                  }
                }
              }            
            }

            // Part 2: Cost functions
            // Let's calculate a cost for each of the 3 lanes
            vector<float> total_cost(3, 0.0);

#ifdef DEBUG_OUTPUT
            std::cout << std::endl << "Cycle " << num_cycles << std::endl;
            std::cout <<  "Our (lane, s, v) = " << lane << ", " << car_s << ", " << car_speed << endl;
            for(int i=0; i<3; i++)
            {
              std::cout <<  "Lane " << i << std::endl;
              std::cout << "Front (s, v) = " << lanes[i][1][0] << ", " << lanes[i][1][1] << std::endl;
            }            
#endif

            for(int i=0; i<3; i++)
            {
              // Cost for distance in front of our car
              double distance_in_front = lanes[i][1][0] - car_s;
              float in_front_cost = 0.0;
              if(lanes[i][1][0] != 99999)
              {
                // Normalize the distance to be between 0 and 1
                // Choose 150 as the max distance to consider for the cost function
                in_front_cost = (float) distance_in_front / DISTANCE_NORMALIZER;
                if(in_front_cost > 1.0)
                {
                  in_front_cost = 1.0;
                }

                in_front_cost = 1.0 - in_front_cost; // Cost shall decrease the further away the car is
              }

              // Cost for distance behind our car
              double distance_behind = car_s - lanes[i][0][0];
              float behind_cost = 0.0;
              if(lanes[i][0][0] != -99999)
              {
                // Normalize the distance to be between 0 and 1
                // Choose 150 as the max distance difference to consider for the cost function
                behind_cost = (float) distance_behind / DISTANCE_NORMALIZER;
                if(behind_cost > 1.0)
                {
                  behind_cost = 1.0;
                }

                behind_cost = 1.0 - behind_cost; // Cost shall decrease the further away the car is
              }

              // Cost for difference in velocity for car in front of our car
              float speed_diff_cost = 0.0;
              double speed_diff = 0;
              if(lanes[i][1][1] != -99999)
              {
                speed_diff = car_speed - lanes[i][1][1];

                // Normalize the speed difference to be between -1 and 1 
                // Choose 30 as the max velocity difference to consider for the cost function
                speed_diff_cost = (float) speed_diff / VELOCITY_NORMALIZER;
                if(speed_diff_cost > 1.0)
                {
                  speed_diff_cost = 1.0;
                }
                else if( speed_diff_cost < -1.0)
                {
                  speed_diff_cost = -1.0;
                }
              }

              // As total cost we simply sum the 3 components, generating a number between -1 and 3
              total_cost[i] = (in_front_cost + behind_cost + speed_diff_cost);

#ifdef DEBUG_OUTPUT
              std::cout << "Total cost for Lane " << i << " " << total_cost[i] << std::endl;
#endif
            }


            // Part 3: Path planning, determine actions
            // Determing which lane and reference velocity to use

            // Do not change lanes immediately after start (500 cycles = 10 seconds)
            // or when the car is within 300 meters from wraparound point
            if(num_cycles < 500 || car_s < 300) 
            {
              if( car_in_front )
              {
                ref_vel -= MAX_ACCELERATION;
              }
              else
              {
                ref_vel += MAX_ACCELERATION;
              }
            }
            else
            {
              // First priority is to handle the scenario that there is a car in front of us
              if( car_in_front )
              {
                if( lane == 1 && !car_to_left && !car_to_right )
                // We are in the middle lane, and nothing to the left or right
                {
                  // Select lane with lowest cost
                  if( total_cost[0] < total_cost[2] )
                  {
                    lane--;
                    last_lane_change = num_cycles;
                  }
                  else
                  {
                    lane++;
                    last_lane_change = num_cycles;
                  }
                }
                else
                {
                  // Prioritize lane change to the left
                  if( !car_to_left && lane > 0 && (num_cycles - last_lane_change) > LANE_CHANGE_HYST )
                  {
                    lane--;
                    last_lane_change = num_cycles;
                   }
                  // Alternatively try lane change to the right
                  else if( !car_to_right && lane < 2 && (num_cycles - last_lane_change) > LANE_CHANGE_HYST )
                  {
                    lane++;
                    last_lane_change = num_cycles;
                  }
                  // We cannot change lane so we have to reduce velocity
                  else
                  {
                    if( lanes[lane][1][0] - car_s < 15 )
                    // Disaster somebody changes lane directly in front of us 
                    {
                      ref_vel -= 2*MAX_ACCELERATION; // Accept high decelleration to avoid collision
                    }
                    else
                    {
                      ref_vel -= MAX_ACCELERATION;
                    }
                  }
                }
              }
              // Second priority: handle situation with no car in front of us
              else
              {
                // Select another lane if cost is lower
                // We are in lane 0 and lane 1 is also free
                if( lane == 0 && !car_to_right && (num_cycles - last_lane_change) > LANE_CHANGE_HYST )
                {
                  if( (total_cost[1] + COST_HYST) < total_cost[0] )
                  {
                    lane++;
                    last_lane_change = num_cycles;
                  }
                }
                // We are in lane 2 and lane 1 is also free
                else if( lane == 2 && !car_to_left && (num_cycles - last_lane_change) > LANE_CHANGE_HYST )
                {
                  if( (total_cost[1] + COST_HYST) < total_cost[2] )
                  {
                    lane--;
                    last_lane_change = num_cycles;
                  }
                }
                // We are in lane 1
                else if( lane == 1 && (num_cycles - last_lane_change) > LANE_CHANGE_HYST )
                {
                  // Both lane 0 and 2 are also free
                  if( !car_to_left && !car_to_right )
                  // Select between all 3 lanes
                  {
                    lane = distance(total_cost.begin(),min_element(total_cost.begin(), total_cost.end()));
                    if(lane != 1)
                    {
                      last_lane_change = num_cycles;                      
                    }
                  }
                  else if( !car_to_left && car_to_right )
                  // Lane 0 is also free
                  {
                    if( (total_cost[0] + COST_HYST) < total_cost[1] )
                    {
                      lane--;
                      last_lane_change = num_cycles;
                    }
                  }
                  else if( car_to_left && !car_to_right )
                  // Lane 2 is also free
                  {
                    if( (total_cost[2] + COST_HYST) < total_cost[1] )
                    {
                      lane++;
                      last_lane_change = num_cycles;
                    }
                  }
                }

                // Increase velocity to get a close as possible to the max allowed velocity
                if( ref_vel < MAX_VELOCITY )
                {
                  ref_vel += MAX_ACCELERATION;
                }
              }
            }

            // Perform safety checks to ensure we never exceeed the max allowed velocity
            if(ref_vel > MAX_VELOCITY)
            {
              ref_vel = MAX_VELOCITY;
            }
            if(ref_vel < 0)
            {
              ref_vel = 0;
            }

            // Part 4: Path planning, trajectory generation
            // Next step is to calculate a new trajectory, using the new values for
            // lane and reference velocity calculated above
            // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30 m
            vector<double> ptsx;
            vector<double> ptsy;

            // reference x, y, yaw states
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            // If previous size is almost emoty use the car as starting reference  
            if(prev_size < 2)
            {
              // Use 2 points that make the path tangent to the car
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
            }
            else // Use previous paths end as starting reference
            {
              // Use last and second last point of previous path
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              double ref_x_prev =  previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
            }

            // In Frenet add 3 more points evenly spaced 30 meters ahead of the starting point.
            vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            for(int i = 0; i < ptsx.size(); i++)
            {
              // Shift car reference angle to 0 degrees
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
              ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
            }

            // Create spline
            tk::spline s;
            s.set_points(ptsx, ptsy);

            // Define the actual (x,y) points for the planner
          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            // Start with all of the previous path points from the last time
            for(int i = 0; i < prev_size; i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);              
            }

            // Calculate how to break up spline points so we travel at our desired reference velocity
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);

            double x_add_on = 0;

            // Fill up the rest of the new trajectory
            for(int i = 1; i <= 50 - prev_size; i++)
            {
              double N = target_dist/(.02*ref_vel/2.24);
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              // Rotate back to global coordinates
              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }
// End code Niels

            // Send the new trajectory to the simulator
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
