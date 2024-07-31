#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <random>
using namespace std;

// Declarations of classes and methods usefull for the simulation of a crossroad with traffic-lights

#define cars_per_s 2  // # of cars that passes a green traffic light per second
#define dt 1          // round(dt * cars_per_s) must be >= 1
#define t_to_leave 4  // time (s) required to leave the crossroad
#define t_mean_green 15  // mean time of a green light



struct trafficlight {
  vector<float> lights_time;
  int light;
  float time;

  trafficlight(vector<float> lights_time, int light, float time);
  trafficlight();

  void time_step();
};

class crossroad {
public:
  vector<trafficlight> tr_light;
  vector<float> p_in;
  vector<int> queue;
  float time_wasted;
  mt19937 gen;        // pseudo-random number generator
  poisson_distribution<> poisson;

//public:
  crossroad(vector<trafficlight> t_light, vector<float> p_in);
  crossroad(vector<float> p_in);

  void print_info();
  void time_step();
  void naive_time_set();    // the i-th green time is proportional to p_in[i]

//private:
  void arriving();
  void leaving();
  void set_starting_t();  // sets the t off_set in order to have just one green light at a time
};
