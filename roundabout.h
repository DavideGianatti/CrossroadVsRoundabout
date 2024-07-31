#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <random>
using namespace std;

/* Declarations of classes and methods usefull for the simulation of a roundabout */

#define dx 1   // unit of measure = 1 m, must be <= dim_car
#define dt 1   // unit of measure = 1 s
#define v_max 10 // maximum velocity (m/s)
#define v_min 3  // minimum velocity (m/s)
#define acc 2    // acceleration (m/s^2)
#define faster 1  // fraction of acc added to the cars in the second lane
#define dim_car 3 // car dimension (m)
#define safety_distance 6  // a car entering must have the this distance behind free (dimension 1/dx),
                           // it can be approximated by v_max * 1s / dx (could be too big)


struct car {
  car(int name, int position, int exit);
  car();

  float velocity = v_min;
  int turning = 0;   // turning indicator on/off
  int name;
  int position;   // m/dx
  int exit;
  int lane = 0;

  void accelerate();

  bool operator==(const car& x);
  bool operator==(const int& car_name);
  bool operator!=(const car& x);
};

class round2 {     // roundabout with two lanes

  int N_boxes;    // number of "boxes" occupied by a single car
  int N_point;          // number of spacial points
  vector<int> pos_exit; // input (m/dx), obtained from pos_float_exit
  vector<float> p_in;     // probabilities per unit time
  vector<float> p_out;    // probabilities (an improvement would be considering conditional ones)
  mt19937 gen;        // pseudo-random number generator
  vector<int> queue;
  poisson_distribution<> poisson;
  uniform_int_distribution<> uniform;
  discrete_distribution<> discrete;
  float time_wasted;

public:
  round2(float circ, vector<float> pos_float_exit, vector<float> p_in, vector<float> p_out);

  vector<vector<int>> positions;      /* positions[1][3:5] = 2, means that the car named "2" is in the
                                         second lane occupying positions from 3 to 5, name = "0" means
                                         no car there */
  vector<car> vehicles;
  void time_step();
  void print_time_wasted();
  void print_car_info();
  void print_round_info();
  void print_round();

private:
  void arriving();
  void cars_in();
  int choose_name();
  void new_car(car x);       // new car in the roundabout
  void draw(const car& x);   // draw a car
  void rub(const car& x);    // rub off a car
  void vanish_car(car& x);     // car exits the roundabout
  void change_lane(car& x);
  bool far_exit(const car& x);
  bool check_side(const car& x); // True if the side is free
  bool check_entrance(const int& x);
  bool check_exit(const car& x);
  bool gotta_exit(const car& x); // True if x have to take the next exit
  vector<car>::iterator find_car(const int& name);  // returns pointer to the car "name" in vehicles
  void update_v();    // the car must have the same velocity as the one in the way
  void move();
  void clear();
  void leave();
  vector<car>::iterator car_intheway(const car& x);
  vector<int> discretization(vector<float> v, float k);

};

class Collision {       // return True if the two cars will collide
  car x;
  int N;    // number of spacial points
public:
  Collision(car xx, int NN): x(xx), N(NN) {}
  bool operator()(const car& y);
};
