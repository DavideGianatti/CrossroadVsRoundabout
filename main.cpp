#include "roundabout.h"
#include "trafficlight.h"
using namespace std;

void print_vec(vector<int> vec) {
  for(int i = 0; i < vec.size(); i++) cout << vec[i] << endl;
}

void print_2lane(vector<vector<int>> positions) {
  for(int i = 0; i < positions[0].size(); i++) {
    cout << positions[1][i] << "  " << positions[0][i] << endl;
  }
}

int main() {

  int Nt = 3600;             // seconds of simulation
  vector<float> p_in(4);     // non-normalized probability of a car arriving at each intersection
  p_in = {0.5,0.4,0.5,0.2};
  vector<float> p_out(4);     // non-normalized probability of a car exiting at each intersection
  p_out = {1,1,1,0.3};
  float circ;                 // lenght of the roundabout
  circ = 100;
  vector<float> pos_exit(4);  // positions of the roundabout's exits
  pos_exit = {0,20,60,80};

  cout << "ROUNDABOUT" << endl;
  round2 round(circ, pos_exit, p_in, p_out);
  for(int i = 0; i < Nt; i++){
    round.time_step();
  }
  round.print_round_info();
  round.print_car_info();
  round.print_time_wasted();
  cout << endl;

  cout << "TRAFFIC LIGHT" << endl;
  vector<trafficlight> t_light(4);       // definiton of the traffic lights times
  t_light[0] = trafficlight({40,20},1,0);
  t_light[1] = trafficlight({45,15},0,0);
  t_light[2] = trafficlight({40,20},0,0);
  t_light[3] = trafficlight({55,5},0,0);

  crossroad sem(t_light, p_in);
  sem.naive_time_set();
  for(int i = 0; i < Nt; i++) {
    sem.time_step();
  }
  sem.print_info();

  return 0;
}
