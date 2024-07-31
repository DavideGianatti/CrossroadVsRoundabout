#include "trafficlight.h"
using namespace std;

// Implementations of trafficlight.h's classes and methods

trafficlight::trafficlight(vector<float> lightslights_time, int lightlight, float timetime)
: lights_time(lightslights_time), light(lightlight), time(timetime) {;}

trafficlight::trafficlight()
: lights_time({0,0}), light(0), time(0) {;}

void trafficlight::time_step() {
  time += dt;
  if( time >= lights_time[light] ) {
    light = 1 - light;
    time = 0;
  }
}

crossroad::crossroad(vector<trafficlight> trtr_light, vector<float> pp_in)
: tr_light(trtr_light), p_in(pp_in)
{
  queue = vector<int>(tr_light.size(), 0);
  time_wasted = 0;

  random_device rd; // seed
  mt19937 aux(rd());  gen = aux;
}

crossroad::crossroad(vector<float> pp_in)
: p_in(pp_in)
{
  tr_light.resize(p_in.size());
  queue = vector<int>(tr_light.size(), 0);
  time_wasted = 0;

  random_device rd; // seed
  mt19937 aux(rd());  gen = aux;
}

void crossroad::naive_time_set() {
  float w = 0;
  w = accumulate(p_in.begin(), p_in.end(), w);
  float T = t_mean_green * p_in.size();

  for(int i = 0; i < p_in.size(); i++) {
    tr_light[i].lights_time[1] = T * p_in[i] / w;
    tr_light[i].lights_time[0] = T - tr_light[i].lights_time[1];
  }
  set_starting_t();
}

void crossroad::time_step() {
  arriving();
  leaving();
  int sum = 0;
  time_wasted += accumulate(queue.begin(), queue.end(), sum) * dt;
  for(int i = 0; i < tr_light.size(); i++) tr_light[i].time_step();
}

void crossroad::arriving() {
  for(int i = 0; i < queue.size(); i++) {
    poisson = poisson_distribution<>(p_in[i] * dt);
    queue[i] += poisson(gen);
  }
}

void crossroad::leaving() {
  for(int i = 0; i < queue.size(); i++) {
    if(tr_light[i].light && (queue[i] > 0)) {
      queue[i] -= round(dt * cars_per_s);
      time_wasted += t_to_leave * round(dt * cars_per_s);
      if(queue[i] < 0) queue[i] = 0;
    }
  }
}

void crossroad::set_starting_t() {
  float aux = 0;
  tr_light[0].light = 1;
  for(int i = 1; i < tr_light.size(); i++) {
    aux += tr_light[i-1].lights_time[1];
    tr_light[i].time = tr_light[i].lights_time[0] - aux;
  }

}

void crossroad::print_info() {
  cout << "Queue:" << endl;
  for(int i = 0; i < queue.size(); i++) cout << i << ") " << queue[i] << endl;

  cout << "Time wasted: " << time_wasted << " s" << endl;

  cout << endl;
}
