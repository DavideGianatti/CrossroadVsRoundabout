#include "roundabout.h"
using namespace std;

/* Implementations of rounddabout.h's classes and methods */

car::car(int namename, int positionposition, int exitexit) : name(namename), position(positionposition), exit(exitexit) {;}
car::car() {;}

void car::accelerate() {
  velocity += (1 + faster*lane) * acc * dt;
  if(velocity > v_max) velocity = v_max;
}

bool car::operator==(const car& x) {
  return x.name == name;
}

bool car::operator==(const int& car_name) {
  return car_name == name;
}

bool car::operator!=(const car& x) {
  return x.name != name;
}

bool Collision::operator()(const car& y) {  // N = number of points of the roundabout
  int new_x = x.position + round(x.velocity * dt / dx + dim_car/dx - 1); // front of the car x after dt
  int new_y = y.position + round(y.velocity * dt / dx);              // back of the car y after dt
  if(new_y >= N) new_y -= N;
  if(new_x < N) {
    return ((x.position < new_y) && (new_y <= new_x)
            && (x.position < y.position) && (y.position <= new_x) /* [x,new_x] must contain y and new_y
                                                                     in order to have a collision */
            && ( ((x.lane == y.lane) && (y.turning == 0))
                || ((x.lane != y.lane) && (y.turning != 0))
                || ((x.lane != y.lane) && (x.turning != 0)) )
            && (x != y));
  }
  else {      // boundary madness
    new_x -= N;
    return ( (((x.position < new_y) && (new_y < N)) || ((0 <= new_y) && (new_y <= new_x)))
    && (((x.position < y.position) && (y.position < N)) || ((0 <= y.position) && (y.position <= new_x)))
    && ( ((x.lane == y.lane) && (y.turning == 0)) || ((x.lane != y.lane) && (y.turning != 0)) )
    && (x != y));
  }
}

round2::round2(float circ, vector<float> pos_float_exit, vector<float> pp_in, vector<float> pp_out)
: p_in(pp_in), p_out(pp_out)
{
  N_point = round(circ/dx);
  N_boxes = round(dim_car / dx);
  positions.resize( 2, vector<int>(N_point) );
  vehicles.resize(0);
  pos_exit = discretization(pos_float_exit, dx);
  queue = vector<int>(pos_exit.size());
  time_wasted = 0;


  // probably there's a smarter way of doing the following initialization
  random_device rd; // seed
  mt19937 aux(rd());  gen = aux;
  uniform = uniform_int_distribution<>(1, round(N_point/N_boxes * 10)); // N_point/N_boxes = max # of cars
  discrete = discrete_distribution<>(p_out.begin(), p_out.end());
}

void round2::arriving() {
  for(int i = 0; i < queue.size(); i++) {
    poisson = poisson_distribution<>(p_in[i] * dt);
    queue[i] += poisson(gen);
  }
}

void round2::cars_in() {
  car aux;
  int n_exit;
  for(int i = 0; i < queue.size(); i++) {
    if((queue[i]) && check_entrance(i)) {
      queue[i] -= 1;
      n_exit = i; // start the while loop
      while(n_exit == i) {  // exit have to be different from entrance
        n_exit = discrete(gen); // clearly there are smarter solutions
      }
      aux = car(choose_name(), pos_exit[i], n_exit);
      if( far_exit(aux) ) aux.turning = 1; // the car will use the fast lane
      new_car(aux);
    }
  }
}

int round2::choose_name() {
  while(true) {
    int trial_name = uniform(gen);
    if( find_car(trial_name) == vehicles.end() ) {
      return trial_name;
    }
  }
  return 0;
}

void round2::draw(const car& x) {
  int aux = x.position - 1;   // -1 compensates the first aux += 1
  for(int i = 0; i<N_boxes; i++) {
    aux += 1;
    if(aux == positions[x.lane].size()) aux = 0;   // closed circonference
    positions[x.lane][aux] = x.name;
  }
}

void round2::rub(const car& x) {
  int aux = x.position - 1;   // -1 compensates the first aux += 1
  for(int i = 0; i<N_boxes; i++) {
    aux += 1;
    if(aux == positions[x.lane].size()) aux = 0;   // closed circonference
    positions[x.lane][aux] = 0;
  }
}

void round2::new_car(car x) {
  vehicles.push_back(x);
  draw(x);
}

void round2::vanish_car(car& x) {
  rub(x);
  vehicles.erase( find_car(x.name) );
}

bool round2::far_exit(const car& x) {
  int half = x.position + round(N_point/2);
  if(half >= N_point) {
    half -= N_point;
    return ( (half < pos_exit[x.exit]) && (pos_exit[x.exit] <= x.position) );
  }
  else {
    return ( ((half < pos_exit[x.exit]) && (pos_exit[x.exit] <= (N_point-1)))
            || ((0 <= pos_exit[x.exit]) && (pos_exit[x.exit] <= x.position)) );
  }
}

bool round2::check_side(const car& x) {
  int sum = 0;
  sum = accumulate(positions[1 - x.lane].begin() + x.position,
                   positions[1 - x.lane].begin() + x.position + N_boxes, sum);
  return (sum == 0);
}

bool round2::check_entrance(const int& x) {
  if(pos_exit[x] == 0) { // boundary madness
    int sum = 0;
    sum = accumulate(positions[0].begin() + N_point - 1 - safety_distance,
                     positions[0].begin() + N_point - 1, sum)
        + accumulate(positions[0].begin(),
                     positions[0].begin() + N_boxes, sum);
    return (sum == 0);
  }
  else {
    int sum = 0;
    sum = accumulate(positions[0].begin() + pos_exit[x] - safety_distance,
                     positions[0].begin() + pos_exit[x] + N_boxes, sum);
    return (sum == 0);
  }
}

bool round2::check_exit(const car& x) {
  int old_pos = x.position - round(x.velocity * dt / dx);
  if(old_pos >= 0) {
    return ((old_pos < pos_exit[x.exit]) && (pos_exit[x.exit] <= x.position) && (x.lane == 0));
  }
  else {      // boundary madness
    old_pos += N_point;
    return ( ( ((old_pos < pos_exit[x.exit]) && (pos_exit[x.exit] < N_point))
            || ((0 <= pos_exit[x.exit]) && (pos_exit[x.exit] <= x.position)) )
           && (x.lane == 0));
  }
}

bool round2::gotta_exit(const car& x) {
  if(x.exit == 0) return ( (pos_exit[pos_exit.size()-1] <= x.position)
                         && (x.position < (N_point - 1)) );
  else return ( (pos_exit[x.exit-1] <= x.position) && (x.position < pos_exit[x.exit]) );
}

void round2::change_lane(car& x) {
  if(check_side(x)) {
    rub(x);
    x.lane = 1 - x.lane;
    x.turning = 0;
    draw(x);
  }
}

vector<car>::iterator round2::car_intheway(const car& x) {
  return find_if(vehicles.begin(), vehicles.end(), Collision(x, N_point));
}

vector<car>::iterator round2::find_car(const int& name) {
  return find(vehicles.begin(), vehicles.end(), name);
}

void round2::update_v() {
  int aux = 1;
  while(aux > 0) {    // aux == 0 means that there will be no collisions
    aux = 0;
    for(int i = 0; i < vehicles.size(); i++) {
      vector<car>::iterator car_itw = car_intheway(vehicles[i]);
      if(car_itw != vehicles.end()) {
        if(vehicles[i].position < (*car_itw).position) {
          // velocity such that the car will arrive one m away from the car ahead
          aux = ( ((*car_itw).position - vehicles[i].position - N_boxes) * dx - 1 ) / dt
                + (*car_itw).velocity;
          if((vehicles[i].velocity > aux) && (aux > 0 )){
            // velocity must always decrease (convergence)
            vehicles[i].velocity = aux;
          }
        }
        if(vehicles[i].position > (*car_itw).position) { // boundary madness
          aux = ( (N_point - 1 - vehicles[i].position + (*car_itw).position - N_boxes) * dx - 1 ) / dt
                + (*car_itw).velocity;
          if((vehicles[i].velocity > aux) && (aux > 0)) {
            // velocity must always decrease (convergence)
            vehicles[i].velocity = aux;
          }
        }
      }
    }
  }
}

void round2::clear() {
  for(int i = 0; i < vehicles.size(); i++) {
    rub(vehicles[i]);
  }
}

void round2::leave() {
  vector<int> aux(0);

  for(int i = 0; i < vehicles.size(); i++) {
    if(check_exit(vehicles[i])) aux.push_back(vehicles[i].name);
  }
  for(int j = 0; j < aux.size(); j++) {
    vanish_car( *find_car(aux[j]) );
  }
}

void round2::move() {
  clear();

  for(int i = 0; i < vehicles.size(); i++) {
    vehicles[i].position += round(vehicles[i].velocity * dt / dx);
    if(vehicles[i].position >= N_point) vehicles[i].position -= N_point;

    draw(vehicles[i]);
    vehicles[i].accelerate();
  }
}

void round2::time_step() {
  arriving();
  cars_in();
  update_v();
  move();

  for(int i = 0; i < vehicles.size(); i++) {
    if(vehicles[i].turning) change_lane(vehicles[i]);

    if((vehicles[i].lane == 1) && gotta_exit(vehicles[i])) vehicles[i].turning = 1;
  }

  leave();
  int sum = 0;
  time_wasted += (accumulate(queue.begin(), queue.end(), sum) + vehicles.size()) * dt;
}

vector<int> round2::discretization(vector<float> v, float k) {
  vector<int> v_disc(v.size());
  for(int i = 0; i < v.size(); i++) {
    v_disc[i] = round(v[i]/k);
  }
  return v_disc;
}

void round2::print_car_info(){
  cout << "Cars informations:" << endl;
  for(int i = 0; i < vehicles.size(); i++) {
    cout << i << ") car " << vehicles[i].name << ", position: " << vehicles[i].position*dx << " m, "
    << "lane: " << vehicles[i].lane << ", speed: " << vehicles[i].velocity*3.6 << " km/h, "
    << "turning: " << vehicles[i].turning << ", exit: " << vehicles[i].exit << endl;

  }
}

void round2::print_round_info() {
  cout << "Queue: " << endl;
  for(int i = 0; i<queue.size(); i++) {
    cout << i << ") " << queue[i] << endl;
  }
}

void round2::print_round() {
  int aux = 0;
  cout << "Roundabout:" << endl;
  for(int i = 0; i < positions[0].size(); i++) {
    cout << positions[1][i] << " | " << positions[0][i];
    if(i == pos_exit[aux]) {
      cout << " | (" << aux << ")" << endl;
      aux++;
      if(aux == pos_exit.size()) aux = 0;
    }
    else cout << endl;
  }
}

void round2::print_time_wasted() {
  cout << "Time wasted: " << time_wasted << " s" << endl;
}
