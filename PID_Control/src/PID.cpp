#include "PID.h"
#include <iostream>
#include <vector>
using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {	
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  int_cte = 0;
  pre_cte = 0;
  diff_cte = 0;
  pre_sp = 0;
}

void PID::UpdateError(double cte) {
  diff_cte = cte - pre_cte;
  pre_cte = cte;
  int_cte += cte;
}

void PID::UpdateSpeed(double speed) {
  pre_cte = speed - 50;
  diff_cte = pre_cte - pre_sp;
  pre_sp = pre_cte;
  int_cte += pre_cte;
}

double PID::TotalError() {
  return (-Kp*pre_cte - Ki*int_cte - Kd*diff_cte);
}

