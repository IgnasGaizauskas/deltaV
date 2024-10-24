// deltaV.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <vector>
#include <math.h>
#include <algorithm>
#include <numeric>

constexpr double gravity = 9.801;     // m / s ^ 2
constexpr double air_density = 1.2;   // kg / m ^ 3
constexpr double M_air = 0.0289644;   // kg / mol
constexpr double R_gas = 8.3144598;   // J / (K * mol)
constexpr double T_standard = 273.15; // K

double altitude_density(double altitude) {
  double tmp = air_density * std::exp((-1 * gravity * M_air * altitude) / (R_gas * T_standard));
  return tmp;
}

class stage {
public: 
  double mass_m0;
  double mass_empty;
  double ve_surf;
  double ve_vac;
  double thrust;
  size_t stage_num;
  double seperation_velocity = 0.;
  double isp_altitude(double altitude) {
    double tmp = (ve_surf - ve_vac) * std::exp((-1 * gravity * M_air * altitude) / (R_gas * T_standard)) + ve_vac;
    return tmp;
  }
  stage(double start_mass, double empty_ratio, double surface_isp, double vacuum_isp, double TWR, size_t stage_number) {
    mass_m0 = start_mass;
    mass_empty = empty_ratio * start_mass;
    thrust = start_mass * gravity * TWR;
    ve_surf = surface_isp * gravity ;
    ve_vac = vacuum_isp * gravity;
    stage_num = stage_number;
  };
} ;


class Rocket {
  
  double add_active_mass(double payload) {
    double tmp_mass = 0.;
    for (auto&& stage : stages) {
      if (stage.mass_m0 > stage.mass_empty) {
        tmp_mass = tmp_mass + stage.mass_m0;
      }
    }
    return tmp_mass + payload;
  };
  double total_rocket_thrust(double altitude, size_t stage_num, double dt) {
    double tmp_thrust = 0.;
    for (auto&& stage : stages)
      //|| (stage.stage_num < stage_num && stage.mass_m0 > stage.mass_empty)
      if (stage.stage_num == stage_num && (stage.mass_m0 > stage.mass_empty)) {
        stage.mass_m0 = stage.mass_m0 - stage.thrust / ((stage.isp_altitude(altitude)))  * dt;
        //std::cout << stage.isp_altitude(altitude)/ gravity << " altitude " << altitude << "\n";
        tmp_thrust = tmp_thrust + stage.thrust;
      } 
    return tmp_thrust;
  };
  double total_rocket_thrust_air_ejector(double altitude, size_t stage_num, double dt) {
    double tmp_thrust = 0.;
    for (auto&& stage : stages)
      //|| (stage.stage_num < stage_num && stage.mass_m0 > stage.mass_empty)
      if (stage.stage_num == stage_num && (stage.mass_m0 > stage.mass_empty)) {
        stage.mass_m0 = stage.mass_m0 - stage.thrust / ((1400*9.8+0*2*stage.isp_altitude(altitude))) * dt;
        //std::cout << stage.isp_altitude(altitude)/ gravity << " altitude " << altitude << "\n";
        tmp_thrust = tmp_thrust + stage.thrust;
      }
    return tmp_thrust;
  };

  double total_rocket_thrust_subsonic(double altitude, size_t stage_num, double dt) {
    double tmp_thrust = 0.;
    for (auto&& stage : stages)
      //|| (stage.stage_num < stage_num && stage.mass_m0 > stage.mass_empty)
      if (stage.stage_num == stage_num && (stage.mass_m0 > stage.mass_empty)) {
        stage.mass_m0 = stage.mass_m0 - stage.thrust / (5000 * 9.8) * dt;
        //std::cout << stage.isp_altitude(altitude)/ gravity << " altitude " << altitude << "\n";
        tmp_thrust = tmp_thrust + stage.thrust;
      }
    return tmp_thrust;
  };

  double total_rocket_thrust_supersonic(double altitude, size_t stage_num, double dt, double speed) {
    auto isp = [speed]() {
      return ( - 3491.605 + (156648800 + 3491.605) / pow((1 + ((speed / 295.0) / 7.27439E-13)), 0.3390746))*9.8*0.8; };
    double tmp_thrust = 0.;
    for (auto&& stage : stages)
      //|| (stage.stage_num < stage_num && stage.mass_m0 > stage.mass_empty)
      if (stage.stage_num == stage_num && (stage.mass_m0 > stage.mass_empty)) {
        stage.mass_m0 = stage.mass_m0 - stage.thrust / isp() * dt;
        //std::cout << stage.isp_altitude(altitude)/ gravity << " altitude " << altitude << "\n";
        tmp_thrust = tmp_thrust + stage.thrust;
      }
    return tmp_thrust;
  };
  
public:
  std::vector<stage> stages;
  void add_stage(stage stage_to_add) {
    stages.push_back(stage_to_add);
  }
  double solve(double deltaV_target) {
    double dt = 0.1;
    double deltaV = 99999999;
    double payload = 0;
    double total_mass;
    double altitude = 1;
    std::vector<stage> tmp;
    size_t i = 0;
    while (deltaV > deltaV_target) {
      tmp = stages;
      deltaV = 300;
      payload = payload + 100;
      bool has_fuel = true;
      size_t stage_num = 1;
      double altitude = 0;
      double acceleration = gravity + 1;
      double dv = 0;
      while (has_fuel==true) {
        total_mass = add_active_mass(payload);
        double total_thrust = 0.0;
        if (deltaV < 295.0 * 1.5) {
          total_thrust = total_rocket_thrust_air_ejector(altitude, stage_num, dt);
          //total_thrust = total_rocket_thrust_subsonic(altitude, stage_num, dt);  
        }
        else if ( (deltaV < 295.0 * 15) && (deltaV > 295.0*1.5) ) {
          total_thrust = total_rocket_thrust_supersonic(altitude, stage_num, dt, deltaV);
        }
        else {
          total_thrust = total_rocket_thrust(altitude, stage_num, dt);
        }
        
        //total_thrust = total_rocket_thrust(altitude, stage_num, dt);
        //std::cout << "total_mass " << total_mass << "\n";
        if (total_thrust < 10.) {
          stage_num = stage_num + 1;
        }
        acceleration = total_thrust / total_mass ;
        
        dv = acceleration * dt;
        altitude = altitude + dv * dt;
        deltaV = deltaV + dv;

        if (total_mass == payload) {
          has_fuel = false;
        }

        for (auto&& stage : stages) {
          if (stage.mass_m0 < stage.mass_empty && stage.seperation_velocity == 0.) {
            stage.thrust = 0;
            stage.mass_m0 = 0;
            stage.seperation_velocity = deltaV;
            //std::cout << "separation velocity " << deltaV << "\n";
          }
        }

        //std::cout << "total_mass " << total_mass << "\n";
      }
      i += 1;
      stages = tmp;
      //std::cout << "Payload " << payload << " deltaV " << deltaV<< "\n";
    }
    return payload;
  }
};

//                         mass,   empty, sisp, visp,  TWR, stage_num
stage core_stage       ( 500000.0,  0.15,  315,  355, 0.67, 1 );
stage liquid_booster   ( 500000.0,  0.15,  315,  355, 2.00, 1 );
stage second_methalox  ( 250000.0,  0.15,  360,  370, 1.00, 2 );
stage second_methalox3 ( 600000.0,  0.06,  360,  370, 1.00, 2 );
stage second_methalox5 (2000000.0,  0.06,  360,  370, 1.00, 2 );
stage hydrolox_second  ( 600000.0,  0.15,  380,  420, 0.25, 1 );
stage hydrolox_secondx5(1000000.0,  0.06,  450,  450, 1.00, 2 );
stage hydrolox_shuttle ( 600000.0,  0.04,  380,  440, 0.25, 1 );
stage methalox_shuttle (  20000.0,  0.95,  300,  300, 0.10, 3 );
stage solid_booster    ( 250000.0,  0.08,  240,  270, 2.00, 1 );
stage sm_solid_booster ( 100000.0,  0.08,  240,  275, 2.00, 1 );
stage sm2_solid_booster(  28140.0,  0.10,  240,  275, 1.50, 1 );
stage jsk1             ( 140000.0,  0.06,  285,  315, 0.70, 1 );
stage jsk2             (  50000.0,  0.06,  295,  325, 1.00, 2 );
stage jsk3             (  10000.0,  0.12,  325,  325, 0.50, 3 );
stage nuclear          ( 100000.0,  0.15, 1000, 1000, 0.10, 3 );
stage UDMH_3rd         (  50000.0,  0.10,  320,  320, 0.10, 3 );
stage hydrogen_3rd     ( 250000.0,  0.10,  460,  460, 0.10, 3 );
stage methalox_3rd     (  30000.0,  0.10,  380,  380, 0.10, 3 );
stage hydrolox_first   ( 800000.0,  0.08,  380,  422, 0.50, 1 );

stage vthl1            (3000000.0,  0.05,  380,  430, 1.40, 1 );
stage vthl2            (2000000.0,  0.15,  380,  420, 0.80, 2 );

stage hthl1            ( 500000.0,  0.15,  380,  440, 0.40, 1 );
stage hthl2            ( 200000.0,  0.15,  375,  375, 1.50, 2 );

stage second_methalox7x(1100000.0,  0.06,  365,  375, 1.00, 2 );
stage hydrolox_second7x(1100000.0,  0.10,  455,  455, 1.00, 2 );

stage air_launch       ( 250000.0,  0.33, 1000,  330, 0.50, 2 );

stage air_launch_1st   (    20000,  0.08,  275,  315, 1.50, 1 );
stage air_launch_2nd   (     5000,  0.08,  295,  325, 1.50, 2 );

int main()
{
  const auto jupiter = new Rocket;
  jupiter->add_stage(core_stage);
  jupiter->add_stage(second_methalox);
  jupiter->add_stage(liquid_booster);
  jupiter->add_stage(liquid_booster);

  const auto payload = jupiter->solve(9250.0);
  std::cout << "payload: " << payload << " kg";
}

