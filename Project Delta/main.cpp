//
//  main.cpp
//  Project Delta
//
//  Created by Wade Wilson on 4/12/17.
//  Copyright © 2017 Wade Wilson. All rights reserved.
//

#include <iostream>
#include <vector>
#include <random>
#include <assert.h>
#include <cmath>
#include <fstream>
#include <math.h>

#include "LY_NN.h"
#include "EA.hpp"

#define WWRAND (double)rand()/RAND_MAX
#define PI 3.14159265359

using namespace std;

class goal{
public:
    
    double xmin = 900;
    double xmax = 910;
    double ymin = 900;
    double ymax = 900;
    
};
class agent{
public:
    double x;
    double y;
    double theta;
    double omega;
    double fitness;
    double delta_theta;
    double distance;
    
    double u;
    double umax=15;
    double umin=-15;
    
    bool win;
    bool offmap;
    string goalnow;
    string goalpast;

    void init();
    void simulate(double v, double dt, double T);
    void checku();
    //void simulate();
    void eval();
    void checkpos();
    void check_goal(goal buoy);
};

class water{
public:
    double xmin=0;
    double xmax=1000;
    
    double ymin=0;
    double ymax=1000;
};

double simulator(double u);
double calc_theta(agent boata, goal buoy);
double calc_distance(agent boata, goal buoy);
bool check_goal(agent boata, goal buoy);


int main() {/// ----------------------------------------------MAIN---------------------------------------
    cout << "Main Start" << endl;
    srand(unsigned(time(NULL)));
    
    
    int pop_size=100;
    int max_generations=26;
    agent boat;
    boat.init();
    
    // Established Values
    double v =3;
    double dt=.2;
    double T=5;
    int tsmax = 5000;;

    goal buoy;
    neural_network NN;
    NN.setup(2,6,1);
    NN.set_in_min_max(1000, 1000); // delta theta
    NN.set_in_min_max(1000, 1000); // distance
    NN.set_out_min_max(-15, 15);

    
    vector<policy> population;
    int num_weights = NN.get_number_of_weights();
    
    population=EA_init(pop_size, num_weights);
    for(int generation = 0; generation<max_generations; generation ++){ ///GENERATION LOOP
        cout << "GENERATION  " << generation << endl;
    //population=EA_replicate(population, pop_size);
        
    for(int pop=0; pop<pop_size; pop++){ /// POPULATION LOOP
        for(int initf=0; initf<population.size(); initf++){ // resets fitness off all pops
            population.at(initf).fitness = -1;
        }
    NN.set_weights(population.at(pop).weights, true);
        boat.init();
        boat.offmap=false;
        int tstep = 0;
    for(int tstep=0; tstep<tsmax; tstep++){ ///SIMULATOR LOOP
        //cout << tstep << endl;
        assert(boat.x > 0);
        assert(boat.y > 0);
        assert(boat.x <1000);
        assert(boat.y <1000);
        vector<double> state;
        boat.delta_theta=calc_theta(boat, buoy);
        boat.distance=calc_distance(boat, buoy);
        state.push_back(boat.delta_theta);
        state.push_back(boat.distance);
        NN.set_vector_input(state);
        NN.execute();
        boat.u=NN.get_output(0);
        boat.checku();
        boat.simulate(v, dt, T);
        boat.checkpos();
        population.at(pop).fitness=population.at(pop).fitness+(boat.distance);
        if(generation == 25 ){
            if(pop==0){
        cout << boat.x << "\t" << boat.y << "\t" << boat.u <<endl;
        }
        }
        boat.check_goal(buoy);
        if(boat.win){
            population.at(pop).fitness=population.at(pop).fitness-100;
            break;
        }
        if( boat.offmap){
            population.at(pop).fitness=5000;
            break;
        }
        //population.at(pop).fitness=tstep;
        if(tstep > tsmax){
            break;
        }
        
                
        
    }//sim loop
        //population.at(pop).fitness=population.at(pop).fitness+(boat.distance)*4;
    }//pop loop
        population=EA_downselect(population, pop_size);
        assert(population.size() == pop_size/2);
        population=EA_replicate(population, pop_size);
        assert(population.size() == pop_size);

    }//gen loop
    
    
    
    
    return 0;
}
void agent::init(){
    x=10;
    y=10;
    theta=0;
    omega=0;
}

void agent::simulate(double v, double dt, double T){
    x=x+v*sin(theta)*dt;
    y=y+v*cos(theta)*dt;
    theta=theta+omega*dt;
    /*if (theta>(2*PI)){
        theta=theta-(2*PI);
    }
    if(theta< (-2*PI)){
        theta=theta+(2*PI);
    }*/
    omega=omega+(u-omega)*dt/T;
}

void agent::checku(){
    if(u > umax){
        u= umax;
    }
    if (u < umin){
        u=umin;
    }
}


void agent::checkpos(){
    if(x>1000){
        offmap=true;
    }
    if(x<0){
        offmap=true;
    }
    if(y>1000){
        offmap=true;
    }
    if(y<0){
        offmap=true;
    }

}

double calc_theta(agent boata, goal buoy){
    double dx1;
    double dx2;
    double dy1;
    double dy2;
    double dx;
    double dy;
    
    dx1=fabs(boata.x-buoy.xmin);
    dx2=fabs(boata.x-buoy.xmax);
    dy1=fabs(boata.y-buoy.ymin);
    dy2=fabs(boata.y-buoy.ymax);
    
    if(dx1>=dx2){
        dx=dx2;
    }
    else{
        dx=dx1;
    }
    if(dy1>=dy2){
        dy=dy2;
    }
    else{
        dy=dy1;
    }
    double dtheta_rad=atan2(dy,dx);
    return dtheta_rad;
}

double calc_distance(agent boata, goal buoy){
    double dx1;
    double dx2;
    double dy1;
    double dy2;
    double dx;
    double dy;
    boata.distance=0;
    
    dx1=fabs(boata.x-buoy.xmin);
    dx2=fabs(boata.x-buoy.xmax);
    dy1=fabs(boata.y-buoy.ymin);
    dy2=fabs(boata.y-buoy.ymax);
    
    if(dx1>=dx2){
        dx=dx2;
    }
    else{
        dx=dx1;
    }
    if(dy1>=dy2){
        dy=dy2;
    }
    else{
        dy=dy1;
    }
    double distance = sqrt(pow(dx, 2) + pow(dy,2));
    return distance;
}
void agent::check_goal(goal buoy){
    goalpast=goalnow;
    win = false;
    bool left=false;
    bool right = false;
    bool belowmax=false;
    bool abovemin=false;
    if(x>buoy.xmin){
        right =true;
    }
    else{
        right = false;
    }
    if(x<buoy.xmin){
        left =true;
    }
    else{
        left = false;
    }
    if(y < buoy.ymax+5){
        belowmax = true;
    }
    if(y > buoy.ymin-5){
        abovemin = false;
    }
    if((abovemin)&&(belowmax)){

        if(left){
            goalnow="left";
        }
        if(right){
            goalnow="right";
        }
        else{
            goalnow="no";
        }
    }
    if(goalpast=="left" && goalnow=="right"){
        win=true;
    }
    if(goalnow=="left" && goalpast=="right"){
        win=true;
    }
    //return false;
}

