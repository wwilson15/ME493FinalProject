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
#include <cstdlib>
#include <limits>

#include "LY_NN.h"
#include "EA.hpp"

#define WWRAND (double)rand()/RAND_MAX
#define PI 3.14159265359

using namespace std;

class goal{ // goal is a large rectangle on the entire right side from x=950 to edge
public:
    
    double xmin = 950;
    double xmax = 1000;
    double ymin = 0;
    double ymax = 1000;
    double bestx=1000;
    double besty=500;
    
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

    void init();
    void simulate(double v, double dt, double T);
    void checku();
    void eval();
    void checkpos();
    void check_goal(goal buoy);
};

double calc_theta(agent boata, goal buoy);
double calc_distance(agent boata, goal buoy);
bool check_goal(agent boata, goal buoy);
void straightboat();

void testing(){
    agent B;
    B.init();
    B.x=0;
    B.y=0;
    B.theta=-135*PI/180;
    B.omega=0;
    B.u=0;
    for(int i=0; i<5; i++){
    B.simulate(4,.1,5);
        cout<< B.x << "\t" << B.y << "\t" << B.theta << endl;
    }
    
}
double v2g(double x, double y){ //output in rad
    double xg=1000;
    double yg=500;
    double dx;
    double dy;
    double out;
    if(y>500){
        dx=xg-x;
        dy=y-yg;
        out=PI/2+atan2(dy,dx);
    }
    else{
        dx=xg-x;
        dy=yg-y;
        out=atan2(dy,dx);
    }
    return out;
}
double b2v(double theta){ //output in rad
    double out;
    theta*=PI/180;
    out=theta;
    return out;
}
void testing_b2g(){
    double x=100;
    double y=900;
    double theta=45;
    double o1=b2v(theta); //rad
    double o2=v2g(x,y); // rad
    double tt=(o2-o1)*180/PI;
    cout<< "Boat to Verticle [degrees]   " << o1*180/PI << endl;
    cout<< "Verticle to Goal [degrees]   " << o2*180/PI << endl;
    cout << "Boat to Goal [degrees]   "<< tt << endl;
    
}

double generateGaussianNoise(double mu, double sigma) //// From wikipedia
{
    const double epsilon = std::numeric_limits<double>::min();
    const double two_pi = 2.0*3.14159265358979323846;
    
    static double z0, z1;
    static bool generate;
    generate = !generate;
    
    if (!generate)
        return z1 * sigma + mu;
    
    double u1, u2;
    do
    {
        u1 = rand() * (1.0 / RAND_MAX);
        u2 = rand() * (1.0 / RAND_MAX);
    }
    while ( u1 <= epsilon );
    
    z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
    z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
    return z0 * sigma + mu;
}


int main() {/// ------------------------------------------MAIN---------------------------------------
    cout << "Main Start" << endl;
    srand(unsigned(time(NULL)));
    
    //straightboat();
    //testingb2h();
    //testing_b2g();
    bool current = true;
    bool sensor_noise=true;
    bool actuator_noise=true;
    
    int pop_size=100;
    int max_generations=30;
    
    agent boat;
    boat.init();
    
    // Established Values
    double v =3;
    double dt=.2;
    double T=5;
    int tsmax = 5000;;

    goal buoy;
    neural_network NN;
    NN.setup(1,6,1);
    NN.set_in_min_max(-4, 4); // delta theta
    NN.set_out_min_max(-15*PI/180, 15*PI/180);

    
    ofstream outputFile;
    outputFile.open("LC.txt");
    vector<policy> population;
    int num_weights = NN.get_number_of_weights();
    
    population=EA_init(pop_size, num_weights);
    for(int generation = 0; generation<max_generations; generation ++){ ///GENERATION LOOP
        cout << "GENERATION  " << generation << endl;
        for(int initf=0; initf<population.size(); initf++){
            population.at(initf).fitness = 0;
        }
    for(int pop=0; pop<pop_size; pop++){ /// POPULATION LOOP

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
        if(sensor_noise){
            boat.delta_theta=calc_theta(boat, buoy)+generateGaussianNoise(0, .0000000000001);
        }
        else{
        boat.delta_theta=calc_theta(boat, buoy);
        }
        state.push_back(boat.delta_theta);
        NN.set_vector_input(state);
        NN.execute();
        if(actuator_noise){
            boat.u=NN.get_output(0)+generateGaussianNoise(0, .000000000000001);
        }
        else{
            boat.u=NN.get_output(0);
        }
        boat.checku();
        boat.simulate(v, dt, T);
        
        if(current){
            //cout << "test" << endl;
            boat.y+=generateGaussianNoise(0, .000000000001);
        }
        
        boat.checkpos();
        population.at(pop).fitness=population.at(pop).fitness + fabs(boat.delta_theta);
        if(generation == 7 ){
            if(pop==0){
        cout << boat.x << "\t" << boat.y << "\t" << boat.u << "\t" << boat.theta << "\t" << boat.delta_theta <<endl;
        }
        }
        boat.check_goal(buoy);
        if(boat.win){
            population.at(pop).fitness=population.at(pop).fitness;
           // cout<<"winner"<<endl;
            break;
        }
        if( boat.offmap){
            population.at(pop).fitness=10000;
            break;
        }
        //population.at(pop).fitness=tstep;
        if(tstep > tsmax){
            break;
        }
        
                
        
    }//sim loop
        
    }//pop loop
        population=EA_downselect(population, pop_size);
        assert(population.size() == pop_size/2);
        population=EA_replicate(population, pop_size);
        assert(population.size() == pop_size);

        if(outputFile.is_open()){
            for(int i=0; i<population.size(); i++){
                outputFile << population.at(i).fitness << "\t";
            }
            outputFile << "\t"<<"\t" << endl;
            //cout << "This generation output to text file" << endl;
        }
    }//gen loop
    
    outputFile.close();
    double initf=1000;
     int sto;
    for(int y=0;y<population.size();y++){
        if(population.at(y).fitness < initf){
            initf=population.at(y).fitness;
            sto=y;
        }
    }
    
    ////-------------------------getting best run sim ------------
    ofstream outputfile;
    outputFile.open("bestrun.txt");
    for(int k=0;k<tsmax;k++){
        vector<double> state;
        if(sensor_noise){
            boat.delta_theta=calc_theta(boat, buoy)+generateGaussianNoise(0, .25);
        }
        else{
            boat.delta_theta=calc_theta(boat, buoy);
        }
        state.push_back(boat.delta_theta);
        NN.set_vector_input(state);
        NN.execute();
        if(actuator_noise){
            boat.u=NN.get_output(0)+generateGaussianNoise(0, .1);
        }
        else{
            boat.u=NN.get_output(0);
        }
        boat.checku();
        boat.simulate(v, dt, T);
        
        if(current){
            cout << "current on" << endl;
            boat.y+=generateGaussianNoise(.1, .1);
        }
        if(outputFile.is_open()){
            outputFile << boat.x << "\t" << boat.y << "\t" << boat.theta << "\t" << "\t";
        }
        boat.checkpos();
        
        boat.check_goal(buoy);
        if(boat.win){
            // cout<<"winner"<<endl;
            break;
        }
        if( boat.offmap){
            break;
        }
    
    }///////////-----------best run sim end------------------------
    cout<<"Best run output to bestrun.txt" << endl;
    outputFile.close();
    return 0;
}
void agent::init(){
    x=50;
    y=600;
    theta=-PI/4;
    omega=0;
}

void agent::simulate(double v, double dt, double T){
    x=x+v*sin(theta)*dt;
    y=y+v*cos(theta)*dt;
    theta=theta+omega*dt;
    while(theta>=PI){
        theta=theta-2*PI;
    }
    while(theta<-PI){
        theta=theta+2*PI;
    }
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
    double dtheta=0;
    double t1;
    double t2;
    t1=boata.theta;  //BV
    t2=v2g(boata.x,boata.y); //VG
    if(boata.y<=500){
        if(boata.theta < PI/2 && boata.theta > -PI/2){  ///Q1,2
            double r2;
            r2=t2-PI;
            if(boata.theta>r2 && boata.theta<0){
            dtheta=t2-t1;
            }
            else if(boata.theta<r2){
                dtheta=-(2*PI-(t2-t1));
            }
            if(dtheta>PI){
                cout<<"Q1,2  " << t1 << "\t" << t2 << "\t" << dtheta<< endl;
            }
            if(dtheta<-PI){
                cout<<"Q1,2  " << t1 << "\t" << t2 << "\t" << dtheta<< endl;
            }
        }
        else if(boata.theta > PI/2){/// Q4
            dtheta=t1-t2;
            if(dtheta>PI){
            cout <<"Q4  "<< t1 << "\t" << t2 << "\t" << dtheta<< endl;
            }
        }
        else if(boata.theta < -PI/2){ /// Q3
            t2=atan2(boata.x,boata.y);
            double r1;
            r1=-PI+t2;
            if(boata.theta>r1){
                dtheta=t2+t1;
            }
            else if(boata.theta<r1){
                dtheta=(2*PI-t2+t1);
            }
            if(dtheta<-PI){
                cout <<"Q3  "<< t1 << "\t" << t2 << "\t" << dtheta<< endl;
            }
        }
    }
    else if(boata.y>500){
        if(boata.theta > -PI/2 && boata.theta<PI/2){///Q1&2
            dtheta=t2-t1;
        }
        if(dtheta>PI){
            cout<<"Q1,2  " << t1 << "\t" << t2 << "\t" << dtheta<< endl;
        }
        if(dtheta<-PI){
            cout<<"Q1,2  " << t1 << "\t" << t2 << "\t" << dtheta<< endl;
        }
        else if(boata.theta<-PI/2){ ///Q2&Q3
            //cout << "T2  " << t2 << endl;
            dtheta=(2*PI-t2+t1);
            
        }
        else if(boata.theta>PI/2){
            dtheta=t1-t2;
        }
    }
    if(dtheta<-PI){
        cout<<"t1 "<<t1<< "   t2 " << t2 << "\t" << dtheta << endl;
    }
    if(dtheta>PI){
        //cout<<"t1 "<<t1<< "   t2 " << t2 << "\t" << dtheta << endl;
    }
    return dtheta;
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
    win = false;
    if(x>950){
        win = true;
    }
}

void straightboat(){
    srand(unsigned(time(NULL)));
    agent boat;
    boat.init();
    
    // Established Values
    double v =3;
    double dt=.2;
    double T=5;
    int tsmax = 5000;;
    
    goal buoy;
    ofstream outputFile;
    outputFile.open("StraightLine.txt");
    
    for(int tstep=0; tstep<tsmax; tstep++){ ///SIMULATOR LOOP
        double px=boat.x;
        //cout << tstep << endl;
        boat.u=0;
        boat.simulate(v, dt, T);
        assert(px==boat.x);  // previous x == current x means boat is not turning
        
    }
    
    outputFile.close();
}


