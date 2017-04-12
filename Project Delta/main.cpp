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

#define WWRAND (double)rand()/RAND_MAX

using namespace std;

class agent{
public:
    double x;
    double y;
    double theta;
    double omega;
    double fitness;
    
    double u;
    double umax=15;
    double umin=-15;
    
    void move(double v, double dt, double T);
    void checku();
    void simulator();
    void eval();
};

class water{
public:
    double xmin;
    double xmax;
    
    double ymin;
    double ymax;
    
    
};

void simulator(double u);


int main() {                                /// MAIN
    cout << "Main Start" << endl;
    srand(unsigned(time(NULL)));
    double u = 5;
    simulator(u);
    
    
    
    
    
    return 0;
}


void agent::move(double v, double dt, double T){
    x=x+v*sin(theta)*dt;
    y=y+v*cos(theta)*dt;
    theta=theta+omega*dt;
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

void simulator(double u){
    agent boat;
    
    // ICs
    double ix = 10;
    double iy = 10;
    double itheta = 0;
    double iomega = 0;
    // Established Values
    double v =3;
    double dt=.2;
    double T=5;
    
    boat.x=ix;
    boat.y=iy;
    boat.theta=itheta;
    boat.omega=iomega;
    
    boat.u=u;
    
    for(int i=0; i<10000; i++){  /// Change to WHILE loops eventually or a break/end
        boat.checku();
        boat.move(v, dt, T);
        }
}
