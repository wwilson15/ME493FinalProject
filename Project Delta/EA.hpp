//
//  main.cpp
//  Projct Gamma
//
//  Created by Wade Wilson on 3/15/17.
//  Copyright © 2017 Wade Wilson. All rights reserved.
//


#ifndef EA_hpp
#define EA_hpp



#include <iostream>
#include <vector>
#include <random>
#include <assert.h>
#include <cmath>
#include <fstream>
#include <math.h>

#define WWRAND (double)rand()/RAND_MAX

using namespace std;

class policy{
public:
    double fitness;
    void mutate();
    void init(int nw);
    void eval();
    void downselect();
    void repop();
    
    vector<double> weights;
};


void policy::init(int num_weights){
    vector<double> b;
    for(int i=0;i<num_weights;i++){
        b.push_back(WWRAND-WWRAND);
    }
    weights=b;
}

void policy::mutate(){
    for(int i=0; i<weights.size(); i++){
        if(rand()%2==0){
            weights.at(i)+=.025*WWRAND - .025*WWRAND;
        }
    }
    
}

void policy::eval(){
    
}

vector<policy> EA_init(int num_policy, int num_weights){ // vector of policies
    vector<policy> population;
    
    for(int i=0; i<num_policy; i++){
        policy A;
        A.init(num_weights);
        population.push_back(A);
    }
    
    return population;
}

vector<policy> EA_replicate(vector<policy> P, int popsize){
    vector<policy> population;
    population = P;
    //cout << population.size() << "\t" << popsize << endl;
    assert(population.size() == popsize/2);
    while(population.size()<popsize){
        int spot = rand()%population.size();
        policy A;
        A = population.at(spot);
        A.mutate();
        population.push_back(A);
    }
    
    assert(population.size() == popsize);
    
    return population;
}


vector<policy> EA_evaluate(vector<policy> P, int pop_size){
    vector<policy> population;
    population = P;
    for(int initf=0; initf<population.size(); initf++){
        population.at(initf).fitness = -1;
    }
    
    for(int i=0; i<population.size(); i++){
        population.at(i).eval();
        assert(population.at(i).fitness != -1);
    }
    
    for(int testing=0; testing<population.size(); testing++){
        assert(population.at(testing).fitness != -1);
    }
    assert(population.size() == pop_size);
    return population;
}

vector<policy> EA_downselect(vector<policy> P, int pop_size){ // binary tournament
    vector<policy> population;
    assert(population.size()==0);
    assert(P.size()==pop_size);
    while(population.size() < pop_size / 2){
        int s1 = rand()%P.size();
        int s2 = rand()%P.size();
        while(s2 == s1){
            s2 = rand()%P.size();
        }
        assert(s1!=s2);
        double fit1 = P.at(s1).fitness;
        double fit2 = P.at(s2).fitness;
        
        if(fit1>fit2){
            policy A = P.at(s2);
            population.push_back(A);
        }
        else if(fit1<=fit2){
            policy B = P.at(s1);
            population.push_back(B);
        }
    }
    
    assert(population.size() == pop_size/2);
    return population;
}

#endif /* EA_hpp */
