#ifndef CLASSIFIER_H
#define CLASSIFIER_H
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>

using namespace std;

class GNB {
public:

	vector<string> possible_labels = {"left","keep","right"};
	
	vector<vector<double>> means_;
	
	vector<vector<double>> sigs_;

    int left = 0;
    int keep = 1;
    int right = 2;
    
    int s = 0;
    int d = 1;
    int s_dot = 2;
    int d_dot = 3;

	/**
  	* Constructor
  	*/
 	GNB();

	/**
 	* Destructor
 	*/
 	virtual ~GNB();

 	void train(vector<vector<double> > data, vector<string>  labels);

  	string predict(vector<double>);
  	
  	double gaussian(double x, double mu, double sig);
  	
  	double mean(vector<double> list);
  	
  	double sig(vector<double> list);

};

#endif



