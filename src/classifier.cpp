#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <numeric>
#include <vector>
#include "classifier.h"

using namespace std;
/**
 * Initializes GNB
 */
GNB::GNB() {

}

GNB::~GNB() {}

void GNB::train(vector<vector<double>> data, vector<string> labels)
{

	/*
		Trains the classifier with N data points and labels.

		INPUTS
		data - array of N observations
		  - Each observation is a tuple with 4 values: s, d, 
		    s_dot and d_dot.
		  - Example : [
			  	[3.5, 0.1, 5.9, -0.02],
			  	[8.0, -0.3, 3.0, 2.2],
			  	...
		  	]

		labels - array of N labels
		  - Each label is one of "left", "keep", or "right".
	*/
	vector<vector<vector<double>>> data_s;
	
	for(int i=0; i<3; i++){
	    vector<vector<double>> class_data;
	    data_s.push_back(class_data);
	    
	    for(int j=0; j<4; j++){
	        vector<double> obs;
	        data_s[i].push_back(obs);
	    }
	}
	
	
	for(int i = 0; i<data.size(); i++){
	    int class_ = left;
	    if(labels[i] == "right"){
	        class_ = right;
	    }else if(labels[i] == "keep"){
	        class_ = keep;
	    }
	    
	    data_s[class_][s].push_back(data[i][s]);
	    data_s[class_][d].push_back(fmod(data[i][d],4.0));
	    data_s[class_][s_dot].push_back(data[i][s_dot]);
	    data_s[class_][d_dot].push_back(data[i][d_dot]);
	}
	
	
	
	vector<vector<double>> means;
	vector<vector<double>> sigs;
	
	for(int i=0; i<3; i++){
	    vector<double> means_row;
	    vector<double> sigs_row;
	    for(int j=0; j<4; j++){
	        means_row.push_back(mean(data_s[i][j]));
	        sigs_row.push_back(sig(data_s[i][j]));
	    }
	    means.push_back(means_row);
	    sigs.push_back(sigs_row);
	}
	

	means_ = means;
	sigs_ = sigs;
}




string GNB::predict(vector<double> sample)
{
	/*
		Once trained, this method is called and expected to return 
		a predicted behavior for the given observation.

		INPUTS

		observation - a 4 tuple with s, d, s_dot, d_dot.
		  - Example: [3.5, 0.1, 8.5, -0.2]

		OUTPUT

		A label representing the best guess of the classifier. Can
		be one of "left", "keep" or "right".
		"""
		# TODO - complete this
	*/
    vector<double> probs;
    
    for(int i=0;i<3;i++){
        double prob = 1;
        
        for(int j=0;j<4;j++){
            if(j == 1){
                sample[j] = fmod(sample[j], 4.0);
            }
            prob*=gaussian(sample[j], means_[i][j], sigs_[i][j]);
        }
        
        probs.push_back(prob);
    }
    
    int best_index = 0;
    double best_prob = probs[best_index];
    
    for(int i=0;i<3;i++){
        if(probs[i]>best_prob){
            best_index = i;
            best_prob = probs[best_index];
        }
    }
    
	return this->possible_labels[best_index];

}

double GNB::gaussian(double x, double mu, double sig){
    double num = pow(x-mu, 2);
    double denum = 2*pow(sig, 2);
    double norm = 1/sqrt(2*M_PI*pow(sig,2));
    return norm * exp(-num/denum);
}

double GNB::mean(vector<double> list){
    double sum = std::accumulate(list.begin(), list.end(), 0);
    return sum/((float)list.size());
}

double GNB::sig(vector<double> list){
    double mean = GNB::mean(list);
    
    double sum_num = 0;
    for(int i=0; i<list.size(); i++){
        sum_num += pow(list[i] - mean,2);
    }
    
    double ms = sum_num / (list.size() - 1);
    
    return sqrt(ms);
}