#pragma once

#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <sstream>  
#include <boost/functional/hash.hpp>
#include <unordered_map>
#include <utility>

typedef std::pair<unsigned int, unsigned int> weight;

class CTRNN
{
public:
  CTRNN(unsigned int inputSize=1, unsigned int hiddenSize=0, unsigned int outputSize=1);
  
  void setWeight(unsigned int i, unsigned int j, double w) {weights[weight(i,j)] = w;}
  void setBias(unsigned int i, double b) {bias[i] = b;}
  std::vector<double> run(std::vector<double> input, double deltaT);
  
  std::string print();

private:
  double sigmoid(double z) {return 1/(1+exp(-z));}
  double activation(unsigned int);
    
  unsigned int inputSize;
  unsigned int outputSize;
  
  std::vector<double> neurons;
  std::vector<double> oldNeurons;
  std::vector<double> bias;
  std::unordered_map< weight, double, boost::hash<weight> > weights;
};
