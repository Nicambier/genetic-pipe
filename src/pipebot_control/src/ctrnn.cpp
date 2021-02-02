#include "ctrnn.h"

using namespace std;

CTRNN::CTRNN(unsigned int inputSize, unsigned int hiddenSize, unsigned int outputSize): inputSize(inputSize), outputSize(outputSize), neurons(vector<double>(inputSize+hiddenSize+outputSize,0)), oldNeurons(neurons), bias(vector<double>(inputSize+hiddenSize+outputSize,0))
{
    
}

double CTRNN::activation(unsigned int i)
{
    double sum = 0;
    for(unsigned int j=0; j<neurons.size(); ++j) {
        sum += weights[weight(j,i)] * oldNeurons[j];
    }
    return 0.1*(-oldNeurons[i] + sigmoid(bias[i] + sum));
}

vector<double> CTRNN::run(vector<double> input, double deltaT)
{
    if(input.size() != inputSize) {
        throw invalid_argument( "Input size does not correspond to input layer size" );
    }
    oldNeurons = neurons;
    vector<double> output;
    for(unsigned int i=0; i<neurons.size(); ++i) {
        if(i<inputSize)
            neurons[i] = input[i];
        neurons[i] += deltaT*activation(i);
        if(i>= neurons.size() - outputSize) {
            output.push_back(neurons[i]);
        }
    }
    return output;
}

std::string CTRNN::print() {
    stringstream s;
    s<<endl;
    for(unsigned int i=0; i<neurons.size(); ++i) {
        for(unsigned int j=0; j<neurons.size(); ++j) {
            s << weights[weight(i,j)] << " ";
        }
        s << endl;
    }
    return s.str();
}
