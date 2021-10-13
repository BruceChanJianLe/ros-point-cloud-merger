#ifndef AXIS_MANAGER
#define AXIS_MANAGER

#include <iostream>

class AxisValue
{
private:
    int x;
    int y;
    int z;
    std::string input_topics;

public:
    AxisValue();

    AxisValue(int a, int b, int c, std::string inputs){ 
        setX(a);
        setY(b);
        setZ(c);
        setInputTopics(inputs);
    }

    int getX(){
        return x;
    }
    void setX(int input){
        x = input;
    }

    int getY(){
        return y;
    }
    void setY(int input){
        y = input;
    }

    int getZ(){
        return z;
    }
    void setZ(int input){
        z = input;
    }

    std::string getInputTopics(){
        return input_topics;
    }
    void setInputTopics(std::string input){
        input_topics = input;
    }
};

#endif