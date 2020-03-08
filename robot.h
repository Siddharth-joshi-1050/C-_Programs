#include <iostream>
#include <ctime>
#include <math.h>

#define MAX_SIZE 100
using namespace std;



class Robot{
private: 
    int size; 
    float theta[MAX_SIZE];
    float length[MAX_SIZE];
    float end_pos_x=0;
    float end_pos_y=0;
    float end_pos_theta=0;

public: 
    Robot();
    Robot(float * a, float * b, int n);

    void forward_kin();
    void forward_kin_var_size();
    bool intersection(double circle_x, double circle_y, double circle_r);
    void inverse_kin(double end_pos_x, double end_pos_y, double end_pos_theta, double b0, double b1, double b2);

};
Robot::Robot() {}

Robot::Robot(float * a, float * b, int n)
{

}

void Robot::forward_kin()
{

}

void Robot::forward_kin_var_size()
{

}

bool Robot::intersection(double circle_x, double circle_y, double circle_r)
{

}

void Robot::inverse_kin(double end_pos_x, double end_pos_y, double end_pos_theta, double linklength0, double linklength1, double linklength2)
{

}

