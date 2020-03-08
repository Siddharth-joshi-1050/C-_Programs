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
    this->size = n;
    for (int i = 0; i < n; i++)
    {
        this->theta[i] = (a[i] * M_PI) / 180;
        cout<< "Theta "<<i<<":"<< this->theta[i] << " radians" << endl;
        this->length[i] = b[i];
        cout<< "Length of Link " <<i<<":"<< this->length[i] << " units" << endl;
    }    
}

void Robot::forward_kin()
{
    this->end_pos_x=(this->length[0]*cos(this->theta[0]))+
                        (this->length[1]*cos(this->theta[0]+this->theta[1]))+
                            (this->length[2]*cos(this->theta[0]+this->theta[1]+this->theta[2]));

    this->end_pos_y=(this->length[0]*sin(this->theta[0]))+
                        (this->length[1]*sin(this->theta[0]+this->theta[1]))+
                            (this->length[2]*sin(this->theta[0]+this->theta[1]+this->theta[2]));

    this->end_pos_theta=((this->theta[1]+this->theta[2]+this->theta[0])*180)/M_PI;


    cout << "\n" << endl;
    cout << "=======================================================" << endl;
    cout << "END-EFFECTOR CREDENTIALS:" << endl;
    cout << "=======================================================" << endl;
    cout << "\n" << endl;
    cout << "X: " << end_pos_x << " units" << endl; 
    cout << "Y: " << end_pos_y << " units" << endl;
    cout << "Theta: " << end_pos_theta << " degrees" << endl;
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

