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
    /*  The following segment is to manipulate forward kinematics 
            for a  3R planar robot arm with user-defined prameters. 
            The number of links and angles are defined for 3 links 
            and adding more links is not facilitated.

            The input parameters are link lengths and corresponding 
            angles. Output is the end-effector pose i.e, position 
            and orientation.
    */
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
    /*  The following segment is to manipulate forward kinematics 
            for a Planar robot arm with user-defined parameters. 
            The number of links and angles are user-defined and 
            adding more links is facilitated in this section.

            The input parameters are number of links, link lengths 
            and corresponding angles. Output is the end-effector 
            pose i.e, position and orientation.
    */

{
    this->end_pos_x = 0;
    this->end_pos_y = 0;
    int i=0,j=0;
    float theta_sum=0;
    for(i=0;i<this->size;i++)
    {
         theta_sum += this->theta[i];
         this->end_pos_x += this->length[i] * cos(theta_sum);
         this->end_pos_y += this->length[i] * sin(theta_sum);
    }
    this->end_pos_theta = theta_sum;
    cout << " \n";
    cout << "=======================================================" << endl;
    cout << "END-EFFECTOR CREDENTIALS:" << endl;
    cout << "=======================================================" << endl;
    cout << " \n" << endl;
    cout << "X: " << end_pos_x << " units" << endl; 
    cout << "Y: " << end_pos_y << " units" << endl;
    cout << "Theta: " << (end_pos_theta*180)/M_PI << " degrees" << endl;
}

bool Robot::intersection(double circle_x, double circle_y, double circle_r)
    /*  The following segment is to check if the end-effector pose lies
            within the user-defined circular boundaries. The end-effector pose is 
            calculated based on the user input of joint-space for a 3R planar robot. 

            The input parameters are link lengths, corresponding 
            angles, circle center (circle_x,circle_y) and circle radius (circle_r) 
            Output is the boolean check on end-effector pose, if it lies within
            defined geometry. 
    */
{
    if(pow((this->end_pos_x - circle_x),2)+pow((this->end_pos_y - circle_y),2) <= pow(circle_r, 2))
    {
        cout << "\n" << endl;
        cout << "=======================================================" << endl;
        cout << "OBSERVATION:" << endl;
        cout << "=======================================================" << endl;
        cout<<"End-effector is within circular bounds"<<endl;
        
        return true;
    }
    else
    {   
        cout << "\n" << endl;
        cout << "=======================================================" << endl;
        cout << "OBSERVATION:" << endl;
        cout << "=======================================================" << endl;
        cout<<"End-effector is outside circular bounds"<<endl;
        
        return false;
    }
}

void Robot::inverse_kin(double end_pos_x, double end_pos_y, double end_pos_theta, double linklength0, double linklength1, double linklength2)
    /*  The following segment is to manipulate a 3R planar robot arm for inverse
            kinematics. Given the end-effector pose as input parameters, joint 
            parameters are calculated. However, this calculation works only with
            3R planar robot. It will raise and error message and break, for input
            links greater than 3, and for links less than 3, the corresponding
            missing parameters will be asked for.

            The input parameters are end-effector pose (task space). Output is 
            the joint space parameters, which in this case will give feasible solutions
            for elbow-up and elbow-down position.
    */
{
    end_pos_theta *= M_PI / 180;
    this->end_pos_x = end_pos_x;
    this->end_pos_y = end_pos_y;
    this->end_pos_theta = end_pos_theta;

    double x, y, c2, s21, s22, theta21, theta22, s11, c11, s12, c12, theta11, theta12, theta31, theta32;

    x = end_pos_x - (linklength2*cos(end_pos_theta));
    y = end_pos_y - (linklength2*sin(end_pos_theta));
    c2 = (pow(x,2) + pow(y,2) - pow(linklength0,2) - pow(linklength1,2))/(2 * linklength0 * linklength1);
    s21 = pow(double(1 - pow(c2,2)), 0.5);
    s22 = s21 * -1;
    theta21 = atan(s21/c2);
    theta22 = atan(s22/c2);

    s11 = ((linklength0 + (linklength1*c2))*y - (linklength1*s21*x))/(pow(x,2) + pow(y,2));
    c11 = ((linklength0 + (linklength1*c2))*x - (linklength1*s21*y))/(pow(x,2) + pow(y,2));

    s12 = ((linklength0 + (linklength1*c2))*y - (linklength1*s22*x))/(pow(x,2) + pow(y,2));
    c12 = ((linklength0 + (linklength1*c2))*x - (linklength1*s22*y))/(pow(x,2) + pow(y,2));

    theta11 = atan(s11/c11);
    theta12 = atan(s12/c12);


    theta31 = end_pos_theta - theta21 - theta11;
    theta32 = end_pos_theta - theta22 - theta12;

    cout << "Trial Outputs: " << endl;
    cout << "X : "<< x << endl;
    cout << "Y : "<< y << endl;
    cout << "c2 : "<< c2 << endl;
    cout << "s21 : " << s21 << endl;
    cout << "s22 : " << s22 << endl; 
    cout << "theta 21 :" << (theta21*180)/M_PI << endl;
    cout << "theta 22 :" << (theta22*180)/M_PI << endl;
    cout << "theta 11 :" << (theta11*180)/M_PI << endl;
    cout << "theta 12 :" << (theta12*180)/M_PI << endl;
    cout << "theta 31 :" << (theta31*180)/M_PI << endl;
    cout << "theta 32 :" << (theta32*180)/M_PI << endl;


    cout << "=======================================================" << endl;
    cout<< "The first set of feasible solution is: "<< endl;
    cout << "=======================================================" << endl;

    cout << "Theta 1 : " << (theta11*180)/M_PI  << " degrees " << endl;
    cout << "Theta 2 : " << (theta21*180)/M_PI <<  " degrees " << endl;
    cout << "Theta 3 : " << (theta31*180)/M_PI <<  " degrees " << endl;

    cout << "=======================================================" << endl;
    cout<< "The second set of feasible solution is: "<< endl;
    cout << "=======================================================" << endl;
    
    cout << "Theta 1 : " << (theta12*180)/M_PI  << " degrees " << endl;
    cout << "Theta 2 : " << (theta22*180)/M_PI <<  " degrees " << endl;
    cout << "Theta 3 : " << (theta32*180)/M_PI <<  " degrees " << endl;
}

