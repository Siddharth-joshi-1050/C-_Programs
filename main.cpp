#include <iostream>
#include <ctime>
#include <math.h>
#include "robot_original.h"

using namespace std;

int main()
{
	std::clock_t start;
    double duration;
    start = std::clock();
    
    int n, i, choice;
    double c_x, c_y, c_r, e_x, e_y, e_t, link_lengths[3], input=0;

    cout << "============================================================================"<< endl;
    cout << "============================================================================"<< endl;
    cout << "CHOICE TABLE" << endl;
    cout << "============================================================================"<< endl;
    cout << " Case 1 : Forward Kinematics( 3 Links) " << endl;
    cout << " Case 2 : Forward Kinematics (variable links) " << endl;
    cout << " Case 3 : Intersection " << endl;
    cout << " Case 4 : Inverse Kinematics" << endl;
    cout << "============================================================================"<< endl;
    cout << "============================================================================"<< endl;

    cout << "Enter your choice value (1, 2, 3 or 4): " << endl;
    cin >> choice;

    if (choice == 1)
    {
         

            cout << "==================== Forward Kinematics (3 links) ==================="<<endl;


            float theta0[3]; 
            float length0[3];
            char demo;

            cout << "\n" << endl;
            cout << "Note : Enter Y for YES | N for NO" << endl;
            cout << "Would you like a test demonstration?" << endl;
            cin >> demo;


            if (demo == 'Y')
            {
                cout << "=======================================================" << endl;
                cout << "Following Demo is for input parameters:" << endl;
                cout << "=======================================================" << endl;
                cout << "\n" << endl;
                cout << "ANGLES : (Theta 0, Theta 1, Theta 2) = (30, 45, 30) degrees" << endl;
                cout << "LENGTHS : (Link 0, Link 1, Link 2) = (1, 2, 3) units" << endl;
                cout << endl;

                float theta0[3] = {30, 45, 30};
                float length0[3] = {1, 2, 3};
                Robot r0(theta0, length0, 3);
                r0.forward_kin();    
            }
            else if (demo == 'N')
            {
                cout<< "Enter the corresponding parameters for 3R planar robot :" <<endl;
                cout<< "Note : Enter angles in degrees." <<endl;
                cout << "\n" << endl;

                for (i = 0; i < 3; i++)
                {
                    cout << "Enter Theta for Link["<< i <<"]"<< endl;
                    cin >> theta0[i];
                    cout << "Enter Length of Link["<< i <<"]"<< endl;
                    cin >> length0[i];
                }

                Robot r0(theta0,length0,3);
                r0.forward_kin();
                cout << endl;
            }
            else
            {
                cout << "Invalid Choice! Kindly enter Y or N next time." << endl;
            }
            
    }


}