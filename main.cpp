/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
+   NOTE : main.cpp and and robot.h should be in the same directory.                             +
+   Execution of the main.cpp can be executed through following steps:                           + 
+                                                                                                +
+   1. Download and extract the zip file in a suitable directory.                                +
+   2. Open terminal and open directory where the zip file is extracted.                         +
+   3. Use Command :  g++ main.cpp                                                               +
+   4. Given the choices, select appropriate option to proceed with your manipulations.          +
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/




#include <iostream>
#include <ctime>
#include <math.h>
#include "robot.h"

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
    else if (choice == 2)
    {

    
            cout << "=============== Forward Kinematics (variable links) ==================="<<endl;
            
            char demo;

            cout << "\n" << endl;
            cout << "Note : Enter Y for YES | N for NO" << endl;
            cout << "Would you like a test demonstration?" << endl;
            cin >> demo;


            if (demo == 'Y')
            {
                cout << "\n" << endl;
                cout << "=======================================================" << endl;
                cout << "Following Demo is for input parameters:" << endl;
                cout << "=======================================================" << endl;
                cout << "\n" << endl;
                cout << "NUMBER OF LINKS = 4" << endl;
                cout << "ANGLES : (Theta 0, Theta 1, Theta 2, Theta 3) = (30, 45, 30, 15) degrees" << endl;
                cout << "LENGTHS : (Link 0, Link 1, Link 2, Link 3) = (1, 2, 3, 4) units" << endl;
                cout << "\n" << endl;


                int n = 4;
                float theta1[n] = {30, 45, 30, 15};
                float length1[n] = {1, 2, 3, 4};

                Robot r1(theta1,length1,n);

                r1.forward_kin_var_size();
                cout << endl;

            }
            
            else if (demo == 'N')
            {
                cout << "\n" << endl;
                cout << "Enter number of links: ";
                cin >> n;

                float theta1[n];
                float length1[n];
                
                for (i = 0; i < n; i++)
                {
                    cout << "Enter Theta for Link ["<< i <<"]"<< endl;
                    cin >> theta1[i];
                    cout << "Enter Length for Link ["<< i <<"]"<< endl;
                    cin >> length1[i];
                    cout << "\n" << endl;
                }

                Robot r1(theta1,length1,n);

                r1.forward_kin_var_size();
                cout << endl;
            }
            else 
            {
                cout << "Invalid Choice! Kindly enter Y or N next time." << endl;
            }
    }
    else if (choice == 3)
    {
          

            cout << "========================== Intersection =============================="<<endl;
            
            char demo;

            cout << "\n" << endl;
            cout << "Note : Enter Y for YES | N for NO" << endl;
            cout << "Would you like a test demonstration?" << endl;
            cin >> demo;


            if (demo == 'Y')
            {

                cout << "\n" << endl;
                cout << "=======================================================" << endl;
                cout << "Following Demo is for input parameters:" << endl;
                cout << "=======================================================" << endl;
                cout << "\n" << endl;
                cout << "ANGLES : (Theta 0, Theta 1, Theta 2) = (30, 45, 30) degrees" << endl;
                cout << "LENGTHS : (Link 0, Link 1, Link 2) = (1, 2, 3) units" << endl;
                cout << "CIRCLE : Center = (1,4) | Radius = 2" << endl;
                cout << "\n" << endl;

                float theta0[3] = {30, 45, 30};
                float length0[3] = {1, 2, 3};
                Robot r0(theta0, length0, 3);
                r0.forward_kin();

                r0.intersection(1, 4, 2);
                cout << endl;

            }
            else if (demo == 'N')
            {
                cout << "\n" << endl;
                cout<< "Enter the corresponding parameters for 3R planar robot :" <<endl;
                cout<< "Note : Enter angles in degrees." <<endl;

                float theta0[3]; 
                float length0[3];
                for (i = 0; i < 3; i++)
                {
                    cout << "Enter Theta for Link["<< i <<"]" << endl;
                    cin >> theta0[i];
                    cout << "Enter Length of Link["<< i <<"]" << endl;
                    cin >> length0[i];
                }

                Robot r0(theta0,length0,3);
                r0.forward_kin();
                cout << endl;


                cout << "\n" << endl;
                cout << "\nEnter details of the circle: "<<endl;
                cout << "Enter x coordinate of centre:"<<endl;
                cin >> c_x;
                cout << "Enter y coordinate of centre:"<<endl;
                cin >> c_y;
                cout << "Enter the radius:"<<endl;
                cin >> c_r;

                r0.intersection(c_x, c_y, c_r);
                cout << endl;
            }
            else
            {
                cout << "Invalid Choice! Kindly enter Y or N next time." << endl;  
            }
            
    }
    else if (choice == 4)
    {

        
            cout << "========================= Inverse Kinematics ==============================="<<endl;

            char demo;

            cout << "\n" << endl;
            cout << "Note : Enter Y for YES | N for NO" << endl;
            cout << "Would you like a test demonstration?" << endl;
            cin >> demo;


            if (demo == 'Y')
            {
                cout << "\n" << endl;
                cout << "==========================================================================================" << endl;
                cout << "Following Demo is for input parameters analogous to output of Forward Kinematics Demo:" << endl;
                cout << "==========================================================================================" << endl;
                cout << "\n" << endl;
                cout << "End-effector orientation : 105 degrees" << endl;
                cout << "LENGTHS : (Link 0, Link 1, Link 2) = (1, 2, 3) units" << endl;
                cout << "Eng-effector position = (0.607, 5.32963)" << endl;
                cout << "\n" << endl;
                
                Robot r2 = Robot();
                r2.inverse_kin(0.607206, 5.32963, 105, 1, 2, 3);

            }
            else if (demo == 'N')
            {

                Robot r2 = Robot();
                i=0;
                cout << "NOTE : Enter angles in degrees" << endl;

                cout<<"\nEnter the x coordinate of the end effector:" << endl;
                cin>>e_x;
                cout<<"Enter the y coordinate of the end effector:" << endl;
                cin>>e_y;
                cout<<"Enter the orientation of the end effector:" << endl;
                cin>>e_t;
                while(input >= 0)
                {
                    cout<<"Enter link-length["<< i <<"] or enter -1 to exit:"<<endl;
                    cin >> input;
                    if(input == -1)
                    {
                        if(i<3)
                        {
                            cout<<"You have to provide exactly 3 links!"<<endl;
                            input=0;
                            continue;
                        }
                        else
                            break;
                    }
                    if(i==3)
                    {
                        cout<<"The calculation is allowed for only 3 links!!"<<endl;
                        break;
                    }
                    if(input != -1)
                    {
                            link_lengths[i++] = input;
                    }
                }
                r2.inverse_kin(e_x, e_y, e_t, link_lengths[0], link_lengths[1], link_lengths[2]);

                duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
                
                std::cout << "duration milliseconds initialize beliefs " << 1000 * duration << '\n';
            }
            else
            {
                cout << "Invalid Choice! Kindly enter Y or N next time." << endl;  
            }
    }
    else 
    {

            cout << "Kindly choose from one of the choices above. Thank you!" << endl;
    }

}