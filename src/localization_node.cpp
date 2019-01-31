#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <math.h>

#include "ros/ros.h"
#include <indoor_localization/AnchorScan.h>


typedef struct  {
	
	int TXcm;
	int TYcm;
	int TZcm;
	
} pos;

// Position calculation of tag using given anchor coordinates and TDOA values
pos calc_pos_2D3A_ite(double SX,double SY,double SZ,double AX,double AY,double AZ,double BX,double BY,double BZ,double TZ,double dist_diff_A_S,double dist_diff_B_S)
{
    pos tagpos;

	double rS,rA,rB;

	int iter_num;
    	int loop_run;

	double M11,M12;
	double M21,M22;

	double N1,N2;

	double TX,TY;
	
	double preTX,preTY;
	
	TX = (SX+AX+BX)/3;
	TY = (SY+AY+BY)/3;

	iter_num = 0;

	loop_run = 1;

	while(loop_run == 1)
	{
		iter_num = iter_num + 1;
		
		rS = sqrt( pow((SX-TX),2) + pow((SY-TY),2) + pow((SZ-TZ),2) );
        	rA=rS+dist_diff_A_S;
		rB=rS+dist_diff_B_S;

		M11 = 2.0*(SX-AX);
		M12 = 2.0*(SY-AY);

		M21 = 2.0*(SX-BX);
		M22 = 2.0*(SY-BY);
		
		N1 = pow(rA,2)-pow(rS,2)-2*(TZ-SZ)*(SZ-AZ)-pow((SX-AX),2)-pow((SY-AY),2)-pow((SZ-AZ),2)+2*SX*(SX-AX)+2*SY*(SY-AY);
		N2 = pow(rB,2)-pow(rS,2)-2*(TZ-SZ)*(SZ-BZ)-pow((SX-BX),2)-pow((SY-BY),2)-pow((SZ-BZ),2)+2*SX*(SX-BX)+2*SY*(SY-BY);

		preTY = (N2-(M21*N1/M11)) / (M22-(M21*M12/M11));
		preTX = ( (-1*M21*N1/M11) + (M21*M12/M11)*preTY ) / (-1*M21);

		if ((iter_num > 20) || ( sqrt(pow((preTX-TX),2) + pow((preTY-TY),2)) < 0.05))
		{
			loop_run = 0;
		}

		TX = preTX;
		TY = preTY;

	}
	  
	tagpos.TXcm = (int)(TX*100);
	tagpos.TYcm = (int)(TY*100);
    	tagpos.TZcm = (int)(TZ*100);

	return tagpos;
}

int main(int argc, char *argv[])
{

    // coordinates of anchors
    double AX = 7.79; 
    double AY = 0.96; 
    double AZ = 1.64;

    double BX = 0.92; 
    double BY = 0.16; 
    double BZ = 1.21;

    double SX = 8.3; 
    double SY = 4.085; 
    double SZ = 1.945;

    double TZ = 1.72;

    // TDOA values
    double dist_diff_A_S = -1.053;
    double dist_diff_B_S = -4.998;
    

    ros::init(argc, argv, "localization_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<indoor_localization::AnchorScan>("IPS", 1000);

    // Loop rate is 1Hz
    ros::Rate loop_rate(1);

    indoor_localization::AnchorScan msg;

    pos tagpos;

    tagpos = calc_pos_2D3A_ite(SX,SY,SZ,AX,AY,AZ,BX,BY,BZ,TZ,dist_diff_A_S,dist_diff_B_S);
	
    while (ros::ok()) 
    {
        msg.X = tagpos.TXcm;
        msg.Y = tagpos.TYcm;
        msg.Y = tagpos.TZcm;

        ROS_INFO("\nTX = %f \nTY = %f \nTZ = %f \n\n", msg.X, msg.Y, msg.Z);

        pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }  

    return 0;
}
