// 6AFS-Test.cpp
#define 	DEBUGSS	0
#include 	"ros/ros.h"
#include 	"geometry_msgs/WrenchStamped.h"

#include	<stdio.h>
#include	<fcntl.h>
#include	<time.h>
#include	<termios.h>
#include	<string.h>

int SetComAttr(int fdc);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "DynPickSensor");

	ros::NodeHandle node;

	ros::Publisher sensor_pub = node.advertise<geometry_msgs::WrenchStamped>("dynpick/wrench", 1000);

	ros::Rate loop_rate(10);

	int				status;
	FILE			*fd;
	int				fdc;
	char			fname[64];
	char			devname[64];
	char			str[256];
	char			str2[256];
	unsigned short	data[6];
	int				comNo;
	int				tick;
	int				clk, clkb, clkb2, clk0;
	int				tw;
	int				NUM;
	int				n,m;
	int 			count;
	float 			sum[6];

	float iniFx = 32.670;
	float iniFy = 32.540;
	float iniFz = 32.930;
	float iniTx = 1647.5;
	float iniTy = 1636.0;
	float iniTz = 1636.0;

	for (m=0;m<6;m++)
	{
		sum[m]=0;
	}

	fd = NULL;
	fdc = -1;

	sprintf(devname, "/dev/ttyUSB_dynpick");
	fdc = open(devname, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fdc < 0)
		ROS_ERROR("device port number is invalid");

	tw = 160;

	SetComAttr(fdc);

	clk0 = clock() / (CLOCKS_PER_SEC / 1000);
	clkb = 0;
	clkb2 = 0;
	NUM = 10;

	// 単データリクエスト（初回分）
	write(fdc, "R", 1);

	count = 0;
	
	while(count < NUM){
		while (true)
		{
		clk = clock() / (CLOCKS_PER_SEC / 1000) - clk0;

		if (clk >= clkb + tw)
			{
			clkb = clk / tw * tw;
			break;
			}
		}

		write(fdc, "R", 1);

		n = read(fdc, str, 27);
		sscanf(str, "%1d%4hx%4hx%4hx%4hx%4hx%4hx",
			&tick, &data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);

		
		for (m=0;m<6;m++)
		{
			sum[m]+=data[m]/float(NUM);
		}
		count++;
	}
	for(int i=0;i<6;i++){
		printf("%lf", sum[i]);
	}
	

	while (ros::ok())
	{
		
		while (true)
		{
		clk = clock() / (CLOCKS_PER_SEC / 1000) - clk0;

		if (clk >= clkb + tw)
			{
			clkb = clk / tw * tw;
			break;
			}
		}

		write(fdc, "R", 1);

		n = read(fdc, str, 27);
		
		sscanf(str, "%1d%4hx%4hx%4hx%4hx%4hx%4hx",
			&tick, &data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);

		
		geometry_msgs::WrenchStamped wrench;

		wrench.header.stamp = ros::Time();

		wrench.wrench.force.x = (data[0]-sum[0])/iniFx;
		wrench.wrench.force.y = (data[1]-sum[1])/iniFy;
		wrench.wrench.force.z = (data[2]-sum[2])/iniFz;
		wrench.wrench.torque.x = (data[3]-sum[3])/iniTx;
		wrench.wrench.torque.y = (data[4]-sum[4])/iniTy;
		wrench.wrench.torque.z = (data[5]-sum[5])/iniTz;
		

		sensor_pub.publish(wrench);

		ros::spinOnce();

		loop_rate.sleep();
	
	}


	return 0;
}




int SetComAttr(int fdc)
{
	int			n;

	struct termios	term;


	// ボーレートなどを設定
	n = tcgetattr(fdc, &term);
	if (n < 0)
		goto over;

	bzero(&term, sizeof(term));

	term.c_cflag = B921600 | CS8 | CLOCAL | CREAD;
	term.c_iflag = IGNPAR;
	term.c_oflag = 0;
	term.c_lflag = 0;/*ICANON;*/

	term.c_cc[VINTR]    = 0;     /* Ctrl-c */
	term.c_cc[VQUIT]    = 0;     /* Ctrl-? */
	term.c_cc[VERASE]   = 0;     /* del */
	term.c_cc[VKILL]    = 0;     /* @ */
	term.c_cc[VEOF]     = 4;     /* Ctrl-d */
	term.c_cc[VTIME]    = 0;
	term.c_cc[VMIN]     = 0;
	term.c_cc[VSWTC]    = 0;     /* '?0' */
	term.c_cc[VSTART]   = 0;     /* Ctrl-q */
	term.c_cc[VSTOP]    = 0;     /* Ctrl-s */
	term.c_cc[VSUSP]    = 0;     /* Ctrl-z */
	term.c_cc[VEOL]     = 0;     /* '?0' */
	term.c_cc[VREPRINT] = 0;     /* Ctrl-r */
	term.c_cc[VDISCARD] = 0;     /* Ctrl-u */
	term.c_cc[VWERASE]  = 0;     /* Ctrl-w */
	term.c_cc[VLNEXT]   = 0;     /* Ctrl-v */
	term.c_cc[VEOL2]    = 0;     /* '?0' */

//	tcflush(fdc, TCIFLUSH);
	n = tcsetattr(fdc, TCSANOW, &term);
over :

	return (n);
}

