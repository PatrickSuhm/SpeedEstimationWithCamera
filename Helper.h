#pragma once

#include<opencv2/opencv.hpp>
#include<iostream>
#include <chrono>

using namespace std;
using namespace cv;
using namespace std::chrono;


Point p11(379, 282);	
Point p12(296, 304);	

Point p21(177, 339);		
Point p22(94, 361);		

Point p31(178, 397);	
Point p32(239, 374);	

Point p41(359, 327);	
Point p42(435, 307);	

int mean_p11, mean_p12, mean_p21, mean_p22, mean_p31, mean_p32, mean_p41, mean_p42 = 0;
int mean_p11_old, mean_p12_old, mean_p21_old, mean_p22_old, mean_p31_old, mean_p32_old, mean_p41_old, mean_p42_old;

enum InwardsState
{
	INIT_INWARDS,
	P11,
	P12,
	P21,
	P22
};
int numCarsInwards = 0;
InwardsState inwards_state = INIT_INWARDS;
uint64_t tick_start_inward;
double vel_in = 0;

enum OutwardsState
{
	INIT_OUTWARDS,
	P31,
	P32,
	P41,
	P42
};
int numCarsOutwards = 0;
OutwardsState outwards_state = INIT_OUTWARDS;
uint64_t tick_start_outward;
double vel_out = 0;


uint64_t timeSinceEpochMillisec() {
	return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

void mouse_callback(int  event, int  x, int  y, int  flag, void *param)
{
	if (event == EVENT_MOUSEMOVE) {
		cout << "(" << x << ", " << y << ")" << endl;
	}
}

void draw_reagions(const Mat & grey, int pointRadius, int status)
{
	int color[8] = { 0 };
	if (status & 0b01)
		color[0] = 255;
	if (status & 0b10)
		color[1] = 255;
	if (status & 0b100)
		color[2] = 255;
	if (status & 0b1000)
		color[3] = 255;
	if (status & 0b10000)
		color[4] = 255;
	if (status & 0b100000)
		color[5] = 255;
	if (status & 0b1000000)
		color[6] = 255;
	if (status & 0b10000000)
		color[7] = 255;	

	circle(grey, p11, pointRadius, Scalar(color[0]), 2);
	circle(grey, p12, pointRadius, Scalar(color[1]), 2);

	circle(grey, p21, pointRadius, Scalar(color[2]), 2);
	circle(grey, p22, pointRadius, Scalar(color[3]), 2);

	circle(grey, p31, pointRadius, Scalar(color[4]), 2);
	circle(grey, p32, pointRadius, Scalar(color[5]), 2);

	circle(grey, p41, pointRadius, Scalar(color[6]), 2);
	circle(grey, p42, pointRadius, Scalar(color[7]), 2);
}

int my_mean(const Mat & grey, const Point point, const int distance)
{
	if (point.x - distance < 0 || point.y - distance < 0 ||
		point.x + distance >= grey.cols || point.y + distance >= grey.rows)
		assert(false);

	int mean = 0;
	
	for(auto y = point.y - distance; y<=point.y + distance; y++)
		for (auto x = point.x - distance; x <= point.x + distance; x++)
		{
			mean += grey.at<uchar>(y, x);		
		}

	double test = pow(2*distance + 1, 2);
	return mean / test;
}

void calc_means(const Mat & grey, const int radius)
{
	mean_p11_old = mean_p11; mean_p11 = my_mean(grey, p11, radius);
	mean_p12_old = mean_p12; mean_p12 = my_mean(grey, p12, radius);

	mean_p21_old = mean_p21; mean_p21 = my_mean(grey, p21, radius);
	mean_p22_old = mean_p22; mean_p22 = my_mean(grey, p22, radius);

	mean_p31_old = mean_p31; mean_p31 = my_mean(grey, p31, radius);
	mean_p32_old = mean_p32; mean_p32 = my_mean(grey, p32, radius);

	mean_p41_old = mean_p41; mean_p41 = my_mean(grey, p41, radius);
	mean_p42_old = mean_p42; mean_p42 = my_mean(grey, p42, radius);
}

int get_status(int threshold)
{
	int status = 0;
	
	if (abs(mean_p11_old - mean_p11) > threshold)
		status = status | 0b1;

	if (abs(mean_p12_old - mean_p12) > threshold)
		status = status | 0b10;

	if (abs(mean_p21_old - mean_p21) > threshold)
		status = status | 0b100;

	if (abs(mean_p22_old - mean_p22) > threshold)
		status = status | 0b1000;

	if (abs(mean_p31_old - mean_p31) > threshold)
		status = status | 0b10000;

	if (abs(mean_p32_old - mean_p32) > threshold)
		status = status | 0b100000;

	if (abs(mean_p41_old - mean_p41) > threshold)
		status = status | 0b1000000;

	if (abs(mean_p42_old - mean_p42) > threshold)
		status = status | 0b10000000;

	return status;
}

void inwards_state_machine(int status)
{

	switch (inwards_state)
	{
	case INIT_INWARDS:
	{
		if (status & 0b1)
		{
			inwards_state = P11;
		}
		break;
	}
	case P11:
	{
		if (status & 0b10)
		{
			inwards_state = P12;
			
			tick_start_inward = timeSinceEpochMillisec();
		}
		break;
	}
	case P12:
	{
		if (status & 0b100)
		{
			inwards_state = P21;
		}
		break;
	}
	case P21:
	{
		if (status & 0b1000)
		{
			inwards_state = P22;

			double time_in = 1e-3*(double)(timeSinceEpochMillisec() - tick_start_inward);
			double dist_in = 12.0;
			vel_in = 3.6*(dist_in / time_in);
		}
		break;
	}
	case P22:
	{
		if (!(status & 0b1000))
		{
			numCarsInwards += 1;
			inwards_state = INIT_INWARDS;
		}
		break;
	}

	}
}

void outwards_state_machine(int status)
{
	switch (outwards_state)
	{
	case INIT_OUTWARDS:
	{
		if (status & 0b10000)
		{
			outwards_state = P31;
		}
		break;
	}
	case P31:
	{
		if (status & 0b100000)
		{
			outwards_state = P32;
			
			tick_start_outward = timeSinceEpochMillisec();
		}
		break;
	}
	case P32:
	{
		if (status & 0b1000000)
		{
			outwards_state = P41;
		}
		break;
	}
	case P41:
	{
		if (status & 0b10000000)
		{
			outwards_state = P42;
			
			double time_out = 1e-3*(double)(timeSinceEpochMillisec() - tick_start_outward)	;
			double dist_out = 16.0;
			vel_out = 3.6*(dist_out / time_out);
		}
		break;
	}
	case P42:
	{
		if (!(status & 0b10000000))
		{
			numCarsOutwards += 1;
			outwards_state = INIT_OUTWARDS;
		}
		break;
	}

	}
}

void print_stats(const Mat & grey)
{
	char color = 255;
	rectangle(grey, Rect(25, 20, 330, 90), Scalar(255-color),FILLED, 8);
	
	putText(grey, "Cars moving north: " + to_string(numCarsInwards), Point(30, 40),
		FONT_HERSHEY_SIMPLEX, 0.6, Scalar(color), 2);
	putText(grey, "Velocity north: " + to_string(vel_in) + " km/h", Point(30, 60),
		FONT_HERSHEY_SIMPLEX, 0.6, Scalar(color), 2);

	putText(grey, "Cars moving south: " + to_string(numCarsOutwards), Point(30, 80),
		FONT_HERSHEY_SIMPLEX, 0.6, Scalar(color), 2);
	putText(grey, "Velocity south: " + to_string(vel_out) + " km/h", Point(30, 100),
		FONT_HERSHEY_SIMPLEX, 0.6, Scalar(color), 2);
}