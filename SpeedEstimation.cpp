#include"Helper.h"

#define SAVE_VIDEO 0
#define STORE_RESULT 0
#define LOAD_VIDEO 0
#define SHOW_MOUSE_COORDINATES 0


int main()
{
#if LOAD_VIDEO
	VideoCapture input_cap("Videos/output4.avi");
#else
	VideoCapture input_cap;
	if (!input_cap.open(1))	
	{
		cout << "No camera found!" << endl; return 0;
	}
	input_cap.set(CAP_PROP_FRAME_WIDTH, 640);		
	input_cap.set(CAP_PROP_FRAME_HEIGHT, 480);
#endif
	
#if SAVE_VIDEO 
	VideoWriter output_cap("Videos/output5.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, cv::Size(input_cap.get(CAP_PROP_FRAME_WIDTH), input_cap.get(CAP_PROP_FRAME_HEIGHT)));
#endif
#if STORE_RESULT
	VideoWriter output_cap("Videos/result.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, cv::Size(2*input_cap.get(CAP_PROP_FRAME_WIDTH), input_cap.get(CAP_PROP_FRAME_HEIGHT)),false);
#endif

	Mat greyOld, frame, mean;
	input_cap >> frame;
	cvtColor(frame, greyOld, COLOR_BGR2GRAY);
	uint i = -1;

	while(true)
	{		
		++i;
		Mat grey, diff;
		input_cap >> frame;
		
		if (frame.empty())
		{
			break;
		}
#if SAVE_VIDEO
		output_cap<<frame;
#endif
		cvtColor(frame, grey, COLOR_BGR2GRAY);

		absdiff(grey, greyOld, diff);
		greyOld = grey.clone();

		Mat thr;
		threshold(diff, thr, 30, 255, 0);
		if (i == 0)
			mean = thr;

		int radius = 5;
		calc_means(grey, radius);


#if SHOW_MOUSE_COORDINATES
		namedWindow("Frame");
		setMouseCallback("Frame", mouse_callback);
#endif
		int mean_change_threshold = 20;
		int status = get_status(mean_change_threshold);

		inwards_state_machine(status);
		outwards_state_machine(status);

		draw_reagions(grey, radius, status);

		print_stats(thr);
		
		Mat dst;
		hconcat(grey, thr, dst);

		imshow("Frame", dst);
#if STORE_RESULT
		output_cap << dst;
#endif
		if (waitKey(10) == 27)
			break;
	}

	return 0;

}

