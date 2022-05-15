#include<iostream>
#include <opencv2\opencv.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\objdetect\objdetect.hpp>
#include <opencv2\imgproc\types_c.h>
#include<cmath>
#include <string>

using namespace cv;
using namespace std;



int main()
{
	Mat frame, R, T;
	Mat cameraMatrix = (Mat_<double>(3, 3) << 1805.397235035347, 0, 291.141526697627,0, 1808.59665092213, 264.2937066534833,0,0,1);
	Mat distCoeffs = (Mat_<double>(1,5) << -1.346354630004874,256.2252136874545,0.008484028111528536,-0.008200297927134349,-10553.52288659948);

	
	VideoCapture cap(1);

	vector<Mat> channels;
	Mat frame_R;
	for (;;)
	{
		//帧读取
		cap >> frame;
		
		Mat edges;
		Mat edgesBlur;
		Mat srcBinary;
		vector<vector<Point>>contours;

		//识别物数据(装甲板）
		double half_width, half_height;
		half_width = 40;
		half_height = 8;
		Mat objpoints;
		objpoints.push_back(Point3f(-half_width, -half_height, 0));
		objpoints.push_back(Point3f(+half_width, -half_height, 0));
		objpoints.push_back(Point3f(+half_width, +half_height, 0));
		objpoints.push_back(Point3f(-half_width, +half_height, 0));//方向相反（与Box.point)

		//图像处理
		cvtColor(frame, edges, COLOR_BGR2GRAY);
		GaussianBlur(edges, edgesBlur, Size(7, 7), 1.5, 1.5);
		threshold(edgesBlur, srcBinary, 190, 255, THRESH_BINARY);
		imshow("22", srcBinary);

		//图像处理第二种方式
		


		//Mat hsvimg;
		//cvtColor(frame, hsvimg, COLOR_BGR2HSV);

		//


		//Mat thresholdimg;
		//inRange(hsvimg, Scalar(60,120, 45), Scalar(120, 255, 255), thresholdimg);//输出图像已经被二值化
		//imshow("22", thresholdimg);
		//

		//开始识别
		findContours(srcBinary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		vector<Point2f> box_points(4);

		for (int j = 0; j < contours.size(); j++)
		{
			RotatedRect Box = minAreaRect(contours[j]);
			float h = Box.size.height;
			float w = Box.size.width;
			float ratio = h < w ? h / w : w / h;
			if (ratio < 0.3 &&ratio>0.1)
				

			//获得角点
			
			Box.points(box_points.data());

			//绘制识别框,识别不出来
			
			for (int j = 0; j < 4; j++)
			{
				line(frame, box_points[j], box_points[(j + 1) % 4], Scalar(255, 255, 255), 3);
			}
			

			//测距
			Mat Rec, Tec;
			solvePnP(objpoints, box_points, cameraMatrix, distCoeffs, Rec, Tec);
			Mat rotM;
			Rodrigues(Rec, rotM);
			Mat P;
			P = (rotM.t()) * Tec;
			stringstream sstr;
			sstr << "Distance:" << P.at<double>(2, 0);//问题3：注意这里的P；只有一列
			putText(frame, sstr.str(), Point(Box.center), FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0,0,0), 2);

			//求角度
			Point original(320,250);
			Point x1( 10000,250);
			Point x2(-10000,250);
			Point y1(320, 10000);
			Point y2(320, -10000);


			line(frame, x1, x2, Scalar(0, 0, 255), 2);
			//line(frame, y1, y2, Scalar(255, 0, 255), 3);
			Point target = Box.center;
			line(frame, original,target, Scalar(255,255,255), 3);

			Vec4d lx(320,250, 10000,250), ltar(320,250, target.x,target.y);

			double x11 = lx[0], y11 = lx[1], x22 = lx[2], y22 = lx[3];
			double a1 = -(y22 - y11), b1 = x22 - x11, c1 = (y22 - y11) * x11 - (x22 - x11) * y11; // 一般式：a1x+b1y1+c1=0
			double x33 = ltar[0], y33 = ltar[1], x44 = ltar[2], y44 = ltar[3];
			double a2 = -(y44 - y33), b2 = x44 - x33, c2 = (y44 - y33) * x33 - (x44 - x33) * y33; // 一般式：a2x+b2y1+c2=0
			
			double angle = 0;
			double a = sqrt(pow(x44 - x22, 2) + pow(y44 - y22, 2));
			double b = sqrt(pow(x44 - original.x, 2) + pow(y44 - original.y, 2));
			double c = sqrt(pow(x22 - original.x, 2) + pow(y22 - original.y, 2));
			angle = acos((b * b + c * c - a * a) / (2 * b * c)) * 180 / CV_PI;
			
			
			stringstream sstr1;
			sstr1 << "A:" << angle;

			putText(frame, sstr1.str(), Point(Box.center.x, Box.center.y - 15), FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(0,0,0), 2);

			
			

		}
		//展示图像
		imshow("Af", frame);
		if (waitKey(30) == 27)
			break;
		else
			continue;
	}

}
	
