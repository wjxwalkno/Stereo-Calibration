#include <iostream>
#include "func.h"
#include "global.h"

using namespace std;

int main()
{
	CStereoCalibrate* stereo=new CStereoCalibrate();
	stereo->Calibrate("C:\\Users\\Administrator\\Desktop\\�к���Ŀ\\StereoCalibration\\StereoCalibration\\StereoCalibration\\StereoCalibration\\");

	stereo->GetMatrix();


	
}