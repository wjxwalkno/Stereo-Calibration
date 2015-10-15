#include <iostream>
#include "func.h"
#include "global.h"

using namespace std;

int main()
{
	CStereoCalibrate* stereo=new CStereoCalibrate();
	stereo->Calibrate("C:\\Users\\Administrator\\Desktop\\ÖÐº½ÏîÄ¿\\StereoCalibration\\StereoCalibration\\StereoCalibration\\StereoCalibration\\");

	stereo->GetMatrix();


	
}