#include "global.h"  //所有输入参数
#include <opencv2/opencv.hpp>
#include <iostream>
#include "func.h"
using namespace std;

CStereoCalibrate::CStereoCalibrate()
{
	Initial();
}
CStereoCalibrate::~CStereoCalibrate()
{

}

//参数初始化
void CStereoCalibrate::Initial()
{
	left_intrinsic = cvCreateMat(3,3,CV_64FC1);
	left_distortion = cvCreateMat(5,1,CV_64FC1);
	left_translation_vector_1 = cvCreateMat(3,1,CV_64FC1);
	right_intrinsic = cvCreateMat(3,3,CV_64FC1);
	right_distortion = cvCreateMat(5,1,CV_64FC1);
	right_translation_vector_1 = cvCreateMat(3,1,CV_64FC1);


	left_rotation_matrix = cvCreateMat(3,3,CV_64FC1);

	right_rotation_matrix = cvCreateMat(3,3,CV_64FC1);

	object_points2 = cvCreateMat(successes_ideal*board_n,3,CV_32FC1);
	image_points2  = cvCreateMat(successes_ideal*board_n,2,CV_32FC1);
	point_counts2  = cvCreateMat(successes_ideal,1,CV_32SC1);
	intrinsic_matrix = cvCreateMat(3,3,CV_32FC1);
	distortion_coeffs = cvCreateMat(5,1,CV_32FC1);
	rotation_vectors = cvCreateMat(successes_ideal,3,CV_64FC1);
	translation_vectors = cvCreateMat(successes_ideal,3,CV_64FC1);

	board_sz = cvSize( board_w, board_h );
	corners = new CvPoint2D32f[ board_n ];//一张图片的理论角点数目 board_n
	successes = 0;
	n_boards=0;              
	num_of_file_name = 0;
	num_of_param_name = 1;
	image = 0;
	gray_image = 0;
	fptr=NULL;
}
//标定过程
void CStereoCalibrate::Calibrate(char *temp1)
{
	/*******************************************************两相机分别标定***********************************************/
	for(int i_bigloop=0;i_bigloop<camera_num;i_bigloop++)
	{		
		ChessimageRead(temp1,num_of_file_name);

		image_points   = cvCreateMat(n_boards*board_n,2,CV_32FC1);//channel is 1
		object_points  = cvCreateMat(n_boards*board_n,3,CV_32FC1);
		point_counts   = cvCreateMat(n_boards,1,CV_32SC1);

		for( int frame=0; frame<n_boards; frame++ ) 
		{
			fscanf(fptr,"%s ",names);
			if(image)
			{
				cvReleaseImage(&image);
				image = 0;
			}
			image = cvLoadImage( names);
			if(gray_image == 0  && image) 
				gray_image = cvCreateImage(cvGetSize(image),8,1);
			if(!image)
				printf("null image\n");

			int found = cvFindChessboardCorners(image,board_sz,corners,&corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);//Get Subpixel accuracy on those corners
			cvCvtColor(image, gray_image, CV_BGR2GRAY);//RGB Image转换成了灰度图
			cvFindCornerSubPix(gray_image, corners, corner_count, 
				cvSize(11,11),cvSize(-1,-1), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

			if( corner_count == board_n ) 
			{
				step = successes*board_n;
				for( int i=step, j=0; j<board_n; ++i,++j ) 
				{
					CV_MAT_ELEM(*image_points, float,i,0) = corners[j].x;
					CV_MAT_ELEM(*image_points, float,i,1) = corners[j].y;
					CV_MAT_ELEM(*object_points,float,i,0) = j/board_w*corner_width;//(0,0)(0,1)(0,2)......
					CV_MAT_ELEM(*object_points,float,i,1) = j%board_w*corner_width;
					CV_MAT_ELEM(*object_points,float,i,2) = 0.0f;
				}
				CV_MAT_ELEM(*point_counts, int,successes,0) = board_n;		
				successes++;//increase step 
			}
		}	
		//TRANSFER THE POINTS INTO THE CORRECT SIZE MATRICES
		for(int i = 0; i<successes*board_n; ++i)
		{
			CV_MAT_ELEM(*image_points2, float,i,0) 	=	CV_MAT_ELEM(*image_points, float,i,0);
			CV_MAT_ELEM(*image_points2, float,i,1) 	= 	CV_MAT_ELEM(*image_points, float,i,1);
			CV_MAT_ELEM(*object_points2,float,i,0) = CV_MAT_ELEM(*object_points,float,i,0) ;
			CV_MAT_ELEM(*object_points2,float,i,1) = CV_MAT_ELEM(*object_points,float,i,1) ;
			CV_MAT_ELEM(*object_points2,float,i,2) = CV_MAT_ELEM(*object_points,float,i,2) ;

		} 
		for(int i=0; i<successes; ++i)
		{
			CV_MAT_ELEM(*point_counts2,int,i, 0) = CV_MAT_ELEM(*point_counts, int,i,0);
		}
		// Initialize the intrinsic matrix such that the two focal
		// lengths have a ratio of 1.0
		//
		CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0f;
		CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0f;
		
		double err_calib = cvCalibrateCamera2(object_points2,image_points2,point_counts2,cvGetSize( image ),intrinsic_matrix,distortion_coeffs,rotation_vectors,translation_vectors,0);//CV_CALIB_FIX_ASPECT_RATIO
		printf("平均像素误差:%f\n",err_calib);

		SaveMatrix(num_of_param_name,intrinsic_matrix,distortion_coeffs,translation_vectors,rotation_vectors);

		num_of_file_name++;
		num_of_param_name++;
		n_boards=0;
		successes = 0;

		ReleaseAll();
	}
	/*********************************************************标定结束*************************************************************/	
}

void CStereoCalibrate::ReleaseAll()
{
	cvReleaseMat(&intrinsic_matrix);
	cvReleaseMat(&distortion_coeffs);
	cvReleaseMat(&translation_vectors);
	cvReleaseMat(&rotation_vectors);
	cvReleaseMat(&object_points);
	cvReleaseMat(&image_points);
	cvReleaseMat(&point_counts);
}

//保存标定后的矩阵，并存放在.xml文件中
void CStereoCalibrate::SaveMatrix(int n,const CvMat *intrinsic_matrix, const CvMat *distortion_coeffs, const CvMat *translation_vectors, const CvMat *rotation_vectors)
{
	CvMat *intrinsic = 0;
	CvMat *distortion = 0;
	CvMat *translation_vector = 0;
	CvMat *rotation_vector = 0;	

	char tempFilename0[256];
	char tempFilename1[256];
	char tempFilename2[256];
	char tempFilename3[256];

	sprintf(tempFilename0, "Intrinsics%d.xml",n);
	sprintf(tempFilename1, "Distortion%d.xml",n);
	sprintf(tempFilename2, "translation_vectors%d.xml",n);
	sprintf(tempFilename3, "rotation_vectors%d.xml",n);

	cvSave(tempFilename0,intrinsic_matrix);
	cvSave(tempFilename1,distortion_coeffs);
	cvSave(tempFilename2,translation_vectors);
	cvSave(tempFilename3,rotation_vectors);

	intrinsic = (CvMat*)cvLoad(tempFilename0);
	distortion = (CvMat*)cvLoad(tempFilename1);
	translation_vector = (CvMat*)cvLoad(tempFilename2);
	rotation_vector = (CvMat*)cvLoad(tempFilename3);
}

//读取本地文件中的棋盘格标定板图片
void CStereoCalibrate::ChessimageRead(char *temp1,int num_file_name)
{
	//sprintf(tempFilename, "chessboards%d.txt",num_of_file_name);     //输入图片文件在chessboards0.txt和chessboards1.txt中
	char temp0[256]="chessboards";
	char temp2[10]=".txt";
	sprintf(tempFilename,"%s%s%d%s",temp1,temp0,num_of_file_name,temp2);
	fptr = fopen(tempFilename,"r");
		//COUNT THE NUMBER OF IMAGES:
	while(fscanf(fptr,"%s ",names)==1)
	{
			n_boards++;
	}
	rewind(fptr);
}
//获取标定参数
void CStereoCalibrate::GetMatrix()
{
	/*********************************************************标定参数输出*************************************************************/
	left_intrinsic = (CvMat*)cvLoad("Intrinsics1.xml");
	left_distortion = (CvMat*)cvLoad("Distortion1.xml");
	CvMat *left_translation_vector = (CvMat*)cvLoad("translation_vectors1.xml");
	CvMat *left_rotation_vector = (CvMat*)cvLoad("rotation_vectors1.xml");

	right_intrinsic = (CvMat*)cvLoad("Intrinsics2.xml");
	right_distortion = (CvMat*)cvLoad("Distortion2.xml");
	CvMat *right_translation_vector = (CvMat*)cvLoad("translation_vectors2.xml");
	CvMat *right_rotation_vector = (CvMat*)cvLoad("rotation_vectors2.xml");


	left_rotation_matrix  = cvCreateMat(3,3,CV_64FC1);
	left_translation_vector_1  = cvCreateMat(3,1,CV_64FC1);
	CvMat* left_rotation_vector_1  = cvCreateMat(1,3,CV_64FC1);

	right_rotation_matrix   = cvCreateMat(3,3,CV_64FC1);
	right_translation_vector_1  = cvCreateMat(3,1,CV_64FC1);
	CvMat* right_rotation_vector_1  = cvCreateMat(1,3,CV_64FC1);

	cvGetRow(left_translation_vector,left_translation_vector_1,0);//key 从0开始
	cvGetRow(left_rotation_vector,left_rotation_vector_1,0);
	cvRodrigues2(left_rotation_vector_1,left_rotation_matrix);

	cvGetRow(right_translation_vector,right_translation_vector_1,0);
	cvGetRow(right_rotation_vector,right_rotation_vector_1,0);
	cvRodrigues2(right_rotation_vector_1,right_rotation_matrix);
	//以下这些参数作为预校准的输入，内参，旋转矩阵，平移矩阵
	/*cout<<"左相机内参:"<<endl;
	cout<<left_intrinsic->data.fl[0]<<"		";cout<<left_intrinsic->data.fl[1]<<"		";cout<<left_intrinsic->data.fl[2]<<endl;
	cout<<left_intrinsic->data.fl[3]<<"		";cout<<left_intrinsic->data.fl[4]<<"		";cout<<left_intrinsic->data.fl[5]<<endl;
	cout<<left_intrinsic->data.fl[6]<<"		";cout<<left_intrinsic->data.fl[7]<<"		";cout<<left_intrinsic->data.fl[8]<<endl;
	cout<<"右相机内参:"<<endl;
	cout<<right_intrinsic->data.fl[0]<<"		";cout<<right_intrinsic->data.fl[1]<<"		";cout<<right_intrinsic->data.fl[2]<<endl;
	cout<<right_intrinsic->data.fl[3]<<"		";cout<<right_intrinsic->data.fl[4]<<"		";cout<<right_intrinsic->data.fl[5]<<endl;
	cout<<right_intrinsic->data.fl[6]<<"		";cout<<right_intrinsic->data.fl[7]<<"		";cout<<right_intrinsic->data.fl[8]<<endl;
	cout<<"左相机旋转矩阵："<<endl;
	cout<<left_rotation_matrix->data.db[0]<<"		";cout<<left_rotation_matrix->data.db[1]<<"		";cout<<left_rotation_matrix->data.db[2]<<endl;
	cout<<left_rotation_matrix->data.db[3]<<"		";cout<<left_rotation_matrix->data.db[4]<<"		";cout<<left_rotation_matrix->data.db[5]<<endl;
	cout<<left_rotation_matrix->data.db[6]<<"		";cout<<left_rotation_matrix->data.db[7]<<"		";cout<<left_rotation_matrix->data.db[8]<<endl;
	cout<<"右相机旋转矩阵："<<endl;
	cout<<right_rotation_matrix->data.db[0]<<"		";cout<<right_rotation_matrix->data.db[1]<<"		";cout<<right_rotation_matrix->data.db[2]<<endl;
	cout<<right_rotation_matrix->data.db[3]<<"		";cout<<right_rotation_matrix->data.db[4]<<"		";cout<<right_rotation_matrix->data.db[5]<<endl;
	cout<<right_rotation_matrix->data.db[6]<<"		";cout<<right_rotation_matrix->data.db[7]<<"		";cout<<right_rotation_matrix->data.db[8]<<endl;
	cout<<"左相机平移矩阵:"<<endl;
	cout<<left_translation_vector_1->data.db[0]<<"		";cout<<left_translation_vector_1->data.db[1]<<"		";cout<<left_translation_vector_1->data.db[2]<<endl;
	cout<<"右相机平移矩阵:"<<endl;
	cout<<right_translation_vector_1->data.db[0]<<"		";cout<<right_translation_vector_1->data.db[1]<<"		";cout<<right_translation_vector_1->data.db[2]<<endl;*/
	/****************************************************************************************************************************************************/

}