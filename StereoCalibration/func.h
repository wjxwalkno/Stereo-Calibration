#ifndef _FUNC_H
#define _FUNC_H
#include "global.h"
#include <opencv2/opencv.hpp>

class CStereoCalibrate
{

public:
	CStereoCalibrate();
	~CStereoCalibrate();
   
	/**�궨����
	 * @param temp1:txt�ļ�����·��
	 */
	void Calibrate(char *temp1);

	//��ȡ�궨����
	void GetMatrix();

private:

	//������ʼ��
	void Initial();
	void ReleaseAll();

	//����궨��ľ��󣬲������.xml�ļ���
	void SaveMatrix(int n,const CvMat *intrinsic_matrix, const CvMat *distortion_coeffs, const CvMat *translation_vectors, const CvMat *rotation_vectors);

	//��ȡ�����ļ��е����̸�궨��ͼƬ
	void ChessimageRead(char tempFilename[],int num_file_name);

public:
	CvMat *left_intrinsic;
	CvMat *left_distortion;
	CvMat *left_translation_vector_1;
	CvMat *right_intrinsic;
	CvMat *right_distortion;
	CvMat *right_translation_vector_1;


	CvMat *left_rotation_matrix;

	CvMat *right_rotation_matrix;

private:
	CvMat* object_points2;
	CvMat* image_points2;
	CvMat* point_counts2;
	CvMat* intrinsic_matrix;
	CvMat* distortion_coeffs;
	CvMat* rotation_vectors;
	CvMat* translation_vectors ;
	CvMat* image_points;//channel is 1
	CvMat* object_points;
	CvMat* point_counts;

	CvSize board_sz;
	CvPoint2D32f* corners;//һ��ͼƬ�����۽ǵ���Ŀ board_n
	int corner_count;
	int successes;
	int step;
	int n_board; //��ͼƬ��
	int n_boards;              
	int num_of_file_name;
	int num_of_param_name;
	char tempFilename[256];
	FILE *fptr;
	char names[2048];
	IplImage* image;
	IplImage* gray_image;
};
	
#endif

//ʹ�����ӣ�
//int main()
//{
//	CStereoCalibrate* stereo=new CStereoCalibrate();
//	stereo->Calibrate("C:\\Users\\Administrator\\Desktop\\�к���Ŀ\\StereoCalibration\\StereoCalibration\\StereoCalibration\\StereoCalibration\\");
//
//	stereo->GetMatrix();
//
//
//
//}