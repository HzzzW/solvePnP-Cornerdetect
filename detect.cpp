#include "Marker.h"

#pragma comment(lib,"libzbar-0.lib")
using namespace std;
using namespace cv;
using namespace Eigen;
using namespace zbar;

int Corner3d = 0;
vector<cv::Point3f> m_markerCorners3d;
vector<cv::Point2f> m_markerCorners2d;

Mat camMatrix;//����ڲ�
Mat distCoeff;//�������������辭������궨�õ���
			  

int main_detect(Mat &frame)
{
	if (Corner3d == 0)
	{
		m_markerCorners3d.push_back(cv::Point3f(+70.0f, +70.0f, 0));//�Զ�ά������Ϊ�������꣬��ά���СΪ200mm��x������Ϊ����y������Ϊ��
		m_markerCorners3d.push_back(cv::Point3f(-70.0f, +70.0f, 0));
		m_markerCorners3d.push_back(cv::Point3f(-70.0f, -70.0f, 0));
		m_markerCorners3d.push_back(cv::Point3f(+70.0f, -70.0f, 0));
		Corner3d = 1;
	}
	

	camMatrix = (Mat_<double>(3, 3) << 2872.24, 0, 1905.33, 0, 2863.94, 1102.70, 0, 0, 1); //�ڲξ���       
	distCoeff = (Mat_<double>(5, 1) << -0.41615, 0.34278, 0.00048529, 0.00056574, -0.31462);  //�������                  

	int x[4], y[4];			//��ά�������

	// ����Zbarɨ�����				
	ImageScanner scanner;  
	// ��ʼ��
	scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
	// ���ض�ά��ͼ������
	Mat srcImage = frame;
	if (!srcImage.data) {
		cout << "Input image error!" << endl;
		system("pause");
		return 0;
	}
	Mat src_gray, result;
	cvtColor(srcImage, src_gray, CV_BGR2GRAY);//��ûҶ�ͼ
	result = src_gray.clone();
	threshold(src_gray, result, 70, 120, CV_THRESH_BINARY); //���ж�ֵ������ѡ��30��200.0Ϊ��ֵ
	cvNamedWindow("�Ҷ�ͼ", CV_WINDOW_NORMAL);
	cv::imshow("�Ҷ�ͼ", result);
	int width = src_gray.cols;
	int height = src_gray.rows;
	// wrap the image
	uchar*raw = (uchar*)result.data;
	Image imageZbar(width, height, "Y800", raw, width*height);
	// ��ʼɨ��
	scanner.scan(imageZbar);
	// ��չ���
	Image::SymbolIterator symbol = imageZbar.symbol_begin();
	if (imageZbar.symbol_begin() == imageZbar.symbol_end()) {
		cout << "ɨ��ʧ�ܣ����ͼƬ����!" << endl;

	}
	else
	{
		for (; symbol != imageZbar.symbol_end(); ++symbol) {
			//cout << "���ͣ�" << endl << symbol->get_type_name() << endl;
			//cout << "���룺" << endl << symbol->get_data() << endl;

			x[0] = symbol->get_location_x(0);
			x[1] = symbol->get_location_x(3);
			x[2] = symbol->get_location_x(2);
			x[3] = symbol->get_location_x(1);
			y[0] = symbol->get_location_y(0);
			y[1] = symbol->get_location_y(3);
			y[2] = symbol->get_location_y(2);
			y[3] = symbol->get_location_y(1);

			//��������㣨�ǵ㣩����������ϵ�ϵ���������
			cout << "���Ͻǵ㣺" << "(" << symbol->get_location_x(0) << "," << symbol->get_location_y(0) << ")" << endl;
			cout << "���Ͻǵ㣺" << "(" << symbol->get_location_x(3) << "," << symbol->get_location_y(3) << ")" << endl;
			cout << "���½ǵ㣺" << "(" << symbol->get_location_x(1) << "," << symbol->get_location_y(1) << ")" << endl;
			cout << "���½ǵ㣺" << "(" << symbol->get_location_x(2) << "," << symbol->get_location_y(2) << ")" << endl;

			line(srcImage, Point(x[0], y[0]), Point(x[1], y[1]), Scalar(0, 0, 255), 8);
			line(srcImage, Point(x[1], y[1]), Point(x[2], y[2]), Scalar(0, 0, 255), 8);
			line(srcImage, Point(x[2], y[2]), Point(x[3], y[3]), Scalar(0, 0, 255), 8);
			line(srcImage, Point(x[3], y[3]), Point(x[0], y[0]), Scalar(0, 0, 255), 8);
		}
		cv::Mat Rvec;
		cv::Mat_<float> Tvec;
		cv::Mat raux, taux;
		
		vector<Point2f> points2d;
		for (int i = 0; i < 4; i++) {
			//cout << Point(x[i], y[i]) << endl;
			points2d.push_back(Point(x[i], y[i]));
		}

		cv::solvePnP(m_markerCorners3d, points2d, camMatrix, distCoeff, raux, taux, false, CV_P3P);  //PNP�㷨��λ��

		raux.convertTo(Rvec, CV_32F);    //��ת����
		taux.convertTo(Tvec, CV_32F);   //ƽ������

		cv::Mat_<float> rotMat(3, 3);
		cv::Rodrigues(Rvec, rotMat);
		

		//float theta_z = atan2(rotMat[1][0], rotMat[0][0])*57.2958;
		//float theta_y = atan2(-rotMat[2][0], sqrt(rotMat[2][0] * rotMat[2][0] + rotMat[2][2] * rotMat[2][2]))*57.2958;
		//float theta_x = atan2(rotMat[2][1], rotMat[2][2])*57.2958;

		//cout << "��x��ת�ǣ�" << theta_x << endl;
		//cout << "��y��ת�ǣ�" << theta_y << endl;
		//out << "��z��ת�ǣ�" << theta_z << endl;
		
		//void cv::cv2eigen(const Mat &rotMat, Eigen::Matrix< float, 1, Eigen::Dynamic > &R_n);


		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> R_n;
		Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> T_n;

		cv2eigen(rotMat, R_n);
		cv2eigen(Tvec, T_n);
		//cout << "R_n�Ż�ǰ����ֵ" << endl;
		//cout << R_n << endl;
		//cout << T_n << endl;

		Eigen::Vector3f P_oc;
		P_oc = -R_n.inverse()*T_n;
		/*cout << endl << "PnP��������������" << endl;
		cout << "camPositionX: " << ("%2f", P_oc[0]) << endl;
		cout << "camPositionY: " << ("%2f", P_oc[1]) << endl;
		cout << "camPositionZ: " << ("%2f", P_oc[2]) << endl;*/

		/********************g2oʵ�����λ�˹���*********************/
		//    ��ʼ��g2o
		typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> > Block;
		Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>(); //���Է��������
		Block* solver_ptr = new Block(std::unique_ptr<Block::LinearSolverType>(linearSolver));       // ���������
		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<Block>(solver_ptr));
		//g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg(std::unique_ptr<Block>(solver_ptr));  DogLeg�Ż�����
		//g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(std::unique_ptr<Block>(solver_ptr)); GN�Ż�����
		g2o::SparseOptimizer optimizer;
		optimizer.setAlgorithm(solver);


	   //    ����ͼ�Ż�����----> pose 
		g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
		Eigen::Matrix3d  R_mat;
		/*R_mat <<
			R_n(0, 0), R_n(0, 1), R_n(0, 2),
			R_n(1, 0), R_n(1, 1), R_n(1, 2),
			R_n(2, 0), R_n(2, 1), R_n(2, 2);*/
		R_mat << 1, 0, 0, 0, 1, 0, 0, 0, 1;
		pose->setId(0);
		pose->setEstimate(g2o::SE3Quat(R_mat,Eigen::Vector3d(T_n(0, 0), T_n(1, 0), T_n(2, 0))));
		optimizer.addVertex(pose);


		//    ����ͼ�Ż�����----> points
		int index = 1;
		for (const Point3f p : m_markerCorners3d)
		{
			g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
			point->setId(index++);
			point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
			point->setMarginalized(true);
			//point->setInformation(Eigen::Matrix3d::Identity());
			optimizer.addVertex(point);
		}


		//   ��������ڲ�
		g2o::CameraParameters* camera = new g2o::CameraParameters(
			camMatrix.at<double>(0, 0), Eigen::Vector2d(camMatrix.at<double>(0, 2), camMatrix.at<double>(1, 2)), 0);
		camera->setId(0);
		optimizer.addParameter(camera);


		//   ����ͼ�Ż���
		index = 1;
		for (const Point2f p : points2d)
		{
			g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
			edge->setId(index);
			edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(index)));
			edge->setVertex(1, pose);
			edge->setMeasurement(Eigen::Vector2d(p.x, p.y));  //���ù۲�ֵ
			edge->setParameterId(0, 0);
			edge->setInformation(Eigen::Matrix2d::Identity());
			optimizer.addEdge(edge);
			index++;
		}


		//   �����Ż���������ʼִ���Ż�
		optimizer.setVerbose(false);
		optimizer.initializeOptimization();
		optimizer.optimize(100);


		//    ����Ż����
		cout << endl << "after optimization:" << endl;
		//cout << "T=" << endl << Eigen::Isometry3d(pose->estimate()).matrix() << endl;  //�任����
		Eigen::Matrix4d  T_4d = Eigen::Isometry3d(pose->estimate()).matrix();
		Eigen::Matrix3d  R_mat_update;       //�Ż�����ת����
		R_mat_update <<
			T_4d(0, 0), T_4d(0, 1), T_4d(0, 2),
			T_4d(1, 0), T_4d(1, 1), T_4d(1, 2),
			T_4d(2, 0), T_4d(2, 1), T_4d(2, 2);
		//cout << R_mat_update << endl;
		
		Eigen::Vector3d T_n_update;         // �Ż���ƽ������
		T_n_update << T_4d(0, 3), T_4d(1, 3), T_4d(2, 3);
		//cout << T_n_update << endl;
		
		Eigen::Vector3d P_oc_update;        // �Ż�����������
		P_oc_update = -R_mat_update.inverse()*T_n_update;
		cout << "�Ż��������������" << endl;
		cout << "camPositionX: " << ("%2f", P_oc_update[0]) << endl;
		cout << "camPositionY: " << ("%2f", P_oc_update[1]) << endl;
		cout << "camPositionZ: " << ("%2f", P_oc_update[2]) << endl;
		cout << "--------------------------------" << endl;
	}

	cv::imshow("IPCamera", srcImage);
	if (waitKey(1) == 27) {
		system("pause");
	}

	return 0;
}


