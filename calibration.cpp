#include<iostream>
#include<string>
using namespace std;
//F:\argc\Debug\argc.exe
#include <opencv2/opencv.hpp>
using std::vector;
using std::cout;
using std::cerr;
using std::endl;
//void help(char **argv) {  // todo rewrite this
//	cout << "\n\n"
//		<< "Example 18-1:\nReading a chessboard’s width and height,\n"
//		<< "              reading and collecting the requested number of views,\n"
//		<< "              and calibrating the camera\n\n"
//		<< "Call:\n" << argv[0] << " <board_width> <board_height> <number_of_boards> <if_video,_delay_between_framee_capture> <image_scaling_factor>\n\n"
//		<< "Example:\n" << argv[0] << " 9 6 15 500 0.5\n"
//		<< "-- to use the checkerboard9x6.png provided\n\n"
//		<< " * First it reads in checker boards and calibrates itself\n"
//		<< " * Then it saves and reloads the calibration matricies\n"
//		<< " * Then it creates an undistortion map and finally\n"
//		<< " * It displays an undistorted image\n"
//		<< endl;
//}
int main()
{
	string argv[5];
	int n_boards = 0;           // will be set by input list
	float image_sf = 0.5f;      // image scaling factor
	float delay = 1.f;
	int board_w = 0;
	int board_h = 0;

	cin >> argv[0] >> argv[1] >> argv[2] >> argv[3] >> argv[4];

	
	board_w = atoi(argv[0].c_str());
	board_h = atoi(argv[1].c_str());
	n_boards = atoi(argv[2].c_str());
	delay = atof(argv[3].c_str());
	image_sf = atof(argv[4].c_str());
	

	int board_n = board_w * board_h;
	cv::Size board_sz = cv::Size(board_w, board_h);
	cv::VideoCapture capture(0);
	if (!capture.isOpened()) {
		cout << "\nCouldn't open the camera\n";
		//help(argv);
		return -1;
	}

	// ALLOCATE STORAGE  allcate storage 存储区分配，分配存储面
	//
	vector<vector<cv::Point2f> > image_points;
	vector<vector<cv::Point3f> > object_points;

	// Capture corner views: loop until we've got n_boards successful
	// captures (all corners on the board are found).   //循环直到我们成功找到所有的角点
	//
	double last_captured_timestamp = 0;
	cv::Size image_size;
	while (image_points.size() < (size_t)n_boards) {
		cv::Mat image0, image;
		capture >> image0;
		image_size = image0.size();
		cv::resize(image0, image, cv::Size(), image_sf, image_sf, cv::INTER_LINEAR);

		// Find the board
		//
		vector<cv::Point2f> corners;
		bool found = cv::findChessboardCorners(image, board_sz, corners);

		if (found)
		{
			cout <<"found it"<<endl;
		}
		else
		{
			cout << "do not find it" << endl;
		}
		// Draw it
		//
		drawChessboardCorners(image, board_sz, corners, found);

		// If we got a good board, add it to our data
		//
		double timestamp = static_cast<double>(clock()) / CLOCKS_PER_SEC;
		if (found && timestamp - last_captured_timestamp > 1) {
			last_captured_timestamp = timestamp;
			image ^= cv::Scalar::all(255);
			cv::Mat mcorners(corners);

			// do not copy the data
			mcorners *= (1.0 / image_sf);

			// scale the corner coordinates
			image_points.push_back(corners);
			object_points.push_back(vector<cv::Point3f>());
			vector<cv::Point3f> &opts = object_points.back();

			opts.resize(board_n);
			for (int j = 0; j < board_n; j++) {
				opts[j] = cv::Point3f(static_cast<float>(j / board_w),
					static_cast<float>(j % board_w), 0.0f);
			}
			cout << "Collected our " << static_cast<uint>(image_points.size())
				<< " of " << n_boards << " needed chessboard images\n" << endl;
		}
		cv::imshow("Calibration", image);

		// show in color if we did collect the image
		if ((cv::waitKey(30) & 255) == 27)
			return -1;
	}

	// END COLLECTION WHILE LOOP.
	cv::destroyWindow("Calibration");
	cout << "\n\n*** CALIBRATING THE CAMERA...\n" << endl;

	// CALIBRATE THE CAMERA!
	//
	cv::Mat intrinsic_matrix, distortion_coeffs;
	double err = cv::calibrateCamera(
		object_points, image_points, image_size, intrinsic_matrix,
		distortion_coeffs, cv::noArray(), cv::noArray(),
		cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT);

	// SAVE THE INTRINSICS AND DISTORTIONS
	cout << " *** DONE!\n\nReprojection error is " << err
		<< "\nStoring Intrinsics.xml and Distortions.xml files\n\n";
	cv::FileStorage fs("intrinsics.xml", cv::FileStorage::WRITE);
	fs << "image_width" << image_size.width << "image_height" << image_size.height
		<< "camera_matrix" << intrinsic_matrix << "distortion_coefficients"
		<< distortion_coeffs;
	fs.release();

	// EXAMPLE OF LOADING THESE MATRICES BACK IN:
	fs.open("intrinsics.xml", cv::FileStorage::READ);
	cout << "\nimage width: " << static_cast<int>(fs["image_width"]);
	cout << "\nimage height: " << static_cast<int>(fs["image_height"]);
	cv::Mat intrinsic_matrix_loaded, distortion_coeffs_loaded;
	fs["camera_matrix"] >> intrinsic_matrix_loaded;
	fs["distortion_coefficients"] >> distortion_coeffs_loaded;
	cout << "\nintrinsic matrix:" << intrinsic_matrix_loaded;
	cout << "\ndistortion coefficients: " << distortion_coeffs_loaded << endl;

	// Build the undistort map which we will use for all
	// subsequent frames.
	//
	cv::Mat map1, map2;
	cv::initUndistortRectifyMap(intrinsic_matrix_loaded, distortion_coeffs_loaded,
		cv::Mat(), intrinsic_matrix_loaded, image_size,
		CV_16SC2, map1, map2);

	// Just run the camera to the screen, now showing the raw and
	// the undistorted image.
	//
	for (;;) {
		cv::Mat image, image0;
		capture >> image0;

		if (image0.empty()) {
			break;
		}
		cv::remap(image0, image, map1, map2, cv::INTER_LINEAR,
			cv::BORDER_CONSTANT, cv::Scalar());
		cv::imshow("Undistorted", image);
		cv::imshow("distorted",image0);
		if ((cv::waitKey(30) & 255) == 27) {
			break;
		}
	}

	return 0;
}