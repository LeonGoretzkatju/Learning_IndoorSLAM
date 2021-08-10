#include "PlaneExtractor.h"

using namespace std;
using namespace cv;
using namespace Eigen;

PlaneDetection::PlaneDetection() {}

PlaneDetection::~PlaneDetection()
{
	cloud.vertices.clear();
	seg_img_.release();
	color_img_.release();
}

bool PlaneDetection::readColorImage(cv::Mat RGBImg)
{
	color_img_ = RGBImg;
	if (color_img_.empty() || color_img_.depth() != CV_8U)
	{
		cout << "ERROR: cannot read color image. No such a file, or the image format is not 8UC3" << endl;
		return false;
	}
	return true;
}

bool PlaneDetection::readDepthImage(const cv::Mat depthImg, const cv::Mat &K, const float &depthMapFactor)
{
	cv::Mat depth_img = depthImg;
	if (depth_img.empty() || depth_img.depth() != CV_16U)
	{
		cout << "WARNING: cannot read depth image. No such a file, or the image format is not 16UC1" << endl;
		return false;
	}

    double width = ceil(depthImg.cols/2.0);
    double height = ceil(depthImg.rows/2.0);
    cloud.vertices.resize(height * width);
	cloud.verticesColour.resize(height * width);
    cloud.w = width;
    cloud.h = height;

    seg_img_ = cv::Mat(height, width, CV_8UC3);

	int rows = depth_img.rows, cols = depth_img.cols;
	int vertex_idx = 0;
	for (int i = 0; i < rows; i+=2)
	{
		for (int j = 0; j < cols; j+=2)
		{
			double z = (double)(depth_img.at<unsigned short>(i, j)) * depthMapFactor;
			if (_isnan(z))
			{
				cloud.vertices[vertex_idx++] = VertexType(0, 0, z);
				continue;
			}
			double x = ((double)j - K.at<float>(0, 2)) * z / K.at<float>(0, 0);
			double y = ((double)i - K.at<float>(1, 2)) * z / K.at<float>(1, 1);
            cloud.verticesColour[vertex_idx] = color_img_.at<cv::Vec3b>(i,j);
			cloud.vertices[vertex_idx++] = VertexType(x, y, z);
		}
	}
	return true;
}
int number = 0;
cv::Mat PlaneDetection::runPlaneDetection()
{
	plane_filter.run(&cloud, &plane_vertices_, &seg_img_);
	plane_num_ = (int)plane_vertices_.size();
    cv::Mat mask_img=cv::Mat(240, 320, CV_8UC1);
    for (int row = 0; row < 240; ++row) {
        for (int col = 0; col < 320; ++col){
            int numP=plane_filter.membershipImg.at<int>(row, col);
            if(numP>=0)
            {
                mask_img.at<uchar>(row,col)=255;
            }
            else if (plane_filter.membershipImg.at<int>(row, col) < 0){
                mask_img.at<uchar >(row,col)=0;
                plane_filter.membershipImg.at<int>(row, col) = plane_num_;
            }
        }
    }
//    seg_img_.at<cv::Vec3b>()
    static cv::Mat kernel_erode = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    erode(mask_img, mask_img, kernel_erode);

    static cv::Mat kernel_dilate = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    dilate(mask_img, mask_img, kernel_dilate);

    static cv::Mat kernel_dilate2 = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    dilate(mask_img, mask_img, kernel_dilate2);

    static cv::Mat kernel_erode2 = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 5));
    erode(mask_img, mask_img, kernel_erode2);
//    cv::GaussianBlur(mask_img, mask_img, cv::Size(3,3), 0.5, 0.5);
    imwrite("/home/nuc/NYU2/mask/"+to_string(number)+".png", mask_img);
    imwrite("/home/nuc/NYU2/seg_img/"+ to_string(number)+".png", seg_img_);
    number ++ ;
    cout << "number is " << "    " << number << endl;
    return mask_img;
}
