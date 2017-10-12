//#ifndef _ARROW_DETECTION_H_ //que este codigo seja incluido duas vezes!
//#define _ARROW_DETECTION_H_

//OpenCV
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

//stabalized functions
using namespace cv;     

cv::Point calculate_center_contour(cv::Point2f contour[],unsigned int size_contour);

/// Morphological operation OPENING
void Opening(Mat & src, Mat & dest,unsigned int erosion_size);

/// Morphological operation Closing
void closing(Mat & src, Mat & dest,unsigned int erosion_size);

void boundig_box( Mat & src_tresh, Mat & threshold_output);

/// Find discriptors (Match points)
void descritor( Mat & img_object, Mat & img_scene);

//#endif