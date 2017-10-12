
#include <arrow_detection/stabelized_func_arrow_detection.h>
//stabalized functions

cv::Point calculate_center_contour(cv::Point2f contour[],unsigned int size_contour)
{
    //falta lançar um erro a dizer que se o contorno for menor que dois pontos
/** @brief Calculates the center of an contour
 *  @param Point2f contour
 *  @param contour size
 *  @return Center Point 
 *  @pre Contour must be greater than 1 point;
 * \n 
 */
    unsigned int cont=0;
    double xx=0,yy=0;
    cv::Point result;
    for(cont=0;cont<size_contour;cont++)
    {
	    xx+=contour[cont].x;
	    yy+=contour[cont].y;
    }
    result.x=xx/cont+1;
    result.y=yy/cont+1;
    
    return result;
}

/// Morphological operation OPENING
void Opening(Mat & src, Mat & dest,unsigned int erosion_size)
{
   /**
   * Morphological operation OPENING
   * Imput: 
   *	1-Src Matix
   * 	2-Oupt Matrix
   * 	3-number of erosions
   * Output:
   * 	-Matrix "Opening"
   */
  Mat element = getStructuringElement( MORPH_RECT,Size( 2*erosion_size + 1, 2*erosion_size+1 ),
						       cv::Point( erosion_size, erosion_size ) );
	
		  dilate( src, src, element ); 
		  erode( src, src, element );
}

/// Morphological operation Closing
void closing(Mat & src, Mat & dest,unsigned int erosion_size)
{
   /**
   * Morphological operation closing
   * Imput: 
   *	1-Src Matix
   * 	2-Oupt Matrix
   * 	3-number of erosions
   * Output:
   * 	-Matrix "closing"
   */
  Mat element = getStructuringElement( MORPH_RECT,Size( 2*erosion_size + 1, 2*erosion_size+1 ),
						       cv::Point( erosion_size, erosion_size ) );
	
		  erode( src, src, element );
		  dilate( src, src, element ); 
}

void boundig_box( Mat & src_tresh, Mat & threshold_output)
{
/** @brief FIND THE BOUDING BOX
 *  @param Mat Src Matix BW
 *  @param Mat Oupt Matrix BW
 *  @return Matrix "Opening" // not implemented yet
 *  @pre requires the source matrix with contours to compute the boundig_box of with one 
 * and the output matrix
 *  @post void!
 * \n 
 */

	 vector<vector<cv::Point> > contours;
	 vector<Vec4i> hierarchy;
	 // random number
		RNG rng(12345);
	/// Find contours
  findContours( src_tresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

	 /// Approximate contours to polygons + get bounding rects and circles
  vector<vector<cv::Point> > contours_poly( contours.size() );
  vector<Rect> boundRect( contours.size() );
  vector<cv::Point2f>center( contours.size() );
  vector<float>radius( contours.size() );

  for( unsigned i = 0; i < contours.size(); i++ )
     { approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
       boundRect[i] = boundingRect( Mat(contours_poly[i]) );
       minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
     }

	/// Draw polygonal contour + bonding rects + circles
  Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
  
  for( unsigned i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, cv::Point() );
       rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
       circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
     }

  /// Show in a window
  
  imshow( "Contours", drawing );

}



/// Find discriptors (Match points)
void descritor( Mat & img_object, Mat & img_scene)
{
  
//*****************************************************
///http://docs.opencv.org/doc/tutorials/features2d/feature_homography/feature_homography.html#feature-homography
//*****************************************************

  
  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 1000;

  SurfFeatureDetector detector( minHessian );

  std::vector<KeyPoint> keypoints_object, keypoints_scene;

  detector.detect( img_object, keypoints_object );
  detector.detect( img_scene, keypoints_scene );

  //-- Step 2: Calculate descriptors (feature vectors)
  SurfDescriptorExtractor extractor;

  Mat descriptors_object, descriptors_scene;

  extractor.compute( img_object, keypoints_object, descriptors_object );
  extractor.compute( img_scene, keypoints_scene, descriptors_scene );

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match( descriptors_object, descriptors_scene, matches );

  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_object.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  //printf("-- Max dist : %f \n", max_dist );
  //printf("-- Min dist : %f \n", min_dist );

  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
  std::vector< DMatch > good_matches;

  for( int i = 0; i < descriptors_object.rows; i++ )
  { if( matches[i].distance < 2*min_dist )
     { good_matches.push_back( matches[i]); }
  }

  Mat img_matches;
  drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  //-- Localize the object
  std::vector<cv::Point2f> obj;
  std::vector<cv::Point2f> scene;

  for(unsigned int i = 0; i < good_matches.size(); i++ )
  {
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
  }

  std::cout<<"Obj "<<obj.size()<<" scene: "<<scene.size()<<std::endl;
//alteração feita pelo jorge! da erro quando obj.size e scene.size <4
	if(obj.size()>=4 && scene.size()>=4)
	{
	  
		Mat H = findHomography( obj, scene, CV_RANSAC );

	  //-- Get the corners from the image_1 ( the object to be "detected" )
	  std::vector<cv::Point2f> obj_corners(4);
	  obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
	  obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
	  std::vector<cv::Point2f> scene_corners(4);

	  perspectiveTransform( obj_corners, scene_corners, H);

	  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
	  line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
	  line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
	  line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
	  line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
	  
	}
  //-- Show detected matches
  imshow( "Good Matches & Object detection", img_matches );

  }


