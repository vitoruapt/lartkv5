 /**************************************************************************************************
  Software License Agreement (BSD License)
 
  Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - http://lars.mec.ua.pt
  All rights reserved.
 
  Redistribution and use in source and binary forms, with or without modification, are permitted
  provided that the following conditions are met:
 
   *Redistributions of source code must retain the above copyright notice, this list of
    conditions and the following disclaimer.
   *Redistributions in binary form must reproduce the above copyright notice, this list of
    conditions and the following disclaimer in the documentation and/or other materials provided
    with the distribution.
   *Neither the name of the University of Aveiro nor the names of its contributors may be used to
    endorse or promote products derived from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
  FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************************************************************************************************/

/*! 
 *  @file   findArrow.cpp
 *  @brief  =========== Find arrow ==========
 *  
 *  @author     César Sousa, cesarsousa@ua.pt
 *  @date 6-3-2014
 *  @version V0.0
 *  @internal
 * 
 *      Revision    ---
 *      Compiler    gcc
 *      Company     DEM - Universidade de Aveiro
 *      Copyright   Copyright (c) 2014, César Sousa
 * 
 *      Info:       
 *  
 *  command to make doxyfile: "make doxyfile" than "make doc"
 * 
 */

#include <arrow_detection/findArrow.h>

    //global vars
    //cv::Point src_centre(0,0);
    //unsigned int src_centre_x=0;
    //unsigned int src_centre_y=0;
    
    bool findArrow::found_new_point=false;

    // Const variables

    const unsigned int findArrow::EROSION_SIZE=1;
    const unsigned int findArrow::EROSION_SIZE_MIN=1;
    const unsigned int findArrow::THRESH = 100;
    const unsigned int findArrow::MAX_THRESH = 255;//Trackbar
    const unsigned int findArrow::LINE_THICKNESS = 2;
  
// For validate_lengths function
    const unsigned short findArrow::VAL_MAX_LENGTH= 500;
    const unsigned short findArrow::VAL_MIN_LENGTH = 100;
    const float findArrow::VAL_MIN_RELATION_LENGTH = 1.3; 
    const float findArrow::VAL_MAX_RELATION_LENGTH = 3.1;
  
// Variables for validate_contour_Area_Length function  
    const double findArrow::BBCA_MIN = 1.3;//1.3;
    const double findArrow::BBCA_MAX = 1.9;//1.7;
    const double findArrow::BBCL_MIN = 7;//12;
    const double findArrow::BBCL_MAX = 35;//30;
    const double findArrow::CACL_MIN = 4;//6;
    const double findArrow::CACL_MAX = 22;//17;
    const double findArrow::AREA_MIN = 500;//17;

// Variable for validate_convexHull     
    const unsigned short findArrow::CONVEX_HULL_CORNERS = 5;
    
// Variable for validate_corners_arrow
    const unsigned short findArrow::CONVEX_ARROW_CORNERS = 7;

Point findArrow::calculate_center_contour(Point2f contour[],unsigned int size_contour)
{
    
/** @brief Calculates the center of an contour
 *  @param Point2f contour
 *  @param contour size
 *  @return Center Point 
 *  @pre Contour must be greater than 1 point;
 * \n 
 */
    assert (size_contour>0);
    unsigned int cont=0;
    double xx=0,yy=0;
    Point result;
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
void findArrow::Opening(Mat & src, Mat & dest,unsigned int erosion_size)
{
   /**
   * Morphological operation OPENING
   * Imput: 
   *    1-Src Matix
   *    2-Oupt Matrix
   *    3-number of erosions
   * Output:
   *    -Matrix "Opening"
   */
    assert(erosion_size > 0);
  Mat element = getStructuringElement( MORPH_RECT,Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                               Point( erosion_size, erosion_size ) );
    
          dilate( src, dest, element ); 
          erode( dest, dest, element );
}

/// Morphological operation Closing
void findArrow::closing(Mat & src, Mat & dest,unsigned int erosion_size)
{
   /**
   * Morphological operation closing
   * Imput: 
   *    1-Src Matix
   *    2-Oupt Matrix
   *    3-number of erosions
   * Output:
   *    -Matrix "closing"
   */
   assert(erosion_size > 0);
  Mat element = getStructuringElement( MORPH_RECT,Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                               Point( erosion_size, erosion_size ) );
    
          erode( src, dest, element );
          dilate( dest, dest, element ); 
}

void findArrow::boundig_box( Mat & src_tresh, Mat & threshold_output)
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

     vector<vector<Point> > contours;
     vector<Vec4i> hierarchy;
     // random number
        RNG rng(12345);
    /// Find contours
  findContours( src_tresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

     /// Approximate contours to polygons + get bounding rects and circles
  vector<vector<Point> > contours_poly( contours.size() );
  vector<Rect> boundRect( contours.size() );
  vector<Point2f>center( contours.size() );
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
       drawContours( drawing, contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
       rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
       circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
     }

  /// Show in a window
  
  //imshow( "Contours", drawing );

}



/// Find discriptors (Match points)
void findArrow::descritor( Mat & img_object, Mat & img_scene)
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

  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );

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
  std::vector<Point2f> obj;
  std::vector<Point2f> scene;

  for(unsigned int i = 0; i < good_matches.size(); i++ )
  {
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
  }

cout<<"Obj "<<obj.size()<<" scene: "<<scene.size()<<endl;
//alteração feita pelo jorge! da erro quando obj.size e scene.size <4
    if(obj.size()>=4 && scene.size()>=4)
    {
      
        Mat H = findHomography( obj, scene, CV_RANSAC );

      //-- Get the corners from the image_1 ( the object to be "detected" )
      std::vector<Point2f> obj_corners(4);
      obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
      obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
      std::vector<Point2f> scene_corners(4);

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
  

bool findArrow::validate_contour_Area_Length(vector<Point> contours,vector<double> bbox_points,
const double bbca_min,
const double bbca_max,const double bbcl_min, const double bbcl_max,
const double cacl_min,const double cacl_max)
{
      /** @file test1.cpp
 *  @brief Validate the realtion between area, length, bb and area size.
 *  @param array with 4 Points (corners of rectangle)
 *  @param (optional) const unsigned short max_length
 *  @param (optional) const unsigned short min_length
 *  @param (optional) const float min_relation_length
 *  @param (optional) const float max_relation_length
 *  @return bool whith the validation of the bounding box
 *  @pre Point2f array with 4 points 
 * \n 
 */
    double bb_length = bbox_points[0];//previously sorted
    double bb_Width = bbox_points[3];
    double bba =  bb_Width*bb_length;
    
    double ca = contourArea(contours);
    double cl = arcLength( contours, true );
    
    double bb_ca = bba/ca;//relations between bounding boxes, countour area and contour lenght
    double bb_cl = bba/cl;
    double ca_cl = ca/cl;
    
    return ((bb_ca > bbca_min && bb_ca < bbca_max) && (bb_cl > bbcl_min && bb_cl < bbcl_max) && 
        (ca_cl > cacl_min && ca_cl < cacl_max) && ca > AREA_MIN);
    
}

/// SORTE POINTS helper validate_lengths 
vector<double> findArrow::sort_points(Point2f rect_points[])
{
    vector<double> res(4);//store distance betwen 2 consecutive points
    
    for(unsigned short w=0; w<4 ; w++)//compute the distance betwen 2 points
    {
    res[w]= norm(Mat(rect_points[w]),Mat(rect_points[(w+1)%4]));
    }
    std::sort(res.begin(),res.end()); //sort lengths
    
    return res;
}


bool findArrow::validate_lengths(Point2f rect_points[], const unsigned short max_length,
                const unsigned short min_length, const float min_relation_length,
                const float max_relation_length)
{ 
    /** @brief Validate the size and relation height/width of bounding box of the arrow
 *  @param array with 4 Points (corners of rectangle)
 *  @param (optional) const unsigned short max_length
 *  @param (optional) const unsigned short min_length
 *  @param (optional) const float min_relation_length
 *  @param (optional) const float max_relation_length
 *  @return bool whith the validation of the bounding box
 *  @pre Point2f array with 4 points 
 * \n 
 */    
    assert((max_length > 0) && (min_length >0) && (min_relation_length > 0) && (max_relation_length > 0));
    vector<double> res(4);
    res = sort_points(rect_points);//sort rect_points
   
    double relation_lengths = res[3]/res[0]; // height/width relation
 
    return (res[3]<max_length && (res[0]+res[3])>min_length && 
            relation_lengths>min_relation_length && relation_lengths< max_relation_length);
        
}

bool findArrow::validate_corners_arrow(vector<Point> contours,
                const unsigned short convex_arrow_corners)
{
    assert(convex_arrow_corners > 0);
    vector<Point> arrow_corners;
    vector<Point> hull( contours.size() );
            
    approxPolyDP(contours, arrow_corners, arcLength(Mat(contours), true) * 0.02, true );
                
    return(arrow_corners.size() == convex_arrow_corners);
}

//check number of corners of convexhull 
bool findArrow::validate_convexHull(vector<Point> contours,
                const unsigned short convex_hull_corners)
{
    vector<Point> approx_hull;
    vector<Point> hull( contours.size() );
    convexHull( Mat(contours), hull, false ); 
            
    approxPolyDP(Mat(hull), approx_hull, arcLength(Mat(hull), true) * 0.02, true );
                
    return(approx_hull.size() == convex_hull_corners);
}

float findArrow::innerAngle(float px1, float py1, float px2, float py2, float cx1, float cy1)  
{  
  ///http://feelmare.blogspot.pt/2012/10/inner-productthe-angle-between-two.html
 float dist1 = sqrt(  (px1-cx1)*(px1-cx1) + (py1-cy1)*(py1-cy1) );  
 float dist2 = sqrt(  (px2-cx1)*(px2-cx1) + (py2-cy1)*(py2-cy1) );  
  
 float Ax, Ay;  
 float Bx, By;  
 float Cx, Cy;  
  
 //find closest point to C  
 //printf("dist = %lf %lf\n", dist1, dist2);  
  
 Cx = cx1;  
 Cy = cy1;  
 if(dist1 < dist2)  
 {    
  Bx = px1;  
  By = py1;    
  Ax = px2;  
  Ay = py2;  
  
  
 }else{  
  Bx = px2;  
  By = py2;  
  Ax = px1;  
  Ay = py1;  
 }  
  
 float Q1 = Cx - Ax;  
 float Q2 = Cy - Ay;  
 float P1 = Bx - Ax;  
 float P2 = By - Ay;    
  
  
 float A = acos( (P1*Q1 + P2*Q2) / ( sqrt(P1*P1+P2*P2) * sqrt(Q1*Q1+Q2*Q2) ) );  
  
 A = A*180/CV_PI;  
  
 return A;  
}  


double findArrow::angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
    /**
 * Helper function to find a cosine of angle between vectors
 * from pt0->pt1 and pt0->pt2
 */
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}


void findArrow::setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
    /**
 * Helper function to display text in the center of a contour
 */
    int fontface = cv::FONT_HERSHEY_SIMPLEX;
    double scale = 0.4;
    int thickness = 1;
    int baseline = 0;

    cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
    cv::Rect r = cv::boundingRect(contour);

    cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
    cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
    cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}


// Functions 

int findArrow::find_arrow(cv::Mat frame,cv::Point & center_point)//unsigned int & size_cols,unsigned int & size_rows)
{
    //doxyfile concerning
/** @brief Find a arrow on a image and return his position
 *  @param Mat & img_scene to compute
 *  @param bool & use_descritor?? (not implemented yet)
 *  @param int cam_number (optional)
 *  @return Point(no implemented yet!) position of arrow 
 *  @pre requires a camera conected to the computer to get the image
 *  @post Point in pixies of the object, maybe will be necessary to return the iamge size! or the distance
 * to the center of the image in percentage!
 * \n 
 */
  
   /*! 
   * Create a obj "cap" that will get the video from the camera 
   * 
   *  Loop cycle:
   *         -# send the cap data into frame
   *         -# BRG to GRAY
   *         -# Histogram Equalization 
   *         -# Adaptative Threshold Otsu
   *         -# Opening
   *         -# find countours
   *         -# Boundig box
   *
   */
    
// Create a obj "cap" that will get the video from the camera 
   
        
// Loop cycle throug all frames
  
    
    Mat edges,edges_b;

    //cout << "size" << frame.size() << endl;
    
    // BRG to GRAY
    
    edges=frame;
    //size_cols = frame.cols;
    //size_rows = frame.rows;
    // Histogram Equalization 
    equalizeHist( edges, edges );
    
    // Adaptive Threshold
    imshow("antes", edges);
    adaptiveThreshold(edges,edges_b, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY,7,5);
    imshow("depois", edges_b);
    // Adaptative Threshold Otsu
    threshold(edges, edges, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    
    // Combine both thresholds 
    edges = edges & edges_b;
    
    // Closing
   
    //closing(edges, edges,1);
    
    // Find countours
    
    vector<vector<Point> > contour = find_contours(edges, edges, LINE_THICKNESS);
     cv::imshow("imagem_teste2", edges);
    cv::waitKey(20);
    //imshow("imagem_teste3", edges);
    // Filtering contours
    edges = edges > 128;
            
    // Boundig box
    
    vector<RotatedRect> pontos_retangulos = boundig_box_small( edges,edges,contour,center_point);
    imshow("imagem_teste4", (edges));
                
//                         
    //process_bbox_pnts_neighbors(pontos_retangulos);
    
    ///PARA VERIFICAR OS CANTOS, É NECESSARIO PRIMEIRO, SABER A QUE CADA PONTO PERTENCE O VERTICE,
        /// ASSIM, DEVE-SE ORDENAR 
//smallFrame = image(Rect(x, y, CROPPING_WIDTH, CROPPING_HEIGHT));
//* a fixed CROPPING_WIDTH or HEIGHT won't do. you've got to check, 
//if your Rect did not end up partly outside the image, i.e if x+CROPPING_WIDTH < img.cols-1

    /// ****EXIT*** wait for 'esc' key press for 10ms. If 'esc' key is pressed, break loop
    

    /// Destroy all Windows
    //cv::destroyAllWindows();
        
    return 0;
}


vector<vector<Point> > findArrow::find_contours(Mat & src, Mat & dest,const unsigned int & line_thinckness)
{ 
/**
 *Find Counturs
 *Imput: 
 *  1-Src Matix
 *  2-Oupt Matrix
 *  3-line_thinckness
 *  4-Erosion size
 * Output:
 *  -Mariz with the coutours! 
 */
 /** @brief Find Counturs
 *  @param Mat & src
 *  @param Mat & dest2
 *  @param const unsigned int & line_thinckness
 *  @param const unsigned int & erosion_size_min
 *  @return Void!
 *  @pre 
 *  @post Point in pixies of the object, maybe will be necessary to return the iamge size! or the distance
 * to the center of the image in percentage!
 * \n 
 */
    vector<vector<Point> > contours,contours2;
    vector<Vec4i> hierarchy;
    findContours(src, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, Point(0,0) );
    Mat drawing = Mat::zeros( dest.size(), CV_8U );
    
    for (unsigned int i=0; i<contours.size(); i++)
    {
        
        if ((hierarchy[i][3] >= 0) && contourArea(contours[i])>AREA_MIN)   //has parent, inner (hole) contour of a closed edge (looks good)
        {
            
            drawContours(drawing, contours, i, Scalar(255, 0, 0), line_thinckness, 8);                              
            
            contours2.push_back(contours[i]);
            
            
        }
    }
    dest=drawing;

    return contours;
}


vector<RotatedRect> findArrow::boundig_box_small( Mat & src_tresh, Mat & threshold_output,vector<vector<Point> > contours,
                                cv::Point & src_centre)
{
    /** @brief FIND THE SMALLEST BOUNDING BOX
 *  @param Mat Src Matix BW
 *  @param Mat Oupt Matrix BW
 *  @return  // not implemented yet
 *  @pre requires the source matrix with contours to compute the boundig_box of with one 
 * and the output matrix
 *  @post void!
 * \n 
 */
    bool control=true;
    vector<vector<Point> > contours2;
    vector<Vec4i> hierarchy;
    RNG rng(12345); // random number
/// Find contours
    //imshow(" Before src_tresh",src_tresh );
    findContours( src_tresh, contours2, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE,
                    Point(0, 0) );
    //imshow(" After src_tresh",src_tresh );
    contours=contours2;
    
    //cout << "number of contours::    " << contours.size() << endl;    
/// Approximate contours to polygons + get bounding rects and circles
    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );
          
/// Find the rotated rectangles and ellipses for each contour
    vector<RotatedRect> minRect( contours.size() );
    vector<RotatedRect> minEllipse( contours.size() );

    for( size_t i = 0; i < contours.size(); i++ )
    { 
    //cout << "contours.size " << contours[i].size() << endl;
    minRect[i] = minAreaRect( Mat(contours[i]) );
    if( contours[i].size() > 5 )
    { 
        minEllipse[i] = fitEllipse( Mat(contours[i]) ); 
    }
    }   
        
/// THIS FUNCTION MUST END HERE

    /// Draw contours + rotated rects + ellipses
    Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
    
    vector<Point> approx;
    vector<Point> approx_hull;
    vector<vector<Point> >hull( contours.size() );
    //travels througth all contours 
    for( size_t i = 0; i< contours.size(); i++ )
    {
    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
              
        Point2f rect_points[4]; 
        minRect[i].points( rect_points );
        vector<double> bbox_points;
    bbox_points = sort_points(rect_points);
    
    /// Start state machine
    control=true;//reset control var

    if(contours[i].size()<7)
    {
        control=false;// cout << "contours.size()<7" << endl;
    }
    ///validate_contour_Area_Length
    if(control)if(!validate_contour_Area_Length(contours[i],bbox_points))
    {
        control=false;// cout << "validate_contour_Area_Length" << endl;
    }
    ///verify lengths
    if(control)if(!validate_lengths(rect_points)) 
    {
        control=false;// cout << "validate_lengths" << endl;
    }
    ///validate_number of arrow corners
    if(control)if(!validate_corners_arrow(contours[i]))
    {
        control=false;// cout << "validate_corners_arrow" << endl;
    }
    /// Find the convex hull object for each contour                
    /// convexHull  
    if(control)if(!validate_convexHull(contours[i]))
    {
        control=false;// cout << "validate_convexHull" << endl;
    }
        
    if(control)
    {
        
        //cout << "centro x: " << calculate_center_contour(rect_points,4).x << endl;
        //cout << "centro y: " << calculate_center_contour(rect_points,4).y << endl;
    //draw point
        //Point src_centre;
        src_centre=calculate_center_contour(rect_points,4);
        
        //std::cout << "*******BB_SMALL__****found_point x:" << src_centre.x << "y:" << src_centre.y 
                        //<< "cols" << src_tresh.cols<< "rows" << src_tresh.rows <<std::endl;
        //src_centre.x = (double)src_tresh.cols/(double)src_centre.x;
        //src_centre.y = src_tresh.rows/src_centre.y;
        //std::cout << "*******BB_SMALL__****found_point x:" << src_centre.x << "y:" << src_centre.y << std::endl;
        found_new_point = true;
        ellipse(drawing,Point(round(src_centre.x),round(src_centre.y)),Size 
        (1,1),0,0,360,Scalar(102,0,255),3);
    //draw rectangle
        for( int j = 0; j < 4; j++ )
        {
            line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
        }
        
    //Draw center of arrow
        Moments centre_arrow;
        centre_arrow = moments( contours[i], false );

    //  Get the mass centers
        Point2f mc;
         
        mc = Point2f( centre_arrow.m10/centre_arrow.m00 , centre_arrow.m01/centre_arrow.m00 ); 
        
        ellipse(drawing,mc,Size(1,1),0,0,360,Scalar(255,255,255),3); 
        drawContours(drawing, contours, i, Scalar(255, 0, 0), 1, 8);
        
         
    }
    }
     
    threshold_output=drawing;
 
 return minRect;
}
