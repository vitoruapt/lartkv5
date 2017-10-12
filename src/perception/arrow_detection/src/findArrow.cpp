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
 *  @brief  using openCV, this classe find an arrow in an image
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
    
    bool findArrow::found_new_point=false;

    // Const variables
    const unsigned int findArrow::EROSION_SIZE=1;
    const unsigned int findArrow::EROSION_SIZE_MIN=1;
    const unsigned int findArrow::THRESH = 100;
    const unsigned int findArrow::MAX_THRESH = 255;//Trackbar
    const unsigned int findArrow::LINE_THICKNESS = 2;
  
// For validate_lengths function
    const unsigned short findArrow::VAL_MAX_LENGTH= 500;
    const unsigned short findArrow::VAL_MIN_LENGTH = 30;//
    const float findArrow::VAL_MIN_RELATION_LENGTH = 1.3; 
    const float findArrow::VAL_MAX_RELATION_LENGTH = 3.1;
  
// Variables for validate_contour_Area_Length function  
    const double findArrow::BBCA_MIN = 1.3;//1.3;
    const double findArrow::BBCA_MAX = 1.9;//1.7;
    const double findArrow::BBCL_MIN = 7;//12;
    const double findArrow::BBCL_MAX = 35;//30;
    const double findArrow::CACL_MIN = 4;//6;
    const double findArrow::CACL_MAX = 22;//17;
    const double findArrow::AREA_MIN = 200;//17;//500

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

bool findArrow::validate_contour_Area_Length(vector<Point> contours,vector<double> bbox_points,
const double bbca_min,
const double bbca_max,const double bbcl_min, const double bbcl_max,
const double cacl_min,const double cacl_max)
{
      /** 
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
  
    edges=frame.clone();
    //imshow("find_contours", (edges));
    //cv::waitKey(20);
//imshow("begin", (edges));
    // Histogram Equalization 
    //equalizeHist( edges, edges );
  //  imshow("equalizeHist", (edges));
    // Adaptive Threshold
    adaptiveThreshold(edges,edges_b, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY,7,5);
        
    //    imshow("adaptiveThreshold", (edges_b));
    
    // Adaptative Threshold Otsu
    threshold(edges, edges, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
   // imshow("threshold", (edges));
    
    // Combine both thresholds 
    edges = edges & edges_b;
     
    // Find countours
    vector<vector<Point> > contour = find_contours(edges, edges, LINE_THICKNESS);
//imshow("find_contours", (edges));
    //cv::waitKey(20);
        
    // Filtering contours
    edges = edges > 128;
            
//imshow("before boundig_box_small", (edges));
    // Boundig box
    vector<RotatedRect> pontos_retangulos = boundig_box_small( edges,edges,contour,center_point);
//imshow(" Arrow detection", (edges));
    
        
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
    
    findContours( src_tresh, contours2, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE,
                    Point(0, 0) );
    
    contours=contours2;
    
    
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
    
        minRect[i] = minAreaRect( Mat(contours[i]) );
        if( contours[i].size() > 5 )
        { 
            minEllipse[i] = fitEllipse( Mat(contours[i]) ); 
        }
    }   
        
/// THIS FUNCTION MUST END HERE

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
    
        /// Start STATE machine
        control=true;//reset control var

        if(contours[i].size()<7)
        {
            control=false;
        }
        ///validate_contour_Area_Length
        if(control)if(!validate_contour_Area_Length(contours[i],bbox_points))
        {
            control=false;
        }
        ///verify lengths
        if(control)if(!validate_lengths(rect_points)) 
        {
            control=false;
        }
        ///validate_number of arrow corners
        if(control)if(!validate_corners_arrow(contours[i]))
        {
            control=false;
        }
        /// Find the convex hull object for each contour                
        /// convexHull  
        if(control)if(!validate_convexHull(contours[i]))
        {
            control=false;
        }
        if(control)
        {
            
        //draw point
            src_centre=calculate_center_contour(rect_points,4);
            
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
