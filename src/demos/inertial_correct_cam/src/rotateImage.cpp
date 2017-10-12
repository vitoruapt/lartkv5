/*******************************************************************************
*******************
  Software License Agreement (BSD License)
 
  Copyright (c) 2011-2013, LAR toolkit developers - University of Aveiro - 
http://lars.mec.ua.pt
  All rights reserved.
 
  Redistribution and use in source and binary forms, with or without 
modification, are permitted
  provided that the following conditions are met:
 
   *Redistributions of source code must retain the above copyright notice, this 
list of
    conditions and the following disclaimer.
   *Redistributions in binary form must reproduce the above copyright notice, 
this list of
    conditions and the following disclaimer in the documentation and/or other 
materials provided
    with the distribution.
   *Neither the name of the University of Aveiro nor the names of its 
contributors may be used to
    endorse or promote products derived from this software without specific 
prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
MERCHANTABILITY AND
  FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY 
OF LIABILITY, WHETHER
  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT
  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH 
DAMAGE.
 
********************************************************************************
*******************/

/*! 
 *      @file   rotateImage.cpp
 *      @brief  this is the main body of the class created. This class will 
subscribe to frames and inertial sensor data and will transform the frame 
acording to the data obtained.
 *  
 *      @author        João Peixoto, joao.peixoto@ua.pt
 *      @date 17/03/2015
 *      @version V0.0
 *      @internal
 * 
 *              Revision        ---
 *              Compiler        gcc
 *              Company         DEM - Universidade de Aveiro
 *              Copyright       Copyright (c) 2015, João Peixoto
 * 
 *              Info:           
 *      
 *      command to make doxyfile: "make doxyfile" than "make doc"
 * 
 */

#include <inertial_correct_cam/rotateImage.h>
 
class rotateImage
{
    
    ros::NodeHandle nh_;                    //handler for ROS node

    image_transport::ImageTransport it_;    //handler for ROS image conversion
    image_transport::Subscriber sub_image_; //subscriber of frame
    ros::Subscriber sub_data_;              //subscriber of sensor data

                                            //publisher of final frame
    image_transport::Publisher publish_frame_; 
    

     
private:
    //variable declaration for calc. purposes
    int Ax, Ay, Az;                         //accelerometers
    int Gx, Gy, Gz;                         //gyroscopes
    int Mx, My, Mz;                         //magnetometers
    imu_network::sensors_network received_values_;
                                            //values received from topic_raw_data
    double angX, angY, angZ;                //angles in three directions
    double maxX, maxY, maxZ;                //maximum values (rest) of the sensors
    double comX, comY, comZ;                //compensation value for A*

    //debug purposes
    bool show_raw_data_;                    //prints (or not) the raw_data in the terminal
    bool calibrate_accel_;                  //runs 1000 iterations and does the average

    int n;
     
public:
    rotateImage()                           //constructor
    : it_(nh_)
    {
        sub_image_ = it_.subscribe("/camera/image_rect_color", 1, &rotateImage::rotate_frame, this);
                                            //subscribes the frame
                                            //every time that the topic is 
                                            //updated, the function 
                                            //rotate_frame is called
        sub_data_ = nh_.subscribe("topic_raw_data", 1000, &rotateImage::sensor_receiver, this);
                                            //subscribes the sensors data
                                            //every time that the topic is 
                                            //updated, the function 
                                            //sensor_receiver is called
        //publish_frame_ = nh_.advertise< sensor_msgs::ImageConstPtr& >( "/corrected_image", 1 );
                                            //publishes the new frame


        //initialization
        angX = 0;
        angY = 0;
        angZ = 0;
        maxX = -255;
        maxY = 255;
        maxZ = 255;
        comX = 12;
        comY = -5;
        comZ = -2;
        show_raw_data_ = false;
        calibrate_accel_ = false;
        n = 0;
    }
     
    ~rotateImage()                          //desconstructor
    {

    }
    
    void sensor_receiver(const imu_network::sensors_network::ConstPtr& msg)
    {
        Ax = msg->sensors_val[0].S_Ax;
        Ay = msg->sensors_val[0].S_Ay;
        Az = msg->sensors_val[0].S_Az;

        Gx = msg->sensors_val[0].S_Gx;
        Gy = msg->sensors_val[0].S_Gy;
        Gz = msg->sensors_val[0].S_Gz;

        Mx = msg->sensors_val[0].S_Mx;
        My = msg->sensors_val[0].S_My;
        Mz = msg->sensors_val[0].S_Mz;

        calculate_angles();

        if(show_raw_data_ == true)
        {
            cout << "===============================================" << endl;
            cout << "===============================================" << endl;
            cout << "Ax = " << Ax <<  "     Ay = " << Ay <<  "     Az = " << Az << endl;
            cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
            cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
            cout << "Gx = " << Gx <<  "     Gy = " << Gy <<  "     Gz = " << Gz << endl;
            cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
            cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
            cout << "Mx = " << Mx <<  "     My = " << My <<  "     Mz = " << Mz << endl;
            cout << "===============================================" << endl;
            cout << "===============================================" << endl << endl << endl << endl;
        }
        if(show_math_data_G == true)
        {
            cout << "===============================================" << endl;
            cout << "===============================================" << endl;
            cout << "Ax = " << Ax <<  "     Ay = " << Ay <<  "     Az = " << Az << endl;
            cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
            cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
            cout << "angX = " << angX <<  "     angY = " << angY <<  "     angZ = " << angZ << endl;
            cout << "===============================================" << endl;
            cout << "===============================================" << endl << endl << endl << endl;
        }
        return;
    }

    void calculate_angles(void)
    {
        if(abs(Ax/maxX) > 1) angX = 0; else angX =  acos(Ax/maxX) * (180/PI);
        if(abs(Ay/maxY) > 1) angY = 0; else angY =  acos(Ay/maxY) * (180/PI);
        if(abs(Az/maxZ) > 1) angZ = 0; else angZ =  acos(Az/maxZ) * (180/PI);

        if(Ay < 0) angX *= -1;


        if(calibrate_accel_ == true)
        {
            comX += Ax;
            comY += Ay;
            comZ += Az;
            n++;
            cout << n << endl;
            int iterations = 1000;
            if(n==iterations)
            {
                comX /= iterations;
                comY /= iterations;
                comZ /= iterations;
                cout << comX << endl;
                cout << comY << endl;
                cout << comZ << endl;
                sleep(10000);
            }
        }
        return;
    }

    void rotate_frame(const sensor_msgs::ImageConstPtr& frame)
    {
        char key;
    /*
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
     ||                                             ||
     ||   CONVERTS THE ROS IMAGE TO OPENCV IMAGE    ||
     ||                                             ||
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
    */
        namedWindow(WINDOW);
        cv_bridge::CvImagePtr src;
        const std::string& encoding = std::string();
        try
        {
            src = cv_bridge::toCvCopy(frame, encoding);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    /*
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
     ||                                             ||
     ||            ROTATES OPENCV IMAGE             ||
     ||                                             ||
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
        ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    */
        Point2f srcTri[3];
        Point2f dstTri[3];

        Mat rot_mat( 2, 3, CV_32FC1 );
        Mat warp_mat( 2, 3, CV_32FC1 );
        Mat warp_dst, warp_rotate_dst;


        warp_dst = Mat::zeros( src->image.rows, src->image.cols, src->image.type() );



        Point center = Point( warp_dst.cols/2, warp_dst.rows/2 );
        double angle, scale, distortion1, distortion2;


        angle = angX;
        distortion1 = 0/90;
        distortion2 = 0/90;
        scale = 1;


        srcTri[0] = Point2f( src->image.cols*0.0, src->image.rows*0.0 );
        srcTri[1] = Point2f( src->image.cols*1.0, src->image.rows*0.0 );
        srcTri[2] = Point2f( src->image.cols*0.0, src->image.rows*1.0 );
            
        dstTri[0] = Point2f( src->image.cols*0.0, src->image.rows*0.0 );
        dstTri[1] = Point2f( src->image.cols*(1-distortion1), src->image.rows*0.0 );
        dstTri[2] = Point2f( src->image.cols*0.0, src->image.rows*(1-distortion2) );


        warp_mat = getAffineTransform( srcTri, dstTri );
        warpAffine( src->image, warp_dst, warp_mat, warp_dst.size() );

        rot_mat = getRotationMatrix2D( center, angle, scale );
        warpAffine( warp_dst, warp_rotate_dst, rot_mat, warp_dst.size() );
        
        if(show_image_G == true)
        {
            imshow( WINDOW, warp_rotate_dst );
            key = waitKey(1);
            if(key == 'q' || key == 'Q')
            {
                show_image_G = false;
                destroyAllWindows();
                cout << "\n   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n||                                            ||\n||   You closed the window.                   ||\n||   Press [Ctrl+C] to stop the programm!     ||\n||                                            ||\n   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  " << endl;
            }
        }
    }
 };
 
 
int main(int argc, char** argv)
{
    setBools();
    ros::init(argc, argv, "inertial_correct_cam");
    rotateImage ic;
    ros::spin();
    return 0;
}


void setBools(void)
{
    string key1;
    string key2;
    do
    {
        cout << "Do yow want to see the angle values in the terminal? [y/n]: ";
        cin >> key1;
    }while(key1[0] != 'y' && key1[0] != 'Y' && key1[0] != 'n' && key1[0] != 'N');
    do
    {
        cout << "Do yow want to see the corrected image? [y/n]: ";
        cin >> key2;
    }while(key2[0] != 'y' && key2[0] != 'Y' && key2[0] != 'n' && key2[0] != 'N');

    if(key1[0] == 'y' || key1[0] == 'Y') show_math_data_G = true; else show_math_data_G = false;
    if(key2[0] == 'y' || key2[0] == 'Y') show_image_G = true;     else show_image_G = false;
}