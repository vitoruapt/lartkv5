#include "peddetect.h"

using namespace cv;

int main(int, char**)
{
    PVector vect;
	vect.clear();
	
    Mat frame;
	string folder_path="/media/Novo volume/bags_tese_results/teste_AtlasCar_outside/Rua_DETI_vir";
	
    path p = folder_path;
  
    try
    {
        if (exists(p))    // does p actually exist?
        {

        if (is_directory(p))      // is p a directory?
        {
        
            copy(directory_iterator(p), directory_iterator(), back_inserter(vect)); 
        
        }
        else
        cout << p << " is not a folder"<<endl;

        }
    
    }
    catch (const filesystem_error& ex)
    {
        cout << ex.what() << '\n';
    }
    
	sort(vect.begin(), vect.end() );
	
	frame=imread(vect[0].string(), CV_LOAD_IMAGE_COLOR);
	
	for(int i=0; i<vect.size(); i++)
	{
	  cout<<vect[i]<<endl;
	}
	
	cout<<vect.size()<<endl;
	// record video
        VideoWriter record("/media/Novo volume/bags_tese_results/videos/Rua_DETI_vir.avi", CV_FOURCC('D','I','V','X'), 16, frame.size(), true);
        if( !record.isOpened() ) {
                printf("VideoWriter failed to open!\n");
                return -1;
        }
        
        namedWindow("video",1);
        
        for(uint n=1; n<vect.size();n++)
        {
                // get a new frame from camera
                string ImgPath=vect[n].string();
                frame=imread(ImgPath, CV_LOAD_IMAGE_COLOR);

                // show frame on screen
                imshow("video", frame); 
                
                // add frame to recorded video
                record << frame; 

                if(waitKey(30) >= 0) break;
        }

        // the camera will be deinitialized automatically in VideoCapture destructor
        // the recorded video will be closed automatically in the VideoWriter destructor
        return 0;
}