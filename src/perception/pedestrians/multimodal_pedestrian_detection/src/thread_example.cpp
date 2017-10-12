#include <iostream>
#include <vector>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
using namespace std;

typedef enum
{
    VEHICLE,
    PEDESTRIAN,
    UNKNOWN
} TypeClassification;

class Candidate
{
    public:
        
        typedef boost::shared_ptr<Candidate> Ptr;
        
        Candidate(double input_image, uint priority_, uint id_)
        {
            classification = UNKNOWN;
            
            id = id_;
            image = input_image;
            priority = priority_;
        }
    
        
        void classify()
        {
            t = boost::thread(boost::bind(&Candidate::do_classfication,this));
        }
        
        string getClassification()
        {
            switch(classification)
            {
                case VEHICLE:
                    return "vehicle";
                case PEDESTRIAN:
                    return "pedestrian";
                case UNKNOWN:
                    return "unknown";
            }
            
            return "unknown";
        }
    
        void do_classfication()
        {
            //Run the heady classifcation code from pedro
            int c = 0;
            cout<<"Processing image"<<endl;
            while(c<100)
            {
                c++;
                boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            }
            
            classification = PEDESTRIAN;
            
            cout<<"done"<<endl;
        }
    
        uint id;
        uint priority;
        TypeClassification classification;
        double image;
        boost::thread t;
    
};


int var1;

void separte_loop()
{
    while(1)
    {
        cout<<"in secondary loop: "<< var1++ <<endl;   
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    }
}

int main()
{
    
    vector<Candidate::Ptr> candidate_list;
    
    
    cout<<"i'm alive"<<endl;
	
    
    for(uint i=0;i<5;i++)
    {
        Candidate::Ptr candidate = Candidate::Ptr(new Candidate(0.0,i,i));
        candidate_list.push_back(candidate);
    }
    
    
    for(uint i=0;i<candidate_list.size();i++)
    {
        if(candidate_list[i]->priority > 2)
            candidate_list[i]->classify();
    }
    
    for(uint i=0;i<candidate_list.size();i++)
    {
        candidate_list[i]->t.join();
    }
//     while(1)
//     {
//         cout<<endl;
    for(uint i=0;i<candidate_list.size();i++)
    {
        cout<<"candidate: "<<candidate_list[i]->id<<" priority: "<<candidate_list[i]->priority<< " classification: "<<candidate_list[i]->getClassification()<<endl;
    }
    
}