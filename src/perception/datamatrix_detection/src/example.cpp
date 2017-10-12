#include <iostream>
#include <string>

using namespace std;


class Bar
{
    public:
        Bar()
        {
        }
        
        
        virtual string getName()
        {
            return string("unnamed");
        }
        
        void formatName()
        {
            cout<<"My name is: "<<getName()<<endl;
        }
    
        string name;
        
    private:
        double p1;
};


class Foo: public Bar
{
    public:
        
        Foo()
        {
        }
        
        string getName()
        {
            return string("joao");
        }
        
};

int main()
{
    cout<<"I'm alive"<<endl;
 
//     Bar b;
    Foo f;
    
    
    f.formatName();
    
//     cout<<"Name: "<<b.getName()<<endl;
//     cout<<"name: "<<f.name<<endl;
    
    return 0;
}