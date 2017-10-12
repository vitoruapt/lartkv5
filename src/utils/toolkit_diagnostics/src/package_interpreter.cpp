#include <iostream>
#include <fstream> 

#include <gtkmm.h>

#include <boost/filesystem.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
 
#ifndef ROOT_PATH
#define ROOT_PATH @ROOT_PATH@
#endif

using namespace std;
using namespace boost;
using namespace boost::property_tree;

class InterpreterGUI: public Gtk::Window
{
    public:
        InterpreterGUI()
        {
            //Set the title of the window and size
            set_title("Package interpreter");
            set_default_size(1100,800);
            
            //Get all paths from a root path must be the src/ folder of a catkin workspace
            getPaths(ROOT_PATH);
            getPackages();
            
            //Create the Tree model
            ref_tree_model = Gtk::ListStore::create(columns);
            
            //Set the tree model to the tree view
            tree_view.set_model(ref_tree_model);
            
            //Add rows to the tree view
            Gtk::TreeModel::Row row;
            for(uint i=0;i<packages.size();i++)
            {
                row = *(ref_tree_model->append());
                
                row[columns.col_package] = packages[i].name;
                row[columns.col_author] = packages[i].author;
                
                string maintainer_plus_email = packages[i].maintainer + " (" + packages[i].maintainer_email + ")";
                row[columns.col_maintainer] = maintainer_plus_email;
                row[columns.col_licence] = packages[i].license;
            }
            
            //Append columns to the tree view
            tree_view.append_column("Package", columns.col_package);
            tree_view.append_column("Author", columns.col_author);
            tree_view.append_column("Maintainer", columns.col_maintainer);
            tree_view.append_column("License", columns.col_licence);
            
            //Set tree view properties
//             tree_view.set_headers_clickable();
            tree_view.set_reorderable();
            tree_view.set_enable_search();
            
            //Add the scrolled window to the window and the tree view to the scrolled window
            add(scrolled_window);
            scrolled_window.add(tree_view);

            //Show all
            show_all_children();
        }
        
        virtual ~InterpreterGUI()
        {
        }
    
    protected:
        
        //The PackageXML structure, holds package info
        struct PackageXML
        {
            boost::filesystem::path path;
            string name;
            string author;
            string maintainer;
            string maintainer_email;
            string license;
        };
        
        //Tree model columns, define the columns of the tree view
        class ModelColumns : public Gtk::TreeModel::ColumnRecord
        {
            public:

                ModelColumns()
                {
                    add(col_package);
                    add(col_author);
                    add(col_maintainer);
                    add(col_licence);
                }

                Gtk::TreeModelColumn<Glib::ustring> col_package;
                Gtk::TreeModelColumn<Glib::ustring> col_author;
                Gtk::TreeModelColumn<Glib::ustring> col_maintainer;
                Gtk::TreeModelColumn<Glib::ustring> col_licence;
        };
        
        //Get all PackageXML structures
        void getPackages()
        {
            for(uint i=0;i<paths.size();i++)
                interpreterFile(paths[i]);
        }
        
        const ptree& empty_ptree()
        {
            static ptree t;
            return t;
        }

        //Read a package.xml file and push the obtained PackageXML structure to the vector
        void interpreterFile(boost::filesystem::path path)
        {
            std::filebuf fb;
            if(!fb.open(path.string().c_str(),std::ios::in))
            {
                cout<<"Cannot open file: "<<path.string()<<endl;
                return;
            }
            
            std::istream is(&fb);
            
            // populate tree structure pt
            using boost::property_tree::ptree;
            ptree pt;
            read_xml(is, pt);

            PackageXML package;
            package.path = path;
            
            BOOST_FOREACH(ptree::value_type const& v, pt.get_child("package") )
            {
                if( v.first == "name" )
                    package.name = v.second.get_value<string>();
                
                if( v.first == "maintainer" )
                {
                    package.maintainer = v.second.get_value<string>();
                    
                    const ptree & attributes = v.second.get_child("<xmlattr>", empty_ptree());
                    BOOST_FOREACH(const ptree::value_type &va, attributes)
                        if(va.first == "email")
                            package.maintainer_email = va.second.get_value<string>();
                }
                
                if( v.first == "author" )
                    package.author = v.second.get_value<string>();
                
                if( v.first == "license" )
                    package.license = v.second.get_value<string>();
                
                if(package.name.empty())
                    package.name = "NO NAME";
                
                if(package.maintainer.empty())
                    package.maintainer = "NO MAINTAINER";
                
                if(package.author.empty())
                    package.author = "NO AUTHOR";
                
                if(package.license.empty())
                    package.license = "NO LICENSE";
            }
            
            packages.push_back(package);
            
            fb.close();
        }
        
        void getPaths(boost::filesystem::path top_path)
        {
            //Get all packages.xml paths, with boost recursive iterator
            boost::filesystem::recursive_directory_iterator dir(top_path), end;
            while (dir != end)
            {
                if (dir->path().filename() == "package.xml")
                    paths.push_back(dir->path());
                
                ++dir;
            }
        }
        
        vector<boost::filesystem::path> paths;
        vector<PackageXML> packages;
        
        ModelColumns columns;
        Gtk::TreeView tree_view;
        Glib::RefPtr<Gtk::ListStore> ref_tree_model;
        Gtk::ScrolledWindow scrolled_window;
};

int main(int argc,char**argv)
{
    Glib::RefPtr<Gtk::Application> app = Gtk::Application::create(argc, argv, "org.gtkmm.example");
 
    InterpreterGUI gui;
    
    return app->run(gui);
}

