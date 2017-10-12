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
#ifndef _MANAGER_GUI_H_
#define _MANAGER_GUI_H_

/**
\file
\brief Atlascar Manager GUI class declaration
*/

#include <gtkmm/window.h>
#include <gtkmm/messagedialog.h>
#include <gtkmm/image.h>
#include <gtkmm/main.h>
#include <gtkmm/label.h>
#include <gtkmm/grid.h>
#include <gtkmm/frame.h>
#include <gtkmm/separator.h>
#include <glibmm/main.h>
#include <gtkmm/drawingarea.h>
#include <gdkmm/pixbuf.h>
#include <cairomm/context.h>
#include <gdkmm/general.h>
#include <glibmm/fileutils.h>


#include <atlascar_base/ManagerCommand.h>
#include <atlascar_base/ManagerStatus.h>
#include <atlascar_base/Gamepad.h>

#include <ros/package.h>

#include <boost/lexical_cast.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/format.hpp>

#include <Eigen/Dense>


using Eigen::Vector2d;

const double pi = boost::math::constants::pi<double>();

/**
 * \brief Atlascar pedal pressure sensors gui frame
 * 
 * Class that hold the pedal pressure sensor values and handlers.
 * Inherits a Gtk::Frame.
 */
class PressureSensors: public Gtk::Frame
{
	public:
		
		/**
		 * \brief Class constructor
		 *
		 * Initializes the labels and arranges the widgets inside the frame.
		 */
		PressureSensors():
		title_throttle_("Throttle:"),
		title_brake_("Brake:"),
		title_clutch_("Clutch:"),
		throttle_("NA"),
		brake_("NA"),
		clutch_("NA")
		{
			set_label("Pressure Sensors");
			set_border_width(20);
			
			add(grid_);
			
			grid_.set_border_width(10);
			
			grid_.attach(title_throttle_,1,1,1,1);
			grid_.attach(title_brake_,1,2,1,1);
			grid_.attach(title_clutch_,1,3,1,1);
			
			grid_.attach(throttle_,2,1,1,1);
			grid_.attach(brake_,2,2,1,1);
			grid_.attach(clutch_,2,3,1,1);
			
			grid_.set_row_spacing(10);
			grid_.set_column_spacing(10);
			
			grid_.set_row_homogeneous(true);
			grid_.set_column_homogeneous(true);
			
			title_throttle_.set_justify(Gtk::JUSTIFY_RIGHT);
			title_throttle_.set_halign(Gtk::ALIGN_END);
			
			throttle_.set_hexpand();
			
			title_brake_.set_justify(Gtk::JUSTIFY_RIGHT);
			title_brake_.set_halign(Gtk::ALIGN_END);
			
			title_clutch_.set_justify(Gtk::JUSTIFY_RIGHT);
			title_clutch_.set_halign(Gtk::ALIGN_END);
		}
	
		/**
		 * \brief Class destructor
		 * 
		 * Does nothing.
		 */
		~PressureSensors()
		{
		}
		
		/**
		 * \brief Function used to set the throttle pressure value
		 * \param throttle the throttle pressure value
		 */
		void setThrottle(double throttle)
		{
			throttle_pressure_=throttle;
			
			boost::format fmt("%3.1f");
			fmt % throttle_pressure_;
			
			throttle_.set_text(fmt.str());
		}
	
		/**
		 * \brief Function used to set the brake pressure value
		 * \param brake the brake pressure value
		 */
		void setBrake(double brake)
		{
			brake_pressure_=brake;
			
			boost::format fmt("%3.1f");
			fmt % brake_pressure_;
			
			brake_.set_text(fmt.str());
		}
		
		/**
		 * \brief Function used to set the clutch pressure value
		 * \param clutch the clutch pressure value
		 */
		void setClutch(double clutch)
		{
			clutch_pressure_=clutch;
			
			boost::format fmt("%3.1f");
			fmt % clutch_pressure_;
			
			clutch_.set_text(fmt.str());
		}
		
	protected:
		
		///Grid that hold the labels
		Gtk::Grid grid_;
		///Throttle title label
		Gtk::Label title_throttle_;
		///Brake title label
		Gtk::Label title_brake_;
		///Clutch title label
		Gtk::Label title_clutch_;
		
		///Throttle value label
		Gtk::Label throttle_;
		///Brake value label
		Gtk::Label brake_;
		///Clutch value label
		Gtk::Label clutch_;
		
		///Throttle value
		double throttle_pressure_;
		///Brake value
		double brake_pressure_;
		///Clutch value
		double clutch_pressure_;
};

/**
 * \brief Drawing area extension class
 * 
 * This class extends the Gtk::DrawingArea by adding text drawing functions and a Gdk::Pixbuf.
 * 
 */
class ExtendedDrawingArea: public Gtk::DrawingArea
{
	public:
		/**
		 * \brief Class constructor
		 * 
		 * Does nothing.
		 */
		ExtendedDrawingArea()
		{
			
		}
		
		/**
		 * \brief Class destructor
		 * 
		 * Does nothing.
		 */
		virtual ~ExtendedDrawingArea()
		{
			
		}
		
		/**
		 * \brief Function to draw text in a Gtk::DrawingArea
		 * \param text std::string containing the text to draw
		 * \param cr Cairo::Context in which the drawing is preformed
		 * \param rectangle_width x center position of the text
		 * \param rectangle_height y center position of the text
		 * \param font_size font size used (default 10)
		 * 
		 * Draw a bold text with Monospace font in a DrawingArea.
		 */
		void drawText(std::string text,const Cairo::RefPtr<Cairo::Context>& cr,const int rectangle_width,const  int rectangle_height,double font_size=10)
		{
			// http://developer.gnome.org/pangomm/unstable/classPango_1_1FontDescription.html
			Pango::FontDescription font;

			font.set_family("Monospace");
			font.set_weight(Pango::WEIGHT_BOLD);
			font.set_size(font_size* PANGO_SCALE);

			// http://developer.gnome.org/pangomm/unstable/classPango_1_1Layout.html
			Glib::RefPtr<Pango::Layout> layout = create_pango_layout(text);

			layout->set_font_description(font);

			int text_width;
			int text_height;

			//get the text dimensions (it updates the variables -- by reference)
			layout->get_pixel_size(text_width, text_height);

			// Position the text in the middle
			cr->move_to(rectangle_width-(text_width/2.), rectangle_height-(text_height/2.));

			layout->show_in_cairo_context(cr);
		}
		
		/**
		 * \brief Function to draw text in a Gtk::DrawingArea in italics
		 * \param text std::string containing the text to draw
		 * \param cr Cairo::Context in which the drawing is preformed
		 * \param rectangle_width x center position of the text
		 * \param rectangle_height y center position of the text
		 * \param font_size font size used (default 10)
		 * 
		 * Draw a bold italic text with Monospace font in a DrawingArea.
		 */
		void drawTextItalics(std::string text,const Cairo::RefPtr<Cairo::Context>& cr,const int rectangle_width,const  int rectangle_height,double font_size=10)
		{
			// http://developer.gnome.org/pangomm/unstable/classPango_1_1FontDescription.html
			Pango::FontDescription font;

			font.set_family("Monospace");
			font.set_weight(Pango::WEIGHT_BOLD);
			font.set_style(Pango::STYLE_ITALIC);
			font.set_size(font_size* PANGO_SCALE);
			
			// http://developer.gnome.org/pangomm/unstable/classPango_1_1Layout.html
			Glib::RefPtr<Pango::Layout> layout = create_pango_layout(text);

			layout->set_font_description(font);

			int text_width;
			int text_height;

			//get the text dimensions (it updates the variables -- by reference)
			layout->get_pixel_size(text_width, text_height);

			// Position the text in the middle
			cr->move_to(rectangle_width-(text_width/2.), rectangle_height-(text_height/2.));

			layout->show_in_cairo_context(cr);
		}
		
		/**
		 * \brief Function to draw text in a Gtk::DrawingArea, italics but not bold
		 * \param text std::string containing the text to draw
		 * \param cr Cairo::Context in which the drawing is preformed
		 * \param rectangle_width x center position of the text
		 * \param rectangle_height y center position of the text
		 * \param font_size font size used (default 10)
		 * 
		 * Draw a italic text with Monospace font in a DrawingArea.
		 */
		void drawTextItalicsNotBold(std::string text,const Cairo::RefPtr<Cairo::Context>& cr,const int rectangle_width,const  int rectangle_height,double font_size=10)
		{
			// http://developer.gnome.org/pangomm/unstable/classPango_1_1FontDescription.html
			Pango::FontDescription font;

			font.set_family("Monospace");
			font.set_style(Pango::STYLE_ITALIC);
			font.set_size(font_size* PANGO_SCALE);
			
			// http://developer.gnome.org/pangomm/unstable/classPango_1_1Layout.html
			Glib::RefPtr<Pango::Layout> layout = create_pango_layout(text);

			layout->set_font_description(font);

			int text_width;
			int text_height;

			//get the text dimensions (it updates the variables -- by reference)
			layout->get_pixel_size(text_width, text_height);

			// Position the text in the middle
			cr->move_to(rectangle_width-(text_width/2.), rectangle_height-(text_height/2.));

			layout->show_in_cairo_context(cr);
		}
	protected:
		
		///Generic pixbuf mostly used to hold a background image
		Glib::RefPtr<Gdk::Pixbuf> image_;
};

/**
 * \brief Gamepad handler class
 * 
 * This class handles communications with the gamepad, as well as the graphical representation of the gamepad.
 * This class inherits a Gtk::DrawingArea
 */
class GamepadInfo: public Gtk::DrawingArea
{
	public:
		/**
		 * \brief Class constructor
		 * 
		 * This constructor receives the path to the gamepad image thumbnail.
		 * It also initializes variables and state images.
		 */
		GamepadInfo(std::string active_path = 
		ros::package::getPath("atlascar_base")+"/images/xbox-360-controller_active_tumb.png"):
		active_path_(active_path),
		fade_(0.2)
		{
			//Set the gamepad device to a default value
			device_="NOT_SET";
			//Set the communication status to false
			active_=false;
			//Reset the topic manager command class
			command_.reset(new atlascar_base::ManagerCommand);
			
			try
			{
				//Get the active gamepad image from file
				active_image_ = Gdk::Pixbuf::create_from_file(active_path_);
				//Increase the saturation of the image
				active_image_ ->saturate_and_pixelate(active_image_ ,5.0,false);
				//Create the not active image from the same file
				not_active_image_ = Gdk::Pixbuf::create_from_file(active_path_);
				//Put the saturation of the not active image to 0
				not_active_image_->saturate_and_pixelate(not_active_image_,0.0,false);
				//Add a alpha channel to the not active image
				not_active_image_->add_alpha(false,0,0,0);
				//Use the composite image to set the alpha value to 100 (in 256)
				not_active_image_->composite_color(not_active_image_,0,0,not_active_image_->get_width(),not_active_image_->get_height(),0,0,1,1,Gdk::INTERP_BILINEAR,100,0,0,1,0xffffffff,0xffffffff);
				//Also create the auxiliary image from the same file
				auxiliary_image_ = Gdk::Pixbuf::create_from_file(active_path_);
				
			}catch(const Glib::FileError& ex)
			{
				std::cerr << "FileError: " << ex.what() << std::endl;
			}catch(const Gdk::PixbufError& ex)
			{
				std::cerr << "PixbufError: " << ex.what() << std::endl;
			}
			
			//Set background color to white
			override_background_color(Gdk::RGBA("white"));
			
			// Show at least a quarter of the image.
			if (active_image_)
				set_size_request(active_image_->get_width(), active_image_->get_height());
		}
		
		/**
		 * \brief Class destructor
		 * 
		 * Does nothing.
		 */
		~GamepadInfo()
		{
		}
		
		/**
		 * \brief Start communications with the gamepad and register callbacks
		 * 
		 * This function initializes the communication with the gamepad and also registers all the callbacks to all the buttons and axes.
		 */
		bool initializeGamepad()
		{
			//Not connected or already active
			if(device_=="Not found" || active_==true)
				return false;
			
			int ret=0;
			
			//Initialize communications with the gamepad
			ret = gamepad_.startComm(device_.c_str());
			if(ret<0)
				return false;
			
			//Register all gamepad callbacks
			gamepad_.buttons(7)->callback = sigc::mem_fun<int>(*this,&GamepadInfo::gamepadIgnitionONCallback);
			gamepad_.buttons(6)->callback = sigc::mem_fun<int>(*this,&GamepadInfo::gamepadIgnitionOFFCallback);
			gamepad_.buttons(4)->callback = sigc::mem_fun<int>(*this,&GamepadInfo::gamepadLeftLightCallback);
			gamepad_.buttons(5)->callback = sigc::mem_fun<int>(*this,&GamepadInfo::gamepadRightLightCallback);
			
			gamepad_.buttons(13)->callback = sigc::mem_fun<int>(*this,&GamepadInfo::gamepadHighLightCallback);
			gamepad_.buttons(12)->callback = sigc::mem_fun<int>(*this,&GamepadInfo::gamepadMediumLightCallback);
			gamepad_.buttons(11)->callback = sigc::mem_fun<int>(*this,&GamepadInfo::gamepadWarningCallback);
			gamepad_.buttons(0)->callback = sigc::mem_fun<int>(*this,&GamepadInfo::gamepadAutoDirectionCallback);
			
			gamepad_.buttons(2)->callback = sigc::mem_fun<int>(*this,&GamepadInfo::gamepadAutoThrottleCallback);
			gamepad_.buttons(1)->callback = sigc::mem_fun<int>(*this,&GamepadInfo::gamepadAutoBrakeCallback);
			gamepad_.buttons(3)->callback = sigc::mem_fun<int>(*this,&GamepadInfo::gamepadAutoClutchCallback);
			
			gamepad_.axes(5)->callback = sigc::mem_fun<int>(*this,&GamepadInfo::gamepadThrottleCallback);
			gamepad_.axes(2)->callback = sigc::mem_fun<int>(*this,&GamepadInfo::gamepadBrakeCallback);
			gamepad_.axes(4)->callback = sigc::mem_fun<int>(*this,&GamepadInfo::gamepadClutchCallback);
			gamepad_.axes(0)->callback = sigc::mem_fun<int>(*this,&GamepadInfo::gamepadSteeringWheelCallback);
			
			active_=true;
			return true;
		}
		
		/**
		 * \brief Graphical representation state manager
		 * 
		 * This function manages the current status of the image representation according to the most recent activity of the gamepad.
		 * 
		 */
		bool stateManager()
		{
			gamepad_.dispatch();
			
			double min_fade=0.7;
			
			if(active_)
			{
				//In fade time
				if((ros::Time::now()-ta_)<fade_)
				{
					double percentage = 1 - (ros::Time::now()-ta_).toSec()/fade_.toSec();
					
					percentage = percentage*(1-min_fade) + min_fade;

					active_image_->composite_color(auxiliary_image_,0,0,not_active_image_->get_width(),not_active_image_->get_height(),0,0,1,1,Gdk::INTERP_BILINEAR,percentage*255,0,0,1,0xffffffff,0xffffffff);
				}else
				{
					active_image_->composite_color(auxiliary_image_,0,0,not_active_image_->get_width(),not_active_image_->get_height(),0,0,1,1,Gdk::INTERP_BILINEAR,min_fade*255,0,0,1,0xffffffff,0xffffffff);
				}
			}else
				auxiliary_image_ = not_active_image_;
			
			queue_draw();
			
			return true;
		}
		
		/**
		 * \brief Mark gamepad activity
		 * 
		 * This function updates the most recent use of the gamepad.
		 */
		bool touch()
		{
			ta_=ros::Time::now();
			
			return true;
		}
		
		///Gamepad device parameter
		std::string device_;
		///Event slot, this event will be called when there is activity with the gamepad
		sigc::slot<void,const atlascar_base::ManagerCommandPtr> event_;
		
	protected:
		
		/**
		 * \brief Linear mapping function
		 * \param value input value
		 * \param min_value minimum value in the old scale
		 * \param max_value maximum value in the old scale
		 * \param min_required minimum value in the requested scale
		 * \param max_required maximum value in the requested scale
		 * \return the remapped value
		 * 
		 * This function maps a value to a different scale.
		 */
		double map(double value,double min_value,double max_value,double min_required,double max_required)
		{
			assert(min_value!=max_value);
			assert(min_required!=max_required);
			
			double m = (max_required-min_required)/(max_value-min_value);
			double b = max_required - m*max_value;
			
			return value*m+b;
		}
		
		/**
		 * \brief On draw signal handler
		 * \param cr cairo context
		 * 
		 * This function is called every time there is a redraw event on the DrawingArea.
		 */
		virtual bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr)
		{
			if (!active_image_)
				return false;
			
			if (!not_active_image_)
				return false;

			Gtk::Allocation allocation = get_allocation();
			const int width = allocation.get_width();
			const int height = allocation.get_height();

			Gdk::Cairo::set_source_pixbuf(cr, auxiliary_image_,
				(width - auxiliary_image_->get_width())/2, (height - auxiliary_image_->get_height())/2);
			
			cr->paint();
			
			if(!active_)
			{
				cr->set_source_rgb(1,0,0);
				cr->set_line_width(3);
				cr->move_to(0,0);
				cr->line_to(width,height);
				cr->stroke();
			}
	
			return true;
		}
		
		//Gamepad Handlers
		
		/**
		 * \brief Gamepad throttle handler
		 * \param value button value
		 * 
		 * Gamepad throttle event handler. This function is called every time the gamepad changes the throttle value.
		 */
		void gamepadThrottleCallback(int value)
		{	
			touch();
			double rescaled = map(value,-32767,+32767,0,1);
			command_->throttle=rescaled;
			if(!event_.empty())
				event_(command_);
		}
		
		/**
		 * \brief Gamepad brake handler
		 * \param value button value
		 * 
		 * Gamepad brake event handler. This function is called every time the gamepad changes the brake value.
		 */
		void gamepadBrakeCallback(int value)
		{
			touch();
			double rescaled = map(value,-32767,+32767,0,1);
			command_->brake=rescaled;
			if(!event_.empty())
				event_(command_);
		}
		
		/**
		 * \brief Gamepad clutch handler
		 * \param value button value
		 * 
		 * Gamepad clutch event handler. This function is called every time the gamepad changes the clutch value.
		 */
		void gamepadClutchCallback(int value)
		{
			if(value>0)
				value=0;
			
			double rescaled = map(value,0,-32767,0,1);
			
			if(rescaled==command_->clutch)
				return;
			
			touch();
			command_->clutch=rescaled;
			if(!event_.empty())
				event_(command_);
			
		}
		
		/**
		 * \brief Gamepad steering wheel handler
		 * \param value button value
		 * 
		 * Gamepad steering wheel event handler. This function is called every time the gamepad changes the steering wheel value.
		 */
		void gamepadSteeringWheelCallback(int value)
		{	
			touch();
			double rescaled = map(value,-32767,+32767,0.5,-0.5);
			command_->steering_wheel=rescaled;
			if(!event_.empty())
				event_(command_);
		}
		
		/**
		 * \brief Gamepad ignition on handler
		 * \param value button value
		 * 
		 * Gamepad ignition on event handler. This function is called every time the gamepad changes the ignition on value.
		 */
		void gamepadIgnitionONCallback(int value)
		{	
			if(value)
			{
				touch();
				command_->ignition=1;
				if(!event_.empty())
					event_(command_);
			}
		}
		
		/**
		 * \brief Gamepad ignition off handler
		 * \param value button value
		 * 
		 * Gamepad ignition off event handler. This function is called every time the gamepad changes the ignition off value.
		 */
		void gamepadIgnitionOFFCallback(int value)
		{	
			if(value)
			{
				touch();
				command_->ignition=0;
				if(!event_.empty())
					event_(command_);
			}
		}
		
		/**
		 * \brief Gamepad left light handler
		 * \param value button value
		 * 
		 * Gamepad left light event handler. This function is called every time the gamepad changes the left light value.
		 */
		void gamepadLeftLightCallback(int value)
		{
			if(value)
			{
				touch();
				command_->lights_left=!command_->lights_left;
				if(!event_.empty())
					event_(command_);
			}
		}
		
		/**
		 * \brief Gamepad right light handler
		 * \param value button value
		 * 
		 * Gamepad right light event handler. This function is called every time the gamepad changes the right light value.
		 */
		void gamepadRightLightCallback(int value)
		{
			if(value)
			{
				touch();
				command_->lights_right=!command_->lights_right;
				if(!event_.empty())
					event_(command_);
			}
		}
		
		/**
		 * \brief Gamepad high light handler
		 * \param value button value
		 * 
		 * Gamepad high light event handler. This function is called every time the gamepad changes the high light value.
		 */
		void gamepadHighLightCallback(int value)
		{
			if(value)
			{
				touch();
				command_->lights_high=!command_->lights_high;
				if(!event_.empty())
					event_(command_);
			}
		}
		
		/**
		 * \brief Gamepad medium light handler
		 * \param value button value
		 * 
		 * Gamepad medium light event handler. This function is called every time the gamepad changes the medium light value.
		 */
		void gamepadMediumLightCallback(int value)
		{
			if(value)
			{
				touch();
				command_->lights_medium=!command_->lights_medium;
				if(!event_.empty())
					event_(command_);
			}
		}
		
		/**
		 * \brief Gamepad warning light handler
		 * \param value button value
		 * 
		 * Gamepad warning light event handler. This function is called every time the gamepad changes the warning light value.
		 */
		void gamepadWarningCallback(int value)
		{
			if(value)
			{
				touch();
				command_->lights_warning=!command_->lights_warning;
				if(!event_.empty())
					event_(command_);
			}
		}
		
		/**
		 * \brief Gamepad auto direction handler
		 * \param value button value
		 * 
		 * Gamepad auto direction event handler. This function is called every time the gamepad changes the auto direction value.
		 */
		void gamepadAutoDirectionCallback(int value)
		{
			if(value)
			{
				touch();
				command_->auto_direction=!command_->auto_direction;
				if(!event_.empty())
					event_(command_);
			}
		}
		
		/**
		 * \brief Gamepad auto throttle handler
		 * \param value button value
		 * 
		 * Gamepad auto throttle event handler. This function is called every time the gamepad changes the auto throttle value.
		 */
		void gamepadAutoThrottleCallback(int value)
		{
			if(value)
			{
				touch();
				command_->auto_throttle=!command_->auto_throttle;
				if(!event_.empty())
					event_(command_);
			}
		}
		
		/**
		 * \brief Gamepad auto brake handler
		 * \param value button value
		 * 
		 * Gamepad auto brake event handler. This function is called every time the gamepad changes the auto brake value.
		 */
		void gamepadAutoBrakeCallback(int value)
		{
			if(value)
			{
				touch();
				command_->auto_brake=!command_->auto_brake;
				if(!event_.empty())
					event_(command_);
			}
		}
		
		/**
		 * \brief Gamepad auto clutch handler
		 * \param value button value
		 * 
		 * Gamepad auto clutch event handler. This function is called every time the gamepad changes the auto clutch value.
		 */
		void gamepadAutoClutchCallback(int value)
		{
			if(value)
			{
				touch();
				command_->auto_clutch=!command_->auto_clutch;
				if(!event_.empty())
					event_(command_);
			}
		}
		
		///Path to the gamepad image thumbnail
		std::string active_path_;
		///Pixbuf that hold the gamepad active thumbnail
		Glib::RefPtr<Gdk::Pixbuf> active_image_;
		///Pixbuf that hold the gamepad not active thumbnail
		Glib::RefPtr<Gdk::Pixbuf> not_active_image_;
		///Pixbuf that hold the gamepad auxiliary thumbnail
		Glib::RefPtr<Gdk::Pixbuf> auxiliary_image_;
		
		///Status of the gamepad connection
		bool active_;
		///Last activity time
		ros::Time ta_;
		///Fade duration
		ros::Duration fade_;
		///Gamepad communication class
		Gamepad gamepad_;
		///Outgoing manager command variable
		atlascar_base::ManagerCommandPtr command_;
};

/**
 * \brief Operation mode DrawingArea
 * 
 * This class graphical represents the current operation mode.
 * The operation mode can either be DIRECT or HIGH.
 * This class inherits the ExtendedDrawingArea class.
 * 
 */
class OperationMode: public ExtendedDrawingArea
{
	public:
		///Operation mode possible states
		enum EnumOperationMode {DIRECT, HIGH};
		
		/**
		 * \brief Class constructor
		 * \param side_length drawing area size
		 * \param operation_mode default operation model (DIRECT)
		 * 
		 */
		OperationMode(int side_length=100,EnumOperationMode operation_mode=DIRECT):
		side_length_(side_length),
		operation_mode_(operation_mode)
		{
			override_background_color(Gdk::RGBA("white"));
			set_size_request(side_length_*2,side_length_);
			add_events(Gdk::BUTTON_PRESS_MASK);
		}
		
		/**
		 * \brief Class destructor
		 * 
		 * Does nothing.
		 */
		virtual ~OperationMode()
		{}
		
		/**
		 * \brief Set the current operation mode
		 * \param mode the desired operation mode
		 */
		void operator=(EnumOperationMode mode)
		{
			operation_mode_ = mode;
			queue_draw();
		}
		
		/**
		 * \brief Check the current operation mode
		 * \param mode the operation mode to check against.
		 * \return True if their equal, false if not
		 */
		bool operator==(EnumOperationMode mode)
		{
			if(operation_mode_==mode)
				return true;
			else
				return false;
		}
		
		/**
		 * \brief Return the operation mode as int
		 * \return Current operation mode as int (DIRECT=1, HIGH=2)
		 */
		int getMode(void)
		{
			switch(operation_mode_)
			{
				case DIRECT:
					return 1;
				case HIGH:
					return 2;
			}
			
			return 1;
		}
		
	protected:
		
		/**
		 * \brief On draw signal handler
		 * \param cr cairo context
		 * 
		 * This function is called every time there is a redraw event on the DrawingArea.
		 */
		virtual bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr)
		{
			Gtk::Allocation allocation = get_allocation();
			const int width = allocation.get_width();
			const int height = allocation.get_height();
			
			switch(operation_mode_)
			{
				case DIRECT:
					cr->set_source_rgb(0,0,0);
					cr->set_line_width(1);
					cr->save();
					
					cr->move_to(0.12*width,0.20*height);
					cr->line_to(0.88*width,0.20*height);
					cr->line_to(0.88*width,0.60*height);
					cr->line_to(0.12*width,0.60*height);
					cr->close_path();
					cr->set_source_rgba(0,0,0,0.7);
					cr->fill_preserve();
					cr->restore();
					cr->stroke();
					cr->set_source_rgb(1,1,1);
					
					drawText("Direct",cr,0.5*width,0.4*height,12);
					cr->set_source_rgba(0,0,0,0.5);
					drawText("High",cr,0.5*width,0.7*height,6);
					break;
					
				case HIGH:
					
					cr->set_source_rgb(0,0,0);
					cr->set_line_width(1);
					cr->save();
					
					cr->move_to(0.12*width,0.40*height);
					cr->line_to(0.88*width,0.40*height);
					cr->line_to(0.88*width,0.80*height);
					cr->line_to(0.12*width,0.80*height);
					cr->close_path();
					cr->set_source_rgba(0,0,0,0.7);
					cr->fill_preserve();
					cr->restore();
					cr->stroke();
					cr->set_source_rgb(1,1,1);
					
					drawText("High",cr,0.5*width,0.6*height,12);
					cr->set_source_rgba(0,0,0,0.5);
					drawText("Direct",cr,0.5*width,0.3*height,6);
					
					break;
			}
			
			return true;
		}
		
		///Widget size
		int side_length_;
		///Current operation mode
		EnumOperationMode operation_mode_;
};

/**
 * \brief Generic pedal drawing area
 * 
 * This class represent the position of a generic pedal using a fill bar.
 * This class inherits the ExtendedDrawingArea class.
 */
class PedalDrawingArea: public ExtendedDrawingArea
{
	public:
		
		/**
		 * \brief Class constructor
		 * \param title title of the pedal
		 * \param min_value minimum value of the pedal
		 * \param max_value maximum value of the pedal
		 * 
		 * Initializes variables, widget size and background color.
		 */
		PedalDrawingArea(std::string title=std::string("NOT_SET"),double min_value=0,double max_value=100):
		title_(title),
		zoom_(1),
		min_value_(min_value),
		max_value_(max_value)
		{
			override_background_color(Gdk::RGBA("white"));
			set_size_request(zoom_*50,zoom_*150);
			value_=min_value_;
		}
		
		/**
		 * \brief Class destructor
		 * 
		 * Does nothing.
		 */
		virtual ~PedalDrawingArea()
		{}
		
		/**
		 * \brief Set the current pedal value
		 * \param value the new pedal value
		 * 
		 */
		void setValue(double value)
		{
			if(value>max_value_)
				value=max_value_;
			
			if(value<min_value_)
				value=min_value_;
			
			if(value != value_)
				queue_draw();
			
			value_=value;
		}
		
	protected:
		
		//Override default signal handler:
		virtual bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr)
		{
			Gtk::Allocation allocation = get_allocation();
			const int width = allocation.get_width();
			const int height = allocation.get_height();

			Cairo::Matrix transformation = Cairo::rotation_matrix(-pi/2);
			
			cr->transform(transformation);
			drawText(title_,cr,-height/2.,0.05*height,8);
			
			transformation.invert();
			cr->transform(transformation);
			
			cr->move_to(.3*width-0.05*width,.05*height-0.05*width);
			cr->line_to(.3*width-0.05*width,.95*height+0.05*width);
			cr->line_to(.6*width+0.05*width,.95*height+0.05*width);
			cr->line_to(.9*width+0.05*width,.05*height-0.05*width);
			cr->close_path();
			cr->set_source_rgb(.4,.4,.4);
			cr->fill();
			
			cr->set_source_rgb(0,0,0);
			cr->set_line_width(2);
			cr->move_to(.3*width,.05*height);
			cr->line_to(.3*width,.95*height);
			cr->line_to(.6*width,.95*height);
			cr->line_to(.9*width,.05*height);
			cr->close_path();
			cr->stroke();
			
			cr->move_to(.3*width,.05*height);
			cr->line_to(.3*width,.95*height);
			cr->line_to(.6*width,.95*height);
			cr->line_to(.9*width,.05*height);
			cr->close_path();
			cr->set_source_rgb(.8,.8,.8);
			cr->fill();
			
			Cairo::RefPtr< Cairo::LinearGradient > gradient_ptr = Cairo::LinearGradient::create(0.5*width,.05*height,0.5*width,0.95*height);
			
			//Set grandient colors
			gradient_ptr->add_color_stop_rgb(0,1,0,0);
			gradient_ptr->add_color_stop_rgb(1,1.0,0.6,0);
			
			//Draw the bar
			double racio = ((value_-min_value_)/(max_value_-min_value_));
			double top_value = -racio*.9*height + 0.95*height;
			double x_value = racio*0.3*width+.6*width;
			
			cr->set_source(gradient_ptr);
			cr->move_to(.3*width,.95*height);
			cr->line_to(.6*width,.95*height);
			cr->line_to(x_value,top_value);
			cr->line_to(0.3*width,top_value);
			cr->close_path();
			cr->fill();
			
			boost::format fmt("%3.1f");
			fmt % value_;
			
			cr->set_source_rgb(0,0,0);
			drawText(fmt.str(),cr,width/2.+0.08*width,0.1*height,8);
			
			return true;
		}
		
		std::string title_;
		double zoom_;
		double value_;
		double min_value_;
		double max_value_;
};

class SwitchButtonDrawingArea: public ExtendedDrawingArea
{
	public:
		SwitchButtonDrawingArea(int side_length=50,std::string title=std::string("NOT_SET")):
		active_(false),
		title_text_(title),
		active_text_(std::string("AUTO")),
		not_active_text_(std::string("MAN")),
		side_length_(side_length)
		{
			override_background_color(Gdk::RGBA("white"));
			set_size_request(0.75*side_length_,side_length_);
		}
		
		virtual ~SwitchButtonDrawingArea()
		{}
		
		void setStatus(bool active)
		{
			if(active_ != active)
				queue_draw();
			active_=active;
		}
		
	protected:
		
		//Override default signal handler:
		virtual bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr)
		{
			Gtk::Allocation allocation = get_allocation();
			const int width = allocation.get_width();
			const int height = allocation.get_height();

			drawText(title_text_,cr,width/2.,0.1*height,7);
			
			if(active_)
				cr->set_source_rgb(1,0,0);
			else
				cr->set_source_rgb(0,0,0);
			
			drawText(active_text_,cr,width/2.,0.22*height,6);
			
			if(!active_)
				cr->set_source_rgb(0,1,0);
			else
				cr->set_source_rgb(0,0,0);
			
			drawText(not_active_text_,cr,width/2.,0.90*height,6);
			
			double center = 0.55*height;
			double length = .25*height;
			
			// Create the linear gradient diagonal
			Cairo::RefPtr< Cairo::LinearGradient > gradient_ptr = Cairo::LinearGradient::create(0,0,width,height);
			Cairo::RefPtr< Cairo::LinearGradient > gradient_t_ptr = Cairo::LinearGradient::create(0,center-0.06*height,width,center);
			Cairo::RefPtr< Cairo::LinearGradient > gradient_b_ptr = Cairo::LinearGradient::create(0,center+0.06*height,width,center);
    
			// Set grandient colors
			gradient_ptr->add_color_stop_rgb(0,0.7,0.7,0.7);
			gradient_ptr->add_color_stop_rgb(1,0.5,0.5,0.5);
	
			gradient_t_ptr->add_color_stop_rgb(0.44,0.7,0.7,0.7);
			gradient_t_ptr->add_color_stop_rgb(0.49,0.1,0.1,0.1);
			
			gradient_b_ptr->add_color_stop_rgb(0.44,0.7,0.7,0.7);
			gradient_b_ptr->add_color_stop_rgb(0.49,0.1,0.1,0.1);
			
			cr->set_line_cap(Cairo::LINE_CAP_ROUND);
			cr->set_line_join(Cairo::LINE_JOIN_ROUND);
			
			cr->save();
			
			cr->move_to(.5*width,.5*height);
			cr->arc(.5*width,center,.20*side_length_,0,2*pi);
			cr->set_source_rgb(0.1,0.1,0.1);
			cr->fill();
			
			cr->set_source(gradient_ptr);
			cr->arc(.5*width,center,.19*side_length_,0,2*pi);
			cr->fill();
			
			cr->arc(.5*width,center,.08*side_length_,0,2*pi);
			cr->set_source_rgb(0.4,0.4,0.4);
			cr->fill();
			
			cr->set_line_width(0.01*side_length_);
			cr->set_source_rgb(0,0,0);
			cr->arc(.5*width,center,.08*side_length_,0,2*pi);
			cr->stroke();
			
			cr->restore();

			if(active_)
			{
				cr->set_line_width(0.11*side_length_);
				cr->set_source_rgb(0,0,0);
				cr->move_to(.5*width,center);
				cr->line_to(0.48*width,center-length);
				cr->stroke();
				cr->move_to(.5*width,center);
				cr->line_to(0.52*width,center-length);
				cr->stroke();
				
				cr->set_source(gradient_b_ptr);
				cr->set_line_width(0.1*side_length_);
				cr->move_to(.5*width,center);
				cr->line_to(0.48*width,center-length);
				cr->stroke();
				cr->move_to(.5*width,center);
				cr->line_to(0.52*width,center-length);
				cr->stroke();	
			}else
			{
				cr->set_line_width(0.11*side_length_);
				cr->set_source_rgb(0,0,0);
				cr->move_to(.5*width,center);
				cr->line_to(0.48*width,center+length);
				cr->stroke();
				cr->move_to(.5*width,center);
				cr->line_to(0.52*width,center+length);
				cr->stroke();
				
				cr->set_source(gradient_t_ptr);
				cr->set_line_width(0.1*side_length_);
				cr->move_to(.5*width,center);
				cr->line_to(0.48*width,center+length);
				cr->stroke();
				cr->move_to(.5*width,center);
				cr->line_to(0.52*width,center+length);
				cr->stroke();
			}
			
			return true;
		}
		
		bool active_;
		std::string title_text_;
		std::string active_text_;
		std::string not_active_text_;
		int side_length_;
};

class WarningDrawingArea: public Gtk::DrawingArea
{
	public:
		WarningDrawingArea(int side_length=50):
		active_(false),
		side_length_(side_length),
		blink_on_(0.5),
		blink_off_(0.2)
		{
			override_background_color(Gdk::RGBA("white"));
			
			set_size_request(side_length_,side_length_);
		}
		
		virtual ~WarningDrawingArea()
		{}
		
		void setStatus(bool active)
		{
			if(!active_ && active)
			{
				ta_=ros::Time::now();
				blink_state_=true;
				queue_draw();
			}else if(active_ && !active)
				queue_draw();
				
				
			
			active_=active;
		}
			
		bool stateManager()
		{
			if(active_)
			{
				if(blink_state_)
				{
					if( ros::Time::now()-ta_ > blink_on_)
					{
						blink_state_=false;
						ta_=ros::Time::now();
						queue_draw();
						return true;
					}
				}else
				{
					if( ros::Time::now()-ta_ > blink_off_)
					{
						blink_state_=true;
						ta_=ros::Time::now();
						queue_draw();
						return true;
					}
				}
			}
			
			return false;
		}
	
	protected:
		
		//Override default signal handler:
		virtual bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr)
		{
			Gtk::Allocation allocation = get_allocation();
			const int width = allocation.get_width();
			const int height = allocation.get_height();

			cr->scale(width,height);
			
			cr->set_line_cap(Cairo::LINE_CAP_ROUND);
			cr->set_line_join(Cairo::LINE_JOIN_ROUND);
			
			if(active_)
			{
				if(blink_state_)//on
				{
					cr->set_source_rgba(1.0,0.0,0.0,0.5);
					
					cr->set_line_width(0.15);
					
					cr->move_to(.5,.1);
					cr->line_to(.9,.9);
					cr->line_to(.1,.9);
					cr->close_path();
					cr->stroke();
					
					cr->set_source_rgb(1.0,0.0,0.0);
				}else
					cr->set_source_rgba(1.0,0.0,0.0,0.7);
			}else
				cr->set_source_rgba(0.6,0.6,0.6,1.0);
			
			cr->set_line_width(0.1);
			
			cr->move_to(.5,.1);
			cr->line_to(.9,.9);
			cr->line_to(.1,.9);
			cr->close_path();
			cr->stroke();
			
			if(active_)
				cr->set_source_rgb(0,0,0);
			else
				cr->set_source_rgb(0.6,0.6,0.6);
			
			cr->move_to(0.48,0.4);
			cr->line_to(0.5,0.60);
			cr->stroke();
			
			cr->move_to(0.52,0.4);
			cr->line_to(0.5,0.60);
			cr->stroke();
			
			cr->move_to(0.50,0.395);
			cr->line_to(0.5,0.60);
			cr->stroke();
						
			cr->set_line_width(0.05);
			
			cr->arc(0.5,0.75,0.07,0,2*pi);
			cr->fill();
			
			return true;
		}
		
		bool active_;
		bool blink_state_;
		int side_length_;
		ros::Duration blink_on_;
		ros::Duration blink_off_;
		ros::Time ta_;
		
};

class IgnitionDrawingArea: public Gtk::DrawingArea
{
	public:
		IgnitionDrawingArea(int side_length=50):
		active_(false),
		side_length_(side_length)
		{
			override_background_color(Gdk::RGBA("white"));
			
			set_size_request(side_length_*0.7,side_length_);
		}
		
		virtual ~IgnitionDrawingArea()
		{}
		
		void setStatus(bool active)
		{
			if(active_!=active)
				queue_draw();
			
			active_=active;
		}
		
	protected:
		
		//Override default signal handler:
		virtual bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr)
		{
			Gtk::Allocation allocation = get_allocation();
			const int width = allocation.get_width();
			const int height = allocation.get_height();

			
			if(active_)
				cr->set_source_rgb(1, 0, 0.0);
			else
				cr->set_source_rgb(0.0, 0.0, 0.0);
			
			cr->scale(width/0.7,height);
			
			cr->set_line_width(0.01);
			cr->set_line_cap(Cairo::LINE_CAP_ROUND);
			
			cr->move_to(0.6,0.1);
			cr->line_to(0.3,0.15);
			cr->line_to(0.2,0.38);
			cr->line_to(0.35,0.33);
			cr->line_to(0.2,0.63);
			cr->line_to(0.3,0.60);
			cr->line_to(0.18,0.9);
			cr->line_to(0.45,0.55);
			cr->line_to(0.32,0.60);
			cr->line_to(0.55,0.28);
			cr->line_to(0.38,0.33);
			cr->line_to(0.6,0.1);
			cr->fill();		
			
			if(active_)
				cr->set_source_rgb(1.0, 0.85, 0.0);
			else
				cr->set_source_rgb(0.2, 0.2, 0.2);
			
			cr->scale(0.9,1);
			
			cr->move_to(0.6,0.1);
			cr->line_to(0.3,0.15);
			cr->line_to(0.2,0.38);
			cr->line_to(0.35,0.33);
			cr->line_to(0.2,0.63);
			cr->line_to(0.3,0.60);
			cr->line_to(0.18,0.9);
			cr->line_to(0.45,0.55);
			cr->line_to(0.32,0.60);
			cr->line_to(0.55,0.28);
			cr->line_to(0.38,0.33);
			cr->line_to(0.6,0.1);
			cr->fill();		
			
			return true;
		}
		
		bool active_;
		int side_length_;
		
};

class LightsDrawingArea: public ExtendedDrawingArea
{
	public:
		enum light_enum {MAXIMUM, MEDIUM};
		
		LightsDrawingArea(light_enum l=MAXIMUM,int side_length=50):
		light_(l),
		active_(false),
		side_length_(side_length)
		{
			override_background_color(Gdk::RGBA("white"));
			
			set_size_request(side_length_,side_length_);
		}
		
		virtual ~LightsDrawingArea()
		{}
		
		void setStatus(bool active)
		{
			if(active_!=active)
				queue_draw();
			
			active_=active;
		}
		
	protected:
		
		//Override default signal handler:
		virtual bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr)
		{
			Gtk::Allocation allocation = get_allocation();
			const int width = allocation.get_width();
			const int height = allocation.get_height();

			switch(light_)
			{
				case MAXIMUM:
					if(active_)
						cr->set_source_rgb(0, 0.2, 1);
					else
						cr->set_source_rgb(.2, .2, .2);
					
					break;
				case MEDIUM:
					if(active_)
						cr->set_source_rgb(.3, 1, 0);
					else
						cr->set_source_rgb(.2, .2, .2);
					
					break;
			}
			
			
			double x0=0.5, y0=0.75, // start point
			x1=0, y1=0.80,  // control point #1
			x2=0, y2=0.20,  // control point #2
			x3=0.5, y3=0.25;  // end point

			// scale to unit square (0 to 1 width and height)
			cr->scale(width, height);

			cr->set_line_width(0.08);
			cr->set_line_cap(Cairo::LINE_CAP_ROUND);
			
			// draw curve
			cr->move_to(x0, y0);
			cr->curve_to(x1, y1, x2, y2, x3, y3);
			
			cr->move_to(x0, y0);
			
			x1=.6;
			y1=.6;
			x2=.6;
			y2=.4;
			
			cr->curve_to(x1, y1, x2, y2, x3, y3);
			cr->stroke();
			
			switch(light_)
			{
				case MAXIMUM:
					cr->move_to(0.65,0.26);
					cr->line_to(0.95,0.26);
					
					cr->move_to(0.7,0.42);
					cr->line_to(0.95,0.42);
					
					cr->move_to(0.7,0.58);
					cr->line_to(0.95,0.58);
					
					cr->move_to(0.65,0.74);
					cr->line_to(0.95,0.74);
					break;
				case MEDIUM:
					cr->move_to(0.65,0.26);
					cr->line_to(0.95,0.26+0.1);
					
					cr->move_to(0.7,0.42+.01);
					cr->line_to(0.95,0.42+0.1);
					
					cr->move_to(0.7,0.58+.01);
					cr->line_to(0.95,0.58+0.1);
					
					cr->move_to(0.65,0.74);
					cr->line_to(0.95,0.74+0.1);
					break;
			}
			
			cr->stroke();
			
			return true;
		}
		
		light_enum light_;
		bool active_;
		int side_length_;
		
};

class SteeringWheelDrawingArea: public ExtendedDrawingArea
{
	public:
		
		SteeringWheelDrawingArea(int side_length=250):
		steering_wheel_(0),
		side_length_(side_length)
		{
			override_background_color(Gdk::RGBA("white"));
			set_size_request(side_length_, side_length_);
		}
		
		virtual ~SteeringWheelDrawingArea()
		{}
		
		void setSteeringWheel(double steering_wheel)
		{
			if(steering_wheel!=steering_wheel_)
				queue_draw();
			
			steering_wheel_=steering_wheel;
		}
		
	protected:
		
		//Override default signal handler:
		virtual bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr)
		{
			Gtk::Allocation allocation = get_allocation();
			const int width = allocation.get_width();
			const int height = allocation.get_height();

			cr->scale(width, height);
			
			Cairo::Matrix transformation = Cairo::identity_matrix();
			transformation.translate(0.5,0.5);
			transformation.rotate(-steering_wheel_);
			
			cr->transform(transformation);
			
			cr->set_line_width(0.05);
			cr->set_line_cap(Cairo::LINE_CAP_ROUND);
			
			cr->set_source_rgb(0.4, 0.4, 0.4);
			cr->arc(0,0,0.41,0,2*pi);
			cr->stroke();
			
			cr->set_source_rgb(0, 0, 0);
			cr->arc(0,0,0.4,0,2*pi);
			cr->stroke();
			
			cr->set_line_width(0.1);
			
			double x0=-0.36, y0=-0.10, // start point
			x1=-.2, y1=0.00,  // control point #1
			x2=0.2, y2=0.00,  // control point #2
			x3=0.36, y3=-0.10;  // end point

			// draw curve
			cr->move_to(x0, y0);
			cr->curve_to(x1, y1, x2, y2, x3, y3);
			cr->stroke();
			
			x0=-.35;
			y0=-.05;
			
			x1=-.2,
			y1=.0;
			
			x2=-.02;
			y2=.1;
			
			x3=-.1;
			y3=.36;
			
			cr->move_to(x0, y0);
			cr->curve_to(x1, y1, x2, y2, x3, y3);
			cr->stroke();
			
			x0=.35;
			y0=-.05;
			
			x1=.2,
			y1=.0;
			
			x2=.02;
			y2=.1;
			
			x3=.1;
			y3=.36;
			
			cr->move_to(x0, y0);
			cr->curve_to(x1, y1, x2, y2, x3, y3);
			cr->stroke();
			
			cr->set_line_width(0.15);
			cr->move_to(0,0.35);
			cr->line_to(0,0);
			cr->stroke();
			
			cr->set_line_width(0.2);
			cr->move_to(-0.1,-0.02);
			cr->line_to(0.1,-0.02);
			cr->stroke();
			
			cr->scale(1./width,1./height);
			
			transformation.invert();
			cr->transform(transformation);
			
			cr->set_source_rgb(1, 1, 1);
			
			boost::format fmt("%3.2f");
			fmt % steering_wheel_;
			
			drawText(fmt.str(),cr,0,0,13);
			drawText(std::string("rad"),cr,0,0.07*height,8);
			
			return true;
		}
		
		double steering_wheel_;
		int side_length_;
};

class GearDrawnigArea: public ExtendedDrawingArea
{
	public:
		enum gear_enum {REVERSE, NEUTRAL, G1, G2, G3, G4, G5};
		
		GearDrawnigArea(gear_enum g,int side_length=50):
		side_length_(side_length),
		gear_(g)
		{
			override_background_color(Gdk::RGBA("white"));
			
			set_size_request(side_length_,side_length_);
		}
		
		virtual ~GearDrawnigArea()
		{}
		
		void setGear(gear_enum gear)
		{
			if(gear!=gear_)
				queue_draw();
			
			gear_=gear;
		}
		
	protected:
		
		//Override default signal handler:
		virtual bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr)
		{
			cr->set_source_rgb(0, 0, 0.0);
			
			int x_offset=-2;
			switch(gear_)
			{
				case REVERSE:
					
					drawTextItalics(std::string("R"),cr,side_length_/2+x_offset,side_length_/2,30);
					break;
					
				case NEUTRAL:
					drawTextItalics(std::string("N"),cr,side_length_/2+x_offset,side_length_/2,30);
					break;
					
				case G1:
					drawTextItalics(std::string("1"),cr,side_length_/2+x_offset,side_length_/2,30);
					break;
					
				case G2:
					drawTextItalics(std::string("2"),cr,side_length_/2+x_offset,side_length_/2,30);
					break;
					
				case G3:
					drawTextItalics(std::string("3"),cr,side_length_/2+x_offset,side_length_/2,30);
					break;
					
				case G4:
					drawTextItalics(std::string("4"),cr,side_length_/2+x_offset,side_length_/2,30);
					break;
					
				case G5:
					drawTextItalics(std::string("5"),cr,side_length_/2+x_offset,side_length_/2,30);
					break;
			}
			
			
			int p01=0.1*side_length_;
			int p09=0.9*side_length_;
				
			cr->set_line_width(2.0);
			cr->set_line_join(Cairo::LINE_JOIN_BEVEL);

			cr->move_to(p01,p01);
			cr->line_to(p01,p09);
			cr->line_to(p09,p09);
			cr->line_to(p09,p01);
			cr->line_to(p01,p01);
			cr->line_to(p01,p09);
			
			cr->stroke();
			
			return true;
		}
		
		int side_length_;
		gear_enum gear_;
};

class TurnSignalDrawnigArea: public Gtk::DrawingArea
{
	public:
		enum orientation_enum {LEFT=1, RIGHT=2};
		
		TurnSignalDrawnigArea(orientation_enum o,int side_length=50):
		side_length_(side_length),
		orientation_(o)
		{
			active_=false;
			
			override_background_color(Gdk::RGBA("white"));
			
			set_size_request(side_length_,side_length_);
		}
		
		virtual ~TurnSignalDrawnigArea()
		{}
		
		void setStatus(bool active)
		{
			active_=active;
			queue_draw();
		}
		
	protected:
		
		//Override default signal handler:
		virtual bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr)
		{
			Gtk::Allocation allocation = get_allocation();
			const int width = allocation.get_width();
			const int height = allocation.get_height();

			cr->scale(width, height);
			
			cr->set_line_width(0.05);
			cr->set_line_cap(Cairo::LINE_CAP_ROUND);
			cr->set_line_join(Cairo::LINE_JOIN_ROUND);
			
			if(active_)
				cr->set_source_rgb(0.0, 1.0, 0.0);
			else
				cr->set_source_rgb(0.2, 0.2, 0.2);
						
			double p01=0.1;
			double p03=0.3;
			double p05=0.5;
			double p07=0.7;
			double p09=0.9;
			
			if(orientation_==LEFT)
			{
				cr->move_to(p01,p05);
				cr->line_to(p05,p01);
				cr->line_to(p05,p03);
				cr->line_to(p09,p03);
				cr->line_to(p09,p07);
				cr->line_to(p05,p07);
				cr->line_to(p05,p09);
				cr->line_to(p01,p05);
				
			}else if(orientation_==RIGHT)
			{
				cr->move_to(p09,p05);
				cr->line_to(p05,p01);
				cr->line_to(p05,p03);
				cr->line_to(p01,p03);
				cr->line_to(p01,p07);
				cr->line_to(p05,p07);
				cr->line_to(p05,p09);
				cr->line_to(p09,p05);
			}
			
			cr->fill();

			cr->set_line_width(0.13);
			
			if(active_)
				cr->set_source_rgba(0.0, 1.0, 0.0,0.3);
			else
				cr->set_source_rgba(0.2, 0.2, 0.2,0.0);
						
			if(orientation_==LEFT)
			{
				cr->move_to(p01,p05);
				cr->line_to(p05,p01);
				cr->line_to(p05,p03);
				cr->line_to(p09,p03);
				cr->line_to(p09,p07);
				cr->line_to(p05,p07);
				cr->line_to(p05,p09);
				cr->line_to(p01,p05);
				
			}else if(orientation_==RIGHT)
			{
				cr->move_to(p09,p05);
				cr->line_to(p05,p01);
				cr->line_to(p05,p03);
				cr->line_to(p01,p03);
				cr->line_to(p01,p07);
				cr->line_to(p05,p07);
				cr->line_to(p05,p09);
				cr->line_to(p09,p05);
			}
			
			cr->stroke();
			
			return true;
		}
		
		bool active_;
		int side_length_;
		
		orientation_enum orientation_;
};

class RpmsDrawingArea: public ExtendedDrawingArea
{
	public:
		RpmsDrawingArea(int side_length=250,double max_rpm=7000, double large_step=1000,double min_angle=145*pi/180., double max_angle=320*pi/180.):
		side_length_(side_length),
		max_rpm_(max_rpm),
		large_step_(large_step),
		min_angle_(min_angle),
		max_angle_(max_angle),
		rpm_value_(0)
		{
			override_background_color(Gdk::RGBA("white"));
			set_size_request(side_length_,side_length_);
		}
		
		virtual ~RpmsDrawingArea()
		{}
		
		void setRpm(double rpm)
		{
			if(rpm!=rpm_value_)
				queue_draw();
			
			if(rpm>max_rpm_)
				rpm=max_rpm_;
			if(rpm<0)
				rpm=0;
			
			rpm_value_=rpm;
		}

	protected:
		
		//Override default signal handler:
		virtual bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr)
		{
			Gtk::Allocation allocation = get_allocation();
			const int width = allocation.get_width();
			const int height = allocation.get_height();
			
			cr->scale(width,height);
			
			Vector2d pc(0.5,0.5);
			
			double pointer_length = 0.48;

			//Draw all the small arcs
			double in_length = 0.44;
			double out_length = 0.49;
			
			cr->set_source_rgb(0,0,0);
			cr->set_line_width(0.008);
			
			double alpha = 0.1;
			for(uint rpm=max_rpm_-3*large_step_;rpm<max_rpm_;rpm+=large_step_)
			{
				double thi = min_angle_+rpm*(max_angle_-min_angle_)/max_rpm_;
				thi+=2.0*pi/180.;
				double thf = min_angle_+(rpm+large_step_/2)*(max_angle_-min_angle_)/max_rpm_;
				thf-=1.0*pi/190.;
				
				Vector2d diri(cos(thi),sin(thi));
				diri.normalize();
				
				Vector2d dirf(cos(thf),sin(thf));
				dirf.normalize();
				
				cr->set_source_rgba(1,0,0,alpha);
				cr->arc(pc(0),pc(1),out_length,thi,thf);
				cr->fill();
				
				Vector2d p1 = pc+diri*out_length;
				Vector2d p2 = pc+dirf*out_length;
				
				cr->move_to(pc(0),pc(1));
				cr->line_to(p1(0),p1(1));
				cr->line_to(p2(0),p2(1));
				cr->close_path();
				cr->fill();
				
				cr->set_source_rgb(1,1,1);
				cr->arc(pc(0),pc(1),in_length,thi,thf);
				cr->fill();
				
				Vector2d p1c = pc+diri*in_length;
				Vector2d p2c = pc+dirf*in_length;
				
				cr->move_to(p1c(0),p1c(1));
				cr->line_to(pc(0),pc(1));
				cr->line_to(p2c(0),p2c(1));
				cr->fill_preserve();
				cr->stroke();
				
				cr->set_source_rgba(1,0,0,alpha+.225);
				
				thi = min_angle_+(rpm+large_step_/2)*(max_angle_-min_angle_)/max_rpm_;
				thi+=1.0*pi/180.;
				thf = min_angle_+(rpm+large_step_)*(max_angle_-min_angle_)/max_rpm_;
				thf-=2.0*pi/190.;
				
				Vector2d diri2(cos(thi),sin(thi));
				diri2.normalize();
				
				Vector2d dirf2(cos(thf),sin(thf));
				dirf2.normalize();
				
				cr->arc(pc(0),pc(1),out_length,thi,thf);
				cr->fill();
				
				Vector2d p12 = pc+diri2*out_length;
				Vector2d p22 = pc+dirf2*out_length;
				
				cr->move_to(pc(0),pc(1));
				cr->line_to(p12(0),p12(1));
				cr->line_to(p22(0),p22(1));
				cr->close_path();
				cr->fill();
				
				cr->set_source_rgb(1,1,1);
				cr->arc(pc(0),pc(1),in_length,thi,thf);
				cr->fill();
				
				Vector2d p12c = pc+diri2*in_length;
				Vector2d p22c = pc+dirf2*in_length;
				
				cr->move_to(p12c(0),p12c(1));
				cr->line_to(pc(0),pc(1));
				cr->line_to(p22c(0),p22c(1));
				cr->fill_preserve();
				cr->stroke();
				
				alpha+=.45;
			}
			
			cr->set_line_width(0.005);
			cr->arc(pc(0),pc(1),0.018,0,2*pi);
			cr->stroke_preserve();
			cr->set_source_rgb(0.4,0.4,0.4);
			cr->fill();
		
			
			for(uint rpm=0;rpm<=max_rpm_;rpm+=large_step_)
			{
				double th = min_angle_+rpm*(max_angle_-min_angle_)/max_rpm_;
				th+=-0.8*pi/180.;
				
				Vector2d dir1(cos(th),sin(th));
				dir1.normalize();
				
				Vector2d pin1 = pc+dir1*in_length;
				Vector2d pout1 = pc+dir1*out_length;
				
				cr->move_to(pin1(0),pin1(1));
				cr->line_to(pout1(0),pout1(1));
				cr->stroke();
				
				th+=2*0.8*pi/180.;
				
				Vector2d dir2(cos(th),sin(th));
				dir2.normalize();
				
				Vector2d pin2 = pc+dir2*in_length;
				Vector2d pout2 = pc+dir2*out_length;
				
				cr->move_to(pin2(0),pin2(1));
				cr->line_to(pout2(0),pout2(1));
				cr->stroke();
			}
			
			for(uint rpm=large_step_/2.;rpm<=max_rpm_;rpm+=large_step_)
			{
				double th = min_angle_+rpm*(max_angle_-min_angle_)/max_rpm_;
				
				Vector2d dir1(cos(th),sin(th));
				dir1.normalize();
				
				Vector2d pin1 = pc+dir1*in_length;
				Vector2d pout1 = pc+dir1*out_length;
				
				cr->move_to(pin1(0),pin1(1));
				cr->line_to(pout1(0),pout1(1));
				cr->stroke();
			}
			
			
			cr->scale(1./width,1./height);
			
			cr->set_source_rgb(0.0, 0.0, 0.0);
			
			drawTextItalics("x1000r/min",cr,width/2.,(1./4.)*height,8);
			
			int rpmi=rpm_value_;
			drawText(boost::lexical_cast<std::string>(rpmi),cr,width/2.,(3./4.)*height,12);
			
			Vector2d pcu(0.5*width,0.5*height);
			
			for(uint rpm=0;rpm<=max_rpm_;rpm+=large_step_)
			{
				double th = min_angle_+rpm*(max_angle_-min_angle_)/max_rpm_;
				
				Vector2d dir(cos(th),sin(th));
				dir.normalize();
				
				Vector2d pin = pcu+dir*0.39*side_length_;
				
				int rpmi = rpm/1000.;
				
				drawTextItalics(boost::lexical_cast<std::string>(rpmi),cr,pin(0),pin(1),11);
			}
			
			cr->scale(width,height);
			
			double theta = min_angle_+rpm_value_*(max_angle_-min_angle_)/max_rpm_;
			
			Vector2d v1(cos(theta),sin(theta));
			
			v1.normalize();
			
			Vector2d pend(pc(0)+v1(0)*pointer_length,pc(1)+v1(1)*pointer_length);
			
			cr->set_line_width(0.025);
			cr->set_line_cap(Cairo::LINE_CAP_ROUND);

			// draw red lines out from the center of the window
			cr->set_source_rgb(0.5, 0.0, 0.0);
			cr->move_to(pc(0), pc(1));
			cr->line_to(pend(0), pend(1));
			
			cr->stroke();
			
			pend=pc+v1*pointer_length;
			
			cr->set_line_width(0.015);
			cr->set_source_rgb(1.0, 0.0, 0.0);
			cr->move_to(pc(0), pc(1));
			cr->line_to(pend(0), pend(1));
			
			cr->stroke();
			
			return true;
		}
		
		
		int side_length_;
		double max_rpm_;
		double large_step_;
		double min_angle_;
		double max_angle_;
		
		double rpm_value_;
};

class VelocityDrawingArea: public ExtendedDrawingArea
{
	public:
		VelocityDrawingArea(int side_length=350,double max_velocity=140, double large_step=10,double min_angle=145*pi/180., double max_angle=350*pi/180.):
		side_length_(side_length),
		max_velocity_(max_velocity),
		large_step_(large_step),
		min_angle_(min_angle),
		max_angle_(max_angle),
		velocity_value_(0)
		{
			override_background_color(Gdk::RGBA("white"));
			set_size_request(side_length,side_length);
		}
		
		virtual ~VelocityDrawingArea()
		{}
		
		void setVelocity(double velocity)
		{
			if(velocity!=velocity_value_)
				queue_draw();
			
			if(velocity<0)
			{
				velocity=-velocity;
				negative_=true;
			}else
				negative_=false;
			
			velocity_value_=velocity;
		}

	protected:
		
		//Override default signal handler:
		virtual bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr)
		{
			Gtk::Allocation allocation = get_allocation();
			const int width = allocation.get_width();
			const int height = allocation.get_height();

			cr->scale(width,height);
			
			Vector2d pc(0.5,0.5);
			
			cr->set_line_width(0.005);
			cr->arc(pc(0),pc(1),0.018,0,2*pi);
			cr->stroke_preserve();
			cr->set_source_rgb(0.4,0.4,0.4);
			cr->fill();
			
			double pointer_length = 0.48;

			//Draw all the small arcs
			double in_length = 0.44;
			double out_length = 0.49;
			
			cr->set_source_rgb(0,0,0);
			cr->set_line_width(0.008);
			
			for(uint vel=0;vel<=max_velocity_;vel+=large_step_)
			{
				double th = min_angle_+vel*(max_angle_-min_angle_)/max_velocity_;
				th+=-0.8*pi/180.;
				
				Vector2d dir1(cos(th),sin(th));
				dir1.normalize();
				
				Vector2d pin1 = pc+dir1*in_length;
				Vector2d pout1 = pc+dir1*out_length;
				
				cr->move_to(pin1(0),pin1(1));
				cr->line_to(pout1(0),pout1(1));
				cr->stroke();
				
				th+=2*0.8*pi/180.;
				
				Vector2d dir2(cos(th),sin(th));
				dir2.normalize();
				
				Vector2d pin2 = pc+dir2*in_length;
				Vector2d pout2 = pc+dir2*out_length;
				
				cr->move_to(pin2(0),pin2(1));
				cr->line_to(pout2(0),pout2(1));
				cr->stroke();
			}
			
			double vel = max_velocity_ + large_step_/2.;
			for(uint i=0;i<3;i++)
			{
				double th = min_angle_+vel*(max_angle_-min_angle_)/max_velocity_;

				Vector2d dir1(cos(th),sin(th));
				dir1.normalize();
				
				Vector2d pin1 = pc+dir1*(in_length+out_length)/2.;
				
				cr->arc(pin1(0),pin1(1),0.008,0,2*pi);
				cr->fill();
				
				vel+=large_step_/2.;
			}
			
			for(uint vel=large_step_/2.;vel<=max_velocity_;vel+=large_step_)
			{
				double th = min_angle_+vel*(max_angle_-min_angle_)/max_velocity_;
				
				Vector2d dir1(cos(th),sin(th));
				dir1.normalize();
				
				Vector2d pin1 = pc+dir1*in_length;
				Vector2d pout1 = pc+dir1*out_length;
				
				cr->move_to(pin1(0),pin1(1));
				cr->line_to(pout1(0),pout1(1));
				cr->stroke();
			}
			
			cr->scale(1./width,1./height);
			
			cr->set_source_rgb(0.0, 0.0, 0.0);
			
			drawTextItalics("km/h",cr,width/2.,(1./4.)*height,8);
			
			int veli = velocity_value_*3.6;//In km/h
			
			if(negative_)
				drawText("-" + boost::lexical_cast<std::string>(veli),cr,width/2.,(3./4.)*height,12);
			else
				drawText(boost::lexical_cast<std::string>(veli),cr,width/2.,(3./4.)*height,12);
			
			Vector2d pcu(0.5*width,0.5*height);
			
			vel=0;
			for(;vel<=max_velocity_;vel+=large_step_)
			{
				double th = min_angle_+vel*(max_angle_-min_angle_)/max_velocity_;
				
				Vector2d dir(cos(th),sin(th));
				dir.normalize();
				
				Vector2d pin = pcu+dir*0.39*side_length_;
				
				drawTextItalics(boost::lexical_cast<std::string>(vel),cr,pin(0),pin(1),11);
			}
			
			vel+=large_step_*1.1;
			
			double th = min_angle_+vel*(max_angle_-min_angle_)/max_velocity_;
			Vector2d dir(cos(th),sin(th));
			Vector2d pin = pcu+dir*0.45*side_length_;
			
			drawTextItalicsNotBold(std::string("\u221E"),cr,pin(0),pin(1),28);
			
			
			cr->scale(width,height);
			
			velocity_value_=velocity_value_*3.6;//Convert to km/h
			
			double theta = min_angle_+velocity_value_*(max_angle_-min_angle_)/max_velocity_;
			
			Vector2d v1(cos(theta),sin(theta));
			
			v1.normalize();
			
			Vector2d pend(pc(0)+v1(0)*pointer_length,pc(1)+v1(1)*pointer_length);
			
			cr->set_line_width(0.025);
			cr->set_line_cap(Cairo::LINE_CAP_ROUND);

			// draw red lines out from the center of the window
			cr->set_source_rgb(0.5, 0.0, 0.0);
			cr->move_to(pc(0), pc(1));
			cr->line_to(pend(0), pend(1));
			
			cr->stroke();
			
			pend=pc+v1*pointer_length;
			
			cr->set_line_width(0.015);
			cr->set_source_rgb(1.0, 0.0, 0.0);
			cr->move_to(pc(0), pc(1));
			cr->line_to(pend(0), pend(1));
			
			cr->stroke();
			
			return true;
		}
		
		int side_length_;
		double max_velocity_;
		double large_step_;
		double min_angle_;
		double max_angle_;
		
		double velocity_value_;
		bool negative_;
};

class ManagerGui: public Gtk::Window
{
	public:
		ManagerGui(volatile sig_atomic_t* shutdown_request = NULL):
		top_box_(false,5),
		main_division_box_(false,5),
		rpms_container_box_(false,5),
		velocity_container_box_(false,5),
		right_container_box_(false,5),
		left_container_box_(false,5),
		turning_signals_box_(false,5),
		lights_box_(false,5),
		warnings_box_(false,5),
		auto_box_(false,5),
		pedal_box_(false,5),
		status_box_(false,5),
		gamepad_info_(),
		nh_("~"),
		left_turn_signal_(TurnSignalDrawnigArea::LEFT),
		right_turn_signal_(TurnSignalDrawnigArea::RIGHT),
		gear_drawing_area_(GearDrawnigArea::NEUTRAL),
		maximum_lights_drawing_area_(LightsDrawingArea::MAXIMUM),
		medium_lights_drawing_area_(LightsDrawingArea::MEDIUM),
		emergency_drawing_area_(150),
		auto_brake_(80,std::string("Brake")),
		auto_clutch_(80,std::string("Clutch")),
		auto_direction_(80,std::string("Direction")),
		auto_ignition_(80,std::string("Ignition")),
		auto_throttle_(80,std::string("Throttle")),
		brake_pedal_(std::string("Brake"),0,1),
		throttle_pedal_(std::string("Throttle"),0,1),
		clutch_pedal_(std::string("Clutch"),0,1),
		operation_mode_(50),
		shutdown_request_(shutdown_request)
		{
			//Set the title of the window
			set_title("Atlascar Manager GUI");
			set_icon_from_file(ros::package::getPath("atlascar_base")+"/images/logo.png");
			
			//Subscribe to the status message
			status_sub_ = nh_.subscribe("status", 1, &ManagerGui::statusCallback, this);
			//Advertise command messages
			command_pub_ = nh_.advertise<atlascar_base::ManagerCommand>("command", 1);
			
			//Get the gamepad device parameter
			nh_.param("gamepad_device",gamepad_info_.device_,std::string("Not found"));
			gamepad_info_.initializeGamepad();
			
			nh_.param("gamepad_priority",gamepad_priority_,2);
			nh_.param("gamepad_lifetime",gamepad_lifetime_,std::numeric_limits<double>::infinity());
			
			//Create interface
			add(top_box_);
			
			top_box_.pack_start(main_division_box_,false,false,0);
			top_box_.pack_start(separator_,false,false,0);
			top_box_.pack_start(status_box_,false,false,5);
			top_box_.set_halign(Gtk::ALIGN_CENTER);
			
			main_division_box_.pack_start(left_container_box_,false,false,5);
			main_division_box_.pack_start(right_container_box_,false,false,5);
			main_division_box_.set_halign(Gtk::ALIGN_CENTER);
			
			right_container_box_.pack_start(rpms_container_box_,false,false,5);
			right_container_box_.pack_start(velocity_container_box_,false,false,5);
			right_container_box_.pack_start(pressure_sensors_,false,false,5);
			
			right_container_box_.set_halign(Gtk::ALIGN_CENTER);
			
			rpms_container_box_.pack_start(rpms_drawing_area_,false,false,5);
			rpms_container_box_.set_halign(Gtk::ALIGN_CENTER);
			velocity_container_box_.pack_start(velocity_drawing_area_,false,false,5);
			velocity_container_box_.set_halign(Gtk::ALIGN_CENTER);
			
			left_container_box_.pack_start(turning_signals_box_,false,false,5);
			left_container_box_.pack_start(lights_box_,false,false,5);
			left_container_box_.pack_start(warnings_box_,false,false,5);
			left_container_box_.pack_start(auto_box_,false,false,5);
			left_container_box_.pack_start(pedal_box_,false,false,5);
			
			lights_box_.pack_start(steering_wheel_drawing_area_,false,false,5);
			lights_box_.set_halign(Gtk::ALIGN_CENTER);
			
			warnings_box_.pack_start(emergency_drawing_area_,false,false,5);
			warnings_box_.set_halign(Gtk::ALIGN_CENTER);
			
			turning_signals_box_.pack_start(left_turn_signal_,false,false,0);
			turning_signals_box_.pack_start(right_turn_signal_,false,false,0);
			
			turning_signals_box_.pack_start(maximum_lights_drawing_area_,false,false,0);
			turning_signals_box_.pack_start(medium_lights_drawing_area_,false,false,0);
			turning_signals_box_.pack_start(gear_drawing_area_,false,false,0);
			turning_signals_box_.pack_start(ignition_drawing_area_,false,false,0);
			turning_signals_box_.set_halign(Gtk::ALIGN_CENTER);
			
			auto_box_.pack_start(auto_brake_,false,false,0);
			auto_box_.pack_start(auto_clutch_,false,false,0);
			auto_box_.pack_start(auto_direction_,false,false,0);
			auto_box_.pack_start(auto_ignition_,false,false,0);
			auto_box_.pack_start(auto_throttle_,false,false,0);
			auto_box_.set_halign(Gtk::ALIGN_CENTER);
			
			pedal_box_.pack_start(brake_pedal_,false,false,0);
			pedal_box_.pack_start(throttle_pedal_,false,false,0);
			pedal_box_.pack_start(clutch_pedal_,false,false,0);
			pedal_box_.set_halign(Gtk::ALIGN_CENTER);
			
			status_box_.pack_start(gamepad_info_,false,false,5);
			status_box_.pack_start(operation_mode_,false,false,5);
			status_box_.set_halign(Gtk::ALIGN_CENTER);
			
			operation_mode_.signal_button_press_event().connect(sigc::mem_fun(*this,&ManagerGui::operationModeClickEvent));
			gamepad_info_.event_ = sigc::mem_fun<const atlascar_base::ManagerCommandPtr>(*this,&ManagerGui::gamepadEvent);

// 			sigc::slot<bool> slow_slot = sigc::bind(sigc::mem_fun(*this,&ManagerGui::slowTimeout),0);
			sigc::slot<bool> rapid_slot = sigc::bind(sigc::mem_fun(*this,&ManagerGui::rapidTimeout),0);
			sigc::slot<bool> ros_slot = sigc::bind(sigc::mem_fun(*this,&ManagerGui::rosTimeout),0);
			
			// This is where we connect the slot to the Glib::signal_timeout()
// 			slowTimeoutConnection = Glib::signal_timeout().connect(slow_slot,100);//10hz
			rapidTimeoutConnection = Glib::signal_timeout().connect(rapid_slot,10);//100hz
			rosTimeoutConnection = Glib::signal_timeout().connect(ros_slot,20);//50hz
			
			command_.reset(new atlascar_base::ManagerCommand);

			override_background_color(Gdk::RGBA("white"));
			top_box_.override_background_color(Gdk::RGBA("white"));
			
			show_all_children();
		}
		
		void selectControlMethodDialog()//Not used
		{
			///Dialog to ask the control method
			Gtk::MessageDialog dialog(*this, "Please select the control operation mode.",false, Gtk::MESSAGE_QUESTION,Gtk::BUTTONS_NONE);
			dialog.set_secondary_text("DIRECT mode sends raw command values to the controller while HIGH sends processed values.");
			
			dialog.add_button("DIRECT",OperationMode::DIRECT);
			dialog.add_button("HIGH",OperationMode::HIGH);
			
			//Run the dialog
			int result = dialog.run();
			
			//Process the result of the dialog
			switch(result)
			{
				case OperationMode::DIRECT:
					operation_mode_=OperationMode::DIRECT;
					break;
				case OperationMode::HIGH:
					operation_mode_=OperationMode::HIGH;
					break;
			}
		}
		
		virtual ~ManagerGui()
		{
		}
	
	protected:
		
		bool operationModeClickEvent(GdkEventButton *event)
		{
			if(operation_mode_==OperationMode::DIRECT)
				operation_mode_=OperationMode::HIGH;
			else
				operation_mode_=OperationMode::DIRECT;
			
			//Get the current operation mode
			command_->direct_control = operation_mode_.getMode();
			//Define the lifetime and priority of the message
			command_->lifetime=gamepad_lifetime_;
			command_->priority=gamepad_priority_;
			//Publish the message
			command_pub_.publish(command_);
			
			return true;
		}

		void gamepadEvent(const atlascar_base::ManagerCommandPtr command)
		{
			//Send command, copy contents from gamepad event
			*command_=*command;
			
			//Get the current operation mode
			command_->direct_control = operation_mode_.getMode();
			//Define the lifetime and priority of the message
			command_->lifetime=gamepad_lifetime_;
			command_->priority=gamepad_priority_;
			//Publish the message
			command_pub_.publish(command_);
		}
		
		bool rosTimeout(int timer_number)
		{
			//Handle any shutdown request
			if(shutdown_request_!=NULL)
				if(*shutdown_request_)
					Gtk::Main::quit();
			
			ros::spinOnce();
			return true;
		}
		
		bool rapidTimeout(int timer_number)
		{
			emergency_drawing_area_.stateManager();
			gamepad_info_.stateManager();

			return true;
		}
		
		bool slowTimeout(int timer_number)
		{			
			static double theta=0;
			static double incrt=1;
			
			if(theta<-3*pi)
				incrt=1;
			
			if(theta>3*pi)
				incrt=-1;
			
			theta+=incrt*0.1;
			
			steering_wheel_drawing_area_.setSteeringWheel(theta);
			
			static double rpms=0;
			static double incr=1;
			
			if(rpms>7000)
				incr=-1;
			
			if(rpms<0)
				incr=1;
			
			rpms+=incr*100;
			
			rpms_drawing_area_.setRpm(rpms);
			
			static double vel=0;
			static double incrv=1;
			
			if(vel>140)
				incrv=-1;;
			
			if(vel<-20)
				incrv=1;
			
			vel+=incrv*1;
			
			velocity_drawing_area_.setVelocity(vel);
			
			if(vel>10 && vel<20)
				left_turn_signal_.setStatus(1);
			else
				left_turn_signal_.setStatus(0);
			
			left_turn_signal_.queue_draw();
			
			if(vel>30 && vel<40)
				right_turn_signal_.setStatus(1);
			else
				right_turn_signal_.setStatus(0);
			
			if(vel<10)
				gear_drawing_area_.setGear(GearDrawnigArea::REVERSE);
			
			if(vel>10 && vel<30)
				gear_drawing_area_.setGear(GearDrawnigArea::NEUTRAL);
			
			if(vel>30 && vel<60)
				gear_drawing_area_.setGear(GearDrawnigArea::G1);
			
			if(vel>60 && vel<90)
				gear_drawing_area_.setGear(GearDrawnigArea::G2);
			
			if(vel>90 && vel<120)
				gear_drawing_area_.setGear(GearDrawnigArea::G3);
			
			if(vel>120 && vel<130)
				gear_drawing_area_.setGear(GearDrawnigArea::G4);
			
			if(vel>130 && vel<140)
				gear_drawing_area_.setGear(GearDrawnigArea::G5);
			
			if(vel>10 && vel<80)
				maximum_lights_drawing_area_.setStatus(true);
			else
				maximum_lights_drawing_area_.setStatus(false);
			
			if(vel>30 && vel<120)
				medium_lights_drawing_area_.setStatus(true);
			else
				medium_lights_drawing_area_.setStatus(false);
			
			if(vel>50)
				ignition_drawing_area_.setStatus(true);
			else
				ignition_drawing_area_.setStatus(false);
			
			if(vel>30 && vel<100)
				emergency_drawing_area_.setStatus(true);
			else
				emergency_drawing_area_.setStatus(false);
			
			if(vel>0 && vel<20)
				auto_brake_.setStatus(true);
			else
				auto_brake_.setStatus(false);
			
			if(vel>20 && vel<40)
				auto_clutch_.setStatus(true);
			else
				auto_clutch_.setStatus(false);
			
			if(vel>40 && vel<60)
				auto_direction_.setStatus(true);
			else
				auto_direction_.setStatus(false);
			
			if(vel>60 && vel<80)
				auto_ignition_.setStatus(true);
			else
				auto_ignition_.setStatus(false);
			
			if(vel>80 && vel<100)
				auto_throttle_.setStatus(true);
			else
				auto_throttle_.setStatus(false);
			
			pressure_sensors_.setThrottle(vel*100.);
			pressure_sensors_.setBrake(vel*12.);
			pressure_sensors_.setClutch(vel*153.3);
			
			brake_pedal_.setValue(vel/14.);
			throttle_pedal_.setValue(rpms/800.);
			clutch_pedal_.setValue(rpms/600.);
			
			return true;
		}
		
		void statusCallback(const atlascar_base::ManagerStatusPtr& status)
		{
			steering_wheel_drawing_area_.setSteeringWheel(status->steering_wheel);
			
			rpms_drawing_area_.setRpm(status->rpm);
			velocity_drawing_area_.setVelocity(status->velocity);

			left_turn_signal_.setStatus(status->lights_left);
			right_turn_signal_.setStatus(status->lights_right);
			
			switch(status->gear)
			{
				case -1:
					gear_drawing_area_.setGear(GearDrawnigArea::REVERSE);
					break;
					
				case 0:
					gear_drawing_area_.setGear(GearDrawnigArea::NEUTRAL);
					break;
					
				case 1:
					gear_drawing_area_.setGear(GearDrawnigArea::G1);
					break;

				case 2:
					gear_drawing_area_.setGear(GearDrawnigArea::G2);
					break;
					
				case 3:
					gear_drawing_area_.setGear(GearDrawnigArea::G3);
					break;
					
				case 4:
					gear_drawing_area_.setGear(GearDrawnigArea::G4);
					break;
					
				case 5:
					gear_drawing_area_.setGear(GearDrawnigArea::G5);
					break;
			}
			
			maximum_lights_drawing_area_.setStatus(status->lights_high);
			medium_lights_drawing_area_.setStatus(status->lights_medium);
			ignition_drawing_area_.setStatus(status->ignition);
			
			emergency_drawing_area_.setStatus(status->emergency);
			
			auto_brake_.setStatus(status->auto_brake);
			auto_clutch_.setStatus(status->auto_clutch);
			auto_direction_.setStatus(status->auto_direction);			
			auto_ignition_.setStatus(status->auto_ignition);
			auto_throttle_.setStatus(status->auto_throttle);
			
			brake_pedal_.setValue(status->brake);
			throttle_pedal_.setValue(status->throttle_pedal);
			clutch_pedal_.setValue(status->clutch);
			
			pressure_sensors_.setThrottle(status->throttle_pressure);
			pressure_sensors_.setBrake(status->brake_pressure);
			pressure_sensors_.setClutch(status->clutch_pressure);
		}
		
		//Child widgets:
		Gtk::VBox top_box_;
		Gtk::HBox main_division_box_;
		Gtk::HBox rpms_container_box_;
		Gtk::HBox velocity_container_box_;
		Gtk::VBox right_container_box_;
		Gtk::VBox left_container_box_;
		Gtk::HBox turning_signals_box_;
		Gtk::HBox lights_box_;
		Gtk::HBox warnings_box_;
		Gtk::HBox auto_box_;
		Gtk::HBox pedal_box_;
		Gtk::HBox status_box_;
		GamepadInfo gamepad_info_;
		Gtk::Separator separator_;
		ros::NodeHandle nh_;
		ros::Subscriber status_sub_;
		ros::Publisher command_pub_;
		
		sigc::connection slowTimeoutConnection;
		sigc::connection rapidTimeoutConnection;
		sigc::connection rosTimeoutConnection;
		
		RpmsDrawingArea rpms_drawing_area_;
		VelocityDrawingArea velocity_drawing_area_;
		TurnSignalDrawnigArea left_turn_signal_;
		TurnSignalDrawnigArea right_turn_signal_;
		GearDrawnigArea gear_drawing_area_;
		SteeringWheelDrawingArea steering_wheel_drawing_area_;
		LightsDrawingArea maximum_lights_drawing_area_;
		LightsDrawingArea medium_lights_drawing_area_;
		IgnitionDrawingArea ignition_drawing_area_;
		WarningDrawingArea emergency_drawing_area_;
		
		SwitchButtonDrawingArea auto_brake_;
		SwitchButtonDrawingArea auto_clutch_;
		SwitchButtonDrawingArea auto_direction_;
		SwitchButtonDrawingArea auto_ignition_;
		SwitchButtonDrawingArea auto_throttle_;
		
		PedalDrawingArea brake_pedal_;
		PedalDrawingArea throttle_pedal_;
		PedalDrawingArea clutch_pedal_;
		
		PressureSensors pressure_sensors_;
		
		OperationMode operation_mode_;
		
		atlascar_base::ManagerCommandPtr command_;
		int gamepad_priority_;
		double gamepad_lifetime_;
		
		sig_atomic_t volatile * shutdown_request_;
};

#endif
