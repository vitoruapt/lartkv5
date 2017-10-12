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
/**
 * \file
 * \brief Create Google Earth compatible paths from lat-long list of points
 */

#include <iostream>
#include <kml/dom.h>

#include <string>
#include <stdio.h>
#include <string.h>
#include <math.h>

using namespace std;

// libkml types are in the kmldom namespace
using kmldom::CoordinatesPtr;
using kmldom::KmlPtr;
using kmldom::KmlFactory;
using kmldom::PlacemarkPtr;
using kmldom::PointPtr;
using kmldom::DocumentPtr;
using kmldom::LineStringPtr;

int main(int argc,char**argv)
{
	
	if(argc>2)
	{
		if(strcmp(argv[2],"conv")==0)
		{
			double ox=atof(argv[3]);
			double oy=atof(argv[4]);
			double theta=atof(argv[5]);
			
			FILE*fp=fopen(argv[1],"r");
			if(!fp)
			{
				perror("Unable to open file");
				return 1;
			}
						
			char s[1024];
			double x,y;
			double nx,ny;

			while(fgets(s,1024,fp))
			{
				sscanf(s,"%lf %lf",&x,&y);
				
				nx=x*cos(theta)-y*sin(theta);
				ny=x*sin(theta)+y*cos(theta);
				
				nx+=ox;
				ny+=oy;
				
				printf("%f\t%f\n",nx,ny);
			}		
			return 0;
		}
	}
	
	if(argc==2)
	{
		//Create kml from file with lat, lon
		
		// Get the factory singleton to create KML elements.
		KmlFactory* factory = KmlFactory::GetFactory();
		DocumentPtr document = factory->CreateDocument();
		
		// Line
		PlacemarkPtr linePlacemark = factory->CreatePlacemark();
		linePlacemark->set_name("Line");
		
		CoordinatesPtr lineCoordinates = factory->CreateCoordinates();
		
		FILE*fp=fopen(argv[1],"r");
		if(!fp)
		{
			perror("Unable to open file");
			return 0;
		}
		
		char s[1024];
		double lat,lon;
		while(fgets(s,1024,fp))
		{
			sscanf(s,"%lf %lf",&lat,&lon);
			lineCoordinates->add_latlng(lat,lon);
		}
		
		LineStringPtr lineString = factory->CreateLineString();
		lineString->set_coordinates(lineCoordinates);
		lineString->set_extrude(false);
		lineString->set_altitudemode(kmldom::ALTITUDEMODE_RELATIVETOGROUND);
		lineString->set_tessellate(false);
		
		linePlacemark->set_geometry(lineString);		
		document->add_feature(linePlacemark);
		
		// Create <kml> and give it <Placemark>.
		KmlPtr kml = factory->CreateKml();
		
		kml->set_feature(document);
		
		// Serialize to XML
		std::string xml = kmldom::SerializePretty(kml);
		
		// Print to stdout
		std::cout << xml;
		
		return 0;
	}
	
	cout<<"Malformed command"<<endl;
	return 0;
}
