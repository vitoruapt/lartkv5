/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Author(s): Matei Ciocarlie

#include <algorithm>
#include <vector>
#include <boost/shared_ptr.hpp>

#include "database_interface/postgresql_database.h"

#include "database_interface/database_test_object.h"

#include <ros/ros.h>

//! A little test program for the model database
int main(int argc, char **argv)
{
  bool TEST_INSERTION = true;
  bool TEST_DELETION = true;
  size_t NUM_OBJECTS = 2;

  database_interface::PostgresqlDatabase database("wgs36", "5432", "willow", 
							   "willow", "database_test");
  if (!database.isConnected())
    {
      ROS_ERROR("Database failed to connect");
      return -1;
    }
  ROS_INFO("Database connected successfully");

  
  std::vector< boost::shared_ptr<database_interface::DatabaseTestObject> > objects;

  if (!database.getList(objects))
  {
    ROS_ERROR("Failed to get list of test objects");
    return -1;
  }

  ROS_INFO("Retrieved %zd objects", objects.size());
  if ( objects.size() != NUM_OBJECTS)
  {
    ROS_ERROR("Expected %d objects", (int)NUM_OBJECTS);
    return -1;
  }

  for (size_t i=0; i<objects.size(); i++)
  {
    ROS_INFO("Object id %d, double field %f, string field %s, foreign field %d",
	     objects[i]->pk_field_.get(),
	     objects[i]->double_field_.get(),
	     objects[i]->string_field_.get().c_str(),
	     objects[i]->foreign_field_.get());
    std::string tags("{");
    for (size_t t=0; t<objects[i]->tags_field_.get().size(); t++)
    {
      if (t!=0) tags+= ",";
      tags += objects[i]->tags_field_.get().at(t);
    }
    tags += "}";
    ROS_INFO("           Tags: %s", tags.c_str());
    
    if (!database.loadFromDatabase(&(objects[i]->binary_field_)) )
    {
      ROS_ERROR("Failed to load binary field for object %zd", i);
      return -1;
    }
    else
    {
      ROS_INFO("           Binary field: %zd bytes", objects[i]->binary_field_.get().size());
    }        
  }

  
  if (NUM_OBJECTS)
  {
    database_interface::DatabaseTestObject clone(objects[0].get());
    ROS_INFO("Clone id %d, double field %f, string field %s, foreign field %d",
	     clone.pk_field_.get(),
	     clone.double_field_.get(),
	     clone.string_field_.get().c_str(),
	     clone.foreign_field_.get());
    if ( clone.string_field_.get() != objects[0]->string_field_.get())
    {
      ROS_ERROR("Clone string_field not identical to original: %s", clone.string_field_.get().c_str());
      return -1;
    }
    if ( clone.binary_field_.get().size() != objects[0]->binary_field_.get().size() )
    {
      ROS_ERROR("Clone binary field size not identical to original: %d", (int)clone.binary_field_.get().size());
      return -1;
    }
    ROS_INFO("Cloning successful");

    std::string old_string = objects[0]->string_field_.get();

    std::string new_string("10011001100122");
    objects[0]->string_field_.fromString(new_string);
    if ( !database.saveToDatabase(&(objects[0]->string_field_)) )
    {
      ROS_ERROR("Failed to save field to database");
      return -1;
    }
    objects[0]->string_field_.get().assign("foo");
    if (!database.loadFromDatabase(&(objects[0]->string_field_)) )
    {
      ROS_ERROR("Failed to retrieve field from database");
      return -1;
    }
    std::string test_string;
    objects[0]->string_field_.toString(test_string);
    if (test_string != new_string)
    {
      ROS_ERROR("Retrieved field does not match: %s", test_string.c_str());
      return -1;
    }

    objects[0]->string_field_.get() = old_string;
    if ( !database.saveToDatabase(&(objects[0]->string_field_)) )
    {
      ROS_ERROR("Failed to revert field to old value");
      return -1;
    }

    ROS_INFO("Saving and retrieving fields successful");
  }

  
  if (TEST_INSERTION)
  {
    database_interface::DatabaseTestObject new_object;

    new_object.double_field_.get()=3.5;
    new_object.string_field_.get()="object3_string";
    new_object.tags_field_.get().push_back("object3_tag1");
    new_object.tags_field_.get().push_back("object3_tag2");
    new_object.tags_field_.get().push_back("object3_tag3");
    new_object.foreign_field_.get() = 300;

    if (!database.insertIntoDatabase(&new_object))
    {
      ROS_ERROR("Failed to insert new model in database");
    }
    else
    {
      ROS_INFO("New model inserted successfully");    
      
      if (TEST_DELETION)
      {
	if (!database.deleteFromDatabase(&new_object))
	{
	  ROS_ERROR("Failed to delete model from database");
	}
	else
	{
	  ROS_INFO("New model deleted successfully");
	}
      }      
    }
  }
  
  ROS_INFO("All tests passed");
  return 0;
}
