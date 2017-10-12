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

#ifndef _DATABASE_TEST_OBJECT_H_
#define _DATABASE_TEST_OBJECT_H_

#include "database_interface/db_class.h"

namespace database_interface {

//! The C++ version of an object stored in a database
/*! Note that:
  - all data stored in the database is declared as DBField<...>
*/
class DatabaseTestObject : public DBClass
{
 public:
  //primary key, an integer
  DBField<int> pk_field_;
  //fields testing various conversions
  DBField<double> double_field_;
  DBField<std::string> string_field_;
  DBField< std::vector<std::string> > tags_field_;
  //binary field
  DBField< std::vector<char> > binary_field_;
  //field from a different table
  DBField<int> foreign_field_;

  //! Places all the fields in the fields_ vector and sets foreign keys and sequences
  /*! Note that:
    - the address of the id_ field is noted as primary key
    - the addresses of all the other fields are stored in fields_
    - all the needed foreign keys are inserted for referencing fields that live in different
      table than out primary key
    - the sequence name used by our primary key field is noted
   */
  void initFields()
  {
    //set the primary key
    primary_key_field_ = &pk_field_;
    //and the rest of the fields
    fields_.push_back(&double_field_);
    fields_.push_back(&string_field_);
    fields_.push_back(&tags_field_);
    fields_.push_back(&binary_field_);
    fields_.push_back(&foreign_field_);

    //foreign keys
    foreign_keys_.insert( std::pair<std::string, DBFieldBase*>("test_object_foreign", &pk_field_) );
    
    //sequences
    pk_field_.setSequenceName("pk_field_sequence");
  }

  //! Initializes permissions for the fields
  /*! Note that:
    - default behavior is set for which fields are to be read from / written to the database.
      That this can always be changed later for any particular instance of this class, if you 
      want to save or load any particular fields.
   */
  void initPermissions()
  {
    setAllFieldsReadFromDatabase(true);
    setAllFieldsWriteToDatabase(true);
    //the fields that are usually used:
    //primary key id_ only syncs from database; it has a sequence which is used by default on insertions
    pk_field_.setWriteToDatabase(false);
    //binary field does not sync by default
    binary_field_.setReadFromDatabase(false);
    binary_field_.setWriteToDatabase(false);
  }

  //! Constructs the fields, then calls initFields() followed by initPermissions()
  /*! Note that:
    - all fields are intialized with the type, owner (this), column name and table name
   */
 DatabaseTestObject()  : 
    pk_field_(DBFieldBase::TEXT, this, "pk_field", "test_object", true),
    double_field_(DBFieldBase::TEXT, this, "double_field", "test_object", true),
    string_field_(DBFieldBase::TEXT, this, "string_field", "test_object", true),
    tags_field_(DBFieldBase::TEXT, this, "tags_field", "test_object", true),
    binary_field_(DBFieldBase::BINARY, this, "binary_field", "test_object", true),
    foreign_field_(DBFieldBase::TEXT, this, "foreign_field", "test_object_foreign", true)

  {
    initFields();
    initPermissions();
  }

    //! Copy-constructs the fields based on the copied instance fields, then calls initFields()
    /*! The data itself in the fields, as well as the permissions, gets copied in the 
      field copy construction.
    */
 DatabaseTestObject(const DatabaseTestObject *other) : 
    pk_field_(this, &other->pk_field_),
    double_field_(this, &other->double_field_),
    string_field_(this, &other->string_field_),
    tags_field_(this, &other->tags_field_),
    binary_field_(this, &other->binary_field_),
    foreign_field_(this, &other->foreign_field_)
      {
	initFields();
	//no need to call initPermissions() since field permissions are copied over from other
      }


};

} //namespace

#endif
