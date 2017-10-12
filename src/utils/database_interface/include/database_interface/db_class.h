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

#ifndef _DB_CLASS_H_
#define _DB_CLASS_H_

#include "database_interface/db_field.h"

namespace database_interface {

//! The base class for all C++ classes that can be stored in the database
/*! A DBClass is a collection of fields. 

  One of the fields is marked as a primary key, which is the only unique identifier for this
  class. The primary key field is expected to reference the primary key column in the main table
  where this class is stored in the database. 

  All the other fields are expected to be stored either in the same table as the primary key, or
  in other tables that reference our primary key in a foreign key column.

  To use this class, any data members that you want to save / load from database must be 
  declared as DBFields. Then, IN THE CONSTRUCTOR, you must tell this interface which of them
  is the primary key, and store the rest in the fields_ vector. After this, all database
  functionality will be automagically done for you.

  Also in the contructor you can set which fields are read from / written to the database by
  default, and which should be only when you specifically ask for them

  For an example, see the DatabaseOriginalModel implementation.
 */
class DBClass
{
 private:
  //! Makes the class non-assignable
  /*! Prevents the use of instances of DBClass in std::vectors since deep copies are extremely hard
    because of the vector of pointers to DBFieldBase, which should have pointers to its own
    members. Use boost::ptr_vector instead for passing around containers of DBClass.
  */
  DBClass& operator = (const DBClass& rhs);

 protected:
  //! The address of the field that acts as a primary key 
  DBFieldBase* primary_key_field_;

  //! The addresses of all the other fields
  std::vector<DBFieldBase*> fields_;

  //! List of foreign keys in OTHER tables that reference our PRIMARY KEY
  /*! Foreign keys are expected to have the *same name* in both tables they join */
  std::map<std::string, DBFieldBase*> foreign_keys_;

 public:
  DBClass() : primary_key_field_(NULL) {}

  size_t getNumFields() const {return fields_.size();}

  DBFieldBase* getField(size_t i) {return fields_.at(i);}
  const DBFieldBase* getField(size_t i) const {return fields_.at(i);}

  DBFieldBase* getField(std::string name) 
  {
    if (primary_key_field_->getName() == name) return primary_key_field_;
    for (size_t t=0; t<fields_.size(); t++)
    {
      if (fields_[t]->getName() == name) return fields_[t];
    }
    return NULL;
  }
  const DBFieldBase* getField(std::string name) const {return getField(name);}

  DBFieldBase* getPrimaryKeyField() {return primary_key_field_;}
  const DBFieldBase* getPrimaryKeyField() const {return primary_key_field_;}

  //! Returns the name of the foreign key column in a given table that references our primary key
  /*! The list of foreign keys must be inserted here IN THE CONSTRUCTOR, if you have fields
    that live in other tables than the primary key.
  */
  bool getForeignKey(std::string table, const DBFieldBase* &key) const
  {
    std::map<std::string, DBFieldBase*>::const_iterator it;
    it = foreign_keys_.find(table);
    if (it==foreign_keys_.end()) return false;
    key = it->second;
    return true;
  }

  bool getForeignKey(std::string table, DBFieldBase* &key)
  {
    return getForeignKey(table, key);
  }

  void setAllFieldsReadFromDatabase(bool sync)
  {
    if (primary_key_field_) primary_key_field_->setReadFromDatabase(sync);
    for (size_t i=0; i<fields_.size(); i++)
    {
      fields_[i]->setReadFromDatabase(sync);
    }
  }

  void setAllFieldsWriteToDatabase(bool sync)
  {
    if (primary_key_field_) primary_key_field_->setWriteToDatabase(sync);
    for (size_t i=0; i<fields_.size(); i++)
    {
      fields_[i]->setWriteToDatabase(sync);
    }
  }

};

} //namespace database_interface

#endif
