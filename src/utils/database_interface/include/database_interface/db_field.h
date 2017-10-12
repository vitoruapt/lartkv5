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

#ifndef _DB_FIELD_H_
#define _DB_FIELD_H_

#include <stdio.h>

#include <vector>
#include <map>
#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>

#include <boost/format.hpp>

//for memcpy
#include <cstring>

namespace database_interface {

class DBClass;

//! The base class for a field of a class stored in the database, corresponding to a column in a table
/*! A class stored in the database (an instance of DBClass) must store all of its database
  data as instances of a DBField. A DBField (see the DBField class documentation) can store any
  type of data, as long as it can be converted to/from string or binary, so that it can be
  retrieved / stored into the database.

  For now, all SQL data types except bytea are retreived / stored via text format. That might
  change in the future. But for now, the requirement for a C++ data type so that it can be used
  as a DBField is that is can be serialized to / from string. The only exception is
  std::vector<char> meant to be used in binary form.

  This base class provides just the interface for storing / retrieving fields. This is all the 
  database interface needs to use this field.
 */
class DBFieldBase
{
 private:
  //! Makes the class non-assignable, since deep copies are hard because of the owner_ field. 
  /*! Use the assignment operator of the DBClass instead, which reasons about this.*/
  const DBFieldBase& operator = (const DBFieldBase &rhs);
 public:
  /*! A field marked as TEXT will be serialized to / from string by calling the toString and
    fromString functions. A field marked as BINARY will be serialized to / from binary
    by calling the toBinary and fromBinary functions.
   */
  enum Type{TEXT, BINARY};
 protected:
  //! The type of this field, either TEXT or BINARY
  Type type_;
  //! The class that this field is part of
  DBClass* owner_;
  //! Marks if this field is allowed to be used to modify the database
  /*! If false, supercedes write_to_database_. Can only be set at construction time.
    Use this for retrieving information from other tables, but information that you
    are not allowed to modify through an instance of this class. */
  bool write_permission_;
  //! Marks if this field should be read from the database when its owner is read
  bool read_from_database_;
  //! Marks if this field should be saved to the database when its owner is saved
  bool write_to_database_;
  //! The name of the column that this field is stored in in the database
  std::string name_;
  //! The name of the table that this field is stored in the database
  std::string table_name_;
  //! Optional: the name of a database sequence that is used as a default value for this field
  std::string sequence_name_;

  //! Copy constructor is protected, it should only be called by derived classes which also copy the data
 DBFieldBase(DBClass *owner, const DBFieldBase *other) : 
  type_(other->type_), owner_(owner),
    write_permission_(other->write_permission_), read_from_database_(other->read_from_database_), 
    write_to_database_(other->write_to_database_), 
    name_(other->name_), table_name_(other->table_name_) {}

 public:
 DBFieldBase(Type type, DBClass* owner, std::string name, std::string table_name, bool write_permission) : 
    type_(type), owner_(owner),
    write_permission_(write_permission), read_from_database_(true), write_to_database_(true), 
    name_(name), table_name_(table_name) {}

  Type getType() const {return type_;}

  //! Sets the value of this field from a text string
  virtual bool fromString(const std::string &str) = 0;
  //! Gets the value of this field as a text string
  virtual bool toString(std::string &str) const = 0;

  //! Sets the value of this field from a binary chunk of data
  virtual bool fromBinary(const char* binary, size_t length) = 0;
  //! Gets a pointer to the value of this field as a binary chunk of data
  /*! Field MAINTAINS OWNERSHIP of the binary data; caller is not allowed to modify / delete it.
  */
  virtual bool toBinary(const char* &binary, size_t &length) const = 0;

  DBClass* getOwner(){return owner_;}
  const DBClass* getOwner() const {return owner_;}

  bool getReadFromDatabase() const {return read_from_database_;}
  void setReadFromDatabase(bool sync) {read_from_database_ = sync;}

  bool getWriteToDatabase() const {return write_to_database_;}
  void setWriteToDatabase(bool sync) {write_to_database_ = sync;}

  bool getWritePermission() const {return write_permission_;}

  void setReadWrite(bool sync) 
  {
    setReadFromDatabase(sync);
    setWriteToDatabase(sync);
  }

  std::string getName() const {return name_;}
  std::string getTableName() const {return table_name_;}
  std::string getSequenceName() const {return sequence_name_;}

  void setSequenceName(std::string seq) {sequence_name_ = seq;}
};

//! Streaming of a vector from a string in accordance to database formatting
template<class V>
std::istream& operator >> (std::istream &iss, std::vector<V> &vec)
{
  char c;
  iss >> c;
  if (iss.eof()) 
  {
    iss.clear();
    return iss;
  }
  if (iss.fail() || c != '{') 
  {
    iss.clear(std::ios::failbit);
    return iss;
  }
  
  while(1)
  {
    V val;
    iss >> val;
    if (iss.eof() || iss.fail()) 
    {
      iss.clear(std::ios::failbit);      
      return iss;
    }
    vec.push_back(val);
    
    iss >> c;
    if (iss.eof() || iss.fail()) 
    {
      iss.clear(std::ios::failbit);      
      return iss;
    }
    if (c == '}') break;
    if (c != ',') 
    {
      iss.clear(std::ios::failbit);      
      return iss;
    }
  }
  return iss;  
}

//! Streaming of a vector into a string in accordance to database formatting
template <class V>
std::ostream& operator << (std::ostream &str, const std::vector<V> &vec)
{
  str << "{";
  for (size_t i=0; i<vec.size(); i++)
  {
    if (i!=0) str << ",";
    str << vec[i];
    if (str.fail()) return str;
  }
  str << "}";
  return str;
}

// We need to use a trait class here. Templated functions are not
// flexible enough since they do not support partial specialization.
template<typename T>
struct DBStreamable
{
  //! Helper function for converting a string to a datatype through the STL istream operator >>
  static bool streamableFromString(T &data, const std::string &str)
  {
    std::istringstream iss(str);
    return !(iss >> data).fail();
  }

  //! Helper function for converting a datatype to a string through the STL ostream operator <<
  static bool streamableToString(const T &data, std::string &str)
  {
    std::ostringstream oss;
    // Only use 5 digits after the comma. Hopefully this solves an
    // issue with reading data from the database and comparing against
    // it later.
    oss << std::setprecision(30) << data;
    if (oss.fail()) return false;
    str = oss.str();
    return true;
  }
};

template<>
struct DBStreamable<double>
{
  //! Helper function for converting a string to a datatype through the STL istream operator >>
  static bool streamableFromString(double &data, const std::string &str)
  {
    // We need to restrict to only 5 digits after point. Otherwise
    // comparison is hell. Serialization uses only 5 digits.
    size_t point_pos = str.find('.');
    std::istringstream iss(str.substr(0, point_pos == std::string::npos ? point_pos : point_pos + 6));
    return !(iss >> data).fail();
  }

  //! Helper function for converting a datatype to a string through the STL ostream operator <<
  static bool streamableToString(double data, std::string &str)
  {
    std::ostringstream oss;
    // Only use 5 digits after the comma. Hopefully this solves an
    // issue with reading data from the database and comparing against
    // it later.
    oss << boost::format("%.5f") % data;
    if (oss.fail()) return false;
    str = oss.str();
    return true;
  }
};

//! A DBFieldBase that also contains data and perform implicit conversion to and from string
/*! Default conversion to and from string is through the stream operators >> and <<. Any data
  type that defines those operators can be used inside this class.  Works well for most
  basic data types (int, double, etc).

  Default behavior for conversion to / from binary is to always fail. This will be
  overidden only in the specialized DBField< std::vector<char> >
 */
template <class T>
class DBFieldData : public DBFieldBase
{
 protected:
  T data_;

  //! Copy constructor also copies the data itself
  /*! Data is copied through a string or a binary chunk. This implies an additional copy
    (extra work and memory) but adds no constraints on the type T, which we already know
    can be converted either to string or to binary.
  */
 DBFieldData(DBClass *owner, const DBFieldData *other) : DBFieldBase(owner, other) 
    {}

  void copy(const DBFieldData<T> *other)
  {
    if (this->type_==DBFieldBase::TEXT)
    {
      //hmm.. these can technically fail. What then?
      std::string copy_str;
      if (!other->toString(copy_str) || !this->fromString(copy_str))
      {
	std::cerr << "ERROR: database field ASCII copy failed during copy constructor of field " 
		  << this->name_ 
		  << ". This is currently an unhandled error, and will probably have serious effects.\n";
      }
      //std::cerr << "Copying field " << this->name_ << ". String: " << copy_str << "\n";
    }
    else if (this->type_==DBFieldBase::BINARY)
    {
      //hmm.. these can technically fail. What then?
      const char *copy_bin; size_t length;
      if (!other->toBinary(copy_bin, length) || !this->fromBinary(copy_bin, length))
      {
	std::cerr << "ERROR: database field BINARY copy failed during copy constructor of field " 
		  << this->name_ 
		  << ". This is currently an unhandled error, and will probably have serious effects.\n";
      }
    }
  }

 public:
  DBFieldData(Type type, DBClass* owner, std::string name, std::string table_name, bool write_permission) : 
    DBFieldBase(type, owner, name, table_name, write_permission){}

  const T& get() const {return data_;}

  T& get() {return data_;}

  const T& data() const {return data_;}

  T& data() {return data_;}

  virtual bool fromString(const std::string &str)
  {
    return DBStreamable<T>::streamableFromString(this->data_, str);
  }

  virtual bool toString(std::string &str) const
  {
    return DBStreamable<T>::streamableToString(this->data_, str);
  }

  virtual bool fromBinary(const char* /*binary*/, size_t /*length*/) {return false;}
  virtual bool toBinary(const char* &/*binary*/, size_t &/*length*/) const {return false;}
};

//! The base class for a usable DBField.
/*! Provides the implicit conversion to and from string inherited from the DBFieldBase.
  In addition, can be specialized for different conversion types as desired.
 */
template <class T>
class DBField : public DBFieldData<T>
{
 public:
  DBField(DBFieldBase::Type type, DBClass *owner, std::string name, std::string table_name, bool write_permission) : 
    DBFieldData<T>(type, owner, name, table_name, write_permission) {}

 DBField(DBClass *owner, const DBField<T> *other) : DBFieldData<T>(owner, other) 
 {
   this->copy(other);
 }

};

//! Specialized version for the bool data type, converts to / from string as SQL expects it.
template <>
class DBField<bool> : public DBFieldData<bool>
{
 public:
  DBField(Type type, DBClass *owner, std::string name, std::string table_name, bool write_permission) : 
    DBFieldData<bool>(type, owner, name, table_name, write_permission) {}

  DBField(DBClass *owner, const DBField<bool> *other) : DBFieldData<bool>(owner, other) 
  {
    this->copy(other);
  }

  virtual bool fromString(const std::string &str) 
  {
    if (str=="true" || str=="t" || str == "True" || str == "TRUE") this->data_ = true;
    else if (str=="false" || str=="f" || str == "False" || str == "FALSE") this->data_ = false;
    else return false;
    return true;
  }

  virtual bool toString(std::string &str) const 
  {
    if (this->data_) str="true";
    else str = "false";
    return true;
  }
};


//! Specialized version for std::vector<char>, the ONLY datatype that provides binary conversion
template <>
class DBField< std::vector<char> > : public DBFieldData< std::vector<char> >
{
 public:
  DBField(Type type, DBClass *owner, std::string name, std::string table_name, bool write_permission) : 
    DBFieldData< std::vector<char> >(type, owner, name, table_name, write_permission) {}

 DBField(DBClass *owner, const DBField< std::vector<char> > *other) : DBFieldData< std::vector<char> >(owner, other) 
      {
	this->copy(other);
      }
  
  virtual bool fromBinary(const char* binary, size_t length) 
  {
    data_.resize(length);
    memcpy(&(data_[0]), binary, length);
    return true;
  }

  virtual bool toBinary(const char* &binary, size_t &length) const 
  {
    length = data_.size();
    if (!data_.empty())
    {
      binary = &(data_[0]);
    }
    return true;
  }
};

//! Specialized version for std::string data type, trivial conversion to/from string
template <>
class DBField<std::string> : public DBFieldData<std::string>
{
 public:

  DBField(Type type, DBClass *owner, std::string name, std::string table_name, bool write_permission) : 
    DBFieldData<std::string>(type, owner, name, table_name, write_permission) {}

  DBField(DBClass *owner, const DBField< std::string > *other) : DBFieldData< std::string >(owner, other) 
      {
	this->copy(other);
      }

  virtual bool fromString(const std::string &str) {data_ = str; return true;}
  virtual bool toString(std::string &str) const {str = data_; return true;}
};

//! Specialized version for std::vector<std::string>
template <>
class DBField< std::vector<std::string> > : public DBFieldData< std::vector<std::string> >
{
 public:
  DBField(Type type, DBClass *owner, std::string name, std::string table_name, bool write_permission) : 
    DBFieldData< std::vector<std::string> >(type, owner, name, table_name, write_permission) {}

 DBField(DBClass *owner, const DBField< std::vector<std::string> > *other) : 
    DBFieldData< std::vector<std::string> >(owner, other) 
      {
	this->copy(other);
      }

  virtual bool fromString(const std::string &str) 
  {
    if (str.empty()) return true;
    if (str.at(0) != '{') return false;

    size_t pos = 1;
    bool done = false;
    while (!done)
    {
      if (pos >= str.size()) return false;
      size_t new_pos = str.find(',',pos);
      if (new_pos == std::string::npos)
      {
	new_pos = str.find('}',pos);
	if (new_pos == std::string::npos) return false;
	done = true;
      }
      if (new_pos == pos) return false;
      this->data_.push_back(str.substr(pos, new_pos-pos));
      pos = new_pos + 1;
    }
    return true;
  }
};

} //namespace database_interface

#endif

