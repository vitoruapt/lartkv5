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

#ifndef _POSTGRESQL_DATABASE_H_
#define _POSTGRESQL_DATABASE_H_

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

//for ROS error messages
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include "database_interface/db_class.h"
#include "database_interface/db_filters.h"

//A bit of an involved way to forward declare PGconn, which is a typedef
struct pg_conn;
typedef struct pg_conn PGconn;

namespace database_interface {

class PostgresqlDatabaseConfig
{
private:
  std::string password_;
  std::string user_;
  std::string host_;
  std::string port_;
  std::string dbname_;

public:
  PostgresqlDatabaseConfig() {  }

  std::string getPassword() const { return password_; }
  std::string getUser() const { return user_; }
  std::string getHost() const { return host_; }
  std::string getPort() const { return port_; }
  std::string getDBname() const { return dbname_; }

  friend void operator>>(const YAML::Node &node, PostgresqlDatabaseConfig &options);
};

/*!
 *\brief Loads YAML doc into configuration params. Throws YAML::ParserException if keys missing.
 */
inline void operator>>(const YAML::Node& node, PostgresqlDatabaseConfig &options)
{
  node["password"] >> options.password_;
  node["user"] >> options.user_;
  node["host"] >> options.host_;
  node["port"] >> options.port_;
  node["dbname"] >> options.dbname_;
}

class PostgresqlDatabase
{
 protected:
  void pgMDBconstruct(std::string host, std::string port, std::string user,
                      std::string password, std::string dbname );

  //! The PostgreSQL database connection we are using
  PGconn* connection_;

  //! Helper class that acts like an auto ptr for a PGresult, with a little more cleanup
  class PGresultAutoPtr;

  // beginTransaction sets this flag. endTransaction clears it.
  bool in_transaction_;

  //! Gets the text value of a given variable
  bool getVariable(std::string name, std::string &value) const;
  
  //! Issues the "rollback" command to the database
  bool rollback();

  //! Isses the "begin" command to the database
  bool begin();

  //! Issues the "commit" command to the database
  bool commit();

  //! Retreives the list of objects of a certain type from the database
  template <class T>
    bool getList(std::vector< boost::shared_ptr<T> > &vec, const T& example, std::string where_clause) const;

  //! Helper function for getList, separates SQL from (templated) instantiation
  bool getListRawResult(const DBClass *example, std::vector<const DBFieldBase*> &fields, 
			std::vector<int> &column_ids, std::string where_clause,
			boost::shared_ptr<PGresultAutoPtr> &result, int &num_tuples) const;

  //! Helper function for getList, separates SQL from (templated) instantiation
  bool populateListEntry(DBClass *entry, boost::shared_ptr<PGresultAutoPtr> result, int row_num,
			 const std::vector<const DBFieldBase*> &fields,
			 const std::vector<int> &column_ids) const;

  //! Returns the 'currval' for the database sequence identified by name
  bool getSequence(std::string name, std::string &value);

  //! Helper function for inserting an instance into the database
  bool insertIntoTable(std::string table_name, const std::vector<const DBFieldBase*> &fields);

  //! Helper function that deletes a row from a table based on the value of the specified field 
  bool deleteFromTable(std::string table_name, const DBFieldBase *key_field);

 public:
  //! Attempts to connect to the specified database
  PostgresqlDatabase(std::string host, std::string port, std::string user,
		     std::string password, std::string dbname);

  //! Attempts to connect to the specified database
  PostgresqlDatabase(const PostgresqlDatabaseConfig &config);


  //! Closes the connection to the database
  ~PostgresqlDatabase();

  //! Returns true if the interface is connected to the database and ready to go
  bool isConnected() const;

  //------- general queries that should work regardless of the datatypes actually being used ------

  //------- retrieval without examples ------- 
  template <class T>
  bool getList(std::vector< boost::shared_ptr<T> > &vec) const
  {
    T example;
    return getList<T>(vec, example, "");
  }
  template <class T>
  bool getList(std::vector< boost::shared_ptr<T> > &vec, const FilterClause clause) const
  {
    T example;
    return getList<T>(vec, example, clause.clause_);
  }
  template <class T>
  bool getList(std::vector< boost::shared_ptr<T> > &vec, std::string where_clause) const
  {
    T example;
    return getList<T>(vec, example, where_clause);
  }

  //------- retrieval with examples ------- 
  template <class T>
  bool getList(std::vector< boost::shared_ptr<T> > &vec, const T &example) const
  {
    return getList<T>(vec, example, "");
  }
  template <class T>
  bool getList(std::vector< boost::shared_ptr<T> > &vec, const T &example, const FilterClause clause) const
  {
    return getList<T>(vec, example, clause.clause_);
  }

  //! Counts the number of instances of a certain type in the database
  bool countList(const DBClass *example, int &count, std::string where_clause) const;

  //! templated implementation of count list that works on filter clauses.
  template <typename T>
  bool countList(int &count, const FilterClause clause=FilterClause()) const
  {
    T example;
    return countList(&example, count, clause.clause_);
  }

  //! Writes the value of one particular field of a DBClass to the database
  bool saveToDatabase(const DBFieldBase* field);

  //! Reads the value of one particular fields of a DBClass from the database
  bool loadFromDatabase(DBFieldBase* field) const;

  //! Inserts a new instance of a DBClass into the database
  bool insertIntoDatabase(DBClass* instance);

  //! Deletes an instance of a DBClass from the database
  bool deleteFromDatabase(DBClass* instance);

};

/*! The datatype T is expected to be derived from DBClass.

  The example is used only to decide which fields should be retrieved from the database.
  Note that the primary key field is ALWAYS retrieved; you can expect the returned list
  to have the primary key set. Any other fields are retrieved ONLY if they are marked
  with syncFromDatabase in the example.

  Note that the example is not used to decide which instanced to retrieve (but only which
  *fields* of the instances). To retrieve only certain fields, you must use the where_clause.
  This is not ideal, as much functionality is hidden from the user who is not exposed
  to SQL syntax. For those functions where the external user needs the where_clause (even if
  he does not know it) we are currently providing public wrappers, but that  might change in 
  the future. 

  The significant difference between this function and the version that reads a 
  certain field is that this function creates new instances of the DBClass and gives them
  the right values of the primary key. The function that reads a certain field expects the 
  instance of DBClass to already exist, and its primary key field to be set correctly 
  already.
*/
template <class T>
bool PostgresqlDatabase::getList(std::vector< boost::shared_ptr<T> > &vec, 
				 const T &example, std::string where_clause) const
{
  //we will store here the fields to be retrieved retrieve from the database
  std::vector<const DBFieldBase*> fields;
  //we will store here their index in the result returned from the database
  std::vector<int> column_ids;
  boost::shared_ptr<PGresultAutoPtr> result;

  int num_tuples;
  //do all the heavy lifting of querying the database and getting the raw result
  if (!getListRawResult(&example, fields, column_ids, where_clause, result, num_tuples))
  {
    return false;
  }

  vec.clear();
  if (!num_tuples)
  {
    return true;
  }

  //parse the raw result and populate the list 
  for (int i=0; i<num_tuples; i++)
  {
    boost::shared_ptr<T> entry(new T);
    if (populateListEntry(entry.get(), result, i, fields, column_ids))
    {
      vec.push_back(entry);
    }
  }
  return true;
}


}//namespace

#endif
