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

#include "database_interface/postgresql_database.h"

// the header of the libpq library
#include <libpq-fe.h>

#include <sstream>
#include <iostream>

namespace database_interface {

/*! A little helper class to behave much like an auto ptr for the
  PGresult, except that instead of deleting it when it goes out of
  scope, it calls PQclear() on it.
*/
class PostgresqlDatabase::PGresultAutoPtr
{
private:
  PGresult* result_;
public:
  PGresultAutoPtr(PGresult *ptr) : result_(ptr){}
  ~PGresultAutoPtr(){PQclear(result_);}
  void reset(PGresult *ptr){PQclear(result_); result_=ptr;}
  PGresult* operator * (){return result_;}
};


void PostgresqlDatabase::pgMDBconstruct(std::string host, std::string port, std::string user,
						 std::string password, std::string dbname )
{
  std::string conn_info = "host=" + host + " port=" + port + 
    " user=" + user + " password=" + password + " dbname=" + dbname;
  connection_= PQconnectdb(conn_info.c_str());
  if (PQstatus(connection_)!=CONNECTION_OK) 
  {
    ROS_ERROR("Database connection failed with error message: %s", PQerrorMessage(connection_));
  }
}

PostgresqlDatabase::PostgresqlDatabase(const PostgresqlDatabaseConfig &config)
  : in_transaction_(false)
{
  pgMDBconstruct(config.getHost(), config.getPort(), config.getUser(), 
                 config.getPassword(), config.getDBname());
}

PostgresqlDatabase::PostgresqlDatabase(std::string host, std::string port, std::string user,
						 std::string password, std::string dbname )
  : in_transaction_(false)
{
  pgMDBconstruct(host, port, user, password, dbname);
}

PostgresqlDatabase::~PostgresqlDatabase()
{
  PQfinish(connection_);
}

bool PostgresqlDatabase::isConnected() const
{
  if (PQstatus(connection_)==CONNECTION_OK) return true;
  else return false;
}

/*! Returns true if the rollback query itself succeeds, false if it does not */
bool PostgresqlDatabase::rollback()
{
  PGresultAutoPtr result((PQexec(connection_,"ROLLBACK;")));
  if (PQresultStatus(*result) != PGRES_COMMAND_OK)
  {
    ROS_ERROR("Rollback failed");
    return false;
  }
  in_transaction_ = false;
  return true;
}

/*! Returns true if the begin query itself succeeds, false if it does not */
bool PostgresqlDatabase::begin()
{
  if( in_transaction_ ) return true;
  //place a begin
  PGresultAutoPtr result(PQexec(connection_, "BEGIN;"));
  if (PQresultStatus(*result) != PGRES_COMMAND_OK)
  {
    ROS_ERROR("Database begin query failed. Error: %s", PQresultErrorMessage(*result));
    return false;
  }
  in_transaction_ = true;
  return true;
}

/*! Returns true if the commit query itself succeeds, false if it does not */
bool PostgresqlDatabase::commit()
{
  PGresultAutoPtr result(PQexec(connection_, "COMMIT;"));
  if (PQresultStatus(*result) != PGRES_COMMAND_OK)
  {
    ROS_ERROR("Database commit query failed. Error: %s", PQresultErrorMessage(*result));
    return false;
  }
  in_transaction_ = false;
  return true;
}

bool PostgresqlDatabase::getVariable(std::string name, std::string &value) const
{
  std::string query("SELECT variable_value FROM variable WHERE variable_name=" + name);
  PGresultAutoPtr result(PQexec(connection_, query.c_str()));  
  if (PQresultStatus(*result) != PGRES_TUPLES_OK)
  {
    ROS_ERROR("Database get variable query failed. Error: %s", PQresultErrorMessage(*result));
    return false;
  }
  if (PQntuples(*result)==0)
  {
    ROS_ERROR("Database get variable query failed. Variable %s not in database", name.c_str());
    return false;
  }
  value = PQgetvalue(*result, 0, 0);
  return true;
}

/*! The value is returned as a string; caller has to convert it. Returns false if the
  query fails or the sequence is not found.
*/
bool PostgresqlDatabase::getSequence(std::string name, std::string &value)
{
  std::string query("SELECT * FROM currval('" + name + "');");
  PGresultAutoPtr result( PQexec(connection_, query.c_str()) );
  if (PQresultStatus(*result) != PGRES_TUPLES_OK)
  {
    ROS_ERROR("Get sequence: query failed. Error: %s", PQresultErrorMessage(*result));
    return false;
  }
  if (!PQntuples(*result))
  {
    ROS_ERROR("Get sequence: sequence %s not found", name.c_str());
    return false;
  }
  const char *id_char = PQgetvalue(*result, 0, 0);
  value.assign(id_char);
  return true;
}

/*! Creates and runs the SQL query for retrieveing the list. Has been separated from the
  rest of the getList function so that we can have only the part that instantiates the entries
  separated from the parts that speak SQL, so that we don't have to have SQL in the header
  of the PostgresqlDatabase class.

  See the general getList(...) documentation for more details. This function will run the 
  query, return its raw result, and populate the fields and columns_ids vectors with the fields
  that were retrieved and the columns in the result that they correspond to.
 */
bool PostgresqlDatabase::getListRawResult(const DBClass *example, 
						   std::vector<const DBFieldBase*> &fields, 
						   std::vector<int> &column_ids,
						   std::string where_clause,
						   boost::shared_ptr<PGresultAutoPtr> &result, int &num_tuples) const
{
  //we cannot handle binary results in here; libpq does not support binary results
  //for just part of the query, so they all have to be text
  if(example->getPrimaryKeyField()->getType() == DBFieldBase::BINARY)
  {
    ROS_ERROR("Database get list: can not use binary primary key (%s)", 
	      example->getPrimaryKeyField()->getName().c_str());
    return false;
  }

  std::string select_query;
  select_query += "SELECT " + example->getPrimaryKeyField()->getName() + " ";  
  fields.push_back(example->getPrimaryKeyField());

  //we will store here the list of tables we will join on
  std::vector<std::string> join_tables;
  std::string join_clauses;
  for (size_t i=0; i<example->getNumFields(); i++)
  {
    if (!example->getField(i)->getReadFromDatabase()) continue;

    if (example->getField(i)->getType()==DBFieldBase::BINARY)
    {
      ROS_WARN("Database get list: binary field (%s) can not be loaded by default", 
	       example->getField(i)->getName().c_str());
      continue;
    }

    select_query += ", " + example->getField(i)->getName();
    fields.push_back(example->getField(i));
    if ( example->getField(i)->getTableName() != example->getPrimaryKeyField()->getTableName() )
    {
      //check if we are already joining on this table
      bool already_join = false;
      for (size_t j=0; j<join_tables.size() && !already_join; j++)
      {
	if (join_tables[j] == example->getField(i)->getTableName()) already_join = true;
      }
      
      if (!already_join)
      {
	const DBFieldBase *foreign_key = NULL;
	if (!example->getForeignKey(example->getField(i)->getTableName(), foreign_key))
	{
	  ROS_ERROR("Database get list: could not find foreign key for table %s", 
		    example->getField(i)->getTableName().c_str());
	  return false;
	}
	join_clauses += " JOIN " + example->getField(i)->getTableName() + " USING (" 
	  + foreign_key->getName() + ") ";
	join_tables.push_back( example->getField(i)->getTableName() );
      }
    }
  }

  select_query += " FROM " + example->getPrimaryKeyField()->getTableName() + " ";

  if (!join_clauses.empty())
  {
    select_query += join_clauses;
  }

  if (!where_clause.empty())
  {
    select_query += " WHERE " + where_clause;
  }

  select_query += ";";

  //ROS_INFO("Query: %s", select_query.c_str());

  PGresult* raw_result = PQexec(connection_, select_query.c_str());
  result.reset( new PGresultAutoPtr(raw_result) );
  if (PQresultStatus(raw_result) != PGRES_TUPLES_OK)
  {
    ROS_ERROR("Database get list: query failed. Error: %s", PQresultErrorMessage(raw_result));
    return false;
  }
  
  num_tuples = PQntuples(raw_result);
  if (!num_tuples) 
  {
    return true;
  }
  
  //store the column id's for each field we retrieved
  for (size_t t=0; t<fields.size(); t++)
  {
    int id =  PQfnumber(raw_result, fields[t]->getName().c_str());
    if (id < 0)
    {
      ROS_ERROR("Database get list: column %s missing in result", fields[t]->getName().c_str());
      return false;
    }
    column_ids.push_back(id);
  }   
  return true;
}

/*! Parses a single, already instantiated entry (an instance of DBClass) from a raw database
  result, given the list of fields that were retrieved and their respective column ids in
  the result. Helper function for getList(...)
*/
bool PostgresqlDatabase::populateListEntry(DBClass *entry, boost::shared_ptr<PGresultAutoPtr> result, 
						    int row_num,
						    const std::vector<const DBFieldBase*> &fields,
						    const std::vector<int> &column_ids) const
{
  for (size_t t=0; t<fields.size(); t++)
  {
    const char* char_value =  PQgetvalue(**result, row_num, column_ids[t]);
    DBFieldBase *entry_field = entry->getField(fields[t]->getName());
    if (!entry_field)
    {
      ROS_ERROR("Database get list: new entry missing field %s", fields[t]->getName().c_str());
      return false;
    }
    if ( !entry_field->fromString(char_value) )
    {
      ROS_ERROR("Database get list: failed to parse response \"%s\" for field \"%s\"",   
                char_value, fields[t]->getName().c_str()); 
      return false;
    }
  }
  return true;
}

/*! The example is used *only* to indicate instances of what class we are counting. To perform
  additional pruning, the where_clause is used. In the future, we might use the example to do
  actual decisions on counting based on the contents of the fields.

  The counting is performed only on the primary key of the given class.
 */
bool PostgresqlDatabase::countList(const DBClass *example, int &count, std::string where_clause) const
{
  const DBFieldBase* pk_field = example->getPrimaryKeyField();
  
  std::string query( "SELECT COUNT(" + pk_field->getName() + ") FROM " + pk_field->getTableName() );
  if (!where_clause.empty())
  {
    query += " WHERE " + where_clause;
  }
  query += ";";

  ROS_INFO("Query (count): %s", query.c_str());
  PGresultAutoPtr result( PQexec(connection_, query.c_str()) );
			 
  if (PQresultStatus(*result) != PGRES_TUPLES_OK)
  {
    ROS_ERROR("Database count list query failed. Error: %s", PQresultErrorMessage(*result));
    return false;
  }
  const char *reply =  PQgetvalue(*result, 0, 0);
  if (!DBStreamable<int>::streamableFromString(count, reply)) 
  {
    ROS_ERROR("Database count list failed. Could not understand reply: %s", reply);
    return false;
  }
  return true;  
}

/*! The instance of DBClass that this implicitly refers to is the *owner* of the field that
  is passed in. If the field that is passed in is not in the same table as the primary key,
  tables are joined based on the primary key. 

  TODO: fix this so that we don't always have to join on the primary key.
 */
bool PostgresqlDatabase::saveToDatabase(const DBFieldBase* field)
{
  if (!field->getWritePermission())
  {
    ROS_ERROR("Database save field: field %s does not have write permission", field->getName().c_str());
    return false;
  }

  const DBFieldBase* key_field;
  if (field->getTableName() == field->getOwner()->getPrimaryKeyField()->getTableName()) 
  {
    key_field = field->getOwner()->getPrimaryKeyField();
  }
  else 
  {
    if (!field->getOwner()->getForeignKey(field->getTableName(), key_field))
    {
      ROS_ERROR("Database save field: could not find foreign key for table %s", 
		field->getTableName().c_str());
      return false;
    }
    //here we could also check if the join is done on our primary key, and 
    //reject if not insted of using the write_permisison flag
  }
 
  //prepare query with parameters so we can use binary data if needed

  std::string query("UPDATE " + field->getTableName() + 
		    " SET " + field->getName() + "=$2"
		    " WHERE " + key_field->getName() + "=$1;");

  std::vector<const char*> param_values(2);
  std::vector<int> param_lengths(2);
  std::vector<int> param_formats(2);
  //first parameter is always text
  std::string id_str;
  if (!key_field->toString(id_str))
  {
    ROS_ERROR("Database save field: failed to convert key id value to string");
    return false;
  }
  param_formats[0] = 0;
  param_values[0] = id_str.c_str();

  //second parameter could be binary
  std::string value_str;
  if (field->getType() == DBFieldBase::TEXT)
  {
    if (!field->toString(value_str)) 
    {
      ROS_ERROR("Database save field: failed to convert field value to string");
      return false;
    }
    param_formats[1] = 0;
    param_values[1] = value_str.c_str();
  }
  else if (field->getType() == DBFieldBase::BINARY)
  {
    size_t length;
    if (!field->toBinary(param_values[1], length))
    {
      ROS_ERROR("Database save field: failed to convert field value to binary");
      return false;
    }
    param_lengths[1] = length;
    param_formats[1] = 1;
  }
  else
  {
    ROS_ERROR("Database save field: unkown field type");
    return false;
  }

  //ROS_INFO("Save field query: %s $1=%s, $2=%s", query.c_str(), param_values[0], param_values[1]);

  PGresultAutoPtr result( PQexecParams(connection_, query.c_str(), 2, 
				       NULL, &param_values[0], &param_lengths[0], &param_formats[0], 0) );
  if (PQresultStatus(*result) != PGRES_COMMAND_OK)
  {
    ROS_ERROR("Database save field: query failed. Error: %s", PQresultErrorMessage(*result));
    return false;
  }
  return true;
}

/*! The instance of DBClass that this implicitly refers to is the *owner* of the field that
  is passed in. If the field that is passed in is not in the same table as the primary key,
  tables are joined based on the primary key which is assumed to be the foreign key in the
  changed field's table. 
 */
bool PostgresqlDatabase::loadFromDatabase(DBFieldBase* field) const
{
  const DBFieldBase* key_field = NULL;
  if (field->getTableName() == field->getOwner()->getPrimaryKeyField()->getTableName())
  {
    key_field = field->getOwner()->getPrimaryKeyField();
  }
  else 
  {
    if (!field->getOwner()->getForeignKey(field->getTableName(), key_field))
    {
      ROS_ERROR("Database load field: could not find foreign key for table %s", 
		field->getTableName().c_str());
      return false;
    }
  }

  std::string id_str;
  if (!key_field->toString(id_str))
  {
    ROS_ERROR("Database load field: failed to convert key id value to string");
    return false;
  }

  std::string query("SELECT " + field->getName() + " FROM " + field->getTableName() + 
		    " WHERE " + key_field->getName() + " ='" + id_str + "';");

  //ROS_INFO_STREAM("Load field query: " << query);

  int data_type;
  if (field->getType() == DBFieldBase::TEXT) data_type = 0;
  else if (field->getType() == DBFieldBase::BINARY) data_type = 1;
  else
  {
    ROS_ERROR("Database load field: unkown field type");
    return false;
  }

  PGresultAutoPtr result( PQexecParams(connection_, query.c_str(), 0, NULL, NULL, NULL, NULL, data_type) );
  if (PQresultStatus(*result) != PGRES_TUPLES_OK)
  {
    ROS_ERROR("Database load field: query failed. Error: %s", PQresultErrorMessage(*result));
    return false;
  }

  if (PQntuples(*result)==0)
  {
    ROS_ERROR("Database load field: no entry found for key value %s on column %s", 
	      id_str.c_str(), key_field->getName().c_str());
    return false;
  }

  const char* result_char =  PQgetvalue(*result, 0, 0);
  if (field->getType() == DBFieldBase::TEXT)
  {
    if ( !field->fromString(result_char) )
    {
      ROS_ERROR("Database load field: failed to parse text result \"%s\" for field \"%s\"", 
                result_char, field->getName().c_str()); 
      return false;
    }
  } 
  else if (field->getType() == DBFieldBase::BINARY)
  {
    size_t length = PQgetlength(*result, 0, 0);
    if (!field->fromBinary(result_char, length))
    {
      ROS_ERROR("Database load field: failed to parse binary result length %d for field \"%s\"",
                (int) length, field->getName().c_str()); 
      return false;
    }
  }
  else
  {
    ROS_ERROR("Database load field: failed to parse unkown field type");
    return false;
  }

  return true;
}

/*! Inserts into the database the fields of an instance that go into a single table.

  If that table is the table of the primary key, everything is inserted normally.

  If not, the *first* entry in the vector of fields is expected to be the primary
  key, with the value already set correctly. The table that we are inserting in is
  expected to have a foreign key that references the primary key field of our class.
 */
bool PostgresqlDatabase::insertIntoTable(std::string table_name,
						  const std::vector<const DBFieldBase*> &fields)
{
  if (fields.empty())
  {
    ROS_ERROR("Insert into table: no columns to insert");
    return false;
  }

  std::string query("INSERT INTO " + table_name + "(");

  //the first field might be the foreign key
  if (table_name == fields[0]->getTableName())
  {
    //it's not, just any other field
    query += fields[0]->getName();
  }
  else
  {
    //we are inserting the foreign key, which for now is always assumed to be referencing our primary key
    const DBFieldBase *foreign_key = NULL;
    if (!fields[0]->getOwner()->getForeignKey(table_name, foreign_key))
    {
      ROS_ERROR("Database insert into table: could not find foreign key for table %s", table_name.c_str());
      return false;
    }
    query += foreign_key->getName();
  }

  for(size_t i=1; i<fields.size(); i++)
  {
    query += "," + fields[i]->getName();
  }
  query += ")";

  query += "VALUES(";
  for (size_t i=0; i<fields.size(); i++)
  {
    if ( i!=0 && table_name!=fields[i]->getTableName())
    {
      ROS_ERROR("Database insert into table: field table does not match table name");
      return false;
    }
    if (i!=0) query += ",";
    std::ostringstream ss;
    ss << i+1;
    query += "$" + ss.str();
  }
  query += ");";
  
  //ROS_INFO("Query: %s", query.c_str());

  //now prepare the arguments
  std::vector<std::string> param_strings(fields.size());
  std::vector<const char*> param_values(fields.size());
  std::vector<int> param_lengths(fields.size());
  std::vector<int> param_formats(fields.size());
  for (size_t i=0; i<fields.size(); i++)
  {
    if (fields[i]->getType() == DBFieldBase::TEXT)
    {
      if (!fields[i]->toString(param_strings[i]))
      {
	ROS_ERROR("Database insert into table: could not parse field %s", fields[i]->getName().c_str());
	return false;
      }
      param_values[i] = param_strings[i].c_str();
      param_formats[i] = 0;
    }
    else if (fields[i]->getType() == DBFieldBase::BINARY)
    {
      size_t length;
      if (!fields[i]->toBinary(param_values[i], length))
      {
	ROS_ERROR("Database insert into table: could not binarize field %s", fields[i]->getName().c_str());
	return false;
      }
      param_lengths[i] = length;
      param_formats[i] = 1;
    }
    else
    {
      ROS_ERROR("Database insert into table: unknown field type");
      return false;
    }
  }

  //and send the query
  PGresultAutoPtr result( PQexecParams(connection_, query.c_str(), fields.size(), NULL, 
				       &(param_values[0]), &(param_lengths[0]), &(param_formats[0]), 0) );
  if (PQresultStatus(*result) != PGRES_COMMAND_OK)
  {
    ROS_ERROR("Database insert into table: query failed.\nError: %s.\nQuery: %s",
              PQresultErrorMessage(*result), query.c_str());
    return false;
  }

  return true;
}

/*! Inserts only the fields marked with writeToDatabase.

  The primary key must either be inserted specifically (and thus marked with writeToDatabase)
  or have a default value associated with a sequence. In the latter case, the value is retrieved
  after insertion, and set to the primary key field of the instance.
 */
bool PostgresqlDatabase::insertIntoDatabase(DBClass* instance)
{
  //primary key must be text; its table is first
  DBFieldBase* pk_field = instance->getPrimaryKeyField();
  if (pk_field->getType() != DBFieldBase::TEXT)
  {
    ROS_ERROR("Database insert: cannot insert binary primary key %s", pk_field->getName().c_str());
    return false;
  }

  //make lists of which fields go in which table
  std::vector<std::string> table_names;
  std::vector< std::vector<const DBFieldBase*> > table_fields;

  //the first table we must insert into always belongs to the primary key
  table_names.push_back(pk_field->getTableName());
  table_fields.push_back( std::vector<const DBFieldBase*>() );
  
  bool insert_pk;
  if (pk_field->getWriteToDatabase())
  {
    //we will explicitly insert the primary key in its own table
    table_fields.back().push_back(pk_field);
    insert_pk = true;
  }
  else
  {
    //we do not insert the primary key in its own table; presumably it has a 
    //default value from a sequence which we will retrieve afterwards
    if (pk_field->getSequenceName().empty())
    {
      ROS_ERROR("Database insert: attempt to insert instance without primary key and no sequence for it");
      return false;
    }
    insert_pk = false;
  }
  
  //prepare insertions into other tables
  //note that even if we are inserting no data, we still need to make an entry
  //containing the foreign key so we have it for future insertions
  for (size_t i=0; i<instance->getNumFields(); i++)
  {
    //see if we are already inserting in this table
    bool found = false;
    size_t t;
    for (t=0; t<table_names.size(); t++)
    {
      if ( table_names[t] == instance->getField(i)->getTableName() )
      {
        found = true;
        break;
      }
    }
    if (!found)
    {
      //if we are not joining on our primary key, we should not be inserting in this table
      const DBFieldBase* foreign_key = NULL;
      if (!instance->getForeignKey(instance->getField(i)->getTableName(), foreign_key))
      {
        ROS_ERROR("Database insert into table: could not find foreign key for table %s", 
          instance->getField(i)->getTableName().c_str());
        return false;
      }
      if (foreign_key != pk_field) 
      {
        continue;
      }
      //we are joining on primary key, so we will need to insert it in other table as well
      table_names.push_back( instance->getField(i)->getTableName() );
      table_fields.push_back( std::vector<const DBFieldBase*>() );
      //in all the other tables we must explicitly insert the value of our 
      //primary key as it is the foreign key in all other tables
      table_fields.back().push_back(pk_field);
      t = table_names.size() - 1;
    }
    if ( !instance->getField(i)->getWriteToDatabase() ) continue;
    if ( instance->getField(i)->getType() != DBFieldBase::TEXT )
    {
      ROS_WARN("Database insert: cannot insert binary field %s in database", 
        instance->getField(i)->getName().c_str());
      continue;
    }
    //insert the field itself
    table_fields[t].push_back(instance->getField(i));
  }
  
  //BEGIN transaction
  if (!begin()) return false;

  //first we insert into the primary key's table
  if (!insertIntoTable(table_names[0], table_fields[0])) 
  {
    rollback();
    return false;
  }

  //if we have to, retrieve the primary key
  if (!insert_pk)
  {
    std::string sequence_value;
    if (!getSequence(pk_field->getSequenceName(), sequence_value) || !pk_field->fromString(sequence_value))
    {
      ROS_ERROR("Database insert: failed to retrieve primary key after insertion");
      rollback();
      return false;
    }
  }

  //insert into the rest of the tables
  for (size_t i=1; i<table_names.size(); i++)
  {
    if (!insertIntoTable(table_names[i], table_fields[i]))
    {
      rollback();
      return false;
    }
  }

  //COMMIT transaction
  if (!commit()) return false;

  return true;
}

/*! Deletes a row from a table based on the value of the specified field */
bool PostgresqlDatabase::deleteFromTable(std::string table_name, const DBFieldBase *key_field)
{
  std::string id_str;
  if (!key_field->toString(id_str))
  {
    ROS_ERROR("Database delete from table: failed to convert key id value to string");
    return false;
  }

  std::string query("DELETE FROM " + table_name + " WHERE " + key_field->getName() + "=" + id_str);
  PGresultAutoPtr result( PQexec(connection_, query.c_str()) );
  if (PQresultStatus(*result) != PGRES_COMMAND_OK)
  {
    ROS_ERROR("Database delete from table: query failed. Error: %s", PQresultErrorMessage(*result));
    return false;
  }
  return true;
}

/*! It removes the entry from the table that holds our primary key, but also from 
  other tables that might hold our fields. Those tables have to be removed first.
*/
bool PostgresqlDatabase::deleteFromDatabase(DBClass* instance)
{
  std::vector<std::string> table_names;
  std::vector<const DBFieldBase*> table_fields;
  DBFieldBase* pk_field = instance->getPrimaryKeyField();
  //the table of the primary key
  table_names.push_back( pk_field->getTableName() );
  table_fields.push_back( pk_field );
  //prepare deletions from the other tables
  for (size_t i=0; i<instance->getNumFields(); i++)
  {
    //see if we are already deleting in this table
    size_t t;
    for (t=0; t<table_names.size(); t++)
    {
      if ( table_names[t] == instance->getField(i)->getTableName() ) break;
    }
    if (t<table_names.size()) continue;

    //if we are not joining on our primary key, we should NOT be deleting from this table
    const DBFieldBase* foreign_key = NULL;
    if (!instance->getForeignKey(instance->getField(i)->getTableName(), foreign_key))
    {
      ROS_ERROR("Database insert into table: could not find foreign key for table %s", 
		instance->getField(i)->getTableName().c_str());
      return false;
    }
    if (foreign_key != pk_field) continue;

    //we are joining on primary key, so we will need to delete the row in the other table as well
    table_names.push_back( instance->getField(i)->getTableName() );
    table_fields.push_back( pk_field );
  }

  //BEGIN transaction
  if (!begin()) return false;
  
  //delete from all tables, but primary key table goes last
  for (int i=(int)table_names.size()-1; i>=0; i--)
  {
    if (!deleteFromTable(table_names[i], table_fields[i]))
    {
      rollback();
      return false;
    }
  }

  //COMMIT transaction
  if (!commit()) return false;

  return true;

}

}//namespace
