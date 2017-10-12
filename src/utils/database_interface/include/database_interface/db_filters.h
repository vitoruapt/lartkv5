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

// Author(s): Lorenz Moesenlechner

#ifndef _DB_FILTERS_H_
#define _DB_FILTERS_H_

#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>

#include <string>

#include "database_interface/db_field.h"

/** This file contains basic filtering operators for getting a list of
 * instances out of the database. This works by overloading the
 * operators to take either a DBField<T> or the wrapper dbField that
 * is used to specify a field by name instead of instance. This
 * completely replaces the 'example' parameter. It is now possible to
 * specify filter clauses for a query in c++ syntax like this:
 *
 *  db.getList(seq, dbField("id") > 20 && dbField("x") < 30);
 *  
 *  */

namespace database_interface
{

template<typename T>
inline std::string toString(const T &data)
{
  return boost::lexical_cast<std::string>(data);
}

template<>
inline std::string toString<double>(const double &data)
{
  return (boost::format("%.5f") % data).str();
}

struct FilterClause
{
  std::string clause_;

  FilterClause() {}
  FilterClause(const std::string clause)
    : clause_(clause) {}
};

struct dbField
{
  std::string name_;

  dbField(const std::string name)
    : name_(name) {}
};

// All operators return a FilterClause that contains a string
// representation of the specific operation. To extend it, you need
// to implement an operator for the classes DBField<T> and dbField
// in all different possible combinations. To get a string
// representation of the arbitrary datatype T, boost::lexical_cast
// is used.
  
// Operator <
template<typename T>
FilterClause operator<(const DBField<T> &lhs, const T &rhs)
{
  return FilterClause(lhs.getName() + " < '" + toString(rhs) + "'");
}
    
template<typename T>
FilterClause operator<(const T &lhs, const DBField<T> &rhs)
{
  return FilterClause("'" + toString(lhs) + "' < " + rhs.getName());
}

template<typename T>
FilterClause operator<(const DBField<T> &lhs, const DBField<T> &rhs)
{
  return FilterClause(lhs.getName() + " < " + rhs.getName());
}

template<typename T>
FilterClause operator<(const dbField &lhs, const T &rhs)
{
  return FilterClause(lhs.name_ + " < '" + toString(rhs) + "'");
}

template<typename T>
FilterClause operator<(const T &lhs, const dbField &rhs)
{
  return FilterClause("'" + toString(lhs) + "' < " + rhs.name_);
}

template<typename T>
FilterClause operator<(const dbField &lhs, const dbField &rhs)
{
  return FilterClause(lhs.name_ + " < " + rhs.name_);
}

// Opetrator <=
template<typename T>
FilterClause operator<=(const DBField<T> &lhs, const T &rhs)
{
  return FilterClause(lhs.getName() + " <= '" + toString(rhs) + "'");
}
    
template<typename T>
FilterClause operator<=(const T &lhs, const DBField<T> &rhs)
{
  return FilterClause("'" + toString(lhs) + "' <= " + rhs.getName());
}

template<typename T>
FilterClause operator<=(const DBField<T> &lhs, const DBField<T> &rhs)
{
  return FilterClause(lhs.getName() + " <= " + rhs.getName());
}

template<typename T>
FilterClause operator<=(const dbField &lhs, const T &rhs)
{
  return FilterClause(lhs.name_ + " <= '" + toString(rhs) + "'");
}

template<typename T>
FilterClause operator<=(const T &lhs, const dbField &rhs)
{
  return FilterClause("'" + toString(lhs) + "' <= " + rhs.name_);
}

template<typename T>
FilterClause operator<=(const dbField &lhs, const dbField &rhs)
{
  return FilterClause(lhs.name_ + " <= " + rhs.name_);
}

// Operator >
template<typename T>
FilterClause operator>(const DBField<T> &lhs, const T &rhs)
{
  return FilterClause(lhs.getName() + " > '" + toString(rhs) + "'");
}
    
template<typename T>
FilterClause operator>(const T &lhs, const DBField<T> &rhs)
{
  return FilterClause("'" + toString(lhs) + "' > " + rhs.getName());
}

template<typename T>
FilterClause operator>(const DBField<T> &lhs, const DBField<T> &rhs)
{
  return FilterClause(lhs.getName() + " > " + rhs.getName());
}

template<typename T>
FilterClause operator>(const dbField &lhs, const T &rhs)
{
  return FilterClause(lhs.name_ + " > '" + toString(rhs) + "'");
}

template<typename T>
FilterClause operator>(const T &lhs, const dbField &rhs)
{
  return FilterClause("'" + toString(lhs) + "' > " + rhs.name_);
}

template<typename T>
FilterClause operator>(const dbField &lhs, const dbField &rhs)
{
  return FilterClause(lhs.name_ + " > " + rhs.name_);
}

// Opetrator >=
template<typename T>
FilterClause operator>=(const DBField<T> &lhs, const T &rhs)
{
  return FilterClause(lhs.getName() + " >= '" + toString(rhs) + "'");
}
    
template<typename T>
FilterClause operator>=(const T &lhs, const DBField<T> &rhs)
{
  return FilterClause("'" + toString(lhs) + "' >= " + rhs.getName());
}

template<typename T>
FilterClause operator>=(const DBField<T> &lhs, const DBField<T> &rhs)
{
  return FilterClause(lhs.getName() + " >= " + rhs.getName());
}

template<typename T>
FilterClause operator>=(const dbField &lhs, const T &rhs)
{
  return FilterClause(lhs.name_ + " >= '" + toString(rhs) + "'");
}

template<typename T>
FilterClause operator>=(const T &lhs, const dbField &rhs)
{
  return FilterClause("'" + toString(lhs) + "' >= " + rhs.name_);
}

template<typename T>
FilterClause operator>=(const dbField &lhs, const dbField &rhs)
{
  return FilterClause(lhs.name_ + " >= " + rhs.name_);
}

// Operator ==
template<typename T>
FilterClause operator==(const DBField<T> &lhs, const T &rhs)
{
  return FilterClause(lhs.getName() + " == '" + toString(rhs) + "'");
}
    
template<typename T>
FilterClause operator==(const T &lhs, const DBField<T> &rhs)
{
  return FilterClause("'" + toString(lhs) + "' == " + rhs.getName());
}

template<typename T>
FilterClause operator==(const DBField<T> &lhs, const DBField<T> &rhs)
{
  return FilterClause(lhs.getName() + " == " + rhs.getName());
}

template<typename T>
FilterClause operator==(const dbField &lhs, const T &rhs)
{
  return FilterClause(lhs.name_ + " == '" + toString(rhs) + "'");
}

template<typename T>
FilterClause operator==(const T &lhs, const dbField &rhs)
{
  return FilterClause("'" + toString(lhs) + "' == " + rhs.name_);
}

template<typename T>
FilterClause operator==(const dbField &lhs, const dbField &rhs)
{
  return FilterClause(lhs.name_ + " == " + rhs.name_);
}

// Operator !=
template<typename T>
FilterClause operator!=(const DBField<T> &lhs, const T &rhs)
{
  return FilterClause(lhs.getName() + " != '" + toString(rhs) + "'");
}

template<typename T>
FilterClause operator!=(const T &lhs, const DBField<T> &rhs)
{
  return FilterClause("'" + toString(lhs) + "' != " + rhs.getName());
}

template<typename T>
FilterClause operator!=(const DBField<T> &lhs, const DBField<T> &rhs)
{
  return FilterClause(lhs.getName() + " != " + rhs.getName());
}

template<typename T>
FilterClause operator!=(const dbField &lhs, const T &rhs)
{
  return FilterClause(lhs.name_ + " != '" + toString(rhs) + "'");
}

template<typename T>
FilterClause operator!=(const T &lhs, const dbField &rhs)
{
  return FilterClause("'" + toString(lhs) + "' != " + rhs.name_);
}

template<typename T>
FilterClause operator!=(const dbField &lhs, const dbField &rhs)
{
  return FilterClause(lhs.name_ + " != " + rhs.name_);
}

// Combination clauses (and, or, ...)
inline FilterClause operator&&(const FilterClause &lhs, const FilterClause &rhs)
{
  return FilterClause(" ( " + lhs.clause_ + " AND " + rhs.clause_ + " )");
}

inline FilterClause operator||(const FilterClause &lhs, const FilterClause &rhs)
{
  return FilterClause(" ( " + lhs.clause_ + " OR " + rhs.clause_ + " )");
}

} //namespace

#endif
