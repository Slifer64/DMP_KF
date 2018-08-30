#ifndef PARAMETERS_LIST_64_H
#define PARAMETERS_LIST_64_H

#include <ros/ros.h>
#include <vector>
#include <string>
#include <sstream>
#include <ctime>

namespace as64_
{

namespace param_
{

/** Variable parameter list
* Creates a list with variable number of arguments.
* The arguments can be of different types.
* The parameters are writen or read as pair of an std::string,
* denoting the parameter name, and a value (of any type) denoting
* the value for this key.
*/
class ParamList
{
public:
  /** Parameter list constructor
  * Initializes the parameter list object assigning a unique id to it.
  * This id is used as a prefix in the ros parameter server.
  */
  ParamList();

  /**
  * Get the value of a specific parameter.
  * @param[in] key The parameter name.
  * @param[in] value The value correspodning to 'key'.
  * @return True if the parameter name is found, false otherwise.
  */
  template<class Type>
  bool getParam(const std::string &key, Type &value) const;

  template<class Type>
  bool getParam(const char *key, Type &value) const;

  /**
  * Set a new parameter (name,value) pair.
  * @param[in] key The parameter name.
  * @param[in] value The value correspodning to 'key'.
  */
  template<class Type>
  void setParam(const std::string &key, Type &value) const;

  template<class Type>
  void setParam(const char *key, Type &value) const;


  void 	setParam(const std::string &key, const char *s) const;
  void 	setParam(const char *key, const char *s) const;

  void 	setParam(const std::string &key, double d) const;
  void 	setParam(const char *key, double d) const;

  void 	setParam(const std::string &key, int i) const;
  void 	setParam(const char *key, int i) const;

  void 	setParam(const std::string &key, bool b) const;
  void 	setParam(const char *key, bool b) const;

private:
  ros::NodeHandle nh;
  std::string paramObjId;
};


template<class Type>
bool ParamList::getParam(const std::string &key, Type &value) const
{
  return this->getParam(key.c_str(), value);
}

template<class Type>
bool ParamList::getParam(const char *key, Type &value) const
{
  return this->nh.getParam(this->paramObjId + key, value);
}

template<class Type>
void ParamList::setParam(const std::string &key, Type &value) const
{
  this->setParam(key.c_str(), value);
}

template<class Type>
void ParamList::setParam(const char *key, Type &value) const
{
  this->nh.setParam(this->paramObjId + key, value);
}

} // namespace param_

} // namespace as64_

#endif // PARAMETERS_LIST_64_H
