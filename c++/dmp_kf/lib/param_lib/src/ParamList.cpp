#include <param_lib/ParamList.h>

namespace as64_
{

namespace param_
{

  ParamList::ParamList()
  {
    long id = clock();
    std::ostringstream o;
    o << id;

    this->paramObjId = "ParamList__" + o.str() + "__";
  }


  void 	ParamList::setParam(const std::string &key, const char *s) const
  {
    this->setParam(key.c_str(), s);
  }

  void 	ParamList::setParam(const char *key, const char *s) const
  {
    this->nh.setParam(this->paramObjId + key, s);
  }


  void 	ParamList::setParam(const std::string &key, double d) const
  {
    this->setParam(key.c_str(), d);
  }

  void 	ParamList::setParam(const char *key, double d) const
  {
    this->nh.setParam(this->paramObjId + key, d);
  }


  void 	ParamList::setParam(const std::string &key, int i) const
  {
    this->setParam(key.c_str(), i);
  }

  void 	ParamList::setParam(const char *key, int i) const
  {
    this->nh.setParam(this->paramObjId + key, i);
  }


  void 	ParamList::setParam(const std::string &key, bool b) const
  {
    this->setParam(key.c_str(), b);
  }

  void 	ParamList::setParam(const char *key, bool b) const
  {
    this->nh.setParam(this->paramObjId + key, b);
  }

} // namespace param_

} // namespace as64_
