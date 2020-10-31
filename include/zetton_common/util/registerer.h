#pragma once

#include <map>
#include <string>
#include <vector>

#include "ros/ros.h"

namespace zetton {
namespace common {

// idea from boost any but make it more simple and don't use type_info.
class Any {
 public:
  Any() : content_(NULL) {}

  template <typename ValueType>
  explicit Any(const ValueType &value)
      : content_(new Holder<ValueType>(value)) {}

  Any(const Any &other)
      : content_(other.content_ ? other.content_->Clone() : nullptr) {}

  ~Any() { delete content_; }

  template <typename ValueType>
  ValueType *AnyCast() {
    return content_ ? &(static_cast<Holder<ValueType> *>(content_)->held_)
                    : nullptr;
  }

 private:
  class PlaceHolder {
   public:
    virtual ~PlaceHolder() {}
    virtual PlaceHolder *Clone() const = 0;
  };

  template <typename ValueType>
  class Holder : public PlaceHolder {
   public:
    explicit Holder(const ValueType &value) : held_(value) {}
    virtual ~Holder() {}
    virtual PlaceHolder *Clone() const { return new Holder(held_); }

    ValueType held_;
  };

  PlaceHolder *content_;
};

class ObjectFactory {
 public:
  ObjectFactory() {}
  virtual ~ObjectFactory() {}
  virtual Any NewInstance() { return Any(); }
  ObjectFactory(const ObjectFactory &) = delete;
  ObjectFactory &operator=(const ObjectFactory &) = delete;

 private:
};

typedef std::map<std::string, ObjectFactory *> FactoryMap;
typedef std::map<std::string, FactoryMap> BaseClassMap;
BaseClassMap &GlobalFactoryMap();

bool GetRegisteredClasses(
    const std::string &base_class_name,
    std::vector<std::string> *registered_derived_classes_names);

#define ZETTON_REGISTER_REGISTERER(base_class)                             \
  class base_class##Registerer {                                           \
    typedef ::zetton::common::Any Any;                                     \
    typedef ::zetton::common::FactoryMap FactoryMap;                       \
                                                                           \
   public:                                                                 \
    static base_class *GetInstanceByName(const ::std::string &name) {      \
      FactoryMap &map = ::zetton::common::GlobalFactoryMap()[#base_class]; \
      FactoryMap::iterator iter = map.find(name);                          \
      if (iter == map.end()) {                                             \
        for (auto c : map) {                                               \
          ROS_ERROR_STREAM("Instance:" << c.first);                        \
        }                                                                  \
        ROS_ERROR_STREAM("Get instance " << name << " failed.");           \
        return nullptr;                                                    \
      }                                                                    \
      Any object = iter->second->NewInstance();                            \
      return *(object.AnyCast<base_class *>());                            \
    }                                                                      \
    static std::vector<base_class *> GetAllInstances() {                   \
      std::vector<base_class *> instances;                                 \
      FactoryMap &map = ::zetton::common::GlobalFactoryMap()[#base_class]; \
      instances.reserve(map.size());                                       \
      for (auto item : map) {                                              \
        Any object = item.second->NewInstance();                           \
        instances.push_back(*(object.AnyCast<base_class *>()));            \
      }                                                                    \
      return instances;                                                    \
    }                                                                      \
    static const ::std::string GetUniqInstanceName() {                     \
      FactoryMap &map = ::zetton::common::GlobalFactoryMap()[#base_class]; \
      if (1 != map.size()) {                                               \
        ROS_FATAL_STREAM("map.size() != 1 (" << map.size() << "vs 1)");    \
      }                                                                    \
      return map.begin()->first;                                           \
    }                                                                      \
    static base_class *GetUniqInstance() {                                 \
      FactoryMap &map = ::zetton::common::GlobalFactoryMap()[#base_class]; \
      if (1 != map.size()) {                                               \
        ROS_FATAL_STREAM("map.size() != 1 (" << map.size() << "vs 1)");    \
      }                                                                    \
      Any object = map.begin()->second->NewInstance();                     \
      return *(object.AnyCast<base_class *>());                            \
    }                                                                      \
    static bool IsValid(const ::std::string &name) {                       \
      FactoryMap &map = ::zetton::common::GlobalFactoryMap()[#base_class]; \
      return map.find(name) != map.end();                                  \
    }                                                                      \
  };

#define ZETTON_REGISTER_CLASS(clazz, name)                                    \
  namespace {                                                                 \
  class ObjectFactory##name : public zetton::common::ObjectFactory {          \
   public:                                                                    \
    virtual ~ObjectFactory##name() {}                                         \
    virtual ::zetton::common::Any NewInstance() {                             \
      return ::zetton::common::Any(new name());                               \
    }                                                                         \
  };                                                                          \
  __attribute__((constructor)) void RegisterFactory##name() {                 \
    ::zetton::common::FactoryMap &map =                                       \
        ::zetton::common::GlobalFactoryMap()[#clazz];                         \
    if (map.find(#name) == map.end()) map[#name] = new ObjectFactory##name(); \
  }                                                                           \
  }

}  // namespace common
}  // namespace zetton
