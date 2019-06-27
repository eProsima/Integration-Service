// ================================ xtypes lib ==================================

#ifndef LIB__XTYPES_HPP
#define LIB__XTYPES_HPP

#include<string>

namespace xtypes
{


class DynamicType
{
public:
    DynamicType(const std::string& name)
        :name(name)
    {
    }

    bool is_subset_of(const DynamicType& /*other*/) const
    {
      return true; //Check compatibility by QoS
    }

    const std::string& get_name() const
    {
        return name;
    }

private:
    std::string name;
};


class DynamicData
{
public:
    DynamicData(const DynamicType& type)
        : type(type)
    {
    }

    const DynamicType& get_type() const
    {
        return type;
    }

private:
    const DynamicType& type;
};


} //xtypes

#endif //LIB__XTYPES_HPP
