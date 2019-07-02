// ================================ xtypes lib ==================================

#ifndef LIB__XTYPES_HPP
#define LIB__XTYPES_HPP

#include<string>
#include<map>

namespace xtypes
{


class DynamicType
{
public:
    enum class Type { INT, STRING };

    DynamicType(const std::string& name)
        :name_(name)
    {
    }

    ~DynamicType() = default;

    bool is_subset_of(const DynamicType& /*other*/) const
    {
      return true; //Check compatibility by QoS
    }

    const std::string& get_name() const
    {
        return name_;
    }

    Type& operator [] (const std::string field)
    {
        return members_[field];
    }

    const std::map<std::string, Type>& get_members() const
    {
        return members_;
    }

private:
    std::string name_;
    std::map<std::string, Type> members_;
};



class DynamicData
{
public:
    class Iterator { };

    DynamicData(const DynamicType& type)
        : type_(type)
    {
    }

    ~DynamicData() = default;

    const DynamicType& get_type() const
    {
        return type_;
    }

    std::string& operator [] (const std::string field)
    {
        return values_[field];
    }

    const std::string& operator [] (const std::string field) const
    {
        return values_.at(field);
    }

    const std::map<std::string, std::string>& get_values() const
    {
        return values_;
    }

    DynamicData::Iterator get_iterator() const
    {
        //TODO
        return Iterator();
    }

private:
    const DynamicType& type_;
    std::map<std::string, std::string> values_;
};


} //xtypes

#endif //LIB__XTYPES_HPP
