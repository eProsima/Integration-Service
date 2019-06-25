// ================================ xtypes lib ==================================

#ifndef LIB__XTYPES_HPP
#define LIB__XTYPES_HPP

namespace xtypes
{

  class DynamicType
  {
    bool is_subset(const DynamicType& /*other*/) const
    {
      return true; //Check compatibility by QoS
    }
  };

  class DynamicData
  {
  };

}

#endif //LIB__XTYPES_HPP
