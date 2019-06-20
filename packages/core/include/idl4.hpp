// ================================ idl4 lib ===================================

#ifndef LIB__IDL4_HPP
#define LIB__IDL4_HPP

#include <xtypes.hpp>

#include <map>
#include <string>

namespace idl4
{

  static std::map<std::string, xtypes::DynamicType> compile(const std::string& /*file_path*/)
  {
      return std::map<std::string, xtypes::DynamicType>();
  }

}

#endif //LIB__IDL4_HPP
