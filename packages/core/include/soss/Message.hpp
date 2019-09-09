/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef SOSS__MESSAGE_HPP
#define SOSS__MESSAGE_HPP


#include <soss/core/export.hpp>
#include <functional>
#include <map>
#include <memory>
#include <string>

namespace soss {

//==============================================================================
class SOSS_CORE_API Field
{
public:

  /// \brief Construct an empty field
  Field();
  Field(const Field& other);
  Field(Field&& other);
  Field& operator=(const Field& other);
  Field& operator=(Field&& other);

  /// \brief Set this field to type T. Any previous data in this field will be
  /// lost.
  /// \param[in] data
  ///   The data to set this field to.
  template<typename T>
  void set(T&& data);

  /// \brief Cast this field to the requested type.
  /// If this field does not contain the requested type, this will be a nullptr.
  template<typename T>
  T* cast();

  /// \brief Cast this field to the requested type with const-qualifications.
  /// If this field does not contain the requested type, this will be a nullptr.
  template <typename T>
  const T* cast() const;

  /// \brief Get the type of this field
  std::string type() const;

  /// \brief Destructor
  ~Field();

private:

  /// \brief Implementation of Field::set() function
  void _set(const std::type_info& type, void* data,
            std::function<void*(const void*)> &&copier,
            std::function<void(void*)>&& deleter);

  /// \brief Implementation of Field::cast() function
  void* _cast(const std::type_info& type);

  /// \brief Implementation of Field::cast() const function.
  const void* _cast(const std::type_info& type) const;

  class Implementation;
  std::unique_ptr<Implementation> _pimpl;
};

//==============================================================================
/// Because of C++ language limitations, you cannot explicitly specify template
/// parameters when using a constructor template to instantiate a class.
/// We provide this convenience function to overcome that limition.
template<typename T, typename... Args>
Field make_field(Args&&... data)
{
  Field field;
  field.set(T(std::forward<Args>(data)...));
  return field;
}

//==============================================================================
class Message
{
public:

  // TODO(MXG): This implementation is a first draft proof-of-concept. After we
  // are satisfied with the first proof-of-concept, we should consider a better
  // encapsulation design that doesn't expose so much implementation detail.
  // E.g. we should use a PIMPL design pattern to ensure that we can maintain a
  // stable ABI

  /// The string that uniquely defines this message type
  std::string type;

  using FieldMap = std::map<std::string, Field>;
  using iterator = FieldMap::iterator;
  using const_iterator = FieldMap::const_iterator;

  /// The data that this message contains. This will point to a middleware
  /// neutral type defined in a soss library. Each middleware will have its own
  /// functions for converting to/from this type.
  FieldMap data;

};

} // namespace soss

#include <soss/detail/Message-impl.hpp>

#endif // SOSS__MESSAGE_HPP
