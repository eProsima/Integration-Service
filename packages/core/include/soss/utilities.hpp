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

#ifndef SOSS__UTILITIES_HPP
#define SOSS__UTILITIES_HPP

#include <soss/Message.hpp>

#include <algorithm>
#include <mutex>
#include <type_traits>
#include <vector>

namespace soss {

//==============================================================================
template<typename Container>
void vector_resize(Container& vector, std::size_t size)
{
  vector.resize(size);
}

template<typename T, std::size_t N>
void vector_resize(std::array<T, N>& /*array*/, std::size_t /*size*/)
{
  // Do nothing. Arrays don't need to be resized.
}

//==============================================================================
template<typename Container>
void vector_reserve(Container& vector, std::size_t size)
{
  vector.reserve(size);
}

template<typename T, std::size_t N>
void vector_reserve(std::array<T, N>& /*array*/, std::size_t /*size*/)
{
  // Do nothing. Arrays don't need to be reserved.
}

//==============================================================================
template<typename Container, typename Arg>
void vector_push_back(Container& vector, Arg&& arg)
{
  vector.push_back(std::forward<Arg>(arg));
}

//==============================================================================
template<typename T, std::size_t N, typename Arg>
void vector_push_back(std::array<T, N>& /*array*/, Arg&& /*arg*/)
{
  // Do nothing. Arrays will never be told to push_back(). This overload only
  // exists to satisfy the compiler.
}

//==============================================================================
/// \brief Convenience function for converting a bounded vector of
/// soss::Messages into a bounded vector of middleware-specific messages.
///
/// To make the limit unbounded, set the UpperBound argument to
/// std::numeric_limits<VectorType::size_type>::max()
///
/// This is effectively a function, but we wrap it in a struct so that it can be
/// used as a template argument.
template<typename ElementType, std::size_t UpperBound>
struct convert_bounded_vector
{
  template<typename FromType, typename ToType,
           typename FromContainer, typename ToContainer>
  static void convert(
      const FromContainer& from,
      ToContainer& to,
      void(*convert)(const FromType& from, ToType& to),
      ToType(*initialize)() = []() { return ToType(); })
  {
    // TODO(MXG): Should we emit a warning when the incoming data exceeds the
    // upper bound?
    const std::size_t N = std::min(from.size(), UpperBound);
    vector_reserve(to, N);
    for(std::size_t i=0; i < N; ++i)
    {
      const FromType& from_msg = from[i];
      if(i < to.size())
      {
        (*convert)(from_msg, to[i]);
      }
      else
      {
        ToType to_msg = (*initialize)();
        (*convert)(from_msg, to_msg);
        vector_push_back(to, std::move(to_msg));
      }
    }

    if(to.size() > N)
      vector_resize(to, N);
  }
};

//==============================================================================
/// \brief This template specialization is needed to deal with the edge case
/// produced by vectors of bools. std::vector<bool> is specialized to be
/// implemented as a bitmap, and as a result its operator[] cannot return its
/// bool elements by reference. Instead it returns a "reference" proxy object,
/// which is not something can be used by the standard convert_bounded_vector
/// function.
template<std::size_t UpperBound>
struct convert_bounded_vector<bool, UpperBound>
{
  template<typename FromContainer, typename ToContainer>
  static void convert(
      const FromContainer& from,
      ToContainer& to,
      void(* /*unused*/ )(const bool& from, bool& to),
      bool(* /*unused*/ )() = []() { return bool(); })
  {
    const std::size_t N = std::min(from.size(), UpperBound);
    vector_resize(to, N);
    std::copy(from.begin(), from.begin() + static_cast<long long int>(N),
              to.begin());
  }
};

//==============================================================================
/// \brief A utility to help with converting data between generic soss Field
/// objects and middleware-specific data structures.
///
/// This struct will work as-is on primitive types (a.k.a. arithmetic types or
/// strings), but you should create a template specialization for converting to
/// or from any complex class types.
template<typename Type>
struct Convert
{
  using native_type = Type;
  using soss_type = native_type;
  using field_iterator = std::map<std::string, Field>::iterator;
  using const_field_iterator = std::map<std::string, Field>::const_iterator;

  static constexpr bool type_is_primitive =
         std::is_arithmetic<Type>::value
      || std::is_same<std::string, Type>::value;

  /// \brief Create an instance of the generic soss version of this type
  ///
  /// For primitive types, this will be the same as the native type
  template<typename... Args>
  static soss_type make_soss(Args&&... args)
  {
    static_assert(type_is_primitive,
      "The soss::Convert struct should be specialized for non-primitive types");
    return soss_type(std::forward<Args>(args)...);
  }

  /// \brief Create a field containing the generic soss version of this type
  template<typename... Args>
  static Field make_soss_field(Args&&... args)
  {
    static_assert(type_is_primitive,
      "The soss::Convert struct should be specialized for non-primitive types");
    return soss::make_field<soss_type>(make_soss(std::forward<Args>(args)...));
  }

  /// \brief Add a field of this type to the specified message
  ///
  /// \param[out] msg
  ///   The message to add the field to
  ///
  /// \param[in] name
  ///   The name that should be given to the field
  static void add_field(soss::Message& msg, const std::string& name)
  {
    static_assert(type_is_primitive,
      "The soss::Convert struct should be specialized for non-primitive types");
    msg.data[name] = make_soss_field();
  }

  /// \brief Move data from a generic soss data structure to a native middleware
  /// data structure
  static void from_soss(const soss_type& from, native_type& to)
  {
    static_assert(type_is_primitive,
      "The soss::Convert struct should be specialized for non-primitive types");
    to = from;
  }

  /// \brief Move data from a generic soss message field to a native middleware
  /// data structure
  static void from_soss_field(const const_field_iterator& from, native_type& to)
  {
    static_assert(type_is_primitive,
      "The soss::Convert struct should be specialized for non-primitive types");
    to = *from->second.cast<soss_type>();
  }

  /// \brief Move data from a native middleware data structure to a generic soss
  /// data structure.
  static void to_soss(const native_type& from, soss_type& to)
  {
    to = from;
  }

  /// \brief Move data from a native middleware data structure to a generic soss
  /// message field.
  static void to_soss_field(const native_type& from, field_iterator to)
  {
    static_assert(type_is_primitive,
      "The soss::Convert struct should be specialized for non-primitive types");
    *to->second.cast<soss_type>() = from;
  }
};

//==============================================================================
/// \brief A class that ensures that low-precision data values will be stored as
/// their higher precision equivalents in the soss::Message, and that low
/// precision values can be assigned from higher precision data within a
/// soss::Message. This helps convert messages between middlewares that care
/// about precision and those that don't.
///
/// The macro SOSS_CONVERT_LOW_PRECISION(H, L) will create the specialization,
/// but most potential low-precision conversions are already provided.
template<typename HighPrecision, typename LowPrecision>
struct LowPrecisionConvert
{
  using native_type = LowPrecision;
  using soss_type = HighPrecision;
  using field_iterator = std::map<std::string, Field>::iterator;
  using const_field_iterator = std::map<std::string, Field>::const_iterator;

  static constexpr bool type_is_primitive =
         std::is_arithmetic<HighPrecision>::value
      || std::is_same<std::string, HighPrecision>::value;

  // Documentation inherited from Convert
  template<typename... Args>
  static soss_type make_soss(Args&&... args)
  {
    return soss_type(std::forward<Args>(args)...);
  }

  // Documentation inherited from Convert
  template<typename... Args>
  static Field make_soss_field(Args&&... args)
  {
    return soss::make_field<soss_type>(make_soss(std::forward<Args>(args)...));
  }

  // Documentation inherited from Convert
  static void add_field(soss::Message& msg, const std::string& name)
  {
    msg.data[name] = make_soss_field();
  }

  // Documentation inherited from Convert
  static void from_soss(const soss_type& from, native_type& to)
  {
    to = static_cast<native_type>(from);
  }

  // Documentation inherited from Convert
  static void from_soss_field(const const_field_iterator& from, native_type& to)
  {
    to = static_cast<native_type>(*from->second.cast<soss_type>());
  }

  // Documentation inherited from Convert
  static void to_soss(const native_type& from, soss_type& to)
  {
    to = from;
  }

  // Documentation inherited from Convert
  static void to_soss_field(const native_type& from, field_iterator to)
  {
    *to->second.cast<soss_type>() = from;
  }
};

#define SOSS_CONVERT_LOW_PRECISION(H, L) \
  template<> struct Convert<L> : LowPrecisionConvert<H, L> { }

SOSS_CONVERT_LOW_PRECISION(double, float);

SOSS_CONVERT_LOW_PRECISION(uint64_t, uint8_t);
SOSS_CONVERT_LOW_PRECISION(uint64_t, uint16_t);
SOSS_CONVERT_LOW_PRECISION(uint64_t, uint32_t);

SOSS_CONVERT_LOW_PRECISION(int64_t, int8_t);
SOSS_CONVERT_LOW_PRECISION(int64_t, int16_t);
SOSS_CONVERT_LOW_PRECISION(int64_t, int32_t);


//==============================================================================
/// \brief A class that helps create a Convert<> specialization for compound
/// message types.
///
/// To create a specialization for a native middleware message type, do the
/// following:
///
/// \code
/// namespace soss {
///
/// template<>
/// struct Convert<native::middleware::type>
///   : soss::MessageConvert<
///        native::middleware::type,
///       &native::middleware::instantiate_type_fnc,
///       &native::middleware::convert_from_soss_fnc,
///       &native::middleware::convert_to_soss_fnc
///   > { };
///
/// } // namespace soss
/// \endcode
///
/// Make sure that this template specialization is put into the root soss
/// namespace.
template<
    typename Type,
    Message (*_initialize_message)(),
    void (*_from_soss)(const soss::Message& from, Type& to),
    void (*_to_soss)(const Type& from, soss::Message& to)>
struct MessageConvert
{
  using native_type = Type;
  using soss_type = Message;
  using field_iterator = std::map<std::string, Field>::iterator;
  using const_field_iterator = std::map<std::string, Field>::const_iterator;

  static constexpr bool type_is_primitive = false;

  // Documentation inherited from Convert
  template<typename... Args>
  static soss_type make_soss(Args&&... args)
  {
    return (*_initialize_message)(std::forward<Args>(args)...);
  }

  // Documentation inherited from Convert
  template<typename... Args>
  static Field make_soss_field(Args&&... args)
  {
    return soss::make_field<soss::Message>(make_soss(std::forward<Args>(args)...));
  }

  // Documentation inherited from Convert
  static void add_field(soss::Message& msg, const std::string& name)
  {
    msg.data[name] = make_soss_field();
  }

  // Documentation inherited from Convert
  static void from_soss(const soss_type& from, native_type& to)
  {
    (*_from_soss)(from, to);
  }

  // Documentation inherited from Convert
  static void from_soss_field(const const_field_iterator& from, native_type& to)
  {
    from_soss(*from->second.cast<soss::Message>(), to);
  }

  // Documentation inherited from Convert
  static void to_soss(const native_type& from, soss_type& to)
  {
    (*_to_soss)(from, to);
  }

  // Documentation inherited from Convert
  static void to_soss_field(const native_type& from, field_iterator to)
  {
    to_soss(from, *to->second.cast<soss::Message>());
  }
};

//==============================================================================
template<
    typename ElementType,
    typename NativeType,
    typename SossType,
    class ContainerConversionImpl>
struct ContainerConvert
{
  using native_type = NativeType;
  using soss_type = SossType;
  using field_iterator = std::map<std::string, Field>::iterator;
  using const_field_iterator = std::map<std::string, Field>::const_iterator;

  static constexpr bool type_is_primitive =
      Convert<ElementType>::type_is_primitive;


  // Documentation inherited from Convert
  template<typename... Args>
  static soss_type make_soss(Args&&... args)
  {
    return soss_type(std::forward<Args>(args)...);
  }

  // Documentation inherited from Convert
  template<typename... Args>
  static Field make_soss_field(Args&&... args)
  {
    return soss::make_field<soss_type>(make_soss(std::forward<Args>(args)...));
  }

  // Documentation inherited from Convert
  static void add_field(soss::Message& msg, const std::string& name)
  {
    msg.data[name] = make_soss_field();
  }

  // Documentation inherited from Convert
  static void from_soss(const soss_type& from, native_type& to)
  {
    ContainerConversionImpl::convert(
          from, to, &Convert<ElementType>::from_soss);
  }

  // Documentation inherited from Convert
  static void from_soss_field(const const_field_iterator& from, native_type& to)
  {
    from_soss(*from->second.cast<soss_type>(), to);
  }

  static void to_soss(const native_type& from, soss_type& to)
  {
    ContainerConversionImpl::convert(
          from, to,
          &Convert<ElementType>::to_soss,
          &Convert<ElementType>::make_soss);
  }

  // Documentation inherited from Convert
  static void to_soss_field(const native_type& from, field_iterator to)
  {
    to_soss(from, *to->second.cast<soss_type>());
  }
};

//==============================================================================
template<typename ElementType, typename Allocator>
struct Convert<std::vector<ElementType, Allocator>>
    : ContainerConvert<
    ElementType,
    std::vector<typename Convert<ElementType>::native_type, Allocator>,
    std::vector<typename Convert<ElementType>::soss_type>,
    soss::convert_bounded_vector<ElementType, std::numeric_limits<
          typename std::vector<ElementType, Allocator>::size_type>::max()>> { };

//==============================================================================
template<typename ElementType, std::size_t N>
struct Convert<std::array<ElementType, N>>
    : ContainerConvert<
    ElementType,
    std::array<typename Convert<ElementType>::native_type, N>,
    std::vector<typename Convert<ElementType>::soss_type>,
    soss::convert_bounded_vector<ElementType, N>> { };

//==============================================================================
/// \brief A thread-safe repository for resources to avoid unnecessary
/// allocations
template<typename Resource, Resource(*initializerT)()>
class ResourcePool
{
public:

  ResourcePool(const std::size_t initial_depth = 1)
  {
    _queue.reserve(initial_depth);
    for(std::size_t i=0; i < initial_depth; ++i)
      _queue.emplace_back((_initializer)());
  }

  void setInitializer(std::function<Resource()> initializer)
  {
    _initializer = std::move(initializer);
  }

  Resource pop()
  {
    if(_queue.empty())
    {
      return (_initializer)();
    }

    std::unique_lock<std::mutex> lock(_mutex);
    Resource r = std::move(_queue.back());
    _queue.pop_back();
    return r;
  }

  void recycle(Resource&& r)
  {
    std::unique_lock<std::mutex> lock(_mutex);
    _queue.emplace_back(std::move(r));
  }


private:

  std::vector<Resource> _queue;
  std::mutex _mutex;
  std::function<Resource()> _initializer = initializerT;

};

//==============================================================================
template<typename Resource>
using UniqueResourcePool =
    ResourcePool<std::unique_ptr<Resource>, &std::make_unique<Resource>>;

template<typename Resource>
std::unique_ptr<Resource> initialize_unique_null() { return nullptr; }

template<typename Resource>
using SharedResourcePool =
    ResourcePool<std::shared_ptr<Resource>, &std::make_shared<Resource>>;

template<typename Resource>
std::shared_ptr<Resource> initialize_shared_null() { return nullptr; }

} // namespace soss

#endif // SOSS__MESSAGEUTILITIES_HPP
