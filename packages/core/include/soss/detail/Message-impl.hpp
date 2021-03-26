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

#ifndef SOSS__DETAIL__MESSAGEIMPL_HPP
#define SOSS__DETAIL__MESSAGEIMPL_HPP

#include <soss/Message.hpp>

#include <type_traits>
#include <limits>

namespace soss {

namespace detail {

//==============================================================================
template<typename T>
using Raw_t = std::remove_cv_t<std::remove_reference_t<T> >;

} // namespace detail

//==============================================================================
template<typename T>
void Field::set(
        T&& data)
{
    _set(typeid(detail::Raw_t<T>),
            new detail::Raw_t<T>(std::forward<T>(data)),
            [](const void* other) -> void*
            {
                return new detail::Raw_t<T>(
                    *static_cast<const detail::Raw_t<T>*>(other));
            },
            [](void* expired)
            {
                delete static_cast<detail::Raw_t<T>*>(expired);
            });
}

//==============================================================================
template<typename T>
T* Field::cast(
        std::enable_if_t<std::is_integral<T>::value>*)
{
    // call the const-qualified overload
    return const_cast<T*>(const_cast<const Field*>(this)->cast<T>());
}

//==============================================================================
template<typename T>
T* Field::cast(
        std::enable_if_t<!std::is_integral<T>::value>*)
{
    return static_cast<T*>(_cast(typeid(detail::Raw_t<T>)));
}

//==============================================================================
template<typename T>
const T* Field::cast(
        std::enable_if_t<std::is_integral<T>::value>*) const
{
    if (type() == typeid(int64_t).name())
    {
        return _integral_cast<const T, int64_t>();
    }
    else if (type() == typeid(uint64_t).name())
    {
        return _integral_cast<const T, uint64_t>();
    }
    else if (type() == typeid(int32_t).name())
    {
        return _integral_cast<const T, int32_t>();
    }
    else if (type() == typeid(uint32_t).name())
    {
        return _integral_cast<const T, uint32_t>();
    }
    else if (type() == typeid(int16_t).name())
    {
        return _integral_cast<const T, int16_t>();
    }
    else if (type() == typeid(uint16_t).name())
    {
        return _integral_cast<const T, uint16_t>();
    }
    else if (type() == typeid(int8_t).name())
    {
        return _integral_cast<const T, int8_t>();
    }
    else if (type() == typeid(uint8_t).name())
    {
        return _integral_cast<const T, uint8_t>();
    }
    else if (type() == typeid(bool).name())
    {
        return _integral_cast<const T, bool>();
    }
    else
    {
        return nullptr;
    }
}

//==============================================================================
template<typename T>
const T* Field::cast(
        std::enable_if_t<!std::is_integral<T>::value>*) const
{
    return static_cast<const T*>(_cast(typeid(detail::Raw_t<T>)));
}

//==============================================================================
template<typename T, typename InnerT>
const T* Field::_integral_cast() const
{
    auto* inner = static_cast<const InnerT*>(_cast(typeid(InnerT)));
    if (!inner)
    {
        return nullptr;
    }

    if (static_cast<T>(*inner) > std::numeric_limits<T>::max()
            || static_cast<T>(*inner) < std::numeric_limits<T>::min())
    {
        return nullptr;
    }
    return reinterpret_cast<T*>(inner);
}

} // namespace soss

#endif // SOSS__DETAIL__MESSAGEIMPL_HPP
