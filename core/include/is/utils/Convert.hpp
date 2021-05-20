/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 * Copyright (C) 2020 - present Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef _IS_UTILS_CONVERT_HPP_
#define _IS_UTILS_CONVERT_HPP_

#include <is/core/Message.hpp>

#include <algorithm>
#include <mutex>
#include <type_traits>
#include <vector>

#include <boost/array.hpp>

namespace eprosima {
namespace is {
namespace utils {

//==============================================================================
/**
 * @brief A utility to help with converting data between generic DynamicData
 *        field objects and middleware-specific data structures.
 *
 *        This struct will work as-is on primitive types (a.k.a. arithmetic types or
 *        strings), but a template specialization for converting to
 *        or from any complex class types should be created.
 */
template<typename Type>
struct Convert
{
    /**
     * @brief Alias for the Type.
     */
    using native_type = Type;

    /**
     * @brief Const expression to check if the type is primitive or not.
     */
    static constexpr bool type_is_primitive =
            std::is_arithmetic<Type>::value
            || std::is_same<std::string, Type>::value
            || std::is_same<std::basic_string<char16_t>, Type>::value;

    /**
     * @brief Move data from a xTypes DynamicData field to a native middleware data structure.
     *
     * @param[in] from A readable reference to the DynamicData field to be transferred.
     * @param[in] to The destination native middleware data structure.
     */
    static void from_xtype_field(
            const xtypes::ReadableDynamicDataRef& from,
            native_type& to)
    {
        to = from.value<native_type>();
    }

    /**
     * @brief Move data from a native middleware data structure to a xTypes DynamicData field.
     *
     * @param[in] from A readable reference to the middleware native data structure to be transferred.
     * @param[in] to A writable reference to the target DynamicData field.
     */
    static void to_xtype_field(
            const native_type& from,
            xtypes::WritableDynamicDataRef to)
    {
        static_assert(type_is_primitive,
                "The is::utils::Convert struct should be specialized for non-primitive types");
        to.value<native_type>(from);
    }

};

//==============================================================================
/**
 * @brief A class that helps create a Convert<> specialization for managing some
 *        char issues.
 *
 * @details 'rosidl' parse 'char' types as 'signed' values from 'msg' files and
 *          parse as 'unsigned' from idl files. This create a mismatched between types.
 *          This patch solves this issue when the native type differs from the
 *          DynamicData type for this specific case.
 *
 * @note This specialization can be removed safety if rosidl modifies its behavior.
 */
struct CharConvert
{
    /**
     * @brief Alias for the Type.
     */
    using native_type = char;

    static constexpr bool type_is_primitive = true;

    /**
     * @brief Documentation inherited from Convert.
     */
    static void from_xtype_field(
            const xtypes::ReadableDynamicDataRef& from,
            native_type& to)
    {
        if (from.type().kind() == xtypes::TypeKind::UINT_8_TYPE)
        {
            to = static_cast<native_type>(from.value<uint8_t>());
        }
        else
        {
            to = from.value<native_type>();
        }
    }

    /**
     * @brief Documentation inherited from Convert.
     */
    static void to_xtype_field(
            const native_type& from,
            xtypes::WritableDynamicDataRef to)
    {
        if (to.type().kind() == xtypes::TypeKind::UINT_8_TYPE)
        {
            to.value<uint8_t>(static_cast<uint8_t>(from));
        }
        else
        {
            to.value<native_type>(from);
        }
    }

};

template<>
struct Convert<char> : CharConvert { };

//==============================================================================
/**
 * @brief A class that helps create a Convert<> specialization for
 *        compound message types.
 *
 * @details To create a specialization for a native middleware message type,
 *          do the following:
 *
 * @code{.cpp}
 * namespace eprosima {
 * namespace is {
 * namespace utils {
 *
 * template<>
 * struct Convert<native::middleware::type>
 *     : MessageConvert<
 *         native::middleware::type,
 *         &native::middleware::convert_from_xtype_fnc,
 *         &native::middleware::convert_to_xtype_fnc
 *     > { };
 *
 * } // namespace utils
 * } // namespace is
 * } // namespace eprosima
 * @endcode
 *
 * @note Make sure that this template specialization is put into the
 *       root Integration Service namespace.
 */
template<
    typename Type,
    void (* _from_xtype)(const xtypes::ReadableDynamicDataRef& from, Type& to),
    void (* _to_xtype)(const Type& from, xtypes::WritableDynamicDataRef to)>
struct MessageConvert
{
    using native_type = Type;

    static constexpr bool type_is_primitive = false;

    /**
     * @brief Documentation inherited from Convert.
     */
    static void from_xtype_field(
            const xtypes::ReadableDynamicDataRef& from,
            native_type& to)
    {
        (*_from_xtype)(from, to);
    }

    /**
     * @brief Documentation inherited from Convert.
     */
    static void to_xtype_field(
            const native_type& from,
            xtypes::WritableDynamicDataRef to)
    {
        (*_to_xtype)(from, to);
    }

};

//==============================================================================
/**
 * @brief A class that helps create a Convert<> specialization for
 *        resizable unbounded container message types.
 *
 * @details To create a specialization for a native middleware message type,
 *          do the following:
 *
 * @code{.cpp}
 * namespace eprosima {
 * namespace is {
 * namespace utils {
 *
 * template<typename ElementType, typename Allocator>
 * struct Convert<native::middleware::type<ElementType, Allocator> >
 *     : ResizableUnboundedContainerConvert<
 *         ElementType,
 *         native::middleware::type,
 *         Allocator,
 *         size_t::upperbound::limit
 *     > { };
 *
 * } // namespace utils
 * } // namespace is
 * } // namespace eprosima
 * @endcode
 *
 * @note The UpperBound limit could typically be calculated as
 *       @code{.cpp}
 *       std::numeric_limits<typename native::middleware::type<ElementType, Allocator>::size_type>::max()
 *       @endcode
 */
template<
    typename ElementType,
    template <typename, typename> class NativeType,
    typename Allocator,
    std::size_t UpperBound,
    std::enable_if_t<std::is_base_of<std::vector<ElementType, Allocator>, NativeType<ElementType, Allocator> >::value,
    bool> = true>
struct ResizableUnboundedContainerConvert
{
    using native_type = NativeType<ElementType, Allocator>;

    static constexpr bool type_is_primitive =
            Convert<ElementType>::type_is_primitive;

    static void from_xtype(
            const xtypes::ReadableDynamicDataRef& from,
            ElementType& to)
    {
        Convert<ElementType>::from_xtype_field(from, to);
    }

    /**
     * @brief This template specialization is needed to deal with the edge case
     *        produced by vectors of bools. std::vector<bool> is specialized to be
     *        implemented as a bitmap, and as a result its operator[] cannot return its
     *        bool elements by reference. Instead it returns a "reference" proxy object.
     */
    static void from_xtype(
            const xtypes::ReadableDynamicDataRef& from,
            std::vector<bool>::reference to)
    {
        bool temp = from;
        to = temp;
    }

    /**
     * @brief Documentation inherited from Convert.
     */
    static void from_xtype_field(
            const xtypes::ReadableDynamicDataRef& from,
            native_type& to)
    {
        std::size_t N = std::min(from.size(), UpperBound);
        to.resize(N);
        for (std::size_t i = 0; i < N; ++i)
        {
            from_xtype(from[i], to[i]);
        }
    }

    /**
     * @brief Documentation inherited from Convert.
     */
    static void to_xtype_field(
            const native_type& from,
            xtypes::WritableDynamicDataRef to)
    {
        const std::size_t N = std::min(from.size(), UpperBound);
        to.resize(N);
        for (std::size_t i = 0; i < N; ++i)
        {
            Convert<ElementType>::to_xtype_field(from[i], to[i]);
        }
    }

};

template<typename ElementType, typename Allocator>
struct Convert<std::vector<ElementType, Allocator> >
    : ResizableUnboundedContainerConvert<
        ElementType,
        std::vector,
        Allocator,
        std::numeric_limits<typename std::vector<ElementType, Allocator>::size_type>::max()>
{
};

//==============================================================================
/**
 * @brief A class that helps create a Convert<> specialization for
 *        resizable bounded container message types.
 *
 * @details To create a specialization for a native middleware message type,
 *          do the following:
 *
 * @code{.cpp}
 * namespace eprosima {
 * namespace is {
 * namespace utils {
 *
 * template<typename ElementType, std::size_t N, typename Allocator, template<typename, std::size_t, typename> class VectorImpl>
 * struct Convert<VectorImpl<ElementType, N, Allocator> >
 *     : ResizableBoundedContainerConvert<
 *         ElementType,
 *         VectorImpl,
 *         Allocator,
 *         N
 *     > { };
 *
 * } // namespace utils
 * } // namespace is
 * } // namespace eprosima
 * @endcode
 */
template<
    typename ElementType,
    template <typename, std::size_t, typename> class NativeType,
    typename Allocator,
    std::size_t UpperBound>
struct ResizableBoundedContainerConvert
{
    using native_type = NativeType<ElementType, UpperBound, Allocator>;

    static constexpr bool type_is_primitive =
            Convert<ElementType>::type_is_primitive;

    static void from_xtype(
            const xtypes::ReadableDynamicDataRef& from,
            ElementType& to)
    {
        Convert<ElementType>::from_xtype_field(from, to);
    }

    /**
     * @brief Documentation inherited from Convert.
     */
    static void from_xtype_field(
            const xtypes::ReadableDynamicDataRef& from,
            native_type& to)
    {
        std::size_t N = std::min(from.size(), UpperBound);
        to.resize(N);
        for (std::size_t i = 0; i < N; ++i)
        {
            from_xtype(from[i], to[i]);
        }
    }

    /**
     * @brief Documentation inherited from Convert.
     */
    static void to_xtype_field(
            const native_type& from,
            xtypes::WritableDynamicDataRef to)
    {
        const std::size_t N = std::min(from.size(), UpperBound);
        to.resize(N);
        for (std::size_t i = 0; i < N; ++i)
        {
            Convert<ElementType>::to_xtype_field(from[i], to[i]);
        }
    }

};

template<typename ElementType, std::size_t N, typename Allocator,
        template<typename, std::size_t, typename> class VectorImpl>
struct Convert<VectorImpl<ElementType, N, Allocator> >
    : ResizableBoundedContainerConvert<
        ElementType,
        VectorImpl,
        Allocator,
        N>
{
};

//==============================================================================
/**
 * @brief A class that helps create a Convert<> specialization for
 *        non resizable container message types.
 *
 * @details To create a specialization for a native middleware message type,
 *          do the following:
 *
 * @code{.cpp}
 * namespace eprosima {
 * namespace is {
 * namespace utils {
 *
 * template<template <typename, std::size_t> class Array, typename ElementType, std::size_t N>
 * struct Convert<Array<ElementType, N> >
 *     : NonResizableContainerConvert<
 *         ElementType,
 *         Array,
 *         N
 *     > { };
 *
 * } // namespace utils
 * } // namespace is
 * } // namespace eprosima
 * @endcode
 */
template<
    typename ElementType,
    template <typename, std::size_t> class NativeType,
    std::size_t UpperBound,
    std::enable_if_t<std::is_base_of<std::array<ElementType, UpperBound>, NativeType<ElementType, UpperBound> >::value
    || std::is_base_of<boost::array<ElementType, UpperBound>, NativeType<ElementType, UpperBound> >::value,
    bool> = true>
struct NonResizableContainerConvert
{
    using native_type = NativeType<ElementType, UpperBound>;

    static constexpr bool type_is_primitive =
            Convert<ElementType>::type_is_primitive;

    static void from_xtype(
            const xtypes::ReadableDynamicDataRef& from,
            ElementType& to)
    {
        Convert<ElementType>::from_xtype_field(from, to);
    }

    /**
     * @brief Documentation inherited from Convert.
     */
    static void from_xtype_field(
            const xtypes::ReadableDynamicDataRef& from,
            native_type& to)
    {
        std::size_t N = std::min(from.size(), UpperBound);
        for (std::size_t i = 0; i < N; ++i)
        {
            from_xtype(from[i], to[i]);
        }
    }

    /**
     * @brief Documentation inherited from Convert.
     */
    static void to_xtype_field(
            const native_type& from,
            xtypes::WritableDynamicDataRef to)
    {
        const std::size_t N = std::min(from.size(), UpperBound);
        for (std::size_t i = 0; i < N; ++i)
        {
            Convert<ElementType>::to_xtype_field(from[i], to[i]);
        }
    }

};

template<template <typename, std::size_t> class Array, typename ElementType, std::size_t N>
struct Convert<Array<ElementType, N> >
    : NonResizableContainerConvert<
        ElementType,
        Array,
        N>
{
};

//==============================================================================
/**
 * @brief A thread-safe repository for resources to avoid unnecessary allocations.
 */
template<typename Resource, Resource(* initializerT)()>
class ResourcePool
{
public:

    ResourcePool(
            const std::size_t initial_depth = 1)
    {
        _queue.reserve(initial_depth);
        for (std::size_t i = 0; i < initial_depth; ++i)
        {
            _queue.emplace_back((_initializer)());
        }
    }

    void setInitializer(
            std::function<Resource()> initializer)
    {
        _initializer = std::move(initializer);
    }

    Resource pop()
    {
        if (_queue.empty())
        {
            return (_initializer)();
        }

        std::unique_lock<std::mutex> lock(_mutex);
        Resource r = std::move(_queue.back());
        _queue.pop_back();
        return r;
    }

    void recycle(
            Resource&& r)
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
        ResourcePool<std::unique_ptr<Resource>, &std::make_unique<Resource> >;

template<typename Resource>
std::unique_ptr<Resource> initialize_unique_null()
{
    return nullptr;
}

template<typename Resource>
using SharedResourcePool =
        ResourcePool<std::shared_ptr<Resource>, &std::make_shared<Resource> >;

template<typename Resource>
std::shared_ptr<Resource> initialize_shared_null()
{
    return nullptr;
}

} //  namespace utils
} //  namespace is
} //  namespace eprosima

#endif //  _IS_UTILS_CONVERT_HPP_
