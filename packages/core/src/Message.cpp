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

#include <soss/Message.hpp>

namespace soss {

//==============================================================================
class Field::Implementation
{
public:

    Implementation()
        : _type(nullptr)
        , _data(nullptr)
    {
        // Do nothing
    }

    Implementation(
            const Implementation& other)
    {
        *this = other;
    }

    Implementation& operator =(
            const Implementation& other)
    {
        _type = other._type;
        _data = other.clone_data();
        _copier = other._copier;
        _deleter = other._deleter;
        return *this;
    }

    void* clone_data() const
    {
        return _copier(_data);
    }

    void _set(
            const std::type_info& type,
            void* data,
            std::function<void* (const void*)>&& copier,
            std::function<void(void*)>&& deleter)
    {
        _type = &type;
        _data = data;
        _copier = copier;
        _deleter = deleter;
    }

    void* _cast(
            const std::type_info& type) const
    {
        // This means we have an empty field
        if (!_type)
        {
            return nullptr;
        }

        // TODO(MXG): If we ever need to support windows, we probably cannot rely
        // on this check
        if (*_type == type)
        {
            return _data;
        }

        // If the type check failed, then we should return a nullptr.
        return nullptr;
    }

    std::string type() const
    {
        // TODO(MXG): Consider demangling this typename
        if (_type)
        {
            return _type->name();
        }

        return "empty";
    }

    ~Implementation()
    {
        if (_deleter && _data)
        {
            _deleter(_data);
        }
    }

private:

    const std::type_info* _type;
    void* _data;
    std::function<void* (const void*)> _copier;
    std::function<void(void*)> _deleter;

};

//==============================================================================
Field::Field()
    : _pimpl(new Implementation)
{
    // Do nothing
}

//==============================================================================
Field::Field(
        const Field& other)
    : _pimpl(new Implementation(*other._pimpl))
{
    // Do nothing
}

//==============================================================================
Field::Field(
        Field&& other)
    : _pimpl(std::move(other._pimpl))
{
    // Do nothing
}

//==============================================================================
Field& Field::operator =(
        const Field& other)
{
    *_pimpl = *other._pimpl;
    return *this;
}

//==============================================================================
Field& Field::operator =(
        Field&& other)
{
    _pimpl = std::move(other._pimpl);
    return *this;
}

//==============================================================================
std::string Field::type() const
{
    return _pimpl->type();
}

//==============================================================================
Field::~Field()
{
    // Do nothing
}

//==============================================================================
void Field::_set(
        const std::type_info& type,
        void* data,
        std::function<void* (const void*)>&& copier,
        std::function<void(void*)>&& deleter)
{
    _pimpl->_set(type, data, std::move(copier), std::move(deleter));
}

//==============================================================================
void* Field::_cast(
        const std::type_info& type)
{
    return _pimpl->_cast(type);
}

//==============================================================================
const void* Field::_cast(
        const std::type_info& type) const
{
    return _pimpl->_cast(type);
}

} // namespace soss


