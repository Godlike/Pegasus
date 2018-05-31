/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_DEBUG_HPP
#define PEGASUS_DEBUG_HPP

#ifdef PEGASUS_DEBUG
#include <pegasus/debug/DebugImplementation.hpp>
#else
#include <pegasus/debug/DebugDummy.hpp>
#endif

namespace pegasus
{

#ifdef PEGASUS_DEBUG
    using Debug = DebugImplementation;
#else
    using Debug = DebugDummy;
#endif

} // namespace pegasus

#endif // PEGASUS_DEBUG_HPP
