/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT) (http://opensource.org/licenses/MIT)
*/

#ifndef PEGASUS_SHARED_MACROS_HPP
#define PEGASUS_SHARED_MACROS_HPP

/**
 *  @brief CMake will define PegasusPhysics_EXPORTS when building the library.
 */

#if defined(_WIN32) && defined(PEGASUS_SHARED)
#ifdef PegasusPhysics_EXPORTS
#define PEGASUS_EXPORT __declspec(dllexport)
#else
#define PEGASUS_EXPORT __declspec(dllimport)
#endif
#else
#define PEGASUS_EXPORT
#endif

#endif // PEGASUS_SHARED_MACROS_HPP
