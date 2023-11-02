// Copyright (c) 2021, ROAS Inc.
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CLOBER_HARDWARE__VISIBILITY_CONTROL_H_
#define CLOBER_HARDWARE__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CLOBER_HARDWARE_EXPORT __attribute__((dllexport))
    #define CLOBER_HARDWARE_IMPORT __attribute__((dllimport))
  #else
    #define CLOBER_HARDWARE_EXPORT __declspec(dllexport)
    #define CLOBER_HARDWARE_IMPORT __declspec(dllimport)
  #endif
  #ifdef CLOBER_HARDWARE_BUILDING_DLL
    #define CLOBER_HARDWARE_PUBLIC CLOBER_HARDWARE_EXPORT
  #else
    #define CLOBER_HARDWARE_PUBLIC CLOBER_HARDWARE_IMPORT
  #endif
    #define CLOBER_HARDWARE_PUBLIC_TYPE CLOBER_HARDWARE_PUBLIC
    #define CLOBER_HARDWARE_LOCAL
  #else
    #define CLOBER_HARDWARE_EXPORT __attribute__((visibility("default")))
    #define CLOBER_HARDWARE_IMPORT
  #if __GNUC__ >= 4
    #define CLOBER_HARDWARE_PUBLIC __attribute__((visibility("default")))
    #define CLOBER_HARDWARE_LOCAL __attribute__((visibility("hidden")))
  #else
    #define CLOBER_HARDWARE_PUBLIC
    #define CLOBER_HARDWARE_LOCAL
  #endif
  #define CLOBER_HARDWARE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // CLOBER_HARDWARE__VISIBILITY_CONTROL_H_