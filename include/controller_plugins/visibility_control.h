#ifndef CONTROLLER_PLUGINS__VISIBILITY_CONTROL_H_
#define CONTROLLER_PLUGINS__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CONTROLLER_PLUGINS_EXPORT __attribute__ ((dllexport))
    #define CONTROLLER_PLUGINS_IMPORT __attribute__ ((dllimport))
  #else
    #define CONTROLLER_PLUGINS_EXPORT __declspec(dllexport)
    #define CONTROLLER_PLUGINS_IMPORT __declspec(dllimport)
  #endif
  #ifdef CONTROLLER_PLUGINS_BUILDING_LIBRARY
    #define CONTROLLER_PLUGINS_PUBLIC CONTROLLER_PLUGINS_EXPORT
  #else
    #define CONTROLLER_PLUGINS_PUBLIC CONTROLLER_PLUGINS_IMPORT
  #endif
  #define CONTROLLER_PLUGINS_PUBLIC_TYPE CONTROLLER_PLUGINS_PUBLIC
  #define CONTROLLER_PLUGINS_LOCAL
#else
  #define CONTROLLER_PLUGINS_EXPORT __attribute__ ((visibility("default")))
  #define CONTROLLER_PLUGINS_IMPORT
  #if __GNUC__ >= 4
    #define CONTROLLER_PLUGINS_PUBLIC __attribute__ ((visibility("default")))
    #define CONTROLLER_PLUGINS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define CONTROLLER_PLUGINS_PUBLIC
    #define CONTROLLER_PLUGINS_LOCAL
  #endif
  #define CONTROLLER_PLUGINS_PUBLIC_TYPE
#endif

#endif  // CONTROLLER_PLUGINS__VISIBILITY_CONTROL_H_
