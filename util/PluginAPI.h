

#ifndef __UTIL_GLOBALS_H__
#define __UTIL_GLOBALS_H__


#if defined(_WIN32)
#  if defined(PLUGIN_API)
#    define PLUGIN_EXPORT __declspec(dllexport)
#  else
#    define PLUGIN_EXPORT __declspec(dllimport)
#  endif
#else
#  define PLUGIN_EXPORT
#endif


#endif
