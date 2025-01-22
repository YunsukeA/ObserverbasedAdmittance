#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define ObserverbasedAdmittance_DLLIMPORT __declspec(dllimport)
#  define ObserverbasedAdmittance_DLLEXPORT __declspec(dllexport)
#  define ObserverbasedAdmittance_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define ObserverbasedAdmittance_DLLIMPORT __attribute__((visibility("default")))
#    define ObserverbasedAdmittance_DLLEXPORT __attribute__((visibility("default")))
#    define ObserverbasedAdmittance_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define ObserverbasedAdmittance_DLLIMPORT
#    define ObserverbasedAdmittance_DLLEXPORT
#    define ObserverbasedAdmittance_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef ObserverbasedAdmittance_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define ObserverbasedAdmittance_DLLAPI
#  define ObserverbasedAdmittance_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef ObserverbasedAdmittance_EXPORTS
#    define ObserverbasedAdmittance_DLLAPI ObserverbasedAdmittance_DLLEXPORT
#  else
#    define ObserverbasedAdmittance_DLLAPI ObserverbasedAdmittance_DLLIMPORT
#  endif // ObserverbasedAdmittance_EXPORTS
#  define ObserverbasedAdmittance_LOCAL ObserverbasedAdmittance_DLLLOCAL
#endif // ObserverbasedAdmittance_STATIC