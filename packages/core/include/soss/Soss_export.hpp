#ifndef MACRO_SOSS_EXPORT_WIN32
#define MACRO_SOSS_EXPORT_WIN32

#if defined(_WIN32)
#define SYSTEM_HANDLE_EXPORT __declspec( dllexport )
#if defined(soss_core_EXPORTS)
#define SOSS_EXPORT __declspec( dllexport )
#else
#define SOSS_EXPORT __declspec( dllimport )
#endif // soss_core_EXPORTS
#else
#define SOSS_EXPORT
#define SYSTEM_HANDLE_EXPORT
#endif // _WIN32

#endif