// Force-included when building the legacy FreeDyn kernel on Linux with GCC.
// Maps the few MSVC/Windows specifics the solver path uses to POSIX.
#pragma once
#ifdef __cplusplus
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <cmath>
#include <algorithm>
#include <cfloat>
using std::signbit;
#endif
#include <sys/stat.h>
#include <unistd.h>

#define __declspec(x)
#define __stdcall
#define __cdecl

#ifdef __cplusplus
#include <cstdarg>
#include <ctime>
template <size_t N, class... A>
inline int sprintf_s(char (&buf)[N], const char* fmt, A... a) { return std::snprintf(buf, N, fmt, a...); }
template <class... A>
inline int sprintf_s(char* buf, size_t n, const char* fmt, A... a) { return std::snprintf(buf, n, fmt, a...); }
inline int ctime_s(char* buf, size_t, const time_t* t) { return ctime_r(t, buf) ? 0 : 1; }
inline bool isnand(double x) { return std::isnan(x); }
inline bool isinfd(double x) { return std::isinf(x); }
inline int SetConsoleTitle(const wchar_t*) { return 1; }
inline int SetConsoleTitle(const char*) { return 1; }
class MBS_ModelObject;  // MSVC accepts its use in MBS_InternalModel.h without a declaration
inline int fopen_s(FILE** f, const char* name, const char* mode) {
  *f = std::fopen(name, mode);
  return *f ? 0 : errno;
}
#endif
#define _access access
#define Sleep(ms) usleep((ms) * 1000)
#define _stat stat
#define fprintf_s fprintf
#define fscanf_s fscanf
#define _fseeki64 fseeko
#define _ftelli64 ftello
