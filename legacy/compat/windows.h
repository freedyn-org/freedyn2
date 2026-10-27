// Minimal Windows API shim for building the legacy FreeDyn solver on Linux.
#pragma once
#include <dlfcn.h>
#include <cwchar>
#include <string>

typedef void* HINSTANCE;
typedef void* HMODULE;
typedef void* HANDLE;
typedef unsigned short WORD;
typedef unsigned long DWORD;
typedef int BOOL;
typedef const wchar_t* LPCWSTR;
typedef const char* LPCSTR;
#ifndef MAX_PATH
#define MAX_PATH 260
#endif

inline HINSTANCE LoadLibrary(const wchar_t* path) {
  std::wstring w(path);
  return dlopen(std::string(w.begin(), w.end()).c_str(), RTLD_NOW);
}
inline HINSTANCE LoadLibrary(const char* path) { return dlopen(path, RTLD_NOW); }
inline void* GetProcAddress(HINSTANCE h, const char* name) { return h ? dlsym(h, name) : nullptr; }
inline BOOL FreeLibrary(HINSTANCE h) { return h ? dlclose(h) == 0 : 0; }

// Console colors: no-ops.
#define STD_OUTPUT_HANDLE 0
#define FOREGROUND_BLUE 1
#define FOREGROUND_GREEN 2
#define FOREGROUND_RED 4
#define FOREGROUND_INTENSITY 8
struct CONSOLE_SCREEN_BUFFER_INFO { WORD wAttributes; };
inline HANDLE GetStdHandle(int) { return nullptr; }
inline BOOL SetConsoleTextAttribute(HANDLE, WORD) { return 1; }
inline BOOL GetConsoleScreenBufferInfo(HANDLE, CONSOLE_SCREEN_BUFFER_INFO* i) { i->wAttributes = 7; return 1; }
