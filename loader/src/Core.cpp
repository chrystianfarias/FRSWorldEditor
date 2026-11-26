// FRS World Loader -- the parts every build shares
//
// The log and the settings beside the plugin, the game folder, and the thread
// flag that keeps the plugin's own file work out of its file hooks. The game
// specific code lives in Main.cpp; the NFS World build uses only this file,
// FileEdits.cpp and NfswMain.cpp.

#include "Loader.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace frs {

HMODULE g_module = nullptr;
__declspec(thread) bool t_inside = false;

// Beside the plugin, by its full path.
//
// A relative path follows the working directory, and the game changes that
// after it starts: the opening lines landed next to the plugin and every line
// after them in the game's root, which made a working run look silent.
std::wstring BesidePlugin(const wchar_t* extension) {
    wchar_t path[MAX_PATH] = {};
    if (!::GetModuleFileNameW(g_module, path, MAX_PATH)) return std::wstring(L".\\FRSWorldLoader") + extension;
    if (wchar_t* dot = ::wcsrchr(path, L'.')) *dot = 0;
    return std::wstring(path) + extension;
}

const wchar_t* LogFile() {
    static const std::wstring path = BesidePlugin(L".log");
    return path.c_str();
}

void Log(const char* format, ...) {
    const Inside inside;

    FILE* file = nullptr;
    if (::_wfopen_s(&file, LogFile(), L"a") != 0 || !file) return;

    va_list args;
    va_start(args, format);
    std::vfprintf(file, format, args);
    va_end(args);

    std::fputc('\n', file);
    std::fclose(file);
}

int Setting(const wchar_t* section, const wchar_t* key, int fallback) {
    static const std::wstring ini = BesidePlugin(L".ini");
    return (int)::GetPrivateProfileIntW(section, key, fallback, ini.c_str());
}

std::wstring GameFolder() {
    wchar_t path[MAX_PATH]{};
    ::GetModuleFileNameW(nullptr, path, MAX_PATH);

    std::wstring folder = path;
    const size_t slash = folder.find_last_of(L"\\/");
    return slash == std::wstring::npos ? L"." : folder.substr(0, slash);
}

} // namespace frs
