// FRS World Loader for Need for Speed: World
//
// Map overlays are whole files under
// <game>\models\files stand in for the game's own, so the editor can write the
// section BUNs it changes and the game reads them while the shipped files stay
// untouched. The Underground 2 memory patches are deliberately not linked in;
// nfsw.exe has its own code and its own addresses.

#include "Loader.h"

BOOL APIENTRY DllMain(HMODULE module, DWORD reason, LPVOID) {
    using namespace frs;

    if (reason == DLL_PROCESS_DETACH) {
        LogFileEditTotals();
        return TRUE;
    }
    if (reason != DLL_PROCESS_ATTACH) return TRUE;

    g_module = module;
    ::DisableThreadLibraryCalls(module);
    ::DeleteFileW(LogFile());
    Log("--- FRS World Loader for Need for Speed: World ---");

    // File replacements are independent of the executable's code, which is
    // why they are the only part this build carries.
    StartFileEdits();
    Log("file edits: models\\files replacements are active");
    return TRUE;
}
