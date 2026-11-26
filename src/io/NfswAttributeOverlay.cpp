#include "io/NfswAttributeOverlay.h"

#include "core/FileMap.h"
#include "nfs/NfswSmackableAttributes.h"

#include <windows.h>
#include <filesystem>

bool InstallNfswAttributeOverlay(const std::wstring& source, const std::wstring& modelsFolder,
                                 std::wstring& error) {
    FileMap file;
    if (source.empty() || !file.Open(source)) {
        error = L"Could not read the selected attribute file.";
        return false;
    }
    const auto data = file.Span();
    std::vector<u8> bytes(data.begin(), data.end());
    file.Close(); // the selected file can already be the external pack
    return WriteNfswAttributeOverlay({ bytes.data(), bytes.size() }, modelsFolder, error);
}

bool WriteNfswAttributeOverlay(ByteSpan pack, const std::wstring& modelsFolder, std::wstring& error) {
    if (modelsFolder.empty()) {
        error = L"Open a World region before importing an attribute pack.";
        return false;
    }
    nfs::NfswSmackableCatalog catalog;
    if (pack.size > MAXDWORD || !nfs::NfswReadSmackableAttributes(pack, catalog)) {
        error = L"The selected file has no readable World smackable vault.";
        return false;
    }
    const std::filesystem::path models(modelsFolder);
    const auto target = models / nfs::kNfswAttributesOverlayPath;
    std::error_code directoryError;
    std::filesystem::create_directories(target.parent_path(), directoryError);
    if (directoryError) {
        error = L"Could not create the external attribute folder.";
        return false;
    }
    const auto shipped = models.parent_path() / nfs::kNfswAttributesPath;
    if (std::filesystem::equivalent(target.parent_path(), shipped.parent_path(), directoryError)) {
        error = L"The external attribute folder points to the game's GLOBAL folder.";
        return false;
    }

    // Keep the staging file outside models/files, which the ASI scans at startup.
    wchar_t staging[MAX_PATH]{};
    if (!::GetTempFileNameW(modelsFolder.c_str(), L"att", 0, staging)) {
        error = L"Could not create a temporary attribute file in the models folder.";
        return false;
    }
    const HANDLE file = ::CreateFileW(staging, GENERIC_WRITE, 0, nullptr, TRUNCATE_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    DWORD length = 0;
    const bool saved = file != INVALID_HANDLE_VALUE && ::WriteFile(file, pack.data, (DWORD)pack.size, &length, nullptr) &&
                       length == pack.size && ::FlushFileBuffers(file);
    if (file != INVALID_HANDLE_VALUE) ::CloseHandle(file);
    if (saved && ::MoveFileExW(staging, target.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) return true;
    error = saved ? L"Could not replace the external attribute pack. Close any application using that file and retry."
                  : L"Could not write the staged attribute pack.";
    ::DeleteFileW(staging);
    return false;
}
