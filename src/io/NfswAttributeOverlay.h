#pragma once

#include "core/Types.h"
#include <string>

// Stages a whole World attribute pack under models/files for the ASI loader.
bool InstallNfswAttributeOverlay(const std::wstring& source, const std::wstring& modelsFolder,
                                 std::wstring& error);
bool WriteNfswAttributeOverlay(ByteSpan pack, const std::wstring& modelsFolder, std::wstring& error);
