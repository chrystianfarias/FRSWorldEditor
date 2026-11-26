#pragma once

#include "core/Types.h"

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace nfs {

inline constexpr wchar_t kNfswAttributesPath[] = L"GLOBAL\\ATTRIBUTES.BIN";
inline constexpr wchar_t kNfswAttributesOverlayPath[] = L"files\\GLOBAL\\ATTRIBUTES.BIN";

struct NfswSmackableAttributes {
    std::string name;
    std::optional<float> mass;
    u32 rigidBody = 0;
    u32 baseMaterial = 0;
    std::string materialName;
};

using NfswSmackableCatalog = std::unordered_map<u32, NfswSmackableAttributes>;

// Reads the smackable collections in the db vault of GLOBAL\ATTRIBUTES.BIN.
bool NfswReadSmackableAttributes(ByteSpan pack, NfswSmackableCatalog& catalog);

} // namespace nfs
