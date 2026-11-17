#include "io/U1TrackBundles.h"

#include "core/Log.h"
#include "nfs/Jdlz.h"
#include "nfs/World.h"

#include <windows.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <map>
#include <vector>

namespace io {
namespace {

constexpr u32 kChunkTable = 0x00034107, kChunkStreamSizes = 0x00034108;
constexpr size_t kEntry = 56;

bool ReadWhole(const std::wstring& path, std::vector<u8>& out) {
    FILE* file = nullptr;
    if (::_wfopen_s(&file, path.c_str(), L"rb") != 0 || !file) return false;
    std::fseek(file, 0, SEEK_END);
    const long size = std::ftell(file);
    std::fseek(file, 0, SEEK_SET);
    out.resize(size > 0 ? (size_t)size : 0);
    const bool ok = size > 0 && std::fread(out.data(), 1, out.size(), file) == out.size();
    std::fclose(file);
    return ok;
}

bool WriteWhole(const std::wstring& path, const std::vector<u8>& bytes) {
    FILE* file = nullptr;
    if (::_wfopen_s(&file, path.c_str(), L"wb") != 0 || !file) return false;
    const bool ok = std::fwrite(bytes.data(), 1, bytes.size(), file) == bytes.size();
    std::fclose(file);
    return ok;
}

// The payload of a top-level chunk, 0x11 filler skipped.
u8* Payload(std::vector<u8>& bundle, u32 id, size_t& size) {
    for (size_t at = 0; at + 8 <= bundle.size();) {
        u32 cid, csize;
        std::memcpy(&cid, bundle.data() + at, 4);
        std::memcpy(&csize, bundle.data() + at + 4, 4);
        if (at + 8 + csize > bundle.size()) break;
        if (cid == id) {
            size_t skip = 0;
            while (skip + 4 <= csize && std::memcmp(bundle.data() + at + 8 + skip, "\x11\x11\x11\x11", 4) == 0) skip += 4;
            size = csize - skip;
            return bundle.data() + at + 8 + skip;
        }
        at += 8 + csize;
    }
    size = 0;
    return nullptr;
}

struct Entry { u32 offset, size, geometry, sum; };

} // namespace

std::wstring U1TrackBundlePath(const std::wstring& gameFolder, const std::wstring& modelsFolder, u32 track) {
    const std::wstring name = L"TRACKB" + std::to_wstring(track) + L".LZC";
    const std::wstring mine = modelsFolder + L"\\files\\TRACKS\\" + name;
    if (::GetFileAttributesW(mine.c_str()) != INVALID_FILE_ATTRIBUTES) return mine;
    return gameFolder + L"\\TRACKS\\" + name;
}

size_t WriteU1TrackBundles(const nfs::World& world, const std::wstring& gameFolder,
                           const std::wstring& modelsFolder, std::wstring& error) {
    if (!world.IsUnderground1() || modelsFolder.empty()) return 0;

    // The sections out of their shipped place, by tag, with their byte sums.
    std::map<std::string, Entry> moved;
    u64 streamEnd = world.StreamBytes().size;
    for (const nfs::Section& section : world.Table().Sections()) {
        streamEnd = std::max<u64>(streamEnd, (u64)section.streamOffset + section.streamSize);
        if (section.streamOffset == section.originalStreamOffset && section.streamSize == section.originalStreamSize &&
            section.geometrySize == section.originalOtherSize)
            continue;
        const ByteSpan bytes = world.SectionBytes(section);
        u32 sum = 0;
        for (size_t i = 0; i < bytes.size; ++i) sum += bytes.data[i];
        moved[section.name] = { (u32)section.streamOffset, section.streamSize, section.geometrySize, sum };
    }

    const std::wstring folder = modelsFolder + L"\\files\\TRACKS";
    const std::wstring signaturePath = folder + L"\\FRS-" + world.Region() + L"-tables.txt";
    std::string signature;
    {
        char line[96];
        std::snprintf(line, sizeof(line), "stream %llu\n", (unsigned long long)streamEnd);
        signature += line;
        for (const auto& [name, e] : moved) {
            std::snprintf(line, sizeof(line), "%s %u %u %u %08X\n", name.c_str(), e.offset, e.size, e.geometry, e.sum);
            signature += line;
        }
    }
    std::vector<u8> before;
    if (ReadWhole(signaturePath, before) && std::string(before.begin(), before.end()) == signature) return 0;

    const std::vector<u32> tracks = world.Underground1AllTracks();
    if (moved.empty()) {
        // Every section where it shipped: the game's own bundles again.
        for (u32 track : tracks)
            ::DeleteFileW((folder + L"\\TRACKB" + std::to_wstring(track) + L".LZC").c_str());
        ::DeleteFileW(signaturePath.c_str());
        LOG_INFO("track bundles %S: every section where it shipped, the game's own bundles are read", world.Region().c_str());
        return 0;
    }

    ::CreateDirectoryW(modelsFolder.c_str(), nullptr);
    ::CreateDirectoryW((modelsFolder + L"\\files").c_str(), nullptr);
    ::CreateDirectoryW(folder.c_str(), nullptr);
    size_t written = 0;
    for (u32 track : tracks) {
        std::vector<u8> file, bundle;
        if (!ReadWhole(gameFolder + L"\\TRACKS\\TRACKB" + std::to_wstring(track) + L".LZC", file)) continue;
        bundle = file.size() >= 4 && std::memcmp(file.data(), "JDLZ", 4) == 0
                     ? nfs::JdlzDecompress(ByteSpan{ file.data(), file.size() }) : file;
        size_t tableSize = 0, sizesSize = 0;
        u8* table = Payload(bundle, kChunkTable, tableSize);
        u8* sizes = Payload(bundle, kChunkStreamSizes, sizesSize);
        if (!table || !sizes || sizesSize < 8) continue;
        size_t changed = 0;
        for (size_t at = 0; at + kEntry <= tableSize; at += kEntry) {
            u8* entry = table + at;
            char tag[9] = {};
            std::memcpy(tag, entry, 8);
            u32 type = 0;
            std::memcpy(&type, entry + 0x10, 4);
            if (type != 1) continue;   // the region stream's sections only
            const auto found = moved.find(tag);
            if (found == moved.end()) continue;
            std::memcpy(entry + 0x14, &found->second.offset, 4);
            std::memcpy(entry + 0x18, &found->second.size, 4);
            std::memcpy(entry + 0x1C, &found->second.geometry, 4);
            std::memcpy(entry + 0x20, &found->second.sum, 4);
            ++changed;
        }
        const u32 streamSize = (u32)streamEnd;
        std::memcpy(sizes + 4, &streamSize, 4);

        const std::vector<u8> packed = nfs::JdlzCompress(ByteSpan{ bundle.data(), bundle.size() });
        if (packed.size() >= bundle.size() || nfs::JdlzDecompress(ByteSpan{ packed.data(), packed.size() }) != bundle) {
            error = L"TRACKB" + std::to_wstring(track) + L".LZC did not compress to something the game can unpack";
            return written;
        }
        if (!WriteWhole(folder + L"\\TRACKB" + std::to_wstring(track) + L".LZC", packed)) {
            error = L"could not write TRACKB" + std::to_wstring(track) + L".LZC";
            return written;
        }
        ++written;
        LOG_INFO("track bundle %u: %zu sections moved or resized, stream %u bytes", track, changed, streamSize);
    }
    WriteWhole(signaturePath, std::vector<u8>(signature.begin(), signature.end()));
    LOG_INFO("track bundles %S: %zu written to %S", world.Region().c_str(), written, folder.c_str());
    return written;
}

} // namespace io
