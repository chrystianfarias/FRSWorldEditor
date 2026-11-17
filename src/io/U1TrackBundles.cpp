#include "io/U1TrackBundles.h"

#include "core/Log.h"
#include "io/MeshOverlay.h"
#include "io/U1Collision.h"
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

bool WriteU1Stream(const nfs::World& world, const std::wstring& modelsFolder, std::wstring& error) {
    if (!world.IsUnderground1() || modelsFolder.empty()) return false;
    const std::wstring region = world.Region();
    const std::string region8(region.begin(), region.end());

    // The edits: the stream's spans, as the editor read them, and the meshes.
    struct Mesh { u64 offset; u32 stride; std::vector<float> positions; };
    std::vector<Mesh> meshes;
    WIN32_FIND_DATAW found{};
    HANDLE search = ::FindFirstFileW((modelsFolder + L"\\*.frsmesh").c_str(), &found);
    if (search != INVALID_HANDLE_VALUE) {
        do {
            std::vector<u8> file;
            if (!ReadWhole(modelsFolder + L"\\" + found.cFileName, file) || file.size() < sizeof(MeshOverlayHeader)) continue;
            MeshOverlayHeader header;
            std::memcpy(&header, file.data(), sizeof(header));
            if (std::memcmp(header.magic, "FRSMESH1", 8) != 0) continue;
            if (std::string(header.region, strnlen(header.region, 8)) != region8) continue;
            const size_t floats = (size_t)header.vertexCount * 3;
            if (file.size() < sizeof(header) + floats * 4 || header.stride < 12) continue;
            Mesh mesh{ header.fileOffset, header.stride, std::vector<float>(floats) };
            std::memcpy(mesh.positions.data(), file.data() + sizeof(header), floats * 4);
            meshes.push_back(std::move(mesh));
        } while (::FindNextFileW(search, &found));
        ::FindClose(search);
    }
    std::vector<const SpanOverlay*> spans;
    for (const SpanOverlay& span : world.Spans())
        if (!span.layoutFile) spans.push_back(&span);

    const std::wstring folder = modelsFolder + L"\\files\\TRACKS";
    const std::wstring path = folder + L"\\STREAM" + region + L".BUN";
    const std::wstring signaturePath = folder + L"\\FRS-" + region + L"-stream.txt";
    if (spans.empty() && meshes.empty()) {
        if (::GetFileAttributesW(path.c_str()) != INVALID_FILE_ATTRIBUTES) {
            ::DeleteFileW(path.c_str());
            LOG_INFO("stream %S: no edits, the game's own is read again", region.c_str());
        }
        ::DeleteFileW(signaturePath.c_str());
        return false;
    }

    // The signature: what the copy holds.
    u64 hash = 1469598103934665603ull;
    const auto mix = [&](const void* data, size_t size) {
        const u8* p = (const u8*)data;
        for (size_t i = 0; i < size; ++i) hash = (hash ^ p[i]) * 1099511628211ull;
    };
    const ByteSpan stream = world.StreamBytes();
    mix(&stream.size, sizeof(stream.size));
    for (const SpanOverlay* span : spans) {
        mix(&span->fileOffset, sizeof(span->fileOffset));
        mix(span->bytes.data(), span->bytes.size());
    }
    for (const Mesh& mesh : meshes) {
        mix(&mesh.offset, sizeof(mesh.offset));
        mix(mesh.positions.data(), mesh.positions.size() * 4);
    }
    char signature[64];
    std::snprintf(signature, sizeof(signature), "%016llX %zu spans %zu meshes\n", (unsigned long long)hash,
                  spans.size(), meshes.size());
    std::vector<u8> before;
    if (ReadWhole(signaturePath, before) && std::string(before.begin(), before.end()) == signature &&
        ::GetFileAttributesW(path.c_str()) != INVALID_FILE_ATTRIBUTES)
        return false;

    ::CreateDirectoryW(modelsFolder.c_str(), nullptr);
    ::CreateDirectoryW((modelsFolder + L"\\files").c_str(), nullptr);
    ::CreateDirectoryW(folder.c_str(), nullptr);
    HANDLE out = ::CreateFileW(path.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (out == INVALID_HANDLE_VALUE) {
        error = L"could not write " + path + L" (is the game running?)";
        return false;
    }
    const auto writeAt = [&](u64 at, const void* data, size_t size) {
        LARGE_INTEGER where;
        where.QuadPart = (LONGLONG)at;
        DWORD written = 0;
        return ::SetFilePointerEx(out, where, nullptr, FILE_BEGIN) &&
               ::WriteFile(out, data, (DWORD)size, &written, nullptr) && written == size;
    };
    bool ok = true;
    for (u64 at = 0; ok && at < stream.size; at += 16u << 20) {
        const size_t piece = (size_t)std::min<u64>(16u << 20, stream.size - at);
        ok = writeAt(at, stream.data + at, piece);
    }
    for (const SpanOverlay* span : spans)
        ok = ok && writeAt(span->fileOffset, span->bytes.data(), span->bytes.size());
    for (const Mesh& mesh : meshes)
        for (size_t v = 0; ok && v < mesh.positions.size() / 3; ++v)
            ok = writeAt(mesh.offset + v * (u64)mesh.stride, &mesh.positions[v * 3], 12);
    ::CloseHandle(out);
    if (!ok) {
        ::DeleteFileW(path.c_str());
        error = L"could not write " + path;
        return false;
    }
    WriteWhole(signaturePath, std::vector<u8>(signature, signature + std::strlen(signature)));
    LOG_INFO("stream %S written whole to %S: %zu section spans, %zu meshes", region.c_str(), path.c_str(), spans.size(),
             meshes.size());
    return true;
}

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
    // The collision edit (io/U1Collision.h), in every track's bundle too.
    U1CollisionEdit collision;
    std::vector<u8> collisionFile;
    const std::wstring collisionPath = U1CollisionEditPath(modelsFolder, world.Region());
    if (ReadU1CollisionEdit(collisionPath, collision) && !collision.Empty() && ReadWhole(collisionPath, collisionFile)) {
        u32 hash = 2166136261u;
        for (u8 b : collisionFile) hash = (hash ^ b) * 16777619u;
        char line[64];
        std::snprintf(line, sizeof(line), "collision %zu out %zu in %08X\n", collision.removed.size(),
                      collision.added.size(), hash);
        signature += line;
    }
    std::vector<u8> before;
    if (ReadWhole(signaturePath, before) && std::string(before.begin(), before.end()) == signature) return 0;

    const std::vector<u32> tracks = world.Underground1AllTracks();
    if (moved.empty() && collision.Empty()) {
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

        size_t polygons = 0;
        if (!collision.Empty()) {
            const std::vector<U1Record> records = ApplyU1CollisionEdit(U1CollisionRecords(bundle), collision);
            std::vector<u8> replaced;
            std::wstring why;
            if (!ReplaceU1Collision(bundle, records, replaced, why)) {
                error = L"TRACKB" + std::to_wstring(track) + L".LZC: " + why;
                return written;
            }
            bundle = std::move(replaced);
            polygons = records.size();
        }

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
        LOG_INFO("track bundle %u: %zu sections moved or resized, stream %u bytes, %zu collision polygons%s", track,
                 changed, streamSize, polygons, collision.Empty() ? " (shipped)" : "");
    }
    WriteWhole(signaturePath, std::vector<u8>(signature.begin(), signature.end()));
    LOG_INFO("track bundles %S: %zu written to %S", world.Region().c_str(), written, folder.c_str());
    return written;
}

} // namespace io
