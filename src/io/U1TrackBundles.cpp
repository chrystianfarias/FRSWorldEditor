#include "io/U1TrackBundles.h"

#include "core/Log.h"
#include "io/MeshOverlay.h"
#include "io/U1Collision.h"
#include "nfs/Jdlz.h"
#include "nfs/World.h"

#include <windows.h>

#include <algorithm>
#include <cmath>
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

// ---- what new roads see ------------------------------------------------------
//
// A drivable zone's load list (0x34152 in TRACKB's 0x80034150: 0x94 bytes,
// section i16 +8, count i32 +0x10, i16 sections[64] +0x14) was made for what
// is seen from the race streets in it; a road baked across the zone looks
// where no race street did, at sections the list never loads. The zone is the
// one whose outline (0x34151: 0x8C bytes, section i16 +8, count u16 +0xA,
// float2 points[16] +0xC) holds a piece of the road (0x446FD0 picks it by
// position; one outline per section, the first). Its list gets the sections
// nearest the road's pieces, while it stays within 64 entries counting the
// detail sections (n + 10) the game adds itself (0x446BB0), and within the
// bytes of the biggest list the region ships, so the stream pool holds it.

constexpr u32 kChunkOutlines = 0x00034151, kChunkLists = 0x00034152;
constexpr size_t kOutline = 0x8C, kList = 0x94;
constexpr float kRoadSight = 260.0f;   // m: sections closer than this to a road piece

struct RoadSight {
    std::vector<std::pair<float, float>> pieces;                 // road pieces' centres (x, y)
    std::vector<std::pair<int, Aabb>> sections;                  // drivable-letter sections' bounds
};

RoadSight CollectRoadSight(const nfs::World& world) {
    RoadSight sight;
    for (const auto& loaded : world.Loaded()) {
        if (!loaded->info || world.IsLibrary(*loaded)) continue;
        for (const nfs::SceneryInstance& instance : loaded->scenery.instances) {
            const nfs::SolidObject* model = world.FindModel(loaded->scenery.ModelHash(instance));
            if (!model || model->name.rfind("FRS_ROAD", 0) != 0) continue;
            sight.pieces.push_back({ instance.transform.m[3][0], instance.transform.m[3][1] });
        }
        // The editor's own placements (baked roads among them) are kept apart.
        for (const nfs::LoadedSection::UserPlacement& placed : loaded->placements) {
            std::string name = placed.instance.modelName;
            if (name.empty())
                if (const nfs::SolidObject* model = world.FindModel(placed.instance.modelHash)) name = model->name;
            if (name.rfind("FRS_ROAD", 0) != 0) continue;
            sight.pieces.push_back({ placed.instance.transform.m[3][0], placed.instance.transform.m[3][1] });
        }
        const int number = (int)loaded->info->number, letter = number / 100;
        if (letter < 1 || letter > 20 || loaded->info->streamSize == 0) continue;   // A..T: the city's own
        const Aabb box = world.SectionBounds(*loaded);
        if (box.Valid() && box.max.x - box.min.x < 2000.0f && box.max.y - box.min.y < 2000.0f)
            sight.sections.push_back({ number, box });
    }
    std::sort(sight.pieces.begin(), sight.pieces.end());
    return sight;
}

bool InsideOutline(const u8* outline, float x, float y) {
    u16 count = 0;
    std::memcpy(&count, outline + 0xA, 2);
    count = std::min<u16>(count, 16);
    bool in = false;
    for (u16 i = 0, j = count ? count - 1 : 0; i < count; j = i++) {
        float a[2], b[2];
        std::memcpy(a, outline + 0xC + 8 * i, 8);
        std::memcpy(b, outline + 0xC + 8 * j, 8);
        if ((a[1] > y) != (b[1] > y) && x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]) in = !in;
    }
    return in;
}

// Adds to every zone a road crosses the sections it sees; returns how many
// sections were added in all.
size_t AddRoadSight(std::vector<u8>& bundle, const RoadSight& sight,
                    const std::map<int, u32>& sectionBytes) {
    if (sight.pieces.empty()) return 0;
    size_t containerSize = 0;
    u8* container = Payload(bundle, 0x80034150, containerSize);
    if (!container) return 0;
    u8* outlines = nullptr;
    u8* lists = nullptr;
    size_t outlinesSize = 0, listsSize = 0;
    for (size_t at = 0; at + 8 <= containerSize;) {
        u32 id, size;
        std::memcpy(&id, container + at, 4);
        std::memcpy(&size, container + at + 4, 4);
        if (at + 8 + size > containerSize) break;
        if (id == kChunkOutlines) { outlines = container + at + 8; outlinesSize = size; }
        if (id == kChunkLists) { lists = container + at + 8; listsSize = size; }
        at += 8 + size;
    }
    if (!outlines || !lists) return 0;

    const auto listBytes = [&](const std::vector<int>& list) {
        u64 bytes = 0;
        for (int s : list) {
            const auto found = sectionBytes.find(s);
            if (found != sectionBytes.end()) bytes += found->second;
        }
        return bytes;
    };
    const auto readList = [&](const u8* record) {
        int32_t count = 0;
        std::memcpy(&count, record + 0x10, 4);
        std::vector<int> list;
        for (int32_t i = 0; i < std::clamp(count, 0, 64); ++i) {
            int16_t s = 0;
            std::memcpy(&s, record + 0x14 + 2 * i, 2);
            list.push_back(s);
        }
        return list;
    };
    // The game's own entries plus the detail sections it adds (n + 10 of
    // each drivable 1..9 not there yet).
    const auto slots = [](const std::vector<int>& list) {
        size_t n = list.size();
        for (int s : list)
            if (s % 100 >= 1 && s % 100 <= 9 && std::find(list.begin(), list.end(), s + 10) == list.end()) ++n;
        return n;
    };

    u64 budget = 0;   // the biggest list the region ships, in bytes
    for (size_t at = 0; at + kList <= listsSize; at += kList) budget = std::max(budget, listBytes(readList(lists + at)));

    size_t added = 0;
    for (size_t at = 0; at + kList <= listsSize; at += kList) {
        u8* record = lists + at;
        int16_t zone = 0;
        std::memcpy(&zone, record + 8, 2);
        const u8* outline = nullptr;
        for (size_t o = 0; o + kOutline <= outlinesSize && !outline; o += kOutline) {
            int16_t s = 0;
            std::memcpy(&s, outlines + o + 8, 2);
            if (s == zone) outline = outlines + o;
        }
        if (!outline) continue;
        std::vector<std::pair<float, float>> inZone;
        for (const auto& p : sight.pieces)
            if (InsideOutline(outline, p.first, p.second)) inZone.push_back(p);
        if (inZone.empty()) continue;

        std::vector<std::pair<float, int>> nearby;
        for (const auto& [number, box] : sight.sections) {
            float best = 1e30f;
            for (const auto& p : inZone) {
                const float dx = std::max({ box.min.x - p.first, 0.0f, p.first - box.max.x });
                const float dy = std::max({ box.min.y - p.second, 0.0f, p.second - box.max.y });
                best = std::min(best, std::sqrt(dx * dx + dy * dy));
            }
            if (best < kRoadSight) nearby.push_back({ best, number });
        }
        std::sort(nearby.begin(), nearby.end());
        std::vector<int> list = readList(record);
        u64 bytes = listBytes(list);
        size_t here = 0;
        for (const auto& [distance, number] : nearby) {
            if (std::find(list.begin(), list.end(), number) != list.end()) continue;
            std::vector<int> grown = list;
            grown.push_back(number);
            const u64 grownBytes = listBytes(grown);
            if (slots(grown) > 64 || grownBytes > budget) continue;
            list = std::move(grown);
            bytes = grownBytes;
            ++here;
        }
        if (!here) continue;
        const int32_t count = (int32_t)list.size();
        std::memcpy(record + 0x10, &count, 4);
        for (size_t i = 0; i < list.size(); ++i) {
            const int16_t s = (int16_t)list[i];
            std::memcpy(record + 0x14 + 2 * i, &s, 2);
        }
        added += here;
        LOG_INFO("zone %c%d: %zu sections a new road looks at added to its load list (%zu entries, %llu KB)",
                 (char)('@' + zone / 100), zone % 100, here, list.size(), (unsigned long long)(bytes / 1024));
    }
    return added;
}

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
    // New roads: the zones they cross load what they look at.
    const RoadSight sight = CollectRoadSight(world);
    if (!sight.pieces.empty()) {
        u32 hash = 2166136261u;
        for (const auto& p : sight.pieces) {
            const int xy[2] = { (int)std::lround(p.first * 10.0f), (int)std::lround(p.second * 10.0f) };
            for (size_t i = 0; i < sizeof(xy); ++i) hash = (hash ^ ((const u8*)xy)[i]) * 16777619u;
        }
        char line[64];
        std::snprintf(line, sizeof(line), "roads %zu %08X sight 1\n", sight.pieces.size(), hash);
        signature += line;
    }
    std::vector<u8> before;
    if (ReadWhole(signaturePath, before) && std::string(before.begin(), before.end()) == signature) return 0;

    const std::vector<u32> tracks = world.Underground1AllTracks();
    if (moved.empty() && collision.Empty() && sight.pieces.empty()) {
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

        std::map<int, u32> sectionBytes;
        for (size_t at = 0; at + kEntry <= tableSize; at += kEntry) {
            u32 number = 0, size = 0;
            std::memcpy(&number, table + at + 8, 4);
            std::memcpy(&size, table + at + 0x18, 4);
            sectionBytes[(int)(number & 0xFFFF)] = size;
        }
        const size_t seen = AddRoadSight(bundle, sight, sectionBytes);

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
        LOG_INFO("track bundle %u: %zu sections moved or resized, stream %u bytes, %zu collision polygons%s, %zu "
                 "sections added to zones new roads cross", track, changed, streamSize, polygons,
                 collision.Empty() ? " (shipped)" : "", seen);
    }
    WriteWhole(signaturePath, std::vector<u8>(signature.begin(), signature.end()));
    LOG_INFO("track bundles %S: %zu written to %S", world.Region().c_str(), written, folder.c_str());
    return written;
}

} // namespace io
