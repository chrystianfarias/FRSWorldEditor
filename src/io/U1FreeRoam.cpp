#include "io/U1FreeRoam.h"

#include "core/Log.h"
#include "io/U1Collision.h"
#include "io/U1TrackBundles.h"
#include "nfs/Jdlz.h"

#include <windows.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <set>
#include <tuple>
#include <unordered_map>

namespace io {
namespace {

using Record = std::array<u8, 32>;

constexpr u32 kChunkQuadtree = 0x34130, kChunkCells = 0x34131, kChunkPolygons = 0x34132,
              kChunkIndices = 0x34133, kChunkBounds = 0x34134;
constexpr u8 kFlagQuad = 0x10, kFlagVertical = 0x02, kFlagWall = 0x04;
constexpr float kCell = 32.0f;            // a cell of the grid the quadtree ends in

bool ReadWholeFile(const std::wstring& path, std::vector<u8>& out) {
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

std::vector<u8> Unpacked(const std::wstring& path) {
    std::vector<u8> file;
    if (!ReadWholeFile(path, file)) return {};
    if (file.size() >= 4 && std::memcmp(file.data(), "JDLZ", 4) == 0)
        return nfs::JdlzDecompress(ByteSpan{ file.data(), file.size() });
    return file;
}

struct TopChunk { u32 id; size_t offset; u32 size; };   // offset of the header

std::vector<TopChunk> TopChunks(const std::vector<u8>& bundle) {
    std::vector<TopChunk> out;
    for (size_t at = 0; at + 8 <= bundle.size();) {
        u32 id, size;
        std::memcpy(&id, bundle.data() + at, 4);
        std::memcpy(&size, bundle.data() + at + 4, 4);
        if (at + 8 + size > bundle.size()) break;
        out.push_back({ id, at, size });
        at += 8 + (size_t)size;
    }
    return out;
}

// A chunk's payload without its 0x11 alignment filler.
std::pair<const u8*, size_t> Payload(const std::vector<u8>& bundle, const TopChunk& chunk) {
    size_t skip = 0;
    while (skip + 4 <= chunk.size && bundle[chunk.offset + 8 + skip] == 0x11 &&
           bundle[chunk.offset + 8 + skip + 1] == 0x11 && bundle[chunk.offset + 8 + skip + 2] == 0x11 &&
           bundle[chunk.offset + 8 + skip + 3] == 0x11)
        skip += 4;
    return { bundle.data() + chunk.offset + 8 + skip, chunk.size - skip };
}

struct Decoded {
    float x[4], y[4], z[4];
    u8 flags;
    bool Quad() const { return (flags & kFlagQuad) != 0; }
};

Decoded Decode(const Record& r) {
    Decoded d{};
    i16 base;
    std::memcpy(&base, &r[4], 2);
    for (int i = 0; i < 4; ++i) {
        i16 x, y, z;
        std::memcpy(&x, &r[8 + i * 2], 2);
        std::memcpy(&y, &r[16 + i * 2], 2);
        std::memcpy(&z, &r[24 + i * 2], 2);
        d.x[i] = x / 8.0f;
        d.y[i] = y / 8.0f;
        d.z[i] = base + z / 256.0f;
    }
    d.flags = r[3];
    return d;
}

// One street, as two tracks may encode it: the corners in plan (1/8) and
// height (1/16), sorted, and whether it is a wall.
using GeometryKey = std::tuple<std::array<i32, 12>, u8>;
GeometryKey KeyOf(const Decoded& d) {
    std::array<std::array<i32, 3>, 4> corners;
    for (int i = 0; i < 4; ++i)
        corners[i] = { (i32)std::lround(d.x[i] * 8), (i32)std::lround(d.y[i] * 8), (i32)std::lround(d.z[i] * 16) };
    std::sort(corners.begin(), corners.end());
    std::array<i32, 12> flat{};
    for (int i = 0; i < 4; ++i)
        for (int k = 0; k < 3; ++k) flat[i * 3 + k] = corners[i][k];
    return { flat, (u8)(d.flags & (kFlagVertical | kFlagWall)) };
}

// The ground polygons of one track, found by plan position.
struct GroundIndex {
    static constexpr float kGrid = 8.0f;
    std::unordered_map<u64, std::vector<const Decoded*>> cells;
    static u64 Key(int gx, int gy) { return ((u64)(u32)gx << 32) | (u32)gy; }

    void Add(const Decoded& d) {
        const float x0 = *std::min_element(d.x, d.x + 4), x1 = *std::max_element(d.x, d.x + 4);
        const float y0 = *std::min_element(d.y, d.y + 4), y1 = *std::max_element(d.y, d.y + 4);
        for (int gx = (int)std::floor(x0 / kGrid); gx <= (int)std::floor(x1 / kGrid); ++gx)
            for (int gy = (int)std::floor(y0 / kGrid); gy <= (int)std::floor(y1 / kGrid); ++gy)
                cells[Key(gx, gy)].push_back(&d);
    }

    // The heights of the ground polygons over (x, y).
    void HeightsAt(float x, float y, std::vector<float>& out) const {
        out.clear();
        const auto found = cells.find(Key((int)std::floor(x / kGrid), (int)std::floor(y / kGrid)));
        if (found == cells.end()) return;
        for (const Decoded* d : found->second) {
            for (int h = 0; h < (d->Quad() ? 2 : 1); ++h) {
                const int a = 0, b = h ? 2 : 1, c = h ? 3 : 2;
                const float den = (d->y[b] - d->y[c]) * (d->x[a] - d->x[c]) + (d->x[c] - d->x[b]) * (d->y[a] - d->y[c]);
                if (std::fabs(den) < 1e-9f) continue;
                const float u = ((d->y[b] - d->y[c]) * (x - d->x[c]) + (d->x[c] - d->x[b]) * (y - d->y[c])) / den;
                const float v = ((d->y[c] - d->y[a]) * (x - d->x[c]) + (d->x[a] - d->x[c]) * (y - d->y[c])) / den;
                const float w = 1.0f - u - v;
                if (u < -1e-4f || v < -1e-4f || w < -1e-4f) continue;
                out.push_back(u * d->z[a] + v * d->z[b] + w * d->z[c]);
            }
        }
    }
};

// A barrier that closes a street for a race: another track has level ground
// on both sides of it and not the barrier -- that track's cars drive through
// where it stands. A building's barrier has no ground behind it.
bool DrivenThroughElsewhere(const Decoded& barrier, const GeometryKey& key, const std::vector<GroundIndex>& ground,
                            const std::vector<std::set<GeometryKey>>& barriers) {
    const float ax = barrier.x[0], ay = barrier.y[0], bx = barrier.x[2], by = barrier.y[2];
    const float length = std::hypot(bx - ax, by - ay);
    if (length < 0.5f) return false;
    const float nx = -(by - ay) / length, ny = (bx - ax) / length;
    constexpr float kProbe = 3.0f, kLevel = 1.5f;
    std::vector<float> one, other;
    for (size_t t = 0; t < ground.size(); ++t) {
        if (barriers[t].count(key)) continue;
        int open = 0;
        for (float s : { 0.25f, 0.5f, 0.75f }) {
            const float mx = ax + (bx - ax) * s, my = ay + (by - ay) * s;
            ground[t].HeightsAt(mx + nx * kProbe, my + ny * kProbe, one);
            ground[t].HeightsAt(mx - nx * kProbe, my - ny * kProbe, other);
            bool level = false;
            for (float a : one)
                for (float b : other)
                    if (std::fabs(a - b) < kLevel) level = true;
            open += level;
        }
        if (open >= 2) return true;
    }
    return false;
}

// A chunk writer that knows where it is: the game's loaders find aligned
// payloads by the absolute offset (0x11 filler before them).
struct Writer {
    std::vector<u8> bytes;
    void U32(u32 v) { const u8* p = (const u8*)&v; bytes.insert(bytes.end(), p, p + 4); }
    void Leaf(u32 id, const std::vector<u8>& data, size_t align) {
        const size_t pad = align ? (align - (bytes.size() + 8) % align) % align : 0;
        U32(id);
        U32((u32)(pad + data.size()));
        bytes.insert(bytes.end(), pad, (u8)0x11);
        bytes.insert(bytes.end(), data.begin(), data.end());
    }
    // A zero chunk so the next chunk starts at an offset that is `rem` modulo `align`.
    void ZeroTo(size_t align, size_t rem = 0) {
        if ((bytes.size() + align - rem % align) % align == 0) return;
        const size_t n = ((rem + align * 2) - (bytes.size() + 8) % align) % align;
        U32(0);
        U32((u32)n);
        bytes.insert(bytes.end(), n, (u8)0);
    }
};

template <typename T> void Put(std::vector<u8>& out, T value) {
    const u8* p = (const u8*)&value;
    out.insert(out.end(), p, p + sizeof(T));
}

} // namespace

bool WriteU1FreeRoamBundle(const std::wstring& gameFolder, const std::wstring& region, u32 track,
                           const std::vector<u32>& regionTracks, U1FreeRoamReport& report) {
    report = {};
    (void)region;
    const std::wstring tracks = gameFolder + L"\\TRACKS\\";
    // The track's bundle as the game is handed it: with the editor's table
    // corrections when there are any (io/U1TrackBundles.h).
    std::vector<u8> base = Unpacked(U1TrackBundlePath(gameFolder, gameFolder + L"\\models", track));
    if (base.empty()) {
        report.error = L"the track's TRACKB could not be read";
        return false;
    }

    // The race's reset zones (0x3414A, 0xB4 bytes each, type at +0: 2 RESET,
    // 3 RESET_TO_POINT) put a car that enters them back on the course -- the
    // race's invisible walls. In a drive they are type 0 (no wrong way), each
    // record where it is, so nothing that counts them moves.
    // They sit in the course's container, 0x80034147.
    for (const TopChunk& course : TopChunks(base)) {
        if (course.id != 0x80034147u) continue;
        for (size_t child = course.offset + 8; child + 8 <= course.offset + 8 + course.size;) {
            u32 id = 0, length = 0;
            std::memcpy(&id, base.data() + child, 4);
            std::memcpy(&length, base.data() + child + 4, 4);
            const size_t next = child + 8 + length;
            if (next > course.offset + 8 + course.size) break;
            if (id != 0x0003414Au) { child = next; continue; }
            size_t skip = 0;
            while (skip + 4 <= length && std::memcmp(base.data() + child + 8 + skip, "\x11\x11\x11\x11", 4) == 0) skip += 4;
            u8* records = base.data() + child + 8 + skip;
            const size_t size = length - skip;
            for (size_t at = 0; at + 0xB4 <= size; at += 0xB4) {
                u32 type = 0;
                std::memcpy(&type, records + at, 4);
                if (type != 2 && type != 3) continue;
                const u32 none = 0;
                std::memcpy(records + at, &none, 4);
                ++report.resetZonesOff;
            }
            child = next;
        }
    }

    // Every track's polygons, each geometry once.
    std::vector<Record> records;
    std::set<GeometryKey> seen;
    std::vector<u32> sources = regionTracks;
    if (std::find(sources.begin(), sources.end(), track) == sources.end()) sources.insert(sources.begin(), track);
    // Each track's own ground and barriers, to tell a race's barriers (above).
    std::vector<std::vector<Decoded>> trackGround;
    std::vector<std::set<GeometryKey>> trackBarriers;
    trackGround.reserve(sources.size());
    for (u32 number : sources) {
        // As the game is handed it: with the editor's collision edit.
        const std::vector<u8> bundle =
            number == track ? base : Unpacked(U1TrackBundlePath(gameFolder, gameFolder + L"\\models", number));
        if (bundle.empty()) continue;
        ++report.tracks;
        trackGround.emplace_back();
        trackBarriers.emplace_back();
        for (const TopChunk& chunk : TopChunks(bundle)) {
            if (chunk.id != kChunkPolygons) continue;
            const auto [data, size] = Payload(bundle, chunk);
            for (size_t at = 0; at + 32 <= size; at += 32) {
                Record r;
                std::memcpy(r.data(), data + at, 32);
                r[0] = 0;   // the query's "taken" mark
                const Decoded d = Decode(r);
                const GeometryKey key = KeyOf(d);
                if (d.flags & kFlagWall) trackBarriers.back().insert(key);
                else if (!(d.flags & kFlagVertical)) trackGround.back().push_back(d);
                if (!seen.insert(key).second) { ++report.duplicates; continue; }
                records.push_back(r);
            }
        }
    }

    std::vector<GroundIndex> trackGroundIndex(trackGround.size());
    for (size_t t = 0; t < trackGround.size(); ++t)
        for (const Decoded& d : trackGround[t]) trackGroundIndex[t].Add(d);

    // U1's race barriers are the walls with real height (flag 0x02 without
    // 0x04: corners with z, e.g. 17.3 to 41 m): every one is out. The city's
    // walls are the barriers (0x04: two ends, unbounded height): they stay,
    // but for the ones a race closes a street with (DrivenThroughElsewhere).
    std::vector<Decoded> decoded;
    decoded.reserve(records.size());
    for (const Record& r : records) decoded.push_back(Decode(r));
    std::vector<Record> kept;
    std::vector<const Decoded*> keptDecoded;
    for (size_t i = 0; i < records.size(); ++i) {
        if (decoded[i].flags & kFlagWall) {
            if (DrivenThroughElsewhere(decoded[i], KeyOf(decoded[i]), trackGroundIndex, trackBarriers)) {
                ++report.raceBarriers;
                continue;
            }
            ++report.cityWalls;
        } else if (decoded[i].flags & kFlagVertical) {
            ++report.raceBarriers;
            continue;
        }
        kept.push_back(records[i]);
        keptDecoded.push_back(&decoded[i]);
    }
    if (kept.size() > 65535) {
        report.error = L"more than 65535 collision polygons together";
        return false;
    }
    report.polygons = kept.size();

    // Cells of 32 units: each holds the polygons whose box touches it.
    std::map<std::pair<int, int>, std::vector<u16>> cells;
    float xmin = 1e30f, ymin = 1e30f, xmax = -1e30f, ymax = -1e30f;
    for (size_t i = 0; i < kept.size(); ++i) {
        const Decoded& d = *keptDecoded[i];
        const float x0 = *std::min_element(d.x, d.x + 4), x1 = *std::max_element(d.x, d.x + 4);
        const float y0 = *std::min_element(d.y, d.y + 4), y1 = *std::max_element(d.y, d.y + 4);
        xmin = std::min(xmin, x0), xmax = std::max(xmax, x1), ymin = std::min(ymin, y0), ymax = std::max(ymax, y1);
        for (int gx = (int)std::floor(x0 / kCell); gx <= (int)std::floor(x1 / kCell); ++gx)
            for (int gy = (int)std::floor(y0 / kCell); gy <= (int)std::floor(y1 / kCell); ++gy)
                cells[{ gx, gy }].push_back((u16)i);
    }
    if (cells.size() > 65534) {
        report.error = L"more than 65534 collision cells";
        return false;
    }
    std::vector<u8> cellData, indexData, polygonData, treeData, boundsData;
    Put(cellData, 1e6f); Put(cellData, 1e6f); Put(cellData, 0u); Put(cellData, 0u);   // cell 0: none
    std::vector<std::array<u16, 4>> nodes(1, { 0, 0, 0, 0 });
    u16 cellNumber = 0;
    u32 refs = 0;
    for (const auto& [where, list] : cells) {
        ++cellNumber;
        Put(cellData, where.first * kCell + kCell / 2);
        Put(cellData, where.second * kCell + kCell / 2);
        Put(cellData, refs);
        Put(cellData, (u32)list.size());
        for (u16 index : list) Put(indexData, index);
        refs += (u32)list.size();
        // the quadtree: ten levels from [-16384, 16384); quadrant = (x >= mid) | (y >= mid) << 1
        size_t node = 0;
        int mx = -16384, my = -16384, half = 16384;
        for (int level = 10; level >= 1; --level) {
            const int q = (where.first * 32 >= mx + half ? 1 : 0) | (where.second * 32 >= my + half ? 2 : 0);
            if (q & 1) mx += half;
            if (q & 2) my += half;
            if (level == 1) { nodes[node][q] = cellNumber; break; }
            if (!nodes[node][q]) {
                if (nodes.size() >= 65535) { report.error = L"more than 65535 quadtree nodes"; return false; }
                nodes.push_back({ 0, 0, 0, 0 });
                nodes[node][q] = (u16)(nodes.size() - 1);
            }
            node = nodes[node][q];
            half /= 2;
        }
    }
    for (const auto& n : nodes)
        for (u16 child : n) Put(treeData, child);
    for (const Record& r : kept) polygonData.insert(polygonData.end(), r.begin(), r.end());
    for (float v : { xmin, ymin, -127.0f, 0.0f, xmax, ymax, 127.0f, 0.0f }) Put(boundsData, v);

    // The bundle: the track's own chunks, its collision replaced.
    Writer out;
    bool collisionWritten = false;
    for (const TopChunk& chunk : TopChunks(base)) {
        if (chunk.id == 0) continue;
        if (chunk.id >= kChunkQuadtree && chunk.id <= kChunkBounds) {
            if (collisionWritten) continue;
            collisionWritten = true;
            out.ZeroTo(16);
            out.Leaf(kChunkQuadtree, treeData, 16);
            out.Leaf(kChunkCells, cellData, 16);
            out.Leaf(kChunkPolygons, polygonData, 0);
            out.Leaf(kChunkIndices, indexData, 0);
            out.Leaf(kChunkBounds, boundsData, 16);
            continue;
        }
        out.ZeroTo(0x80, chunk.offset % 0x80);   // copied chunks keep their offset modulo 0x80
        out.bytes.insert(out.bytes.end(), base.begin() + (ptrdiff_t)chunk.offset,
                         base.begin() + (ptrdiff_t)(chunk.offset + 8 + chunk.size));
    }
    if (!collisionWritten) {
        report.error = L"the track's TRACKB has no collision to replace";
        return false;
    }

    // Really compressed: the game unpacks TRACKB in place, the packed bytes at
    // the end of the unpacked buffer, so a stored (larger) block overruns it.
    const std::vector<u8> packed = nfs::JdlzCompress(ByteSpan{ out.bytes.data(), out.bytes.size() });
    if (packed.size() >= out.bytes.size() ||
        nfs::JdlzDecompress(ByteSpan{ packed.data(), packed.size() }) != out.bytes) {
        report.error = L"the bundle did not compress to something the game can unpack in place";
        return false;
    }
    const std::wstring folder = gameFolder + L"\\models\\freeroam";
    ::CreateDirectoryW((gameFolder + L"\\models").c_str(), nullptr);
    ::CreateDirectoryW(folder.c_str(), nullptr);
    report.path = folder + L"\\TRACKB" + std::to_wstring(track) + L".LZC";
    FILE* file = nullptr;
    if (::_wfopen_s(&file, report.path.c_str(), L"wb") != 0 || !file) {
        report.error = L"could not write " + report.path;
        return false;
    }
    const bool ok = std::fwrite(packed.data(), 1, packed.size(), file) == packed.size();
    std::fclose(file);
    if (!ok) report.error = L"could not write " + report.path;
    LOG_INFO("free roam bundle %S: %zu tracks, %zu polygons (%zu duplicates, %zu race barriers out, %zu walls kept), "
             "%zu cells, %zu nodes, %zu reset zones off", report.path.c_str(), report.tracks, report.polygons,
             report.duplicates, report.raceBarriers, report.cityWalls, cells.size(), nodes.size(), report.resetZonesOff);
    return ok;
}

} // namespace io
