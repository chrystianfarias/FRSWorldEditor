#include "io/U1Collision.h"

#include "core/Log.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>

namespace io {
namespace {

constexpr u32 kChunkQuadtree = 0x34130, kChunkCells = 0x34131, kChunkPolygons = 0x34132,
              kChunkIndices = 0x34133, kChunkBounds = 0x34134;
constexpr float kCell = 32.0f;
constexpr char kMagic[8] = { 'F', 'R', 'S', 'U', '1', 'C', 'O', 'L' };

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

void PlanBox(const U1Record& r, float& x0, float& y0, float& x1, float& y1) {
    x0 = y0 = 1e30f;
    x1 = y1 = -1e30f;
    for (int c = 0; c < 4; ++c) {
        i16 x, y;
        std::memcpy(&x, &r[8 + c * 2], 2);
        std::memcpy(&y, &r[16 + c * 2], 2);
        x0 = std::min(x0, x / 8.0f), x1 = std::max(x1, x / 8.0f);
        y0 = std::min(y0, y / 8.0f), y1 = std::max(y1, y / 8.0f);
    }
}

} // namespace

std::wstring U1CollisionEditPath(const std::wstring& modelsFolder, const std::wstring& region) {
    return modelsFolder + L"\\" + region + L"-collision.frsu1col";
}

bool ReadU1CollisionEdit(const std::wstring& path, U1CollisionEdit& out) {
    out = {};
    FILE* file = nullptr;
    if (::_wfopen_s(&file, path.c_str(), L"rb") != 0 || !file) return false;
    char magic[8];
    u32 removed = 0, added = 0;
    bool ok = std::fread(magic, 1, 8, file) == 8 && std::memcmp(magic, kMagic, 8) == 0 &&
              std::fread(&removed, 4, 1, file) == 1 && std::fread(&added, 4, 1, file) == 1 &&
              removed < 1000000 && added < 1000000;
    if (ok) {
        out.removed.resize(removed);
        out.added.resize(added);
        ok = (removed == 0 || std::fread(out.removed.data(), 32, removed, file) == removed) &&
             (added == 0 || std::fread(out.added.data(), 32, added, file) == added);
    }
    std::fclose(file);
    if (!ok) out = {};
    return ok;
}

bool WriteU1CollisionEdit(const std::wstring& path, const U1CollisionEdit& edit) {
    if (edit.Empty()) {
        ::_wremove(path.c_str());
        return true;
    }
    FILE* file = nullptr;
    if (::_wfopen_s(&file, path.c_str(), L"wb") != 0 || !file) return false;
    const u32 removed = (u32)edit.removed.size(), added = (u32)edit.added.size();
    bool ok = std::fwrite(kMagic, 1, 8, file) == 8 && std::fwrite(&removed, 4, 1, file) == 1 &&
              std::fwrite(&added, 4, 1, file) == 1 &&
              (removed == 0 || std::fwrite(edit.removed.data(), 32, removed, file) == removed) &&
              (added == 0 || std::fwrite(edit.added.data(), 32, added, file) == added);
    std::fclose(file);
    return ok;
}

U1PolygonKey KeyOf(const nfs::CollisionPolygon& polygon) {
    U1PolygonKey key{};
    key[0] = (polygon.barrier ? 1 : 0) | polygon.attributes[3] << 8 | polygon.attributes[2] << 16;
    for (int c = 0; c < 4; ++c) {
        // A barrier is its two ends; its height is the viewer's.
        const Vec3& p = polygon.barrier ? polygon.corners[c < 2 ? 0 : 2] : polygon.corners[c];
        key[2 + c * 3] = (i32)std::lround(p.x * 8.0f);
        key[3 + c * 3] = (i32)std::lround(p.y * 8.0f);
        key[4 + c * 3] = polygon.barrier ? 0 : (i32)std::lround(p.z * 256.0f);
    }
    return key;
}

U1PolygonKey KeyOfRecord(const U1Record& record) {
    std::vector<nfs::CollisionPolygon> one;
    nfs::ParseCollisionPolygons(ByteSpan{ record.data(), record.size() }, one);
    return one.empty() ? U1PolygonKey{} : KeyOf(one.front());
}

void DiffU1Collision(const std::vector<nfs::CollisionPolygon>& shipped,
                     const std::vector<nfs::CollisionPolygon>& current, U1CollisionEdit& out, std::wstring& error) {
    out = {};
    std::map<U1PolygonKey, int> count;
    for (const auto& p : current) ++count[KeyOf(p)];
    std::map<U1PolygonKey, int> was;
    for (const auto& p : shipped) {
        const U1PolygonKey key = KeyOf(p);
        auto found = count.find(key);
        if (found != count.end() && found->second > 0) { --found->second; continue; }
        U1Record r{};
        if (!nfs::EncodeCollisionPolygon(p, r.data(), error)) return;
        out.removed.push_back(r);
    }
    // What is left of `current` is new.
    for (const auto& p : current) {
        auto found = count.find(KeyOf(p));
        if (found == count.end() || found->second <= 0) continue;
        --found->second;
        U1Record r{};
        if (!nfs::EncodeCollisionPolygon(p, r.data(), error)) return;
        out.added.push_back(r);
    }
}

std::vector<U1Record> ApplyU1CollisionEdit(const std::vector<U1Record>& records, const U1CollisionEdit& edit) {
    std::map<U1PolygonKey, int> gone;
    for (const U1Record& r : edit.removed) ++gone[KeyOfRecord(r)];
    std::vector<U1Record> out;
    out.reserve(records.size() + edit.added.size());
    for (const U1Record& r : records) {
        auto found = gone.find(KeyOfRecord(r));
        if (found != gone.end() && found->second > 0) { --found->second; continue; }
        out.push_back(r);
    }
    for (U1Record r : edit.added) {
        r[0] = 0;
        out.push_back(r);
    }
    return out;
}

std::vector<U1Record> U1CollisionRecords(const std::vector<u8>& bundle) {
    std::vector<U1Record> out;
    for (const TopChunk& chunk : TopChunks(bundle)) {
        if (chunk.id != kChunkPolygons) continue;
        size_t skip = 0;
        while (skip + 4 <= chunk.size && std::memcmp(bundle.data() + chunk.offset + 8 + skip, "\x11\x11\x11\x11", 4) == 0)
            skip += 4;
        const u8* data = bundle.data() + chunk.offset + 8 + skip;
        for (size_t at = 0; at + 32 <= chunk.size - skip; at += 32) {
            U1Record r;
            std::memcpy(r.data(), data + at, 32);
            r[0] = 0;   // the query's "taken" mark
            out.push_back(r);
        }
    }
    return out;
}

bool ReplaceU1Collision(const std::vector<u8>& bundle, const std::vector<U1Record>& records,
                        std::vector<u8>& out, std::wstring& error) {
    if (records.size() > 65535) {
        error = L"more than 65535 collision polygons in one track";
        return false;
    }
    // Cells of 32 units: each holds the polygons whose box touches it.
    std::map<std::pair<int, int>, std::vector<u16>> cells;
    float xmin = 1e30f, ymin = 1e30f, xmax = -1e30f, ymax = -1e30f;
    for (size_t i = 0; i < records.size(); ++i) {
        float x0, y0, x1, y1;
        PlanBox(records[i], x0, y0, x1, y1);
        xmin = std::min(xmin, x0), xmax = std::max(xmax, x1), ymin = std::min(ymin, y0), ymax = std::max(ymax, y1);
        for (int gx = (int)std::floor(x0 / kCell); gx <= (int)std::floor(x1 / kCell); ++gx)
            for (int gy = (int)std::floor(y0 / kCell); gy <= (int)std::floor(y1 / kCell); ++gy)
                cells[{ gx, gy }].push_back((u16)i);
    }
    if (cells.size() > 65534) {
        error = L"more than 65534 collision cells";
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
                if (nodes.size() >= 65535) { error = L"more than 65535 quadtree nodes"; return false; }
                nodes.push_back({ 0, 0, 0, 0 });
                nodes[node][q] = (u16)(nodes.size() - 1);
            }
            node = nodes[node][q];
            half /= 2;
        }
    }
    for (const auto& n : nodes)
        for (u16 child : n) Put(treeData, child);
    for (const U1Record& r : records) polygonData.insert(polygonData.end(), r.begin(), r.end());
    if (records.empty()) xmin = ymin = xmax = ymax = 0.0f;
    for (float v : { xmin, ymin, -127.0f, 0.0f, xmax, ymax, 127.0f, 0.0f }) Put(boundsData, v);

    // The bundle: its own chunks, the collision replaced.
    Writer w;
    bool written = false;
    for (const TopChunk& chunk : TopChunks(bundle)) {
        if (chunk.id == 0) continue;
        if (chunk.id >= kChunkQuadtree && chunk.id <= kChunkBounds) {
            if (written) continue;
            written = true;
            w.ZeroTo(16);
            w.Leaf(kChunkQuadtree, treeData, 16);
            w.Leaf(kChunkCells, cellData, 16);
            w.Leaf(kChunkPolygons, polygonData, 0);
            w.Leaf(kChunkIndices, indexData, 0);
            w.Leaf(kChunkBounds, boundsData, 16);
            continue;
        }
        w.ZeroTo(0x80, chunk.offset % 0x80);   // copied chunks keep their offset modulo 0x80
        w.bytes.insert(w.bytes.end(), bundle.begin() + (ptrdiff_t)chunk.offset,
                       bundle.begin() + (ptrdiff_t)(chunk.offset + 8 + chunk.size));
    }
    if (!written) {
        error = L"the track's bundle has no collision to replace";
        return false;
    }
    out = std::move(w.bytes);
    return true;
}

} // namespace io
