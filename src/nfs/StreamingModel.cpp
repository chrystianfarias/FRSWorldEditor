#include "nfs/StreamingModel.h"

#include "core/Log.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace nfs {

namespace {

constexpr u32 kRoutesContainer = 0x80034147;
constexpr u32 kPathZones       = 0x0003414A;
constexpr size_t kZoneHeader   = 0x44;

u16 Read16(ByteSpan data, size_t at) { return data.read<u16>(at); }
u32 Read32(ByteSpan data, size_t at) { return data.read<u32>(at); }
float ReadF32(ByteSpan data, size_t at) { return data.read<float>(at); }

} // namespace

bool StreamingModel::PathZone::Contains(float x, float y) const {
    if (x < boundsMin.x || x > boundsMax.x || y < boundsMin.y || y > boundsMax.y) return false;
    bool inside = false;
    const size_t count = ring.size();
    for (size_t i = 0, j = count - 1; i < count; j = i++) {
        const Vec3& a = ring[i];
        const Vec3& b = ring[j];
        if ((a.y > y) != (b.y > y) &&
            x < (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x)
            inside = !inside;
    }
    return inside;
}

bool StreamingModel::PathZone::HeightAllows(float z) const {
    // The layout stores a signed threshold at +0x14; in the observed data a
    // positive value is a floor and a negative one a ceiling. sub_742C20's
    // exact comparison is still being confirmed, so a rejected zone falls
    // through to the polygon lookup in SectionAt.
    if (height > 0.0f) return z >= height;
    if (height < 0.0f) return z <= std::fabs(height);
    return true;
}

bool StreamingModel::Parse(ByteSpan layout) {
    pathZones_.clear();
    lastTime_ = -1.0;

    ChunkReader top(layout, 0);
    Chunk chunk;
    Chunk routes;
    while (top.Next(chunk)) {
        if (chunk.id == kRoutesContainer) { routes = chunk; break; }
    }
    if (routes.id == 0) return false;

    ChunkReader children = ChunkReader::Children(routes);
    Chunk zones;
    while (children.Next(chunk)) {
        if (chunk.id == kPathZones) { zones = chunk; break; }
    }
    if (zones.id == 0) return false;

    const ByteSpan data = zones.data;
    size_t at = 0;
    while (at + kZoneHeader <= data.size) {
        const u32 type = Read32(data, at);
        const u16 size = Read16(data, at + 0x42);
        if (size < kZoneHeader || at + size > data.size) break;
        if (type == 6) {
            const u16 points = Read16(data, at + 0x40);
            if (kZoneHeader + (size_t)points * 8 <= size && points >= 3) {
                PathZone zone;
                zone.height = ReadF32(data, at + 0x14);
                zone.boundsMin = { ReadF32(data, at + 0x20), ReadF32(data, at + 0x24), 0.0f };
                zone.boundsMax = { ReadF32(data, at + 0x28), ReadF32(data, at + 0x2C), 0.0f };
                if (zone.boundsMin.x > zone.boundsMax.x) std::swap(zone.boundsMin.x, zone.boundsMax.x);
                if (zone.boundsMin.y > zone.boundsMax.y) std::swap(zone.boundsMin.y, zone.boundsMax.y);
                for (int i = 0; i < 4; ++i) zone.candidates[i] = Read16(data, at + 0x30 + i * 2);
                zone.ring.reserve(points);
                for (u16 i = 0; i < points; ++i)
                    zone.ring.push_back({ ReadF32(data, at + kZoneHeader + i * 8),
                                          ReadF32(data, at + kZoneHeader + i * 8 + 4), 0.0f });
                pathZones_.push_back(std::move(zone));
            }
        }
        at += size;
    }

    LOG_INFO("NFS World streaming: %zu path zones (type 6)", pathZones_.size());
    return !pathZones_.empty();
}

int StreamingModel::SectionAt(const Vec3& eye, double time, const Preculler& visibility,
                              Vec3* predicted) const {
    Vec3 velocity{};
    if (lastTime_ >= 0.0 && time > lastTime_) {
        const double dt = time - lastTime_;
        if (dt < 1.0) velocity = (eye - lastEye_) * (float)(1.0 / dt);
    }
    lastEye_ = eye;
    lastTime_ = time;

    Vec3 point = eye;
    const float speed = Length(velocity);
    if (speed > 1.0f && speed <= 178.81091f) {
        const float ahead = std::min(1.5f, 100.0f / speed);
        point = eye + velocity * ahead;
    }
    if (predicted) *predicted = point;

    for (const PathZone& zone : pathZones_) {
        if (!zone.Contains(point.x, point.y) || !zone.HeightAllows(point.z)) continue;
        const int candidate = visibility.ZoneAt(point.x, point.y);
        if (candidate < 0) continue;
        const u32 id = visibility.Zones()[(size_t)candidate].id;
        for (u16 wanted : zone.candidates) {
            if (!wanted || wanted != id) continue;
            const std::vector<u32>* list = visibility.ListOf(wanted);
            if (list && !list->empty()) return (int)wanted;
        }
    }

    const int fallback = visibility.ZoneAt(eye.x, eye.y);
    return fallback >= 0 ? (int)visibility.Zones()[(size_t)fallback].id : -1;
}

} // namespace nfs
