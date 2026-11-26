#include "io/NfswSmackableWrite.h"

#include "nfs/NfswCollision.h"
#include "nfs/World.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace {

constexpr u32 kSceneryEntries = 0x34108;
constexpr u32 kCopySpawnId = 0x46520000;
constexpr size_t kSpawnSize = 64;
constexpr size_t kMaxSpawns = 0x1000;  // the game's per-chunk smashed bitset
constexpr std::array<u16, 3> kVacantEntry{ 0, 0, 0x18 };

template<class Container, class Value>
void Put(Container& bytes, size_t offset, Value value) {
    std::memcpy(bytes.data() + offset, &value, sizeof(value));
}

nfs::Chunk FindTop(ByteSpan bytes, u32 id) {
    nfs::ChunkReader reader(bytes, 0);
    nfs::Chunk chunk;
    while (reader.Next(chunk))
        if (chunk.id == id) return chunk;
    return {};
}

ByteSpan SpawnPayload(const nfs::Chunk& chunk) {
    const size_t padding = (16 - ((size_t)chunk.headerOffset + 8) % 16) % 16;
    const ByteSpan data = chunk.raw.sub(padding, chunk.raw.size);
    if (data.size < 16 || data.read<u32>(0) != 11 || data.read<u32>(4) != 11 ||
        data.read<u16>(10) > (data.size - 16) / kSpawnSize) return {};
    return data;
}

bool IsCopy(ByteSpan spawn, size_t first, size_t end) {
    const u32 entry = spawn.read<u32>(0x28);
    return entry >= first && entry < end && spawn.read<u32>(0x2C) == (kCopySpawnId | entry);
}

Vec3 ToPhysics(const Vec3& point) {
    return { -point.y, point.z, point.x };
}

Vec3 FromPhysics(const Vec3& point) {
    return { point.z, -point.x, point.y };
}

bool SpawnRotation(const Mat4& transform, std::array<float, 4>& rotation) {
    // The physics axes map to -Y, +Z, +X in the editor. Express each
    // transformed axis in physics space and convert the resulting rotation.
    const Vec3 x = ToPhysics(transform.TransformDirection({ 0, -1, 0 }));
    const Vec3 y = ToPhysics(transform.TransformDirection({ 0, 0, 1 }));
    const Vec3 z = ToPhysics(transform.TransformDirection({ 1, 0, 0 }));
    if (!(std::abs(Dot(x, x) - 1) < 0.005f && std::abs(Dot(y, y) - 1) < 0.005f &&
          std::abs(Dot(z, z) - 1) < 0.005f && std::abs(Dot(x, y)) < 0.005f &&
          std::abs(Dot(x, z)) < 0.005f && std::abs(Dot(y, z)) < 0.005f && Dot(Cross(x, y), z) > 0.99f))
        return false;  // the spawn format cannot represent a scaled box

    const float m[3][3] = { { x.x, y.x, z.x }, { x.y, y.y, z.y }, { x.z, y.z, z.z } };
    const float trace = m[0][0] + m[1][1] + m[2][2];
    if (trace > 0) {
        const float s = 2 * std::sqrt(1 + trace);
        rotation = { (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s,
                     (m[1][0] - m[0][1]) / s, 0.25f * s };
    } else {
        int i = m[1][1] > m[0][0] ? 1 : 0;
        if (m[2][2] > m[i][i]) i = 2;
        const int j = (i + 1) % 3, k = (i + 2) % 3;
        const float s = 2 * std::sqrt(1 + m[i][i] - m[j][j] - m[k][k]);
        rotation[i] = 0.25f * s;
        rotation[j] = (m[j][i] + m[i][j]) / s;
        rotation[k] = (m[k][i] + m[i][k]) / s;
        rotation[3] = (m[k][j] - m[j][k]) / s;
    }
    return true;
}

bool MoveSpawn(std::array<u8, 64>& spawn, const Mat4& from, const Mat4& to) {
    const ByteSpan raw{ spawn.data(), spawn.size() };
    const Vec3 position = FromPhysics({ raw.read<float>(16), raw.read<float>(20), raw.read<float>(24) });
    const Mat4 delta = InverseAffine(from) * to;
    const bool sameBasis = std::memcmp(from.m, to.m, 3 * sizeof(from.m[0])) == 0;
    if (!std::isfinite(Length(position))) return false;
    if (sameBasis) {
        Put(spawn, 16, ToPhysics(position + (to.TransformPoint({}) - from.TransformPoint({}))));
    } else {
        std::array<float, 4> rotation;
        if (!SpawnRotation(delta, rotation)) return false;
        const Vec3 a{ rotation[0], rotation[1], rotation[2] };
        const Vec3 b{ raw.read<float>(0), raw.read<float>(4), raw.read<float>(8) };
        const float w = raw.read<float>(12);
        const Vec3 v = b * rotation[3] + a * w + Cross(a, b);
        Put(spawn, 0, std::array<float, 4>{ v.x, v.y, v.z, rotation[3] * w - Dot(a, b) });
        Put(spawn, 16, ToPhysics(delta.TransformPoint(position)));
    }
    return true;
}

} // namespace

void NfswSmackableWrite::ReadGeometry(ByteSpan layout) {
    const auto list = FindTop(layout, nfs::kSmackableGeometryList);
    nfs::ChunkReader reader = nfs::ChunkReader::Children(list);
    nfs::Chunk child;
    while (reader.Next(child)) {
        if (child.id != nfs::kSmackableGeometry) continue;
        const size_t prefix = (16 - (child.headerOffset + 8) % 16) % 16;
        const auto geometry = child.raw.sub(prefix, child.raw.size);
        if (geometry.size >= 16) geometries_.emplace(geometry.read<u32>(0), geometry);
    }
}

u32 NfswSmackableWrite::GeometryFor(const nfs::NfswSmackableLink& link) {
    if (!link.halfSizes) return link.geometry;
    const auto source = geometries_.find(link.geometry);
    if (source == geometries_.end() || !nfs::NfswIsSingleSmackableBox(source->second) ||
        std::any_of(link.halfSizes->begin(), link.halfSizes->end(), [](i16 x) { return x <= 0; })) return 0;

    std::array<u8, 80> geometry;
    std::copy_n(source->second.data, geometry.size(), geometry.begin());
    Put(geometry, 32, *link.halfSizes);
    for (const auto& [key, existing] : geometries_)
        if (nfs::NfswIsSingleSmackableBox(existing) &&
            !std::memcmp(existing.data + 4, geometry.data() + 4, geometry.size() - 4)) return key;
    for (const auto& [key, existing] : newBoxes_)
        if (!std::memcmp(existing.data() + 4, geometry.data() + 4, geometry.size() - 4)) return key;

    u32 key = 0x46534200; // editor geometry keys, skipping all keys in the layout
    while (geometries_.count(key) || newBoxes_.count(key)) ++key;
    Put(geometry, 0, key);
    newBoxes_.emplace(key, geometry);
    return key;
}

bool NfswSmackableWrite::PatchBindings(std::vector<u8>& section,
                                      const std::vector<nfs::NfswSmackableLink>& links) {
    std::map<std::pair<u32, u32>, const nfs::NfswSmackableLink*> pending;
    for (const auto& link : links) {
        if (!link.BindingChanged() && !link.halfSizes) continue;
        const ByteSpan original{ link.spawn.data(), link.spawn.size() };
        if (!pending.emplace(std::make_pair(link.entry, original.read<u32>(0x2C)), &link).second)
            return false;
    }
    if (pending.empty()) return true;

    nfs::ChunkReader reader({ section.data(), section.size() }, 0);
    nfs::Chunk chunk;
    while (reader.Next(chunk) && !pending.empty()) {
        if (chunk.id != nfs::kChunkSmackableSpawners) continue;
        const ByteSpan data = SpawnPayload(chunk);
        for (size_t i = 0; i < data.read<u16>(10); ++i) {
            const ByteSpan spawn = data.sub(16 + i * kSpawnSize, kSpawnSize);
            const auto match = pending.find({ spawn.read<u32>(0x28), spawn.read<u32>(0x2C) });
            if (match == pending.end() || std::memcmp(spawn.data, match->second->spawn.data(), kSpawnSize))
                continue;
            const size_t at = (size_t)(spawn.data - section.data()) + nfs::kSmackableAttributeOffset;
            Put(section, at, match->second->physics);
            if (match->second->halfSizes) {
                const u32 geometry = GeometryFor(*match->second);
                if (!geometry) return false;
                Put(section, at - 4, geometry);
            }
            pending.erase(match);
        }
    }
    return pending.empty();
}

bool NfswSmackableWrite::Start(const nfs::World& world, ByteSpan stockLayout, ByteSpan editedLayout) {
    const ByteSpan stock = FindTop(stockLayout, kSceneryEntries).raw;
    const ByteSpan layout = FindTop(editedLayout, kSceneryEntries).raw;
    const ByteSpan baseline = FindTop(world.LayoutBytes(), kSceneryEntries).raw;
    if (stock.empty() || stock.size % 6 || layout.size % 6 || layout.size < stock.size ||
        layout.size / 6 > 65536 || baseline.size < stock.size || baseline.size % 6) return false;
    stockCount_ = stock.size / 6;
    entries_.resize(layout.size / 6);
    for (size_t i = 0; i < entries_.size(); ++i)
        entries_[i] = { layout.read<u16>(i * 6), layout.read<u16>(i * 6 + 2),
                        layout.read<u16>(i * 6 + 4) };
    baselineEntries_.resize(baseline.size / 6);
    for (size_t i = 0; i < baselineEntries_.size(); ++i)
        baselineEntries_[i] = { baseline.read<u16>(i * 6), baseline.read<u16>(i * 6 + 2),
                                baseline.read<u16>(i * 6 + 4) };
    nfs::NfswReadSmackableTypes(world.LayoutBytes(), types_);
    for (const auto& section : world.Loaded()) {
        if (!section->info || section->smackableBoxes.empty()) continue;
        nfs::ChunkReader reader(world.SectionBytes(*section->info), 0);
        nfs::Chunk chunk;
        while (reader.Next(chunk)) {
            if (chunk.id != nfs::kChunkSmackableSpawners) continue;
            const ByteSpan data = SpawnPayload(chunk);
            for (size_t i = 0; i < data.read<u16>(10); ++i) {
                const ByteSpan spawn = data.sub(16 + i * kSpawnSize, kSpawnSize);
                if (!IsCopy(spawn, stockCount_, entries_.size())) stockIds_.insert(spawn.read<u32>(0x2C));
                const u32 entry = spawn.read<u32>(0x28);
                if (entry >= baselineEntries_.size() || !types_.count(spawn.read<u32>(0x20)) ||
                    (entry >= stockCount_ && !IsCopy(spawn, stockCount_, entries_.size()))) continue;
                const u16 owner = baseline.read<u16>(entry * 6);
                const u16 record = baseline.read<u16>(entry * 6 + 2);
                const u16 flags = baseline.read<u16>(entry * 6 + 4);
                if (owner != data.read<u16>(8) || (flags & 0x18)) continue;
                const auto& scenery = section->scenery;
                const auto instance = std::find_if(scenery.instances.begin(), scenery.instances.end(),
                    [&](const auto& item) { return item.sceneryNumber == owner && item.record == record; });
                if (instance == scenery.instances.end()) continue;
                const u32 hash = scenery.ModelHash(*instance);
                if (!hash) continue;
                const auto [at, inserted] = models_.try_emplace(hash, Template{ {}, flags });
                if (inserted)
                    std::copy_n(spawn.data, kSpawnSize, at->second.spawn.begin());
                else if (std::memcmp(at->second.spawn.data() + 0x1C, spawn.data + 0x1C, 12) != 0)
                    at->second.ambiguous = true;
            }
        }
    }
    return true;
}

bool NfswSmackableWrite::RewriteSection(ByteSpan section, const nfs::Scenery& stock,
                                      const nfs::Scenery& baseline, const nfs::Scenery& edited,
                                      const Copies& copies,
                                      const std::vector<size_t>& changed, std::set<size_t>& dynamic,
                                      std::vector<u8>& out, std::string& error) {
    std::map<u16, std::vector<Spawn>> additions;
    std::map<std::pair<u16, u16>, u32> copyFlags;
    std::map<size_t, nfs::Chunk> spawners;
    std::map<u32, Spawn> moved;
    std::set<u32> removed;
    const auto findRecord = [](const nfs::Scenery& scenery, const Entry& entry) {
        return std::find_if(scenery.instances.begin(), scenery.instances.end(), [&](const auto& instance) {
            return instance.sceneryNumber == entry[0] && instance.record == entry[1];
        });
    };
    nfs::ChunkReader reader(section, 0);
    nfs::Chunk chunk;
    while (reader.Next(chunk)) {
        if (chunk.id != nfs::kChunkSmackableSpawners) continue;
        const ByteSpan data = SpawnPayload(chunk);
        if (data.empty()) return false;
        spawners.emplace((size_t)chunk.headerOffset, chunk);
        for (size_t i = 0; i < data.read<u16>(10); ++i) {
            const ByteSpan spawn = data.sub(16 + i * kSpawnSize, kSpawnSize);
            const u32 entry = spawn.read<u32>(0x28);
            const bool editorCopy = IsCopy(spawn, stockCount_, baselineEntries_.size());
            if (entry >= baselineEntries_.size() || (entry >= stockCount_ && !editorCopy) ||
                baselineEntries_[entry][0] != data.read<u16>(8) ||
                (baselineEntries_[entry][2] & 0x18)) continue;
            const auto before = findRecord(baseline, baselineEntries_[entry]);
            if (before == baseline.instances.end()) continue;
            if (!entries_[entry][0] && (entries_[entry][2] & 0x10)) {
                removed.insert(entry);
                if (editorCopy) entries_[entry] = kVacantEntry;
                continue;
            }
            const auto after = findRecord(edited, entries_[entry]);
            if (after == edited.instances.end() || before->guid != after->guid) continue;
            dynamic.insert((size_t)(after - edited.instances.begin()));
            if (!std::memcmp(&before->transform, &after->transform, sizeof(Mat4))) continue;
            Spawn copy;
            std::copy_n(spawn.data, copy.size(), copy.begin());
            if (!MoveSpawn(copy, before->transform, after->transform)) {
                error = "a dynamic scenery instance only supports translation and rotation";
                return false;
            }
            moved.emplace(entry, copy);
        }
    }

    std::set<std::pair<u32, u32>> native;
    for (const auto& instance : stock.instances)
        native.emplace(instance.sceneryNumber, instance.guid);
    size_t slot = stockCount_;
    size_t exactCopies = 0;
    for (size_t index = 0; index < edited.instances.size(); ++index) {
        const auto& instance = edited.instances[index];
        const auto exact = copies.find({ instance.sceneryNumber, instance.record });
        Spawn spawn;
        u16 entryFlags;
        if (exact != copies.end()) {
            const auto& source = *exact->second;
            if ((source.flags & 0x18) || !types_.count(source.link.geometry)) {
                error = "the copied dynamic scenery has an event registration or missing collision geometry";
                return false;
            }
            spawn = source.link.spawn;
            Put(spawn, nfs::kSmackableAttributeOffset, source.link.physics);
            if (source.link.halfSizes) {
                const u32 geometry = GeometryFor(source.link);
                if (!geometry) { error = "the copied collision box could not be resized"; return false; }
                Put(spawn, 0x20, geometry);
            }
            entryFlags = source.flags;
            if (!MoveSpawn(spawn, source.sourceTransform, instance.transform)) {
                error = "a dynamic scenery copy only supports translation and rotation";
                return false;
            }
            ++exactCopies;
        } else {
            if (dynamic.count(index) || !std::binary_search(changed.begin(), changed.end(), index) ||
                native.count({ instance.sceneryNumber, instance.guid })) continue;
            const auto source = models_.find(edited.ModelHash(instance));
            if (source == models_.end() || source->second.ambiguous) continue;
            std::array<float, 4> rotation;
            if (!SpawnRotation(instance.transform, rotation)) continue;
            const Vec3 position = ToPhysics(instance.transform.TransformPoint({}));
            if (!std::isfinite(Length(position))) return false;
            spawn = source->second.spawn;
            entryFlags = source->second.flags;
            Put(spawn, 0, rotation);
            Put(spawn, 16, position);
        }
        if (instance.sceneryNumber > INT16_MAX || instance.record > UINT16_MAX) return false;
        for (; slot < 65536; ++slot) {
            if (slot == entries_.size()) {
                if (slot + 64 > 65536) return false;
                entries_.resize(slot + 64, kVacantEntry);
            }
            if (entries_[slot] == kVacantEntry && !stockIds_.count(kCopySpawnId | (u32)slot)) break;
        }
        if (slot == 65536) return false;

        const u16 owner = (u16)instance.sceneryNumber, record = (u16)instance.record;
        entries_[slot] = { owner, record, entryFlags };
        const ByteSpan reference{ spawn.data(), spawn.size() };
        const u32 flags = (reference.read<u32>(0x30) & 0xFF7F0000u) | entryFlags;
        Put(spawn, 0x28, (u32)slot);
        Put(spawn, 0x2C, kCopySpawnId | (u32)slot);
        Put(spawn, 0x30, flags);
        Put(spawn, 0x38, u64(0)); // runtime pointers
        additions[owner].push_back(spawn);
        copyFlags[{ owner, record }] = flags;
        dynamic.insert(index);
        ++slot;
    }
    if (exactCopies != copies.size()) return false;
    if (additions.empty() && moved.empty() && removed.empty()) return true;

    out.assign(section.begin(), section.end());
    reader = nfs::ChunkReader(section, 0);
    while (reader.Next(chunk)) {
        if (chunk.id != nfs::kChunkTrackScenery) continue;
        const auto header = nfs::FindChunk(chunk, nfs::kChunkSceneryHeader);
        const auto instances = nfs::FindChunk(chunk, nfs::kChunkSceneryInstances);
        const u16 owner = (u16)header.data.read<u32>(0x0C);
        auto at = copyFlags.lower_bound({ owner, 0 });
        while (at != copyFlags.end() && at->first.first == owner) {
            const size_t record = (size_t)at->first.second * 96;
            if (record + 96 > instances.data.size) return false;
            Put(out, (size_t)instances.dataOffset + record + 12, at->second);
            at = copyFlags.erase(at);
        }
    }
    if (!copyFlags.empty()) return false;
    const auto writePack = [&](size_t start, size_t oldEnd, ByteSpan old, u16 owner,
                               const std::vector<Spawn>& added) {
        std::vector<u8> payload(16);
        if (!old.empty()) std::copy_n(old.data, 16, payload.begin());
        else {
            Put(payload, 0, u32(11));
            Put(payload, 4, u32(11));
            Put(payload, 8, owner);
        }
        for (size_t i = 0; i < old.read<u16>(10); ++i) {
            const ByteSpan spawn = old.sub(16 + i * kSpawnSize, kSpawnSize);
            const u32 entry = spawn.read<u32>(0x28);
            if (removed.count(entry)) continue;
            const auto replacement = moved.find(entry);
            if (replacement != moved.end())
                payload.insert(payload.end(), replacement->second.begin(), replacement->second.end());
            else
                payload.insert(payload.end(), spawn.begin(), spawn.end());
        }
        for (const auto& spawn : added) payload.insert(payload.end(), spawn.begin(), spawn.end());
        const size_t count = (payload.size() - 16) / kSpawnSize;
        if (count > kMaxSpawns) return false;
        Put(payload, 10, (u16)count);

        std::vector<u8> bytes(8, 0);
        bytes.resize(8 + (16 - (start + 8) % 16) % 16, 0x11);
        bytes.insert(bytes.end(), payload.begin(), payload.end());
        // Later chunks can contain 128-byte aligned vertices and textures.
        bytes.resize(bytes.size() + (oldEnd + 128 - (start + bytes.size()) % 128) % 128, 0);
        Put(bytes, 0, (u32)nfs::kChunkSmackableSpawners);
        Put(bytes, 4, (u32)(bytes.size() - 8));
        out.erase(out.begin() + start, out.begin() + oldEnd);
        out.insert(out.begin() + start, bytes.begin(), bytes.end());
        return true;
    };
    for (auto it = spawners.rbegin(); it != spawners.rend(); ++it) {
        const auto& [start, pack] = *it;
        const ByteSpan old = SpawnPayload(pack);
        const u16 owner = old.read<u16>(8);
        const auto wanted = additions.find(owner);
        bool replace = wanted != additions.end();
        for (size_t i = 0; i < old.read<u16>(10) && !replace; ++i) {
            const ByteSpan spawn = old.sub(16 + i * kSpawnSize, kSpawnSize);
            const u32 entry = spawn.read<u32>(0x28);
            replace = moved.count(entry) || removed.count(entry);
        }
        if (!replace) continue;
        const std::vector<Spawn> empty;
        if (!writePack(start, start + 8 + pack.raw.size, old, owner,
                       wanted == additions.end() ? empty : wanted->second)) return false;
        if (wanted != additions.end()) additions.erase(wanted);
    }
    for (const auto& [owner, spawns] : additions)
        if (!writePack(out.size(), out.size(), {}, owner, spawns)) return false;
    return true;
}

bool NfswSmackableWrite::WriteLayout(std::vector<u8>& layout) const {
    const auto table = FindTop({ layout.data(), layout.size() }, kSceneryEntries);
    const size_t length = entries_.size() * 6;
    if (!table.id || length < table.raw.size || (length - table.raw.size) % 128) return false;
    const size_t start = (size_t)table.headerOffset + 8;
    layout.insert(layout.begin() + start + table.raw.size, length - table.raw.size, 0);
    Put(layout, start - 4, (u32)length);
    for (size_t i = 0; i < entries_.size(); ++i)
        Put(layout, start + i * 6, entries_[i]);
    if (newBoxes_.empty()) return true;

    const auto list = FindTop({ layout.data(), layout.size() }, nfs::kSmackableGeometryList);
    if (!list.id) return false;
    const size_t end = (size_t)list.headerOffset + 8 + list.raw.size;
    // Adding 128 bytes per child preserves the loader's alignment for later layout chunks.
    constexpr size_t recordSize = 128;
    const size_t prefix = (16 - (end + 8) % 16) % 16;
    std::vector<u8> bytes(newBoxes_.size() * recordSize);
    size_t at = 0;
    for (const auto& [key, geometry] : newBoxes_) {
        Put(bytes, at, nfs::kSmackableGeometry);
        Put(bytes, at + 4, u32(recordSize - 8));
        std::fill_n(bytes.begin() + at + 8, prefix, 0x11);
        std::copy(geometry.begin(), geometry.end(), bytes.begin() + at + 8 + prefix);
        at += recordSize;
    }
    layout.insert(layout.begin() + end, bytes.begin(), bytes.end());
    Put(layout, (size_t)list.headerOffset + 4, (u32)(list.raw.size + bytes.size()));
    return true;
}
