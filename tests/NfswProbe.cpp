// Headless validation of the NFS World format layer. It reads the shipped
// files only and never opens the editor's window.

#include "io/NfswPlacementWrite.h"
#include "io/NfswAttributeOverlay.h"
#include "io/SelectionImport.h"
#include "io/CollisionEdit.h"
#include "io/CollisionGenerate.h"
#include "nfs/ChunkIds.h"
#include "nfs/ChunkReader.h"
#include "nfs/NfswCollision.h"
#include "nfs/NfswFormats.h"
#include "nfs/SectionTable.h"
#include "nfs/StreamingModel.h"
#include "nfs/TextureDecode.h"
#include "nfs/TexturePack.h"
#include "nfs/World.h"
#include "roads/NfswRoadBake.h"
#include "roads/NfswRoadTemplate.h"
#include "roads/RoadNetwork.h"

#include <windows.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <cstring>
#include <fstream>
#include <iterator>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace {

std::wstring Tracks() {
    // The reference install; FRS_NFSW_TRACKS points it at another machine.
    const wchar_t* env = _wgetenv(L"FRS_NFSW_TRACKS");
    return env && *env ? env : L"E:/Need For Speed World/TRACKS";
}

// 0x34C03 must describe every non-zero 0x3410E/0x3410F record exactly once.
// A record outside all ranges is never submitted by the vectorized cull path.
bool VectorRangesCover(ByteSpan section, u32 sceneryNumber, std::string& error) {
    nfs::ChunkReader top(section, 0);
    nfs::Chunk chunk;
    while (top.Next(chunk)) {
        if (chunk.id != nfs::kChunkTrackScenery) continue;
        u32 number = 0;
        ByteSpan vectorized, cull;
        nfs::ChunkReader children = nfs::ChunkReader::Children(chunk);
        nfs::Chunk child;
        while (children.Next(child)) {
            if (child.id == nfs::kChunkSceneryHeader && child.data.size >= 0x10)
                number = child.data.read<u32>(0x0C);
            else if (child.id == nfs::kChunkSceneryVectorized)
                vectorized = child.data;
            else if (child.id == nfs::kChunkSceneryPrecullNodes)
                cull = child.data;
        }
        if (number != sceneryNumber) continue;
        if (vectorized.empty() || cull.empty()) return true;

        const size_t entries = cull.size / 32;
        std::vector<unsigned char> covered(entries, 0);
        const size_t records = std::min<size_t>(64, vectorized.size / 20);
        std::map<std::pair<u32, u32>, bool> ranges;
        for (size_t type = 0; type < records; ++type) {
            const u32 count = vectorized.read<u32>(type * 20 + 4);
            const u32 first = vectorized.read<u32>(type * 20 + 8);
            if (!count) continue;
            if ((u64)first + count > entries) {
                error = "0x34C03 range outside 0x3410F";
                return false;
            }
            ranges[{ first, count }] = true;
        }
        for (const auto& range : ranges) {
            for (u32 i = 0; i < range.first.second; ++i) ++covered[range.first.first + i];
        }
        for (size_t i = 0; i < entries; ++i) {
            bool nonzero = false;
            for (size_t byte = 0; byte < 32 && !nonzero; ++byte)
                nonzero = cull.data[i * 32 + byte] != 0;
            if ((covered[i] != 0) != nonzero || covered[i] > 1) {
                error = "0x34C03 does not cover 0x3410F exactly once";
                return false;
            }
        }
        return true;
    }
    return true;
}

bool ReadFile(const std::wstring& path, std::vector<unsigned char>& out) {
    FILE* file = nullptr;
    if (::_wfopen_s(&file, path.c_str(), L"rb") != 0 || !file) return false;
    std::fseek(file, 0, SEEK_END);
    const long size = std::ftell(file);
    std::fseek(file, 0, SEEK_SET);
    out.resize((size_t)size);
    const bool ok = size > 0 && std::fread(out.data(), 1, out.size(), file) == out.size();
    std::fclose(file);
    return ok;
}

bool WriteFile(const std::wstring& path, const std::vector<u8>& data) {
    FILE* file = nullptr;
    if (::_wfopen_s(&file, path.c_str(), L"wb") != 0 || !file) return false;
    const bool ok = std::fwrite(data.data(), 1, data.size(), file) == data.size();
    return std::fclose(file) == 0 && ok;
}

bool CheckZ0StreamParts(const nfs::World& world, ByteSpan layout, ByteSpan z0, size_t& entry) {
    nfs::SectionTable table;
    if (!table.ParseNfsw(layout)) return false;
    const nfs::Section* before = world.Table().FindByNumber(25000);
    const nfs::Section* after = table.FindByNumber(25000);
    if (!before || !after) return false;
    entry = (size_t)after->tableEntryOffset;
    const ByteSpan original = world.SectionBytes(*before);
    const i64 delta = (i64)z0.size - (i64)original.size;
    for (size_t part = 0; part < 5; ++part) {
        const size_t field = 0x60 + 12 * part;
        for (size_t word = 0; word < 12; word += 4) {
            const u32 stock = world.LayoutBytes().read<u32>(before->tableEntryOffset + field + word);
            if ((i64)layout.read<u32>(entry + field + word) != (i64)stock + delta) return false;
        }
        const u32 end = layout.read<u32>(entry + field);
        if (part == 4) {
            if (end != z0.size) return false;
        } else {
            const u32 texture = (end + 127) & ~127u;
            const u32 size = z0.read<u32>(texture + 4);
            const size_t stockTexture = (size_t)((i64)texture - delta);
            if (z0.read<u32>(texture) != 0xB3300011 || texture + 8ull + size > z0.size ||
                stockTexture + 8ull + size > original.size ||
                std::memcmp(z0.data + texture, original.data + stockTexture, 8ull + size) != 0)
                return false;
        }
    }
    return true;
}

bool CheckStockCollisionAlignment(const nfs::World& world) {
    const auto* info = world.Table().FindByNumber(100);
    const auto loaded = std::find_if(world.Loaded().begin(), world.Loaded().end(),
        [&](const auto& section) { return section->info == info; });
    if (loaded == world.Loaded().end()) return false;
    const nfs::LoadedSection& section = **loaded;
    const auto pack = std::find_if(section.collision.begin(), section.collision.end(),
        [](const auto& collision) { return collision.sectionNumber == 100; });
    if (pack == section.collision.end() || section.smackableBoxes.empty()) return false;
    const nfs::NfswSmackableBox& pole = section.smackableBoxes.front();
    if (Length(pole.front() - Vec3{ 2630.9637f, 2259.9560f, 135.7051f }) > 0.002f ||
        Length(pole.back() - Vec3{ 2631.5939f, 2259.5337f, 138.9278f }) > 0.002f) return false;

    // Compare the stock CI 0 strip faces to A100's rendered ground. With
    // the mirrored local axes only 50 of these 160 samples meet the road.
    const Vec3 origin{ 2603.5f, 2058.5f, 128.25f };
    std::vector<std::array<Vec3, 3>> triangles;
    for (const auto& instance : section.scenery.instances) {
        if (Length(instance.worldBounds.Center() - origin) > 150) continue;
        const auto* model = world.FindModel(section.scenery.ModelHash(instance));
        if (!model) continue;
        for (size_t i = 0; i + 2 < model->indices.size(); i += 3) {
            const auto point = [&](size_t v) {
                return instance.transform.TransformPoint(model->vertices[model->indices[i + v]].position);
            };
            const Vec3 a = point(0), b = point(1), c = point(2);
            if (std::abs(Normalize(Cross(b - a, c - a)).z) > 0.7f)
                triangles.push_back({ a, b, c });
        }
    }
    size_t aligned = 0, samples = 0;
    for (const auto& polygon : pack->polygons) {
        if (!polygon.nfswCollisionSource || polygon.Vertical() ||
            (polygon.nfswCollisionSource >> 32 & 0xFFFF) != 0) continue;
        const Vec3 p = (polygon.corners[0] + polygon.corners[1] + polygon.corners[2]) * (1.0f / 3.0f);
        const Ray ray{ p + Vec3{ 0, 0, 10 }, { 0, 0, -1 } };
        for (const auto& t : triangles) {
            const float d = IntersectRayTriangle(ray, t[0], t[1], t[2]);
            if (std::abs(d - 10.0f) < 0.15f) {
                ++aligned;
                break;
            }
        }
        ++samples;
    }
    size_t smackables = 0;
    for (const auto& loadedSection : world.Loaded()) smackables += loadedSection->smackableBoxes.size();
    std::printf("NFSW collision: %zu/%zu A100 road samples align, %zu smackable boxes loaded\n",
                aligned, samples, smackables);
    return samples == 160 && aligned == samples && smackables > 20000;
}

std::vector<float> NativeBoundaryHits(ByteSpan mesh, const Vec3& origin, const Ray& ray) {
    const size_t start = 16 + mesh.read<u16>(2);
    const size_t count = mesh.read<u16>(4);
    if (mesh.read<u16>(6) != count * 32 || start + count * 32 > mesh.size) return {};
    std::vector<float> hits;
    for (size_t i = 0; i < count; ++i) {
        const ByteSpan record = mesh.sub(start + i * 32, 32);
        if (record.read<u16>(12) != 0x200) continue;
        const Vec3 a = origin + Vec3{ record.read<float>(0), record.read<float>(4), record.read<float>(8) };
        const Vec3 b = origin + Vec3{ record.read<float>(16), record.read<float>(20), record.read<float>(24) };
        if (ray.origin.y <= std::min(a.y, b.y) || ray.origin.y >= std::max(a.y, b.y)) continue;
        const Vec3 wall = b - a, delta = a - ray.origin;
        const float length = std::hypot(wall.x, wall.z);
        if (std::abs(record.read<float>(28) * length - 1.0f) > 0.001f) return {};
        const float denom = ray.direction.x * wall.z - ray.direction.z * wall.x;
        if (std::abs(denom) < 1e-5f) continue;
        const float distance = (delta.x * wall.z - delta.z * wall.x) / denom;
        const float across = (delta.x * ray.direction.z - delta.z * ray.direction.x) / denom;
        if (distance >= 0 && across >= 0 && across <= 1) hits.push_back(distance);
    }
    std::sort(hits.begin(), hits.end());
    hits.erase(std::unique(hits.begin(), hits.end(), [](float a, float b) {
        return std::abs(a - b) < 0.001f;
    }), hits.end());
    return hits;
}

bool CheckCopiedSpawner(ByteSpan section, ByteSpan entries, const nfs::SceneryInstance& copy,
                         u32 geometry, u32 attributes, float rotationY) {
    nfs::ChunkReader reader(section, 0);
    nfs::Chunk chunk;
    size_t matches = 0;
    while (reader.Next(chunk)) {
        if (chunk.id != nfs::kChunkSmackableSpawners) continue;
        const size_t prefix = (size_t)((16 - (chunk.headerOffset + 8) % 16) % 16);
        const ByteSpan data = chunk.raw.sub(prefix, chunk.raw.size);
        if (data.read<u16>(8) != copy.sceneryNumber) continue;
        for (size_t i = 0; i < data.read<u16>(10); ++i) {
            const ByteSpan spawn = data.sub(16 + i * 64, 64);
            const size_t entry = (size_t)spawn.read<u32>(0x28) * 6;
            if (entry + 6 > entries.size || entries.read<u16>(entry) != copy.sceneryNumber ||
                entries.read<u16>(entry + 2) != copy.record) continue;
            if (spawn.read<u32>(0x20) != geometry || spawn.read<u32>(0x24) != attributes ||
                spawn.read<u64>(0x38) || spawn.read<u32>(0x30) != copy.flags ||
                std::abs(spawn.read<float>(4) - rotationY) > 0.001f) return false;
            const Vec3 position{ spawn.read<float>(24), -spawn.read<float>(16), spawn.read<float>(20) };
            if (Length(position - copy.transform.TransformPoint({})) > 0.002f) return false;
            ++matches;
        }
    }
    return matches == 1;
}

bool CheckCopiedCollision(nfs::World& world, const std::wstring& folder) {
    const nfs::SolidObject* road = nullptr;
    const nfs::SolidObject* phone = nullptr;
    world.LoadAllSections();
    for (const auto* model : world.AllModels()) {
        if (model->name == "SHAPE02_CHOP_F115_R0") road = model;
        if (model->name == "XO_PHONEBOOTHA_1B_00") phone = model;
    }
    if (!phone) {
        for (const auto& section : world.Loaded())
            for (const auto& info : section->scenery.infos)
                if (info.name == "XOs_PhoneBoothA_1b_00") phone = world.FindModel(info.solidHash);
    }
    const auto target = std::find_if(world.Loaded().begin(), world.Loaded().end(),
        [](const auto& section) { return section->info && section->info->number == 2410; });
    const auto b832 = std::find_if(world.Loaded().begin(), world.Loaded().end(),
        [](const auto& section) { return section->info && section->info->number == 1832; });
    constexpr u32 generatorHash = 0x65E7C2B8;
    const auto* generator = world.FindModel(generatorHash);
    if (target == world.Loaded().end() || b832 == world.Loaded().end() ||
        !road || !phone || !generator || !road->HasNfswRoadSurface()) return false;
    const auto add = [](nfs::LoadedSection& section, const nfs::SolidObject& model, const Mat4& transform) {
        nfs::PlacedInstance item;
        item.modelHash = model.hash;
        item.modelName = model.name;
        item.modelBounds = model.bounds;
        item.transform = transform;
        section.placements.push_back({ section.info->number, item });
    };
    const Mat4 phoneTransform = Mat4::Translation({ 5473.6f, 461.1f, 48.2f });
    const Mat4 roadTransform = Mat4::Translation({ 6200, 701, 172 });
    add(**target, *phone, phoneTransform);
    add(**target, *road, roadTransform);
    const Mat4 generatorTransform = nfs::ComposePlacement((*b832)->info->position + Vec3{ 4, 0, 0 },
                                                         90, 0, 0, 1);
    add(**b832, *generator, generatorTransform);
    std::vector<nfs::CollisionPolygon> ground;
    collision::GenerateNfswPlacementCollision(*road, roadTransform, ground);
    if (ground.empty() || std::any_of(ground.begin(), ground.end(),
        [](const auto& face) { return face.Vertical() || face.surface != nfs::kSurfaceRoad; })) return false;
    Vec3 roadPoint = (ground.front().corners[0] + ground.front().corners[1] +
                      ground.front().corners[2]) * (1.0f / 3.0f);
    roadPoint = { -roadPoint.y, roadPoint.z, roadPoint.x };

    std::wstring report;
    if (!WriteNfswPlacementOverlays(world, folder, report)) {
        std::printf("NFSW copied phone, road and registered generator: %ls\n", report.c_str());
        return false;
    }
    const std::wstring tracks = folder + L"\\files\\TRACKS\\";
    std::vector<u8> saved;
    if (!ReadFile(tracks + L"STREAML5RA_25000.BUN", saved)) return false;
    const std::wstring generatorPath = tracks + L"STREAML5RA_1832.BUN";
    std::vector<u8> generatorSaved, generatorResaved;
    std::string error;
    if (!ReadFile(generatorPath, generatorSaved) ||
        !WriteNfswPlacementOverlays(world, folder, report) ||
        !ReadFile(generatorPath, generatorResaved) || generatorResaved != generatorSaved ||
        !VectorRangesCover({ generatorSaved.data(), generatorSaved.size() }, 1832, error)) return false;
    bool roadHit = false;
    nfs::ChunkReader reader({ saved.data(), saved.size() }, 0);
    nfs::Chunk chunk;
    while (reader.Next(chunk)) {
        if (chunk.id != 0x3B801) continue;
        const size_t padding = (size_t)((16 - (chunk.headerOffset + 8) % 16) % 16);
        const ByteSpan wrapper = chunk.raw.sub(padding, chunk.raw.size);
        const ByteSpan pack = wrapper.sub(16, wrapper.read<u32>(0));
        constexpr char label[] = "FRSPlacementCollision";
        const size_t name = 32 + pack.read<u32>(44);
        if (name + sizeof(label) > pack.size ||
            std::memcmp(pack.data + name, label, sizeof(label) - 1) != 0) continue;
        const size_t ci = 32 + (pack.read<u32>(24) - 1) * 16;
        const size_t instances = ci + pack.read<u32>(ci + 12);
        for (size_t i = 0; i < pack.read<u32>(ci + 8); ++i) {
            const size_t at = instances + i * 64;
            const Vec3 origin{ -pack.read<float>(at + 48), -pack.read<float>(at + 52),
                               -pack.read<float>(at + 56) };
            const size_t ca = 48 + pack.read<u16>(at + 26) * 16;
            const ByteSpan mesh = pack.sub(ca + pack.read<u32>(ca + 12), pack.read<u32>(ca + 4) >> 8);
            if (mesh.read<u16>(4)) {
                if (std::abs(origin.x + 461.1f) < 3 && std::abs(origin.z - 5473.6f) < 3) return false;
                continue;
            }
            const size_t strips = mesh.read<u16>(0);
            if (mesh.read<u32>(16 + mesh.read<u16>(2)) != 0x6CA26F9B) continue;
            for (size_t i = 0; i < strips; ++i) {
                const size_t sphere = 16 + i * 16;
                const size_t vertices = 16 + mesh.read<u16>(sphere + 14);
                const Vec3 center = origin + Vec3{ mesh.read<float>(sphere), mesh.read<float>(sphere + 4),
                                                    mesh.read<float>(sphere + 8) };
                Vec3 p[3];
                for (size_t v = 0; v < 3; ++v) {
                    const size_t vertex = vertices + v * 8;
                    p[v] = center + Vec3{ mesh.read<i16>(vertex) / 128.0f,
                                          mesh.read<i16>(vertex + 2) / 128.0f,
                                          mesh.read<i16>(vertex + 4) / 128.0f };
                }
                Vec3 normal = Cross(p[1] - p[0], p[2] - p[0]);
                if (mesh.read<u16>(vertices + 14) & 1) normal = -normal;
                const float height = IntersectRayTriangle({ roadPoint + Vec3{ 0, 10, 0 }, { 0, -1, 0 } },
                                                          p[0], p[1], p[2]);
                roadHit |= normal.y > 0 && std::abs(height - 10) < 0.02f;
            }
        }
    }
    nfs::World reopened;
    reopened.SetOverlayFolder(folder);
    if (!reopened.Open(world.TracksDirectory(), world.Region())) return false;
    const auto* sectionInfo = reopened.Table().FindByNumber(2410);
    const auto* section = sectionInfo ? reopened.LoadSection(*sectionInfo) : nullptr;
    ByteSpan entries;
    nfs::ChunkReader layoutReader(reopened.LayoutBytes(), 0);
    while (layoutReader.Next(chunk))
        if (chunk.id == 0x34108) { entries = chunk.raw; break; }
    bool phoneCopy = false;
    if (section && section->nfswEditedScenery.size() == 2) {
        for (size_t index : section->nfswEditedScenery) {
            const auto& copy = section->scenery.instances[index];
            if (section->scenery.ModelHash(copy) != phone->hash) continue;
            phoneCopy = section->smackableScenery.count(index) &&
                CheckCopiedSpawner(reopened.SectionBytes(*sectionInfo), entries, copy,
                                          0x06326A8Bu, 0xA98D0CB7u, 0);
        }
    }
    const auto* generatorInfo = reopened.Table().FindByNumber(1832);
    const auto* generatorSection = generatorInfo ? reopened.LoadSection(*generatorInfo) : nullptr;
    bool generatorCopy = false;
    u32 generatorRecord = ~0u;
    if (generatorSection) {
        const auto& scenery = generatorSection->scenery;
        for (size_t index : generatorSection->nfswEditedScenery) {
            const auto& instance = scenery.instances[index];
            if (scenery.ModelHash(instance) != generatorHash ||
                std::memcmp(&instance.transform, &generatorTransform, sizeof(Mat4)) != 0) continue;
            generatorRecord = instance.record;
            generatorCopy = generatorSection->smackableScenery.count(index) &&
                !(instance.flags & 0x00800018u) &&
                std::none_of(scenery.overrides.begin(), scenery.overrides.end(),
                    [index](const auto& entry) { return entry.instance == index; }) &&
                CheckCopiedSpawner(reopened.SectionBytes(*generatorInfo), entries, instance,
                                   0x0F25C5B2u, 0x263C5311u, -0.70710678f);
        }
    }
    reopened.LoadAllSections();
    bool deletedSpawner = false;
    if (generatorCopy && WriteNfswPlacementOverlays(reopened, folder, report) &&
        ReadFile(generatorPath, generatorResaved) && generatorResaved == generatorSaved) {
        for (const auto& candidate : reopened.Loaded()) {
            if (candidate.get() != generatorSection) continue;
            candidate->removedNatives.push_back({ 1832, generatorRecord });
            if (!WriteNfswPlacementOverlays(reopened, folder, report) ||
                !ReadFile(generatorPath, generatorResaved)) break;
            const ByteSpan bytes{ generatorResaved.data(), generatorResaved.size() };
            const nfs::Scenery remaining = nfs::NfswReadSectionScenery(bytes);
            deletedSpawner = std::none_of(remaining.instances.begin(), remaining.instances.end(),
                [&](const auto& instance) { return remaining.ModelHash(instance) == generatorHash; });
            nfs::ChunkReader spawns(bytes, 0);
            while (spawns.Next(chunk)) {
                if (chunk.id != nfs::kChunkSmackableSpawners) continue;
                const size_t prefix = (size_t)((16 - (chunk.headerOffset + 8) % 16) % 16);
                const ByteSpan data = chunk.raw.sub(prefix, chunk.raw.size);
                for (size_t i = 0; i < data.read<u16>(10); ++i)
                    if ((data.read<u32>(16 + i * 64 + 0x2C) & 0xFFFF0000u) == 0x46520000u)
                        deletedSpawner = false;
            }
            break;
        }
    }
    reopened.Close();
    for (const wchar_t* path : { L"STREAML5RA_25000.BUN", L"STREAML5RA_2410.BUN", L"L5RA.BUN" })
        ::DeleteFileW((tracks + path).c_str());
    (*target)->placements.clear();
    ::DeleteFileW(generatorPath.c_str());
    (*b832)->placements.clear();
    std::printf("NFSW copies: dynamic phone booth %d, rotated dynamic B832 generator %d, "
                "spawner removed on delete %d, road height hit %d\n",
                phoneCopy, generatorCopy, deletedSpawner, roadHit);
    return phoneCopy && generatorCopy && deletedSpawner && roadHit;
}

bool CheckNativeSmackablePose(nfs::World& world, const std::wstring& folder) {
    const auto a100 = std::find_if(world.Loaded().begin(), world.Loaded().end(),
        [](const auto& loaded) { return loaded->info && loaded->info->number == 100; });
    if (a100 == world.Loaded().end()) return false;
    nfs::LoadedSection& section = **a100;
    const auto found = section.smackableScenery.find(0);
    if (found == section.smackableScenery.end()) return false;
    const auto& source = section.scenery.instances[0];
    const auto link = section.smackableLinks[found->second];
    if (link.box == SIZE_MAX) return false;
    const auto oldBox = section.smackableBoxes[link.box];
    const auto* model = world.FindModel(section.scenery.ModelHash(source));
    if (!model) return false;
    nfs::PlacedInstance moved;
    moved.modelHash = model->hash;
    moved.modelBounds = model->bounds;
    moved.originalSection = 100;
    moved.originalSceneryNumber = source.sceneryNumber;
    moved.originalRecord = source.record;
    moved.transform = source.transform * nfs::ComposePlacement({}, 25, 0, 0, 1);
    const Vec3 position = source.transform.TransformPoint({}) + Vec3{ 2, -1, 0.5f };
    moved.transform.m[3][0] = position.x;
    moved.transform.m[3][1] = position.y;
    moved.transform.m[3][2] = position.z;
    section.placements.push_back({ source.sceneryNumber, moved });
    section.removedNatives.push_back({ source.sceneryNumber, source.record });
    std::wstring report;
    if (!WriteNfswPlacementOverlays(world, folder, report)) {
        std::printf("NFSW native smackable move: %ls\n", report.c_str());
        return false;
    }
    nfs::World check;
    const auto load = [&]() -> const nfs::LoadedSection* {
        check.SetOverlayFolder(folder);
        if (!check.Open(world.TracksDirectory(), world.Region())) return nullptr;
        const auto* info = check.Table().FindByNumber(100);
        return info ? check.LoadSection(*info) : nullptr;
    };
    const auto* shifted = load();
    if (!shifted || shifted->smackableLinks.size() != section.smackableLinks.size()) return false;
    const auto shape = shifted->smackableScenery.find(0);
    if (shape == shifted->smackableScenery.end() || shifted->smackableLinks[shape->second].entry != link.entry)
        return false;
    const size_t box = shifted->smackableLinks[shape->second].box;
    if (box == SIZE_MAX) return false;
    const Mat4 delta = InverseAffine(source.transform) * moved.transform;
    for (size_t i = 0; i < oldBox.size(); ++i)
        if (Length(delta.TransformPoint(oldBox[i]) - shifted->smackableBoxes[box][i]) > 0.02f)
            return false;
    check.Close();

    section.placements.clear();
    if (!WriteNfswPlacementOverlays(world, folder, report)) {
        std::printf("NFSW native smackable delete: %ls\n", report.c_str());
        return false;
    }
    const auto* remaining = load();
    const bool deleted = remaining && remaining->smackableLinks.size() + 1 == section.smackableLinks.size() &&
        std::none_of(remaining->smackableLinks.begin(), remaining->smackableLinks.end(),
            [&](const auto& spawn) { return spawn.entry == link.entry; }) &&
        std::none_of(remaining->scenery.instances.begin(), remaining->scenery.instances.end(),
            [&](const auto& instance) { return instance.guid == source.guid; });
    check.Close();
    section.removedNatives.clear();
    for (const wchar_t* name : { L"STREAML5RA_100.BUN", L"STREAML5RA_25000.BUN", L"L5RA.BUN" })
        ::DeleteFileW((folder + L"\\files\\TRACKS\\" + name).c_str());
    std::printf("NFSW native A100 smackable: pose verified, scenery and spawn removed %d\n", deleted);
    return deleted;
}

bool CheckExactSmackableCopy(nfs::World& world, const std::wstring& folder) {
    static constexpr u32 number = 517;
    static constexpr u32 sceneryNumber = 117;
    const auto selected = std::find_if(world.Loaded().begin(), world.Loaded().end(), [](const auto& section) {
        return section->info && section->info->number == number;
    });
    if (selected == world.Loaded().end()) {
        std::printf("NFSW construction sign: section %u not loaded\n", number);
        return false;
    }
    nfs::LoadedSection& section = **selected;
    const auto& instances = section.scenery.instances;
    const auto sign = std::find_if(instances.begin(), instances.end(), [](const auto& item) {
        return item.sceneryNumber == sceneryNumber && item.record == 46;
    });
    if (sign == instances.end()) {
        std::printf("NFSW construction sign: record missing in %zu instances\n", instances.size());
        return false;
    }
    nfs::PlacedInstance copy;
    copy.smackable = world.SmackablePlacement(section, (size_t)(sign - instances.begin()));
    copy.modelHash = section.scenery.ModelHash(*sign);
    const auto* model = world.FindModel(copy.modelHash);
    if (!copy.smackable || copy.smackable->link.geometry != 0x6966F6D1u || !model) {
        std::printf("NFSW construction sign: model %08X resolved %d, geometry %08X\n",
                    copy.modelHash, model != nullptr, copy.smackable ? copy.smackable->link.geometry : 0);
        return false;
    }
    copy.modelName = model->name;
    copy.modelBounds = model->bounds;
    copy.transform = sign->transform * nfs::ComposePlacement({}, 35, 0, 0, 1);
    const Vec3 position = sign->transform.TransformPoint({}) + Vec3{ 5, 2, 1 };
    copy.transform.m[3][0] = position.x;
    copy.transform.m[3][1] = position.y;
    copy.transform.m[3][2] = position.z;
    const u32 originalPreset = copy.smackable->link.physics;
    if (originalPreset == 0x263C5311 || !world.RebindSmackable(section, copy, 0x263C5311) ||
        world.FindSmackableLink(section, (size_t)(sign - instances.begin()))->physics != originalPreset) return false;
    const auto preview = copy.smackable->BoxAt(copy.transform);
    section.placements.push_back({ sceneryNumber, copy });
    std::wstring report;
    if (!WriteNfswPlacementOverlays(world, folder, report)) {
        std::printf("NFSW exact construction sign copy: %ls\n", report.c_str());
        return false;
    }
    section.placements.clear();
    section.placementsChanged = false;

    nfs::World reopened;
    reopened.SetOverlayFolder(folder);
    if (!reopened.Open(world.TracksDirectory(), world.Region())) return false;
    const auto* info = reopened.Table().FindByNumber(number);
    const auto* saved = info ? reopened.LoadSection(*info) : nullptr;
    if (!saved || saved->scenery.instances.size() != instances.size() + 1 ||
        saved->smackableLinks.size() != section.smackableLinks.size() + 1) {
        std::printf("NFSW construction sign: reloaded %zu/%zu scenery, %zu/%zu dynamic records\n",
                    saved ? saved->scenery.instances.size() : 0, instances.size() + 1,
                    saved ? saved->smackableLinks.size() : 0, section.smackableLinks.size() + 1);
        return false;
    }
    const auto& reloaded = saved->scenery.instances;
    const auto instance = std::find_if(reloaded.begin(), reloaded.end(), [&](const auto& item) {
        return item.sceneryNumber == sceneryNumber &&
            std::memcmp(&item.transform, &copy.transform, sizeof(Mat4)) == 0;
    });
    if (instance == reloaded.end()) return false;
    const auto actual = reopened.SmackablePlacement(*saved, (size_t)(instance - reloaded.begin()));
    if (!actual || actual->link.entry == copy.smackable->link.entry ||
        actual->link.geometry != copy.smackable->link.geometry || actual->link.physics != copy.smackable->link.physics)
        return false;
    for (size_t i = 0; i < preview.size(); ++i)
        if (Length(actual->box[i] - preview[i]) > 0.02f) return false;

    const std::wstring tracks = folder + L"\\files\\TRACKS\\";
    const std::wstring path = tracks + world.NfswSectionFileName(*section.info);
    std::vector<u8> first, second;
    if (!ReadFile(path, first) || !WriteNfswPlacementOverlays(reopened, folder, report) ||
        !ReadFile(path, second) || first != second) return false;
    reopened.Close();
    ::DeleteFileW(path.c_str());
    for (const wchar_t* name : { L"STREAML5RA_25000.BUN", L"L5RA.BUN" })
        ::DeleteFileW((tracks + name).c_str());
    std::printf("NFSW construction sign copy: rebound preset, source geometry and reopened registration verified\n");
    return true;
}

bool CheckNativeSmackableBinding(nfs::World& world, const std::wstring& folder) {
    const auto selected = std::find_if(world.Loaded().begin(), world.Loaded().end(), [](const auto& section) {
        return section->info && section->info->number == 100;
    });
    if (selected == world.Loaded().end()) return false;
    auto& section = **selected;
    const auto* link = world.FindSmackableLink(section, 0);
    constexpr u32 key = 0x263C5311;
    if (!link || link->physics == key || world.RebindSmackable(section, 0, 0)) return false;
    const u32 initial = link->physics;
    const ByteSpan baseline = world.SectionBytes(*section.info);
    std::vector<u8> expected(baseline.begin(), baseline.end());
    const auto spawn = std::search(expected.begin(), expected.end(), link->spawn.begin(), link->spawn.end());
    if (spawn == expected.end() ||
        std::search(spawn + 1, expected.end(), link->spawn.begin(), link->spawn.end()) != expected.end()) return false;
    std::memcpy(&*(spawn + nfs::kSmackableAttributeOffset), &key, sizeof(key));

    const std::wstring path = folder + L"\\files\\TRACKS\\" + world.NfswSectionFileName(*section.info);
    std::vector<u8> written;
    std::wstring report;
    if (!world.RebindSmackable(section, 0, key) || !section.placementsChanged ||
        !WriteNfswPlacementOverlays(world, folder, report) || !ReadFile(path, written) || written != expected) {
        std::printf("NFSW native Smackable binding: %ls\n", report.c_str());
        return false;
    }
    section.placementsChanged = false;
    if (!WriteNfswPlacementOverlays(world, folder, report) || !ReadFile(path, written) || written != expected)
        return false;

    nfs::World reopened;
    reopened.SetOverlayFolder(folder);
    if (!reopened.Open(world.TracksDirectory(), world.Region())) return false;
    const auto* info = reopened.Table().FindByNumber(100);
    const auto* loaded = info ? reopened.LoadSection(*info) : nullptr;
    const auto* saved = loaded ? reopened.FindSmackableLink(*loaded, 0) : nullptr;
    if (!saved || saved->physics != key || saved->entry != link->entry || saved->geometry != link->geometry)
        return false;
    reopened.Close();

    if (!world.RebindSmackable(section, 0, initial) || !WriteNfswPlacementOverlays(world, folder, report) ||
        !ReadFile(path, written) || written.size() != baseline.size ||
        std::memcmp(written.data(), baseline.data, baseline.size)) return false;
    section.placementsChanged = false;
    ::DeleteFileW(path.c_str());
    std::printf("NFSW native Smackable binding: one hash changed, reopened, resaved and restored\n");
    return true;
}

bool CheckSmackableAttributeOverlay(nfs::World& world, const std::wstring& source) {
    wchar_t temp[MAX_PATH]{}, scratch[MAX_PATH]{};
    if (!::GetTempPathW(MAX_PATH, temp) || !::GetTempFileNameW(temp, L"wsa", 0, scratch) ||
        !::DeleteFileW(scratch) || !::CreateDirectoryW(scratch, nullptr)) return false;
    const std::wstring models = scratch;
    const std::wstring overlay = models + L"\\" + nfs::kNfswAttributesOverlayPath;
    const auto mass = [&]() -> std::optional<float> {
        const auto* preset = world.FindSmackableAttributes(0x263C5311);
        return preset ? preset->mass : std::nullopt;
    };
    const auto stockMass = mass();
    std::vector<u8> input, staged;
    std::wstring error;
    const bool installed = ReadFile(source, input) && InstallNfswAttributeOverlay(source, models, error);
    world.SetOverlayFolder(models);
    const bool loaded = installed && world.ReloadSmackableAttributes() && world.UsesSmackableAttributeOverlay();
    const auto overlayMass = loaded ? mass() : std::nullopt;
    const bool rejected = installed && !InstallNfswAttributeOverlay(
        world.TracksDirectory() + L"\\" + world.Region() + L".BUN", models, error);
    const bool unchanged = installed && ReadFile(overlay, staged) && staged == input;

    ::DeleteFileW(overlay.c_str());
    ::RemoveDirectoryW((models + L"\\files\\GLOBAL").c_str());
    ::RemoveDirectoryW((models + L"\\files").c_str());
    ::RemoveDirectoryW(models.c_str());
    const bool fallback = world.ReloadSmackableAttributes() && !world.UsesSmackableAttributeOverlay() && mass() == stockMass;
    world.SetOverlayFolder(L"");
    if (!installed) std::printf("NFSW attributes import failed: %ls\n", error.c_str());
    std::printf("NFSW attribute overlay: stock mass %.6g, external mass %.6g, invalid pack rejected %d, fallback %d\n",
                stockMass.value_or(-1), overlayMass.value_or(-1), rejected, fallback);
    return loaded && overlayMass.has_value() && rejected && unchanged && fallback;
}

bool CheckSmackableBoxResize(nfs::World& world, const std::wstring& folder) {
    const auto chosen = std::find_if(world.Loaded().begin(), world.Loaded().end(), [](const auto& section) {
        return section->info && section->info->number == 100;
    });
    if (chosen == world.Loaded().end()) return false;
    auto& section = **chosen;
    const auto binding = std::find_if(section.smackableScenery.begin(), section.smackableScenery.end(),
        [&](const auto& item) { return section.smackableLinks[item.second].singleBox; });
    if (binding == section.smackableScenery.end()) return false;
    const size_t index = binding->first;
    const auto original = section.smackableLinks[binding->second];
    const auto& stockBox = section.smackableBoxes[original.box];
    const nfs::NfswSmackableHalfSizes sizes{ 600, 900, 1200 };
    if (!world.ResizeSmackableBox(section, index, sizes)) return false;
    const auto preview = section.smackableLinks[binding->second].EditedBox(stockBox);
    std::wstring report;
    if (!WriteNfswPlacementOverlays(world, folder, report)) {
        std::printf("NFSW box size save: %ls\n", report.c_str());
        return false;
    }
    nfs::World reopened;
    reopened.SetOverlayFolder(folder);
    if (!reopened.Open(world.TracksDirectory(), world.Region())) return false;
    const auto* info = reopened.Table().FindByNumber(100);
    const auto* edited = info ? reopened.LoadSection(*info) : nullptr;
    const auto* link = edited ? reopened.FindSmackableLink(*edited, index) : nullptr;
    if (!link || link->geometry == original.geometry || link->physics != original.physics || !link->singleBox ||
        !edited || edited->smackableLinks.size() != section.smackableLinks.size()) return false;
    for (size_t i = 0; i < section.smackableLinks.size(); ++i) {
        if (i == binding->second) continue;
        if (edited->smackableLinks[i].spawn != section.smackableLinks[i].spawn) return false;
    }
    const auto& actual = edited->smackableBoxes[link->box];
    for (size_t i = 0; i < actual.size(); ++i)
        if (Length(actual[i] - preview[i]) > 0.005f) return false;
    nfs::NfswSmackableTypes before, after;
    nfs::NfswReadSmackableTypes(world.LayoutBytes(), before);
    nfs::NfswReadSmackableTypes(reopened.LayoutBytes(), after);
    if (after.size() != before.size() + 1 ||
        Length(before.at(original.geometry).halfSize - after.at(original.geometry).halfSize) != 0) return false;

    const std::wstring tracks = folder + L"\\files\\TRACKS\\";
    const std::wstring layoutFile = tracks + world.Region() + L".BUN";
    std::vector<u8> first, second;
    if (!ReadFile(layoutFile, first) || !WriteNfswPlacementOverlays(world, folder, report) ||
        !ReadFile(layoutFile, second) || first != second) return false;
    reopened.Close();
    section.smackableLinks[binding->second].halfSizes.reset();
    section.placementsChanged = false;
    ::DeleteFileW(layoutFile.c_str());
    ::DeleteFileW((tracks + world.NfswSectionFileName(*section.info)).c_str());
    std::printf("NFSW single collision box: resized preview, separate geometry, unchanged neighbours and repeat save verified\n");
    return true;
}

bool CheckMissingRoadGround(nfs::World& world) {
    const Vec3 origin{ 100, 100, 2000 };
    const auto triangle = [](Vec3 p) {
        const Vec3 corners[]{ p, p + Vec3{ 6, 0, 0 }, p + Vec3{ 0, 6, 0 } };
        return collision::MakeGround(corners, 3, nfs::kSurfaceRoad);
    };
    // One triangle is already supported. An identical XY triangle one
    // story higher and a neighbouring gap both need their own collision.
    const auto existing = triangle(origin);
    std::vector<nfs::CollisionPolygon> candidates{
        existing, triangle(origin + Vec3{ 0, 0, 4 }), triangle(origin + Vec3{ 6, 0, 0 })
    };
    nfs::CollisionPack stock;
    stock.polygons.push_back(existing);
    for (const auto& corner : existing.corners) stock.bounds.Add(corner);
    auto* section = world.MutableLoaded(0);
    if (!section) return false;
    section->collision.push_back(std::move(stock));
    const size_t covered = collision::RemoveCoveredRoadGround(world, candidates);
    bool ok = covered == 1 && candidates.size() == 2 &&
              candidates[0].corners[0].z == origin.z + 4 && candidates[1].corners[0].x == origin.x + 6;

    auto& pack = section->collision.back();
    for (const auto& polygon : candidates) {
        for (const auto& corner : polygon.corners) pack.bounds.Add(corner);
        pack.polygons.push_back(polygon);
    }
    ok = ok && collision::RemoveCoveredRoadGround(world, candidates) == 2 && candidates.empty();
    section->collision.pop_back();
    if (ok) std::printf("NFSW road collision: existing ground skipped, upper deck and gap kept, repeat adds nothing\n");
    return ok;
}

bool CheckRoadAssets(nfs::World& world, const nfs::LoadedSection& loaded) {
    // These shipped meshes use map coordinates and an instance at the origin.
    const auto source = std::find_if(loaded.scenery.instances.begin(), loaded.scenery.instances.end(), [&](const auto& instance) {
        const auto* model = world.FindModel(loaded.scenery.ModelHash(instance));
        return model && model->name == "RD_FWY_ROAD_16_CHOP_A100_R0";
    });
    if (source == loaded.scenery.instances.end()) return false;
    nfs::PlacedInstance copy;
    copy.modelHash = loaded.scenery.ModelHash(*source);
    copy.modelBounds = world.FindModel(copy.modelHash)->bounds;
    copy.transform = source->transform;
    const auto anchor = world.RoadPlacementAnchor(copy);
    if (!anchor || Length(*anchor - copy.transform.TransformPoint({})) < 1000.0f) return false;
    const Vec3 destination{ 4153.25f, -99.04f, 42.0f };
    copy.transform = copy.transform * Mat4::Translation(destination - *anchor);
    const auto moved = world.RoadPlacementAnchor(copy);
    if (!moved || Length(*moved - destination) > 0.003f ||
        IntersectRayAabb({ *moved + Vec3{ 0, 0, 100 }, { 0, 0, -1 } }, copy.WorldBounds()) < 0.0f) return false;
    std::printf("NFSW road copy: map-space geometry anchored at the pasted road surface\n");

    world.LoadAllSections();
    const u32 yellow = TextureNameHash("TRN_PAVA_YL_01_D");
    const u32 white = TextureNameHash("TRN_PAVA_WD_01_D");
    const u32 junction = TextureNameHash("TRS_PAVA_INT_01_D");
    if (world.NativeRoadTexture("RDP_LANEA_YS_AA_KT") != yellow ||
        world.NativeRoadTexture("RDP_LANEB_WD_AA_KT") != white ||
        world.NativeRoadTexture("RDP_INTERSECTMIDTILE") != junction ||
        world.NativeRoadTexture("ARC_SIDEWALK_E") != 0) return false;

    // World materials are assigned to the seven existing UG2 types.
    const std::array<std::string, 7> centreByType{
        "TRN_PAVA_YL_01_D", "TRN_PAVB_YL_01_D", "TRN_PAVA_YL_01_D", "TRN_PAVB_YL_01_D",
        "TRN_PAVA_YL_01_D", "TRN_PAVB_YL_01_D", "TRN_CONCRETEHORIZCUT_YL_01_D"
    };
    const size_t bandCounts[]{ 2, 4, 5, 7, 4, 5, 2 };
    const int forwardLanes[]{ 1, 2, 2, 3, 2, 3, 1 }, backwardLanes[]{ 1, 2, 2, 3, 0, 0, 1 };
    for (size_t i = 0; i < centreByType.size(); ++i) {
        roads::RoadProfile preset;
        preset.lanesForward = forwardLanes[i];
        preset.lanesBackward = backwardLanes[i];
        roads::ApplyWorldRoadMaterials(world, (roads::WorldRoadType)i, preset);
        if (preset.laneCentre != centreByType[i] || !roads::IsWorldLaneTexture(preset.laneCentre) ||
            !roads::IsWorldLaneTexture(preset.laneInner) || !world.FindTexture(TextureNameHash(preset.laneInner)) ||
            !world.FindTexture(TextureNameHash(preset.sidewalkTexture))) return false;
        if (preset.surfaceBands.size() != bandCounts[i] || preset.ambient == 0x383C42 ||
            !world.FindModel(TextureNameHash(preset.surfaceTemplate))) return false;
        float across = -0.5f;
        for (const auto& band : preset.surfaceBands) {
            if (std::abs(band.left - across) > 0.001f || !world.FindTexture(TextureNameHash(band.texture)) ||
                std::abs(band.uvPerMetre.y) < 0.01f) return false;
            across = band.right;
        }
        if (std::abs(across - 0.5f) > 0.001f) return false;
        if (i == 0 && (std::abs(preset.laneWidth - 6.0f) > 0.03f ||
                       preset.sidewalkWidth < 3.5f || preset.surfaceBands.front().uvPerMetre.y *
                         preset.surfaceBands.back().uvPerMetre.y >= 0.0f)) return false;
    }

    roads::RoadProfile legacy;
    legacy.laneWidth = 5.0f;
    roads::ApplyWorldRoadSurface(world, legacy);
    if (legacy.laneWidth != 5.0f || legacy.surfaceBands.size() != 2 ||
        legacy.laneCentre != "TRN_PAVA_YL_01_D") return false;
    roads::Network network;
    roads::RoadProfile profile;
    profile.traffic = profile.gps = false;
    roads::ApplyWorldRoadMaterials(world, roads::WorldRoadType::TwoLane, profile);
    const auto a = network.AddNode(*anchor);
    const auto b = network.AddNode(*anchor + Vec3{ 12, 0, 0 });
    auto* penRoad = network.AddSegment(a, b, profile);
    if (!penRoad) return false;
    const auto penCurve = roads::PenCurve(*anchor, *anchor + Vec3{ 12, 0, 0 }, { 4, 4, 0 }, { 4, -4, 0 });
    const Vec3 leaveA = penCurve.p1 - penCurve.p0, leaveB = penCurve.p2 - penCurve.p3;
    penRoad->shape = 2;
    penRoad->dirA = Degrees(std::atan2(leaveA.y, leaveA.x));
    penRoad->dirB = Degrees(std::atan2(leaveB.y, leaveB.x));
    penRoad->handleA = Length(leaveA);
    penRoad->handleB = Length(leaveB);

    // Retain the surface-band data written by the previous editor.
    roads::RoadProfile sampled = profile;
    sampled.surfaceTemplate = "RD_FWY_ROAD_16_CHOP_A100_R0";
    sampled.surfaceBands = {
        { "TRN_PAVB_YL_01_D", -0.5f, 0, 0, 0, { 1, 0.1f }, { 0, 0.2f }, { 0, 0.3f } },
        { "TRN_PAVB_WD_01_D", 0, 0.5f, 0, 0, { 0, 0.2f }, { 1, 0.1f }, { 0, 0.3f } },
    };
    std::set<u32> expectedTemplateTextures;
    for (const auto& band : sampled.surfaceBands) expectedTemplateTextures.insert(TextureNameHash(band.texture));
    const auto c = network.AddNode(*anchor + Vec3{ 0, 0, 12 });
    const auto d = network.AddNode(*anchor + Vec3{ 12, 0, 12 });
    const auto* usingTemplate = network.AddSegment(c, d, sampled);
    if (!usingTemplate) return false;
    const u32 templateSegment = usingTemplate->id;
    wchar_t temp[MAX_PATH]{}, path[MAX_PATH]{};
    const DWORD length = ::GetTempPathW(MAX_PATH, temp);
    if (!length || length >= MAX_PATH || !::GetTempFileNameW(temp, L"frt", 0, path)) return false;
    roads::Network saved;
    const bool restored = network.Save(path) && saved.Load(path) && !saved.WorldRoadTextures();
    ::DeleteFileW(path);
    const auto* savedRoad = saved.FindSegment(templateSegment);
    const auto* savedPreset = saved.FindSegment(network.Segments().front().id);
    if (!restored || !savedPreset || savedPreset->profile.laneCentre != profile.laneCentre ||
        !savedRoad || savedRoad->profile.surfaceTemplate != sampled.surfaceTemplate ||
        savedRoad->profile.surfaceBands.size() != sampled.surfaceBands.size()) return false;
    const auto restoredCurve = saved.CurveOf(*savedPreset);
    if (Length(restoredCurve.p1 - penCurve.p1) > 0.01f || Length(restoredCurve.p2 - penCurve.p2) > 0.01f)
        return false;

    std::vector<NfswMeshOverlay> files;
    nfs::NfswRoadBuildReport report;
    std::wstring error;
    if (!roads::PlanWorldRoadBake(world, saved, files, report, error)) {
        std::printf("NFSW World road textures: %ls\n", error.c_str());
        return false;
    }
    std::set<u32> textures;
    bool hasWorldRoadShader = false;
    for (const auto& file : files) {
        if (file.modelsRoot || file.file.rfind(L"STREAM", 0) != 0) continue;
        nfs::ChunkReader reader({ file.bytes.data(), file.bytes.size() }, 0);
        nfs::Chunk chunk;
        while (reader.Next(chunk)) {
            if (chunk.id != nfs::kChunkSolidListPack) continue;
            nfs::SolidList list;
            if (!nfs::NfswParseSolidList(chunk, list)) continue;
            for (const auto& model : list.objects) {
                if (model.name.rfind("FRS_ROAD_", 0) != 0) continue;
                textures.insert(model.textureHashes.begin(), model.textureHashes.end());
                for (const auto& material : model.materials)
                    if (material.nfswRoadSurface && material.textureIndex < model.textureHashes.size() &&
                        model.textureHashes[material.textureIndex] == yellow &&
                        material.nfswMapIds[1] < model.textureHashes.size() &&
                        material.nfswMapIds[3] < model.textureHashes.size()) hasWorldRoadShader = true;
            }
        }
    }
    if (!hasWorldRoadShader || !textures.count(yellow) ||
        !std::includes(textures.begin(), textures.end(), expectedTemplateTextures.begin(), expectedTemplateTextures.end()))
        return false;
    std::puts("NFSW road templates: seven stock-derived cross-sections, World UVs, and native shader maps");
    std::puts("NFSW road pen: curve survives network save/load and native mesh bake");
    return true;
}

bool CheckWorldRoadBake(nfs::World& world, const nfs::LoadedSection& loaded) {
    const auto* z0 = world.Table().FindByNumber(nfs::kNfswCollisionSection);
    const auto* z0Loaded = z0 ? world.LoadSection(*z0) : nullptr;
    if (!z0Loaded || z0Loaded->texturePacks.empty() ||
        z0Loaded->texturePacks.front()->Textures().empty()) return false;
    nfs::NfswNativeRoads stock;
    if (!nfs::NfswReadNativeRoads(world.LayoutBytes(), stock)) return false;

    // A known 1+1-lane junction node and a branch through a new junction.
    const Vec3 start{ 4153.25049f, -99.03721f, 38.61115f };
    Aabb roadBounds;
    roadBounds.Add(start - Vec3{ 4, 4, 1 });
    roadBounds.Add(start + Vec3{ 4, 4, 1 });
    const auto nativeEnd = nfs::NfswFindRoadEnd(world.LayoutBytes(), start, roadBounds, { 1, 0, 0 });
    if (!nativeEnd || Length(nativeEnd->position - start) > 0.01f ||
        nativeEnd->forward != 1 || nativeEnd->backward != 1) return false;
    const Vec3 extension = nativeEnd->heading * (24.0f / 3.0f);
    const std::vector<nfs::NfswRoadSegmentBuild> nativePull{
        { 0, 1, { nativeEnd->position, nativeEnd->position + extension, nativeEnd->position + extension * 2,
                  nativeEnd->position + extension * 3 }, nativeEnd->forward, nativeEnd->backward,
          nativeEnd->laneWidth, nativeEnd->medianWidth, nativeEnd->nativeProfile }
    };
    const auto originalLayout = world.LayoutBytes();
    std::vector<u8> connected(originalLayout.begin(), originalLayout.end());
    nfs::NfswRoadBuildReport connection;
    std::string graphError;
    if (!nfs::NfswAppendRoadGraph(connected, nativePull, connection, graphError) ||
        connection.stockJoins != 1 || connection.segments != 1) {
        std::printf("NFSW native road pull: %s\n", graphError.c_str());
        return false;
    }
    std::printf("NFSW native road pull: original lane profile and heading, one native AI join\n");
    const auto route = [](size_t a, size_t b, const roads::Curve& curve) {
        return nfs::NfswRoadSegmentBuild{ a, b, { curve.p0, curve.p1, curve.p2, curve.p3 } };
    };
    const Vec3 junction = start + Vec3{ 25, 0, 0 };
    const Vec3 penEnd = junction + Vec3{ 25, 0, 0 };
    const Vec3 sharedHandle{ 7, 9, 0 };
    const auto approach = roads::PenCurve(start, junction, {}, sharedHandle);
    const auto exit = roads::PenCurve(junction, penEnd, sharedHandle, { 8, -5, 0 });
    const auto branch = roads::PenCurve(junction, junction + Vec3{ 0, 25, 0 }, {}, {});
    if (Length(approach.Tangent(1) - exit.Tangent(0)) > 0.01f ||
        exit.At(0.5f).y < junction.y + 4.0f ||
        Length(branch.At(0.5f) - junction - Vec3{ 0, 12.5f, 0 }) > 0.001f) return false;
    const std::vector<nfs::NfswRoadSegmentBuild> routes{
        route(0, 1, approach), route(1, 2, exit), route(1, 3, branch)
    };
    std::puts("NFSW road pen: straight corner and shared Bezier handles verified");

    NfswRoadPlacement road;
    road.section = 0;
    road.position = { 2290.0f, 2160.0f, 550.0f };
    road.model.name = "FRS_ROAD_TEST";
    road.model.hash = TextureNameHash(road.model.name);
    road.model.textureHashes = { z0Loaded->texturePacks.front()->Textures().front().hash };
    for (Vec3 point : { Vec3{ -10,-4,0 }, Vec3{ 10,-4,0 }, Vec3{ 10,4,0 }, Vec3{ -10,4,0 } }) {
        road.model.vertices.push_back({ point, { 0,0,1 }, 0xFFFFFFFF, { point.x / 20, point.y / 8 } });
        road.model.bounds.Add(point);
    }
    road.model.indices = { 0,1,2, 0,2,3 };
    nfs::SolidMaterial material;
    material.indexCount = 6;
    material.bounds = road.model.bounds;
    road.model.materials = { material };

    nfs::NfswCollisionPlacement ground;
    collision::AppendGround({ road.position + Vec3{ -10,-4,0 }, road.position + Vec3{ 10,-4,0 },
                             road.position + Vec3{ 10,4,0 }, road.position + Vec3{ -10,4,0 } },
                            nfs::kSurfaceRoad, ground.polygons);
    if (ground.polygons.empty()) return false;
    std::vector<NfswMeshOverlay> files;
    nfs::NfswRoadBuildReport nav;
    std::wstring error;
    if (!PlanNfswRoadOverlays(world, { road }, routes, { ground }, files, nav, error)) {
        std::printf("NFSW road bake: %ls\\n", error.c_str());
        return false;
    }
    std::printf("NFSW road bake planned %zu files, %zu nodes, %zu segments, %zu native joins\n",
                files.size(), nav.nodes, nav.segments, nav.stockJoins);
    if (nav.segments != 3 || nav.nodes != 3 || nav.stockJoins != 1 || files.size() != 4) return false;

    wchar_t temp[MAX_PATH]{}, scratch[MAX_PATH]{};
    const DWORD length = ::GetTempPathW(MAX_PATH, temp);
    if (!length || length >= MAX_PATH || !::GetTempFileNameW(temp, L"frb", 0, scratch) ||
        !::DeleteFileW(scratch) || !::CreateDirectoryW(scratch, nullptr)) return false;
    const std::wstring folder = scratch;
    bool ok = false;
    {
        nfs::World reopened;
        reopened.SetOverlayFolder(folder);
        const auto verify = [&]() {
            if (!WriteNfswMeshOverlays(files, folder, error) || !reopened.Open(Tracks(), L"L5RA")) return false;
            nfs::NfswNativeRoads graph;
            if (!nfs::NfswReadNativeRoads(reopened.LayoutBytes(), graph) ||
                graph.segments.size() != stock.segments.size() + routes.size() || graph.network == stock.network)
                return false;
            for (size_t i = 0; i < stock.segments.size(); ++i)
                if (std::memcmp(graph.segments[i].data(), stock.segments[i].data(), sizeof(Vec3) * 2)) return false;
            for (size_t i = 0; i < routes.size(); ++i) {
                const auto& leg = graph.segments[stock.segments.size() + i];
                if (Length(leg[0] - routes[i].cubic.front()) > 0.001f ||
                    Length(leg[1] - routes[i].cubic.back()) > 0.001f) return false;
            }

            const auto* a100 = reopened.Table().FindByNumber(loaded.info->number);
            const auto* collision = reopened.Table().FindByNumber(nfs::kNfswCollisionSection);
            const auto* section = a100 ? reopened.LoadSection(*a100) : nullptr;
            if (!section || !collision || !reopened.LoadSection(*collision)) return false;
            const auto* mesh = reopened.FindModel(road.model.hash);
            if (!mesh || mesh->name != road.model.name || mesh->indices != road.model.indices ||
                mesh->textureHashes != road.model.textureHashes || mesh->vertices.size() != road.model.vertices.size()) return false;
            if (!std::any_of(section->scenery.instances.begin(), section->scenery.instances.end(), [&](const auto& instance) {
                return section->scenery.ModelHash(instance) == road.model.hash &&
                       Length(instance.transform.TransformPoint({}) - road.position) < 0.001f;
            })) return false;
            auto alreadyBaked = ground.polygons;
            // Coverage works on the triangles a second generation would make.
            alreadyBaked.clear();
            const Vec3 corners[]{ road.position + Vec3{ -10,-4,0 }, road.position + Vec3{ 10,-4,0 },
                                  road.position + Vec3{ 10,4,0 } };
            alreadyBaked.push_back(collision::MakeGround(corners, 3, nfs::kSurfaceRoad));
            if (collision::RemoveCoveredRoadGround(reopened, alreadyBaked) != 1 || !alreadyBaked.empty()) return false;

            // A placement save must keep the authored road collision.
            const auto z0File = reopened.NfswSectionFileName(*collision);
            const auto initial = std::find_if(files.begin(), files.end(), [&](const auto& f) {
                return f.file == z0File;
            });
            std::vector<u8> savedZ0;
            if (initial == files.end() || !WriteNfswPlacementOverlays(reopened, folder, error) ||
                !ReadFile(folder + L"\\files\\TRACKS\\" + z0File, savedZ0) ||
                initial->bytes != savedZ0) return false;

            // The editor deliberately hides the generated scenery pack in
            // its stock-collision view. Read the Z0 data the game receives.
            nfs::ChunkReader top(reopened.SectionBytes(*collision), 0);
            nfs::Chunk chunk;
            const Vec3 probe{ -road.position.y, road.position.z + 10, road.position.x };
            while (top.Next(chunk)) {
                if (chunk.id != 0x3B801) continue;
                const size_t padding = (size_t)((16 - (chunk.headerOffset + 8) % 16) % 16);
                const ByteSpan wrapper = chunk.raw.sub(padding, chunk.raw.size);
                const ByteSpan pack = wrapper.sub(16, wrapper.read<u32>(0));
                constexpr char label[] = "FRSPlacementCollision";
                const size_t name = 32 + pack.read<u32>(44);
                if (name + sizeof(label) > pack.size ||
                    std::memcmp(pack.data + name, label, sizeof(label) - 1) != 0) continue;
                const size_t ci = 32 + (pack.read<u32>(24) - 1) * 16;
                const size_t instances = ci + pack.read<u32>(ci + 12);
                for (size_t i = 0; i < pack.read<u32>(ci + 8); ++i) {
                    const size_t at = instances + i * 64;
                    const Vec3 origin{ -pack.read<float>(at + 48), -pack.read<float>(at + 52), -pack.read<float>(at + 56) };
                    const size_t ca = 48 + pack.read<u16>(at + 26) * 16;
                    const ByteSpan mesh = pack.sub(ca + pack.read<u32>(ca + 12), pack.read<u32>(ca + 4) >> 8);
                    if (mesh.read<u32>(16 + mesh.read<u16>(2)) != 0x6CA26F9B) continue;
                    for (size_t strip = 0; strip < mesh.read<u16>(0); ++strip) {
                        const size_t sphere = 16 + strip * 16;
                        const size_t vertices = 16 + mesh.read<u16>(sphere + 14);
                        const Vec3 centre = origin + Vec3{ mesh.read<float>(sphere), mesh.read<float>(sphere + 4),
                                                           mesh.read<float>(sphere + 8) };
                        Vec3 p[3];
                        for (size_t v = 0; v < 3; ++v) {
                            const size_t vertex = vertices + v * 8;
                            p[v] = centre + Vec3{ mesh.read<i16>(vertex) / 128.0f, mesh.read<i16>(vertex + 2) / 128.0f,
                                                  mesh.read<i16>(vertex + 4) / 128.0f };
                        }
                        Vec3 normal = Cross(p[1] - p[0], p[2] - p[0]);
                        if (mesh.read<u16>(vertices + 14) & 1) normal = -normal;
                        const float distance = IntersectRayTriangle({ probe, { 0,-1,0 } }, p[0], p[1], p[2]);
                        if (normal.y > 0 && std::fabs(distance - 10) < 0.02f) return true;
                    }
                }
            }
            return false;
        };
        ok = verify();
    }
    if (ok) std::printf("NFSW road bake: native mesh, texture, scenery, driveable ground and %zu route segments verified\n", nav.segments);
    else if (!error.empty()) std::printf("NFSW road bake: %ls\\n", error.c_str());
    for (const auto& file : files)
        ::DeleteFileW((folder + (file.modelsRoot ? L"\\" : L"\\files\\TRACKS\\") + file.file).c_str());
    ::RemoveDirectoryW((folder + L"\\files\\TRACKS").c_str());
    ::RemoveDirectoryW((folder + L"\\files").c_str());
    ::RemoveDirectoryW(folder.c_str());
    return ok;
}

bool CheckWorldMeshImport(nfs::World& world, const nfs::LoadedSection& loaded) {
    const auto& section = *loaded.info;
    const ByteSpan base = world.SectionBytes(section); // cache the original before seeding a saved edit
    const auto scenery = nfs::NfswReadSectionScenery(base);
    if (scenery.instances.size() < 2) return false;

    // Pick an existing A100 mesh with its own native vertices and scenery record.
    const nfs::SolidObject* model = nullptr;
    const nfs::SceneryInstance* placement = nullptr;
    for (const auto& item : scenery.instances) {
        const auto* candidate = world.FindModel(scenery.ModelHash(item));
        if (!candidate || candidate->vertices.size() > 1500 || candidate->indices.size() < 6) continue;
        nfs::NfswMeshUpdate check{ candidate->name, candidate->hash, {}, candidate->indices };
        for (const auto& vertex : candidate->vertices) check.positions.push_back(vertex.position);
        std::vector<u8> copy(base.begin(), base.end());
        std::string error;
        if (!nfs::NfswPatchSolidMeshes(copy, { &check }, error)) continue;
        model = candidate;
        placement = &item;
        break;
    }
    if (!model) return false;
    const auto& other = &scenery.instances.front() == placement ? scenery.instances[1] : scenery.instances.front();
    auto otherTransform = other.transform;
    otherTransform.m[3][2] += 0.5f;
    Aabb otherBounds = other.worldBounds;
    otherBounds.min.z += 0.5f;
    otherBounds.max.z += 0.5f;
    const std::wstring sectionFile = world.NfswSectionFileName(section);
    NfswMeshOverlay saved{ sectionFile, {} };
    std::string patchError;
    if (!nfs::NfswPatchScenery(base, { { other.sceneryNumber, other.record, otherTransform, otherBounds } },
                               saved.bytes, patchError)) return false;

    wchar_t temp[MAX_PATH]{}, folder[MAX_PATH]{};
    const DWORD length = ::GetTempPathW(MAX_PATH, temp);
    if (!length || length >= MAX_PATH || !::GetTempFileNameW(temp, L"frm", 0, folder) ||
        !::DeleteFileW(folder) || !::CreateDirectoryW(folder, nullptr)) return false;
    const std::wstring game = folder;
    const std::wstring models = game + L"\\models";
    const std::wstring obj = game + L"\\edit.obj";
    const std::wstring manifest = game + L"\\edit.json";
    const std::wstring written = models + L"\\files\\TRACKS\\" + sectionFile;
    world.SetOverlayFolder(models);
    std::wstring report;

    const size_t corner = model->indices.front();
    const Mat4 transform = placement->transform;
    const Vec3 origin = Transform(model->bounds, transform).Center();
    const auto emitObj = [&](float lift, bool badFace) {
        FILE* file = _wfopen(obj.c_str(), L"w");
        if (!file) return false;
        std::fprintf(file, "o edit\n");
        for (size_t i = 0; i < model->vertices.size(); ++i) {
            Vec3 local = model->vertices[i].position;
            if (i == corner) local.z += lift;
            const Vec3 point = transform.TransformPoint(local) - origin;
            std::fprintf(file, "v %.4f %.4f %.4f\n", point.x, point.z, -point.y);
        }
        // Blender can regroup faces by material and start at any triangle corner.
        for (size_t i = model->indices.size(); i >= 3; i -= 3)
            std::fprintf(file, "f %u %u %u\n", model->indices[i - 2] + 1u,
                         model->indices[i - 1] + 1u, model->indices[i - 3] + 1u);
        if (badFace) std::fprintf(file, "f 0 2 3\n");
        return std::fclose(file) == 0;
    };
    const auto verify = [&](float lift) {
        nfs::World reopened;
        reopened.SetOverlayFolder(models);
        if (!reopened.Open(Tracks(), L"L5RA")) return false;
        const auto* info = reopened.Table().FindByName(section.name);
        const auto* updated = info ? reopened.LoadSection(*info) : nullptr;
        const auto* actual = reopened.FindModel(model->hash);
        if (!updated || !actual || actual->indices != model->indices ||
            actual->vertices.size() != model->vertices.size() || reopened.SectionBytes(*info).size != base.size)
            return false;
        for (size_t i = 0; i < actual->vertices.size(); ++i) {
            const auto& a = actual->vertices[i];
            const auto& b = model->vertices[i];
            const Vec3 expected = b.position + Vec3{ 0, 0, i == corner ? lift : 0.0f };
            if (Length(a.position - expected) > 0.002f || a.color != b.color ||
                a.uv.x != b.uv.x || a.uv.y != b.uv.y || Length(a.normal - b.normal) > 0.0001f)
                return false;
        }
        const auto preserved = std::find_if(updated->scenery.instances.begin(), updated->scenery.instances.end(),
            [&](const auto& instance) { return instance.record == other.record && instance.sceneryNumber == other.sceneryNumber; });
        if (preserved == updated->scenery.instances.end() ||
            std::fabs(preserved->transform.m[3][2] - otherTransform.m[3][2]) > 0.0001f) return false;
        ImportPlan unchanged;
        return PlanImport(reopened, obj, unchanged) && unchanged.worldMeshOverlays.empty();
    };
    const auto save = [&](float lift) {
        if (!emitObj(lift, false)) return false;
        ImportPlan plan;
        ImportSummary summary;
        if (!PlanImport(world, obj, plan) || plan.worldMeshOverlays.size() != 1 || plan.worldVerticesChanged != 1 ||
            !ApplyImport(plan, game, world.Region(), summary)) {
            std::printf("NFSW Blender mesh: %ls %ls\n", plan.report.c_str(), summary.error.c_str());
            return false;
        }
        return verify(lift);
    };
    FILE* file = _wfopen(manifest.c_str(), L"w");
    if (file) {
        std::fprintf(file, "{\"region\":\"L5RA\", \"origin\":[%.4f,%.4f,%.4f], \"objects\":["
                           "{\"name\":\"edit\",\"model\":\"%s\",\"hash\":\"%08X\",\"vertices\":%zu,\"transform\":[",
                     origin.x, origin.y, origin.z, model->name.c_str(), model->hash, model->vertices.size());
        for (int row = 0; row < 4; ++row)
            for (int col = 0; col < 3; ++col)
                std::fprintf(file, "%s%.9g", row || col ? "," : "", transform.m[row][col]);
        std::fprintf(file, "]}]}\n");
        std::fclose(file);
    }
    ImportPlan bad;
    std::vector<u8> unchangedStock;
    const bool ok = file && WriteNfswMeshOverlays({ saved }, models, report) &&
        save(12.0f) && save(15.0f) && emitObj(16.0f, true) && !PlanImport(world, obj, bad) &&
        bad.worldMeshOverlays.empty() && ReadFile(Tracks() + L"\\" + sectionFile, unchangedStock) &&
        unchangedStock.size() == base.size && std::memcmp(unchangedStock.data(), base.data, base.size) == 0;
    if (ok) std::printf("NFSW Blender mesh %s: repeat OBJ saves, stock unchanged, prior placement, and topology refusal verified\n",
                        model->name.c_str());
    for (const auto& path : { obj, manifest, written }) ::DeleteFileW(path.c_str());
    for (const auto& path : { models + L"\\files\\TRACKS", models + L"\\files", models, game })
        ::RemoveDirectoryW(path.c_str());
    return ok;
}

bool CheckWallCollision(const nfs::World& world, ByteSpan edited, const Mat4& transform) {
    const nfs::Section* z0 = world.Table().FindByNumber(25000);
    if (!z0) return false;
    const ByteSpan shipped = world.SectionBytes(*z0);
    const Vec3 center{ -transform.m[3][1], transform.m[3][2], transform.m[3][0] };
    u16 generated = 0;
    u16 wall = 0;
    bool wallFound = false, indexed = false;
    nfs::ChunkReader top(edited, 0);
    nfs::Chunk chunk;
    while (top.Next(chunk)) {
        if (chunk.id != 0x3B801 && chunk.id != 0x3B802) continue;
        const size_t aligned = (size_t)((chunk.headerOffset + 23) & ~15ull);
        const ByteSpan wrapper = edited.sub(aligned, chunk.raw.size - (aligned - chunk.headerOffset - 8));
        const ByteSpan pack = wrapper.sub(16, wrapper.read<u32>(0));
        if (chunk.id == 0x3B801) {
            // The stock collision articles precede our new 0x3B801 pack.
            if (pack.read<u32>(32) != 0x4E616D65) continue;
            const size_t name = 32 + pack.read<u32>(44);
            constexpr char label[] = "FRSPlacementCollision";
            if (name + sizeof(label) > pack.size ||
                std::memcmp(pack.data + name, label, sizeof(label) - 1) != 0) continue;
            if (chunk.headerOffset > shipped.size ||
                std::memcmp(edited.data, shipped.data, (size_t)chunk.headerOffset) != 0) return false;
            generated = (u16)wrapper.read<u32>(4);
            const u32 count = pack.read<u32>(24);
            if (count < 2 || count > (pack.size - 32) / 16) return false;
            const size_t ci = 32 + (count - 1) * 16;
            if (pack.read<u32>(ci) != 0x63690000 || pack.read<u32>(ci + 8) != 3) return false;
            const size_t instances = ci + pack.read<u32>(ci + 12);
            for (size_t i = 0; i < 3; ++i) {
                const size_t at = instances + i * 64;
                if (std::abs(pack.read<float>(at + 48) + center.x) > 30 ||
                    std::abs(pack.read<float>(at + 56) + center.z) > 30) continue;
                const size_t ca = 48 + pack.read<u16>(at + 0x1A) * 16;
                const size_t mesh = ca + pack.read<u32>(ca + 12);
                if (pack.read<u16>(mesh) || pack.read<u16>(mesh + 4) < 4) return false;
                const Vec3 origin{ -pack.read<float>(at + 48), -pack.read<float>(at + 52),
                                   -pack.read<float>(at + 56) };
                const float extent = pack.read<float>(at + 60) + 2.0f;
                for (const Vec3& axis : { Vec3{ 1, 0, 0 }, Vec3{ -1, 0, 0 },
                                         Vec3{ 0, 0, 1 }, Vec3{ 0, 0, -1 } }) {
                    const Ray ray{ origin - axis * extent, axis };
                    const ByteSpan geometry = pack.sub(mesh, pack.read<u32>(ca + 4) >> 8);
                    if (NativeBoundaryHits(geometry, origin, ray).empty()) return false;
                }
                wall = (u16)i;
                wallFound = true;
            }
        } else {
            const size_t cells = 32 + pack.read<u32>(44);
            const u32 count = pack.read<u32>(40);
            size_t at = cells;
            for (u32 i = 0; i < count; ++i) {
                if (at + 20 > pack.size) return false;
                const u8 refs = pack.read<u8>(at + 8);
                if (pack.read<u16>(at + 4) == 6959) {  // F115 wall's original broad-phase cell
                    for (size_t j = 0; j < refs; ++j)
                        if (pack.read<u32>(at + 20 + j * 4) == ((u32)generated << 16 | wall))
                            indexed = true;
                }
                at += 20 + 4 * (refs + pack.read<u8>(at + 9) + pack.read<u8>(at + 10) +
                                pack.read<u8>(at + 11));
            }
        }
    }
    if (!generated || !wallFound || !indexed) return false;

    std::vector<u8> removed;
    std::string error;
    if (!nfs::NfswBuildPlacementCollision(world.LayoutBytes(), edited, {}, {}, removed, error)) {
        std::printf("NFSW placement collision removal: %s\n", error.c_str());
        return false;
    }
    nfs::ChunkReader restored({ removed.data(), removed.size() }, 0);
    while (restored.Next(chunk)) {
        if (chunk.id != 0x3B801) continue;
        if (chunk.data.read<u32>(4) == generated) return false;
    }
    return true;
}

bool CheckPlacementStreaming(nfs::World& world, const std::wstring& folder) {
    world.LoadAllSections();
    nfs::LoadedSection* source = nullptr;
    nfs::LoadedSection* target = nullptr;
    for (const auto& section : world.Loaded()) {
        if (!section->info) continue;
        if (section->info->number == 5115) source = section.get();
        if (section->info->number == 4007) target = section.get();
    }
    if (!source || !target || target->sceneryNumbers.empty() ||
        std::none_of(source->collision.begin(), source->collision.end(),
            [](const nfs::CollisionPack& pack) { return pack.sectionNumber == 5115; })) return false;

    constexpr u32 wallHash = 0xD84F472E;
    constexpr u32 trainHash = 0xFA75A95F;
    const nfs::SolidObject* wall = world.FindModel(wallHash);
    const nfs::SolidObject* train = world.FindModel(trainHash);
    const auto oldWall = std::find_if(source->scenery.instances.begin(), source->scenery.instances.end(),
        [&](const nfs::SceneryInstance& item) { return source->scenery.ModelHash(item) == wallHash; });
    const auto* stockList = world.Visibility().ListOf(4007);
    if (!wall || !train || oldWall == source->scenery.instances.end() || !stockList) return false;
    const auto listed = [](const std::vector<u32>& list, u32 number) {
        return std::find(list.begin(), list.end(), number) != list.end();
    };
    const size_t trainHome = world.ModelHome(trainHash);
    if (trainHome >= world.Loaded().size() || !world.Loaded()[trainHome]->info) return false;
    const u32 trainSection = world.Loaded()[trainHome]->info->number;
    if (listed(*stockList, trainSection)) return false;

    nfs::PlacedInstance moved;
    moved.modelHash = wallHash;
    moved.modelName = wall->name;
    moved.modelBounds = wall->bounds;
    moved.originalSection = 5115;
    moved.originalSceneryNumber = oldWall->sceneryNumber;
    moved.originalRecord = oldWall->record;
    moved.transform = oldWall->transform;
    moved.transform.m[3][0] = target->info->position.x;
    moved.transform.m[3][1] = target->info->position.y;
    moved.transform.m[3][2] = target->info->position.z;
    source->removedNatives.push_back({ moved.originalSceneryNumber, moved.originalRecord });
    target->placements.push_back({ 4007, moved });
    nfs::PlacedInstance added;
    added.modelHash = trainHash;
    added.modelName = train->name;
    added.modelBounds = train->bounds;
    added.transform = moved.transform;
    added.transform.m[3][0] += 10;
    target->placements.push_back({ 4007, added });
    nfs::PlacedInstance wallCopy;
    wallCopy.modelHash = wallHash;
    wallCopy.modelName = wall->name;
    wallCopy.modelBounds = wall->bounds;
    wallCopy.transform = oldWall->transform;
    wallCopy.transform.m[3][0] = 8306.159f;
    wallCopy.transform.m[3][1] = -2499.760f;
    wallCopy.transform.m[3][2] = 105.028f;
    source->placements.push_back({ 5115, wallCopy });
    target->removedNatives.push_back({ 4007, 116 });
    target->removedNatives.push_back({ 4007, 117 });
    // Shipped entry 26467 names record 118. Removing it too exercises the
    // neutralized branch of the group table remap with real data.
    target->removedNatives.push_back({ 4007, 118 });

    const std::wstring tracks = folder + L"\\files\\TRACKS\\";
    const std::wstring layoutPath = tracks + L"L5RA.BUN";
    const std::wstring sourcePath = tracks + world.NfswSectionFileName(*source->info);
    const std::wstring targetPath = tracks + world.NfswSectionFileName(*target->info);
    const std::wstring collisionPath = tracks + L"STREAML5RA_25000.BUN";
    std::vector<u8> layout, resaved, from, to, collision;
    std::wstring report;
    if (!WriteNfswPlacementOverlays(world, folder, report) || !ReadFile(layoutPath, layout) ||
        !ReadFile(sourcePath, from) || !ReadFile(targetPath, to) || !ReadFile(collisionPath, collision) ||
        !WriteNfswPlacementOverlays(world, folder, report) || !ReadFile(layoutPath, resaved)) {
        std::printf("NFSW placement streaming write failed: %ls\n", report.c_str());
        return false;
    }
    size_t z0Entry = 0;
    if (!CheckZ0StreamParts(world, { layout.data(), layout.size() },
                           { collision.data(), collision.size() }, z0Entry)) {
        std::printf("NFSW placement collision: Z0 streamed texture slices are invalid\n");
        return false;
    }
    // Reproduce the broken collision overlay's table: the old writer used
    // the whole file size for part 0 and left the other endpoints at stock.
    std::vector<u8> legacy = layout;
    const size_t stockEntry = (size_t)world.Table().FindByNumber(25000)->tableEntryOffset;
    const u32 wholeSize = (u32)collision.size();
    for (size_t word = 0; word < 12; word += 4)
        std::memcpy(legacy.data() + z0Entry + 0x60 + word, &wholeSize, sizeof(wholeSize));
    std::memcpy(legacy.data() + z0Entry + 0x6C, world.LayoutBytes().data + stockEntry + 0x6C, 48);
    if (!WriteFile(layoutPath, legacy)) return false;
    {
        nfs::World reopened;
        reopened.SetOverlayFolder(folder);
        if (!reopened.Open(Tracks(), L"L5RA")) return false;
        for (u32 number : { 4007u, 5115u, trainSection, 25000u }) {
            const nfs::Section* section = reopened.Table().FindByNumber(number);
            if (!section || !reopened.LoadSection(*section)) return false;
        }
        std::vector<u8> collisionAfterReopen;
        std::vector<u8> repairedLayout;
        if (!WriteNfswPlacementOverlays(reopened, folder, report) ||
            !ReadFile(collisionPath, collisionAfterReopen) || collisionAfterReopen != collision ||
            !ReadFile(layoutPath, repairedLayout) || repairedLayout != layout) {
            std::printf("NFSW placement collision: old Z0 layout repair failed: %ls\n",
                        report.c_str());
            return false;
        }
        nfs::LoadedSection* z0 = nullptr;
        size_t zoneIndex = 0;
        for (size_t i = 0; i < reopened.Loaded().size(); ++i) {
            const auto& section = reopened.Loaded()[i];
            if (section->info && section->info->number == 25000) {
                z0 = section.get();
                zoneIndex = i;
                break;
            }
        }
        if (!z0 || z0->collision.size() < 2 || !z0->collision[0].polygons.empty()) return false;
        nfs::CollisionPolygon wall;
        wall.quad = true;
        wall.attributes[2] = nfs::kSurfaceSolidWall;
        wall.attributes[3] = 0x12;
        wall.SyncFromAttributes();
        // A surface hash the shipped Z0 packs use; the article must keep it.
        wall.nfswSurface = 0x35187254;
        wall.corners[0] = { 8305, -2498, 105 };
        wall.corners[1] = { 8305, -2498, 108 };
        wall.corners[2] = { 8305, -2496, 108 };
        wall.corners[3] = { 8305, -2496, 105 };
        std::vector<nfs::CollisionPolygon> authored{ wall };
        if (!collision::AppendGround({ { 5458, 429, 48 }, { 5480, 429, 48 },
                                       { 5480, 435, 48 }, { 5458, 435, 48 } },
                                     nfs::kSurfaceRoad, authored) || authored.size() != 2) return false;
        collision::Editor groundEditor;
        groundEditor.SetWorld(&reopened);
        groundEditor.SetLogging(false);
        if (groundEditor.Add(authored, L"Draw wall and ground").size() != authored.size()) return false;
        std::vector<nfs::CollisionPolygon> decoded;
        if (!WriteNfswPlacementOverlays(reopened, folder, report) ||
            !ReadFile(collisionPath, collisionAfterReopen) ||
            !nfs::NfswReadCollisionWalls({ collisionAfterReopen.data(), collisionAfterReopen.size() }, decoded) ||
            decoded.size() != 2 || !decoded[0].Vertical() || !decoded[0].quad || decoded[1].Vertical() ||
            decoded[0].nfswSurface != 0x35187254 || decoded[1].nfswSurface != nfs::kRoadSurface ||
            std::memcmp(decoded[0].corners, wall.corners, sizeof(wall.corners)) != 0 ||
            std::memcmp(decoded[1].corners, authored[1].corners, sizeof(wall.corners)) != 0) {
            std::printf("NFSW placement collision: drawn wall and ground did not round-trip: %ls\n", report.c_str());
            return false;
        }
        if (!groundEditor.Undo()) return false;
        if (!WriteNfswPlacementOverlays(reopened, folder, report) ||
            !ReadFile(collisionPath, collisionAfterReopen) ||
            !nfs::NfswReadCollisionWalls({ collisionAfterReopen.data(), collisionAfterReopen.size() }, decoded) ||
            !decoded.empty()) return false;

        const auto native = std::find_if(z0->collision.begin() + 1, z0->collision.end(),
            [](const nfs::CollisionPack& pack) { return pack.sectionNumber == 5115; });
        if (native == z0->collision.end() || native->polygons.empty()) return false;
        const auto findFace = [&](const Vec3& firstCorner, bool quad) {
            return std::find_if(native->polygons.begin(), native->polygons.end(),
                [&](const nfs::CollisionPolygon& face) {
                    return face.quad == quad && Length(face.corners[0] - firstCorner) < 0.002f;
                });
        };
        // Independent expectations from the stock F115 CI/CA records.
        const auto strip = findFace({ 8290.25f, -2521.125f, 105.02734375f }, false);
        const auto boundary = findFace({ 8301.38f, -2496.49f, 104.58f }, true);
        if (strip == native->polygons.end() || strip->Vertical() ||
            boundary == native->polygons.end() || !boundary->Vertical() ||
            Length(boundary->corners[2] - Vec3{ 8299.79f, -2499.38f, 112.42f }) >= 0.002f) {
            std::printf("NFSW native F115 collision: centered strip or boundary was decoded incorrectly\n");
            return false;
        }
        const u32 packIndex = (u32)(native - z0->collision.begin());
        const std::vector<collision::PolygonRef> refs{
            { (u32)zoneIndex, packIndex, (u32)(strip - native->polygons.begin()) },
            { (u32)zoneIndex, packIndex, (u32)(boundary - native->polygons.begin()) }
        };
        const nfs::CollisionPolygon originals[] = { *strip, *boundary };
        collision::Editor editor;
        editor.SetWorld(&reopened);
        editor.SetLogging(false);
        if (!editor.BeginMove(refs)) return false;
        const Vec3 shift{ 13, 0, 0 };
        editor.UpdateMove(shift);
        editor.EndMove(true);
        if (!WriteNfswPlacementOverlays(reopened, folder, report) ||
            !ReadFile(collisionPath, collisionAfterReopen)) {
            std::printf("NFSW native collision move failed: %ls\n", report.c_str());
            return false;
        }
        const ByteSpan movedSection{ collisionAfterReopen.data(), collisionAfterReopen.size() };
        std::vector<nfs::CollisionPack> afterMove;
        if (!nfs::NfswReadNativeCollision(movedSection, afterMove) ||
            !nfs::NfswReadCollisionWalls(movedSection, decoded)) return false;
        const auto sameFace = [](const nfs::CollisionPolygon& face,
                                  const nfs::CollisionPolygon& original, const Vec3& offset) {
            if (face.quad != original.quad || face.Vertical() != original.Vertical()) return false;
            for (int i = 0; i < 4; ++i)
                if (Length(face.corners[i] - original.corners[i] - offset) > 0.02f) return false;
            return true;
        };
        const auto changed = std::find_if(afterMove.begin(), afterMove.end(),
            [](const nfs::CollisionPack& pack) { return pack.sectionNumber == 5115; });
        if (changed == afterMove.end()) return false;
        for (const auto& original : originals) {
            if (std::any_of(changed->polygons.begin(), changed->polygons.end(),
                    [&](const nfs::CollisionPolygon& face) { return sameFace(face, original, {}); }) ||
                !std::any_of(decoded.begin(), decoded.end(),
                    [&](const nfs::CollisionPolygon& face) { return sameFace(face, original, shift); }))
                return false;
        }
        if (!editor.Undo() || !editor.Remove(refs) ||
            !WriteNfswPlacementOverlays(reopened, folder, report) ||
            !ReadFile(collisionPath, collisionAfterReopen) ||
            !nfs::NfswReadCollisionWalls({ collisionAfterReopen.data(), collisionAfterReopen.size() }, decoded) ||
            !decoded.empty()) return false;
        if (!editor.Undo() || !WriteNfswPlacementOverlays(reopened, folder, report) ||
            !ReadFile(collisionPath, collisionAfterReopen) || collisionAfterReopen != collision) return false;
        std::printf("NFSW native F115 collision strip and boundary: coordinates, move, delete and undo verified\n");
    }
    ::DeleteFileW(collisionPath.c_str());
    ::DeleteFileW(layoutPath.c_str());
    ::DeleteFileW(sourcePath.c_str());
    ::DeleteFileW(targetPath.c_str());
    if (layout != resaved) {
        std::printf("NFSW placement streaming: repeated save changed the layout\n");
        return false;
    }
    if (!CheckWallCollision(world, { collision.data(), collision.size() }, wallCopy.transform)) {
        std::printf("NFSW placement collision: F115 wall or collision grid check failed\n");
        return false;
    }
    std::printf("NFSW placement collision: F115 copy blocks four boundary rays, cell 6959 verified, "
                "generated collision removable\n");
    std::printf("NFSW Z0: all five stream parts verified, old layout repaired, editable wall round-tripped\n");

    const auto groupEntries = [](ByteSpan bytes) {
        nfs::ChunkReader reader(bytes, 0);
        nfs::Chunk chunk;
        while (reader.Next(chunk))
            if (chunk.id == 0x34108) return chunk.raw;
        return ByteSpan{};
    };
    const ByteSpan oldEntries = groupEntries(world.LayoutBytes());
    const ByteSpan newEntries = groupEntries({ layout.data(), layout.size() });
    if (oldEntries.empty() || oldEntries.size != newEntries.size) return false;
    // E7's group table used to keep pointing at 118 after deleting earlier
    // objects; that record would then be a race barrier. Every entry naming a
    // removed record must be neutralized, the rest follow the instance down.
    const std::set<u32> removedE7 = { 116, 117, 118 };
    for (size_t at = 0; at + 6 <= oldEntries.size; at += 6) {
        if (oldEntries.read<u16>(at) != 4007) continue;
        const u16 record = oldEntries.read<u16>(at + 2);
        const u16 flags = oldEntries.read<u16>(at + 4);
        u16 shifted = 0;
        bool removed = false;
        for (u32 gone : removedE7) {
            if (record > gone) ++shifted;
            else if (record == gone) removed = true;
        }
        const u16 expect[3] = { removed ? u16(0) : u16(4007),
                                removed ? u16(0) : u16(record - shifted),
                                removed ? u16(flags | 0x10) : flags };
        const u16 written[3] = { newEntries.read<u16>(at), newEntries.read<u16>(at + 2),
                                 newEntries.read<u16>(at + 4) };
        if (std::memcmp(expect, written, sizeof(expect)) != 0) {
            std::printf("NFSW placement streaming: E7 group entry %zu did not follow the instance\n",
                        at / 6);
            return false;
        }
    }

    nfs::Preculler visibility;
    if (!visibility.ParseNfsw({ layout.data(), layout.size() })) return false;
    const auto* list = visibility.ListOf(4007);
    if (!list || !listed(*list, 5115) || !listed(*list, trainSection)) {
        std::printf("NFSW placement streaming: E7 is missing a model section (%d F115, %d W955)\n",
                    list && listed(*list, 5115), list && listed(*list, trainSection));
        return false;
    }
    std::string error;
    if (!VectorRangesCover({ from.data(), from.size() }, 5115, error) ||
        !VectorRangesCover({ to.data(), to.size() }, 4007, error)) {
        std::printf("NFSW placement streaming: %s\n", error.c_str());
        return false;
    }

    nfs::ChunkReader reader({ to.data(), to.size() }, 0);
    nfs::Chunk chunk;
    nfs::Scenery after;
    while (reader.Next(chunk))
        if (chunk.id == nfs::kChunkTrackScenery) nfs::NfswParseScenery(chunk, after);
    // The saved section mirrors the editor state: loaded records minus the
    // removals, plus the placements appended for this section.
    size_t expectedInstances = target->scenery.instances.size();
    for (const auto& [goneChunk, goneRecord] : target->removedNatives)
        if (goneChunk == 4007) --expectedInstances;
    for (const auto& [placedChunk, placed] : target->placements)
        if (placedChunk == 4007) ++expectedInstances;
    if (after.instances.size() != expectedInstances ||
        after.ModelHash(after.instances.back()) != trainHash) return false;
    const auto& raw = after.infos[after.instances.back().infoIndex].raw;
    for (size_t at = 0x18; at < 0x28; at += 4) {
        const u32 hash = ByteSpan{ raw.data(), raw.size() }.read<u32>(at);
        if (!hash) continue;
        const nfs::SolidObject* lod = world.FindModel(hash);
        if (!lod) {
            std::printf("NFSW placement streaming: shipped LOD %08X is unresolved\n", hash);
            return false;
        }
        bool foundModel = false;
        for (const auto& section : world.Loaded())
            if (section->info && listed(*list, section->info->number) && section->HoldsModel(hash))
                foundModel = true;
        if (!foundModel) return false;
        for (u32 texture : lod->textureHashes) {
            if (!texture) continue;
            if (!world.FindTexture(texture) && world.FindDisplayTexture(texture)) continue;
            bool foundTexture = false;
            for (const auto& section : world.Loaded()) {
                if (!section->info) continue;
                const u32 number = section->info->number;
                if ((number < 25000 || number > 25002) && !listed(*list, number)) continue;
                for (const auto& pack : section->texturePacks)
                    if (pack->Find(texture)) foundTexture = true;
            }
            if (!foundTexture) {
                std::printf("NFSW placement streaming: texture %08X unavailable in E7\n", texture);
                return false;
            }
        }
    }
    std::printf("NFSW placement streaming: F115 -> E7 and train in E7, "
                "zone assets, cull ranges and group indices verified, resave identical\n");
    return true;
}

std::string Lower(const std::string& text) {
    std::string lower;
    lower.reserve(text.size());
    for (char c : text) lower.push_back((char)std::tolower((unsigned char)c));
    return lower;
}

// Every placement of every model whose info name or resolved model name
// contains `name`, with the section, chunk, record, flags, bounds, holiday
// group and zone-list context: enough to tell a shipped stack of props from
// a layer the game never draws together.
void FindInstances(const nfs::World& world, const std::string& name) {
    const std::string needle = Lower(name);
    const auto matches = [&](const std::string& text) {
        return Lower(text).find(needle) != std::string::npos;
    };
    const auto listed = [](const std::vector<u32>& list, u32 number) {
        return std::find(list.begin(), list.end(), number) != list.end();
    };

    for (const auto& section : world.Loaded()) {
        if (!section->info) continue;
        const nfs::Scenery& scenery = section->scenery;
        for (size_t i = 0; i < scenery.infos.size(); ++i) {
            const nfs::SceneryInfo& info = scenery.infos[i];
            const nfs::SolidObject* model = world.FindModel(info.solidHash);
            if (!matches(info.name) && !(model && matches(model->name))) continue;
            std::printf("model %s (info '%s', hash %08X, %u tris",
                        model ? model->name.c_str() : "?", info.name.c_str(), info.solidHash,
                        model ? model->TriangleCount() : 0);
            if (model && model->bounds.Valid())
                std::printf(", bounds %.1f x %.1f x %.1f",
                            model->bounds.max.x - model->bounds.min.x,
                            model->bounds.max.y - model->bounds.min.y,
                            model->bounds.max.z - model->bounds.min.z);
            const size_t home = world.ModelHome(info.solidHash);
            if (home < world.Loaded().size() && world.Loaded()[home]->info)
                std::printf(", home %s%s", world.Loaded()[home]->info->name.c_str(),
                            world.IsLibrary(*world.Loaded()[home]) ? " (library)" : "");
            else
                std::printf(", home library?");
            std::printf(")\n");
            for (size_t k = 0; k < scenery.instances.size(); ++k) {
                const nfs::SceneryInstance& instance = scenery.instances[k];
                if (instance.infoIndex != i) continue;
                std::vector<u16> entries;
                for (const nfs::SceneryOverride& o : scenery.overrides)
                    if (o.instance == k) entries.push_back(o.entry);

                const Vec3 at{ instance.transform.m[3][0], instance.transform.m[3][1],
                               instance.transform.m[3][2] };
                std::vector<u32> list;
                const bool claimed = world.Visibility().VisibleAt(at, list);
                std::printf("  %s(%u) chunk %u record %u flags %04X at %.1f %.1f %.1f"
                            " bounds %.1f..%.1f %.1f..%.1f %.1f..%.1f",
                            section->info->name.c_str(), section->info->number,
                            instance.sceneryNumber, instance.record, instance.flags,
                            at.x, at.y, at.z,
                            instance.worldBounds.min.x, instance.worldBounds.max.x,
                            instance.worldBounds.min.y, instance.worldBounds.max.y,
                            instance.worldBounds.min.z, instance.worldBounds.max.z);
                if (world.HolidayHides(section->info->number, (int)k)) std::printf(" holiday-hidden");
                if (!entries.empty()) {
                    std::printf(" group-entries");
                    for (u16 entry : entries) std::printf(" %u", entry);
                }
                std::printf(" | zone %s", claimed ? "yes" : "no");
                if (claimed) std::printf(", section in list %s",
                                         listed(list, section->info->number) ? "yes" : "no");
                std::printf("\n");
            }
        }
    }
}

// Pairs of different models whose instance bounds overlap by more than a
// tenth of the smaller box, aggregated over the whole map so one report shows
// every family of the problem instead of one spot at a time. Instances the
// current scenery-group filter hides are skipped: what is left is what a
// player would actually see drawn on top of each other.
void OverlappingInstances(const nfs::World& world) {
    struct Item {
        const nfs::LoadedSection* section;
        size_t instance;
        const nfs::SolidObject* model;
    };
    constexpr float kCell = 64.0f;
    std::unordered_map<u64, std::vector<Item>> grid;
    const auto cellKey = [](int cx, int cy) {
        return ((u64)(u32)cx << 32) | (u32)(u32)cy;
    };

    for (const auto& section : world.Loaded()) {
        if (!section->info) continue;
        const nfs::Scenery& scenery = section->scenery;
        for (size_t k = 0; k < scenery.instances.size(); ++k) {
            if (world.HolidayHides(section->info->number, (int)k)) continue;   // group-filtered out
            const nfs::SceneryInstance& instance = scenery.instances[k];
            const nfs::SolidObject* model = world.FindModel(scenery.ModelHash(instance));
            if (!model || model->IsTerrain() || model->IsBackdrop() || model->IsShadow())
                continue;
            if (!instance.worldBounds.Valid()) continue;
            const Aabb& box = instance.worldBounds;
            for (int cx = (int)std::floor(box.min.x / kCell); cx <= (int)std::floor(box.max.x / kCell); ++cx)
                for (int cy = (int)std::floor(box.min.y / kCell); cy <= (int)std::floor(box.max.y / kCell); ++cy)
                    grid[cellKey(cx, cy)].push_back({ section.get(), k, model });
        }
    }

    struct Overlap {
        size_t count = 0;
        std::string example;
    };
    std::map<std::pair<u32, u32>, Overlap> betweenModels;
    // A prop can sit in several cells; the same pair must be counted once, so
    // placements are keyed by their underlying (section, instance), not by the
    // per-cell copies.
    std::map<std::pair<u64, u64>, char> seen;
    const auto placementKey = [](const Item& item) {
        return (u64)(uintptr_t)item.section * 1000003ull + item.instance;
    };
    size_t pairs = 0;
    for (const auto& [key, items] : grid) {
        for (size_t a = 0; a < items.size(); ++a) {
            for (size_t b = a + 1; b < items.size(); ++b) {
                const Item& one = items[a];
                const Item& two = items[b];
                if (one.model->hash == two.model->hash) continue;
                const u64 keyA = placementKey(one), keyB = placementKey(two);
                if (!seen.try_emplace({ std::min(keyA, keyB), std::max(keyA, keyB) }, 1).second)
                    continue;
                const Aabb& x = one.section->scenery.instances[one.instance].worldBounds;
                const Aabb& y = two.section->scenery.instances[two.instance].worldBounds;
                const float ix = std::min(x.max.x, y.max.x) - std::max(x.min.x, y.min.x);
                const float iy = std::min(x.max.y, y.max.y) - std::max(x.min.y, y.min.y);
                const float iz = std::min(x.max.z, y.max.z) - std::max(x.min.z, y.min.z);
                if (ix <= 0.0f || iy <= 0.0f || iz <= 0.0f) continue;
                const float inter = ix * iy * iz;
                const float volX = (x.max.x - x.min.x) * (x.max.y - x.min.y) * (x.max.z - x.min.z);
                const float volY = (y.max.x - y.min.x) * (y.max.y - y.min.y) * (y.max.z - y.min.z);
                const float smaller = std::min(volX, volY);
                if (smaller <= 0.0f || inter < smaller * 0.1f) continue;

                ++pairs;
                Overlap& overlap = betweenModels[{ std::min(one.model->hash, two.model->hash),
                                                   std::max(one.model->hash, two.model->hash) }];
                ++overlap.count;
                if (overlap.example.empty()) {
                    const nfs::SceneryInstance& i1 = one.section->scenery.instances[one.instance];
                    const nfs::SceneryInstance& i2 = two.section->scenery.instances[two.instance];
                    char text[512];
                    std::snprintf(text, sizeof(text),
                                  "%s/%s(%u) x %s/%s(%u)%s at %.0f %.0f / %.0f %.0f",
                                  one.section->info->name.c_str(), one.model->name.c_str(),
                                  i1.flags, two.section->info->name.c_str(), two.model->name.c_str(),
                                  i2.flags,
                                  one.section->info->number + 400 == two.section->info->number ||
                                          two.section->info->number + 400 == one.section->info->number
                                      ? " [+400 pair]"
                                      : "",
                                  i1.transform.m[3][0], i1.transform.m[3][1],
                                  i2.transform.m[3][0], i2.transform.m[3][1]);
                    overlap.example = text;
                }
            }
        }
    }

    std::printf("overlapping different-model instance pairs: %zu\n", pairs);
    std::vector<std::pair<size_t, const std::pair<u32, u32>*>> order;
    for (const auto& key : betweenModels) order.push_back({ key.second.count, &key.first });
    std::sort(order.begin(), order.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });
    for (size_t i = 0; i < order.size() && i < 40; ++i) {
        const Overlap& overlap = betweenModels[*order[i].second];
        std::printf("  %4zu x %08X/%08X: %s\n", overlap.count,
                    order[i].second->first, order[i].second->second, overlap.example.c_str());
    }
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    {
        const auto regions = nfs::World::AvailableRegions(Tracks());
        std::printf("regions:");
        for (const std::wstring& region : regions) std::printf(" %ls", region.c_str());
        std::printf("\n");
        if (regions.empty()) return 13;
    }

    std::vector<unsigned char> layout;
    if (!ReadFile(Tracks() + L"/L5RA.BUN", layout)) {
        std::printf("cannot read L5RA.BUN\n");
        return 1;
    }

    nfs::SectionTable table;
    if (!table.ParseNfsw({ layout.data(), layout.size() })) {
        std::printf("section table failed\n");
        return 2;
    }
    std::printf("sections: %zu\n", table.Sections().size());
    const nfs::Section* a100 = table.FindByName("A100");
    if (!a100) {
        std::printf("A100 missing\n");
        return 3;
    }
    std::printf("A100: number %u, uid %u, fileSize %u, position %.1f %.1f %.1f\n",
                a100->number, a100->uid, a100->fileSize, a100->position.x, a100->position.y,
                a100->position.z);


    std::vector<unsigned char> sectionBytes;
    if (!ReadFile(Tracks() + L"/STREAML5RA_100.BUN", sectionBytes)) {
        std::printf("cannot read A100 section file\n");
        return 4;
    }

    size_t solidLists = 0, objects = 0, triangles = 0;
    nfs::Scenery scenery;
    size_t sceneryChunks = 0;
    u32 sceneryNumber = 0;
    int decodedPixels = 0;
    nfs::ChunkReader top({ sectionBytes.data(), sectionBytes.size() }, 0);
    nfs::Chunk chunk;
    while (top.Next(chunk)) {
        if (chunk.id == nfs::kChunkSolidListPack) {
            nfs::SolidList list;
            if (nfs::NfswParseSolidList(chunk, list)) {
                ++solidLists;
                objects += list.objects.size();
                for (const nfs::SolidObject& object : list.objects) triangles += object.TriangleCount();
            }
        } else if (chunk.id == nfs::kChunkTrackScenery) {
            const ByteSpan whole{ chunk.raw.data - 8, chunk.raw.size + 8 };
            nfs::NfswSceneryChunkInfo info;
            if (nfs::NfswReadSceneryChunk(whole, 0, info)) {
                std::printf("scenery chunk #%u: %zu infos, %zu instances\n",
                            info.number, info.infoCount, info.instanceCount);
                sceneryNumber = info.number;
                ++sceneryChunks;
            }
            nfs::NfswParseScenery(chunk, scenery);
        }
    }
    std::printf("A100: %zu solid lists, %zu objects, %zu triangles, %zu scenery infos, %zu instances\n",
                solidLists, objects, triangles, scenery.infos.size(), scenery.instances.size());
    if (!solidLists || !objects || !triangles || !sceneryChunks) return 5;

    // One library file, to prove the object hashes scenery infos name resolve.
    std::vector<unsigned char> library;
    if (ReadFile(Tracks() + L"/STREAML5RA_0x35E3129F.BUN", library)) {
        size_t found = 0;
        nfs::ChunkReader lib({ library.data(), library.size() }, 0);
        nfs::Chunk c;
        while (lib.Next(c)) {
            if (c.id != nfs::kChunkSolidListPack) continue;
            nfs::SolidList list;
            if (!nfs::NfswParseSolidList(c, list)) continue;
            for (const nfs::SolidObject& object : list.objects)
                if (object.hash == 0xB29C1AEBu) {
                    ++found;
                    std::printf("library model: %s, %u triangles\n", object.name.c_str(),
                                object.TriangleCount());
                }
        }
        if (!found) {
            std::printf("library model 0xB29C1AEB missing\n");
            return 6;
        }
    }

    // A normal texture pack: parse, then decode one BC texture on the CPU.
    std::vector<unsigned char> tpkBytes;
    if (ReadFile(Tracks() + L"/STREAML5RA_0x01415B60.BUN", tpkBytes)) {
        nfs::ChunkReader tpk({ tpkBytes.data(), tpkBytes.size() }, 0);
        nfs::Chunk c;
        while (tpk.Next(c)) {
            if (c.id != nfs::kChunkTexturePack) continue;
            nfs::TexturePack pack;
            if (!pack.ParseNfsw(c)) continue;
            std::printf("texture pack: %zu textures\n", pack.Textures().size());
            for (const nfs::Texture& texture : pack.Textures()) {
                std::printf("  %08X %s %ux%u %s mips %u\n", texture.hash, texture.name.c_str(),
                            texture.width, texture.height, nfs::ToString(texture.format),
                            texture.mipCount);
                std::vector<u32> pixels;
                u32 width = 0, height = 0;
                if (nfs::DecodeTexture(texture, 0, pixels, width, height)) ++decodedPixels;
            }
            break;
        }
    }
    if (!decodedPixels) {
        std::printf("no texture decoded\n");
        return 7;
    }
    std::printf("decoded textures: %d\n", decodedPixels);
    // Exercise the World integration as well: open the region, load A100 and
    // let the shared section-file cache feed the parsers.
    {
        nfs::World world;
        if (!world.Open(Tracks(), L"L5RA")) {
            std::printf("World::Open failed\n");
            return 8;
        }
        const nfs::Section* section = world.Table().FindByName("A100");
        const nfs::LoadedSection* loaded = section ? world.LoadSection(*section) : nullptr;
        if (!loaded) {
            std::printf("World::LoadSection A100 failed\n");
            return 9;
        }
        std::printf("World A100: %zu objects, %zu triangles, %zu scenery instances\n",
                    loaded->ObjectCount(), loaded->TriangleCount(),
                    loaded->scenery.instances.size());
        if (loaded->ObjectCount() == 0 || loaded->TriangleCount() == 0) return 10;

        // The game's own zone lists are what keep the preplaced/distant layers
        // off the detailed roads. Check the three areas that exposed the bug.
        {
            const nfs::Preculler& visibility = world.Visibility();
            const auto listed = [](const std::vector<u32>& list, u32 number) {
                return std::find(list.begin(), list.end(), number) != list.end();
            };
            std::vector<u32> c439, e435, a100;
            const bool c439ok = visibility.VisibleAt({ 3876.7f, 1251.1f, 200.0f }, c439);
            const bool e435ok = visibility.VisibleAt({ 6048.7f, 1570.2f, 400.0f }, e435);
            const bool a100ok = visibility.VisibleAt({ 2273.2f, 2146.8f, 536.0f }, a100);
            const bool ok = c439ok && e435ok && a100ok &&
                            listed(c439, 2439) && !listed(c439, 833) &&
                            listed(e435, 4035) && listed(e435, 4435) &&
                            listed(a100, 100) && listed(a100, 500) && !listed(a100, 833);
            std::printf("NFSW visibility: %zu zones; C439 list %zu (C439 %s, A833 %s); "
                        "E435 list %zu (E35 %s, E435 %s); A100 list %zu (A100 %s, A500 %s, A833 %s)\n",
                        visibility.ZoneCount(), c439.size(),
                        listed(c439, 2439) ? "yes" : "no", listed(c439, 833) ? "yes" : "no",
                        e435.size(), listed(e435, 4035) ? "yes" : "no",
                        listed(e435, 4435) ? "yes" : "no", a100.size(),
                        listed(a100, 100) ? "yes" : "no", listed(a100, 500) ? "yes" : "no",
                        listed(a100, 833) ? "yes" : "no");
            if (!ok) return 44;
        }

        const wchar_t* mode = argc > 1 ? argv[1] : L"";
        std::string findName;
        if (std::wcscmp(mode, L"--find") == 0 && argc > 2) {
            for (const wchar_t* p = argv[2]; *p; ++p) findName.push_back((char)*p);
        }
        const bool wantOverlaps = std::wcscmp(mode, L"--overlaps") == 0;
        const bool wantZone = std::wcscmp(mode, L"--zone") == 0 && argc > 3;
        const bool wantStream = std::wcscmp(mode, L"--stream") == 0 && argc > 3;
        const bool wantUnresolved = std::wcscmp(mode, L"--unresolved") == 0;
        const bool wantPatch = std::wcscmp(mode, L"--patch") == 0;
        const bool wantOverlay = std::wcscmp(mode, L"--overlay") == 0;
        const bool wantRebuild = std::wcscmp(mode, L"--rebuild") == 0;
        const bool wantRebuildOverlay = std::wcscmp(mode, L"--rebuild-overlay") == 0;
        if (std::wcscmp(mode, L"--nfsw-surfaces") == 0) {
            const nfs::Section* z0 = world.Table().FindByNumber(nfs::kNfswCollisionSection);
            if (!z0) {
                std::printf("no Z0 section in the table\n");
                return 57;
            }
            std::ifstream in(world.TracksDirectory() + L"\\" + world.NfswSectionFileName(*z0),
                             std::ios::binary);
            if (!in) {
                std::printf("cannot read the Z0 section file\n");
                return 57;
            }
            std::vector<u8> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
            std::map<u32, size_t> strips, boundaries;
            nfs::NfswSurveyNativeSurfaces({ bytes.data(), bytes.size() }, strips, boundaries);
            const auto label = [](u32 hash) {
                if (hash == nfs::kRoadSurface) return " road";
                if (hash == nfs::kSolidSurface) return " solid wall";
                return "";
            };
            std::printf("NFSW native surfaces: %zu strip meshes, %zu boundary records\n",
                        strips.size(), boundaries.size());
            for (const auto& [hash, count] : strips)
                std::printf("  strips %08X%s: %zu\n", hash, label(hash), count);
            for (const auto& [hash, count] : boundaries)
                std::printf("  boundary %08X%s: %zu\n", hash, label(hash), count);
        } else if (std::wcscmp(mode, L"--skydome") == 0) {
            // The dome's own textures: GLOBAL\SkyDomeTextures.bin, parsed the
            // way World::OpenGlobalPacks does.
            const std::wstring tracks = world.TracksDirectory();
            const size_t slash = tracks.find_last_of(L"\\/");
            const std::wstring game = slash == std::wstring::npos ? L"." : tracks.substr(0, slash);
            std::ifstream in(game + L"\\GLOBAL\\SkyDomeTextures.bin", std::ios::binary);
            if (!in) {
                std::printf("cannot read the GLOBAL sky dome pack\n");
                return 58;
            }
            std::vector<u8> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
            nfs::ChunkReader top({ bytes.data(), bytes.size() }, 0);
            nfs::Chunk chunk;
            bool found = false;
            while (top.Next(chunk)) {
                if (chunk.id != nfs::kChunkTexturePack) continue;
                nfs::TexturePack pack;
                if (!pack.ParseNfsw(chunk)) continue;
                for (const nfs::Texture& texture : pack.Textures()) {
                    std::printf("  %s: %ux%u %s mips %u, base %u, scroll %d (%.4f, %.4f)\n",
                                texture.name.c_str(), texture.width, texture.height,
                                nfs::ToString(texture.format), texture.mipCount,
                                texture.baseLevelSize, texture.scrollType,
                                texture.scrollS, texture.scrollT);
                    // The opened region must serve every band to the renderer.
                    if (!world.FindDisplayTexture(texture.hash)) {
                        std::printf("  band %s (%08X) is missing from the region's global packs\n",
                                    texture.name.c_str(), texture.hash);
                        return 58;
                    }
                }
                found = true;
                // With a second argument, write every band as a 24-bit BMP
                // so the band order and seam can be checked by eye.
                if (argc > 2) {
                    std::vector<u32> pixels;
                    u32 width = 0, height = 0;
                    for (const nfs::Texture& texture : pack.Textures()) {
                        if (!nfs::DecodeTexture(texture, 0, pixels, width, height)) continue;
                        std::vector<u8> row((size_t)width * 3);
                        const size_t stride = ((size_t)width * 3 + 3) & ~size_t(3);
                        std::vector<u8> bmp(stride * height + 54, 0);
                        bmp[0] = 'B'; bmp[1] = 'M';
                        const u32 bits = (u32)(stride * height);
                        const u32 size = bits + 54;
                        const u32 offset54 = 54, header40 = 40;
                        std::memcpy(bmp.data() + 2, &size, 4);
                        std::memcpy(bmp.data() + 10, &offset54, 4);
                        std::memcpy(bmp.data() + 14, &header40, 4);
                        std::memcpy(bmp.data() + 18, &width, 4);
                        std::memcpy(bmp.data() + 22, &height, 4);
                        bmp[26] = 1; bmp[28] = 24;
                        std::memcpy(bmp.data() + 34, &bits, 4);
                        for (u32 y = 0; y < height; ++y) {
                            for (u32 x = 0; x < width; ++x) {
                                const u32 argb = pixels[(size_t)y * width + x];
                                u8* out = bmp.data() + 54 + (size_t)(height - 1 - y) * stride + x * 3;
                                out[0] = (u8)(argb & 0xFF);
                                out[1] = (u8)((argb >> 8) & 0xFF);
                                out[2] = (u8)((argb >> 16) & 0xFF);
                            }
                        }
                        std::ofstream out(std::wstring(argv[2]) + L"\\" +
                                          std::wstring(texture.name.begin(), texture.name.end()) + L".bmp",
                                          std::ios::binary);
                        out.write((const char*)bmp.data(), (std::streamsize)bmp.size());
                    }
                }
            }
            if (!found) {
                std::printf("no texture pack in the sky dome file\n");
                return 58;
            }
        } else if (std::wcscmp(mode, L"--toll-site") == 0) {
            const auto* section = world.Table().FindByNumber(5115);
            const auto* toll = section ? world.LoadSection(*section) : nullptr;
            if (!toll) return 56;
            Vec3 focus;
            bool located = false;
            for (const auto& item : toll->scenery.instances) {
                if (item.infoIndex >= toll->scenery.infos.size()) continue;
                auto name = toll->scenery.infos[item.infoIndex].name;
                for (char& c : name) c = (char)std::toupper((unsigned char)c);
                if (name.find("ROCKPORTTURNPIKE") == std::string::npos) continue;
                const Vec3 point = item.worldBounds.Valid() ? item.worldBounds.Center() : item.transform.TransformPoint({});
                std::printf("Rockport Turnpike: section %s (%u), %s at %.3f %.3f %.3f\n",
                            section->name.c_str(), section->number, name.c_str(), point.x, point.y, point.z);
                focus = point;
                located = true;
            }
            if (!located) return 56;
            world.LoadAllSections();
            const Vec3 exit{ 8305.421f, -2497.036f, 105.027f };
            Aabb exitBounds;
            exitBounds.Add(exit - Vec3{ 3, 3, 2 });
            exitBounds.Add(exit + Vec3{ 3, 3, 2 });
            const auto terminal = nfs::NfswFindRoadEnd(world.LayoutBytes(), exit, exitBounds, { 0, -1, 0 });
            if (!terminal || Length(terminal->position - exit) > 0.03f || terminal->forward != 3 || terminal->backward != 3)
                return 56;
            std::printf("  Toll terminal is %d+%d lanes with shoulders %.2f m and %.2f m, median %.2f m\n",
                        terminal->forward, terminal->backward, terminal->leftShoulder, terminal->rightShoulder,
                        terminal->medianWidth);
            const Vec3 extension = terminal->heading * 10.0f;
            const std::vector<nfs::NfswRoadSegmentBuild> pull{
                { 0, 1, { exit, exit + extension, exit + extension * 2, exit + extension * 3 },
                  terminal->forward, terminal->backward, terminal->laneWidth, terminal->medianWidth,
                  terminal->nativeProfile }
            };
            const ByteSpan layout = world.LayoutBytes();
            std::vector<u8> extended(layout.begin(), layout.end());
            std::string error;
            nfs::NfswRoadBuildReport report;
            if (!nfs::NfswAppendRoadGraph(extended, pull, report, error) || report.stockJoins != 1) return 56;
            roads::Network network;
            roads::RoadProfile profile;
            profile.lanesForward = terminal->forward;
            profile.lanesBackward = terminal->backward;
            profile.laneWidth = terminal->laneWidth;
            profile.medianType = 1;
            profile.medianWidth = terminal->medianWidth;
            profile.shoulderType = 2;
            profile.shoulderWidth = terminal->leftShoulder;
            profile.worldLaneProfile = terminal->nativeProfile;
            roads::ApplyWorldRoadMaterials(world, roads::WorldRoadType::Highway, profile);
            const u32 startNode = network.AddNode(terminal->position);
            const u32 endNode = network.AddNode(terminal->position + extension * 3);
            if (!network.AddSegment(startNode, endNode, profile)) return 56;
            std::vector<NfswMeshOverlay> overlays;
            std::wstring bakeError;
            if (!roads::PlanWorldRoadBake(world, network, overlays, report, bakeError) ||
                report.stockJoins != 1 || overlays.empty()) {
                std::printf("  Toll terminal bake: %ls\n", bakeError.c_str());
                return 56;
            }
            std::printf("  Toll extension: %zu overlays planned with a native AI join\n", overlays.size());
            bool hasLink = false;
            for (const auto& loaded : world.Loaded()) {
                for (const auto& item : loaded->scenery.instances) {
                    const auto* model = world.FindModel(loaded->scenery.ModelHash(item));
                    if (!model || !model->bounds.Valid() || !model->HasNfswRoadSurface()) continue;
                    nfs::PlacedInstance placement;
                    placement.modelHash = model->hash;
                    placement.transform = item.transform;
                    const Aabb bounds = Transform(model->bounds, item.transform);
                    if (bounds.min.x > exit.x + 20 || bounds.max.x < exit.x - 20 ||
                        bounds.min.y > exit.y + 20 || bounds.max.y < exit.y - 20 ||
                        bounds.min.z > exit.z + 10 || bounds.max.z < exit.z - 10) continue;
                    const auto end = nfs::NfswFindRoadEnd(world.LayoutBytes(), exit, bounds, { 0, -1, 0 });
                    if (!end) continue;
                    hasLink = true;
                    std::printf("  %s (%s): native link at %.3f %.3f %.3f, heading (%.2f, %.2f), %d+%d lanes\n",
                                loaded->info->name.c_str(), model->name.c_str(),
                                end->position.x, end->position.y, end->position.z,
                                end->heading.x, end->heading.y, end->forward, end->backward);
                }
            }
            if (!hasLink) std::printf("  The actual terminal sits beyond the bounds of nearby road meshes.\n");
        } else if (std::wcscmp(mode, L"--road-assets") == 0) {
            if (!CheckRoadAssets(world, *loaded)) return 55;
        } else if (std::wcscmp(mode, L"--road-ground") == 0) {
            if (!CheckMissingRoadGround(world)) return 54;
        } else if (std::wcscmp(mode, L"--road-bake") == 0) {
            if (!CheckWorldRoadBake(world, *loaded)) return 53;
        } else if (std::wcscmp(mode, L"--blender-mesh") == 0) {
            if (!CheckWorldMeshImport(world, *loaded)) return 52;
        } else if (std::wcscmp(mode, L"--smackable-attributes") == 0) {
            if (argc > 2 && !CheckSmackableAttributeOverlay(world, argv[2])) return 49;
            world.LoadAllSections();
            std::set<u32> boundKeys;
            size_t withoutBox = 0;
            for (const auto& loadedSection : world.Loaded()) {
                for (const auto& link : loadedSection->smackableLinks) boundKeys.insert(link.physics);
                for (const auto& binding : loadedSection->smackableScenery) {
                    const auto* link = world.FindSmackableLink(*loadedSection, binding.first);
                    if (link && link->box == SIZE_MAX && world.FindSmackableAttributes(link->physics)) ++withoutBox;
                }
            }
            const auto resolved = std::count_if(boundKeys.begin(), boundKeys.end(), [&](u32 key) {
                return world.FindSmackableAttributes(key) != nullptr;
            });
            std::printf("NFSW smackable attributes: %zu/%zu section keys resolve in GLOBAL/ATTRIBUTES.BIN\n",
                        (size_t)resolved, boundKeys.size());
            std::printf("  %zu scenery instances resolve without a collision box preview\n", withoutBox);
            if (!withoutBox) return 49;
            constexpr u32 samples[] = { 0x263C5311, 0xA98D0CB7, 0x8C582F44 };
            for (u32 key : samples) {
                const auto* preset = world.FindSmackableAttributes(key);
                if (!boundKeys.count(key) || !preset || preset->name.empty() || !preset->mass ||
                    !preset->rigidBody || !preset->baseMaterial || preset->materialName.empty()) return 49;
                std::printf("  %08X %s, mass %.6g, rigidbodyspecs %08X, BASE_MATERIAL %s\n",
                            key, preset->name.c_str(), *preset->mass, preset->rigidBody,
                            preset->materialName.c_str());
            }
        } else if (wantStream) {
            world.LoadAllSections();
            const float x = (float)std::wcstod(argv[2], nullptr);
            const float y = (float)std::wcstod(argv[3], nullptr);
            const float z = argc > 4 ? (float)std::wcstod(argv[4], nullptr) : 0.0f;
            nfs::StreamingModel model;
            const bool parsed = model.Parse(world.LayoutBytes());
            const Vec3 point{ x, y, z };
            const int loading = model.SectionAt(point, 0.0, world.Visibility());
            const int polygon = world.Visibility().ZoneAt(x, y);
            const int polygonId = polygon >= 0
                ? (int)world.Visibility().Zones()[(size_t)polygon].id : -1;
            std::printf("stream at %.1f %.1f %.1f: path zones %zu (%s), loading zone %d",
                        x, y, z, model.PathZoneCount(), parsed ? "parsed" : "EMPTY", loading);
            if (loading >= 0) {
                const auto* list = world.Visibility().ListOf((u32)loading);
                std::printf(" list %zu", list ? list->size() : 0);
            }
            std::printf(", polygon zone %d\n", polygonId);
            for (const nfs::StreamingModel::PathZone& zone : model.PathZones()) {
                if (!zone.Contains(x, y)) continue;
                std::printf("  path zone %u %u %u %u height %.1f\n", zone.candidates[0],
                            zone.candidates[1], zone.candidates[2], zone.candidates[3], zone.height);
            }
        } else if (wantZone) {
            world.LoadAllSections();
            const float x = (float)std::wcstod(argv[2], nullptr);
            const float y = (float)std::wcstod(argv[3], nullptr);
            const nfs::Preculler& visibility = world.Visibility();
            const int index = visibility.ZoneAt(x, y);
            std::printf("zone at %.1f %.1f: %d", x, y,
                        index >= 0 ? (int)visibility.Zones()[index].id : -1);
            if (index >= 0) {
                const auto* list = visibility.ListOf(visibility.Zones()[index].id);
                std::printf(" list %zu", list ? list->size() : 0);
            }
            std::printf("\n");
            for (const nfs::Preculler::Zone& zone : visibility.Zones()) {
                if (!visibility.Contains(zone, x, y)) continue;
                const auto* list = visibility.ListOf(zone.id);
                std::printf("  holds %u list %zu area %.0f\n", zone.id, list ? list->size() : 0,
                            (zone.boundsMax.x - zone.boundsMin.x) *
                            (zone.boundsMax.y - zone.boundsMin.y));
            }
        } else if (wantUnresolved) {
            world.LoadAllSections();
            struct Missing {
                std::string name;
                std::string firstSection;
                u32 hashes[4] = {};
                size_t instances = 0;
                size_t anyResolved = 0;
            };
            std::map<std::tuple<std::string, u32, u32, u32, u32>, Missing> groups;
            for (const auto& section : world.Loaded()) {
                const nfs::Scenery& scenery = section->scenery;
                for (const nfs::SceneryInstance& instance : scenery.instances) {
                    if (instance.infoIndex >= scenery.infos.size()) continue;
                    const nfs::SceneryInfo& info = scenery.infos[instance.infoIndex];
                    if (world.FindModel(info.solidHash)) continue;
                    u32 hashes[4] = {};
                    if (info.raw.size() >= 0x28)
                        std::memcpy(hashes, info.raw.data() + 0x18, sizeof(hashes));
                    const auto key = std::make_tuple(info.name, hashes[0], hashes[1], hashes[2], hashes[3]);
                    Missing& missing = groups[key];
                    if (missing.instances == 0) {
                        missing.name = info.name;
                        std::memcpy(missing.hashes, hashes, sizeof(hashes));
                        missing.firstSection = section->info->name;
                    }
                    ++missing.instances;
                    for (u32 hash : hashes)
                        if (hash && world.FindModel(hash)) { ++missing.anyResolved; break; }
                }
            }
            std::vector<const Missing*> ordered;
            for (const auto& entry : groups) ordered.push_back(&entry.second);
            std::sort(ordered.begin(), ordered.end(),
                      [](const Missing* a, const Missing* b) { return a->instances > b->instances; });
            size_t instances = 0, resolvable = 0;
            for (const Missing* missing : ordered) {
                instances += missing->instances;
                resolvable += missing->anyResolved;
            }
            std::printf("unresolved infos: %zu groups, %zu instances (%zu have another hash that resolves)\n",
                        ordered.size(), instances, resolvable);
            for (size_t i = 0; i < ordered.size() && i < 60; ++i) {
                const Missing* missing = ordered[i];
                std::printf("  %-24s %zu instances, %zu any-resolved, %s, %08X %08X %08X %08X\n",
                            missing->name.c_str(), missing->instances, missing->anyResolved,
                            missing->firstSection.c_str(), missing->hashes[0], missing->hashes[1],
                            missing->hashes[2], missing->hashes[3]);
            }
        } else if (wantPatch) {
            // Equal-size writeback: move one A100 record, reparse the patched
            // BUN and check the transform and stored bounds came back.
            const nfs::SceneryInstance* source = nullptr;
            for (const nfs::SceneryInstance& instance : loaded->scenery.instances)
                if (instance.sceneryNumber && instance.worldBounds.Valid()) {
                    source = &instance;
                    break;
                }
            if (!source) {
                std::printf("no A100 scenery record to patch\n");
                return 45;
            }

            nfs::NfswSceneryPatch patch;
            patch.sceneryNumber = source->sceneryNumber;
            patch.record = source->record;
            patch.transform = source->transform;
            patch.transform.m[3][0] += 100.0f;
            patch.bounds = source->worldBounds;
            patch.bounds.min.x += 100.0f;
            patch.bounds.max.x += 100.0f;

            const ByteSpan bytes = world.SectionBytes(*section);
            std::vector<u8> patched;
            std::string error;
            if (!nfs::NfswPatchScenery(bytes, { patch }, patched, error)) {
                std::printf("NFSW scenery patch failed: %s\n", error.c_str());
                return 45;
            }
            if (patched.size() != bytes.size) {
                std::printf("NFSW scenery patch changed the file size: %zu -> %zu\n",
                            bytes.size, patched.size());
                return 45;
            }

            nfs::Scenery check;
            bool foundChunk = false;
            nfs::ChunkReader top({ patched.data(), patched.size() }, 0);
            nfs::Chunk c;
            while (top.Next(c)) {
                if (c.id != nfs::kChunkTrackScenery) continue;
                const ByteSpan whole{ c.raw.data - 8, c.raw.size + 8 };
                nfs::NfswSceneryChunkInfo info;
                if (!nfs::NfswReadSceneryChunk(whole, 0, info) ||
                    info.number != patch.sceneryNumber)
                    continue;
                nfs::NfswParseScenery(c, check);
                foundChunk = true;
                break;
            }
            if (!foundChunk) {
                std::printf("NFSW scenery patch: chunk missing after reparse\n");
                return 45;
            }
            const nfs::SceneryInstance* moved = nullptr;
            for (const nfs::SceneryInstance& instance : check.instances)
                if (instance.record == patch.record) { moved = &instance; break; }
            if (!moved) {
                std::printf("NFSW scenery patch: record missing after reparse\n");
                return 45;
            }
            const float dx = moved->transform.m[3][0] - patch.transform.m[3][0];
            const float bx = moved->worldBounds.min.x - patch.bounds.min.x;
            if (std::fabs(dx) > 0.01f || std::fabs(bx) > 0.01f) {
                std::printf("NFSW scenery patch: mismatch x %.3f/%.3f bounds %.3f/%.3f\n",
                            moved->transform.m[3][0], patch.transform.m[3][0],
                            moved->worldBounds.min.x, patch.bounds.min.x);
                return 45;
            }
            std::printf("NFSW scenery patch: size %zu, record %u chunk %u, x %.1f -> %.1f, "
                        "bounds x %.1f -> %.1f\n",
                        patched.size(), patch.record, patch.sceneryNumber,
                        source->transform.m[3][0], moved->transform.m[3][0],
                        source->worldBounds.min.x, moved->worldBounds.min.x);
        } else if (wantOverlay) {
            // Simulate the editor's detach + save: one A100 record becomes a
            // placement with originalRecord, then the overlay writer writes the
            // whole section BUN to a temporary models folder.
            nfs::LoadedSection* current = nullptr;
            for (auto& item : world.Loaded())
                if (item->info == section) { current = item.get(); break; }
            if (!current) {
                std::printf("NFSW overlay: A100 is not loaded\n");
                return 46;
            }
            const nfs::SceneryInstance* source = nullptr;
            for (const nfs::SceneryInstance& instance : current->scenery.instances)
                if (instance.sceneryNumber && instance.worldBounds.Valid()) {
                    source = &instance;
                    break;
                }
            if (!source) {
                std::printf("NFSW overlay: no A100 record\n");
                return 46;
            }

            nfs::PlacedInstance placed;
            placed.modelHash = current->scenery.ModelHash(*source);
            placed.modelName = current->scenery.infos[source->infoIndex].name;
            placed.originalSection = section->number;
            placed.originalSceneryNumber = source->sceneryNumber;
            placed.originalRecord = source->record;
            placed.transform = source->transform;
            placed.transform.m[3][0] += 50.0f;
            placed.flags = (u16)source->flags;
            if (const nfs::SolidObject* model = world.FindModel(placed.modelHash))
                placed.modelBounds = model->bounds;

            current->placements.push_back({ source->sceneryNumber, placed });
            current->removedNatives.push_back({ source->sceneryNumber, source->record });
            current->placementsChanged = true;

            wchar_t temp[MAX_PATH] = {};
            ::GetTempPathW(MAX_PATH, temp);
            const std::wstring folder = std::wstring(temp) + L"nfsw-overlay-test";
            std::wstring report;
            if (!WriteNfswPlacementOverlays(world, folder, report)) {
                std::printf("NFSW overlay write failed: %ls\n", report.c_str());
                return 46;
            }
            const std::wstring path = folder + L"\\files\\TRACKS\\" + world.NfswSectionFileName(*section);
            std::vector<unsigned char> patched;
            if (!ReadFile(path, patched)) {
                std::printf("NFSW overlay: missing %ls\n", path.c_str());
                return 46;
            }

            nfs::Scenery check;
            bool found = false;
            nfs::ChunkReader top({ patched.data(), patched.size() }, 0);
            nfs::Chunk c;
            while (top.Next(c)) {
                if (c.id != nfs::kChunkTrackScenery) continue;
                const ByteSpan whole{ c.raw.data - 8, c.raw.size + 8 };
                nfs::NfswSceneryChunkInfo info;
                if (!nfs::NfswReadSceneryChunk(whole, 0, info) ||
                    info.number != source->sceneryNumber)
                    continue;
                nfs::NfswParseScenery(c, check);
                found = true;
                break;
            }
            const nfs::SceneryInstance* moved = nullptr;
            for (const nfs::SceneryInstance& instance : check.instances)
                if (instance.record == source->record) { moved = &instance; break; }
            if (!found || !moved || std::fabs(moved->transform.m[3][0] - placed.transform.m[3][0]) > 0.01f) {
                std::printf("NFSW overlay: written transform did not round-trip\n");
                return 46;
            }
            std::printf("NFSW overlay write: %ls, record %u chunk %u, x %.1f -> %.1f\n",
                        path.c_str(), placed.originalRecord, source->sceneryNumber,
                        source->transform.m[3][0], moved->transform.m[3][0]);

            ::DeleteFileW(path.c_str());
            ::RemoveDirectoryW((folder + L"\\files\\TRACKS").c_str());
            ::RemoveDirectoryW((folder + L"\\files").c_str());
            ::RemoveDirectoryW(folder.c_str());
        } else if (wantRebuild) {
            // Validate the rebuild primitive without writing a game file:
            // no-op identity, one removal, one same-section append, one
            // cross-section append A100 -> A101.
            const nfs::Section* a101 = world.Table().FindByName("A101");
            if (!a101) {
                std::printf("A101 missing\n");
                return 47;
            }
            std::vector<unsigned char> a101Bytes;
            if (!ReadFile(Tracks() + L"/STREAML5RA_101.BUN", a101Bytes)) {
                std::printf("cannot read A101 section file\n");
                return 47;
            }
            const nfs::LoadedSection* target = world.LoadSection(*a101);
            if (!target || target->sceneryNumbers.empty()) {
                std::printf("A101 has no scenery chunk\n");
                return 47;
            }
            const u32 targetNumber = target->sceneryNumbers[0];
            const std::vector<nfs::SceneryInstance>& source = loaded->scenery.instances;
            if (source.empty()) {
                std::printf("A100 has no scenery instances\n");
                return 47;
            }
            std::vector<bool> overridden(source.size(), false);
            for (const nfs::SceneryOverride& entry : loaded->scenery.overrides)
                if (entry.instance < overridden.size()) overridden[entry.instance] = true;

            u32 removal = ~0u, templateRecord = ~0u;
            for (size_t i = 0; i < source.size(); ++i) {
                if (overridden[i]) continue;
                if (removal == ~0u) removal = source[i].record;
                if (source[i].sceneryNumber == 100 && source[i].infoIndex < loaded->scenery.infos.size()) {
                    templateRecord = source[i].record;
                    break;
                }
            }
            if (removal == ~0u || templateRecord == ~0u) {
                std::printf("no usable A100 records for the rebuild test\n");
                return 47;
            }

            const ByteSpan sourceBytes{ sectionBytes.data(), sectionBytes.size() };
            std::string error;

            std::vector<u8> noop;
            if (!nfs::NfswRebuildScenery(sourceBytes, 100, {}, {}, {}, noop, error)) {
                std::printf("NFSW rebuild no-op failed: %s\n", error.c_str());
                return 47;
            }
            if (noop.size() != sourceBytes.size ||
                std::memcmp(noop.data(), sourceBytes.data, sourceBytes.size) != 0) {
                std::printf("NFSW rebuild no-op is not byte-identical\n");
                return 47;
            }

            std::vector<u8> removed;
            if (!nfs::NfswRebuildScenery(sourceBytes, 100, {}, { removal }, {}, removed, error)) {
                std::printf("NFSW rebuild removal failed: %s\n", error.c_str());
                return 47;
            }
            std::string rangeError;
            if (!VectorRangesCover({ removed.data(), removed.size() }, 100, rangeError)) {
                std::printf("NFSW rebuild removal vectorized ranges: %s\n", rangeError.c_str());
                return 47;
            }
            const nfs::SceneryInstance* templateInstance = nullptr;
            for (const nfs::SceneryInstance& instance : source)
                if (instance.record == templateRecord) { templateInstance = &instance; break; }
            if (!templateInstance) {
                std::printf("template record vanished\n");
                return 47;
            }

            nfs::NfswSceneryAppend append;
            append.sceneryNumber = 100;
            append.templateSection = sourceBytes;
            append.templateSceneryNumber = 100;
            append.templateRecord = templateRecord;
            append.modelHash = loaded->scenery.ModelHash(*templateInstance);
            append.transform = templateInstance->transform;
            append.transform.m[3][0] += 25.0f;
            append.bounds = templateInstance->worldBounds;
            append.bounds.min.x += 25.0f;
            append.bounds.max.x += 25.0f;

            std::vector<u8> appended;
            if (!nfs::NfswRebuildScenery(sourceBytes, 100, {}, {}, { append }, appended, error)) {
                std::printf("NFSW rebuild append failed: %s\n", error.c_str());
                return 47;
            }

            append.sceneryNumber = targetNumber;
            std::vector<u8> cross;
            if (!nfs::NfswRebuildScenery({ a101Bytes.data(), a101Bytes.size() }, targetNumber,
                                         {}, {}, { append }, cross, error)) {
                std::printf("NFSW rebuild cross-section append failed: %s\n", error.c_str());
                return 47;
            }

            if (!VectorRangesCover({ appended.data(), appended.size() }, 100, rangeError) ||
                !VectorRangesCover({ cross.data(), cross.size() }, targetNumber, rangeError)) {
                std::printf("NFSW rebuild vectorized ranges: %s\n", rangeError.c_str());
                return 47;
            }
            std::printf("NFSW rebuild: no-op identical, remove %u, append -> %zu bytes, "
                        "cross A100->A%u -> %zu bytes, ranges cover all cull entries\n",
                        removal, appended.size(), targetNumber, cross.size());
        } else if (wantRebuildOverlay) {
            // End-to-end rebuild writer test in a temporary models folder:
            // cross-section move A100 -> A101, delete one A100 record, and add
            // a new placement cloned from A100. Every output BUN and the layout
            // size patch are checked before the temporary files are removed.
            const nfs::Section* a101 = world.Table().FindByName("A101");
            if (!a101) {
                std::printf("A101 missing\n");
                return 48;
            }
            std::vector<unsigned char> a101Bytes;
            if (!ReadFile(Tracks() + L"/STREAML5RA_101.BUN", a101Bytes)) {
                std::printf("cannot read A101 section file\n");
                return 48;
            }
            world.LoadSection(*a101);
            nfs::LoadedSection* sourceSection = nullptr;
            nfs::LoadedSection* targetSection = nullptr;
            for (auto& item : world.Loaded()) {
                if (item->info == section) sourceSection = item.get();
                if (item->info == a101) targetSection = item.get();
            }
            if (!sourceSection || !targetSection || targetSection->sceneryNumbers.empty()) {
                std::printf("A100/A101 not loaded\n");
                return 48;
            }
            const std::vector<nfs::SceneryInstance>& instances = sourceSection->scenery.instances;
            std::vector<bool> overridden(instances.size(), false);
            for (const nfs::SceneryOverride& entry : sourceSection->scenery.overrides)
                if (entry.instance < overridden.size()) overridden[entry.instance] = true;

            const nfs::SceneryInstance* moved = nullptr;
            const nfs::SceneryInstance* deleted = nullptr;
            for (size_t i = 0; i < instances.size(); ++i) {
                if (overridden[i] || instances[i].sceneryNumber != 100) continue;
                if (!world.FindModel(sourceSection->scenery.ModelHash(instances[i]))) continue;
                if (!moved) { moved = &instances[i]; continue; }
                if (!deleted) { deleted = &instances[i]; break; }
            }
            if (!moved || !deleted) {
                std::printf("not enough A100 records for the rebuild writer test\n");
                return 48;
            }
            const u32 targetNumber = targetSection->sceneryNumbers[0];

            nfs::PlacedInstance placed;
            placed.modelHash = sourceSection->scenery.ModelHash(*moved);
            placed.modelName = sourceSection->scenery.infos[moved->infoIndex].name;
            placed.originalSection = 100;
            placed.originalSceneryNumber = 100;
            placed.originalRecord = moved->record;
            placed.transform = moved->transform;
            placed.transform.m[3][0] += 25.0f;
            placed.flags = (u16)moved->flags;
            if (const nfs::SolidObject* model = world.FindModel(placed.modelHash))
                placed.modelBounds = model->bounds;

            sourceSection->removedNatives.push_back({ 100, moved->record });
            sourceSection->removedNatives.push_back({ 100, deleted->record });
            sourceSection->placementsChanged = true;
            targetSection->placements.push_back({ targetNumber, placed });
            targetSection->placementsChanged = true;

            nfs::PlacedInstance added;
            added.modelHash = placed.modelHash;
            added.modelName = placed.modelName;
            added.transform = moved->transform;
            added.transform.m[3][0] += 50.0f;
            if (const nfs::SolidObject* model = world.FindModel(added.modelHash))
                added.modelBounds = model->bounds;
            targetSection->placements.push_back({ targetNumber, added });
            targetSection->placementsChanged = true;

            wchar_t temp[MAX_PATH] = {};
            ::GetTempPathW(MAX_PATH, temp);
            const std::wstring folder = std::wstring(temp) + L"nfsw-rebuild-overlay-test";
            std::wstring report;
            if (!WriteNfswPlacementOverlays(world, folder, report)) {
                std::printf("NFSW rebuild overlay failed: %ls\n", report.c_str());
                return 48;
            }

            const std::wstring sourcePath = folder + L"\\files\\TRACKS\\STREAML5RA_100.BUN";
            const std::wstring targetPath = folder + L"\\files\\TRACKS\\STREAML5RA_101.BUN";
            const std::wstring layoutPath = folder + L"\\files\\TRACKS\\L5RA.BUN";
            std::vector<unsigned char> sourceOut, targetOut, layoutOut;
            if (!ReadFile(sourcePath, sourceOut) || !ReadFile(targetPath, targetOut) ||
                !ReadFile(layoutPath, layoutOut)) {
                std::printf("NFSW rebuild overlay: missing output file\n");
                return 48;
            }
            if (sourceOut.size() >= sectionBytes.size() || targetOut.size() <= a101Bytes.size()) {
                std::printf("NFSW rebuild overlay: unexpected sizes %zu/%zu -> %zu/%zu\n",
                            sectionBytes.size(), a101Bytes.size(), sourceOut.size(), targetOut.size());
                return 48;
            }
            nfs::SectionTable patched;
            if (!patched.ParseNfsw({ layoutOut.data(), layoutOut.size() })) {
                std::printf("NFSW rebuild overlay: patched layout does not parse\n");
                return 48;
            }
            const nfs::Section* p100 = patched.FindByNumber(100);
            const nfs::Section* p101 = patched.FindByNumber(101);
            if (!p100 || !p101 || p100->fileSize != sourceOut.size() ||
                p101->fileSize != targetOut.size()) {
                std::printf("NFSW rebuild overlay: layout sizes not patched\n");
                return 48;
            }
            std::printf("NFSW rebuild overlay: A100 %zu -> %zu, A101 %zu -> %zu, layout patched\n",
                        sectionBytes.size(), sourceOut.size(), a101Bytes.size(), targetOut.size());

            ::DeleteFileW(sourcePath.c_str());
            ::DeleteFileW(targetPath.c_str());
            ::DeleteFileW(layoutPath.c_str());
            ::DeleteFileW((folder + L"\\files\\TRACKS\\STREAML5RA_25000.BUN").c_str());
            sourceSection->removedNatives.clear();
            sourceSection->placementsChanged = false;
            targetSection->placements.clear();
            targetSection->placementsChanged = false;
            if (!CheckCopiedCollision(world, folder)) {
                std::printf("NFSW copied phone booth/road collision regression failed\n");
                return 48;
            }
            if (!CheckNativeSmackablePose(world, folder)) {
                std::printf("NFSW native smackable regression failed\n");
                return 48;
            }
            if (!CheckExactSmackableCopy(world, folder)) {
                std::printf("NFSW exact smackable copy regression failed\n");
                return 48;
            }
            if (!CheckNativeSmackableBinding(world, folder)) {
                std::printf("NFSW native smackable binding regression failed\n");
                return 48;
            }
            if (!CheckSmackableBoxResize(world, folder)) {
                std::printf("NFSW single collision box regression failed\n");
                return 48;
            }
            if (!CheckPlacementStreaming(world, folder)) {
                std::printf("NFSW placement streaming regression failed\n");
                return 48;
            }
            ::RemoveDirectoryW((folder + L"\\files\\TRACKS").c_str());
            ::RemoveDirectoryW((folder + L"\\files").c_str());
            ::RemoveDirectoryW(folder.c_str());
        } else if (!findName.empty() || wantOverlaps) {
            world.LoadAllSections();
            if (!findName.empty()) FindInstances(world, findName);
            if (wantOverlaps) OverlappingInstances(world);
        } else if (std::wcscmp(mode, L"--all") == 0) {
            const clock_t start = std::clock();
            world.LoadAllSections();
            if (!CheckStockCollisionAlignment(world)) {
                std::printf("NFSW stock collision: A100 ground or smackable bounds are misaligned\n");
                return 1;
            }
            size_t objects = 0, triangles = 0;
            for (const auto& section : world.Loaded()) {
                objects += section->ObjectCount();
                triangles += section->TriangleCount();
            }
            std::printf("World full: %zu sections, %zu objects, %zu triangles, %zu instances, "
                        "%zu models, %zu textures, %.1f s\n",
                        world.Loaded().size(), objects, triangles, world.InstanceCount(),
                        world.AllModels().size(), world.AllTextures().size(),
                        (double)(std::clock() - start) / CLOCKS_PER_SEC);
            {
                const u32 hashes[] = { 0x739618D3u, 0x04F4057Au, 0x1113B973u };
                size_t resolved = 0;
                for (u32 hash : hashes) {
                    const nfs::Texture* global = world.FindDisplayTexture(hash);
                    std::printf("global compressed texture %08X: %s %ux%u format %d\n", hash,
                                global ? global->name.c_str() : "MISSING",
                                global ? global->width : 0, global ? global->height : 0,
                                global ? (int)global->format : -1);
                    if (global && global->pixels.size) ++resolved;
                }
                if (resolved != 3) return 47;
            }
            // The seven Z0 tracks are the city's moving landmarks. Each must
            // name a shipped instance by GUID and start on that instance.
            {
                const u32 expected[] = {
                    0x54256272u, 0x54256273u, 0x795d1913u, 0xb66d5ae4u,
                    0xaca0af9au, 0x6abbfaf3u, 0x6abbfaf4u,
                };
                size_t found = 0;
                const auto& animations = world.SceneryAnimations();
                for (u32 guid : expected) {
                    const auto animation = animations.find(guid);
                    const nfs::SceneryInstance* placed = nullptr;
                    const nfs::LoadedSection* owner = nullptr;
                    for (const auto& section : world.Loaded()) {
                        for (const nfs::SceneryInstance& candidate : section->scenery.instances) {
                            if (candidate.guid != guid) continue;
                            placed = &candidate;
                            owner = section.get();
                            break;
                        }
                        if (placed) break;
                    }
                    const nfs::SolidObject* model = nullptr;
                    if (placed && owner) model = world.FindModel(owner->scenery.ModelHash(*placed));
                    if (animation == animations.end()) {
                        std::printf("  animation %08X MISSING\n", guid);
                        continue;
                    }
                    const nfs::NfswObjectAnimation& track = animation->second;
                    std::printf("  animation %08X %-28s frames %5zu speed %.2f (%.0f fps)%s\n", guid,
                                model ? model->name.c_str() : "?", track.frames.size(), track.speed,
                                15.0f * track.speed, placed ? "" : " UNPLACED");
                    if (!placed || track.frames.empty()) continue;
                    const Mat4& first = track.frames.front();
                    const float dx = first.m[3][0] - placed->transform.m[3][0];
                    const float dy = first.m[3][1] - placed->transform.m[3][1];
                    const float dz = first.m[3][2] - placed->transform.m[3][2];
                    if (dx * dx + dy * dy + dz * dz > 1.0f) {
                        std::printf("    first frame starts %.1f %.1f %.1f, instance at %.1f %.1f %.1f\n",
                                    first.m[3][0], first.m[3][1], first.m[3][2],
                                    placed->transform.m[3][0], placed->transform.m[3][1],
                                    placed->transform.m[3][2]);
                        continue;
                    }
                    ++found;
                }
                std::printf("scenery animations: %zu tracks in region, %zu of 7 Z0 tracks verified\n",
                            animations.size(), found);
                if (found != 7) return 46;
            }
            // The E35 landscape ground is 15 TRN_CT_PROPS_HIGHWAY4AUX_*_E35_R0
            // models in section 4435. They must survive the low-LOD and
            // duplicate filters, or the ground next to the E35 road vanishes.
            {
                const u32 aux[] = {
                    0xe82b3e04u, 0xd40c6e24u, 0x378a4785u, 0x236b77a5u,
                    0x86e95106u, 0x72ca8126u, 0xd6485a87u, 0xc2298aa7u,
                    0x25a76408u, 0x75066d89u, 0xc465770au, 0x13c4808bu,
                    0xffa5b0abu, 0x63238a0cu, 0x4f04ba2cu,
                };
                size_t instances = 0, resolved = 0;
                const nfs::LoadedSection* e435 = nullptr;
                for (const auto& section : world.Loaded())
                    if (section->info && section->info->number == 4435) e435 = section.get();
                if (e435) {
                    for (const nfs::SceneryInstance& instance : e435->scenery.instances) {
                        const u32 hash = e435->scenery.ModelHash(instance);
                        bool wanted = false;
                        for (u32 h : aux) wanted = wanted || h == hash;
                        if (!wanted) continue;
                        ++instances;
                        if (world.FindModel(hash)) ++resolved;
                    }
                }
                std::printf("E435 AUX ground: %zu instances, %zu resolved (no name filter)\n",
                            instances, resolved);
                if (instances != 15 || resolved != 15) return 45;
                const u32 textures[] = { 0x5d0b5f6bu, 0xc65bfb53u, 0x5d0c5e43u };
                for (u32 hash : textures) {
                    const nfs::Texture* texture = world.FindTexture(hash);
                    if (!texture) {
                        std::printf("  AUX texture %08X unresolved\n", hash);
                        continue;
                    }
                    std::printf("  AUX texture %08X %s %ux%u format %d alpha %d/%d\n",
                                hash, texture->name.c_str(), texture->width, texture->height,
                                (int)texture->format, (int)texture->alphaUsage,
                                (int)texture->alphaBlend);
                }
            }
            const nfs::SolidObject* shblock = world.FindModel(0xDBB237AFu);
            std::printf("name-based scenery filters removed; CU_SHBLOCK201 %s\n",
                        shblock ? (shblock->IsShadow() ? "shadow" : "NOT shadow") : "missing");
            if (!shblock || !shblock->IsShadow()) return 24;
            if (!world.FindModel(0xB29C1AEBu)) {
                std::printf("scenery model did not resolve after a full load\n");
                return 11;
            }
            const nfs::LoadedSection* a100 = world.LoadSection(*section);
            size_t slots = 0, found = 0;
            for (const nfs::SolidList& list : a100->solidLists)
                for (const nfs::SolidObject& object : list.objects)
                    for (const nfs::SolidMaterial& material : object.materials) {
                        if (material.textureIndex >= object.textureHashes.size()) continue;
                        ++slots;
                        if (world.FindDisplayTexture(object.textureHashes[material.textureIndex])) ++found;
                    }
            std::printf("A100 texture slots: %zu, resolved %zu\n", slots, found);
            if (found * 10 < slots * 9) return 12;

            size_t allSlots = 0, allFound = 0;
            std::vector<u32> missingTextures;
            for (const auto& section : world.Loaded())
                for (const nfs::SolidList& list : section->solidLists)
                    for (const nfs::SolidObject& object : list.objects)
                        for (const nfs::SolidMaterial& material : object.materials) {
                            if (material.textureIndex >= object.textureHashes.size()) continue;
                            ++allSlots;
                            const u32 hash = object.textureHashes[material.textureIndex];
                            if (world.FindDisplayTexture(hash)) ++allFound;
                            else missingTextures.push_back(hash);
                        }
            std::sort(missingTextures.begin(), missingTextures.end());
            missingTextures.erase(std::unique(missingTextures.begin(), missingTextures.end()),
                                  missingTextures.end());
            std::printf("unresolved texture hashes: %zu\n", missingTextures.size());
            for (size_t i = 0; i < missingTextures.size() && i < 20; ++i)
                std::printf("  %08X\n", missingTextures[i]);
            std::printf("all texture slots: %zu, resolved %zu (%.1f%%)\n", allSlots, allFound,
                        allSlots ? 100.0 * allFound / allSlots : 0.0);
            {
                size_t shadowModels = 0;
                u64 shadowTriangles = 0;
                for (const nfs::SolidObject* model : world.AllModels())
                    if (model->IsShadow()) {
                        ++shadowModels;
                        shadowTriangles += model->TriangleCount();
                    }
                std::printf("shadow/reflection proxies skipped: %zu models, %llu triangles\n",
                            shadowModels, (unsigned long long)shadowTriangles);
            }
            {
                const auto& animations = world.TextureAnimations();
                size_t cycles = 0;
                for (const auto& [hash, animation] : animations)
                    if (animation.startFrame == 0) ++cycles;
                std::printf("texture animations: %zu cycles, %zu material hashes\n", cycles,
                            animations.size());
                const auto check = [&](u32 hash, const char* label) {
                    const auto found = animations.find(hash);
                    if (found == animations.end()) {
                        std::printf("  missing animation %s (%08X)\n", label, hash);
                        return false;
                    }
                    std::printf("  %s: %zu frames at %.0f/s\n", label, found->second.frames.size(),
                                found->second.fps);
                    return true;
                };
                const bool ok = check(0x357E346Du, "SGN_CASINOLUCKY") |
                                check(0xC3F334D0u, "ANM_STADIUMTEXT") |
                                check(0x22F5B530u, "TRS_WATERA_ANIM");
                size_t frameSlots = 0, frameFound = 0;
                for (const auto& [hash, animation] : animations) {
                    if (animation.startFrame != 0) continue;
                    for (u32 frame : animation.frames) {
                        ++frameSlots;
                        if (world.FindDisplayTexture(frame)) ++frameFound;
                    }
                }
                std::printf("animation frames: %zu slots, %zu resolved\n", frameSlots, frameFound);
                size_t scrolling = 0;
                for (const nfs::Texture* texture : world.AllTextures())
                    if (texture->scrollType != 0) ++scrolling;
                const nfs::Texture* wave = world.FindDisplayTexture(0x9799726Au);
                const nfs::Texture* ticker = world.FindDisplayTexture(0xDA463E9Cu);
                const nfs::Texture* stepped = world.FindDisplayTexture(0xD9836F67u);
                std::printf("scrolling textures: %zu; ocean wave type %u T %.4f, "
                            "ticker type %u S %.4f, stepped type %u T %.4f\n",
                            scrolling,
                            wave ? wave->scrollType : 0, wave ? wave->scrollT : 0.0f,
                            ticker ? ticker->scrollType : 0, ticker ? ticker->scrollS : 0.0f,
                            stepped ? stepped->scrollType : 0, stepped ? stepped->scrollT : 0.0f);
                if (!ok || frameFound * 10 < frameSlots * 9 || !wave || !ticker || !stepped ||
                    wave->scrollType != 1 || ticker->scrollType != 1 || stepped->scrollType != 2 ||
                    wave->scrollT < 0.06f || wave->scrollT > 0.065f ||
                    ticker->scrollS < 0.09f || ticker->scrollS > 0.11f)
                    return 17;
            }
            {
                const nfs::SolidObject* tuner = world.FindModel(0xF28B01BAu);
                const nfs::SolidObject* island = world.FindModel(0x7B1DE63Cu);
                std::printf("backdrop samples: PANO_TUNER %s, CU_CT_ISLANDS_03 %s\n",
                            tuner ? (tuner->IsBackdrop() ? "backdrop" : "NOT backdrop") : "missing",
                            island ? (island->IsBackdrop() ? "backdrop" : "NOT backdrop") : "missing");
                if (!tuner || !tuner->IsBackdrop() || !island || !island->IsBackdrop()) return 18;
            }
            {
                // Holiday groups: A31 instance 204 is a Christmas light, and
                // A436's 0x34106 entry 8905 is in the Halloween group. A436 has
                // two scenery chunks, so its local instance 275 is combined
                // index 277 (two instances come from the first chunk). Normal
                // must hide both; each holiday must show its own.
                world.SetHoliday(nfs::Holiday::Normal);
                const bool normalChristmas = world.HolidayHides(31, 204);
                const bool normalMoon = world.HolidayHides(436, 277);
                const size_t normalHidden = world.HolidayHiddenCount();
                world.SetHoliday(nfs::Holiday::Christmas);
                const bool christmasLight = world.HolidayHides(31, 204);
                world.SetHoliday(nfs::Holiday::Halloween);
                const bool halloweenMoon = world.HolidayHides(436, 277);
                std::printf("holiday scenery: %zu groups, %zu overrides, %zu hides; "
                            "A31 light normal %d christmas %d; A436 halloween normal %d halloween %d\n",
                            world.SceneryGroupCount(), world.SceneryOverrideCount(), normalHidden,
                            normalChristmas, christmasLight, normalMoon, halloweenMoon);
                if (!normalChristmas || christmasLight || !normalMoon || halloweenMoon) return 19;
                world.SetHoliday(nfs::Holiday::Normal);
            }
            {
                size_t refl = 0, rfl = 0, shd = 0, shadow = 0, pan = 0, lowLod = 0, total = 0;
                for (const auto& section : world.Loaded())
                    for (const nfs::SceneryInstance& instance : section->scenery.instances) {
                        const nfs::SolidObject* model =
                            world.FindModel(section->scenery.ModelHash(instance));
                        if (!model) continue;
                        ++total;
                        const std::string& name = model->name;
                        if (name.rfind("REFL_", 0) == 0) ++refl;
                        else if (name.rfind("RFL_", 0) == 0) ++rfl;
                        else if (name.rfind("SHD_", 0) == 0) ++shd;
                        else if (name.rfind("SHADOW", 0) == 0) ++shadow;
                        else if (name.rfind("PAN_", 0) == 0) ++pan;
                        else if (name.find("_1Z_") != std::string::npos) {
                            ++lowLod;
                            if (lowLod <= 12)
                                std::printf("  low-LOD instance %-36s in %-6s at %.1f %.1f %.1f\n",
                                            name.c_str(),
                                            section->info ? section->info->name.c_str() : "?",
                                            instance.transform.m[3][0], instance.transform.m[3][1],
                                            instance.transform.m[3][2]);
                        }
                    }
                std::printf("instances by proxy/LOD prefix: total resolved %zu, REFL_ %zu, RFL_ %zu, "
                            "SHD_ %zu, SHADOW %zu, PAN_ %zu, _1Z_ %zu\n",
                            total, refl, rfl, shd, shadow, pan, lowLod);
            }
            {
                std::map<std::string, size_t> diffuseFormats;
                for (const auto& section : world.Loaded())
                    for (const nfs::SolidList& list : section->solidLists)
                        for (const nfs::SolidObject& object : list.objects)
                            for (const nfs::SolidMaterial& material : object.materials) {
                                if (material.textureIndex >= object.textureHashes.size()) continue;
                                const nfs::Texture* t = world.FindDisplayTexture(object.textureHashes[material.textureIndex]);
                                if (!t) continue;
                                if (t->format == nfs::TextureFormat::Bc4 || t->format == nfs::TextureFormat::Bc5)
                                    std::printf("  ATI diffuse: %s %s %08X %s\n", object.name.c_str(),
                                                t->name.c_str(), t->hash, nfs::ToString(t->format));
                            }
                for (const auto& [name, count] : diffuseFormats)
                    std::printf("  diffuse format %s: %zu\n", name.c_str(), count);
            }
            // Effect 10 (GLASS_REFLECT) must decode its UVs and vertex colour;
            // the default branch used to leave every vertex at (0,0) / white,
            // which turns the whole mesh into one flat texel.
            {
                const nfs::SolidObject* hotel = nullptr;
                for (const nfs::SolidObject* model : world.AllModels())
                    if (model->name == "SHD_HOTELSSHPEREA_01") hotel = model;
                if (hotel) {
                    size_t withUv = 0;
                    for (const nfs::SolidVertex& vertex : hotel->vertices)
                        if (vertex.uv.x != 0.0f || vertex.uv.y != 0.0f) ++withUv;
                    std::printf("effect 10 sample SHD_HOTELSSHPEREA_01: %zu/%zu vertices carry a UV, "
                                "shadow proxy %s\n",
                                withUv, hotel->vertices.size(),
                                hotel->IsShadow() ? "yes" : "no");
                    if (!withUv) return 14;
                } else {
                    std::printf("effect 10 sample SHD_HOTELSSHPEREA_01 not found\n");
                }
            }
            // The user-visible example: the dam-cliff reflection face that
            // covered the road must be classified as a proxy and skipped.
            {
                const nfs::SolidObject* dam = world.FindModel(0x222812BDu);
                std::printf("REFL_DAMCLIFF_01: %s, shadow proxy %s, %u triangles\n",
                            dam ? dam->name.c_str() : "missing",
                            dam && dam->IsShadow() ? "yes" : "no",
                            dam ? dam->TriangleCount() : 0u);
                if (!dam || !dam->IsShadow()) return 16;
            }
            // The +400 siblings are additional placement layers, not hidden
            // alternates: A431 carries the refinery towers that A31 only
            // lights. Check that both are loaded and that the towers resolve.
            // A31's refinery towers live in library section X549. Check that
            // the model is loaded and that the section's scenery instance can
            // name it; this separates a data problem from a draw/cull problem.
            {
                const nfs::LoadedSection* a31 = nullptr;
                const nfs::LoadedSection* a431 = nullptr;
                const nfs::LoadedSection* x549 = nullptr;
                for (const auto& section : world.Loaded()) {
                    if (!section->info) continue;
                    if (section->info->name == "A31") a31 = section.get();
                    if (section->info->name == "A431") a431 = section.get();
                    if (section->info->name == "X549") x549 = section.get();
                }
                const nfs::SolidObject* tower = world.FindModel(0x7ED60103u);
                std::printf("refinery tower model 7ED60103: %s, %s, %u triangles\n",
                            tower ? tower->name.c_str() : "missing",
                            x549 ? "library X549 loaded" : "X549 not loaded",
                            tower ? tower->TriangleCount() : 0u);
                const auto countTower = [&](const nfs::LoadedSection* section, const char* label) {
                    if (!section) { std::printf("  %s not loaded\n", label); return; }
                    size_t towerInfo = SIZE_MAX;
                    for (size_t i = 0; i < section->scenery.infos.size(); ++i)
                        if (section->scenery.infos[i].name == "XBu_RefineryB_1b_00") towerInfo = i;
                    size_t named = 0, resolved = 0;
                    for (const nfs::SceneryInstance& instance : section->scenery.instances) {
                        if (instance.infoIndex != towerInfo) continue;
                        ++named;
                        if (world.FindModel(section->scenery.ModelHash(instance))) ++resolved;
                    }
                    std::printf("  %s: %zu instances, %zu objects, %zu infos, alternate %s, "
                                "tower info %zu: %zu instances, %zu resolved\n",
                                label, section->scenery.instances.size(), section->ObjectCount(),
                                section->scenery.infos.size(),
                                section->info && section->info->IsAlternate() ? "yes" : "no",
                                towerInfo, named, resolved);
                };
                countTower(a31, "A31");
                countTower(a431, "A431");
                if (a431) {
                    for (const nfs::SceneryInfo& info : a431->scenery.infos) {
                        const nfs::SolidObject* model = world.FindModel(info.solidHash);
                        std::printf("  A431 info %-32s %08X %s\n", info.name.c_str(), info.solidHash,
                                    model ? model->name.c_str() : "unresolved");
                    }
                }
                if (!tower || !a431) return 15;
            }
            size_t unresolved = 0;
            for (const auto& section : world.Loaded())
                for (const nfs::SceneryInstance& instance : section->scenery.instances)
                    if (!world.FindModel(section->scenery.ModelHash(instance))) ++unresolved;
            std::printf("unresolved scenery instances: %zu\n", unresolved);
            {
                size_t checkedObjects = 0, badObjects = 0;
                double maxObjectError = 0.0;
                for (const auto& section : world.Loaded())
                    for (const nfs::SolidList& list : section->solidLists)
                        for (const nfs::SolidObject& object : list.objects) {
                            if (!object.bounds.Valid() || object.vertices.empty()) continue;
                            Aabb decoded;
                            for (const nfs::SolidVertex& v : object.vertices) decoded.Add(v.position);
                            if (!decoded.Valid()) continue;
                            const Vec3 a = decoded.Center(), b = object.bounds.Center();
                            const double error = std::sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z));
                            ++checkedObjects;
                            if (error > maxObjectError) maxObjectError = error;
                            if (error > 10.0) {
                                ++badObjects;
                                if (badObjects <= 12)
                                    std::printf("  bad object %s: err %.1f header %.1f %.1f %.1f decoded %.1f %.1f %.1f\n",
                                                object.name.c_str(), error,
                                                object.bounds.min.x, object.bounds.min.y, object.bounds.min.z,
                                                decoded.min.x, decoded.min.y, decoded.min.z);
                            }
                        }
                std::printf("object vertex bounds: checked %zu, >10 units %zu, max %.1f\n",
                            checkedObjects, badObjects, maxObjectError);
            }
            {
                size_t checked = 0, bad = 0;
                double maxError = 0.0;
                for (const auto& section : world.Loaded())
                    for (const nfs::SceneryInstance& instance : section->scenery.instances) {
                        const nfs::SolidObject* model = world.FindModel(section->scenery.ModelHash(instance));
                        if (!model || !model->bounds.Valid() || !instance.worldBounds.Valid()) continue;
                        const Aabb box = Transform(model->bounds, instance.transform);
                        const Vec3 a = box.Center(), b = instance.worldBounds.Center();
                        const double error = std::sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z));
                        ++checked;
                        if (error > maxError) maxError = error;
                        if (error > 50.0) ++bad;
                    }
                std::printf("scenery bounds: checked %zu, >50 units %zu, max %.1f\n", checked, bad, maxError);
            }
            {
                std::map<std::string, size_t> formats;
                for (const nfs::Texture* texture : world.AllTextures())
                    formats[nfs::ToString(texture->format)]++;
                for (const auto& [name, count] : formats)
                    std::printf("  texture format %s: %zu\n", name.c_str(), count);
            }
        }
    }

    std::printf("OK\n");
    return 0;
}
