
#include "roads/NfswRoadBake.h"
#include "roads/NfswRoadTemplate.h"

#include "core/FileMap.h"
#include "io/CollisionEdit.h"
#include "io/TextureImport.h"
#include "nfs/World.h"
#include "roads/RoadBake.h"

#include <windows.h>

#include <array>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <map>
#include <set>

namespace roads {
namespace {

std::wstring JournalPath(const nfs::World& world) {
    return world.OverlayFolder() + L"\\world-roads-" + world.Region() + L".frsworld";
}

std::wstring OverlayPath(const nfs::World& world, const NfswMeshOverlay& file) {
    return world.OverlayFolder() + (file.modelsRoot ? L"\\" : L"\\files\\TRACKS\\") + file.file;
}

bool ReadOverlay(const nfs::World& world, NfswMeshOverlay& file) {
    const auto path = OverlayPath(world, file);
    const DWORD attributes = ::GetFileAttributesW(path.c_str());
    if (attributes == INVALID_FILE_ATTRIBUTES) {
        const DWORD error = ::GetLastError();
        file.remove = error == ERROR_FILE_NOT_FOUND || error == ERROR_PATH_NOT_FOUND;
        return file.remove;
    }
    FileMap map;
    if (!map.Open(path)) return false;
    const auto bytes = map.Span();
    file.bytes.assign(bytes.begin(), bytes.end());
    return true;
}

std::vector<u8> FileKey(const std::vector<u8>& bytes) {
    u64 hash = 14695981039346656037ull;
    for (u8 byte : bytes) hash = (hash ^ byte) * 1099511628211ull;
    std::vector<u8> key(sizeof(hash));
    std::memcpy(key.data(), &hash, sizeof(hash));
    return key;
}

bool IsTrackName(const std::string& name) {
    return name.size() > 4 && name.substr(name.size() - 4) == ".BUN" &&
        std::all_of(name.begin(), name.end() - 4, [](char c) {
            return (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_';
        });
}

std::vector<Vec3> SamplePoints(const MeshPiece& piece) {
    const size_t stride = std::max<size_t>(1, piece.VertexCount() / 40);
    std::vector<Vec3> points;
    size_t n = 0;
    for (const auto& group : piece.groups)
        for (const auto& vertex : group.vertices)
            if (n++ % stride == 0) points.push_back(vertex.position);
    return points;
}

using RoadMaps = std::unordered_map<u32, std::array<u32, 5>>;

RoadMaps StockRoadMaps(const nfs::World& world) {
    RoadMaps out;
    for (const auto* model : world.AllModels()) {
        if (model->name.rfind("RD_", 0) != 0) continue;
        for (const auto& material : model->materials) {
            if (!material.nfswRoadSurface || material.textureIndex >= model->textureHashes.size()) continue;
            const u32 diffuse = model->textureHashes[material.textureIndex];
            if (out.count(diffuse) || !world.FindTexture(diffuse)) continue;
            std::array<u32, 5> maps{};
            bool available = true;
            for (size_t i = 0; i < maps.size(); ++i) {
                const u8 slot = material.nfswMapIds[i];
                if (slot == 0xff) continue;
                if (slot >= model->textureHashes.size()) { available = false; break; }
                maps[i] = model->textureHashes[slot];
                if (maps[i] && !world.FindTexture(maps[i])) { available = false; break; }
            }
            if (available && maps[0] == diffuse) out.emplace(diffuse, maps);
        }
    }
    return out;
}

bool PlaceRoadMesh(const nfs::World& world, MeshPiece piece, unsigned depth, u32 fallback,
                   bool stockTextures, const RoadMaps& roadMaps,
                   std::vector<NfswRoadPlacement>& placed, std::wstring& error) {
    if (!piece.VertexCount()) return true;
    const Aabb bounds = piece.Bounds();
    const size_t target = world.SectionForPlacementAlong(SamplePoints(piece));
    if (target == SIZE_MAX) {
        if (!depth) {
            error = L"A road piece is outside the map's streamable areas.";
            return false;
        }
        // Keep every triangle in a section loaded along its full footprint.
        const bool yAxis = bounds.Extent().y > bounds.Extent().x;
        const Vec3 centre = bounds.Center();
        MeshPiece halves[2];
        for (const MeshGroup& group : piece.groups) {
            for (size_t i = 0; i + 2 < group.indices.size(); i += 3) {
                const Vec3 point = (group.vertices[group.indices[i]].position +
                                    group.vertices[group.indices[i + 1]].position +
                                    group.vertices[group.indices[i + 2]].position) * (1.0f / 3.0f);
                const bool high = yAxis ? point.y >= centre.y : point.x >= centre.x;
                auto& half = halves[high].Group(group.texture);
                for (size_t corner = 0; corner < 3; ++corner) {
                    half.indices.push_back((u32)half.vertices.size());
                    half.vertices.push_back(group.vertices[group.indices[i + corner]]);
                }
            }
        }
        return PlaceRoadMesh(world, std::move(halves[0]), depth - 1, fallback, stockTextures, roadMaps, placed, error) &&
               PlaceRoadMesh(world, std::move(halves[1]), depth - 1, fallback, stockTextures, roadMaps, placed, error);
    }

    NfswRoadPlacement road;
    road.section = target;
    road.position = bounds.Center();
    char name[24];
    std::snprintf(name, sizeof(name), "FRS_ROAD_%08X", (u32)placed.size() + 1);
    road.model.name = name;
    road.model.hash = TextureNameHash(name);
    road.model.transform = Mat4::Identity();
    road.model.hasNormals = true;

    for (const auto& group : piece.groups) {
        if (group.indices.empty()) continue;
        std::string textureName = group.texture;
        for (char& c : textureName) c = (char)std::toupper((unsigned char)c);
        const u32 hash = TextureNameHash(textureName);
        const u32 resolved = ResolveWorldRoadTexture(world, textureName, hash, stockTextures);
        const u32 texture = resolved ? resolved : fallback;
        auto& hashes = road.model.textureHashes;
        const auto slotFor = [&](u32 key) {
            const auto slot = std::find(hashes.begin(), hashes.end(), key);
            if (slot != hashes.end()) return (u32)(slot - hashes.begin());
            hashes.push_back(key);
            return (u32)hashes.size() - 1;
        };
        nfs::SolidMaterial material;
        material.textureIndex = slotFor(texture);
        if (const auto stock = roadMaps.find(resolved); stock != roadMaps.end()) {
            material.nfswRoadSurface = true;
            for (size_t i = 0; i < stock->second.size(); ++i)
                if (stock->second[i]) {
                    const u32 slot = slotFor(stock->second[i]);
                    if (slot >= 254) {
                        error = L"A generated World road piece has too many texture slots.";
                        return false;
                    }
                    material.nfswMapIds[i] = (u8)slot;
                }
        }
        material.firstIndex = (u32)road.model.indices.size();
        material.indexCount = (u32)group.indices.size();
        // Emit complete triangle corners so material ranges and vertices
        // agree even when the procedural mesh leaves unused source vertices.
        for (u32 i : group.indices) {
            if (i >= group.vertices.size() || road.model.vertices.size() >= UINT16_MAX - 1) {
                error = L"A generated road piece exceeds the World mesh vertex limit.";
                return false;
            }
            const auto& vertex = group.vertices[i];
            const Vec3 local = vertex.position - road.position;
            road.model.indices.push_back((u16)road.model.vertices.size());
            road.model.vertices.push_back({ local, vertex.normal, vertex.color, vertex.uv });
            material.bounds.Add(local);
            road.model.bounds.Add(local);
        }
        road.model.materials.push_back(material);
    }
    if (!road.model.indices.empty()) placed.push_back(std::move(road));
    return true;
}

bool BuildRoutes(const Network& network, std::vector<nfs::NfswRoadSegmentBuild>& routes, std::wstring& error) {
    std::map<u32, size_t> endpoints;
    size_t count = 0;
    const auto node = [&](u32 id) {
        const auto [entry, added] = endpoints.emplace(id, count);
        if (added) ++count;
        return entry->second;
    };
    for (const auto& road : network.Segments()) {
        if (road.profile.keepGround || (!road.profile.gps && !road.profile.traffic)) continue;
        if (road.profile.gps != road.profile.traffic) {
            error = L"World roads use one route graph. Turn Traffic and GPS both on or both off for this road.";
            return false;
        }
        if (!network.FindNode(road.a) || !network.FindNode(road.b)) {
            error = L"A World road is missing an endpoint.";
            return false;
        }
        const Curve curve = network.CurveOf(road);
        const float length = Length(curve.p1 - curve.p0) + Length(curve.p2 - curve.p1) + Length(curve.p3 - curve.p2);
        if (!std::isfinite(length) || length < 1.0f || length > 10000.0f) {
            error = L"A World road's curve length is out of range.";
            return false;
        }
        const int parts = std::max(1, (int)std::ceil(length / 70.0f));
        size_t a = node(road.a);
        for (int i = 0; i < parts; ++i) {
            const size_t b = i + 1 == parts ? node(road.b) : count++;
            const float t0 = (float)i / parts, t1 = (float)(i + 1) / parts;
            const Vec3 p0 = curve.At(t0), p3 = curve.At(t1);
            const auto handle = [&](float t) {
                const Vec3 d = curve.Tangent(t) * ((t1 - t0) / 3.0f);
                return Length(d) > 0.01f ? d : (p3 - p0) * (1.0f / 3.0f);
            };
            routes.push_back({ a, b, { p0, p0 + handle(t0), p3 - handle(t1), p3 },
                               road.profile.lanesForward, road.profile.lanesBackward,
                               road.profile.laneWidth, road.profile.MedianWidth(), road.profile.worldLaneProfile });
            a = b;
        }
    }
    return true;
}

} // namespace

bool PlanWorldRoadBake(const nfs::World& world, const Network& network,
                       std::vector<NfswMeshOverlay>& files, nfs::NfswRoadBuildReport& report,
                       std::wstring& error) {
    std::vector<nfs::NfswRoadSegmentBuild> routes;
    if (!BuildRoutes(network, routes, error)) return false;

    u32 fallback = 0;
    for (const auto* model : world.AllModels()) {
        for (const auto& material : model->materials) {
            if (!material.nfswRoadSurface || material.textureIndex >= model->textureHashes.size()) continue;
            const auto* texture = world.FindTexture(model->textureHashes[material.textureIndex]);
            if (texture && texture->format == nfs::TextureFormat::Dxt1) fallback = texture->hash;
            if (fallback) break;
        }
        if (fallback) break;
    }
    if (!fallback) {
        error = L"The loaded World sections have no stock road texture.";
        return false;
    }

    std::vector<NfswRoadPlacement> models;
    std::vector<nfs::NfswCollisionPlacement> colliders;
    const RoadMaps roadMaps = StockRoadMaps(world);
    nfs::SolidObject wallModel;
    wallModel.name = "FRS road wall";
    for (auto& piece : network.BuildMesh()) {
        nfs::NfswCollisionPlacement ground, walls{ &wallModel, {} };
        for (const auto& strip : piece.strips)
            if (strip.left.size() >= 2 && strip.left.size() == strip.right.size())
                collision::AppendStrip(strip.left, strip.right, strip.surface, ground.polygons);
        for (const auto& area : piece.areas)
            if (area.outline.size() >= 3)
                collision::AppendGround(area.outline, area.surface, ground.polygons);
        for (const auto& wall : piece.walls) {
            if (wall.points.size() < 2) continue;
            collision::WallOptions options;
            options.kind = collision::WallKind::Vertical;
            options.height = wall.barrier ? 1.5f : wall.height;
            options.sink = 0.1f;
            options.surface = wall.surface;
            collision::AppendWall(wall.points, options, walls.polygons);
        }
        if (!ground.polygons.empty()) colliders.push_back(std::move(ground));
        if (!walls.polygons.empty()) colliders.push_back(std::move(walls));
        if (!PlaceRoadMesh(world, std::move(piece), 6, fallback, network.WorldRoadTextures(), roadMaps, models, error))
            return false;
    }
    return PlanNfswRoadOverlays(world, models, routes, colliders, files, report, error);
}

bool InstallWorldRoadBake(const nfs::World& world, const std::vector<NfswMeshOverlay>& files,
                          std::wstring& error) {
    const auto path = JournalPath(world);
    io::WorldFile journal;
    for (const auto& file : files) {
        NfswMeshOverlay before{ file.file, {}, false, file.modelsRoot };
        if (!ReadOverlay(world, before)) {
            error = L"Could not read the previous " + file.file;
            return false;
        }
        const std::string name(file.file.begin(), file.file.end());
        const auto key = (file.modelsRoot ? "models/" : "tracks/") + name;
        journal.Put("written/" + key, FileKey(file.bytes));
        if (!before.remove) journal.Put("before/" + key, std::move(before.bytes));
    }
    if (!journal.Save(path, error)) return false;
    if (WriteNfswMeshOverlays(files, world.OverlayFolder(), error)) return true;
    ::DeleteFileW(path.c_str());
    return false;
}

bool RemoveWorldRoadBake(const nfs::World& world, WorldRoadBake& previous, std::wstring& error) {
    const auto path = JournalPath(world);
    if (::GetFileAttributesW(path.c_str()) == INVALID_FILE_ATTRIBUTES) {
        const auto why = ::GetLastError();
        if (why == ERROR_FILE_NOT_FOUND || why == ERROR_PATH_NOT_FOUND) return true;
        error = L"Could not read the World road bake record.";
        return false;
    }
    if (!previous.journal.Load(path, error)) return false;

    std::vector<NfswMeshOverlay> restored;
    for (const auto& entry : previous.journal.Names()) {
        if (entry.rfind("written/", 0) != 0) continue;
        const auto key = entry.substr(8);
        const bool modelsRoot = key.rfind("models/", 0) == 0;
        const bool tracks = key.rfind("tracks/", 0) == 0;
        const auto name = key.substr(7);
        const auto collisionFile = NfswBakedRoadCollisionFile(world.Region());
        const bool knownModelFile = name == std::string(collisionFile.begin(), collisionFile.end());
        if ((!modelsRoot && !tracks) || (modelsRoot && !knownModelFile) ||
            (tracks && !IsTrackName(name))) {
            error = L"The World road bake record has an invalid file name.";
            return false;
        }
        NfswMeshOverlay current{ std::wstring(name.begin(), name.end()), {}, false, modelsRoot };
        std::vector<u8> expected;
        if (!ReadOverlay(world, current) || current.remove ||
            !previous.journal.Get(entry, expected) || FileKey(current.bytes) != expected) {
            error = L"An overlay used by the last bake has changed: " + current.file +
                    L". Keep those edits before replacing the bake.";
            return false;
        }
        NfswMeshOverlay before{ current.file, {}, false, modelsRoot };
        before.remove = !previous.journal.Get("before/" + key, before.bytes);
        restored.push_back(std::move(before));
        previous.installed.push_back(std::move(current));
    }
    if (restored.empty()) {
        error = L"The World road bake record is empty.";
        return false;
    }
    if (!WriteNfswMeshOverlays(restored, world.OverlayFolder(), error)) return false;
    if (::DeleteFileW(path.c_str())) return true;
    error = L"Could not remove the World road bake record.";
    WriteNfswMeshOverlays(previous.installed, world.OverlayFolder(), error);
    return false;
}

bool ReplayWorldRoadBake(const nfs::World& world, const WorldRoadBake& previous, std::wstring& error) {
    if (previous.installed.empty()) return true;
    return previous.journal.Save(JournalPath(world), error) &&
           WriteNfswMeshOverlays(previous.installed, world.OverlayFolder(), error);
}

} // namespace roads
