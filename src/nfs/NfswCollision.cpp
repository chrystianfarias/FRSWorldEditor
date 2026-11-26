#include "nfs/NfswCollision.h"

#include "nfs/ChunkReader.h"
#include "nfs/Collision.h"
#include "nfs/SolidList.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstring>
#include <map>
#include <set>

namespace nfs {

namespace {

constexpr u32 kWorld = 0x3B800;
constexpr u32 kCollisionPack = 0x3B801;
constexpr u32 kCollisionCells = 0x3B802;
constexpr u32 kCarp = 0x43415250;
constexpr u32 kArticles = 0x41727469;
constexpr u32 kCollisionData = 0x43446174;
constexpr u32 kGrid = 0x43477264;
constexpr u32 kCells = 0x4347636E;
constexpr u32 kName = 0x4E616D65;
constexpr u32 kMesh = 0x63610000;
constexpr u32 kInstances = 0x63690000;
constexpr char kGeneratedName[] = "FRSPlacementCollision";
constexpr char kGeneratedVersion[] = "v3:";
constexpr size_t kFacesPerMesh = 512;
constexpr size_t kBoundaryRecordSize = 32;
constexpr size_t kRoadNodeBytes = 32;
constexpr size_t kRoadSegmentBytes = 24;

// Hash only the positions and endpoint indices; the runtime may update other
// fields in its road tables after the layout has been read.
u64 RoadNetworkKey(ByteSpan nodes, ByteSpan segments) {
    u64 key = 14695981039346656037ull;
    const auto add = [&](ByteSpan bytes, size_t stride, size_t used) {
        for (size_t at = 0; at < bytes.size; at += stride)
            for (size_t i = 0; i < used; ++i) {
                key ^= bytes.data[at + i];
                key *= 1099511628211ull;
            }
        key ^= bytes.size / stride;
        key *= 1099511628211ull;
    };
    add(nodes, kRoadNodeBytes, 12);
    add(segments, kRoadSegmentBytes, 4);
    return key;
}

template<class T>
void Put(std::vector<u8>& out, size_t at, const T& value) {
    std::memcpy(out.data() + at, &value, sizeof(value));
}

size_t Align16(size_t at) { return (at + 15) & ~size_t(15); }

ByteSpan Span(const std::vector<u8>& bytes) { return { bytes.data(), bytes.size() }; }

ByteSpan AlignedPayload(const Chunk& chunk) {
    const size_t at = (size_t)chunk.headerOffset + 8;
    return chunk.raw.sub(Align16(at) - at, chunk.raw.size);
}

struct Article {
    size_t row = 0;
    size_t offset = 0;
    ByteSpan bytes;
};

// PRAC group offsets are in units of 16. Leaf offsets are byte offsets
// from their own table row.
size_t FindRow(ByteSpan carp, size_t group, u32 tag) {
    const u32 flags = carp.read<u32>(group + 4);
    if (!(flags & 2)) return 0;
    const size_t count = (flags >> 5) + (size_t)carp.read<u32>(group + 8);
    const size_t first = group + (size_t)carp.read<u32>(group + 12) * 16;
    if (first > carp.size || count > (carp.size - first) / 16) return 0;
    for (size_t i = 0; i < count; ++i)
        if (carp.read<u32>(first + 16 * i) == tag) return first + 16 * i;
    return 0;
}

Article FindArticle(ByteSpan carp, u32 group, u32 tag) {
    if (carp.read<u32>(0) != kCarp) return {};
    const size_t parent = FindRow(carp, 0, group);
    if (!parent) return {};
    const size_t row = FindRow(carp, parent, tag);
    if (!row || !(carp.read<u32>(row + 4) & 2)) return {};
    const size_t offset = row + carp.read<u32>(row + 12);
    const size_t size = carp.read<u32>(row + 4) >> 8;
    if (offset > carp.size || size > carp.size - offset) return {};
    return { row, offset, carp.sub(offset, size) };
}

enum class GeneratedFormat { Legacy, CenteredStrips, WorldToLocal };

bool GeneratedPack(ByteSpan carp, size_t& walls, GeneratedFormat* format = nullptr) {
    const ByteSpan name = FindArticle(carp, kArticles, kName).bytes;
    const size_t prefix = sizeof(kGeneratedName) - 1;
    if (name.size < prefix + 1 || name.data[name.size - 1] != 0 ||
        std::memcmp(name.data, kGeneratedName, prefix) != 0) return false;
    walls = 0;
    if (format) *format = GeneratedFormat::Legacy;
    if (name.size == prefix + 1) return true;
    if (name.data[prefix] != ':') return false;
    const char* begin = reinterpret_cast<const char*>(name.data + prefix + 1);
    const char* end = reinterpret_cast<const char*>(name.data + name.size - 1);
    if (end - begin >= 3 && begin[0] == 'v' && begin[2] == ':') {
        if (begin[1] != '2' && begin[1] != '3') return false;
        if (format) *format = begin[1] == '3' ? GeneratedFormat::WorldToLocal
                                               : GeneratedFormat::CenteredStrips;
        begin += 3;
    }
    const auto count = std::from_chars(begin, end, walls);
    return count.ec == std::errc{} && count.ptr == end;
}

struct Grid {
    float left = 0;
    float bottom = 0;
    float cellSize = 0;
    u32 width = 0;
    u32 height = 0;
};

bool ReadGrid(ByteSpan layout, Grid& grid) {
    ChunkReader reader(layout, 0);
    Chunk chunk;
    while (reader.Next(chunk)) {
        if (chunk.id != kWorld) continue;
        const ByteSpan data = FindArticle(AlignedPayload(chunk), kCollisionData, kGrid).bytes;
        if (data.size < 32) return false;
        grid.left = data.read<float>(0);
        grid.bottom = data.read<float>(8);
        grid.cellSize = data.read<float>(16);
        grid.width = data.read<u32>(28);
        grid.height = data.read<u32>(24);
        return std::isfinite(grid.left) && std::isfinite(grid.bottom) &&
               std::isfinite(grid.cellSize) && grid.cellSize > 0 &&
               grid.width && grid.height && (u64)grid.width * grid.height <= 65536;
    }
    return false;
}

struct Face {
    Vec3 corners[3];
    bool wallBoundary = false;
};

struct CollisionMesh {
    std::vector<u8> faces;
    std::array<u8, 64> instance{};
    Aabb bounds;
};

struct Strip {
    ByteSpan sphere, vertices;
};

bool ReadStrips(ByteSpan mesh, std::vector<Strip>& strips) {
    const size_t count = mesh.read<u16>(0);
    const size_t end = 16 + (size_t)mesh.read<u16>(2);
    if (mesh.size < 16 || 16 + count * 16 > end || end > mesh.size) return false;
    strips.clear();
    for (size_t i = 0; i < count; ++i) {
        const ByteSpan sphere = mesh.sub(16 + i * 16, 16);
        const size_t at = 16 + (size_t)sphere.read<u16>(14);
        const size_t vertices = mesh.read<u16>(at + 6);
        if (vertices < 3 || at < 16 + count * 16 || at + vertices * 8 > end) return false;
        strips.push_back({ sphere, mesh.sub(at, vertices * 8) });
    }
    return true;
}

Vec3 FloatPoint(ByteSpan bytes, size_t offset = 0) {
    return { bytes.read<float>(offset), bytes.read<float>(offset + 4), bytes.read<float>(offset + 8) };
}

Vec3 FromPhysics(const Vec3& point) {
    return { point.z, -point.x, point.y };
}

Vec3 Rotate(const std::array<float, 4>& q, const Vec3& point) {
    const Vec3 axis{ q[0], q[1], q[2] };
    const Vec3 cross = Cross(axis, point) * 2.0f;
    return point + cross * q[3] + Cross(axis, cross);
}

Vec3 StripPoint(const Strip& strip, size_t index, bool centered = true) {
    const ByteSpan vertex = strip.vertices.sub(index * 8, 8);
    const Vec3 relative{ vertex.read<i16>(0) / 128.0f, vertex.read<i16>(2) / 128.0f,
                         vertex.read<i16>(4) / 128.0f };
    // Each strip is quantized around its sphere's center in CI space.
    return centered ? FloatPoint(strip.sphere) + relative : relative;
}

Vec3 InstancePoint(ByteSpan instance, const Vec3& local) {
    // CI maps the game's world ray into CA space. Invert its rigid transform
    // before converting from the game's Y-up coordinates to the editor.
    const Vec3 right = FloatPoint(instance);
    const Vec3 forward = FloatPoint(instance, 32);
    const Vec3 up = instance.read<u8>(0x12) & 3 ? Cross(forward, right) : Vec3{ 0, 1, 0 };
    const Vec3 p = local - FloatPoint(instance, 48);
    return FromPhysics({ Dot(right, p), Dot(up, p), Dot(forward, p) });
}

std::array<size_t, 3> StripTriangle(size_t triangle) {
    return { triangle + (triangle & 1), triangle + ((triangle & 1) ^ 1), triangle + 2 };
}

Vec3 FaceNormal(const CollisionPolygon& polygon) {
    return Cross(polygon.corners[1] - polygon.corners[0], polygon.corners[2] - polygon.corners[0]);
}

void ClassifyFace(CollisionPolygon& polygon) {
    const Vec3 normal = FaceNormal(polygon);
    const bool wall = std::abs(normal.z) < Length(normal) * 0.70710678f;
    polygon.attributes[2] = wall ? kSurfaceSolidWall : kSurfaceRoad;
    polygon.attributes[3] = (wall ? 0x02 : 0) | (polygon.quad ? 0x10 : 0);
    polygon.SyncFromAttributes();
}

CollisionPolygon StockFace(ByteSpan instance, const Strip& strip, u32 owner, size_t ci,
                           size_t group, size_t triangle) {
    CollisionPolygon polygon;
    const auto corners = StripTriangle(triangle);
    for (size_t i = 0; i < corners.size(); ++i)
        polygon.corners[i] = InstancePoint(instance, StripPoint(strip, corners[i]));
    polygon.corners[3] = polygon.corners[2];
    ClassifyFace(polygon);
    polygon.nfswCollisionSource = ((u64)owner << 48) | ((u64)ci << 32) |
                                  ((u64)group << 16) | (triangle + 1);
    return polygon;
}

bool SelectableFace(const CollisionPolygon& polygon) {
    const float length = Length(FaceNormal(polygon));
    return std::isfinite(length) && length > 0.0001f;
}

ByteSpan BoundaryRecords(ByteSpan mesh) {
    const size_t at = 16 + (size_t)mesh.read<u16>(2);
    const size_t length = (size_t)mesh.read<u16>(4) * kBoundaryRecordSize;
    if (length != mesh.read<u16>(6) || at > mesh.size || length > mesh.size - at) return {};
    return mesh.sub(at, length);
}

CollisionPolygon StockBoundary(ByteSpan instance, ByteSpan boundary, u32 owner, size_t ci,
                               size_t index) {
    // The game intersects a segment in the XZ plane and checks the hit
    // against both Y limits. The stored points are opposite corners.
    const Vec3 a = FloatPoint(boundary), b = FloatPoint(boundary, 16);
    const float bottom = std::min(a.y, b.y), top = std::max(a.y, b.y);
    CollisionPolygon polygon;
    polygon.quad = true;
    polygon.corners[0] = InstancePoint(instance, { a.x, bottom, a.z });
    polygon.corners[1] = InstancePoint(instance, { a.x, top, a.z });
    polygon.corners[2] = InstancePoint(instance, { b.x, top, b.z });
    polygon.corners[3] = InstancePoint(instance, { b.x, bottom, b.z });
    polygon.attributes[2] = kSurfaceSolidWall;
    polygon.attributes[3] = 0x12;
    polygon.SyncFromAttributes();
    polygon.nfswCollisionSource = ((u64)owner << 48) | ((u64)ci << 32) |
                                  CollisionPolygon::kNfswBoundarySource | (index + 1);
    return polygon;
}

// A shared CA cannot be changed through one of its instances: doing so
// would edit the other instance too. Group-owned CIs keep their stock data.
template<class Visit>
bool VisitNativeMeshes(ByteSpan carp, u32 owner, Visit visit) {
    const ByteSpan instances = FindArticle(carp, kArticles, kInstances).bytes;
    std::map<u16, size_t> references;
    for (size_t ci = 0; ci + 64 <= instances.size; ci += 64)
        ++references[instances.read<u16>(ci + 0x1A)];

    std::vector<Strip> strips;
    for (size_t ci = 0; ci + 64 <= instances.size; ci += 64) {
        const ByteSpan instance = instances.sub(ci, 64);
        const u16 index = instance.read<u16>(0x1A);
        if (instance.read<u16>(0x18) || references[index] != 1) continue;
        const Article article = FindArticle(carp, kArticles, kMesh + index);
        if (article.bytes.empty() || !ReadStrips(article.bytes, strips)) continue;
        if (!visit(article, instance, owner, ci / 64, strips)) return false;
    }
    return true;
}

Vec3 ToCollision(const Vec3& world) {
    return { -world.y, world.z, world.x };
}

Vec3 SnapCollision(const Vec3& point) {
    return { std::round(point.x * 128) / 128, std::round(point.y * 128) / 128,
             std::round(point.z * 128) / 128 };
}

bool Quantize(const Vec3& corner, i16 (&out)[3]) {
    const float values[] = { corner.x, corner.y, corner.z };
    for (size_t i = 0; i < 3; ++i) {
        const float value = std::round(values[i] * 128.0f);
        if (!std::isfinite(value) || value < -32768 || value > 32767) return false;
        out[i] = (i16)value;
    }
    return true;
}

bool BuildMesh(const std::vector<Face>& faces, CollisionMesh& out, u32 surface = kSolidSurface) {
    Aabb bounds;
    for (const Face& face : faces)
        for (const Vec3& corner : face.corners) bounds.Add(corner);
    const Vec3 center = SnapCollision(bounds.Center());

    // The CI takes the ray to the mesh origin. Rotation and scale are baked
    // into the faces so nonuniformly scaled objects still fit.
    const Vec3 extent{ std::max(bounds.max.x - center.x, center.x - bounds.min.x),
                       std::max(bounds.max.y - center.y, center.y - bounds.min.y),
                       std::max(bounds.max.z - center.z, center.z - bounds.min.z) };
    const float radius = Length(extent) + 0.125f;
    const float vertical = extent.y + 0.125f;
    const float instance[] = { 1, 0, 0, radius, 0, vertical, 0, 0,
                               0, 0, 1, radius, -center.x, -center.y, -center.z, radius };
    std::memcpy(out.instance.data(), instance, sizeof(instance));
    const u16 stamp = 0xFFFF;
    std::memcpy(out.instance.data() + 0x10, &stamp, sizeof(stamp));
    out.bounds = bounds;

    // The height query reads the strips. A rectangular wall belongs only in
    // the boundary list, so it cannot also be picked up as wheel support.
    const size_t wallCount = std::count_if(faces.begin(), faces.end(),
        [](const Face& face) { return face.wallBoundary; });
    const size_t stripCount = faces.size() - wallCount;
    const size_t boundaryAt = 16 + stripCount * (16 + 24);
    const size_t wallBytes = wallCount * kBoundaryRecordSize;
    out.faces.resize(boundaryAt + wallBytes + 4, 0);
    Put(out.faces, 0, (u16)stripCount);
    Put(out.faces, 2, (u16)(boundaryAt - 16));
    Put(out.faces, 4, (u16)wallCount);
    Put(out.faces, 6, (u16)wallBytes);
    out.faces[9] = 1;
    Put(out.faces, 0x0A, (u16)4);
    Put(out.faces, out.faces.size() - 4, surface);
    size_t boundary = boundaryAt;
    size_t strip = 0;
    for (const Face& face : faces) {
        const auto& corners = face.corners;
        if (face.wallBoundary) {
            const Vec3 a = corners[0] - center, b = corners[2] - center;
            const float length = std::hypot(b.x - a.x, b.z - a.z);
            if (length <= 0) return false;
            Put(out.faces, boundary, a);
            Put(out.faces, boundary + 12, (u32)0x00000200); // surface 0, ordinary boundary
            Put(out.faces, boundary + 16, b);
            Put(out.faces, boundary + 28, 1.0f / length);
            boundary += kBoundaryRecordSize;
            continue;
        }
        const Vec3 mid = SnapCollision((corners[0] + corners[1] + corners[2]) * (1.0f / 3.0f)) - center;
        const size_t vertices = 16 + stripCount * 16 + strip * 24;
        float range = 0;
        for (size_t v = 0; v < 3; ++v) {
            i16 xyz[3];
            if (!Quantize(corners[v] - center - mid, xyz)) return false;
            range = std::max(range, Length({ xyz[0] / 128.0f, xyz[1] / 128.0f, xyz[2] / 128.0f }));
            std::memcpy(out.faces.data() + vertices + v * 8, xyz, sizeof(xyz));
        }
        Put(out.faces, vertices + 6, (u16)3);
        const Vec3 normal = Cross(corners[1] - corners[0], corners[2] - corners[0]);
        Put(out.faces, vertices + 14, (u16)(normal.y < 0));
        const size_t descriptor = 16 + strip * 16;
        Put(out.faces, descriptor, mid.x);
        Put(out.faces, descriptor + 4, mid.y);
        Put(out.faces, descriptor + 8, mid.z);
        Put(out.faces, descriptor + 12, (u16)std::ceil((range + 0.125f) * 16));
        Put(out.faces, descriptor + 14, (u16)(vertices - 16));
        ++strip;
    }
    return true;
}

bool HasNativeBoundary(const CollisionPolygon& polygon, const Vec3 (&p)[4]) {
    return polygon.quad && polygon.Vertical() &&
           p[0].x == p[1].x && p[0].z == p[1].z && p[2].x == p[3].x && p[2].z == p[3].z &&
           p[0].y < p[1].y && p[0].y == p[3].y && p[1].y == p[2].y &&
           (p[0].x != p[2].x || p[0].z != p[2].z);
}

bool BuildMeshes(const std::vector<NfswCollisionPlacement>& placements,
                 std::vector<CollisionMesh>& meshes, std::string& error) {
    std::vector<Face> faces;
    for (const NfswCollisionPlacement& placement : placements) {
        const SolidObject* model = placement.model;
        const u32 surface = !model || model->IsTerrain() || model->HasNfswRoadSurface() ? kRoadSurface : kSolidSurface;
        Aabb bounds;
        error = model ? "collision geometry could not be encoded for " + model->name
                      : "road collision geometry could not be encoded";
        const auto emit = [&]() {
            if (faces.empty()) return true;
            if (meshes.size() >= 0x7FFF) return false;
            CollisionMesh mesh;
            if (!BuildMesh(faces, mesh, surface)) return false;
            meshes.push_back(std::move(mesh));
            faces.clear();
            bounds = {};
            return true;
        };
        const auto fits = [](const Aabb& box) {
            const Vec3 span = box.max - box.min;
            return std::max({ span.x, span.y, span.z }) < 380.0f;
        };
        const auto append = [&](auto&& split, const Face& face, unsigned depth) -> bool {
            const Vec3 n = Cross(face.corners[1] - face.corners[0],
                                 face.corners[2] - face.corners[0]);
            if (Length(n) < 0.0001f) return true;
            Aabb extent;
            for (const Vec3& vertex : face.corners) extent.Add(vertex);
            if (!face.wallBoundary && !fits(extent)) {
                if (!depth) return false;
                size_t edge = 0;
                for (size_t i = 1; i < 3; ++i)
                    if (Length(face.corners[i] - face.corners[(i + 1) % 3]) >
                        Length(face.corners[edge] - face.corners[(edge + 1) % 3])) edge = i;
                const size_t next = (edge + 1) % 3;
                const Vec3 middle = SnapCollision((face.corners[edge] + face.corners[next]) * 0.5f);
                Face a = face, b = face;
                a.corners[next] = middle;
                b.corners[edge] = middle;
                return split(split, a, depth - 1) && split(split, b, depth - 1);
            }

            Aabb combined = extent;
            if (bounds.Valid()) {
                combined.Add(bounds.min);
                combined.Add(bounds.max);
            }
            if (!fits(combined) || faces.size() == kFacesPerMesh) {
                if (!emit()) return false;
                combined = extent;
            }
            bounds = combined;
            faces.push_back(face);
            return true;
        };
        for (const CollisionPolygon& polygon : placement.polygons) {
            Vec3 p[4];
            for (size_t i = 0; i < 4; ++i) {
                const Vec3& point = polygon.corners[i];
                if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                    return false;
                p[i] = SnapCollision(ToCollision(point));
            }
            const bool boundary = HasNativeBoundary(polygon, p);
            if (!append(append, { { p[0], p[1], p[2] }, boundary }, 16) ||
                (!boundary && polygon.quad && !append(append, { { p[0], p[2], p[3] } }, 16))) return false;
        }
        if (!emit()) return false;
    }
    return true;
}

bool AddWalls(const std::vector<CollisionPolygon>& walls, std::vector<CollisionMesh>& meshes) {
    if (walls.size() + meshes.size() >= 0x7FFF) return false;
    for (const CollisionPolygon& wall : walls) {
        if (wall.barrier) return false;
        Vec3 corners[4];
        for (size_t i = 0; i < 4; ++i)
            corners[i] = SnapCollision(ToCollision(wall.corners[i]));
        const bool boundary = HasNativeBoundary(wall, corners);
        std::vector<Face> faces{ { { corners[0], corners[1], corners[2] }, boundary } };
        if (!boundary && wall.quad) faces.push_back({ { corners[0], corners[2], corners[3] } });
        CollisionMesh mesh;
        if (!BuildMesh(faces, mesh, wall.nfswSurface ? wall.nfswSurface
                                                     : wall.Vertical() ? kSolidSurface : kRoadSurface))
            return false;
        meshes.push_back(std::move(mesh));
    }
    return true;
}

void WriteRow(std::vector<u8>& out, size_t row, u32 tag, u32 flags, u32 count, u32 offset) {
    Put(out, row, tag);
    Put(out, row + 4, flags);
    Put(out, row + 8, count);
    Put(out, row + 12, offset);
}

std::vector<u8> BuildCarp(const std::vector<CollisionMesh>& meshes, size_t walls) {
    const size_t count = meshes.size() + 2;
    std::vector<u8> carp(32 + count * 16, 0);
    WriteRow(carp, 0, kCarp, 0x2A, 0, 1);
    WriteRow(carp, 16, kArticles, 0x12, (u32)count, 1);

    const size_t name = carp.size();
    const std::string label = std::string(kGeneratedName) + ":" +
                              kGeneratedVersion + std::to_string(walls);
    carp.insert(carp.end(), label.c_str(), label.c_str() + label.size() + 1);
    carp.resize(Align16(carp.size()), 0xAA);
    WriteRow(carp, 32, kName, ((u32)(label.size() + 1) << 8) | 2, 0, (u32)(name - 32));
    for (size_t i = 0; i < meshes.size(); ++i) {
        const size_t row = 48 + i * 16;
        const size_t start = carp.size();
        const auto& faces = meshes[i].faces;
        carp.insert(carp.end(), faces.begin(), faces.end());
        carp.resize(Align16(carp.size()), 0);
        Put(carp, start + 12, (u16)(0x8000 + i));
        WriteRow(carp, row, kMesh + (u32)i, ((u32)(carp.size() - start) << 8) | 3u,
                 1, (u32)(start - row));
    }
    const size_t row = 48 + meshes.size() * 16;
    WriteRow(carp, row, kInstances, ((u32)(meshes.size() * 64) << 8) | 3u,
             (u32)meshes.size(), (u32)(carp.size() - row));
    for (size_t i = 0; i < meshes.size(); ++i) {
        const size_t at = carp.size();
        carp.insert(carp.end(), meshes[i].instance.begin(), meshes[i].instance.end());
        Put(carp, at + 0x1A, (u16)i);
    }
    return carp;
}

// GCgn uses a sparse 96 m grid. Static geometry is group 0 of each cell,
// while the other three groups index separate game data.
bool ReferenceMeshes(const Grid& grid, const std::vector<CollisionMesh>& meshes, u32 owner,
                     std::map<u16, std::vector<u32>>& cells) {
    const auto cell = [&](float value, float origin, u32 dimension) {
        const float index = std::floor((value - origin) / grid.cellSize);
        return (u32)std::clamp(index, 0.0f, (float)(dimension - 1));
    };
    for (size_t i = 0; i < meshes.size(); ++i) {
        const Aabb& box = meshes[i].bounds;
        const u32 left = cell(box.min.x - 0.25f, grid.left, grid.width);
        const u32 right = cell(box.max.x + 0.25f, grid.left, grid.width);
        const u32 bottom = cell(box.min.z - 0.25f, grid.bottom, grid.height);
        const u32 top = cell(box.max.z + 0.25f, grid.bottom, grid.height);
        for (u32 y = bottom; y <= top; ++y)
            for (u32 x = left; x <= right; ++x) {
                auto& refs = cells[(u16)(y * grid.width + x)];
                if (refs.size() == 255) return false;
                refs.push_back((owner << 16) | (u32)i);
            }
    }
    return true;
}

bool BuildGrid(ByteSpan carp, u16 gridOwner, u16 oldOwner,
               std::map<u16, std::vector<u32>>& additions, std::vector<u8>& out) {
    const Article cells = FindArticle(carp, kArticles, kCells);
    const Article name = FindArticle(carp, kArticles, kName);
    if (cells.bytes.empty() || name.bytes.empty() || name.offset < cells.offset + cells.bytes.size)
        return false;

    out.assign(carp.data, carp.data + cells.offset);
    const size_t count = carp.read<u32>(cells.row + 8);
    size_t read = 0;
    u32 written = 0;
    for (size_t i = 0; i < count; ++i) {
        const ByteSpan record = cells.bytes.sub(read, cells.bytes.size);
        if (record.size < 20) return false;
        const u16 cell = record.read<u16>(4);
        const u16 owner = record.read<u16>(6);
        const size_t first = record.read<u8>(8);
        const size_t total = first + record.read<u8>(9) + record.read<u8>(10) + record.read<u8>(11);
        const size_t length = 20 + total * 4;
        if (length > record.size || record.read<u16>(12) != 0) return false;
        read += length;

        std::vector<u32> references;
        for (size_t j = 0; j < first; ++j) {
            const u32 ref = record.read<u32>(20 + j * 4);
            if (!oldOwner || (ref >> 16) != oldOwner) references.push_back(ref);
        }
        const auto found = owner == gridOwner ? additions.find(cell) : additions.end();
        if (found != additions.end()) {
            references.insert(references.end(), found->second.begin(), found->second.end());
            additions.erase(found);
        }
        if (references.empty() && first && first == total) continue;
        if (references.size() > 255) return false;

        const size_t at = out.size();
        out.insert(out.end(), record.data, record.data + 20);
        if (!references.empty()) {
            const u8* bytes = reinterpret_cast<const u8*>(references.data());
            out.insert(out.end(), bytes, bytes + references.size() * 4);
        }
        out.insert(out.end(), record.data + 20 + first * 4, record.data + length);
        out[at + 8] = (u8)references.size();
        for (size_t offset = 14; offset <= 18; offset += 2) {
            const u16 value = record.read<u16>(offset);
            if (value < first * 4) return false;
            Put(out, at + offset, (u16)(value - first * 4 + references.size() * 4));
        }
        ++written;
    }
    if (read != cells.bytes.size) return false;

    for (const auto& [cell, refs] : additions) {
        const size_t at = out.size();
        const u16 size = (u16)(refs.size() * 4);
        out.resize(at + 20 + size, 0);
        Put(out, at + 4, cell);
        Put(out, at + 6, gridOwner);
        out[at + 8] = (u8)refs.size();
        for (size_t offset = 14; offset <= 18; offset += 2) Put(out, at + offset, size);
        std::memcpy(out.data() + at + 20, refs.data(), size);
        ++written;
    }
    const size_t bytes = out.size() - cells.offset;
    if (bytes > 0xFFFFFF) return false;
    out.resize(Align16(out.size()), 0xAA);
    const size_t tail = out.size();
    size_t sourceTail = cells.offset + cells.bytes.size;
    const size_t alignedTail = Align16(sourceTail);
    // The earlier bake may already have aligned this gap. Rebuild its
    // padding once, keeping the name and any other trailing articles.
    if (alignedTail <= name.offset &&
        std::all_of(carp.data + sourceTail, carp.data + alignedTail, [](u8 v) { return v == 0xAA; }))
        sourceTail = alignedTail;
    out.insert(out.end(), carp.data + sourceTail, carp.data + carp.size);
    const u32 flags = (u32)(bytes << 8) | (carp.read<u32>(cells.row + 4) & 0xFFu);
    Put(out, cells.row + 4, flags);
    Put(out, cells.row + 8, written);
    const size_t nameAt = tail + name.offset - sourceTail;
    Put(out, name.row + 12, (u32)(nameAt - name.row));
    return true;
}

void AppendChunk(std::vector<u8>& section, size_t fileOffset, u32 tag, u16 owner, ByteSpan carp) {
    const size_t at = section.size();
    const size_t pad = Align16(fileOffset + at + 8) - (fileOffset + at + 8);
    section.resize(at + 8 + pad, 0x11);
    Put(section, at, tag);
    Put(section, at + 4, (u32)(pad + 16 + carp.size));
    const size_t payload = section.size();
    section.resize(payload + 16, 0);
    Put(section, payload, (u32)carp.size);
    Put(section, payload + 4, (u32)owner);
    section.insert(section.end(), carp.data, carp.data + carp.size);
}

using NativeFaces = std::map<u64, const CollisionPolygon*>;

bool PatchNativeMesh(ByteSpan mesh, ByteSpan instance, u32 owner, size_t ci,
                     const std::vector<Strip>& strips, const NativeFaces& edited,
                     std::vector<CollisionPolygon>& moved,
                     std::vector<u8>& rebuilt) {
    std::vector<u8> spheres, vertices;
    bool changed = false;
    for (size_t group = 0; group < strips.size(); ++group) {
        const Strip& strip = strips[group];
        std::vector<size_t> keep;
        const size_t triangles = strip.vertices.size / 8 - 2;
        for (size_t i = 0; i < triangles; ++i) {
            const CollisionPolygon original = StockFace(instance, strip, owner, ci, group, i);
            const auto found = edited.find(original.nfswCollisionSource);
            if (!SelectableFace(original) || (found != edited.end() &&
                std::memcmp(original.corners, found->second->corners, sizeof(original.corners)) == 0)) {
                keep.push_back(i);
                continue;
            }
            if (found != edited.end()) moved.push_back(*found->second);
        }

        const auto addSphere = [&] {
            const size_t at = spheres.size();
            spheres.insert(spheres.end(), strip.sphere.begin(), strip.sphere.end());
            Put(spheres, at + 14, (u16)vertices.size());
        };
        if (keep.size() == triangles) {
            if (vertices.size() > UINT16_MAX) return false;
            addSphere();
            vertices.insert(vertices.end(), strip.vertices.begin(), strip.vertices.end());
            continue;
        }
        changed = true;
        for (size_t triangle : keep) {
            if (vertices.size() > UINT16_MAX) return false;
            addSphere();
            const size_t at = vertices.size();
            for (size_t corner : StripTriangle(triangle)) {
                const ByteSpan vertex = strip.vertices.sub(corner * 8, 8);
                vertices.insert(vertices.end(), vertex.begin(), vertex.end());
            }
            Put(vertices, at + 6, (u16)3);
            Put(vertices, at + 14, strip.vertices.read<u16>(14));
        }
    }

    const ByteSpan records = BoundaryRecords(mesh);
    std::vector<u8> boundaries;
    boundaries.reserve(records.size);
    for (size_t offset = 0; offset < records.size; offset += kBoundaryRecordSize) {
        const ByteSpan record = records.sub(offset, kBoundaryRecordSize);
        const CollisionPolygon original = StockBoundary(instance, record, owner, ci,
                                                        offset / kBoundaryRecordSize);
        const auto found = edited.find(original.nfswCollisionSource);
        if (!SelectableFace(original) || (found != edited.end() &&
            std::memcmp(original.corners, found->second->corners, sizeof(original.corners)) == 0)) {
            boundaries.insert(boundaries.end(), record.begin(), record.end());
            continue;
        }
        if (found != edited.end()) moved.push_back(*found->second);
        changed = true;
    }
    if (!changed) return true;
    if (spheres.size() + vertices.size() > UINT16_MAX) return false;
    for (size_t at = 14; at < spheres.size(); at += 16) {
        const u16 old = Span(spheres).read<u16>(at);
        Put(spheres, at, (u16)(old + spheres.size()));
    }
    rebuilt.assign(mesh.data, mesh.data + 16);
    Put(rebuilt, 0, (u16)(spheres.size() / 16));
    Put(rebuilt, 2, (u16)(spheres.size() + vertices.size()));
    if (!records.empty()) {
        Put(rebuilt, 4, (u16)(boundaries.size() / kBoundaryRecordSize));
        Put(rebuilt, 6, (u16)boundaries.size());
    }
    rebuilt.insert(rebuilt.end(), spheres.begin(), spheres.end());
    rebuilt.insert(rebuilt.end(), vertices.begin(), vertices.end());
    rebuilt.insert(rebuilt.end(), boundaries.begin(), boundaries.end());
    const size_t tail = 16 + mesh.read<u16>(2) + records.size;
    rebuilt.insert(rebuilt.end(), mesh.data + tail, mesh.end());
    rebuilt.resize(Align16(rebuilt.size()), 0);
    return true;
}

bool PatchNativePack(ByteSpan carp, u32 owner, const NativeFaces& edited,
                     std::vector<CollisionPolygon>& moved, std::vector<u8>& out) {
    std::map<size_t, std::pair<Article, std::vector<u8>>> patches;
    if (!VisitNativeMeshes(carp, owner, [&](Article article, ByteSpan instance, u32 number,
                                          size_t ci, const std::vector<Strip>& strips) {
        std::vector<u8> bytes;
        if (!PatchNativeMesh(article.bytes, instance, number, ci, strips, edited, moved, bytes))
            return false;
        if (!bytes.empty()) patches.emplace(article.offset, std::make_pair(article, std::move(bytes)));
        return true;
    })) return false;
    if (patches.empty()) return true;

    const size_t group = FindRow(carp, 0, kArticles);
    const size_t first = group + (size_t)carp.read<u32>(group + 12) * 16;
    const size_t count = (carp.read<u32>(group + 4) >> 5) + carp.read<u32>(group + 8);
    size_t cursor = 0;
    for (const auto& [offset, patch] : patches) {
        if (offset < cursor || offset < first + count * 16) return false;
        out.insert(out.end(), carp.data + cursor, carp.data + offset);
        out.insert(out.end(), patch.second.begin(), patch.second.end());
        cursor = offset + patch.first.bytes.size;
    }
    out.insert(out.end(), carp.data + cursor, carp.end());

    for (size_t i = 0; i < count; ++i) {
        const size_t row = first + i * 16;
        const u32 flags = carp.read<u32>(row + 4);
        if (!(flags & 2)) continue;
        const size_t initial = row + carp.read<u32>(row + 12);
        i64 delta = 0;
        for (const auto& [offset, patch] : patches) {
            if (offset >= initial) break;
            delta += (i64)patch.second.size() - (i64)patch.first.bytes.size;
        }
        Put(out, row + 12, (u32)((i64)initial + delta - (i64)row));
        const auto patched = patches.find(initial);
        if (patched != patches.end())
            Put(out, row + 4, ((u32)patched->second.second.size() << 8) | (flags & 0xFFu));
    }
    const size_t tail = (carp.size % 128 + 128 - out.size() % 128) % 128;
    out.resize(out.size() + tail, 0xAA);
    return true;
}

bool PatchNativeSection(ByteSpan section, const std::vector<const CollisionPack*>& packs,
                        std::vector<CollisionPolygon>& moved, std::vector<u8>& out) {
    std::map<u32, NativeFaces> edited;
    for (const CollisionPack* pack : packs) {
        auto& faces = edited[pack->sectionNumber];
        for (const CollisionPolygon& polygon : pack->polygons)
            faces.emplace(polygon.nfswCollisionSource, &polygon);
    }
    size_t copied = 0;
    ChunkReader reader(section, 0);
    Chunk chunk;
    while (reader.Next(chunk)) {
        if (chunk.id != kCollisionPack) continue;
        const ByteSpan data = AlignedPayload(chunk);
        const u32 owner = data.read<u32>(4);
        const auto found = edited.find(owner);
        if (found == edited.end()) continue;
        std::vector<u8> carp;
        if (!PatchNativePack(data.sub(16, data.read<u32>(0)), owner, found->second, moved, carp))
            return false;
        if (carp.empty()) continue;
        const size_t at = (size_t)chunk.headerOffset;
        out.insert(out.end(), section.data + copied, section.data + at);
        AppendChunk(out, 0, kCollisionPack, (u16)owner, Span(carp));
        copied = at + 8 + chunk.raw.size;
    }
    if (copied) out.insert(out.end(), section.data + copied, section.end());
    return true;
}

} // namespace

bool NfswBuildPlacementCollision(ByteSpan layout, ByteSpan section,
                                 const std::vector<NfswCollisionPlacement>& placements,
                                 const std::vector<CollisionPolygon>& walls,
                                 std::vector<u8>& out, std::string& error,
                                 const std::vector<const CollisionPack*>& editedPacks) {
    std::vector<CollisionPolygon> moved;
    std::vector<u8> editedSection;
    if (!editedPacks.empty() && !PatchNativeSection(section, editedPacks, moved, editedSection)) {
        error = "an edited native collision mesh could not be rebuilt";
        return false;
    }
    if (!editedSection.empty()) section = Span(editedSection);
    ChunkReader reader(section, 0);
    Chunk chunk, gridChunk, generated;
    std::set<u32> owners;
    while (reader.Next(chunk)) {
        if (chunk.id == kCollisionCells) gridChunk = chunk;
        if (chunk.id != kCollisionPack) continue;
        const ByteSpan data = AlignedPayload(chunk);
        owners.insert(data.read<u32>(4));
        const ByteSpan carp = data.sub(16, data.read<u32>(0));
        size_t previousWalls = 0;
        if (GeneratedPack(carp, previousWalls)) generated = chunk;
    }
    if (placements.empty() && walls.empty() && moved.empty() && !generated.id) {
        out.assign(section.begin(), section.end());
        return true;
    }

    Grid grid;
    if (!gridChunk.id || !ReadGrid(layout, grid)) {
        error = "the NFS World static collision grid is missing";
        return false;
    }
    const ByteSpan oldGrid = AlignedPayload(gridChunk);
    const ByteSpan gridCarp = oldGrid.sub(16, oldGrid.read<u32>(0));
    const u16 gridOwner = (u16)oldGrid.read<u32>(4);
    const u16 oldOwner = (u16)AlignedPayload(generated).read<u32>(4);
    u16 owner = oldOwner ? oldOwner : 25999;
    // The collision manager covers section ids below 26000. The synthetic
    // pack is loaded with Z0 and has no placement or geometry section.
    if (!oldOwner) {
        while (owner > kNfswCollisionSection && owners.count(owner)) --owner;
        if (owner <= kNfswCollisionSection) {
            error = "no free collision pack number";
            return false;
        }
    }
    std::vector<CollisionMesh> meshes;
    if (!BuildMeshes(placements, meshes, error)) return false;
    if (!AddWalls(moved, meshes)) {
        error = "a moved native collision face exceeds the static collision format";
        return false;
    }
    if (!AddWalls(walls, meshes)) {
        error = "an edited wall exceeds the static collision format";
        return false;
    }
    std::map<u16, std::vector<u32>> references;
    std::vector<u8> newGrid;
    if (!ReferenceMeshes(grid, meshes, owner, references) ||
        !BuildGrid(gridCarp, gridOwner, oldOwner, references, newGrid)) {
        error = "the static collision cells could not be rebuilt";
        return false;
    }

    const size_t end = (size_t)gridChunk.headerOffset + 8 + gridChunk.raw.size;
    const size_t start = generated.id ? (size_t)generated.headerOffset : (size_t)gridChunk.headerOffset;
    if (generated.id && start + 8 + generated.raw.size != gridChunk.headerOffset) {
        error = "the generated collision pack is separated from its grid";
        return false;
    }
    std::vector<u8> replacement;
    if (!meshes.empty()) {
        const std::vector<u8> carp = BuildCarp(meshes, walls.size() + moved.size());
        AppendChunk(replacement, start, kCollisionPack, owner, Span(carp));
    }
    const size_t gridAt = start + replacement.size();
    const size_t after = Align16(gridAt + 8) + 16 + newGrid.size();
    const size_t padding = (end % 128 + 128 - after % 128) % 128;
    newGrid.resize(newGrid.size() + padding, 0xAA);
    AppendChunk(replacement, start, kCollisionCells, gridOwner, Span(newGrid));
    out.assign(section.data, section.data + start);
    out.insert(out.end(), replacement.begin(), replacement.end());
    out.insert(out.end(), section.data + end, section.data + section.size);
    return true;
}

bool NfswReadNativeCollision(ByteSpan section, std::vector<CollisionPack>& packs) {
    ChunkReader reader(section, 0);
    Chunk chunk;
    while (reader.Next(chunk)) {
        if (chunk.id != kCollisionPack) continue;
        const ByteSpan data = AlignedPayload(chunk);
        const u32 owner = data.read<u32>(4);
        const ByteSpan carp = data.sub(16, data.read<u32>(0));
        size_t generated = 0;
        if (!owner || owner >= kNfswCollisionSection || GeneratedPack(carp, generated)) continue;
        CollisionPack pack;
        pack.sectionNumber = owner;
        VisitNativeMeshes(carp, owner, [&](Article article, ByteSpan instance, u32 number,
                                           size_t ci, const std::vector<Strip>& strips) {
            const auto add = [&](const CollisionPolygon& polygon) {
                if (!SelectableFace(polygon)) return;
                for (const Vec3& point : polygon.corners) pack.bounds.Add(point);
                pack.polygons.push_back(polygon);
            };
            for (size_t group = 0; group < strips.size(); ++group) {
                const Strip& strip = strips[group];
                for (size_t i = 0; i + 2 < strip.vertices.size / 8; ++i)
                    add(StockFace(instance, strip, number, ci, group, i));
            }
            const ByteSpan boundaries = BoundaryRecords(article.bytes);
            for (size_t at = 0; at < boundaries.size; at += kBoundaryRecordSize)
                add(StockBoundary(instance, boundaries.sub(at, kBoundaryRecordSize), number,
                                  ci, at / kBoundaryRecordSize));
            return true;
        });
        if (!pack.polygons.empty()) packs.push_back(std::move(pack));
    }
    return true;
}

void NfswSurveyNativeSurfaces(ByteSpan section, std::map<u32, size_t>& strips,
                              std::map<u32, size_t>& boundaries) {
    ChunkReader reader(section, 0);
    Chunk chunk;
    while (reader.Next(chunk)) {
        if (chunk.id != kCollisionPack) continue;
        const ByteSpan data = AlignedPayload(chunk);
        const u32 owner = data.read<u32>(4);
        const ByteSpan carp = data.sub(16, data.read<u32>(0));
        size_t generated = 0;
        if (!owner || owner >= kNfswCollisionSection || GeneratedPack(carp, generated)) continue;
        const ByteSpan instances = FindArticle(carp, kArticles, kInstances).bytes;
        for (size_t ci = 0; ci + 64 <= instances.size; ci += 64) {
            const ByteSpan instance = instances.sub(ci, 64);
            const Article article = FindArticle(carp, kArticles, kMesh + instance.read<u16>(0x1A));
            if (article.bytes.empty()) continue;
            // The trail sits after the boundary records; the article is not
            // padded before it.
            const size_t end = 16 + (size_t)article.bytes.read<u16>(2) +
                               (size_t)article.bytes.read<u16>(6);
            if (article.bytes.size >= end + 4) ++strips[article.bytes.read<u32>(end)];
            const ByteSpan records = BoundaryRecords(article.bytes);
            for (size_t at = 0; at + kBoundaryRecordSize <= records.size; at += kBoundaryRecordSize)
                ++boundaries[records.read<u32>(at + 12)];
        }
    }
}

void NfswReadGeneratedRoadGround(ByteSpan section, CollisionPack& out) {
    ChunkReader reader(section, 0);
    Chunk chunk;
    while (reader.Next(chunk)) {
        if (chunk.id != kCollisionPack) continue;
        const ByteSpan data = AlignedPayload(chunk);
        const ByteSpan carp = data.sub(16, data.read<u32>(0));
        size_t editable = 0;
        GeneratedFormat format;
        if (!GeneratedPack(carp, editable, &format) || format != GeneratedFormat::WorldToLocal) continue;
        const ByteSpan instances = FindArticle(carp, kArticles, kInstances).bytes;
        const size_t total = instances.size / 64;
        if (editable > total) continue;
        std::vector<Strip> strips;
        for (size_t i = 0; i < total - editable; ++i) {
            const ByteSpan instance = instances.sub(i * 64, 64);
            const ByteSpan mesh = FindArticle(carp, kArticles, kMesh + instance.read<u16>(26)).bytes;
            if (mesh.read<u32>(16 + mesh.read<u16>(2)) != kRoadSurface || !ReadStrips(mesh, strips)) continue;
            for (size_t group = 0; group < strips.size(); ++group) {
                const Strip& strip = strips[group];
                for (size_t t = 0; t + 2 < strip.vertices.size / 8; ++t) {
                    const auto face = StockFace(instance, strip, 0, i, group, t);
                    if (!SelectableFace(face) || face.Vertical()) continue;
                    out.polygons.push_back(face);
                    for (const Vec3& point : face.corners) out.bounds.Add(point);
                }
            }
        }
    }
}

bool NfswReadCollisionWalls(ByteSpan section, std::vector<CollisionPolygon>& walls) {
    ChunkReader reader(section, 0);
    Chunk chunk;
    while (reader.Next(chunk)) {
        if (chunk.id != kCollisionPack) continue;
        const ByteSpan data = AlignedPayload(chunk);
        const ByteSpan carp = data.sub(16, data.read<u32>(0));
        size_t count = 0;
        GeneratedFormat format;
        if (!GeneratedPack(carp, count, &format) || count == 0) continue;
        const ByteSpan instances = FindArticle(carp, kArticles, kInstances).bytes;
        const size_t total = instances.size / 64;
        if (count > total) return false;
        std::vector<CollisionPolygon> parsed;
        std::vector<Strip> strips;
        for (size_t i = total - count; i < total; ++i) {
            const ByteSpan instance = instances.sub(i * 64, 64);
            const ByteSpan mesh = FindArticle(carp, kArticles,
                                              kMesh + instance.read<u16>(0x1A)).bytes;
            // The surface hash trails the article, so an edited face keeps its
            // surface across a save.
            const size_t trail = 16 + (size_t)mesh.read<u16>(2) + (size_t)mesh.read<u16>(6);
            const u32 surface = mesh.size >= trail + 4 ? mesh.read<u32>(trail) : 0;
            const u16 faces = mesh.read<u16>(0);
            if (!faces && format == GeneratedFormat::WorldToLocal) {
                const ByteSpan boundary = BoundaryRecords(mesh);
                if (boundary.size != kBoundaryRecordSize) return false;
                CollisionPolygon wall = StockBoundary(instance, boundary, 0, 0, 0);
                wall.nfswCollisionSource = 0;
                wall.nfswSurface = surface;
                parsed.push_back(wall);
                continue;
            }
            if ((faces != 1 && faces != 2) || !ReadStrips(mesh, strips)) return false;
            for (const Strip& strip : strips)
                if (strip.vertices.size != 24) return false;
            CollisionPolygon wall;
            wall.nfswSurface = surface;
            wall.quad = faces == 2;
            const auto point = [&](const Strip& strip, size_t v) {
                const Vec3 local = StripPoint(strip, v, format != GeneratedFormat::Legacy);
                if (format == GeneratedFormat::WorldToLocal) return InstancePoint(instance, local);
                // Earlier editor overlays stored authored coordinates in a
                // mirrored frame. Restore those edits before writing v3.
                return -FromPhysics(FloatPoint(instance, 48) + local);
            };
            for (size_t v = 0; v < 3; ++v)
                wall.corners[v] = point(strips[0], v);
            wall.corners[3] = wall.quad ? point(strips[1], 2) : wall.corners[2];
            ClassifyFace(wall);
            parsed.push_back(wall);
        }
        walls.swap(parsed);
        return true;
    }
    walls.clear();
    return true;
}

bool NfswReadNativeRoads(ByteSpan layout, NfswNativeRoads& roads) {
    nfs::ChunkReader reader(layout, 0);
    nfs::Chunk chunk;
    while (reader.Next(chunk) && chunk.id != kWorld) {}
    if (chunk.id != kWorld) return false;
    const auto carp = AlignedPayload(chunk);
    constexpr u32 group = 0x524E6770; // RNpg
    const auto header = FindArticle(carp, group, 0x524E6864).bytes; // RNhd
    const auto nodes = FindArticle(carp, group, 0x524E6E64).bytes; // RNnd
    const auto segments = FindArticle(carp, group, 0x524E7367).bytes; // RNsg
    const u16 nodeCount = header.read<u16>(2), segmentCount = header.read<u16>(4);
    if (!nodeCount || !segmentCount || segmentCount > INT16_MAX ||
        nodes.size != nodeCount * kRoadNodeBytes || segments.size != segmentCount * kRoadSegmentBytes) return false;

    NfswNativeRoads parsed;
    parsed.network = RoadNetworkKey(nodes, segments);
    parsed.segments.reserve(segmentCount);
    for (size_t i = 0; i < segmentCount; ++i) {
        const size_t at = i * kRoadSegmentBytes;
        const u16 a = segments.read<u16>(at), b = segments.read<u16>(at + 2);
        if (a >= nodeCount || b >= nodeCount) return false;
        const Vec3 start = FromPhysics(FloatPoint(nodes, a * kRoadNodeBytes));
        const Vec3 end = FromPhysics(FloatPoint(nodes, b * kRoadNodeBytes));
        if (!std::isfinite(Length(start)) || !std::isfinite(Length(end))) return false;
        parsed.segments.push_back({ start, end });
    }
    roads = std::move(parsed);
    return true;
}

namespace {
bool ReadRoadEndProfile(ByteSpan profile, NfswRoadEnd& end) {
    if (profile.size != 64) return false;
    const int count = profile.read<u8>(0), back = profile.read<u8>(1);
    if (count < 1 || count > 14 || back > count) return false;
    constexpr float unit = 0.012208521366119385f;
    float innerOffset = 1e6f;
    for (int lane = 0; lane < count; ++lane) {
        const u32 word = profile.read<u32>(4 + lane * 4);
        const u32 type = word & 0xf;
        const float width = ((word >> 4) & 0x3fff) * unit;
        const float offset = (word >> 18) * unit;
        if (type == 1) {
            if (lane < back) ++end.backward;
            else ++end.forward;
            if (end.laneWidth > 0 && std::abs(end.laneWidth - width) > 0.55f) return false;
            end.laneWidth = width;
            innerOffset = std::min(innerOffset, offset);
        } else if ((type == 2 || type == 3) && (lane == 0 || lane + 1 == count)) {
            (lane == 0 ? end.leftShoulder : end.rightShoulder) = width;
        } else if (type == 4 && offset < 0.1f) {
            end.medianWidth = width;
        } else {
            return false;
        }
    }
    if (!end.medianWidth)
        end.medianWidth = std::max(0.0f, 2 * innerOffset - end.laneWidth);
    return end.laneWidth >= 2.0f && end.laneWidth <= 8.0f && end.medianWidth <= 32.0f &&
           end.leftShoulder <= 10 && end.rightShoulder <= 10;
}
} // namespace

std::optional<NfswRoadEnd> NfswFindRoadEnd(ByteSpan layout, const Vec3& point, const Aabb& roadBounds,
                                          const Vec3& toward) {
    if (!roadBounds.Valid()) return {};
    ChunkReader reader(layout, 0);
    Chunk chunk;
    while (reader.Next(chunk) && chunk.id != kWorld) {}
    if (chunk.id != kWorld) return {};
    const ByteSpan carp = AlignedPayload(chunk);
    constexpr u32 group = 0x524E6770;
    const auto nodes = FindArticle(carp, group, 0x524E6E64).bytes;
    const auto segments = FindArticle(carp, group, 0x524E7367).bytes;
    const auto profiles = FindArticle(carp, group, 0x524E7066).bytes;

    std::optional<NfswRoadEnd> found;
    float best = 65.0f;
    for (size_t at = 0; at + kRoadSegmentBytes <= segments.size; at += kRoadSegmentBytes) {
        const size_t indices[] = { segments.read<u16>(at) * kRoadNodeBytes, segments.read<u16>(at + 2) * kRoadNodeBytes };
        if (std::max(indices[0], indices[1]) + kRoadNodeBytes > nodes.size) continue;
        for (size_t end = 0; end < 2; ++end) {
            const size_t node = indices[end];
            if (nodes.read<u8>(node + 16) >= 7) continue;
            const Vec3 pos = FromPhysics(FloatPoint(nodes, node));
            if (pos.x < roadBounds.min.x - 3 || pos.x > roadBounds.max.x + 3 ||
                pos.y < roadBounds.min.y - 3 || pos.y > roadBounds.max.y + 3 ||
                pos.z < roadBounds.min.z - 3 || pos.z > roadBounds.max.z + 3 ||
                std::abs(pos.z - point.z) > 6.0f) continue;
            const size_t direction = at + (end == 0 ? 0x15 : 0x12);
            Vec3 heading = -FromPhysics({ (float)segments.read<i8>(direction), (float)segments.read<i8>(direction + 1),
                                          (float)segments.read<i8>(direction + 2) });
            if (Length(heading) < 1) heading = pos - FromPhysics(FloatPoint(nodes, indices[1 - end]));
            heading = Normalize({ heading.x, heading.y, 0 });
            if (Length(heading) < 0.5f) continue;
            const float score = Length(pos - point) - 2 * Dot(heading, toward);
            if (!(score < best)) continue;

            NfswRoadEnd candidate{ pos, heading };
            candidate.nativeProfile = nodes.read<u16>(node + 14);
            if (!ReadRoadEndProfile(profiles.sub((size_t)candidate.nativeProfile * 64, 64), candidate)) continue;
            found = candidate;
            best = score;
        }
    }
    return found;
}

bool NfswAppendRoadGraph(std::vector<u8>& layout, const std::vector<NfswRoadSegmentBuild>& input,
                          NfswRoadBuildReport& report, std::string& error) {
    report = {};
    if (input.empty()) return true;
    ChunkReader reader(Span(layout), 0);
    Chunk chunk;
    while (reader.Next(chunk) && chunk.id != kWorld) {}
    if (chunk.id != kWorld) { error = "the World road network container is missing"; return false; }
    const ByteSpan carp = AlignedPayload(chunk);
    constexpr u32 group = 0x524E6770;
    auto header = FindArticle(carp, group, 0x524E6864);
    auto profiles = FindArticle(carp, group, 0x524E7066);
    auto nodes = FindArticle(carp, group, 0x524E6E64);
    auto segments = FindArticle(carp, group, 0x524E7367);
    auto roads = FindArticle(carp, group, 0x524E7264);
    auto lanes = FindArticle(carp, group, 0x524E6C6D);
    if (header.bytes.size != 14 || header.bytes.read<u16>(6) != 0 ||
        profiles.bytes.size != header.bytes.read<u16>(0) * 64u ||
        nodes.bytes.size != header.bytes.read<u16>(2) * 32u ||
        segments.bytes.size != header.bytes.read<u16>(4) * 24u ||
        roads.bytes.size != header.bytes.read<u16>(8) * 8u ||
        lanes.bytes.size != header.bytes.read<u16>(12) * 32u ||
        profiles.bytes.empty() || nodes.bytes.empty() || segments.bytes.empty() ||
        roads.bytes.empty() || lanes.bytes.empty()) {
        error = "the native RNhd counts disagree with the World road records";
        return false;
    }
    const size_t stockNodes = nodes.bytes.size / 32, stockSegments = segments.bytes.size / 24;
    if (stockSegments + input.size() >= INT16_MAX || roads.bytes.size / 8 + input.size() >= INT16_MAX) {
        error = "too many World road segments";
        return false;
    }
    std::vector<u8> newHeader(header.bytes.begin(), header.bytes.end());
    std::vector<u8> newProfiles(profiles.bytes.begin(), profiles.bytes.end());
    std::vector<u8> newNodes(nodes.bytes.begin(), nodes.bytes.end());
    std::vector<u8> newSegments(segments.bytes.begin(), segments.bytes.end());
    std::vector<u8> newRoads(roads.bytes.begin(), roads.bytes.end());
    std::vector<u8> newLanes(lanes.bytes.begin(), lanes.bytes.end());
    struct MappedNode { u16 index, profile; };
    std::map<size_t, MappedNode> mapped;

    const auto profileExists = [&](ByteSpan data) {
        for (size_t at = 0; at < newProfiles.size(); at += 64)
            if (!std::memcmp(newProfiles.data() + at, data.data, 64)) return at / 64;
        return newProfiles.size() / 64;
    };
    const auto profileFor = [&](const NfswRoadSegmentBuild& road, u16& index) {
        if (road.nativeProfile >= 0) {
            NfswRoadEnd original;
            if (!ReadRoadEndProfile(profiles.bytes.sub((size_t)road.nativeProfile * 64, 64), original) ||
                original.forward != road.forward || original.backward != road.backward ||
                std::abs(original.laneWidth - road.width) > 0.04f ||
                std::abs(original.medianWidth - road.median) > 0.2f) return false;
            index = (u16)road.nativeProfile;
            return true;
        }
        const int count = road.forward + road.backward;
        if (road.forward < 0 || road.backward < 0 || count < 1 || count > 14 ||
            !std::isfinite(road.width) || !std::isfinite(road.median) ||
            road.width < 2.0f || road.width > 8.0f || road.median < 0.0f || road.median > 32.0f) return false;
        std::vector<u8> profile(64, 0xAA);
        profile[0] = (u8)count;
        profile[1] = (u8)road.backward;
        constexpr float unit = 0.012208521366119385f;
        const u32 width = (u32)std::lround(road.width / unit);
        for (size_t i = 0; i < 15; ++i) {
            u32 word = 0xFEBFFAFF;
            if (i < (size_t)count) {
                const float lane = i < (size_t)road.backward ? (float)(road.backward - i) - 0.5f
                                                            : (float)(i - road.backward) + 0.5f;
                const u32 offset = (u32)std::lround((lane * road.width + road.median * 0.5f) / unit);
                if (offset > 8191) return false;
                word = (offset << 18) | (width << 4) | 1u;
            }
            Put(profile, 4 + i * 4, word);
        }
        const size_t slot = profileExists(Span(profile));
        if (slot >= INT16_MAX) return false;
        index = (u16)slot;
        if (slot * 64 == newProfiles.size())
            newProfiles.insert(newProfiles.end(), profile.begin(), profile.end());
        return true;
    };
    const auto position = [&](u16 index) {
        return FromPhysics(FloatPoint(Span(newNodes), (size_t)index * 32));
    };
    const auto resolveNode = [&](size_t id, const Vec3& point, u16 profile, bool exact, u16& node) {
        if (!std::isfinite(Length(point))) return false;
        if (const auto at = mapped.find(id); at != mapped.end()) {
            node = at->second.index;
            return at->second.profile == profile && Length(position(node) - point) <= 1.25f;
        }
        size_t bestNode = SIZE_MAX;
        float best = 1.25f;
        const auto p = Span(newProfiles).sub((size_t)profile * 64, 64);
        for (size_t i = 0; i < stockNodes; ++i) {
            const size_t at = i * 32;
            const float distance = Length(position((u16)i) - point);
            if (distance >= best || newNodes[at + 16] >= 7) continue;
            const u16 stockProfile = Span(newNodes).read<u16>(at + 14);
            const auto q = profiles.bytes.sub((size_t)stockProfile * 64, 64);
            if (q.size != 64 || q.data[0] != p.data[0] || q.data[1] != p.data[1]) continue;
            if (exact) {
                if (std::memcmp(q.data, p.data, 64) != 0) continue;
            } else {
                bool compatible = true;
                for (size_t lane = 0; lane < p.data[0]; ++lane) {
                    const u32 x = p.read<u32>(4 + lane * 4), y = q.read<u32>(4 + lane * 4);
                    compatible &= (y & 0xf) == 1 && std::abs((int)(x >> 18) - (int)(y >> 18)) <= 42;
                }
                if (!compatible) continue;
            }
            bestNode = i;
            best = distance;
        }

        if (bestNode != SIZE_MAX) {
            node = (u16)bestNode;
            ++report.stockJoins;
        } else {
            const size_t at = newNodes.size();
            if (at / 32 >= INT16_MAX) return false;
            node = (u16)(at / 32);
            newNodes.resize(at + 32, 0xAA);
            Put(newNodes, at, Vec3{ -point.y, point.z, point.x });
            Put(newNodes, at + 12, node);
            Put(newNodes, at + 14, profile);
            newNodes[at + 16] = 0;
            ++report.nodes;
        }
        mapped.emplace(id, MappedNode{ node, profile });
        return true;
    };

    const auto direction = [&](const Vec3& from, const Vec3& to, size_t magnitude, size_t components) {
        const Vec3 delta = to - from;
        const float length = Length(delta);
        if (!std::isfinite(length) || length < 0.001f || length > 500.0f) return false;
        Put(newSegments, magnitude, (u16)std::lround(length * (65535.0f / 500.0f)));
        Put(newSegments, components + 0, (i8)std::lround(-delta.y * 127.0f / length));
        Put(newSegments, components + 1, (i8)std::lround(delta.z * 127.0f / length));
        Put(newSegments, components + 2, (i8)std::lround(delta.x * 127.0f / length));
        return true;
    };

    for (const auto& road : input) {
        u16 a = 0, b = 0, profile = 0;
        const bool exact = road.nativeProfile >= 0;
        if (!profileFor(road, profile) || !resolveNode(road.a, road.cubic[0], profile, exact, a) ||
            !resolveNode(road.b, road.cubic[3], profile, exact, b) || a == b) {
            error = "a junction has incompatible lane profiles, or a road node is invalid";
            return false;
        }
        const Vec3 start = position(a), end = position(b);
        float length = 0.0f;
        Vec3 previous = start;
        for (int i = 1; i <= 16; ++i) {
            const float t = i / 16.0f, s = 1.0f - t;
            const Vec3 next = start * (s * s * s) + road.cubic[1] * (3 * s * s * t) +
                              road.cubic[2] * (3 * s * t * t) + end * (t * t * t);
            length += Length(next - previous);
            previous = next;
        }
        if (!std::isfinite(length) || length < 1.0f || length > 1000.0f) {
            error = "a World road segment has an invalid curve length";
            return false;
        }
        const int laneCount = newProfiles[(size_t)profile * 64];
        u16 mapId = 0;
        for (; mapId < newLanes.size() / 32; ++mapId) {
            const u8* map = newLanes.data() + (size_t)mapId * 32;
            if (map[0] != laneCount || map[16] != laneCount) continue;
            bool identity = true;
            for (int i = 0; i < laneCount; ++i)
                identity &= map[1 + i] == i && map[17 + i] == i;
            if (identity) break;
        }
        if (mapId == newLanes.size() / 32) {
            const size_t startMap = newLanes.size();
            if (mapId >= INT16_MAX) { error = "too many World lane maps"; return false; }
            newLanes.resize(startMap + 32);
            for (size_t side : { startMap, startMap + 16 }) {
                newLanes[side] = (u8)laneCount;
                for (int i = 0; i < laneCount; ++i) newLanes[side + 1 + i] = (u8)i;
            }
        }

        const size_t at = newSegments.size();
        const u16 index = (u16)(at / 24), roadId = (u16)(newRoads.size() / 8);
        newSegments.resize(at + 24);
        // An ordinary curved road, with its own road-data entry and identity lane map.
        const std::array<u8, 8> roadData{ 0, 1, 0xFF, 0xFF, 0xFF, 101, 0, 0 };
        newRoads.insert(newRoads.end(), roadData.begin(), roadData.end());
        Put(newSegments, at, a);
        Put(newSegments, at + 2, b);
        Put(newSegments, at + 4, (u16)std::lround(length * (65535.0f / 1000.0f)));
        Put(newSegments, at + 6, roadId);
        Put(newSegments, at + 8, index);
        Put(newSegments, at + 10, mapId);
        Put(newSegments, at + 12, u16{0x100});
        if (!direction(road.cubic[0], road.cubic[1], at + 0x10, at + 0x15) ||
            !direction(road.cubic[3], road.cubic[2], at + 0x0E, at + 0x12)) {
            error = "a road tangent is outside the native fixed-point range";
            return false;
        }
        for (u16 node : { a, b }) {
            const size_t base = (size_t)node * 32;
            u8& links = newNodes[base + 16];
            if (links >= 7) { error = "a World node can connect at most seven segments"; return false; }
            Put(newNodes, base + 18 + (size_t)links * 2, index);
            ++links;
        }
        ++report.segments;
    }

    Put(newHeader, 0, (u16)(newProfiles.size() / 64));
    Put(newHeader, 2, (u16)(newNodes.size() / 32));
    Put(newHeader, 4, (u16)(newSegments.size() / 24));
    Put(newHeader, 8, (u16)(newRoads.size() / 8));
    Put(newHeader, 12, (u16)(newLanes.size() / 32));
    const size_t root = (size_t)(carp.data - layout.data());
    const size_t end = (size_t)chunk.headerOffset + 8 + chunk.raw.size;
    // Keep all other CARP articles where they shipped. New road records go
    // behind the old payload, with leaf offsets still relative to their rows.
    std::vector<u8> updated(layout.begin(), layout.begin() + end);
    const auto append = [&](const Article& article, const std::vector<u8>& bytes, u32 count) {
        updated.resize(Align16(updated.size()), 0);
        const size_t at = updated.size();
        if (bytes.size() >= 1u << 24 || at - root - article.row > UINT32_MAX) return false;
        updated.insert(updated.end(), bytes.begin(), bytes.end());
        Put(updated, root + article.row + 4, ((u32)bytes.size() << 8) | (carp.read<u32>(article.row + 4) & 0xff));
        Put(updated, root + article.row + 8, count);
        Put(updated, root + article.row + 12, (u32)(at - root - article.row));
        return true;
    };
    if (!append(header, newHeader, 1) || !append(profiles, newProfiles, (u32)(newProfiles.size() / 64)) ||
        !append(nodes, newNodes, (u32)(newNodes.size() / 32)) ||
        !append(segments, newSegments, (u32)(newSegments.size() / 24)) ||
        !append(roads, newRoads, (u32)(newRoads.size() / 8)) ||
        !append(lanes, newLanes, (u32)(newLanes.size() / 32))) {
        error = "the World road articles exceed their section size limit";
        return false;
    }
    updated.resize(end + ((updated.size() - end + 127) & ~size_t(127)), 0);
    Put(updated, (size_t)chunk.headerOffset + 4, (u32)(updated.size() - chunk.headerOffset - 8));
    updated.insert(updated.end(), layout.begin() + end, layout.end());
    layout.swap(updated);
    return true;
}

bool NfswIsSingleSmackableBox(ByteSpan geometry) {
    // 0x40 is the box primitive. A compound or a shape with extra primitive
    // data cannot be resized by writing the root half-extents alone.
    return geometry.size >= 80 && geometry.read<u32>(4) == 1 &&
           (geometry.read<u16>(16 + 14) & 0xC0) == 0x40 &&
           geometry.read<u16>(16 + 30) == 0xFFFF &&
           std::all_of(geometry.data + 64, geometry.end(), [](u8 byte) { return byte == 0; });
}

void NfswReadSmackableTypes(ByteSpan layout, NfswSmackableTypes& types) {
    types.clear();
    ChunkReader reader(layout, 0);
    Chunk chunk;
    while (reader.Next(chunk)) {
        if (chunk.id != kSmackableGeometryList) continue;
        ChunkReader children = ChunkReader::Children(chunk);
        Chunk child;
        while (children.Next(child)) {
            if (child.id != kSmackableGeometry) continue;
            const ByteSpan geometry = AlignedPayload(child);
            const size_t count = geometry.read<u32>(4);
            if (geometry.size < 16 || !count || count > (geometry.size - 16) / 48) continue;
            // The first Bound is the outer box queried when a smackable spawns.
            const ByteSpan bound = geometry.sub(16, 48);
            const auto millimetres = [&](size_t offset) {
                return Vec3{ bound.read<i16>(offset) * 0.001f,
                             bound.read<i16>(offset + 2) * 0.001f,
                             bound.read<i16>(offset + 4) * 0.001f };
            };
            NfswSmackableBounds box;
            box.center = millimetres(8);
            box.halfSize = millimetres(16);
            if (std::min({ box.halfSize.x, box.halfSize.y, box.halfSize.z }) < 0 ||
                Dot(box.halfSize, box.halfSize) == 0) continue;
            for (size_t i = 0; i < box.rotation.size(); ++i)
                box.rotation[i] = bound.read<i16>(i * 2) / 32767.0f;
            box.singleBox = box.halfSize.x > 0 && box.halfSize.y > 0 && box.halfSize.z > 0 &&
                            NfswIsSingleSmackableBox(geometry);
            types.emplace(geometry.read<u32>(0), box);
        }
    }
}

void NfswReadSmackableBoxes(const Chunk& chunk, const NfswSmackableTypes& types,
                          std::vector<NfswSmackableBox>& boxes, std::vector<NfswSmackableLink>& links) {
    const ByteSpan data = AlignedPayload(chunk);
    if (data.size < 16 || data.read<u32>(0) != 11 || data.read<u32>(4) != 11) return;
    const size_t count = data.read<u16>(0x0A);
    if (count > (data.size - 16) / 64) return;
    boxes.reserve(boxes.size() + count);
    links.reserve(links.size() + count);
    for (size_t i = 0; i < count; ++i) {
        const ByteSpan spawn = data.sub(16 + i * 64, 64);
        NfswSmackableLink link{ spawn.read<u32>(0x28), spawn.read<u32>(0x20),
                              spawn.read<u32>(kSmackableAttributeOffset), {} };
        std::copy_n(spawn.data, link.spawn.size(), link.spawn.begin());
        links.push_back(link);
        const auto found = types.find(link.geometry);
        if (found == types.end()) continue;
        const Vec3 position = FloatPoint(spawn, 16);
        std::array<float, 4> rotation;
        for (size_t q = 0; q < rotation.size(); ++q)
            rotation[q] = spawn.read<float>(q * 4);
        const Vec3 axis{ rotation[0], rotation[1], rotation[2] };
        const float norm = Dot(axis, axis) + rotation[3] * rotation[3];
        // The game builds the matrix directly from the stored components.
        if (!std::isfinite(Length(position)) || !std::isfinite(norm)) continue;
        const NfswSmackableBounds& bound = found->second;
        NfswSmackableBox box;
        for (size_t c = 0; c < box.size(); ++c) {
            const Vec3 corner{ c & 1 ? bound.halfSize.x : -bound.halfSize.x,
                               c & 2 ? bound.halfSize.y : -bound.halfSize.y,
                               c & 4 ? bound.halfSize.z : -bound.halfSize.z };
            const Vec3 local = bound.center + Rotate(bound.rotation, corner);
            box[c] = FromPhysics(position + Rotate(rotation, local));
        }
        links.back().box = boxes.size();
        links.back().singleBox = bound.singleBox;
        boxes.push_back(box);
    }
}

NfswSmackableBox NfswSmackableLink::EditedBox(const NfswSmackableBox& original) const {
    if (!halfSizes) return original;
    const Vec3 center = (original[0] + original[7]) * 0.5f;
    const Vec3 x = Normalize(original[1] - original[0]) * ((*halfSizes)[0] * 0.001f);
    const Vec3 y = Normalize(original[2] - original[0]) * ((*halfSizes)[1] * 0.001f);
    const Vec3 z = Normalize(original[4] - original[0]) * ((*halfSizes)[2] * 0.001f);
    NfswSmackableBox edited;
    for (size_t i = 0; i < edited.size(); ++i)
        edited[i] = center + x * (i & 1 ? 1.0f : -1.0f) + y * (i & 2 ? 1.0f : -1.0f) + z * (i & 4 ? 1.0f : -1.0f);
    return edited;
}

NfswSmackableBox NfswSmackablePlacement::BoxAt(const Mat4& transform) const {
    const Mat4 delta = InverseAffine(sourceTransform) * transform;
    NfswSmackableBox moved = link.EditedBox(box);
    for (Vec3& corner : moved) corner = delta.TransformPoint(corner);
    return moved;
}

} // namespace nfs
