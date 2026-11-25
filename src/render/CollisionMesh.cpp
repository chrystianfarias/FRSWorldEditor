#include "render/CollisionMesh.h"

#include "nfs/NfswCollision.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace {

struct CollisionVertex {
    float    x, y, z;
    D3DCOLOR colour;
};

// Collision lies almost exactly on the road surface. Lifting it a hair keeps
// it from z-fighting with the render mesh without visibly floating.
constexpr float kLift = 0.05f;
// World keeps its static wall polygons in Z0; a single draw can exceed the
// primitive limit on Direct3D 9 devices.
constexpr u32 kPrimitivesPerDraw = 65535;

constexpr D3DCOLOR kBarrierColour   = 0x90FF3030;
constexpr D3DCOLOR kRaceColour      = 0x80FF9A20;   // a race barrier group: absent in free roam
constexpr D3DCOLOR kGroundOutline   = 0xFF101010;
constexpr D3DCOLOR kBarrierOutline  = 0xFFFFD040;
constexpr D3DCOLOR kVerticalOutline = 0xFFFF8040;
constexpr D3DCOLOR kTallWallOutline = 0xFFD79B69;
constexpr D3DCOLOR kSmackableOutline = 0xFF69DFF2;
constexpr D3DCOLOR kRaceOutline     = 0xFFFFE080;

D3DCOLOR Scale(u32 rgb, float shade, u32 alpha) {
    auto channel = [&](int shift) {
        const float v = (float)((rgb >> shift) & 0xFF) * shade;
        return (u32)(v > 255.0f ? 255.0f : v);
    };
    return D3DCOLOR_ARGB(alpha, channel(16), channel(8), channel(0));
}

} // namespace

u32 CollisionMesh::SurfaceRgb(u8 surface) {
    switch (surface) {
        case 1:  return 0x8C8C96;   // ROAD
        case 2:  return 0x6E7A96;   // ROAD_WET
        case 3: case 4: case 5: case 6: return 0x9C8CB4;   // drift / smoke
        case 7:  return 0x7890B4;   // ROAD_BRIDGE
        case 8:  return 0xA0784A;   // DIRT
        case 9:  return 0xB4A07A;   // GRAVEL
        case 10: return 0x8C7860;   // ROUGH_ROAD
        case 11: return 0xA08C78;   // COBBLESTONE
        case 12: return 0xC8B48C;   // STAIRS
        case 13: return 0x50B4DC;   // PUDDLE
        case 14: return 0x2864DC;   // DEEP_WATER
        case 15: return 0x50B446;   // GRASS
        case 16: return 0xE6DCB4;   // SIDEWALK
        case 17: return 0xC88232;   // WOOD
        case 18: return 0xB45AC8;   // PLASTIC
        case 19: return 0x96E6F0;   // GLASS
        case 20: return 0xC84646;   // SOLID_WALL
        case 21: return 0xDC7878;   // SEE_THROUGH_WALL
        case 22: return 0x3C963C;   // PLANT
        case 23: case 24: return 0xB4B4B4;   // POST, PILLAR
        case 25: case 26: return 0x8CAAC8;   // METAL_GRATE, METAL
        case 27: return 0xA0A0C8;   // CHAINLINK
        default: return 0xF0F040;
    }
}

namespace {

void Push(std::vector<CollisionVertex>& out, const Vec3& p, D3DCOLOR colour, float lift = kLift) {
    out.push_back({ p.x, p.y, p.z + lift, colour });
}

bool SamePoint(const Vec3& a, const Vec3& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

bool IsInternalStripEdge(const nfs::CollisionPolygon& polygon,
                         const nfs::CollisionPolygon& neighbour, const Vec3& normal,
                         const Vec3& a, const Vec3& b) {
    if (neighbour.quad || neighbour.surface != polygon.surface ||
        (polygon.nfswCollisionSource >> 16) != (neighbour.nfswCollisionSource >> 16)) return false;
    const Vec3* p = neighbour.corners;
    const Vec3 otherNormal = Normalize(Cross(p[1] - p[0], p[2] - p[0]));
    if (std::abs(Dot(normal, otherNormal)) < 0.9996f) return false;
    bool hasA = false, hasB = false;
    for (int c = 0; c < 3; ++c) {
        hasA |= SamePoint(a, p[c]);
        hasB |= SamePoint(b, p[c]);
    }
    return hasA && hasB;
}

} // namespace

CollisionMesh::~CollisionMesh() {
    Release();
}

void CollisionMesh::Release() {
    if (buffer_) { buffer_->Release(); buffer_ = nullptr; }
    surfaceTriangles_ = lineStart_ = lineCount_ = 0;
    polygonCount_ = barrierCount_ = 0;
    bounds_ = {};
}

bool CollisionMesh::Upload(IDirect3DDevice9* device, const nfs::LoadedSection& section,
                           bool showRaceGroups, const nfs::CollisionPack* placed,
                           const std::vector<nfs::NfswSmackableBox>* editedBoxes) {
    Release();
    const auto& boxes = editedBoxes ? *editedBoxes : section.smackableBoxes;
    if (section.collision.empty() && boxes.empty() && !placed) return false;
    sectionNumber_ = section.info ? section.info->number : 0;

    std::vector<CollisionVertex> surfaces, lines;

    const auto uploadPack = [&](const nfs::CollisionPack& pack, bool preview) {
        sectionNumber_ = pack.sectionNumber;
        pack.ResolveBarriers();   // barrier heights, chain by chain

        for (size_t index = 0; index < pack.polygons.size(); ++index) {
            const nfs::CollisionPolygon& polygon = pack.polygons[index];
            ++polygonCount_;
            const Vec3* p = polygon.corners;
            const bool race = polygon.raceGroup != 0;
            if (race && !showRaceGroups) continue;

            if (polygon.barrier) {
                ++barrierCount_;
                Vec3 wall[4];
                pack.BarrierQuad(polygon, wall);
                const Vec3 &a0 = wall[0], &a1 = wall[1], &b1 = wall[2], &b0 = wall[3];

                const D3DCOLOR fill = race ? kRaceColour : kBarrierColour;
                const D3DCOLOR edge = race ? kRaceOutline : kBarrierOutline;
                for (const Vec3& v : { a0, a1, b1, a0, b1, b0 }) Push(surfaces, v, fill);
                for (const Vec3& v : { a1, b1, a0, b0 })         Push(lines, v, edge);

                bounds_.Add(a0); bounds_.Add(a1); bounds_.Add(b0); bounds_.Add(b1);
                continue;
            }

            // Shaded by slope against a light from above and to one side, so a
            // ramp or a camber stands out from the flat road around it.
            const Vec3 normal = Normalize(Cross(p[1] - p[0], p[2] - p[0]));
            const Vec3 light  = Normalize(Vec3{ 0.4f, 0.3f, 1.0f });
            const float shade = 0.45f + 0.55f * std::fabs(Dot(normal, light));
            const bool nativeStrip = polygon.nfswCollisionSource && !polygon.quad;
            const bool tallBoundary = polygon.TallNfswBoundary();
            // A World polygon carrying a native surface hash shows the two
            // known ones in their usual colours, any other in violet.
            const u32 rgb = !polygon.nfswSurface ? SurfaceRgb(polygon.surface)
                          : polygon.nfswSurface == nfs::kRoadSurface ? SurfaceRgb(nfs::kSurfaceRoad)
                          : polygon.nfswSurface == nfs::kSolidSurface ? SurfaceRgb(nfs::kSurfaceSolidWall)
                          : 0xB478DC;
            const D3DCOLOR colour = race ? kRaceColour
                                  : Scale(rgb, shade,
                                          polygon.Vertical() ? 0xC0 : (nativeStrip ? 0x60 : 0xA0));
            const D3DCOLOR outline = preview ? 0xFF89D58A
                                   : race ? kRaceOutline
                                   : tallBoundary ? kTallWallOutline
                                   : polygon.Vertical() ? kVerticalOutline : kGroundOutline;

            // The stock bounds can span the whole map vertically. Their
            // full-height wireframe leaves the road and smaller walls visible.
            if (!tallBoundary) {
                for (int c : { 0, 1, 2 }) Push(surfaces, p[c], colour);
                if (polygon.quad)
                    for (int c : { 0, 2, 3 }) Push(surfaces, p[c], colour);
            }

            const int corners = polygon.quad ? 4 : 3;
            for (int c = 0; c < corners; ++c) {
                bounds_.Add(p[c]);
                const Vec3& a = p[c];
                const Vec3& b = p[(c + 1) % corners];
                if (nativeStrip &&
                    ((index > 0 &&
                      IsInternalStripEdge(polygon, pack.polygons[index - 1], normal, a, b)) ||
                     (index + 1 < pack.polygons.size() &&
                      IsInternalStripEdge(polygon, pack.polygons[index + 1], normal, a, b)))) continue;
                Push(lines, a, outline);
                Push(lines, b, outline);
            }
        }
    };
    for (const nfs::CollisionPack& pack : section.collision) uploadPack(pack, false);
    if (placed) uploadPack(*placed, true);

    for (const nfs::NfswSmackableBox& box : boxes) {
        for (size_t corner = 0; corner < box.size(); ++corner) {
            bounds_.Add(box[corner]);
            for (size_t axis : { 1u, 2u, 4u }) {
                if (corner & axis) continue;
                Push(lines, box[corner], kSmackableOutline, 0.0f);
                Push(lines, box[corner | axis], kSmackableOutline, 0.0f);
            }
        }
    }

    if (surfaces.empty() && lines.empty()) return false;

    surfaceTriangles_ = (u32)(surfaces.size() / 3);
    lineStart_        = (u32)surfaces.size();
    lineCount_        = (u32)(lines.size() / 2);
    surfaces.insert(surfaces.end(), lines.begin(), lines.end());

    const UINT bytes = (UINT)(surfaces.size() * sizeof(CollisionVertex));
    if (FAILED(device->CreateVertexBuffer(bytes, D3DUSAGE_WRITEONLY, kFvf, D3DPOOL_MANAGED,
                                          &buffer_, nullptr)))
        return false;

    void* target = nullptr;
    if (FAILED(buffer_->Lock(0, bytes, &target, 0))) {
        Release();
        return false;
    }
    std::memcpy(target, surfaces.data(), bytes);
    buffer_->Unlock();
    return true;
}

void CollisionMesh::DrawSurfaces(IDirect3DDevice9* device) const {
    if (!buffer_ || !surfaceTriangles_) return;
    device->SetStreamSource(0, buffer_, 0, sizeof(CollisionVertex));
    for (u32 first = 0; first < surfaceTriangles_; first += kPrimitivesPerDraw)
        device->DrawPrimitive(D3DPT_TRIANGLELIST, first * 3,
                              std::min(kPrimitivesPerDraw, surfaceTriangles_ - first));
}

void CollisionMesh::DrawOutlines(IDirect3DDevice9* device) const {
    if (!buffer_ || !lineCount_) return;
    device->SetStreamSource(0, buffer_, 0, sizeof(CollisionVertex));
    for (u32 first = 0; first < lineCount_; first += kPrimitivesPerDraw)
        device->DrawPrimitive(D3DPT_LINELIST, lineStart_ + first * 2,
                              std::min(kPrimitivesPerDraw, lineCount_ - first));
}
