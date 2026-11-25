#include "render/WorldRenderer.h"

#include "core/Log.h"
#include "io/CollisionGenerate.h"
#include "io/DistrictFog.h"
#include "io/TextureImport.h"
#include "render/RenderExtension.h"
#include "render/GameLook.h"

#include <windows.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <unordered_map>
#include <unordered_set>

namespace {

// Opaque mid grey. Drawn in place of a texture that could not be decoded, which
// reads as "asset missing" instead of vanishing into the night lighting.
constexpr D3DCOLOR kMissingTextureColor = 0xFF9A9A9A;

// Converts our row-major Mat4 into the D3DMATRIX the fixed-function pipeline
// expects. Both are row-major with translation in the last row, so the copy is
// a straight reinterpretation.
D3DMATRIX ToD3D(const Mat4& source) {
    D3DMATRIX result;
    std::memcpy(&result, source.m, sizeof(result));
    return result;
}

double SecondsNow() {
    LARGE_INTEGER frequency, counter;
    ::QueryPerformanceFrequency(&frequency);
    ::QueryPerformanceCounter(&counter);
    return (double)counter.QuadPart / (double)frequency.QuadPart;
}

// NFS World's object tracks advance by 15 * speed frames per second:
// sub_74F520 adds 15 * controllerRate * speed * dt to the frame position, and
// the speed byte's high nibble is in 15 fps steps (0x10 = 15, 0x20 = 30,
// 0x40 = 60), the low nibble in single fps steps.
constexpr double kSceneryAnimationTicksPerSecond = 15.0;

// A free camera crosses the zone seams much faster than a car does, and the
// polygons overlap with hard edges. Holding a new zone's list for half a
// second keeps a camera on a boundary from strobing between two sets.
constexpr double kZoneSwitchDwellSeconds = 0.5;

// FNV-1a over the visible section numbers. Two lists can hold the same count
// while naming different sections, and the zone log has to tell them apart.
u32 HashSections(const std::vector<u32>& sections) {
    u32 hash = 2166136261u;
    for (u32 number : sections) {
        hash ^= number;
        hash *= 16777619u;
    }
    return hash;
}

// Placements two sorted section lists have in common: how much of the picture
// a zone switch keeps.
size_t KeptSections(const std::vector<u32>& before, const std::vector<u32>& after) {
    size_t kept = 0;
    size_t i = 0, j = 0;
    while (i < before.size() && j < after.size()) {
        if (before[i] < after[j]) ++i;
        else if (after[j] < before[i]) ++j;
        else { ++kept; ++i; ++j; }
    }
    return kept;
}

} // namespace

void WorldRenderer::Clear() {
    objectByHash_.clear();
    for (RenderExtension* extension : extensions_) extension->ReleaseGpu();
    foreignGpu_.clear();
    selectedOwner_ = nullptr;
    selectedExtra_ = -1;
    ReleaseThumbnailTargets();
    ReleaseGameLook();
    sections_.clear();
    // The selection names GPU slots of the world being dropped.
    selection_.clear();
    collisionSelection_.clear();
    collisionDrawn_.clear();
    uploadedSections_ = 0;
    unresolvedInstances_ = 0;
    // The visible set names sections of the world being dropped.
    visibleSections_.clear();
    visibleFrom_ = {};
    haveVisible_ = false;
    visibleZoneId_ = 0;
    visibleZoneChanged_ = 0.0;
    textures_.SetWorld(nullptr);
    if (worldSkyVertices_) { worldSkyVertices_->Release(); worldSkyVertices_ = nullptr; }
    if (worldSkyIndices_) { worldSkyIndices_->Release(); worldSkyIndices_ = nullptr; }
    worldSkyBands_ = 0;
    minimapPreview_.Release();   // MINIMAP PREVIEW (temporary)
    world_  = nullptr;
    bounds_ = {};
    stats_  = {};
}

void WorldRenderer::SyncWithWorld(const nfs::World& world) {
    if (!context_ || !context_->IsValid()) return;
    objectByHash_.clear();

    if (world_ != &world) {
        sections_.clear();
        uploadedSections_ = 0;
        bounds_ = {};
        world_ = &world;
        textures_.SetWorld(&world);
        streamingModelReady_ = world.IsWorld() && streamingModel_.Parse(world.LayoutBytes());
    }

    // Keep a one-to-one mapping with the world's sections. A section may hold
    // scenery and no geometry of its own, and its instances still have to be
    // drawn, so an entry is kept even when there is nothing to upload.
    const auto& loaded = world.Loaded();
    sections_.resize(loaded.size());

    for (size_t i = uploadedSections_; i < loaded.size(); ++i) {
        SectionRender& target = sections_[i];
        target.alternate = loaded[i]->info && world.IsUnusedAlternate(*loaded[i]->info);
        if (loaded[i]->info) {
            target.name = loaded[i]->info->name;
            target.number = loaded[i]->info->number;
            target.tablePosition = loaded[i]->info->position;
            target.positioned = true;
        }

        auto gpu = std::make_unique<GpuSection>();
        if (gpu->Upload(context_->Device(), *loaded[i])) {
            target.libraryBounds = gpu->Bounds();
            target.gpu = std::move(gpu);
        }

        RefreshCollision(i);

        for (const nfs::SceneryInstance& instance : loaded[i]->scenery.instances)
            target.instanceBounds.Add(instance.worldBounds);

        bounds_.Add(target.instanceBounds);
    }
    uploadedSections_ = loaded.size();

    BuildInstances(world);

    u32 vertices = 0, triangles = 0;
    size_t instances = 0;
    for (const SectionRender& section : sections_) {
        if (section.gpu) {
            vertices  += section.gpu->VertexCount();
            triangles += section.gpu->TriangleCount();
        }
        instances += section.instances.size();
    }
    size_t collisionPolygons = 0, collisionBarriers = 0, collisionSections = 0;
    for (const SectionRender& section : sections_) {
        if (!section.collision) continue;
        collisionSections++;
        collisionPolygons += section.collision->PolygonCount();
        collisionBarriers += section.collision->BarrierCount();
    }
    LOG_INFO("collision: %zu sections, %zu polygons of which %zu barriers",
             collisionSections, collisionPolygons, collisionBarriers);

    size_t alternateSections = 0, alternateInstances = 0;
    for (const SectionRender& section : sections_) {
        if (!section.alternate) continue;
        alternateSections++;
        alternateInstances += section.instances.size();
    }

    LOG_INFO("uploaded %zu sections: %u vertices, %u triangles, %zu instances drawn "
             "(%zu unresolved, %zu redundant)",
             sections_.size(), vertices, triangles, instances,
             unresolvedInstances_, duplicateInstances_);
    LOG_INFO("  of those, %zu sections holding %zu instances are the alternate set",
             alternateSections, alternateInstances);
}

void WorldRenderer::BuildInstances(const nfs::World& world) {
    unresolvedInstances_ = 0;
    duplicateInstances_  = 0;
    foreignGpu_.clear();

    // Where each model ended up on the GPU.
    std::unordered_map<u32, std::pair<u32, u32>> modelIndex;
    std::unordered_set<u64> placements;

    for (size_t s = 0; s < sections_.size(); ++s) {
        const GpuSection* gpu = sections_[s].gpu.get();
        if (!gpu) continue;
        const auto& objects = gpu->Objects();
        for (size_t o = 0; o < objects.size(); ++o) {
            if (objects[o].source)
                modelIndex.emplace(objects[o].source->hash, std::make_pair((u32)s, (u32)o));
        }
    }

    const auto& loaded = world.Loaded();
    for (size_t i = 0; i < loaded.size(); ++i) {
        const nfs::Scenery& scenery = loaded[i]->scenery;
        std::vector<GpuInstance>& out = sections_[i].instances;
        out.clear();
        out.reserve(scenery.instances.size());

        const auto& removed = loaded[i]->removedNatives;
        for (size_t n = 0; n < scenery.instances.size(); ++n) {
            const nfs::SceneryInstance& instance = scenery.instances[n];
            if (std::find(removed.begin(), removed.end(),
                          std::make_pair(instance.sceneryNumber, instance.record)) != removed.end())
                continue;
            const u32 modelHash = scenery.ModelHash(instance);
            const auto it = modelIndex.find(modelHash);
            if (it == modelIndex.end()) {
                unresolvedInstances_++;
                continue;
            }

            const nfs::SolidObject* model =
                sections_[it->second.first].gpu->Objects()[it->second.second].source;

            // Every record places its mesh, with no special case for terrain.
            //
            // There was one here, built on the belief that section Q38 listed the
            // same ground tile six times twelve units apart. It did not: those
            // records named trees and lamp posts, and a missing index base was
            // resolving them to terrain. With the base applied the duplication is
            // gone, and a special case that skipped the record took the asphalt
            // out from under the lane markings.
            const Mat4 transform = instance.transform;

            // Deduplicated on the transform the mesh is actually drawn with.
            // Neighbouring sections list the same shared prop, and a
            // pre-placed mesh is named by every record that describes it, so
            // without this the ground would still be drawn once per record.
            if (!placements.insert(nfs::PlacementKey(modelHash, transform)).second) {
                duplicateInstances_++;
                continue;
            }

            GpuInstance gpuInstance;
            gpuInstance.section     = it->second.first;
            gpuInstance.object      = it->second.second;
            gpuInstance.transform   = transform;
            gpuInstance.mirrored    = Determinant3x3(transform) < 0.0f;
            gpuInstance.native      = (int)n;
            gpuInstance.guid        = instance.guid;
            gpuInstance.sceneryNumber = instance.sceneryNumber;
            if (instance.infoIndex < scenery.infos.size()) {
                const nfs::SceneryInfo& info = scenery.infos[instance.infoIndex];
                gpuInstance.radius = info.radius;
                gpuInstance.identityPlaced = (instance.flags & 0x100) != 0;
                u32 reflectHash = 0;
                if (info.raw.size() >= 0x2C) std::memcpy(&reflectHash, info.raw.data() + 0x28, 4);
                const auto reflected = reflectHash ? modelIndex.find(reflectHash) : modelIndex.end();
                if ((instance.flags & 0x40) && !(instance.flags & 0x410) && reflected != modelIndex.end()) {
                    gpuInstance.reflects = true;
                    gpuInstance.reflectSection = reflected->second.first;
                    gpuInstance.reflectObject = reflected->second.second;
                }
            }

            // Built from the mesh, not taken from the instance record.
            //
            // The pair of vectors at the start of that record is not the
            // object's box: for the airport parking building the mesh is
            // 75x173x12 while the record's pair spans 8x8x21, and no scale
            // reconciles the two. Their midpoint is the instance position, so
            // they are something else. Culling against them decides what to
            // draw from a number that has nothing to do with where the mesh
            // actually is.
            if (model && model->bounds.Valid()) {
                for (int corner = 0; corner < 8; ++corner) {
                    const Vec3 point{
                        (corner & 1) ? model->bounds.max.x : model->bounds.min.x,
                        (corner & 2) ? model->bounds.max.y : model->bounds.min.y,
                        (corner & 4) ? model->bounds.max.z : model->bounds.min.z };
                    gpuInstance.worldBounds.Add(transform.TransformPoint(point));
                }
            }
            out.push_back(gpuInstance);
        }

        // The editor's placements, never deduplicated: each was put there on
        // purpose, even on top of something else.
        const auto& edited = loaded[i]->placements;
        for (size_t p = 0; p < edited.size(); ++p) {
            const nfs::PlacedInstance& placed = edited[p].instance;
            auto it = modelIndex.find(placed.modelHash);
            if (it == modelIndex.end()) {
                // A model from outside the region (nfs/ForeignModels.h),
                // drawn from an extension's copy until a save brings it in.
                const GpuSection* gpu = nullptr;
                const GpuObject* object = nullptr;
                for (RenderExtension* extension : extensions_)
                    if (extension->FindModel(placed.modelHash, gpu, object)) break;
                if (!gpu || !object || gpu->Objects().empty()) { unresolvedInstances_++; continue; }
                size_t slot = std::find(foreignGpu_.begin(), foreignGpu_.end(), gpu) - foreignGpu_.begin();
                if (slot == foreignGpu_.size()) foreignGpu_.push_back(gpu);
                it = modelIndex.emplace(placed.modelHash,
                                        std::make_pair((u32)(kForeignSection + slot),
                                                       (u32)(object - gpu->Objects().data()))).first;
            }

            const nfs::SolidObject* model = GpuOf(it->second.first)->Objects()[it->second.second].source;

            GpuInstance gpuInstance;
            gpuInstance.section   = it->second.first;
            gpuInstance.object    = it->second.second;
            gpuInstance.transform = placed.transform;
            gpuInstance.mirrored  = Determinant3x3(placed.transform) < 0.0f;
            gpuInstance.placement = (int)p;
            if (model && model->bounds.Valid())
                gpuInstance.worldBounds = Transform(model->bounds, placed.transform);
            out.push_back(gpuInstance);
            sections_[i].instanceBounds.Add(gpuInstance.worldBounds);
        }
    }
    size_t reflecting = 0, total = 0;
    for (const SectionRender& section : sections_) {
        total += section.instances.size();
        for (const GpuInstance& instance : section.instances) reflecting += instance.reflects;
    }
    LOG_INFO("road reflection: %zu of %zu placements reflect (flag 0x40, SceneryInfo +0x28 model)", reflecting, total);
}

void WorldRenderer::RebuildInstances(const nfs::World& world) {
    if (world_ != &world) return;
    BuildInstances(world);
    if (world.IsWorld()) {
        for (size_t i = 0; i < sections_.size(); ++i) {
            const auto& section = *world.Loaded()[i];
            if (sections_[i].hasPlacementCollision || !section.placements.empty() ||
                !section.nfswEditedScenery.empty() ||
                (!section.removedNatives.empty() && !section.smackableScenery.empty()) ||
                std::any_of(section.smackableLinks.begin(), section.smackableLinks.end(),
                    [](const auto& link) { return link.halfSizes.has_value(); }))
                sections_[i].collisionPreviewDirty = true;
        }
    }
    ++hiddenVersion_;   // the per-instance flags follow the new lists
}

void WorldRenderer::UpdatePlacement(u32 loadedSection, int placement, const Mat4& transform) {
    if (loadedSection >= sections_.size()) return;
    if (world_ && world_->IsWorld()) sections_[loadedSection].collisionPreviewDirty = true;
    for (GpuInstance& instance : sections_[loadedSection].instances) {
        if (instance.placement != placement) continue;
        const GpuSection* owner = GpuOf(instance.section);
        const nfs::SolidObject* model =
            owner && instance.object < owner->Objects().size() ? owner->Objects()[instance.object].source
                                                                : nullptr;
        instance.transform = transform;
        instance.mirrored  = Determinant3x3(transform) < 0.0f;
        if (model && model->bounds.Valid()) {
            instance.worldBounds = Transform(model->bounds, transform);
            sections_[loadedSection].instanceBounds.Add(instance.worldBounds);
        }
        for (PickResult& picked : selection_) {
            if (picked.loadedSection != loadedSection || picked.placement != placement) continue;
            picked.transform = transform;
            picked.worldBounds = instance.worldBounds;
        }
    }
}

// The other winding, for placements that mirror their mesh.
DWORD OppositeCull(DWORD mode) {
    return mode == D3DCULL_CW ? (DWORD)D3DCULL_CCW : (DWORD)D3DCULL_CW;
}

void WorldRenderer::ApplyRenderStates() {
    IDirect3DDevice9* device = context_->Device();

    device->SetRenderState(D3DRS_ZENABLE, D3DZB_TRUE);
    device->SetRenderState(D3DRS_ZWRITEENABLE, TRUE);
    device->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);

    // Fences, foliage, railings and signs are cut out through the texture alpha
    // rather than modelled, so without an alpha test they show up as opaque
    // rectangles. A plain test avoids the draw ordering that blending would
    // require, and textures with no alpha are fully opaque and pass anyway.
    device->SetRenderState(D3DRS_ALPHATESTENABLE, TRUE);
    device->SetRenderState(D3DRS_ALPHAFUNC, D3DCMP_GREATEREQUAL);
    device->SetRenderState(D3DRS_ALPHAREF, 128);

    // Stands in for a missing texture, and supplies opaque alpha wherever the
    // texture alpha is not the right source.
    device->SetRenderState(D3DRS_TEXTUREFACTOR, kMissingTextureColor);

    // Cull clockwise, not counter-clockwise.
    //
    // Direct3D culls by the winding a triangle ends up with on screen, and the
    // right-handed view and projection used here flip that parity compared with
    // the left-handed matrices the API assumes by default. The track meshes are
    // wound so that D3DCULL_CW discards their back faces; D3DCULL_CCW discards
    // the front ones instead, which leaves every surface in the city inside out.
    device->SetRenderState(D3DRS_CULLMODE,
                           backfaceCulling_ ? cullMode_ : D3DCULL_NONE);

    device->SetRenderState(D3DRS_FILLMODE,
                           mode_ == RenderMode::Wireframe ? D3DFILL_WIREFRAME : D3DFILL_SOLID);

    // Lighting is already baked into the vertex colours, so the dynamic lighting
    // pipeline stays off and the colours are used as they are.
    device->SetRenderState(D3DRS_LIGHTING, FALSE);
    device->SetRenderState(D3DRS_SPECULARENABLE, FALSE);

    device->SetTextureStageState(1, D3DTSS_COLOROP, D3DTOP_DISABLE);
    device->SetTextureStageState(1, D3DTSS_ALPHAOP, D3DTOP_DISABLE);

    device->SetSamplerState(0, D3DSAMP_ADDRESSU, D3DTADDRESS_WRAP);
    device->SetSamplerState(0, D3DSAMP_ADDRESSV, D3DTADDRESS_WRAP);
    device->SetSamplerState(0, D3DSAMP_MINFILTER, D3DTEXF_LINEAR);
    device->SetSamplerState(0, D3DSAMP_MAGFILTER, D3DTEXF_LINEAR);
    device->SetSamplerState(0, D3DSAMP_MIPFILTER, D3DTEXF_LINEAR);
    device->SetSamplerState(0, D3DSAMP_MAXANISOTROPY, 4);

    device->SetFVF(kWorldVertexFvf);
}

void WorldRenderer::BindTextureStage(IDirect3DDevice9* device, const CachedTexture& cached) {
    IDirect3DTexture9* texture = cached.texture;
    device->SetTexture(0, texture);

    // Colour comes from the texture when it decoded and from the flat factor
    // colour when it did not, so an undecodable texture is obvious rather than
    // invisible. The lighting-only modes ignore both.
    const DWORD colorSource = texture ? (DWORD)D3DTA_TEXTURE : (DWORD)D3DTA_TFACTOR;

    switch (mode_) {
        case RenderMode::Textured:
            device->SetTextureStageState(0, D3DTSS_COLOROP,   D3DTOP_MODULATE2X);
            device->SetTextureStageState(0, D3DTSS_COLORARG1, colorSource);
            device->SetTextureStageState(0, D3DTSS_COLORARG2, D3DTA_DIFFUSE);
            break;

        case RenderMode::Unlit:
            device->SetTextureStageState(0, D3DTSS_COLOROP,   D3DTOP_SELECTARG1);
            device->SetTextureStageState(0, D3DTSS_COLORARG1, colorSource);
            break;

        default:
            device->SetTextureStageState(0, D3DTSS_COLOROP,   D3DTOP_SELECTARG1);
            device->SetTextureStageState(0, D3DTSS_COLORARG1, D3DTA_DIFFUSE);
            break;
    }

    // Alpha is opacity only where the texture's header says so. An opaque
    // material takes the solid alpha of the factor colour, so the alpha test
    // always passes: the road textures keep gloss in their alpha, and testing
    // against it erases the asphalt and leaves only the lane markings.
    const bool alphaIsOpacity = texture && cached.blend != SurfaceBlend::Opaque;
    device->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_SELECTARG1);
    device->SetTextureStageState(0, D3DTSS_ALPHAARG1,
                                 alphaIsOpacity ? D3DTA_TEXTURE : D3DTA_TFACTOR);
}

bool WorldRenderer::DrawObject(IDirect3DDevice9* device, const GpuObject& object,
                               RenderStats& stats, DrawPass pass) {
    // SHD_/RFL_ meshes are the engine's shadow and reflection proxies, not
    // world geometry; drawing them puts a second shell over the real building.
    if (world_ && world_->IsWorld() && object.source && object.source->IsShadow() &&
        !showProxies_)
        return false;
    if (glActive_) return DrawObjectGameLook(device, object, stats, pass);
    const bool wantsTexture = mode_ == RenderMode::Textured || mode_ == RenderMode::Unlit;
    bool other = false;

    for (const GpuDraw& draw : object.draws) {
        if (wantsTexture) {
            const CachedTexture& texture = textures_.Get(device, AnimatedTexture(draw.textureHash));
            // A texture that failed to decode is drawn as the grey stand-in,
            // which is solid whatever the header asked for.
            const bool translucent = texture.texture && texture.Translucent();
            if (translucent != (pass == DrawPass::Translucent)) { other = true; continue; }

            if (pass == DrawPass::Translucent)
                device->SetRenderState(D3DRS_DESTBLEND,
                                       texture.blend == SurfaceBlend::Additive
                                           ? D3DBLEND_ONE : D3DBLEND_INVSRCALPHA);
            BindTextureStage(device, texture);
            ScrollTextureStage(device, texture);
        } else if (pass == DrawPass::Translucent) {
            continue;   // the lighting-only views draw everything solid
        }

        device->DrawIndexedPrimitive(D3DPT_TRIANGLELIST,
                                     (INT)object.baseVertex,
                                     draw.minVertex,
                                     draw.vertexCount,
                                     draw.startIndex,
                                     draw.primitiveCount);

        stats.drawCalls++;
        stats.triangles += draw.primitiveCount;
    }
    // Nothing drawn after this one should inherit the scroll (ImGui included).
    if (scrolling_) ScrollTextureStage(device, CachedTexture{});
    return other;
}

// The game's texture scrolling (TextureInfo::GetScroll): the fraction of
// time * speed, time counted in whole steps for the stepped kind.
static void ScrollOffset(const CachedTexture& texture, double clock, float& s, float& t) {
    double time = clock;
    if (texture.scrollType == 2) time = std::floor(time / texture.scrollStep);
    const auto fraction = [](double v) { return (float)(v - std::floor(v)); };
    s = fraction(time * texture.scrollS);
    t = fraction(time * texture.scrollT);
}

void WorldRenderer::ScrollTextureStage(IDirect3DDevice9* device, const CachedTexture& texture) {
    const bool scroll = animate_ && texture.scrollType != 0;
    if (!scroll) {
        if (scrolling_) {
            device->SetTextureStageState(0, D3DTSS_TEXTURETRANSFORMFLAGS, D3DTTFF_DISABLE);
            scrolling_ = false;
        }
        return;
    }
    D3DMATRIX m{};
    m._11 = m._22 = m._33 = m._44 = 1.0f;
    // Two texture coordinates: the third row moves them.
    ScrollOffset(texture, animationClock_, m._31, m._32);
    device->SetTransform(D3DTS_TEXTURE0, &m);
    if (!scrolling_) device->SetTextureStageState(0, D3DTSS_TEXTURETRANSFORMFLAGS, D3DTTFF_COUNT2);
    scrolling_ = true;
}

void WorldRenderer::QueueTranslucent(const GpuObject& object, size_t section, const Mat4& transform,
                                     bool mirrored, const Aabb& bounds, const Vec3& eye) {
    const Vec3 centre = bounds.Valid() ? (bounds.min + bounds.max) * 0.5f
                                       : Vec3{ transform.m[3][0], transform.m[3][1], transform.m[3][2] };
    const Vec3 offset = centre - eye;
    translucent_.push_back({ &object, section, transform, mirrored, Dot(offset, offset) });
}

void WorldRenderer::BeginTranslucentStates(IDirect3DDevice9* device) {
    // Tested and blended both: a texel with no opacity at all is skipped
    // outright, and the rest is mixed in without touching the depth buffer, so
    // a glow never hides what stands behind it.
    device->SetRenderState(D3DRS_ALPHAREF, 4);
    device->SetRenderState(D3DRS_ALPHABLENDENABLE, TRUE);
    device->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_SRCALPHA);
    device->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA);
    device->SetRenderState(D3DRS_ZWRITEENABLE, FALSE);
}

void WorldRenderer::EndTranslucentStates(IDirect3DDevice9* device) {
    device->SetRenderState(D3DRS_ZWRITEENABLE, TRUE);
    device->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
    device->SetRenderState(D3DRS_ALPHAREF, 128);
}

void WorldRenderer::DrawTranslucent(IDirect3DDevice9* device, RenderStats& stats) {
    if (translucent_.empty()) return;

    // Farthest first, so nearer smoke and tape blend over what is behind them.
    std::sort(translucent_.begin(), translucent_.end(),
              [](const TranslucentObject& a, const TranslucentObject& b) {
                  return a.distance > b.distance;
              });

    BeginTranslucentStates(device);
    for (const TranslucentObject& item : translucent_) {
        const GpuSection* itemGpu = GpuOf(item.section);
        if (!itemGpu) continue;
        const GpuSection& gpu = *itemGpu;
        device->SetStreamSource(0, gpu.VertexBuffer(), 0, sizeof(nfs::SolidVertex));
        device->SetIndices(gpu.IndexBuffer());
        if (backfaceCulling_)
            device->SetRenderState(D3DRS_CULLMODE,
                                   item.mirrored ? OppositeCull(cullMode_) : cullMode_);
        const D3DMATRIX world = ToD3D(item.transform);
        device->SetTransform(D3DTS_WORLD, &world);
        DrawObject(device, *item.object, stats, DrawPass::Translucent);
    }
    EndTranslucentStates(device);
    device->SetRenderState(D3DRS_CULLMODE, backfaceCulling_ ? cullMode_ : D3DCULL_NONE);
    translucent_.clear();
}

void WorldRenderer::Render(const Camera& camera) {
    if (!context_ || !context_->IsValid()) return;

    const double start = SecondsNow();

    IDirect3DDevice9* device = context_->Device();
    const float aspect = context_->Height()
                             ? (float)context_->Width() / (float)context_->Height()
                             : 1.0f;

    const Mat4 view       = camera.ViewMatrix();
    const Mat4 projection = camera.ProjectionMatrix(aspect);

    // Game look: the road reflection is drawn first, into its own target, as
    // eDisplayFrame does before the player views.
    const bool gameLook = gameLook_ && mode_ != RenderMode::Wireframe && EnsureGameLook(device);
    if (!worldOnly_) {
        glReflectionValid_ = false;
        if (gameLook) {
            UpdateGameZone(camera.Position());
            RenderReflection(device, camera, view, projection);
        }
    }

    const D3DMATRIX d3dView       = ToD3D(view);
    const D3DMATRIX d3dProjection = ToD3D(projection);
    device->SetTransform(D3DTS_VIEW, &d3dView);
    device->SetTransform(D3DTS_PROJECTION, &d3dProjection);

    Frustum frustum;
    frustum.Build(view * projection);

    // A mirrored view swaps which winding faces the camera.
    cullMode_ = camera.ViewIsMirrored() ? D3DCULL_CCW : D3DCULL_CW;

    ApplyRenderStates();
    animationClock_ = start;
    if (gameLook) {
        BeginGameLook(device, view, projection, !worldOnly_, camera.Position());
    }

    // The sky first, behind everything.
    RenderStats skyStats;
    if (showSky_) DrawSky(device, camera, skyStats);

    // What the game would draw from here. Falling back to distance keeps the
    // viewport usable above the rooftops, where no zone claims the camera.
    bool usingVisibility = false;
    if (streaming_ && world_) {
        const Vec3 eye = camera.Position();
        const float moved = (eye.x - visibleFrom_.x) * (eye.x - visibleFrom_.x) +
                            (eye.y - visibleFrom_.y) * (eye.y - visibleFrom_.y);
        if (!haveVisible_ || moved > 4.0f) {
            // The zones do not tile the ground without gaps, and a camera that
            // steps on a seam should not empty the world for a frame. Keeping
            // the zone the camera is already in -- and keeping the last set
            // when no zone claims the point -- is what the game's own streaming
            // does, and it stops the view flickering.
            const std::vector<u32> previous = visibleSections_;
            const u32 previousHash = HashSections(previous);
            // The hold/dwell rules below are the NFS World streaming behaviour;
            // UG2 keeps its original immediate zone switch.
            const bool holdsZone = world_->IsWorld() && visibleZoneId_ != 0 &&
                world_->Visibility().ZoneContains(visibleZoneId_, eye.x, eye.y);

            std::vector<u32> found;
            int listZone = holdsZone ? (int)visibleZoneId_ : -1;
            bool claimed = holdsZone;
            bool fallback = false;
            size_t zoneListCount = holdsZone ? visibleSections_.size() : 0;
            const int modelZone = streamingModelReady_
                ? streamingModel_.SectionAt(eye, animationClock_, world_->Visibility()) : -1;
            if (!holdsZone) {
                if (modelZone >= 0) {
                    const auto* list = world_->Visibility().ListOf((u32)modelZone);
                    if (list && !list->empty()) {
                        found = *list;
                        listZone = modelZone;
                        claimed = true;
                    }
                }
                if (!claimed) claimed = world_->Visibility().VisibleAt(eye, found, &listZone);
                zoneListCount = found.size();
                if (!claimed && gameZoneFallback_ && world_->IsWorld() && !haveVisible_) {
                    // WhatSectionsShouldWeDraw: with no zone, the game draws the
                    // sections whose numeric part is below 10. Only the first
                    // list takes it: a camera crossing a seam keeps the zone it
                    // came from instead of collapsing to the fallback set.
                    for (const nfs::Section& section : world_->Table().Sections())
                        if (section.number % 1000 < 10) found.push_back(section.number);
                    fallback = !found.empty();
                    claimed = fallback;
                }
            }
            const size_t listedCount = found.size();
            const int zone = world_->Visibility().ZoneAt(eye.x, eye.y);
            // A second zone on the other side of a hard edge must not replace
            // the list the moment the camera crosses: wait for the dwell so a
            // camera hovering on the seam does not strobe between both sets.
            const bool deferred = world_->IsWorld() && claimed && !fallback && haveVisible_ &&
                visibleZoneId_ != 0 && (u32)listZone != visibleZoneId_ &&
                animationClock_ - visibleZoneChanged_ < kZoneSwitchDwellSeconds;
            if (claimed && !holdsZone && !deferred && world_->IsWorld()) {
                // The same function appends the camera's own section, its +400
                // sibling, and the Z0 global layer (25000) while it is loaded.
                const int appendZone = listZone >= 0 ? listZone
                    : (zone >= 0 ? (int)world_->Visibility().Zones()[zone].id : -1);
                if (appendZone >= 0) {
                    found.push_back((u32)appendZone);
                    found.push_back((u32)appendZone + 400);
                }
                if (world_->Table().FindByNumber(25000)) found.push_back(25000);
            }
            if (claimed && !holdsZone && !deferred) {
                std::sort(found.begin(), found.end());
                found.erase(std::unique(found.begin(), found.end()), found.end());
                visibleSections_.swap(found);
                haveVisible_ = true;
                visibleZoneId_ = fallback ? 0 : (u32)listZone;
                visibleZoneChanged_ = animationClock_;
            }
            visibleFrom_ = eye;

            const int zoneId = (holdsZone || deferred)
                ? (int)visibleZoneId_
                : (zone >= 0 ? (int)world_->Visibility().Zones()[zone].id : -1);
            const size_t appended = (!holdsZone && !deferred && claimed)
                ? visibleSections_.size() - listedCount : 0;
            const u32 visibleHash = HashSections(visibleSections_);

            // Logged on every change of zone, list or fallback state, not every
            // frame: which zone the camera is in decides the whole picture, and
            // when the picture looks wrong this is the first thing worth knowing.
            static size_t lastCount = (size_t)-1;
            static u32    lastHash = 0;
            static int    lastZone = -2;
            static bool   lastClaimed = false;
            static bool   lastFallback = false;
            static bool   lastDeferred = false;
            if (visibleSections_.size() != lastCount || visibleHash != lastHash ||
                zoneId != lastZone || haveVisible_ != lastClaimed || fallback != lastFallback ||
                deferred != lastDeferred) {
                const char* mode = deferred
                    ? "new zone deferred"
                    : claimed
                          ? (fallback
                                 ? (listZone >= 0 ? "zone list empty, game fallback n%1000<10"
                                                  : "no zone, game fallback n%1000<10")
                                 : "inside a zone")
                          : haveVisible_
                                ? "no new zone, keeping the last list"
                                : (world_->IsWorld()
                                       ? (gameZoneFallback_ ? "no zone, game fallback n%1000<10"
                                                            : "no zone, drawing all sections")
                                       : "no zone, using distance");
                LOG_INFO("camera (%.0f, %.0f, %.0f): %s, %zu sections visible "
                         "(zoneAt %d, listZone %d/%zu, fallback %d, deferred %d, appended %zu, "
                         "was %zu/%08X, kept %zu, now %08X, moved %.1f)",
                         eye.x, eye.y, eye.z, mode, visibleSections_.size(),
                         zoneId, listZone, zoneListCount, fallback ? 1 : 0, deferred ? 1 : 0,
                         appended, previous.size(), previousHash,
                         KeptSections(previous, visibleSections_), visibleHash, std::sqrt(moved));
                lastCount = visibleSections_.size();
                lastHash = visibleHash;
                lastZone = zoneId;
                lastClaimed = haveVisible_;
                lastFallback = fallback;
                lastDeferred = deferred;
            }
        }
        usingVisibility = haveVisible_ && !visibleSections_.empty();
    }

    const bool wantsTexture = mode_ == RenderMode::Textured || mode_ == RenderMode::Unlit;
    if (!wantsTexture && !glActive_) BindTextureStage(device, CachedTexture{});
    translucent_.clear();
    const Vec3 eye = camera.Position();

    RenderStats stats;
    stats.sectionsTotal = (u32)sections_.size();

    // Which section's buffers are currently bound. Instances reach into other
    // sections, so the binding is tracked rather than set once per section.
    size_t boundSection = (size_t)-1;
    auto bind = [&](size_t index) {
        if (boundSection == index) return;
        const GpuSection& gpu = *GpuOf((u32)index);
        device->SetStreamSource(0, gpu.VertexBuffer(), 0, sizeof(nfs::SolidVertex));
        device->SetIndices(gpu.IndexBuffer());
        boundSection = index;
    };

    std::vector<size_t> collisionVisible;
    const auto* sceneryAnimations = (world_ && world_->IsWorld() &&
                                     !world_->SceneryAnimations().empty())
        ? &world_->SceneryAnimations() : nullptr;

    for (size_t i = 0; i < sections_.size(); ++i) {
        const SectionRender& section = sections_[i];

        if (section.gpu) stats.objectsTotal += (u32)section.gpu->Objects().size();
        stats.instancesTotal += (u32)section.instances.size();

        if (section.alternate && !showAlternate_) continue;
        if (GameHidesSection(section.number)) continue;   // SetGameDrawFilter

        // Collision is culled on its own footprint: a section's instances can
        // sit far from its ground, and the section may place nothing at all.
        if (showCollision_ && section.collisionPreviewDirty) RefreshCollision(i);
        if (showCollision_ && section.collision) {
            const Aabb& bounds = section.collision->Bounds();
            bool keep = true;
            if (usingVisibility)
                keep = std::binary_search(visibleSections_.begin(), visibleSections_.end(),
                                          section.number);
            else if (DistanceCulls()) {
                const Vec3 centre = (bounds.min + bounds.max) * 0.5f;
                const float dx = centre.x - camera.Position().x;
                const float dy = centre.y - camera.Position().y;
                keep = dx * dx + dy * dy <= streamRadius_ * streamRadius_;
            }
            if (keep && culling_ && !frustum.Intersects(bounds)) keep = false;
            if (keep) collisionVisible.push_back(i);
        }

        const Aabb cullBounds = section.CullBounds(showLibrary_);
        const SectionPreview* preview = Previewed(section.number);   // SetSectionPreviews: bounds are stale
        const bool previewed = preview != nullptr;

        if (previewed) {
        } else if (usingVisibility) {
            if (!std::binary_search(visibleSections_.begin(), visibleSections_.end(),
                                    section.number))
                continue;
        }
        // Distance is measured on the ground plane: the camera is often well
        // above the street, and counting that height in would drop the
        // surrounding blocks the moment anyone looked down at the city.
        else if (DistanceCulls() && cullBounds.Valid()) {
            const Vec3 centre = (cullBounds.min + cullBounds.max) * 0.5f;
            const float dx = centre.x - camera.Position().x;
            const float dy = centre.y - camera.Position().y;
            if (dx * dx + dy * dy > streamRadius_ * streamRadius_) continue;
        }
        if (!previewed && culling_ && cullBounds.Valid() && !frustum.Intersects(cullBounds)) continue;
        stats.sectionsVisible++;

        // The stored meshes, at the storage transforms in their headers. This is
        // the model library and it is not part of the world; see LibraryVisible.
        if (showLibrary_ && section.gpu) {
            const auto& objects = section.gpu->Objects();
            for (const GpuObject& object : objects) {
                if (BackdropHidden(object)) continue;
                if (HiddenBuilding(object)) continue;   // MINIMAP PREVIEW (temporary)
                if (HiddenModel(object)) continue;
                // An object with no stored bounds cannot be culled, only drawn:
                // testing its empty-box sentinel would reject it every frame.
                if (culling_ && object.worldBounds.Valid() &&
                    !frustum.Intersects(object.worldBounds)) continue;

                stats.objectsVisible++;
                bind(i);

                const D3DMATRIX world = ToD3D(object.transform);
                device->SetTransform(D3DTS_WORLD, &world);
                if (DrawObject(device, object, stats))
                    QueueTranslucent(object, i, object.transform, false, object.worldBounds, eye);
            }
        }

        if (!showScenery_) continue;

        const std::vector<char>& hiddenHere = HiddenFlags(i);
        for (size_t k = 0; k < section.instances.size(); ++k) {
            const GpuInstance& instance = section.instances[k];
            if (hiddenHere[k]) continue;   // hidden in the editor
            const GpuObject& object = GpuOf(instance.section)->Objects()[instance.object];
            if (InstanceHolidayHidden(section.number, instance.native, &object)) continue;
            // A file can embed another section's scenery chunk (chunk 6009
            // inside STREAML5RA_6409.BUN). That chunk is only drawn when its
            // own number is in the visible list, not the file's section.
            if (world_ && world_->IsWorld() && usingVisibility && instance.sceneryNumber &&
                instance.sceneryNumber != section.number &&
                !std::binary_search(visibleSections_.begin(), visibleSections_.end(),
                                    instance.sceneryNumber))
                continue;

            const nfs::NfswObjectAnimation* animation = nullptr;
            if (sceneryAnimations && instance.guid) {
                const auto found = sceneryAnimations->find(instance.guid);
                if (found != sceneryAnimations->end() && !found->second.frames.empty())
                    animation = &found->second;
            }
            // An animated instance leaves its shipped bounds, so frustum culling
            // against them would drop it over most of the route.
            if (!previewed && !animation && culling_ && instance.worldBounds.Valid() &&
                !frustum.Intersects(instance.worldBounds)) continue;

            if (BackdropHidden(object)) continue;
            if (BackdropOutOfZone(object, section.number)) continue;   // Game look: not in the camera's zone list
            if (HiddenBuilding(object)) continue;   // MINIMAP PREVIEW (temporary)
            if (HiddenModel(object)) continue;
            if (GameHidesModel(object)) continue;   // SetGameDrawFilter: no loaded pack holds the model

            stats.instancesVisible++;
            bind(instance.section);

            Mat4 posed = instance.transform;
            if (animation) {
                const size_t frame = animate_
                    ? (size_t)(animationClock_ * kSceneryAnimationTicksPerSecond * animation->speed) %
                          animation->frames.size()
                    : 0;
                posed = animation->frames[frame];
            }
            const Mat4 placed = previewed ? posed * preview->delta : posed;
            const bool mirrored = animation ? Determinant3x3(placed) < 0.0f : instance.mirrored;

            if (backfaceCulling_)
                device->SetRenderState(D3DRS_CULLMODE,
                                       mirrored ? OppositeCull(cullMode_) : cullMode_);

            const D3DMATRIX world = ToD3D(placed);
            device->SetTransform(D3DTS_WORLD, &world);
            if (DrawObject(device, object, stats))
                QueueTranslucent(object, instance.section, placed, mirrored,
                                 previewed ? Transform(instance.worldBounds, preview->delta) : instance.worldBounds, eye);
        }
    }

    // MINIMAP PREVIEW (temporary)
    if (showMinimap_) {
        minimapPreview_.Draw(device, backfaceCulling_ ? cullMode_ : D3DCULL_NONE,
                             mode_ == RenderMode::Wireframe);
        device->SetFVF(kWorldVertexFvf);
    }

    if (showScenery_) DrawObjectAnimations(device, usingVisibility, eye, stats);
    DrawTranslucent(device, stats);
    if (glActive_) EndGameLook(device);
    if (worldOnly_) {
        // RenderWorldOnly: the world is done; nothing of the editor's goes into a reflection.
        device->SetTexture(0, nullptr);
        return;
    }
    for (RenderExtension* extension : extensions_) extension->DrawWithWorld(device, *this);
    DrawSelection(device);
    if (showCollision_) DrawCollision(device, collisionVisible);
    collisionDrawn_ = showCollision_ ? collisionVisible : std::vector<size_t>{};
    for (RenderExtension* extension : extensions_) extension->DrawOverlay(device, camera, *this);
    DrawToolOverlay(device);
    DrawGizmo(device, camera);

    device->SetTexture(0, nullptr);

    stats.frameMilliseconds = (float)((SecondsNow() - start) * 1000.0);
    stats_ = stats;
}

void WorldRenderer::RenderWorldOnly(const Camera& camera) {
    if (worldOnly_) return;   // never nested
    // Render overwrites these for its own frame; the outer frame keeps going after this call.
    const DWORD outerCull = cullMode_;
    const bool outerStreaming = streaming_;
    worldOnly_ = true;
    streaming_ = false;       // the visible-set cache follows the main camera, not this one
    Render(camera);
    streaming_ = outerStreaming;
    worldOnly_ = false;
    cullMode_ = outerCull;
}

// ---------------------------------------------------------------------------
// Picking
// ---------------------------------------------------------------------------

void WorldRenderer::PickOne(const Ray& ray, const GpuObject& object, const Mat4& transform,
                            const Aabb& worldBounds, bool fromInstance, size_t sectionIndex,
                            u32 ownerSection, u32 ownerObject, PickResult& best,
                            int placement, int native) const {
    if (!object.source) return;
    if (fromInstance && InstanceHolidayHidden(sections_[sectionIndex].number, native, &object))
        return;
    if (placement >= 0 && placement == excludePlacement_ && sectionIndex == excludeSection_) return;
    // Pickable exactly when drawn: with the game look on, the panorama and
    // distant pieces are drawn as ordinary scenery even with the backdrop off.
    if (BackdropHidden(object)) return;
    if (world_ && world_->IsWorld() && object.source->IsShadow() && !showProxies_) return;
    if (HiddenBuilding(object)) return;   // MINIMAP PREVIEW (temporary)
    if (HiddenModel(object)) return;

    // A bounds test rejects almost everything; only survivors pay for triangles.
    // Objects with no bounds have to be tested directly.
    if (worldBounds.Valid() && IntersectRayAabb(ray, worldBounds) < 0.0f) return;

    const Mat4 toLocal = InverseAffine(transform);
    Ray local;
    local.origin    = toLocal.TransformPoint(ray.origin);
    local.direction = toLocal.TransformDirection(ray.direction);

    const nfs::SolidObject& source = *object.source;
    const size_t triangles = source.indices.size() / 3;

    for (size_t t = 0; t < triangles; ++t) {
        const u16 i0 = source.indices[t * 3 + 0];
        const u16 i1 = source.indices[t * 3 + 1];
        const u16 i2 = source.indices[t * 3 + 2];
        if (i0 >= source.vertices.size() || i1 >= source.vertices.size() ||
            i2 >= source.vertices.size())
            continue;

        const float hit = IntersectRayTriangle(local,
                                               source.vertices[i0].position,
                                               source.vertices[i1].position,
                                               source.vertices[i2].position);
        if (hit < 0.0f) continue;

        // The local ray is not unit length once a transform scales it, so the
        // distance has to be measured back in world space.
        const Vec3 localPoint = local.origin + local.direction * hit;
        const Vec3 worldPoint = transform.TransformPoint(localPoint);
        const float distance = Dot(worldPoint - ray.origin, ray.direction);
        if (distance <= 0.0f || (best.object && distance >= best.distance)) continue;

        best.object       = &source;
        best.transform    = transform;
        best.worldBounds  = worldBounds;
        best.distance     = distance;
        best.fromInstance = fromInstance;
        best.gpuSection   = ownerSection;
        best.gpuObject    = ownerObject;
        best.loadedSection = (u32)sectionIndex;
        best.placement    = placement;
        best.native       = native;
        best.sectionName.clear();

        if (world_ && sectionIndex < world_->Loaded().size()) {
            const nfs::Section* info = world_->Loaded()[sectionIndex]->info;
            if (info) best.sectionName = info->name;
        }
    }
}

// Checks that each instance really points at the model it claims.
//
// An instance names its model through an index into its section's info table,
// which turns into a hash, which turns into a mesh. A slip anywhere along that
// chain puts a real model, correctly placed, in a spot that belongs to another
// one -- the city fills with plausible objects in wrong places and loses the
// ones that should be there, which is exactly how it looks on screen.
//
// The file gives a way to test it: every instance stores its own world bounding
// box. Transforming the referenced model's own bounds by the instance matrix
// has to reproduce that box. Comparing the stored box with itself, which is
// what an earlier check did, proves nothing at all.
bool WorldRenderer::CheckReferences(const std::wstring& path) const {
    FILE* file = nullptr;
    if (::_wfopen_s(&file, path.c_str(), L"w") != 0 || !file) return false;

    std::fprintf(file, "# model hash centreError sizeRatio rawError rawRatio modelIsWorldSpace section\n");

    size_t tested = 0;
    std::vector<float> errors;
    for (const SectionRender& section : sections_) {
        for (const GpuInstance& instance : section.instances) {
            const GpuSection* owner = GpuOf(instance.section);
            if (!owner || instance.object >= owner->Objects().size()) continue;

            const nfs::SolidObject* model = owner->Objects()[instance.object].source;
            if (!model || !model->bounds.Valid() || !instance.worldBounds.Valid()) continue;

            // The eight corners, so a rotated box is measured by what it really
            // covers rather than by its axis-aligned extent before rotation.
            Aabb derived;
            for (int corner = 0; corner < 8; ++corner) {
                const Vec3 point{
                    (corner & 1) ? model->bounds.max.x : model->bounds.min.x,
                    (corner & 2) ? model->bounds.max.y : model->bounds.min.y,
                    (corner & 4) ? model->bounds.max.z : model->bounds.min.z };
                derived.Add(instance.transform.TransformPoint(point));
            }

            const Vec3 storedCentre  = (instance.worldBounds.min + instance.worldBounds.max) * 0.5f;
            const Vec3 derivedCentre = (derived.min + derived.max) * 0.5f;
            const Vec3 storedSize    = instance.worldBounds.max - instance.worldBounds.min;
            const Vec3 derivedSize   = derived.max - derived.min;

            const float centreError = Length(derivedCentre - storedCentre);
            const float storedSpan  = Length(storedSize);
            const float derivedSpan = Length(derivedSize);
            const float ratio = storedSpan > 0.001f ? derivedSpan / storedSpan : 0.0f;

            // The same comparison with the model's bounds left alone. Terrain
            // is stored already standing in the world, so for those the raw
            // box is the answer and transforming it again is the mistake --
            // measuring both says which models are which instead of assuming.
            const Vec3 rawCentre = (model->bounds.min + model->bounds.max) * 0.5f;
            const Vec3 rawSize   = model->bounds.max - model->bounds.min;
            const float rawError = Length(rawCentre - storedCentre);
            const float rawSpan  = Length(rawSize);
            const float rawRatio = storedSpan > 0.001f ? rawSpan / storedSpan : 0.0f;

            ++tested;
            // Only the centre is judged. The file's box is tighter than the one
            // that comes from enclosing a rotated model's transformed corners,
            // so a size ratio above one is the normal case for anything turned
            // off the axes and says nothing about whether the placement is right.
            // A threshold would only hide the shape of the error, so the whole
            // distribution is reported instead.
            errors.push_back(centreError < rawError ? centreError : rawError);

            std::fprintf(file, "%s %08X %.3f %.4f %.3f %.4f %d %s\n",
                         model->name.c_str(), model->hash, centreError, ratio,
                         rawError, rawRatio,
                         rawError < centreError ? 1 : 0,
                         section.name.empty() ? "?" : section.name.c_str());
        }
    }

    std::fclose(file);
    std::sort(errors.begin(), errors.end());
    const auto at = [&](double q) {
        return errors.empty() ? 0.0f : errors[(size_t)(q * (errors.size() - 1))];
    };
    LOG_INFO("reference check: %zu instances, distance from the box the file stores "
             "for them -- median %.2f, p90 %.2f, p99 %.2f, worst %.1f",
             tested, at(0.5), at(0.9), at(0.99), errors.empty() ? 0.0f : errors.back());
    return true;
}

bool WorldRenderer::DumpPlacements(const std::wstring& path) const {
    FILE* file = nullptr;
    if (::_wfopen_s(&file, path.c_str(), L"w") != 0 || !file) {
        LOG_ERROR("could not open the placement dump for writing");
        return false;
    }

    // One line per placement, so the file diffs cleanly against a list pulled
    // out of the running game. The position is the translation of the matrix
    // the renderer actually binds, not a recomputed one: a dump that took its
    // own path through the maths could agree with the game while the picture
    // still disagreed.
    std::fprintf(file, "# model hash x y z r00 r01 r02 r10 r11 r12 r20 r21 r22 source section sectionX sectionY alternate\n");

    size_t written = 0;
    for (size_t index = 0; index < sections_.size(); ++index) {
        const SectionRender& section = sections_[index];

        if (section.gpu) {
            for (const GpuObject& object : section.gpu->Objects()) {
                if (!object.source) continue;
                std::fprintf(file, "%s %08X %.3f %.3f %.3f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f library %s %.1f %.1f %d\n",
                             object.source->name.c_str(), object.source->hash,
                             object.transform.m[3][0], object.transform.m[3][1],
                             object.transform.m[3][2],
                             object.transform.m[0][0], object.transform.m[0][1],
                             object.transform.m[0][2], object.transform.m[1][0],
                             object.transform.m[1][1], object.transform.m[1][2],
                             object.transform.m[2][0], object.transform.m[2][1],
                             object.transform.m[2][2],
                             section.name.empty() ? "?" : section.name.c_str(),
                             section.tablePosition.x, section.tablePosition.y,
                             section.alternate ? 1 : 0);
                ++written;
            }
        }

        for (const GpuInstance& instance : section.instances) {
            const GpuSection* owner = GpuOf(instance.section);
            if (!owner || instance.object >= owner->Objects().size()) continue;

            const nfs::SolidObject* source = owner->Objects()[instance.object].source;
            if (!source) continue;

            // Where the mesh ends up, not the matrix's translation row. For a
            // pre-placed mesh the two are different things: the translation is
            // what remains after bringing the mesh back to its own origin, and
            // reading it as a position puts terrain in nonsense places.
            Vec3 centre = (source->bounds.min + source->bounds.max) * 0.5f;
            centre = instance.transform.TransformPoint(centre);

            std::fprintf(file, "%s %08X %.3f %.3f %.3f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f instance %s %.1f %.1f %d\n",
                         source->name.c_str(), source->hash,
                         centre.x, centre.y,
                         centre.z,
                         instance.transform.m[0][0], instance.transform.m[0][1],
                         instance.transform.m[0][2], instance.transform.m[1][0],
                         instance.transform.m[1][1], instance.transform.m[1][2],
                         instance.transform.m[2][0], instance.transform.m[2][1],
                         instance.transform.m[2][2],
                         section.name.empty() ? "?" : section.name.c_str(),
                         section.tablePosition.x, section.tablePosition.y,
                         section.alternate ? 1 : 0);
            ++written;
        }
    }

    std::fclose(file);
    LOG_INFO("placement dump written: %zu placements", written);
    return true;
}

std::string WorldRenderer::InstanceKey(const std::string& section, int native, int placement) {
    if (native >= 0) return "N " + section + " " + std::to_string(native);
    if (placement >= 0) return "P " + section + " " + std::to_string(placement);
    return {};
}

std::string WorldRenderer::HiddenKey(const PickResult& picked) const {
    if (picked.loadedSection >= sections_.size()) return {};
    return InstanceKey(sections_[picked.loadedSection].name, picked.native, picked.placement);
}

const std::vector<char>& WorldRenderer::HiddenFlags(size_t i) const {
    const SectionRender& section = sections_[i];
    if (section.hiddenVersion != hiddenVersion_ || section.hidden.size() != section.instances.size()) {
        section.hidden.assign(section.instances.size(), 0);
        if (!hiddenKeys_.empty())
            for (size_t k = 0; k < section.instances.size(); ++k) {
                const std::string key =
                    InstanceKey(section.name, section.instances[k].native, section.instances[k].placement);
                if (!key.empty() && hiddenKeys_.count(key)) section.hidden[k] = 1;
            }
        if (editedOnly_)
            for (size_t k = 0; k < section.instances.size(); ++k)
                if (!InstanceEdited(section, section.instances[k])) section.hidden[k] = 1;
        section.hiddenVersion = hiddenVersion_;
    }
    return section.hidden;
}

bool WorldRenderer::InstanceEdited(const SectionRender& section, const GpuInstance& instance) const {
    if (instance.placement >= 0) return true;    // made or moved in the editor
    if (instance.native >= 0 && !editedNatives_.empty() &&
        editedNatives_.count(InstanceKey(section.name, instance.native, -1)))
        return true;
    if (!GpuOf(instance.section)) return false;
    const auto& objects = GpuOf(instance.section)->Objects();
    if (instance.object >= objects.size() || !objects[instance.object].source) return false;
    const nfs::SolidObject& model = *objects[instance.object].source;
    if (editedModels_.count(model.hash)) return true;
    if (!editedTextures_.empty())
        for (const u32 texture : model.textureHashes)
            if (editedTextures_.count(texture)) return true;
    return false;
}

bool WorldRenderer::Pick(const Ray& ray, PickResult& out) const {
    PickResult best;

    for (size_t i = 0; i < sections_.size(); ++i) {
        const SectionRender& section = sections_[i];
        if (section.alternate && !showAlternate_) continue;

        if (showLibrary_ && section.gpu) {
            const auto& objects = section.gpu->Objects();
            for (size_t o = 0; o < objects.size(); ++o)
                PickOne(ray, objects[o], objects[o].transform, objects[o].worldBounds,
                        false, i, (u32)i, (u32)o, best);
        }

        if (!showScenery_) continue;

        const std::vector<char>& hiddenHere = HiddenFlags(i);
        for (size_t k = 0; k < section.instances.size(); ++k) {
            const GpuInstance& instance = section.instances[k];
            if (hiddenHere[k]) continue;
            const GpuObject& object = GpuOf(instance.section)->Objects()[instance.object];
            PickOne(ray, object, instance.transform, instance.worldBounds, true, i,
                    instance.section, instance.object, best, instance.placement, instance.native);
        }
    }

    out = best;
    return best.Hit();
}

// Two placements are the same selection when they are the same mesh put in the
// same spot; a model drawn twice nearby is two different things to pick.
bool WorldRenderer::ToggleSelection(const PickResult& selection) {
    collisionSelection_.clear();
    selectedOwner_ = nullptr; selectedExtra_ = -1;
    const auto same = [&](const PickResult& other) {
        return other.object == selection.object &&
               std::memcmp(&other.transform, &selection.transform, sizeof(Mat4)) == 0;
    };

    const auto it = std::find_if(selection_.begin(), selection_.end(), same);
    if (it != selection_.end()) {
        selection_.erase(it);
        return false;
    }

    selection_.push_back(selection);
    return true;
}

void WorldRenderer::DrawSelection(IDirect3DDevice9* device) {
    if (selection_.empty()) return;

    // The meshes themselves are painted rather than boxed. A box says where an
    // object roughly is; the paint says which object, which is what someone
    // reporting a wrong model needs when the thing sits among a dozen
    // neighbours -- and with several selected, boxes would be a thicket.
    constexpr D3DCOLOR kSelectionColour = 0xA030FF90;   // alpha carries the tint

    device->SetTexture(0, nullptr);
    device->SetTextureStageState(0, D3DTSS_COLOROP,   D3DTOP_SELECTARG1);
    device->SetTextureStageState(0, D3DTSS_COLORARG1, D3DTA_TFACTOR);
    device->SetTextureStageState(0, D3DTSS_ALPHAOP,   D3DTOP_SELECTARG1);
    device->SetTextureStageState(0, D3DTSS_ALPHAARG1, D3DTA_TFACTOR);
    device->SetRenderState(D3DRS_TEXTUREFACTOR, kSelectionColour);

    device->SetRenderState(D3DRS_ALPHATESTENABLE, FALSE);
    device->SetRenderState(D3DRS_ALPHABLENDENABLE, TRUE);
    device->SetRenderState(D3DRS_SRCBLEND,  D3DBLEND_SRCALPHA);
    device->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA);

    // Drawn over the surface it belongs to, not into the depth buffer: equal
    // depth passes so the paint lands exactly on the object's own pixels, and
    // leaving the buffer alone keeps it from cutting into whatever draws next.
    device->SetRenderState(D3DRS_ZFUNC, D3DCMP_LESSEQUAL);
    device->SetRenderState(D3DRS_ZWRITEENABLE, FALSE);

    // Both faces, because a selected object is often seen from inside or edge
    // on, and half a painted shell reads as a hole.
    device->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
    device->SetFVF(kWorldVertexFvf);

    for (const PickResult& picked : selection_) {
        const GpuSection* owner = GpuOf((u32)picked.gpuSection);
        if (!owner || picked.gpuObject >= owner->Objects().size()) continue;

        const GpuObject& object = owner->Objects()[picked.gpuObject];

        const D3DMATRIX world = ToD3D(picked.transform);
        device->SetTransform(D3DTS_WORLD, &world);
        device->SetStreamSource(0, owner->VertexBuffer(), 0, sizeof(nfs::SolidVertex));
        device->SetIndices(owner->IndexBuffer());

        for (const GpuDraw& draw : object.draws)
            device->DrawIndexedPrimitive(D3DPT_TRIANGLELIST, (INT)object.baseVertex,
                                         draw.minVertex, draw.vertexCount,
                                         draw.startIndex, draw.primitiveCount);
    }

    device->SetRenderState(D3DRS_ZWRITEENABLE, TRUE);
    device->SetRenderState(D3DRS_ZFUNC, D3DCMP_LESS);
    device->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
    device->SetRenderState(D3DRS_ALPHATESTENABLE, TRUE);
    device->SetRenderState(D3DRS_CULLMODE, backfaceCulling_ ? cullMode_ : D3DCULL_NONE);
}

void WorldRenderer::DrawCollision(IDirect3DDevice9* device, const std::vector<size_t>& visible) {
    if (visible.empty()) return;

    device->SetTexture(0, nullptr);
    device->SetTextureStageState(0, D3DTSS_COLOROP,   D3DTOP_SELECTARG1);
    device->SetTextureStageState(0, D3DTSS_COLORARG1, D3DTA_DIFFUSE);
    device->SetTextureStageState(0, D3DTSS_ALPHAOP,   D3DTOP_SELECTARG1);
    device->SetTextureStageState(0, D3DTSS_ALPHAARG1, D3DTA_DIFFUSE);
    const D3DMATRIX identity = ToD3D(Mat4::Identity());
    device->SetTransform(D3DTS_WORLD, &identity);
    device->SetFVF(CollisionMesh::kFvf);

    device->SetRenderState(D3DRS_FILLMODE, D3DFILL_SOLID);
    device->SetRenderState(D3DRS_ALPHATESTENABLE, FALSE);
    device->SetRenderState(D3DRS_ALPHABLENDENABLE, TRUE);
    device->SetRenderState(D3DRS_SRCBLEND,  D3DBLEND_SRCALPHA);
    device->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA);

    // Tested against the world so a building still hides the road behind it,
    // but not written: translucent layers would otherwise punch holes in one
    // another depending on the order the sections happen to come in.
    device->SetRenderState(D3DRS_ZFUNC, D3DCMP_LESSEQUAL);
    device->SetRenderState(D3DRS_ZWRITEENABLE, FALSE);

    // Ground polygons have no consistent winding and walls are seen from both
    // sides, so nothing is culled.
    device->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);

    for (size_t index : visible) sections_[index].collision->DrawSurfaces(device);

    device->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
    for (size_t index : visible) sections_[index].collision->DrawOutlines(device);

    DrawCollisionSelection(device);

    device->SetRenderState(D3DRS_ZWRITEENABLE, TRUE);
    device->SetRenderState(D3DRS_ZFUNC, D3DCMP_LESS);
    device->SetRenderState(D3DRS_ALPHATESTENABLE, TRUE);
    device->SetRenderState(D3DRS_CULLMODE, backfaceCulling_ ? cullMode_ : D3DCULL_NONE);
    device->SetRenderState(D3DRS_FILLMODE,
                           mode_ == RenderMode::Wireframe ? D3DFILL_WIREFRAME : D3DFILL_SOLID);
    device->SetFVF(kWorldVertexFvf);
}

// ---------------------------------------------------------------------------
// Collision selection
// ---------------------------------------------------------------------------

bool WorldRenderer::CollisionCorners(const CollisionPick& pick, Vec3 out[4], bool& quad) const {
    if (!world_ || pick.section >= world_->Loaded().size()) return false;
    const auto& packs = world_->Loaded()[pick.section]->collision;
    if (pick.pack >= packs.size() || pick.polygon >= packs[pick.pack].polygons.size())
        return false;

    const nfs::CollisionPack& pack = packs[pick.pack];
    const nfs::CollisionPolygon& polygon = pack.polygons[pick.polygon];
    if (polygon.barrier) {
        pack.BarrierQuad(polygon, out);
        quad = true;
        return true;
    }
    for (int c = 0; c < 4; ++c) out[c] = polygon.corners[c];
    quad = polygon.quad;
    return true;
}

u32 WorldRenderer::AnimatedTexture(u32 hash) const {
    if (!animate_ || !world_) return hash;
    const auto& animations = world_->TextureAnimations();
    if (animations.empty()) return hash;
    const auto found = animations.find(hash);
    if (found == animations.end()) return hash;
    const nfs::TextureAnimation& animation = found->second;
    const size_t step = (size_t)(animationClock_ * animation.fps);
    const size_t frame = (animation.startFrame + step) % animation.frames.size();
    return animation.frames[frame];
}

void WorldRenderer::DrawSky(IDirect3DDevice9* device, const Camera& camera, RenderStats& stats) {
    // World's dome comes from the GLOBAL sky bands, not from a backdrop model.
    if (world_ && world_->IsWorld()) return DrawWorldSky(device, camera, stats);
    // The dome is a backdrop model; it is drawn round the eye, so however far
    // the camera flies the sky stays as far away -- and with depth off, it is
    // behind whatever the world draws after it.
    for (size_t i = 0; i < sections_.size(); ++i) {
        if (!sections_[i].gpu) continue;
        for (const GpuObject& object : sections_[i].gpu->Objects()) {
            if (!object.source || !object.isBackdrop || object.source->name.find("SKYDOME") == std::string::npos)
                continue;
            const Aabb& local = object.source->bounds;
            const float radius = local.Valid() ? std::max({ local.max.x - local.min.x, local.max.y - local.min.y,
                                                            local.max.z - local.min.z }) * 0.5f : 1000.0f;
            // Inside the far plane, whatever size it was made.
            const float scale = std::min(1.0f, camera.FarPlane() * 0.45f / std::max(radius, 1.0f));
            static bool told = false;
            if (!told) {
                told = true;
                LOG_INFO("sky: %s, radius %.0f, drawn at scale %.3f round the camera", object.source->name.c_str(),
                         radius, scale);
                for (u32 hash : object.source->textureHashes) {
                    const nfs::Texture* found = world_->FindDisplayTexture(hash);
                    LOG_INFO("sky texture %08X: %s", hash, found ? found->name.c_str() : "NOT FOUND");
                    const auto anim = world_->TextureAnimations().find(hash);
                    if (anim != world_->TextureAnimations().end())
                        for (u32 frame : anim->second.frames) {
                            const nfs::Texture* f = world_->FindDisplayTexture(frame);
                            LOG_INFO("  frame %08X: %s", frame, f ? f->name.c_str() : "NOT FOUND");
                        }
                }
            }
            Mat4 transform = Mat4::Identity();
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c) transform.m[r][c] = object.transform.m[r][c] * scale;
            const Vec3 eye = camera.Position();
            transform.m[3][0] = eye.x;
            transform.m[3][1] = eye.y;
            transform.m[3][2] = eye.z;

            const IDirect3DVertexBuffer9* none = nullptr;
            (void)none;
            device->SetStreamSource(0, sections_[i].gpu->VertexBuffer(), 0, sizeof(nfs::SolidVertex));
            device->SetIndices(sections_[i].gpu->IndexBuffer());
            device->SetFVF(kWorldVertexFvf);
            const D3DMATRIX world = ToD3D(transform);
            device->SetTransform(D3DTS_WORLD, &world);
            device->SetRenderState(D3DRS_ZENABLE, FALSE);
            device->SetRenderState(D3DRS_ZWRITEENABLE, FALSE);
            device->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
            glDrawingSky_ = true;
            const bool layered = DrawObject(device, object, stats);
            if (layered) {
                BeginTranslucentStates(device);
                device->SetRenderState(D3DRS_ZENABLE, FALSE);
                device->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
                DrawObject(device, object, stats, DrawPass::Translucent);
                EndTranslucentStates(device);
            }
            glDrawingSky_ = false;
            device->SetRenderState(D3DRS_ZENABLE, TRUE);
            device->SetRenderState(D3DRS_ZWRITEENABLE, TRUE);
            device->SetRenderState(D3DRS_CULLMODE, backfaceCulling_ ? cullMode_ : D3DCULL_NONE);
            return;   // one dome
        }
    }
}

// GLOBAL\SkyDomeTextures.bin holds six time-of-day sets of four photo
// panoramas; A is the zenith band, D the horizon haze. The dome stacks the
// bands over equal elevation steps, the same one-wrap-around layout the
// textures are painted for. NOON until a weather look asks for another set.
struct WorldSkyBand {
    const char* texture;
    float top, bottom;   // elevation, in halves of pi (1 = zenith, 0 = horizon)
};
constexpr WorldSkyBand kWorldSkyBands[] = {
    // The bands stack by their pixel heights (2:2:1:1 of the dome over the
    // horizon), so the cloud decks come down near the horizon the way the
    // games paint them. The horizon band dips well past the horizon line:
    // the camera is rarely at the dome's centre, and past the city's edge
    // the game shows sky, not void.
    { "SKY_1NOONA", 1.0f, 0.4646f },    // asin(4/6): the zenith half
    { "SKY_1NOONB", 0.4646f, 0.2163f }, // asin(2/6)
    { "SKY_1NOONC", 0.2163f, 0.1066f }, // asin(1/6), the sun's band
    { "SKY_1NOOND", 0.1066f, -0.5f },   // the horizon strip, skirted below
};
constexpr unsigned kWorldSkySegments = 48;
constexpr unsigned kWorldSkyRows = 10;  // quads per band
// Each band blends this share of its own span into the one above it, so the
// texture swap never shows as a hard arc.
constexpr float kWorldSkyFade = 0.25f;

bool WorldRenderer::BuildWorldSky() {
    if (!context_ || !context_->IsValid()) return false;
    IDirect3DDevice9* device = context_->Device();
    if (!device) return false;

    struct SkyVertex { float x, y, z; D3DCOLOR colour; float u, v; };
    constexpr float kSkyPi = 3.14159265f;
    const unsigned columns = kWorldSkySegments + 1;
    std::vector<SkyVertex> vertices;
    std::vector<u16> indices;
    for (unsigned b = 0; b < std::size(kWorldSkyBands); ++b) {
        const WorldSkyBand& band = kWorldSkyBands[b];
        const float top = band.top * 90.0f, bottom = band.bottom * 90.0f;
        // Row 0 of a band is its top edge, so the bands run downhill. Every
        // band but the zenith reaches past its own top edge with a fade, and
        // paints over the band above it.
        const float fade = b ? std::clamp(kWorldSkyFade * (top - bottom), 2.0f, 6.0f) : 0.0f;
        const float extent = top + fade;
        const unsigned base = (unsigned)vertices.size();
        for (unsigned j = 0; j <= kWorldSkyRows; ++j) {
            const float t = (float)j / kWorldSkyRows;
            const float elevation = extent - t * (extent - bottom);
            const float theta = elevation * (kSkyPi / 180.0f);
            // v measures the band's own span, so the fade rows reach past the
            // texture's top edge, which the sampler clamp edge-stretches.
            const float v = (elevation - bottom) / (top - bottom);
            const float alpha = fade > 0.0f
                ? std::clamp((extent - elevation) / fade, 0.0f, 1.0f) : 1.0f;
            const D3DCOLOR colour = D3DCOLOR_COLORVALUE(1.0f, 1.0f, 1.0f, alpha);
            for (unsigned k = 0; k <= kWorldSkySegments; ++k) {
                const float phi = 2.0f * kSkyPi * (float)k / kWorldSkySegments;
                vertices.push_back({ std::cos(phi) * std::cos(theta), std::sin(phi) * std::cos(theta),
                                     std::sin(theta), colour, (float)k / kWorldSkySegments, v });
            }
        }
        for (unsigned j = 0; j < kWorldSkyRows; ++j)
            for (unsigned k = 0; k < kWorldSkySegments; ++k) {
                const unsigned a = base + j * columns + k;
                const unsigned b2 = a + columns;
                const u16 quad[6] = { (u16)a, (u16)b2, (u16)(a + 1), (u16)(a + 1), (u16)b2, (u16)(b2 + 1) };
                indices.insert(indices.end(), quad, quad + 6);
            }
    }

    IDirect3DVertexBuffer9* vertices9 = nullptr;
    IDirect3DIndexBuffer9* indices9 = nullptr;
    if (FAILED(device->CreateVertexBuffer((UINT)(vertices.size() * sizeof(SkyVertex)), 0,
                                          D3DFVF_XYZ | D3DFVF_DIFFUSE | D3DFVF_TEX1,
                                          D3DPOOL_MANAGED, &vertices9, nullptr)))
        return false;
    if (FAILED(device->CreateIndexBuffer((UINT)(indices.size() * sizeof(u16)), 0, D3DFMT_INDEX16,
                                         D3DPOOL_MANAGED, &indices9, nullptr))) {
        vertices9->Release();
        return false;
    }
    void* locked = nullptr;
    if (FAILED(vertices9->Lock(0, 0, &locked, 0))) {
        vertices9->Release();
        indices9->Release();
        return false;
    }
    std::memcpy(locked, vertices.data(), vertices.size() * sizeof(SkyVertex));
    vertices9->Unlock();
    if (FAILED(indices9->Lock(0, 0, &locked, 0))) {
        vertices9->Release();
        indices9->Release();
        return false;
    }
    std::memcpy(locked, indices.data(), indices.size() * sizeof(u16));
    indices9->Unlock();

    worldSkyVertices_ = vertices9;
    worldSkyIndices_ = indices9;
    worldSkyBands_ = (unsigned)std::size(kWorldSkyBands);
    return true;
}

void WorldRenderer::DrawWorldSky(IDirect3DDevice9* device, const Camera& camera, RenderStats& stats) {
    if (!worldSkyVertices_ && !BuildWorldSky()) return;
    // Keep the dome round the eye and inside the far plane. The map bounds
    // include the huge backdrop/library shells, so anchoring to them put the
    // horizon beyond the far plane: from a car only the zenith was visible.
    const Vec3 base = camera.Position();
    const float radius = std::max(camera.FarPlane() * 0.90f, 1000.0f);
    // Row-vector math: scale the unit dome first, then move it to its base.
    const Mat4 transform = Mat4::Scale(Vec3{ radius, radius, radius }) * Mat4::Translation(base);

    device->SetRenderState(D3DRS_ZENABLE, FALSE);
    device->SetRenderState(D3DRS_ZWRITEENABLE, FALSE);
    device->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
    // The bands fade into each other through their vertex alpha.
    device->SetRenderState(D3DRS_ALPHABLENDENABLE, TRUE);
    device->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_SRCALPHA);
    device->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA);
    const D3DMATRIX world = ToD3D(transform);
    device->SetTransform(D3DTS_WORLD, &world);
    device->SetStreamSource(0, worldSkyVertices_, 0, 24);
    device->SetIndices(worldSkyIndices_);
    device->SetFVF(D3DFVF_XYZ | D3DFVF_DIFFUSE | D3DFVF_TEX1);
    device->SetTextureStageState(0, D3DTSS_COLOROP, D3DTOP_SELECTARG1);
    device->SetTextureStageState(0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
    device->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_SELECTARG1);
    device->SetTextureStageState(0, D3DTSS_ALPHAARG1, D3DTA_DIFFUSE);
    device->SetSamplerState(0, D3DSAMP_ADDRESSU, D3DTADDRESS_WRAP);
    device->SetSamplerState(0, D3DSAMP_ADDRESSV, D3DTADDRESS_CLAMP);
    device->SetSamplerState(0, D3DSAMP_MAGFILTER, D3DTEXF_LINEAR);
    device->SetSamplerState(0, D3DSAMP_MINFILTER, D3DTEXF_LINEAR);

    const unsigned faces = kWorldSkyRows * kWorldSkySegments * 2;
    for (unsigned band = 0; band < worldSkyBands_; ++band) {
        const auto& texture = textures_.Get(device, TextureNameHash(kWorldSkyBands[band].texture));
        if (!texture.texture) continue;
        device->SetTexture(0, texture.texture);
        device->DrawIndexedPrimitive(D3DPT_TRIANGLELIST, 0, 0,
                                     (kWorldSkyRows + 1) * (kWorldSkySegments + 1),
                                     band * faces, faces);
        ++stats.drawCalls;
    }
    // The world pass only renews its stream and world matrix per section; the
    // FVF and the sampler addresses come from ApplyRenderStates, once per
    // frame, so anything this pass leaves behind corrupts the whole frame.
    device->SetTexture(0, nullptr);
    device->SetFVF(kWorldVertexFvf);
    device->SetSamplerState(0, D3DSAMP_ADDRESSV, D3DTADDRESS_WRAP);
    device->SetTextureStageState(0, D3DTSS_COLOROP, D3DTOP_MODULATE2X);
    device->SetTextureStageState(0, D3DTSS_COLORARG1, D3DTA_TEXTURE);
    device->SetTextureStageState(0, D3DTSS_COLORARG2, D3DTA_DIFFUSE);
    device->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_SELECTARG1);
    device->SetTextureStageState(0, D3DTSS_ALPHAARG1, D3DTA_TFACTOR);
    device->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
    device->SetRenderState(D3DRS_ZENABLE, TRUE);
    device->SetRenderState(D3DRS_ZWRITEENABLE, TRUE);
    device->SetRenderState(D3DRS_CULLMODE, backfaceCulling_ ? cullMode_ : D3DCULL_NONE);
}

void WorldRenderer::SetRaceGroupsVisible(bool visible) {
    if (showRaceGroups_ == visible) return;
    showRaceGroups_ = visible;
    if (!world_) return;
    for (size_t i = 0; i < sections_.size() && i < world_->Loaded().size(); ++i) RefreshCollision(i);
    collisionSelection_.erase(
        std::remove_if(collisionSelection_.begin(), collisionSelection_.end(), [&](const CollisionPick& pick) {
            const auto& packs = world_->Loaded()[pick.section]->collision;
            return !visible && pick.pack < packs.size() && pick.polygon < packs[pick.pack].polygons.size() &&
                   packs[pick.pack].polygons[pick.polygon].raceGroup != 0;
        }),
        collisionSelection_.end());
}

void WorldRenderer::RefreshCollision(size_t loadedSection) {
    if (!world_ || !context_ || loadedSection >= sections_.size() ||
        loadedSection >= world_->Loaded().size())
        return;
    const nfs::LoadedSection& section = *world_->Loaded()[loadedSection];
    nfs::CollisionPack placed;
    std::vector<nfs::NfswSmackableBox> editedBoxes;
    bool boxesChanged = false;
    if (world_->IsWorld()) {
        placed.sectionNumber = section.info ? section.info->number : 0;
        const auto add = [&](u32 hash, const Mat4& transform) {
            if (const nfs::SolidObject* model = world_->FindModel(hash))
                collision::GenerateNfswPlacementCollision(*model, transform, placed.polygons);
        };
        for (size_t index : section.nfswEditedScenery) {
            if (section.smackableScenery.count(index)) continue;
            const nfs::SceneryInstance& instance = section.scenery.instances[index];
            const auto key = std::make_pair(instance.sceneryNumber, instance.record);
            if (std::find(section.removedNatives.begin(), section.removedNatives.end(), key) ==
                section.removedNatives.end())
                add(section.scenery.ModelHash(instance), instance.transform);
        }
        std::set<size_t> hiddenBoxes;
        for (const auto& [index, link] : section.smackableScenery) {
            const size_t box = section.smackableLinks[link].box;
            if (box == SIZE_MAX) continue;
            const auto& source = section.scenery.instances[index];
            const auto key = std::make_pair(source.sceneryNumber, source.record);
            if (std::find(section.removedNatives.begin(), section.removedNatives.end(), key) !=
                section.removedNatives.end())
                hiddenBoxes.insert(box);
        }
        boxesChanged = !hiddenBoxes.empty() || std::any_of(section.smackableLinks.begin(), section.smackableLinks.end(),
            [](const auto& link) { return link.halfSizes.has_value(); }) ||
            std::any_of(section.placements.begin(), section.placements.end(),
            [](const auto& placement) { return placement.instance.smackable.has_value(); });
        if (boxesChanged) {
            std::unordered_map<size_t, const nfs::NfswSmackableLink*> resized;
            for (const auto& link : section.smackableLinks)
                if (link.halfSizes) resized.emplace(link.box, &link);
            for (size_t i = 0; i < section.smackableBoxes.size(); ++i) {
                if (hiddenBoxes.count(i)) continue;
                const auto shape = resized.find(i);
                const auto& box = section.smackableBoxes[i];
                editedBoxes.push_back(shape == resized.end() ? box : shape->second->EditedBox(box));
            }
        }
        for (const auto& placement : section.placements) {
            const auto& item = placement.instance;
            if (item.smackable)
                editedBoxes.push_back(item.smackable->BoxAt(item.transform));
            else
                add(item.modelHash, item.transform);
        }
    }
    SectionRender& target = sections_[loadedSection];
    target.hasPlacementCollision = !placed.polygons.empty() || boxesChanged;
    target.collisionPreviewDirty = false;
    auto mesh = std::make_unique<CollisionMesh>();
    if (mesh->Upload(context_->Device(), section, showRaceGroups_,
                     placed.polygons.empty() ? nullptr : &placed, boxesChanged ? &editedBoxes : nullptr))
        target.collision = std::move(mesh);
    else
        target.collision.reset();
}

void WorldRenderer::SetCollisionSelectionList(std::vector<CollisionPick> picks) {
    selection_.clear();
    selectedOwner_ = nullptr; selectedExtra_ = -1;
    collisionSelection_ = std::move(picks);
}

void WorldRenderer::DrawToolOverlay(IDirect3DDevice9* device) {
    bool any = false;
    for (const auto& [owner, layer] : toolLayers_) any = any || !layer.triangles.empty() || !layer.lines.empty();
    if (!any) return;
    BeginOverlay(device);
    device->SetRenderState(D3DRS_ALPHABLENDENABLE, TRUE);
    device->SetRenderState(D3DRS_SRCBLEND,  D3DBLEND_SRCALPHA);
    device->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA);
    device->SetRenderState(D3DRS_ZWRITEENABLE, FALSE);
    for (const auto& [owner, layer] : toolLayers_) {
        device->SetRenderState(D3DRS_ZENABLE, TRUE);
        if (!layer.triangles.empty())
            device->DrawPrimitiveUP(D3DPT_TRIANGLELIST, (UINT)(layer.triangles.size() / 3),
                                    layer.triangles.data(), sizeof(OverlayVertex));
        // The lines show through everything: a point being placed behind a
        // kerb still has to be seen.
        device->SetRenderState(D3DRS_ZENABLE, FALSE);
        if (!layer.lines.empty())
            device->DrawPrimitiveUP(D3DPT_LINELIST, (UINT)(layer.lines.size() / 2),
                                    layer.lines.data(), sizeof(OverlayVertex));
    }
    EndOverlay(device);
}

void WorldRenderer::UpdateModelColours(u32 modelHash, const std::vector<u32>& colours) {
    for (SectionRender& section : sections_) {
        if (!section.gpu) continue;
        for (const GpuObject& object : section.gpu->Objects()) {
            if (!object.source || object.source->hash != modelHash) continue;
            const size_t count = std::min(colours.size(), object.source->vertices.size());
            IDirect3DVertexBuffer9* buffer = section.gpu->VertexBuffer();
            void* data = nullptr;
            const UINT offset = (UINT)(object.baseVertex * sizeof(nfs::SolidVertex));
            const UINT bytes = (UINT)(count * sizeof(nfs::SolidVertex));
            if (!buffer || !count || FAILED(buffer->Lock(offset, bytes, &data, 0))) continue;
            auto* vertices = (nfs::SolidVertex*)data;
            for (size_t i = 0; i < count; ++i) vertices[i].color = colours[i];
            buffer->Unlock();
        }
    }
}

void WorldRenderer::UpdateModelPositions(u32 modelHash, const std::vector<Vec3>& positions) {
    for (SectionRender& section : sections_) {
        if (!section.gpu) continue;
        for (const GpuObject& object : section.gpu->Objects()) {
            if (!object.source || object.source->hash != modelHash) continue;
            const size_t count = std::min(positions.size(), object.source->vertices.size());
            IDirect3DVertexBuffer9* buffer = section.gpu->VertexBuffer();
            void* data = nullptr;
            const UINT offset = (UINT)(object.baseVertex * sizeof(nfs::SolidVertex));
            const UINT bytes = (UINT)(count * sizeof(nfs::SolidVertex));
            if (!buffer || !count || FAILED(buffer->Lock(offset, bytes, &data, 0))) continue;
            auto* vertices = (nfs::SolidVertex*)data;
            for (size_t i = 0; i < count; ++i) vertices[i].position = positions[i];
            buffer->Unlock();
        }
    }
}

bool WorldRenderer::PickCollision(const Ray& ray, CollisionPick& out) const {
    if (!world_ || !showCollision_) return false;

    CollisionPick best;
    bool found = false;

    for (size_t index : collisionDrawn_) {
        if (index >= world_->Loaded().size() || !sections_[index].collision) continue;
        if (IntersectRayAabb(ray, sections_[index].collision->Bounds()) < 0.0f) continue;

        const auto& packs = world_->Loaded()[index]->collision;
        for (size_t p = 0; p < packs.size(); ++p) {
            if (world_->IsWorld() && !packs[p].edited && packs[p].bounds.Valid() &&
                IntersectRayAabb(ray, packs[p].bounds) < 0.0f) continue;
            for (size_t i = 0; i < packs[p].polygons.size(); ++i) {
                if (!showRaceGroups_ && packs[p].polygons[i].raceGroup != 0) continue;
                CollisionPick candidate{ (u32)index, (u32)p, (u32)i };
                Vec3 corners[4];
                bool quad = false;
                if (!CollisionCorners(candidate, corners, quad)) continue;

                float hit = IntersectRayTriangle(ray, corners[0], corners[1], corners[2]);
                if (quad) {
                    const float second = IntersectRayTriangle(ray, corners[0], corners[2], corners[3]);
                    if (second >= 0.0f && (hit < 0.0f || second < hit)) hit = second;
                }
                if (hit < 0.0f || (found && hit >= best.distance)) continue;

                candidate.distance = hit;
                best  = candidate;
                found = true;
            }
        }
    }

    if (found) out = best;
    return found;
}

void WorldRenderer::SetCollisionSelection(const CollisionPick& pick) {
    selection_.clear();
    selectedOwner_ = nullptr; selectedExtra_ = -1;
    collisionSelection_ = { pick };
}

bool WorldRenderer::ToggleCollisionSelection(const CollisionPick& pick) {
    selection_.clear();
    selectedOwner_ = nullptr; selectedExtra_ = -1;
    const auto it = std::find_if(collisionSelection_.begin(), collisionSelection_.end(),
                                 [&](const CollisionPick& other) { return other.SamePolygon(pick); });
    if (it != collisionSelection_.end()) {
        collisionSelection_.erase(it);
        return false;
    }
    collisionSelection_.push_back(pick);
    return true;
}

size_t WorldRenderer::SelectCollisionPack(const CollisionPick& pick, bool add) {
    selection_.clear();
    selectedOwner_ = nullptr; selectedExtra_ = -1;
    if (!add) collisionSelection_.clear();
    if (!world_ || pick.section >= world_->Loaded().size()) return 0;
    const auto& packs = world_->Loaded()[pick.section]->collision;
    if (pick.pack >= packs.size()) return 0;

    size_t added = 0;
    for (size_t i = 0; i < packs[pick.pack].polygons.size(); ++i) {
        const CollisionPick one{ pick.section, pick.pack, (u32)i };
        const bool present = std::any_of(collisionSelection_.begin(), collisionSelection_.end(),
                                         [&](const CollisionPick& other) { return other.SamePolygon(one); });
        if (present) continue;
        collisionSelection_.push_back(one);
        ++added;
    }
    return added;
}

void WorldRenderer::DrawCollisionSelection(IDirect3DDevice9* device) {
    if (collisionSelection_.empty()) return;

    struct Vertex { float x, y, z; D3DCOLOR colour; };
    constexpr D3DCOLOR kFill    = 0xB000E8FF;
    constexpr D3DCOLOR kOutline = 0xFFFFFFFF;
    constexpr float    kLift    = 0.1f;   // above the ordinary collision layer

    std::vector<Vertex> fill, lines;
    fill.reserve(collisionSelection_.size() * 6);
    lines.reserve(collisionSelection_.size() * 8);

    for (const CollisionPick& pick : collisionSelection_) {
        Vec3 c[4];
        bool quad = false;
        if (!CollisionCorners(pick, c, quad)) continue;
        auto push = [](std::vector<Vertex>& out, const Vec3& p, D3DCOLOR colour) {
            out.push_back({ p.x, p.y, p.z + kLift, colour });
        };
        const auto& polygon = world_->Loaded()[pick.section]->collision[pick.pack].polygons[pick.polygon];
        if (!polygon.TallNfswBoundary()) {
            for (int k : { 0, 1, 2 }) push(fill, c[k], kFill);
            if (quad) for (int k : { 0, 2, 3 }) push(fill, c[k], kFill);
        }
        const int corners = quad ? 4 : 3;
        for (int k = 0; k < corners; ++k) {
            push(lines, c[k], kOutline);
            push(lines, c[(k + 1) % corners], kOutline);
        }
    }

    device->SetFVF(CollisionMesh::kFvf);
    if (!fill.empty()) {
        device->SetRenderState(D3DRS_ALPHABLENDENABLE, TRUE);
        device->DrawPrimitiveUP(D3DPT_TRIANGLELIST, (UINT)(fill.size() / 3),
                                fill.data(), sizeof(Vertex));
        device->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
    }
    if (!lines.empty())
        device->DrawPrimitiveUP(D3DPT_LINELIST, (UINT)(lines.size() / 2),
                                lines.data(), sizeof(Vertex));
}

bool WorldRenderer::SelectPlacement(u32 loadedSection, int placement) {
    if (loadedSection >= sections_.size()) return false;
    for (const GpuInstance& instance : sections_[loadedSection].instances) {
        if (instance.placement != placement) continue;
        const GpuSection* owner = GpuOf(instance.section);
        if (!owner || instance.object >= owner->Objects().size()) return false;

        PickResult picked;
        picked.object        = owner->Objects()[instance.object].source;
        picked.transform     = instance.transform;
        picked.worldBounds   = instance.worldBounds;
        picked.fromInstance  = true;
        picked.gpuSection    = instance.section;
        picked.gpuObject     = instance.object;
        picked.loadedSection = loadedSection;
        picked.placement     = placement;
        if (world_ && loadedSection < world_->Loaded().size() && world_->Loaded()[loadedSection]->info)
            picked.sectionName = world_->Loaded()[loadedSection]->info->name;
        SetSelection(picked);
        return true;
    }
    return false;
}

void WorldRenderer::SelectPlacements(const std::vector<std::pair<u32, int>>& placements) {
    std::vector<PickResult> chosen;
    for (const auto& [section, placement] : placements)
        if (SelectPlacement(section, placement)) chosen.push_back(selection_.front());
    selection_ = std::move(chosen);
    collisionSelection_.clear();
    selectedOwner_ = nullptr; selectedExtra_ = -1;
}

// ---------------------------------------------------------------------------
// Extensions
// ---------------------------------------------------------------------------

bool WorldRenderer::PickExtension(const Ray& ray, const RenderExtension*& owner, int& index,
                                  float& distance) const {
    owner = nullptr;
    index = -1;
    for (const RenderExtension* extension : extensions_) {
        int hit = -1;
        float at = 0.0f;
        if (!extension->Pick(ray, hit, at) || hit < 0) continue;
        if (owner && at >= distance) continue;
        owner = extension;
        index = hit;
        distance = at;
    }
    return owner != nullptr;
}

void WorldRenderer::BeginModels(IDirect3DDevice9* device, const GpuSection& gpu) {
    device->SetFVF(kWorldVertexFvf);
    device->SetStreamSource(0, gpu.VertexBuffer(), 0, sizeof(nfs::SolidVertex));
    device->SetIndices(gpu.IndexBuffer());
    device->SetRenderState(D3DRS_CULLMODE, backfaceCulling_ ? cullMode_ : D3DCULL_NONE);
    if (mode_ != RenderMode::Textured && mode_ != RenderMode::Unlit) BindTextureStage(device, CachedTexture{});
}

void WorldRenderer::DrawModel(IDirect3DDevice9* device, const GpuObject& object, bool translucent) {
    RenderStats unused;
    DrawObject(device, object, unused, translucent ? DrawPass::Translucent : DrawPass::Solid);
}

void WorldRenderer::BeginTranslucentModels(IDirect3DDevice9* device) { BeginTranslucentStates(device); }
void WorldRenderer::EndTranslucentModels(IDirect3DDevice9* device) { EndTranslucentStates(device); }

void WorldRenderer::DrawObjectAnimations(IDirect3DDevice9* device, bool usingVisibility, const Vec3& eye,
                                         RenderStats& stats) {
    if (!world_) return;
    const nfs::WorldAnimations& animations = world_->ObjectAnimations();
    if (animations.Placements().empty()) return;
    const GpuSection* unusedGpu = nullptr;
    const GpuObject* unusedObject = nullptr;
    FindGpuModel(0, unusedGpu, unusedObject);   // fills objectByHash_
    const double seconds = animate_ ? animationClock_ : 0.0;
    std::vector<Mat4> pose;
    device->SetFVF(kWorldVertexFvf);
    size_t bound = (size_t)-1;
    for (const nfs::WorldAnimationPlacement& placement : animations.Placements()) {
        // The game shows a tree only while its section is streamed in.
        if (usingVisibility && !std::binary_search(visibleSections_.begin(), visibleSections_.end(),
                                                   (u32)placement.section))
            continue;
        if (GameHidesSection((u32)placement.section)) continue;   // SetGameDrawFilter
        animations.Pose(placement, seconds, pose);
        const nfs::WorldAnimationTree& tree = animations.Trees()[placement.tree];
        for (size_t i = 0; i < tree.parts.size() && i < pose.size(); ++i) {
            if (!tree.parts[i].model) continue;
            const auto found = objectByHash_.find(tree.parts[i].model);
            if (found == objectByHash_.end()) continue;
            const size_t s = found->second.first;
            const GpuSection& gpu = *sections_[s].gpu;
            const GpuObject& object = gpu.Objects()[found->second.second];
            if (bound != s) {
                device->SetStreamSource(0, gpu.VertexBuffer(), 0, sizeof(nfs::SolidVertex));
                device->SetIndices(gpu.IndexBuffer());
                bound = s;
            }
            const bool mirrored = Determinant3x3(pose[i]) < 0.0f;
            if (backfaceCulling_)
                device->SetRenderState(D3DRS_CULLMODE, mirrored ? OppositeCull(cullMode_) : cullMode_);
            const D3DMATRIX world = ToD3D(pose[i]);
            device->SetTransform(D3DTS_WORLD, &world);
            stats.instancesVisible++;
            if (DrawObject(device, object, stats))
                QueueTranslucent(object, s, pose[i], mirrored,
                                 object.source ? Transform(object.source->bounds, pose[i]) : Aabb{}, eye);
        }
    }
    if (backfaceCulling_) device->SetRenderState(D3DRS_CULLMODE, cullMode_);
    static bool told = false;
    if (!told) {
        told = true;
        size_t visible = 0, parts = 0, missing = 0;
        for (const nfs::WorldAnimationPlacement& placement : animations.Placements()) {
            if (usingVisibility && !std::binary_search(visibleSections_.begin(), visibleSections_.end(),
                                                       (u32)placement.section))
                continue;
            ++visible;
            for (const nfs::WorldAnimationPart& part : animations.Trees()[placement.tree].parts)
                if (part.model) (objectByHash_.count(part.model) ? parts : missing)++;
        }
        LOG_INFO("object animations: %zu of %zu placements in view, %zu parts drawn, %zu models not found",
                 visible, animations.Placements().size(), parts, missing);
    }
}

bool WorldRenderer::FindGpuModel(u32 hash, const GpuSection*& gpu, const GpuObject*& object) {
    if (objectByHash_.empty()) {
        for (size_t s = 0; s < sections_.size(); ++s) {
            if (!sections_[s].gpu) continue;
            const auto& objects = sections_[s].gpu->Objects();
            for (size_t o = 0; o < objects.size(); ++o)
                if (objects[o].source && !objects[o].draws.empty())
                    objectByHash_.emplace(objects[o].source->hash, std::make_pair(s, o));
        }
    }
    const auto found = objectByHash_.find(hash);
    if (found == objectByHash_.end()) return false;
    gpu = sections_[found->second.first].gpu.get();
    object = &gpu->Objects()[found->second.second];
    return true;
}

void WorldRenderer::BeginUserMesh(IDirect3DDevice9* device) {
    const D3DMATRIX identity = ToD3D(Mat4::Identity());
    device->SetTransform(D3DTS_WORLD, &identity);
    device->SetFVF(kWorldVertexFvf);
    device->SetRenderState(D3DRS_CULLMODE, backfaceCulling_ ? cullMode_ : D3DCULL_NONE);
    device->SetRenderState(D3DRS_FILLMODE, mode_ == RenderMode::Wireframe ? D3DFILL_WIREFRAME : D3DFILL_SOLID);
}

void WorldRenderer::BindUserTexture(IDirect3DDevice9* device, u32 textureHash) {
    const bool wantsTexture = mode_ == RenderMode::Textured || mode_ == RenderMode::Unlit;
    BindTextureStage(device, wantsTexture ? textures_.Get(device, textureHash) : CachedTexture{});
}

void WorldRenderer::BeginOverlay(IDirect3DDevice9* device) {
    device->SetTexture(0, nullptr);
    device->SetTextureStageState(0, D3DTSS_COLOROP,   D3DTOP_SELECTARG1);
    device->SetTextureStageState(0, D3DTSS_COLORARG1, D3DTA_DIFFUSE);
    device->SetTextureStageState(0, D3DTSS_ALPHAOP,   D3DTOP_SELECTARG1);
    device->SetTextureStageState(0, D3DTSS_ALPHAARG1, D3DTA_DIFFUSE);
    const D3DMATRIX identity = ToD3D(Mat4::Identity());
    device->SetTransform(D3DTS_WORLD, &identity);
    device->SetFVF(CollisionMesh::kFvf);
    device->SetRenderState(D3DRS_FILLMODE, D3DFILL_SOLID);
    device->SetRenderState(D3DRS_ALPHATESTENABLE, FALSE);
    device->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
}

void WorldRenderer::EndOverlay(IDirect3DDevice9* device) {
    device->SetRenderState(D3DRS_ZENABLE, TRUE);
    device->SetRenderState(D3DRS_ZWRITEENABLE, TRUE);
    device->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
    device->SetRenderState(D3DRS_SRCBLEND,  D3DBLEND_SRCALPHA);
    device->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA);
    device->SetRenderState(D3DRS_ALPHATESTENABLE, TRUE);
    device->SetRenderState(D3DRS_CULLMODE, backfaceCulling_ ? cullMode_ : D3DCULL_NONE);
    device->SetRenderState(D3DRS_FILLMODE,
                           mode_ == RenderMode::Wireframe ? D3DFILL_WIREFRAME : D3DFILL_SOLID);
    device->SetFVF(kWorldVertexFvf);
}

// ---------------------------------------------------------------------------
// Thumbnails
// ---------------------------------------------------------------------------

namespace {

// A neutral mid grey that neither dark roofs nor pale concrete disappear into.
constexpr D3DCOLOR kThumbnailBackground = 0xFF3C4148;

void ReleaseCom(IUnknown*& object) {
    if (object) object->Release();
    object = nullptr;
}

} // namespace

void WorldRenderer::ReleaseThumbnailTargets() {
    IUnknown* target = thumbTarget_;
    IUnknown* depth = thumbDepth_;
    IUnknown* readback = thumbReadback_;
    ReleaseCom(target);
    ReleaseCom(depth);
    ReleaseCom(readback);
    thumbTarget_ = nullptr;
    thumbDepth_ = nullptr;
    thumbReadback_ = nullptr;
    thumbSize_ = 0;
}

bool WorldRenderer::RenderThumbnail(u32 modelHash, u32 size, std::vector<u32>& bgra) {
    if (!context_ || !context_->IsValid() || size == 0 || size > 1024) return false;
    IDirect3DDevice9* device = context_->Device();
    if (device->TestCooperativeLevel() != D3D_OK) return false;

    if (objectByHash_.empty()) {
        for (size_t s = 0; s < sections_.size(); ++s) {
            if (!sections_[s].gpu) continue;
            const auto& objects = sections_[s].gpu->Objects();
            for (size_t o = 0; o < objects.size(); ++o)
                if (objects[o].source && !objects[o].draws.empty())
                    objectByHash_.emplace(objects[o].source->hash, std::make_pair(s, o));
        }
    }
    const GpuSection* gpuFound = nullptr;
    const GpuObject* objectFound = nullptr;
    const auto found = objectByHash_.find(modelHash);
    if (found != objectByHash_.end()) {
        gpuFound = sections_[found->second.first].gpu.get();
        objectFound = &gpuFound->Objects()[found->second.second];
    } else {
        // A model of an extension's own, which the region does not hold.
        for (RenderExtension* extension : extensions_)
            if (extension->FindModel(modelHash, gpuFound, objectFound)) break;
    }
    if (!gpuFound || !objectFound || !objectFound->source) return false;
    const GpuSection& gpu = *gpuFound;
    const GpuObject& object = *objectFound;
    const nfs::SolidObject& model = *object.source;

    Aabb box = model.bounds;
    if (!box.Valid())
        for (const nfs::SolidVertex& vertex : model.vertices) box.Add(vertex.position);
    if (!box.Valid()) return false;

    // Drawn at twice the size and averaged down: the cheapest antialiasing
    // there is, and thin things -- poles, rails, wires -- survive it.
    const u32 render = size * 2;
    if (thumbSize_ != render) {
        ReleaseThumbnailTargets();
        if (FAILED(device->CreateTexture(render, render, 1, D3DUSAGE_RENDERTARGET, D3DFMT_A8R8G8B8,
                                         D3DPOOL_DEFAULT, &thumbTarget_, nullptr)) ||
            (FAILED(device->CreateDepthStencilSurface(render, render, D3DFMT_D24S8,
                                                      D3DMULTISAMPLE_NONE, 0, TRUE, &thumbDepth_,
                                                      nullptr)) &&
             FAILED(device->CreateDepthStencilSurface(render, render, D3DFMT_D16,
                                                      D3DMULTISAMPLE_NONE, 0, TRUE, &thumbDepth_,
                                                      nullptr))) ||
            FAILED(device->CreateOffscreenPlainSurface(render, render, D3DFMT_A8R8G8B8,
                                                       D3DPOOL_SYSTEMMEM, &thumbReadback_,
                                                       nullptr))) {
            ReleaseThumbnailTargets();
            return false;
        }
        thumbSize_ = render;
    }

    IDirect3DSurface9* oldTarget = nullptr;
    IDirect3DSurface9* oldDepth = nullptr;
    D3DVIEWPORT9 oldViewport{};
    device->GetRenderTarget(0, &oldTarget);
    device->GetDepthStencilSurface(&oldDepth);
    device->GetViewport(&oldViewport);

    IDirect3DSurface9* target = nullptr;
    thumbTarget_->GetSurfaceLevel(0, &target);
    device->SetRenderTarget(0, target);
    device->SetDepthStencilSurface(thumbDepth_);
    const D3DVIEWPORT9 viewport{ 0, 0, render, render, 0.0f, 1.0f };
    device->SetViewport(&viewport);
    device->Clear(0, nullptr, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, kThumbnailBackground, 1.0f, 0);

    // From in front, to the left and above, the way a catalogue photographs
    // a thing: three faces showing. Two shapes need a view of their own,
    // since from there they are a line: something lying flat -- a start line
    // painted on the road, a planter bed -- is seen from nearly overhead, and
    // a panel standing up -- a wall, a sign, a fence -- from in front of its
    // face.
    const Vec3 extent = box.max - box.min;
    const Vec3 centre = (box.min + box.max) * 0.5f;
    const float across = std::max(extent.x, extent.y);
    Vec3 toEye{ -0.62f, -0.78f, 0.52f };
    if (extent.z < across * 0.12f)
        toEye = { -0.22f, -0.38f, 1.0f };
    else if (std::min(extent.x, extent.y) < across * 0.15f)
        toEye = extent.x < extent.y ? Vec3{ -1.0f, -0.32f, 0.3f } : Vec3{ -0.32f, -1.0f, 0.3f };
    toEye = Normalize(toEye);

    const float radius = std::max(0.25f, Length(extent) * 0.5f);
    const Mat4 view = LookAtRH(centre + toEye * (radius * 4.0f), centre, Vec3{ 0.0f, 0.0f, 1.0f });

    // Orthographic, fitted to the box as it lands on screen rather than to
    // the sphere around it: a guard rail forty units long and one high fills
    // its square instead of being a thread across the middle of it.
    float left = 1e30f, right = -1e30f, bottom = 1e30f, top = -1e30f, nearZ = 1e30f, farZ = -1e30f;
    for (int corner = 0; corner < 8; ++corner) {
        const Vec3 p{ corner & 1 ? box.max.x : box.min.x, corner & 2 ? box.max.y : box.min.y,
                      corner & 4 ? box.max.z : box.min.z };
        const Vec3 v = view.TransformPoint(p);
        left = std::min(left, v.x);     right = std::max(right, v.x);
        bottom = std::min(bottom, v.y); top = std::max(top, v.y);
        nearZ = std::min(nearZ, -v.z);  farZ = std::max(farZ, -v.z);
    }
    const float half = std::max(0.05f, std::max(right - left, top - bottom) * 0.5f * 1.06f);
    const float midX = (left + right) * 0.5f, midY = (bottom + top) * 0.5f;
    left = midX - half; right = midX + half; bottom = midY - half; top = midY + half;
    nearZ = std::max(0.01f, nearZ - radius * 0.1f);
    farZ += radius * 0.1f;

    Mat4 projection;
    projection.m[0][0] = 2.0f / (right - left);
    projection.m[1][1] = 2.0f / (top - bottom);
    projection.m[2][2] = 1.0f / (nearZ - farZ);
    projection.m[3][0] = (left + right) / (left - right);
    projection.m[3][1] = (top + bottom) / (bottom - top);
    projection.m[3][2] = nearZ / (nearZ - farZ);
    projection.m[3][3] = 1.0f;

    const D3DMATRIX d3dView = ToD3D(view);
    const D3DMATRIX d3dProjection = ToD3D(projection);
    const D3DMATRIX d3dWorld = ToD3D(Mat4::Identity());

    // Unlit: the colours hold the game's night lighting, which leaves half the
    // city too dark to recognise at this size. Both sides drawn, since a shell
    // seen from the wrong side would otherwise come out empty.
    const RenderMode mode = mode_;
    mode_ = RenderMode::Unlit;
    ApplyRenderStates();
    device->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);

    bool drawn = false;
    if (SUCCEEDED(device->BeginScene())) {
        device->SetTransform(D3DTS_VIEW, &d3dView);
        device->SetTransform(D3DTS_PROJECTION, &d3dProjection);
        device->SetTransform(D3DTS_WORLD, &d3dWorld);
        device->SetStreamSource(0, gpu.VertexBuffer(), 0, sizeof(nfs::SolidVertex));
        device->SetIndices(gpu.IndexBuffer());
        RenderStats stats;
        if (DrawObject(device, object, stats)) {
            BeginTranslucentStates(device);
            DrawObject(device, object, stats, DrawPass::Translucent);
            EndTranslucentStates(device);
        }
        device->EndScene();
        drawn = true;
    }
    mode_ = mode;

    bool read = false;
    if (drawn && SUCCEEDED(device->GetRenderTargetData(target, thumbReadback_))) {
        D3DLOCKED_RECT locked{};
        if (SUCCEEDED(thumbReadback_->LockRect(&locked, nullptr, D3DLOCK_READONLY))) {
            bgra.assign((size_t)size * size, 0);
            for (u32 y = 0; y < size; ++y) {
                const u32* a = (const u32*)((const u8*)locked.pBits + (size_t)(2 * y) * locked.Pitch);
                const u32* b = (const u32*)((const u8*)locked.pBits + (size_t)(2 * y + 1) * locked.Pitch);
                for (u32 x = 0; x < size; ++x) {
                    const u32 p[4] = { a[2 * x], a[2 * x + 1], b[2 * x], b[2 * x + 1] };
                    u32 out = 0xFF000000u;
                    for (u32 shift = 0; shift < 24; shift += 8) {
                        u32 sum = 0;
                        for (u32 k = 0; k < 4; ++k) sum += (p[k] >> shift) & 0xFF;
                        out |= ((sum + 2) / 4) << shift;
                    }
                    bgra[(size_t)y * size + x] = out;
                }
            }
            thumbReadback_->UnlockRect();
            read = true;
        }
    }

    device->SetRenderTarget(0, oldTarget);
    device->SetDepthStencilSurface(oldDepth);
    device->SetViewport(&oldViewport);
    if (target) target->Release();
    if (oldTarget) oldTarget->Release();
    if (oldDepth) oldDepth->Release();
    return read;
}

void WorldRenderer::DrawGizmo(IDirect3DDevice9* device, const Camera& camera) {
    if (!gizmoVisible_ || !context_) return;
    GizmoView view;
    const float aspect = context_->Height() ? (float)context_->Width() / (float)context_->Height() : 1.0f;
    view.viewProjection = camera.ViewMatrix() * camera.ProjectionMatrix(aspect);
    view.width = (float)context_->Width();
    view.height = (float)context_->Height();
    view.eye = camera.Position();

    std::vector<GizmoVertex> lines, triangles;
    BuildGizmo(gizmoMode_, gizmoOrigin_, view, gizmoHot_, lines, triangles);

    device->SetTexture(0, nullptr);
    device->SetTextureStageState(0, D3DTSS_COLOROP,   D3DTOP_SELECTARG1);
    device->SetTextureStageState(0, D3DTSS_COLORARG1, D3DTA_DIFFUSE);
    device->SetTextureStageState(0, D3DTSS_ALPHAOP,   D3DTOP_SELECTARG1);
    device->SetTextureStageState(0, D3DTSS_ALPHAARG1, D3DTA_DIFFUSE);
    const D3DMATRIX identity = ToD3D(Mat4::Identity());
    device->SetTransform(D3DTS_WORLD, &identity);
    device->SetFVF(CollisionMesh::kFvf);
    device->SetRenderState(D3DRS_FILLMODE, D3DFILL_SOLID);
    device->SetRenderState(D3DRS_ALPHATESTENABLE, FALSE);
    device->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
    device->SetRenderState(D3DRS_ALPHABLENDENABLE, TRUE);
    device->SetRenderState(D3DRS_SRCBLEND,  D3DBLEND_SRCALPHA);
    device->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA);
    // Over the world, never buried in the building it belongs to.
    device->SetRenderState(D3DRS_ZENABLE, FALSE);
    if (!triangles.empty())
        device->DrawPrimitiveUP(D3DPT_TRIANGLELIST, (UINT)(triangles.size() / 3), triangles.data(),
                                sizeof(GizmoVertex));
    if (!lines.empty())
        device->DrawPrimitiveUP(D3DPT_LINELIST, (UINT)(lines.size() / 2), lines.data(), sizeof(GizmoVertex));
    device->SetRenderState(D3DRS_ZENABLE, TRUE);
    device->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
    device->SetRenderState(D3DRS_ALPHATESTENABLE, TRUE);
    device->SetRenderState(D3DRS_CULLMODE, backfaceCulling_ ? cullMode_ : D3DCULL_NONE);
}

// ---------------------------------------------------------------------------
// MINIMAP PREVIEW (temporary)
// ---------------------------------------------------------------------------

std::vector<minimap::PlacedModel> WorldRenderer::PlacedBuildings() const {
    std::vector<minimap::PlacedModel> out;
    for (const SectionRender& section : sections_) {
        if (section.alternate && !showAlternate_) continue;
        for (const GpuInstance& instance : section.instances) {
            const GpuSection* gpu = GpuOf(instance.section);
            if (!gpu || instance.object >= gpu->Objects().size()) continue;
            const nfs::SolidObject* model = gpu->Objects()[instance.object].source;
            if (model && minimap::IsMinimapBuilding(*model))
                out.push_back({ model, instance.transform });
        }
    }
    return out;
}

bool WorldRenderer::SetMinimapBuildings(const std::vector<minimap::Building>& buildings) {
    if (!context_ || !context_->IsValid()) return false;
    return minimapPreview_.Upload(context_->Device(), buildings);
}

// ---------------------------------------------------------------------------
// Game look: the game's own WORLD / WORLDREFLECT / WORLDNOFOG effects
// ---------------------------------------------------------------------------

namespace {

constexpr int kEffectWorldNoFog = 7;   // gEffects[7]

template <typename T>
void ReleaseTyped(T*& object) {
    if (object) object->Release();
    object = nullptr;
}

} // namespace

bool WorldRenderer::BackdropHidden(const GpuObject& object) const {
    // DrawSky owns the dome; drawing the SKYDOME mesh as well double-draws it.
    if (world_ && world_->IsWorld() && object.source &&
        object.source->name.rfind("SKYDOME", 0) == 0)
        return true;
    if (object.backdropKind == kBackdropPanorama && hideBackground_) return true;
    if (object.backdropKind == kBackdropDistantLod && hideDistantLod_) return true;
    if (!object.isBackdrop || showBackdrop_) return false;
    // The game draws the panorama pieces as ordinary scenery (in the zones
    // that list their sections, see BackdropOutOfZone); the dome is drawn
    // round the camera by DrawSky.
    if (gameLook_ && gameLookReady_ && object.source && object.source->name.rfind("SKYDOME", 0) != 0) return false;
    return true;
}

// The section set the game draws for the camera (STREAMING-STUDY 1.7, sub_5E99C0 / sub_5E96B0): the zone
// under the camera (sub_5E00D0) and its visible-section list; with no zone, only sections numbered n % 100
// < 10. Kept while the camera stays within 2 units.
void WorldRenderer::UpdateGameZone(const Vec3& eye) {
    const float dx = eye.x - glZoneFrom_.x, dy = eye.y - glZoneFrom_.y;
    if (dx * dx + dy * dy < 4.0f || !world_) return;
    glZoneFrom_ = eye;
    std::vector<u32> found;
    glZoneFound_ = world_->Visibility().VisibleAt(eye, found);
    std::sort(found.begin(), found.end());
    glZoneList_.swap(found);
}

bool WorldRenderer::BackdropOutOfZone(const GpuObject& object, u32 sectionNumber) const {
    if (object.backdropKind == kBackdropNone || !glActive_ || !glFogged_) return false;
    if (!glZoneFound_) return sectionNumber % 100 >= 10;
    return !std::binary_search(glZoneList_.begin(), glZoneList_.end(), sectionNumber);
}

void WorldRenderer::ApplyBaseVisibility() {
    culling_ = true;
    backfaceCulling_ = true;
    showScenery_ = true;
    showProxies_ = false;
    showLibrary_ = false;
    showAlternate_ = false;
    showCollision_ = false;
    showRaceGroups_ = false;
    editedOnly_ = false;
    // Race props (barriers, start lines) belong to the events that name their
    // scenery group; free roam never shows them (EnableBarrierSceneryGroup).
    showRaceProps_ = false;
    animate_ = true;
    showSky_ = true;
    showMinimap_ = false;
    showOriginalBuildings_ = true;
    hiddenModels_ = nullptr;
    gameSection_ = nullptr;
    gameModel_ = nullptr;
    // Presets are a reset: show the user-hidden items again.
    hiddenKeys_.clear();
    ++hiddenVersion_;
}

void WorldRenderer::ApplyGameStandardView() {
    ApplyBaseVisibility();
    showBackdrop_ = true;
    hideBackground_ = false;
    hideDistantLod_ = false;
    gameZoneFallback_ = true;
    if (world_ && !world_->Visibility().IsEmpty()) streaming_ = true;
}

bool WorldRenderer::DistanceCulls() const {
    return streaming_ && (!world_ || !world_->IsWorld() || haveVisible_);
}

void WorldRenderer::ReleaseDeviceObjects() {
    ReleaseTyped(glReflection_);
    ReleaseTyped(glReflectionDepth_);
    ReleaseTyped(glReflectionBlur_);
    glReflectionFailed_ = false;
    glReflectionValid_ = false;
}

void WorldRenderer::ReleaseGameLook() {
    ReleaseDeviceObjects();
    ReleaseTyped(glWorldVs_);
    ReleaseTyped(glWorldPs_);
    ReleaseTyped(glReflectVs_);
    ReleaseTyped(glReflectPs_);
    ReleaseTyped(glNoFogVs_);
    ReleaseTyped(glNoFogPs_);
    ReleaseTyped(glFilterVs_);
    ReleaseTyped(glFilterPs_);
    ReleaseTyped(glMissing_);
    gameLookReady_ = false;
    gameLookTried_ = false;
    fogRegion_.clear();
    glZoneList_.clear();
    glZoneFound_ = false;
    glZoneFrom_ = Vec3{ 1e30f, 1e30f, 1e30f };
}

bool WorldRenderer::EnsureGameLook(IDirect3DDevice9* device) {
    if (gameLookReady_) return true;
    if (gameLookTried_ || !device) return false;
    gameLookTried_ = true;
    if (gameFolder_.empty()) {
        gameLookStatus_ = "Game look: no game folder";
        return false;
    }
    std::wstring exe = gameFolder_;
    if (exe.back() != L'\\' && exe.back() != L'/') exe += L'\\';
    exe += L"SPEED2.EXE";
    gamelook::WorldEffects effects;
    std::string why;
    if (!gamelook::ExtractWorldEffects(exe, effects, why)) {
        gameLookStatus_ = "Game look unavailable: " + why;
        LOG_WARN("%s", gameLookStatus_.c_str());
        return false;
    }
    auto vs = [&](const std::vector<uint32_t>& code, IDirect3DVertexShader9*& out) {
        return SUCCEEDED(device->CreateVertexShader(reinterpret_cast<const DWORD*>(code.data()), &out));
    };
    auto ps = [&](const std::vector<uint32_t>& code, IDirect3DPixelShader9*& out) {
        return SUCCEEDED(device->CreatePixelShader(reinterpret_cast<const DWORD*>(code.data()), &out));
    };
    if (!vs(effects.world.vertexShader, glWorldVs_) || !ps(effects.world.pixelShader, glWorldPs_) ||
        !vs(effects.reflect.vertexShader, glReflectVs_) || !ps(effects.reflect.pixelShader, glReflectPs_) ||
        !vs(effects.noFog.vertexShader, glNoFogVs_) || !ps(effects.noFog.pixelShader, glNoFogPs_)) {
        ReleaseGameLook();
        gameLookTried_ = true;
        gameLookStatus_ = "Game look unavailable: the device refused the game's vs_1_1 / ps_1_1";
        LOG_WARN("%s", gameLookStatus_.c_str());
        return false;
    }
    // What was taken, instruction by instruction, to compare with the published disassembly.
    const std::pair<const char*, const std::vector<uint32_t>*> listed[] = {
        { "WORLD vs", &effects.world.vertexShader },          { "WORLD ps", &effects.world.pixelShader },
        { "WORLDREFLECT vs", &effects.reflect.vertexShader }, { "WORLDREFLECT ps", &effects.reflect.pixelShader },
        { "WORLDNOFOG vs", &effects.noFog.vertexShader },     { "WORLDNOFOG ps", &effects.noFog.pixelShader },
    };
    for (const auto& item : listed) {
        LOG_INFO("game look: %s, %zu bytes", item.first, item.second->size() * 4);
        for (const std::string& line : gamelook::Disassemble(*item.second)) LOG_INFO("    %s", line.c_str());
    }
    // FILTER pass p3 for the level-3 blur of the reflection (sub_5C7E40); optional.
    if (!effects.filter.vertexShader.empty() && !effects.filter.pixelShader.empty()) {
        if (!vs(effects.filter.vertexShader, glFilterVs_) || !ps(effects.filter.pixelShader, glFilterPs_)) {
            ReleaseTyped(glFilterVs_);
            ReleaseTyped(glFilterPs_);
        } else {
            LOG_INFO("game look: FILTER p3 vs %zu bytes, ps %zu bytes", effects.filter.vertexShader.size() * 4,
                     effects.filter.pixelShader.size() * 4);
            for (const std::string& line : gamelook::Disassemble(effects.filter.vertexShader)) LOG_INFO("    %s", line.c_str());
            for (const std::string& line : gamelook::Disassemble(effects.filter.pixelShader)) LOG_INFO("    %s", line.c_str());
        }
    }
    if (SUCCEEDED(device->CreateTexture(1, 1, 1, 0, D3DFMT_A8R8G8B8, D3DPOOL_MANAGED, &glMissing_, nullptr))) {
        D3DLOCKED_RECT locked{};
        if (SUCCEEDED(glMissing_->LockRect(0, &locked, nullptr, 0))) {
            *static_cast<u32*>(locked.pBits) = kMissingTextureColor;
            glMissing_->UnlockRect(0);
        }
    }
    // sub_60F880 without the time-of-day option (dword_870D14 != 1) attaches this replacement table to
    // SKYDOME: SKY_SUNRISE_A -> SKY_NIGHT_A, SKY_SUNRISE_A_CAP -> SKY_NIGHT_A_CAP.
    glSkyRemap_.clear();
    glSkyRemap_[gamelook::GameHash("SKY_SUNRISE_A")] = gamelook::GameHash("SKY_NIGHT_A");
    glSkyRemap_[gamelook::GameHash("SKY_SUNRISE_A_CAP")] = gamelook::GameHash("SKY_NIGHT_A_CAP");
    gameLookReady_ = true;
    gameLookStatus_ = "Game look: WORLD, WORLDREFLECT and WORLDNOFOG from SPEED2.EXE";
    LOG_INFO("%s", gameLookStatus_.c_str());
    return true;
}

void WorldRenderer::BeginGameLook(IDirect3DDevice9* device, const Mat4& view, const Mat4& projection,
                                  bool mainView, const Vec3& eye) {
    glActive_ = true;
    glFogged_ = mainView;
    glBound_ = -1;
    glView_ = view;
    glProjection_ = projection;

    // The fog struct of this view (sub_614300) and the constants eDisplayFrame derives from it.
    if (mainView) {
        std::string region;
        if (world_)
            for (wchar_t ch : world_->Region()) region += (char)ch;
        if (region != fogRegion_) {
            fogRegion_ = region;
            std::wstring folder = gameFolder_;
            while (!folder.empty() && (folder.back() == L'\\' || folder.back() == L'/')) folder.pop_back();
            // The copy the game will read when the world has one (fog by neighbourhood, io/DistrictFog).
            const std::wstring bun = io::RegionFileForGame(folder, std::wstring(region.begin(), region.end()));
            if (glFogMap_.Load(bun))
                LOG_INFO("game look: fog from the \"Normal Fog\" parameter map of %S", bun.c_str());
            else
                LOG_WARN("game look: %S has no \"Normal Fog\" map; using the exe's fallback fog (0x800E18)",
                         bun.c_str());
        }
        const gamelook::FogState fog = glFogMap_.Sample(eye.x, eye.y);
        const gamelook::FogConstants c = gamelook::MakeFogConstants(fog);
        std::memcpy(glFogValue_, c.value, sizeof glFogValue_);
        static int lastSet = -2;
        const int set = glFogMap_.SetIndexAt(eye.x, eye.y);
        if (set != lastSet) {
            lastSet = set;
            LOG_INFO("game look: fog set %d at (%.0f, %.0f): start %.1f, factor %.3f, colour %.0f %.0f %.0f -> "
                     "FOGVALUE (%.0f, %.5f, %.3f), FOGCOLOR %08X",
                     set, eye.x, eye.y, fog.start, fog.factor, fog.r, fog.g, fog.b, c.value[0], c.value[1],
                     c.value[2], (unsigned)c.colour);
        }
        device->SetRenderState(D3DRS_FOGCOLOR, c.colour);
    }
    // FOGENABLE comes from each effect's pass (FogOn = g_FogEnable for WORLD / WORLDREFLECT, FALSE for
    // WORLDNOFOG); the shaders write oFog, so no fixed-function fog formula is wanted.
    device->SetRenderState(D3DRS_FOGTABLEMODE, D3DFOG_NONE);
    device->SetRenderState(D3DRS_FOGVERTEXMODE, D3DFOG_NONE);
    device->SetRenderState(D3DRS_FOGENABLE, FALSE);
    device->SetRenderState(D3DRS_LIGHTING, FALSE);
    device->SetRenderState(D3DRS_COLORWRITEENABLE, 15);
    device->SetRenderState(D3DRS_ZFUNC, D3DCMP_LESSEQUAL);
    device->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);   // every world effect's .fxo: CULLMODE = NONE
    device->SetRenderState(D3DRS_ALPHAFUNC, D3DCMP_GREATEREQUAL);
    device->SetFVF(kWorldVertexFvf);
    for (DWORD stage = 0; stage < 2; ++stage) {
        device->SetTextureStageState(stage, D3DTSS_TEXCOORDINDEX, stage);
        device->SetTextureStageState(stage, D3DTSS_TEXTURETRANSFORMFLAGS, D3DTTFF_DISABLE);
    }
    scrolling_ = false;
}

void WorldRenderer::EndGameLook(IDirect3DDevice9* device) {
    glActive_ = false;
    glBound_ = -1;
    device->SetVertexShader(nullptr);
    device->SetPixelShader(nullptr);
    device->SetRenderState(D3DRS_FOGENABLE, FALSE);
    device->SetTexture(1, nullptr);
    device->SetTextureStageState(1, D3DTSS_TEXTURETRANSFORMFLAGS, D3DTTFF_DISABLE);
    device->SetTextureStageState(1, D3DTSS_COLOROP, D3DTOP_DISABLE);
    device->SetTextureStageState(1, D3DTSS_ALPHAOP, D3DTOP_DISABLE);
    device->SetTextureStageState(0, D3DTSS_TEXTURETRANSFORMFLAGS, D3DTTFF_DISABLE);
    scrolling_ = false;
    // Back to the states ApplyRenderStates leaves for the editor's own drawing.
    device->SetRenderState(D3DRS_ALPHATESTENABLE, TRUE);
    device->SetRenderState(D3DRS_ALPHAFUNC, D3DCMP_GREATEREQUAL);
    device->SetRenderState(D3DRS_ALPHAREF, 128);
    device->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
    device->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_SRCALPHA);
    device->SetRenderState(D3DRS_DESTBLEND, D3DBLEND_INVSRCALPHA);
    device->SetRenderState(D3DRS_ZWRITEENABLE, TRUE);
    device->SetRenderState(D3DRS_CULLMODE, backfaceCulling_ ? cullMode_ : D3DCULL_NONE);
    device->SetFVF(kWorldVertexFvf);
    BindTextureStage(device, CachedTexture{});
}

bool WorldRenderer::DrawObjectGameLook(IDirect3DDevice9* device, const GpuObject& object, RenderStats& stats,
                                       DrawPass pass) {
    // sub_5BDDE0: WORLDVIEW = l2w * View, WORLDVIEWPROJECTION = l2w * ViewProj. The vs_1_1 code reads
    // them as columns (dp4 oPos.x, v0, c0). The game's view space looks down +z; the editor's (right
    // handed) looks down -z, so WorldView's third column -- the one the fog reads -- goes in negated.
    D3DMATRIX d3dWorld{};
    device->GetTransform(D3DTS_WORLD, &d3dWorld);
    Mat4 world;
    std::memcpy(world.m, &d3dWorld, sizeof d3dWorld);
    const Mat4 worldView = world * glView_;
    const Mat4 worldViewProjection = worldView * glProjection_;
    float c[7][4];
    for (int r = 0; r < 4; ++r)
        for (int k = 0; k < 4; ++k) c[r][k] = worldViewProjection.m[k][r];
    for (int r = 0; r < 3; ++r)
        for (int k = 0; k < 4; ++k) c[4 + r][k] = worldView.m[k][r] * (r == 2 ? -1.0f : 1.0f);
    device->SetVertexShaderConstantF(0, &c[0][0], 7);

    bool other = false;
    for (const GpuDraw& draw : object.draws) {
        u32 hash = AnimatedTexture(draw.textureHash);
        if (glDrawingSky_) {
            const auto remap = glSkyRemap_.find(hash);
            if (remap != glSkyRemap_.end()) hash = remap->second;
        }
        const CachedTexture& texture = textures_.Get(device, hash);
        const bool translucent = texture.texture && texture.Translucent();
        if (translucent != (pass == DrawPass::Translucent)) { other = true; continue; }

        // eViewPlatInterface::Render (0x5C5930): the mesh's effect; views other than the player's draw
        // WORLDNOFOG; an additive texture (AlphaBlendType 2) turns WORLD into WORLDNOFOG. WORLDREFLECT
        // needs this frame's reflection; without one it falls back to WORLD, as the game's remap table
        // 0x7FF0D8 does with the road reflection off. CAR (the signs) and WORLDBONE are drawn with
        // WORLD: their lighting rigs are not part of this view.
        int effect = kEffectWorld;
        if (!glFogged_) effect = kEffectWorldNoFog;
        else if (object.effect == kEffectWorldReflect && glReflectionValid_) effect = kEffectWorldReflect;
        if (effect == kEffectWorld && texture.blend == SurfaceBlend::Additive) effect = kEffectWorldNoFog;

        if (effect != glBound_) {
            glBound_ = effect;
            if (effect == kEffectWorldReflect) {
                device->SetVertexShader(glReflectVs_);
                device->SetPixelShader(glReflectPs_);
                device->SetRenderState(D3DRS_FOGENABLE, TRUE);
                // reflected_sampler: WRAP, MIP/MIN/MAG LINEAR; TEXTURETRANSFORMFLAGS[1] = PROJECTED.
                device->SetTexture(1, glReflection_);
                device->SetTextureStageState(1, D3DTSS_TEXCOORDINDEX, 1);
                // The .fxo states PROJECTED alone; with a vertex shader, current drivers need the count
                // to divide by the fourth coordinate (oT1.w = clip w), as old ones did implicitly.
                device->SetTextureStageState(1, D3DTSS_TEXTURETRANSFORMFLAGS, D3DTTFF_COUNT4 | D3DTTFF_PROJECTED);
                device->SetSamplerState(1, D3DSAMP_ADDRESSU, D3DTADDRESS_WRAP);
                device->SetSamplerState(1, D3DSAMP_ADDRESSV, D3DTADDRESS_WRAP);
                device->SetSamplerState(1, D3DSAMP_MINFILTER, D3DTEXF_LINEAR);
                device->SetSamplerState(1, D3DSAMP_MAGFILTER, D3DTEXF_LINEAR);
                device->SetSamplerState(1, D3DSAMP_MIPFILTER, D3DTEXF_LINEAR);
                device->SetVertexShaderConstantF(7, glFogValue_, 1);
            } else {
                device->SetVertexShader(effect == kEffectWorld ? glWorldVs_ : glNoFogVs_);
                device->SetPixelShader(effect == kEffectWorld ? glWorldPs_ : glNoFogPs_);
                device->SetRenderState(D3DRS_FOGENABLE, effect == kEffectWorld ? TRUE : FALSE);
                device->SetTexture(1, nullptr);
                device->SetTextureStageState(1, D3DTSS_TEXTURETRANSFORMFLAGS, D3DTTFF_DISABLE);
                if (effect == kEffectWorld) device->SetVertexShaderConstantF(8, glFogValue_, 1);
            }
        }
        // TEXTUREOFFSET (sub_5BCC30, the texture's scroll): WORLD c7, WORLDNOFOG c4. WORLDNOFOG's c4
        // overwrites WorldView's first column, which neither WORLD nor WORLDREFLECT reads.
        if (effect != kEffectWorldReflect) {
            float offset[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
            if (animate_ && texture.scrollType != 0) ScrollOffset(texture, animationClock_, offset[0], offset[1]);
            device->SetVertexShaderConstantF(effect == kEffectWorld ? 7 : 4, offset, 1);
        }

        // sub_5BCC30: ALPHATEST at 0x5F for PUNCHTHRU, blend from AlphaBlendType, ZWRITE = opaque.
        device->SetTexture(0, texture.texture ? texture.texture : glMissing_);
        const bool punch = texture.texture && texture.punchThru;
        device->SetRenderState(D3DRS_ALPHATESTENABLE, punch ? TRUE : FALSE);
        device->SetRenderState(D3DRS_ALPHAREF, punch ? 0x5F : 0);
        device->SetRenderState(D3DRS_ALPHABLENDENABLE, translucent ? TRUE : FALSE);
        if (translucent) {
            device->SetRenderState(D3DRS_SRCBLEND, D3DBLEND_SRCALPHA);
            device->SetRenderState(D3DRS_DESTBLEND,
                                   texture.blend == SurfaceBlend::Additive ? D3DBLEND_ONE : D3DBLEND_INVSRCALPHA);
        }
        device->SetRenderState(D3DRS_ZWRITEENABLE, translucent || glDrawingSky_ ? FALSE : TRUE);
        device->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);

        device->DrawIndexedPrimitive(D3DPT_TRIANGLELIST, (INT)object.baseVertex, draw.minVertex, draw.vertexCount,
                                     draw.startIndex, draw.primitiveCount);
        stats.drawCalls++;
        stats.triangles += draw.primitiveCount;
    }
    return other;
}

// The road reflection (eDisplayFrame 0x5CAB15..0x5CACC6, g_RoadReflectionEnable = 3): view 4 shares the
// player camera and its View; it draws into a 320x240 A8R8G8B8 target (sub_5BA060, own D16 depth),
// cleared to opaque black (sub_5BA3C0), no sky. The scenery set (preculler flag 0x800, sub_5D6570) is
// the instances with flag 0x40, without 0x10 and 0x400, drawn with SceneryInfo +0x28's model when they
// cover 32 pixels or more, each flipped by negating element [2][2] of its matrix (0x5D67A4) -- for an
// upright object, a mirror image about the plane through its own pivot. Every effect but CAR becomes
// WORLDNOFOG in it. At level 3 sub_5C7E40 then blurs it with FILTER p3. WORLDREFLECT samples it at the
// road pixel's own screen position.
void WorldRenderer::RenderReflection(IDirect3DDevice9* device, const Camera& camera, const Mat4& view,
                                     const Mat4& projection) {
    if (glReflectionFailed_ || !device) return;
    constexpr UINT kWidth = 320, kHeight = 240;
    if (!glReflection_) {
        if (FAILED(device->CreateTexture(kWidth, kHeight, 1, D3DUSAGE_RENDERTARGET, D3DFMT_A8R8G8B8, D3DPOOL_DEFAULT,
                                         &glReflection_, nullptr)) ||
            FAILED(device->CreateDepthStencilSurface(kWidth, kHeight, D3DFMT_D16, D3DMULTISAMPLE_NONE, 0, TRUE,
                                                     &glReflectionDepth_, nullptr))) {
            ReleaseDeviceObjects();
            glReflectionFailed_ = true;
            LOG_WARN("game look: no road reflection (the device cannot make a 320x240 render target)");
            return;
        }
        // The second target of the blur (created when the FILTER effect is available, dword_8764EC == 3).
        if (glFilterVs_ && glFilterPs_ &&
            FAILED(device->CreateTexture(kWidth, kHeight, 1, D3DUSAGE_RENDERTARGET, D3DFMT_A8R8G8B8, D3DPOOL_DEFAULT,
                                         &glReflectionBlur_, nullptr)))
            glReflectionBlur_ = nullptr;
    }
    IDirect3DSurface9* surface = nullptr;
    if (FAILED(glReflection_->GetSurfaceLevel(0, &surface)) || !surface) return;

    IDirect3DSurface9* oldTarget = nullptr;
    IDirect3DSurface9* oldDepth = nullptr;
    D3DVIEWPORT9 oldViewport{};
    device->GetRenderTarget(0, &oldTarget);
    device->GetDepthStencilSurface(&oldDepth);
    device->GetViewport(&oldViewport);
    for (DWORD s = 0; s < 4; ++s) device->SetTexture(s, nullptr);

    device->SetRenderTarget(0, surface);
    device->SetDepthStencilSurface(glReflectionDepth_);
    const D3DVIEWPORT9 viewport{ 0, 0, kWidth, kHeight, 0.0f, 1.0f };
    device->SetViewport(&viewport);
    device->Clear(0, nullptr, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, 0xFF000000u, 1.0f, 0);

    const D3DMATRIX d3dView = ToD3D(view);
    const D3DMATRIX d3dProjection = ToD3D(projection);
    device->SetTransform(D3DTS_VIEW, &d3dView);
    device->SetTransform(D3DTS_PROJECTION, &d3dProjection);
    ApplyRenderStates();
    const Vec3 eye = camera.Position();
    BeginGameLook(device, view, projection, false, eye);

    // The frustum test uses the instance's own, unmirrored bounds, as the game's does.
    Frustum frustum;
    frustum.Build(view * projection);
    const Vec3 forward = camera.Forward();
    // sub_5D64D0: pixels = K * r / (d - r) with r = SceneryInfo radius + 6 (flt_7A47CC), d the distance
    // from the eye, K the view's pixel scale (taken here as half the target height over tan(fov / 2)).
    const float pixelScale = 0.5f * (float)kHeight * projection.m[1][1];
    const bool usingVisibility = streaming_ && haveVisible_ && !visibleSections_.empty();

    RenderStats stats;
    translucent_.clear();
    size_t boundSection = (size_t)-1;
    for (size_t i = 0; i < sections_.size() && showScenery_; ++i) {
        const SectionRender& section = sections_[i];
        if (section.alternate && !showAlternate_) continue;
        if (GameHidesSection(section.number)) continue;   // SetGameDrawFilter
        if (usingVisibility) {
            if (!std::binary_search(visibleSections_.begin(), visibleSections_.end(), section.number)) continue;
        } else if (DistanceCulls() && section.instanceBounds.Valid()) {
            const Vec3 centre = (section.instanceBounds.min + section.instanceBounds.max) * 0.5f;
            const float dx = centre.x - eye.x, dy = centre.y - eye.y;
            if (dx * dx + dy * dy > streamRadius_ * streamRadius_) continue;
        }
        if (culling_ && section.instanceBounds.Valid() && !frustum.Intersects(section.instanceBounds)) continue;

        const std::vector<char>& hiddenHere = HiddenFlags(i);
        for (size_t k = 0; k < section.instances.size(); ++k) {
            const GpuInstance& instance = section.instances[k];
            if (!instance.reflects || hiddenHere[k]) continue;
            if (instance.worldBounds.Valid() && !frustum.Intersects(instance.worldBounds)) continue;
            const Vec3 position{ instance.transform.m[3][0], instance.transform.m[3][1], instance.transform.m[3][2] };
            const Vec3 offset = position - eye;
            const float r = instance.radius + 6.0f;
            if (Dot(offset, forward) < -r) continue;
            const float d = Length(offset) - r;
            const float pixels = d > r ? pixelScale * r / d : pixelScale;
            if (pixels < 32.0f) continue;

            const GpuSection* gpu = sections_[instance.reflectSection].gpu.get();
            if (!gpu || instance.reflectObject >= gpu->Objects().size()) continue;
            const GpuObject& object = gpu->Objects()[instance.reflectObject];
            if (InstanceHolidayHidden(section.number, instance.native, &object)) continue;
            if (HiddenModel(object)) continue;
            // View 4 shares the player camera, so its drawing set is the same zone list.
            if (object.backdropKind != kBackdropNone &&
                (BackdropHidden(object) ||
                 (glZoneFound_ ? !std::binary_search(glZoneList_.begin(), glZoneList_.end(), section.number)
                               : section.number % 100 >= 10)))
                continue;
            if (boundSection != instance.reflectSection) {
                device->SetStreamSource(0, gpu->VertexBuffer(), 0, sizeof(nfs::SolidVertex));
                device->SetIndices(gpu->IndexBuffer());
                boundSection = instance.reflectSection;
            }
            Mat4 transform = instance.transform;
            if (!instance.identityPlaced) transform.m[2][2] = -transform.m[2][2];
            const D3DMATRIX world = ToD3D(transform);
            device->SetTransform(D3DTS_WORLD, &world);
            if (DrawObject(device, object, stats))
                QueueTranslucent(object, instance.reflectSection, transform, false, instance.worldBounds, eye);
        }
    }
    DrawTranslucent(device, stats);
    EndGameLook(device);

    surface->Release();
    if (glReflectionBlur_) BlurReflection(device);

    // FRS_REFLECTION_CAPTURE=<file.bmp>: the reflection target as it is sampled, rewritten every 120
    // frames, to check what the roads add.
    static unsigned dumpFrame = 0;
    if (++dumpFrame % 120 == 0) {
        wchar_t path[MAX_PATH] = {};
        if (::GetEnvironmentVariableW(L"FRS_REFLECTION_CAPTURE", path, MAX_PATH)) {
            IDirect3DSurface9* source = nullptr;
            IDirect3DSurface9* copy = nullptr;
            if (SUCCEEDED(glReflection_->GetSurfaceLevel(0, &source)) &&
                SUCCEEDED(device->CreateOffscreenPlainSurface(kWidth, kHeight, D3DFMT_A8R8G8B8, D3DPOOL_SYSTEMMEM, &copy,
                                                              nullptr)) &&
                SUCCEEDED(device->GetRenderTargetData(source, copy))) {
                D3DLOCKED_RECT locked{};
                FILE* file = nullptr;
                if (SUCCEEDED(copy->LockRect(&locked, nullptr, D3DLOCK_READONLY))) {
                    if (::_wfopen_s(&file, path, L"wb") == 0 && file) {
                        BITMAPFILEHEADER header{};
                        BITMAPINFOHEADER info{};
                        header.bfType = 0x4D42;
                        header.bfOffBits = sizeof(header) + sizeof(info);
                        header.bfSize = header.bfOffBits + kWidth * kHeight * 4;
                        info.biSize = sizeof(info);
                        info.biWidth = (LONG)kWidth;
                        info.biHeight = -(LONG)kHeight;   // top-down rows
                        info.biPlanes = 1;
                        info.biBitCount = 32;
                        std::fwrite(&header, sizeof header, 1, file);
                        std::fwrite(&info, sizeof info, 1, file);
                        for (UINT row = 0; row < kHeight; ++row)
                            std::fwrite((const u8*)locked.pBits + (size_t)row * locked.Pitch, 4, kWidth, file);
                        std::fclose(file);
                    }
                    copy->UnlockRect();
                }
            }
            if (copy) copy->Release();
            if (source) source->Release();
        }
    }

    device->SetRenderTarget(0, oldTarget);
    device->SetDepthStencilSurface(oldDepth);
    device->SetViewport(&oldViewport);
    if (oldTarget) oldTarget->Release();
    if (oldDepth) oldDepth->Release();
    glReflectionValid_ = true;
}

// sub_5C7E40: two passes of FILTER p3 (sub_5C5F70 with four taps), the reflection into the second target
// along x (table 0x7FF1CC) and back along y (table 0x7FF19C). Taps {weight, offset in texels}: (1/3, -1),
// (1/3, +1), (1/6, -3), (1/6, +3) -- no centre tap.
void WorldRenderer::BlurReflection(IDirect3DDevice9* device) {
    constexpr float kWidth = 320.0f, kHeight = 240.0f;
    struct Vertex { float x, y, z, w; float uv[4][2]; };
    const float weights[4][4] = { { 1 / 3.0f, 1 / 3.0f, 1 / 3.0f, 1 / 3.0f },
                                  { 1 / 3.0f, 1 / 3.0f, 1 / 3.0f, 1 / 3.0f },
                                  { 1 / 6.0f, 1 / 6.0f, 1 / 6.0f, 1 / 6.0f },
                                  { 1 / 6.0f, 1 / 6.0f, 1 / 6.0f, 1 / 6.0f } };
    const float taps[4] = { -1.0f, 1.0f, -3.0f, 3.0f };

    device->SetVertexShader(glFilterVs_);
    device->SetPixelShader(glFilterPs_);
    device->SetFVF(D3DFVF_XYZW | D3DFVF_TEX4);
    device->SetPixelShaderConstantF(0, &weights[0][0], 4);
    device->SetRenderState(D3DRS_ZENABLE, FALSE);
    device->SetRenderState(D3DRS_ZWRITEENABLE, FALSE);
    device->SetRenderState(D3DRS_CULLMODE, D3DCULL_NONE);
    device->SetRenderState(D3DRS_ALPHATESTENABLE, FALSE);
    device->SetRenderState(D3DRS_ALPHABLENDENABLE, FALSE);
    device->SetRenderState(D3DRS_FOGENABLE, FALSE);
    device->SetRenderState(D3DRS_COLORWRITEENABLE, 15);
    for (DWORD s = 0; s < 4; ++s) {
        device->SetSamplerState(s, D3DSAMP_ADDRESSU, D3DTADDRESS_CLAMP);
        device->SetSamplerState(s, D3DSAMP_ADDRESSV, D3DTADDRESS_CLAMP);
        device->SetSamplerState(s, D3DSAMP_MINFILTER, D3DTEXF_LINEAR);
        device->SetSamplerState(s, D3DSAMP_MAGFILTER, D3DTEXF_LINEAR);
        device->SetSamplerState(s, D3DSAMP_MIPFILTER, D3DTEXF_NONE);
        device->SetTextureStageState(s, D3DTSS_TEXCOORDINDEX, s);
        device->SetTextureStageState(s, D3DTSS_TEXTURETRANSFORMFLAGS, D3DTTFF_DISABLE);
    }

    for (int pass = 0; pass < 2; ++pass) {
        IDirect3DTexture9* source = pass == 0 ? glReflection_ : glReflectionBlur_;
        IDirect3DTexture9* target = pass == 0 ? glReflectionBlur_ : glReflection_;
        IDirect3DSurface9* surface = nullptr;
        if (FAILED(target->GetSurfaceLevel(0, &surface)) || !surface) break;
        device->SetRenderTarget(0, surface);
        // Pixel centres on texel centres: the quad is moved half a pixel up and left.
        const float px = 1.0f / kWidth, py = 1.0f / kHeight;
        const float corners[4][4] = { { -1 - px, 1 + py, 0, 0 }, { 1 - px, 1 + py, 1, 0 },
                                      { -1 - px, -1 + py, 0, 1 }, { 1 - px, -1 + py, 1, 1 } };
        Vertex quad[4];
        for (int v = 0; v < 4; ++v) {
            quad[v].x = corners[v][0];
            quad[v].y = corners[v][1];
            quad[v].z = 0.5f;
            quad[v].w = 1.0f;
            for (int t = 0; t < 4; ++t) {
                quad[v].uv[t][0] = corners[v][2] + (pass == 0 ? taps[t] * px : 0.0f);
                quad[v].uv[t][1] = corners[v][3] + (pass == 1 ? taps[t] * py : 0.0f);
            }
        }
        for (DWORD s = 0; s < 4; ++s) device->SetTexture(s, source);
        device->DrawPrimitiveUP(D3DPT_TRIANGLESTRIP, 2, quad, sizeof(Vertex));
        for (DWORD s = 0; s < 4; ++s) device->SetTexture(s, nullptr);
        surface->Release();
    }
    device->SetVertexShader(nullptr);
    device->SetPixelShader(nullptr);
    device->SetRenderState(D3DRS_ZENABLE, TRUE);
    device->SetRenderState(D3DRS_ZWRITEENABLE, TRUE);
    device->SetFVF(kWorldVertexFvf);
    for (DWORD s = 1; s < 4; ++s) {
        device->SetTextureStageState(s, D3DTSS_TEXCOORDINDEX, s);
        device->SetTextureStageState(s, D3DTSS_COLOROP, D3DTOP_DISABLE);
        device->SetTextureStageState(s, D3DTSS_ALPHAOP, D3DTOP_DISABLE);
    }
    device->SetSamplerState(0, D3DSAMP_ADDRESSU, D3DTADDRESS_WRAP);
    device->SetSamplerState(0, D3DSAMP_ADDRESSV, D3DTADDRESS_WRAP);
    device->SetSamplerState(0, D3DSAMP_MIPFILTER, D3DTEXF_LINEAR);
}

// ---------------------------------------------------------------------------
// Extra passes over the world (drive/fx/HeadlightProjector.h)
// ---------------------------------------------------------------------------

void WorldRenderer::VisitSolidDraws(IDirect3DDevice9* device, const Aabb& bounds,
                                    const std::function<void(const SolidVisit&)>& draw) {
    if (!showScenery_ || !bounds.Valid() || mode_ == RenderMode::Wireframe) return;
    auto overlaps = [&](const Aabb& b) {
        return !b.Valid() || (b.min.x <= bounds.max.x && b.max.x >= bounds.min.x && b.min.y <= bounds.max.y &&
                              b.max.y >= bounds.min.y && b.min.z <= bounds.max.z && b.max.z >= bounds.min.z);
    };
    size_t boundSection = (size_t)-1;
    for (size_t i = 0; i < sections_.size(); ++i) {
        const SectionRender& section = sections_[i];
        if (section.alternate && !showAlternate_) continue;
        if (GameHidesSection(section.number)) continue;
        const std::vector<char>& hiddenHere = HiddenFlags(i);
        for (size_t k = 0; k < section.instances.size(); ++k) {
            const GpuInstance& instance = section.instances[k];
            if (hiddenHere[k] || !overlaps(instance.worldBounds)) continue;
            const GpuObject& object = GpuOf(instance.section)->Objects()[instance.object];
            if (InstanceHolidayHidden(section.number, instance.native, &object)) continue;
            if (world_ && world_->IsWorld() && object.source && object.source->IsShadow() &&
                !showProxies_)
                continue;
            if (BackdropHidden(object) || HiddenBuilding(object) || HiddenModel(object) || GameHidesModel(object))
                continue;
            for (const GpuDraw& d : object.draws) {
                const CachedTexture& texture = textures_.Get(device, AnimatedTexture(d.textureHash));
                if (texture.texture && texture.Translucent()) continue;
                if (boundSection != instance.section) {
                    const GpuSection& gpu = *GpuOf(instance.section);
                    device->SetStreamSource(0, gpu.VertexBuffer(), 0, sizeof(nfs::SolidVertex));
                    device->SetIndices(gpu.IndexBuffer());
                    boundSection = instance.section;
                }
                SolidVisit v;
                v.transform = &instance.transform;
                v.mirrored = instance.mirrored;
                v.baseVertex = (INT)object.baseVertex;
                v.draw = &d;
                v.texture = texture.texture;
                v.cutOut = texture.texture && (texture.blend == SurfaceBlend::CutOut || texture.punchThru);
                draw(v);
            }
        }
    }
}
