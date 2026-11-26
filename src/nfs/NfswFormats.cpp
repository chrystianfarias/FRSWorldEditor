#include "nfs/NfswFormats.h"

#include "nfs/ByteRead.h"

#include "core/Log.h"

#include <algorithm>
#include <cstring>
#include <map>
#include <unordered_map>
#include <unordered_set>

namespace nfs {

namespace {

// --- Solid object header, chunk 0x00134011 ---------------------------------
constexpr size_t kHdrVersion   = 0x0C;
constexpr size_t kHdrHash      = 0x10;
constexpr size_t kHdrBoundsMin = 0x20;
constexpr size_t kHdrBoundsMax = 0x30;
constexpr size_t kHdrTransform = 0x40;
constexpr size_t kHdrName      = 0xA0;

// --- Material, chunk 0x00134B02 --------------------------------------------
constexpr size_t kMaterialEntrySize = 116;
constexpr size_t kMatEffect         = 0x08;
constexpr size_t kMatBoundsMin      = 0x10;
constexpr size_t kMatBoundsMax      = 0x1C;
// The five map ids are contiguous: diffuse, normal, height, specular,
// opacity. Reading +0x2C would take opacity, which is usually the same slot
// but not always; the ones that differ then draw as a flat stand-in.
constexpr size_t kMatMapIds         = 0x28;
constexpr size_t kMatNumVerts       = 0x40;
constexpr size_t kMatNumIndices     = 0x44;
constexpr size_t kMatNumTris        = 0x48;
constexpr size_t kMatIndexOffset    = 0x4C;

// Deinstanced world geometry: 0x134C05 maps vertex ranges to 0x134C06 matrices.
constexpr u32 kMeshMorphList    = 0x00134C05;
constexpr u32 kMeshMorphMatrices = 0x00134C06;

// --- Scenery, chunk 0x80034100 ---------------------------------------------
constexpr size_t kInfoEntrySize = 72;
constexpr size_t kInstEntrySize = 96;
constexpr size_t kInstBoundsMin = 0x00;
constexpr size_t kInstFlags     = 0x0C;
constexpr size_t kInstBoundsMax = 0x10;
constexpr size_t kInstTranslation = 0x20;
constexpr size_t kInstRotation    = 0x2C;  // 3x3 floats, row major
constexpr size_t kInstGuid        = 0x50;
constexpr size_t kInstInfoIndex   = 0x54;

Vec3 ReadShort4N(ByteSpan span, size_t offset) {
    const float x = (float)span.read<i16>(offset) / 32768.0f;
    const float y = (float)span.read<i16>(offset + 2) / 32768.0f;
    const float z = (float)span.read<i16>(offset + 4) / 32768.0f;
    const Vec3 n{ x, y, z };
    return Length(n) > 1e-6f ? n : Vec3{ 0.0f, 0.0f, 1.0f };
}

u32 StrideForEffect(u32 effect) {
    switch (effect) {
        case 0:  // WorldShader
        case 1:  // WorldZBiasShader
        case 5:  // WorldZBiasPrelitShader
        case 10: // GLASS_REFLECT
        case 12: // Tree
        case 15: // WATER
            return 36;
        case 2:  // WorldNormalMap
        case 3:  // WorldRoadShader
        case 7:  // WorldFEShader
        case 11: // GLASS_REFLECTNM
            return 44;
        case 4:  // WorldPrelitShader
            return 28;
        case 8:  // CarShader
        case 9:  // CARNORMALMAP
            return 32;
        default:
            return 0;
    }
}

void DecodeVertex(ByteSpan data, u32 effect, SolidVertex& out) {
    out.position = ReadVec3(data, 0);
    switch (effect) {
        // WorldShader, WorldZBiasShader, WorldNormalMap, WorldRoadShader,
        // WorldFEShader, GLASS_REFLECT/NM, Tree and WATER share this prefix;
        // the normal-mapped variants only append a tangent the editor does not
        // read yet. Leaving any of these to the default turns the whole mesh
        // into a single white texel.
        case 0:
        case 1:
        case 2:
        case 3:
        case 7:
        case 10:
        case 11:
        case 12:
        case 15:
            out.normal = ReadShort4N(data, 12);
            out.uv     = { data.read<float>(20), data.read<float>(24) };
            out.color  = data.read<u32>(28);
            break;
        case 4:
            out.normal = { 0.0f, 0.0f, 1.0f };
            out.color  = data.read<u32>(12);
            out.uv     = { data.read<float>(20), data.read<float>(24) };
            break;
        case 5:
            out.normal = ReadShort4N(data, 12);
            out.color  = data.read<u32>(20);
            out.uv     = { data.read<float>(28), data.read<float>(32) };
            break;
        case 8:
        case 9:
            out.position = ReadShort4N(data, 0) * 8.0f;
            out.uv       = { (float)data.read<i16>(8) / 4096.0f,
                             (float)data.read<i16>(10) / 4096.0f - 1.0f };
            out.color    = data.read<u32>(12);
            out.normal   = ReadShort4N(data, 16);
            break;
        default:
            out.normal = { 0.0f, 0.0f, 1.0f };
            out.color  = 0xFFFFFFFFu;
            out.uv     = { 0.0f, 0.0f };
            break;
    }
}

struct MorphEntry {
    u32 first = 0;
    u32 last = 0;
    u32 matrix = 0;
};

struct MaterialIn {
    u32 effect = 0;
    u32 numVerts = 0;
    u32 numIndices = 0;
    u32 numTris = 0;
    u32 indexOffset = 0;
    std::array<u8, 5> maps{};
    Aabb bounds;
};

struct MeshSource {
    struct Buffer {
        ByteSpan bytes;
        u32 effect;
        u32 stride;
        u32 count;
        size_t first;
    };

    ByteSpan header;
    std::vector<ByteSpan> materials;
    std::vector<Buffer> buffers;
    std::vector<Mat4> morphs;
    bool complete = true;
};

bool ParseObject(const Chunk& objectChunk, SolidObject& out, MeshSource* source = nullptr) {
    ChunkReader reader = ChunkReader::Children(objectChunk);
    Chunk child;

    bool haveHeader = false;
    std::vector<ByteSpan> vertexBuffers;
    std::vector<u16> indices;
    std::vector<MaterialIn> materials;
    std::vector<std::vector<MorphEntry>> morphLists;
    std::vector<Mat4> morphMatrices;

    while (reader.Next(child)) {
        switch (child.id) {
            case kChunkSolidObjectHeader: {
                const ByteSpan data = child.data;
                if (source) source->header = data;
                out.version = data.read<u32>(kHdrVersion);
                out.hash    = data.read<u32>(kHdrHash);
                out.bounds.min = ReadVec3(data, kHdrBoundsMin);
                out.bounds.max = ReadVec3(data, kHdrBoundsMax);
                out.transform  = ReadMat4(data, kHdrTransform);
                out.name       = ReadName(data, kHdrName,
                                          data.size > kHdrName ? data.size - kHdrName : 0);
                out.kind       = ClassifyObject(out.name, true);
                haveHeader = true;
                break;
            }
            case kChunkSolidObjectTexRefs: {
                const size_t count = child.data.size / 8;
                out.textureHashes.reserve(out.textureHashes.size() + count);
                for (size_t i = 0; i < count; ++i)
                    out.textureHashes.push_back(child.data.read<u32>(i * 8));
                break;
            }
            case kChunkMeshContainer: {
                ChunkReader mesh = ChunkReader::Children(child);
                mesh.SetAlignment(16);   // NFS World vertex buffers are 16-byte aligned
                Chunk part;
                while (mesh.Next(part)) {
                    switch (part.id) {
                        case kChunkMeshVertices:
                            vertexBuffers.push_back(part.data);
                            break;
                        case kChunkMeshIndices: {
                            const size_t count = part.data.size / 2;
                            indices.reserve(indices.size() + count);
                            for (size_t i = 0; i < count; ++i)
                                indices.push_back(part.data.read<u16>(i * 2));
                            break;
                        }
                        case kMeshMorphList: {
                            const size_t count = part.data.size / 16;
                            std::vector<MorphEntry> list;
                            list.reserve(count);
                            for (size_t i = 0; i < count; ++i) {
                                MorphEntry entry;
                                entry.first  = part.data.read<u32>(i * 16);
                                entry.last   = part.data.read<u32>(i * 16 + 4);
                                entry.matrix = part.data.read<u32>(i * 16 + 8);
                                list.push_back(entry);
                            }
                            morphLists.push_back(std::move(list));
                            break;
                        }
                        case kMeshMorphMatrices: {
                            const size_t count = part.data.size / 64;
                            morphMatrices.reserve(morphMatrices.size() + count);
                            for (size_t i = 0; i < count; ++i)
                                morphMatrices.push_back(ReadMat4(part.data, i * 64));
                            break;
                        }
                        case kChunkMeshMaterials: {
                            const size_t count = part.data.size / kMaterialEntrySize;
                            materials.reserve(materials.size() + count);
                            for (size_t i = 0; i < count; ++i) {
                                const ByteSpan entry =
                                    part.data.sub(i * kMaterialEntrySize, kMaterialEntrySize);
                                MaterialIn material;
                                material.effect      = entry.read<u32>(kMatEffect);
                                material.numVerts    = entry.read<u32>(kMatNumVerts);
                                material.numIndices  = entry.read<u32>(kMatNumIndices);
                                material.numTris     = entry.read<u32>(kMatNumTris);
                                material.indexOffset = entry.read<u32>(kMatIndexOffset);
                                for (size_t k = 0; k < material.maps.size(); ++k)
                                    material.maps[k] = entry.read<u8>(kMatMapIds + k);
                                material.bounds.min  = ReadVec3(entry, kMatBoundsMin);
                                material.bounds.max  = ReadVec3(entry, kMatBoundsMax);
                                if (material.numIndices == 0 && material.numTris)
                                    material.numIndices = material.numTris * 3;
                                materials.push_back(material);
                                if (source) source->materials.push_back(entry);
                            }
                            break;
                        }
                        default:
                            break;
                    }
                }
                break;
            }
            default:
                break;
        }
    }

    if (!haveHeader || materials.empty() || vertexBuffers.empty() || indices.empty())
        return false;

    // Consecutive materials with the same effect share one vertex buffer; the
    // game's loader uses the same run rule. Indices are local to that run.
    struct Run {
        u32 effect = 0;
        u32 firstMaterial = 0;
        u32 materialCount = 0;
        u32 totalVerts = 0;
    };
    std::vector<Run> runs;
    for (u32 i = 0; i < (u32)materials.size(); ++i) {
        if (runs.empty() || materials[i].effect != runs.back().effect) {
            Run run;
            run.effect = materials[i].effect;
            run.firstMaterial = i;
            runs.push_back(run);
        }
        runs.back().materialCount++;
        runs.back().totalVerts += materials[i].numVerts;
    }

    if (runs.size() != vertexBuffers.size()) {
        if (source) source->complete = false;
        LOG_WARN("object '%s': %zu effect runs but %zu vertex buffers",
                 out.name.c_str(), runs.size(), vertexBuffers.size());
    }

    std::vector<u32> runBases;
    for (size_t r = 0; r < runs.size(); ++r) {
        const Run& run = runs[r];
        const u32 stride = StrideForEffect(run.effect);
        if (stride == 0 || r >= vertexBuffers.size()) {
            if (source) source->complete = false;
            LOG_WARN("object '%s': unsupported effect %u or missing vertex buffer",
                     out.name.c_str(), run.effect);
            continue;
        }

        const ByteSpan vb = vertexBuffers[r];
        const size_t expected = (size_t)run.totalVerts * stride;
        if (vb.size < expected) {
            if (source) source->complete = false;
            LOG_WARN("object '%s': effect %u buffer %zu bytes, expected %zu",
                     out.name.c_str(), run.effect, vb.size, expected);
            continue;
        }

        const u32 runBase = (u32)out.vertices.size();
        if (source) source->buffers.push_back({ vb, run.effect, stride, run.totalVerts, runBase });
        runBases.push_back(runBase);
        out.vertices.resize(out.vertices.size() + run.totalVerts);
        for (u32 i = 0; i < run.totalVerts; ++i)
            DecodeVertex(vb.sub((size_t)i * stride, stride), run.effect,
                         out.vertices[(size_t)runBase + i]);

        for (u32 m = 0; m < run.materialCount; ++m) {
            const MaterialIn& raw = materials[(size_t)run.firstMaterial + m];
            SolidMaterial material;
            material.textureIndex = raw.maps[0];
            material.nfswMapIds = raw.maps;
            material.nfswRoadSurface = raw.effect == 3;   // WorldRoadShader
            material.bounds       = raw.bounds;
            material.firstIndex   = (u32)out.indices.size();

            const u32 end = std::min(raw.indexOffset + raw.numIndices, (u32)indices.size());
            for (u32 i = raw.indexOffset; i + 2 < end; i += 3) {
                const u16 a = indices[i], b = indices[i + 1], c = indices[i + 2];
                if (a >= run.totalVerts || b >= run.totalVerts || c >= run.totalVerts) continue;
                out.indices.push_back((u16)(runBase + a));
                out.indices.push_back((u16)(runBase + b));
                out.indices.push_back((u16)(runBase + c));
            }
            material.indexCount = (u32)out.indices.size() - material.firstIndex;
            if (source && material.indexCount != raw.numIndices) source->complete = false;
            out.materials.push_back(material);
        }
    }

    // DEINST chunks are authored in local space and placed by their morph
    // matrices. Apply the ranges after the runs are widened; their bounds then
    // match the object header's world bounds.
    if (source && !morphLists.empty()) source->morphs.assign(out.vertices.size(), Mat4::Identity());
    for (size_t r = 0; r < morphLists.size() && r < runBases.size(); ++r) {
        for (const MorphEntry& entry : morphLists[r]) {
            if (entry.matrix >= morphMatrices.size()) continue;
            const size_t first = (size_t)runBases[r] + entry.first;
            const size_t last  = (size_t)runBases[r] + entry.last;
            if (first >= out.vertices.size()) continue;
            const size_t end = std::min(last, out.vertices.size() - 1);
            for (size_t i = first; i <= end; ++i) {
                const Mat4& morph = morphMatrices[entry.matrix];
                out.vertices[i].position = morph.TransformPoint(out.vertices[i].position);
                if (source) source->morphs[i] = source->morphs[i] * morph;
            }
        }
    }

    out.hasNormals   = true;
    out.vertexStride = sizeof(SolidVertex);
    return !out.vertices.empty() && !out.indices.empty();
}

bool PatchMeshObject(std::vector<u8>& section, const Chunk& object, const NfswMeshUpdate& update,
                     std::string& error) {
    MeshSource source;
    SolidObject original;
    if (!ParseObject(object, original, &source) || !source.complete ||
        source.header.size < kHdrName || source.materials.size() != original.materials.size() ||
        original.name != update.name || original.indices != update.indices ||
        original.vertices.size() != update.positions.size()) {
        error = update.name + ": the stored mesh does not match the exported vertex layout";
        return false;
    }

    const auto put = [&](ByteSpan data, size_t offset, const Vec3& v) {
        const size_t at = (size_t)(data.data - section.data()) + offset;
        std::memcpy(section.data() + at, &v, sizeof(v));
    };
    Aabb bounds = original.bounds;
    for (const MeshSource::Buffer& buffer : source.buffers) {
        for (size_t i = 0; i < buffer.count; ++i) {
            const size_t vertex = buffer.first + i;
            const Vec3& point = update.positions[vertex];
            if (Length(point - original.vertices[vertex].position) <= 0.001f) continue;
            if (buffer.effect == 8 || buffer.effect == 9) {
                error = update.name + ": packed car vertices cannot use this mesh writeback";
                return false;
            }
            const Vec3 local = source.morphs.empty() ? point
                : InverseAffine(source.morphs[vertex]).TransformPoint(point);
            if (!std::isfinite(Length(local)) ||
                (!source.morphs.empty() && Length(source.morphs[vertex].TransformPoint(local) - point) > 0.001f)) {
                error = update.name + ": an edited vertex has an invalid position or morph transform";
                return false;
            }
            put(buffer.bytes, i * buffer.stride, local);
            bounds.Add(point);
        }
    }
    if (bounds.Valid()) {
        put(source.header, kHdrBoundsMin, bounds.min);
        put(source.header, kHdrBoundsMax, bounds.max);
    }
    for (size_t m = 0; m < original.materials.size(); ++m) {
        const auto& material = original.materials[m];
        Aabb box = material.bounds;
        for (size_t i = material.firstIndex; i < (size_t)material.firstIndex + material.indexCount; ++i) {
            const size_t vertex = original.indices[i];
            if (Length(update.positions[vertex] - original.vertices[vertex].position) > 0.001f)
                box.Add(update.positions[vertex]);
        }
        if (box.Valid()) {
            put(source.materials[m], kMatBoundsMin, box.min);
            put(source.materials[m], kMatBoundsMax, box.max);
        }
    }
    return true;
}

// --- Scenery rebuild ---------------------------------------------------------

struct SceneryParts {
    ByteSpan header;
    ByteSpan tree;       // 0x34105, 48 bytes per node
    ByteSpan infos;
    ByteSpan instances;
    ByteSpan bindings;   // 0x3410D, 32 bytes per bound instance
    ByteSpan precull;    // 0x3410E, 32 bytes per precull record
    ByteSpan cull;       // 0x3410F, 32 bytes per precull record
    ByteSpan vectorized; // 0x34C03, 64 x 20-byte cull ranges
    ByteSpan overrides;  // 0x34106
};

bool SceneryChunkNumber(const Chunk& scenery, u32& number) {
    ChunkReader children = ChunkReader::Children(scenery);
    Chunk child;
    while (children.Next(child)) {
        if (child.id != kChunkSceneryHeader || child.data.size < 0x10) continue;
        number = child.data.read<u32>(0x0C);
        return true;
    }
    return false;
}

bool ParseSceneryParts(const Chunk& scenery, SceneryParts& out) {
    ChunkReader children = ChunkReader::Children(scenery);
    Chunk child;
    bool haveHeader = false;
    while (children.Next(child)) {
        switch (child.id) {
            case kChunkSceneryHeader:                 out.header = child.data; haveHeader = true; break;
            case kChunkSceneryInfos:                  out.infos = child.data; break;
            case kChunkSceneryInstances:              out.instances = child.data; break;
            case kChunkSceneryCullTree:               out.tree = child.data; break;
            case kChunkSceneryOverrides:              out.overrides = child.data; break;
            // The runtime takes this array straight after the chunk header.
            case kChunkSceneryInstanceBindings:       out.bindings = child.raw; break;
            case kChunkSceneryPrecullBoxes:           out.precull = child.data; break;
            case kChunkSceneryPrecullNodes:           out.cull = child.data; break;
            case kChunkSceneryVectorized:             out.vectorized = child.data; break;
            default: break;
        }
    }
    return haveHeader;
}

bool FindSceneryChunk(ByteSpan section, u32 number, Chunk& out) {
    ChunkReader top(section, 0);
    Chunk chunk;
    while (top.Next(chunk)) {
        if (chunk.id != kChunkTrackScenery) continue;
        u32 found = 0;
        if (!SceneryChunkNumber(chunk, found) || found != number) continue;
        out = chunk;
        return true;
    }
    return false;
}

void PutF(std::vector<u8>& out, size_t at, float value) {
    std::memcpy(out.data() + at, &value, sizeof(value));
}

void PutU32(std::vector<u8>& out, size_t at, u32 value) {
    std::memcpy(out.data() + at, &value, sizeof(value));
}

void WriteInstanceTransform(std::vector<u8>& instance, size_t at, const Mat4& transform,
                            const Aabb& bounds) {
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col)
            PutF(instance, at + kInstRotation + (size_t)(row * 3 + col) * 4,
                 transform.m[row][col]);
    PutF(instance, at + kInstTranslation + 0, transform.m[3][0]);
    PutF(instance, at + kInstTranslation + 4, transform.m[3][1]);
    PutF(instance, at + kInstTranslation + 8, transform.m[3][2]);
    PutF(instance, at + kInstBoundsMin + 0, bounds.min.x);
    PutF(instance, at + kInstBoundsMin + 4, bounds.min.y);
    PutF(instance, at + kInstBoundsMin + 8, bounds.min.z);
    PutF(instance, at + kInstBoundsMax + 0, bounds.max.x);
    PutF(instance, at + kInstBoundsMax + 4, bounds.max.y);
    PutF(instance, at + kInstBoundsMax + 8, bounds.max.z);
}

void WritePrecullBounds(std::vector<u8>& precull, size_t at, const Aabb& bounds) {
    const float radius = Length((bounds.max - bounds.min) * 0.5f);
    PutF(precull, at + 0x00, bounds.min.x);
    PutF(precull, at + 0x04, bounds.min.y);
    PutF(precull, at + 0x08, bounds.min.z);
    PutF(precull, at + 0x0C, radius);
    PutF(precull, at + 0x10, bounds.max.x);
    PutF(precull, at + 0x14, bounds.max.y);
    PutF(precull, at + 0x18, bounds.max.z);
    PutF(precull, at + 0x1C, radius);
}

bool NonZeroRecord(const u8* entry, size_t size) {
    for (size_t i = 0; i < size; ++i)
        if (entry[i]) return true;
    return false;
}
constexpr size_t kCullNodeSize = 48;
constexpr size_t kCullMaxChildren = 5;

u16 CullChildCount(const u8* node) {
    u16 count = 0;
    std::memcpy(&count, node + 0x0C, sizeof(count));
    return (u16)std::min<size_t>(count, kCullMaxChildren);
}

void CullSetChildCount(u8* node, u16 count) {
    std::memcpy(node + 0x0C, &count, sizeof(count));
}

void CullExpandNode(u8* node, const Aabb& bounds) {
    if (!bounds.Valid()) return;
    float value = 0.0f;
    for (size_t i = 0; i < 3; ++i) {
        std::memcpy(&value, node + i * 4, sizeof(value));
        value = std::min(value, (&bounds.min.x)[i]);
        std::memcpy(node + i * 4, &value, sizeof(value));
        std::memcpy(&value, node + 0x10 + i * 4, sizeof(value));
        value = std::max(value, (&bounds.max.x)[i]);
        std::memcpy(node + 0x10 + i * 4, &value, sizeof(value));
    }
}

// Removes records from the 0x34105 tree and shifts the surviving instance
// indices, the same way the game would after a delete.
bool CullRemoveInstances(std::vector<u8>& tree, const std::vector<u32>& removals,
                         size_t oldCount, std::string& error) {
    if (tree.empty()) return true;
    if (tree.size() % kCullNodeSize != 0) {
        error = "0x34105 is not a whole number of 48-byte nodes";
        return false;
    }
    std::vector<u32> map(oldCount, ~0u);
    size_t next = 0;
    for (u32 i = 0; i < oldCount; ++i) {
        const bool removed = std::find(removals.begin(), removals.end(), i) != removals.end();
        if (!removed) map[i] = (u32)next++;
    }
    for (size_t nodeAt = 0; nodeAt < tree.size(); nodeAt += kCullNodeSize) {
        u8* node = tree.data() + nodeAt;
        const u16 count = CullChildCount(node);
        std::vector<i16> kept;
        for (u16 i = 0; i < count; ++i) {
            i16 child = 0;
            std::memcpy(&child, node + 0x1C + i * 2, sizeof(child));
            if (child < 0) {
                kept.push_back(child);
                continue;
            }
            if ((u32)child >= oldCount) {
                error = "0x34105 names an instance outside the array";
                return false;
            }
            if (map[(u32)child] == ~0u) continue;
            if (map[(u32)child] > 0x7FFFu) {
                error = "0x34105 instance index no longer fits 16 bits";
                return false;
            }
            kept.push_back((i16)map[(u32)child]);
        }
        if (kept.size() > kCullMaxChildren) {
            error = "0x34105 node has too many children";
            return false;
        }
        for (size_t i = 0; i < kCullMaxChildren; ++i) {
            const i16 value = i < kept.size() ? kept[i] : 0;
            std::memcpy(node + 0x1C + i * 2, &value, sizeof(value));
        }
        CullSetChildCount(node, (u16)kept.size());
    }
    return true;
}

// Adds a new instance to the first node with a free child slot. Every node box
// is widened so the cull path cannot reject the instance by its old bounds.
bool CullAddInstance(std::vector<u8>& tree, u32 instance, const Aabb& bounds,
                     std::string& error) {
    if (tree.empty() || tree.size() % kCullNodeSize != 0) {
        error = "the section has no 0x34105 cull tree";
        return false;
    }
    if (instance > 0x7FFFu) {
        error = "too many instances for a 16-bit cull tree index";
        return false;
    }
    const size_t nodes = tree.size() / kCullNodeSize;
    for (size_t i = 0; i < nodes; ++i) CullExpandNode(tree.data() + i * kCullNodeSize, bounds);
    for (size_t i = 0; i < nodes; ++i) {
        u8* node = tree.data() + i * kCullNodeSize;
        const u16 count = CullChildCount(node);
        if (count >= kCullMaxChildren) continue;
        const i16 child = (i16)instance;
        std::memcpy(node + 0x1C + count * 2, &child, sizeof(child));
        CullSetChildCount(node, count + 1);
        return true;
    }
    // Every node is full: wrap the old root in a new root, which keeps the
    // game's root-at-index-0 convention. Old node numbers shift by one, so
    // every negative child reference becomes one more negative.
    std::vector<u8> grown(kCullNodeSize, 0);
    grown.insert(grown.end(), tree.begin(), tree.end());
    for (size_t nodeAt = kCullNodeSize; nodeAt < grown.size(); nodeAt += kCullNodeSize) {
        u8* node = grown.data() + nodeAt;
        const u16 count = CullChildCount(node);
        for (u16 i = 0; i < count; ++i) {
            i16 child = 0;
            std::memcpy(&child, node + 0x1C + i * 2, sizeof(child));
            if (child < 0) {
                child = (i16)(child - 1);
                std::memcpy(node + 0x1C + i * 2, &child, sizeof(child));
            }
        }
    }
    // The new root takes the old root's box, widened for the new instance.
    const u8* oldRoot = grown.data() + kCullNodeSize;
    std::memcpy(grown.data() + 0x00, oldRoot + 0x00, 12);
    std::memcpy(grown.data() + 0x10, oldRoot + 0x10, 12);
    CullExpandNode(grown.data(), bounds);
    u8* root = grown.data();
    const i16 oldRootChild = -1;
    const i16 instanceChild = (i16)instance;
    std::memcpy(root + 0x1C, &oldRootChild, sizeof(oldRootChild));
    std::memcpy(root + 0x1E, &instanceChild, sizeof(instanceChild));
    CullSetChildCount(root, 2);
    tree.swap(grown);
    return true;
}

// NFS World's 0x34C03 holds 64 records of five dwords. The renderer indexes a
// record by view type and submits [first, first + count) of the 0x3410E /
// 0x3410F arrays; the other fields are the same range again and a type tag.
// Appending 0x3410E/0x3410F entries at the end therefore hides them from the
// real draw path even though 0x34105 names them.
constexpr size_t kVectorEntrySize  = 20;
constexpr size_t kVectorEntryCount = 64;

struct VectorRange {
    u32 first = 0;
    u32 count = 0;
};

bool ReadVectorRange(ByteSpan vectorized, size_t type, VectorRange& out) {
    if ((type + 1) * kVectorEntrySize > vectorized.size) return false;
    out.count = vectorized.read<u32>(type * kVectorEntrySize + 4);
    out.first = vectorized.read<u32>(type * kVectorEntrySize + 8);
    return true;
}

// The source record's view type. Aliases (types 2/4/5/22..27 for type 1, and
// so on) share a range, so the first match is the canonical type.
u32 VectorTypeForEntry(ByteSpan vectorized, size_t entry) {
    const size_t records = std::min(kVectorEntryCount, vectorized.size / kVectorEntrySize);
    for (size_t type = 0; type < records; ++type) {
        VectorRange range;
        if (!ReadVectorRange(vectorized, type, range) || !range.count) continue;
        if (entry >= range.first && entry < (size_t)range.first + range.count)
            return (u32)type;
    }
    return 1;
}

// Removing a 0x3410E/0x3410F entry shifts every later range. Update the
// vectorized ranges before the arrays themselves are erased.
void VectorRemoveEntry(std::vector<u8>& vectorized, size_t entry) {
    const size_t records = std::min(kVectorEntryCount, vectorized.size() / kVectorEntrySize);
    std::map<std::pair<u32, u32>, std::pair<u32, u32>> updates;
    for (size_t type = 0; type < records; ++type) {
        VectorRange range;
        if (!ReadVectorRange({ vectorized.data(), vectorized.size() },
                             type, range) || !range.count)
            continue;
        if (entry < range.first) {
            updates[{ range.first, range.count }] = { range.first - 1, range.count };
        } else if (entry < (size_t)range.first + range.count) {
            updates[{ range.first, range.count }] = { range.first, range.count - 1 };
        }
    }
    for (size_t type = 0; type < records; ++type) {
        VectorRange range;
        if (!ReadVectorRange({ vectorized.data(), vectorized.size() },
                             type, range) || !range.count)
            continue;
        const auto found = updates.find({ range.first, range.count });
        if (found == updates.end()) continue;
        PutU32(vectorized, type * kVectorEntrySize + 4, found->second.second);
        PutU32(vectorized, type * kVectorEntrySize + 8, found->second.first);
        if (ByteSpan{ vectorized.data(), vectorized.size() }.read<u32>(type * kVectorEntrySize + 12) == range.first)
            PutU32(vectorized, type * kVectorEntrySize + 12, found->second.first);
    }
}

struct PendingCullEntry {
    u32                type = 1;
    std::vector<u8>    box;
    std::vector<u8>    node;
};

// Inserts copied cull records at the end of the target range that belongs to
// the source record's view type, then rewrites every range boundary.
bool InsertVectorizedCulls(std::vector<u8>& vectorized,
                           std::vector<u8>& precull, std::vector<u8>& cull,
                           const std::vector<PendingCullEntry>& pending,
                           std::string& error) {
    if (pending.empty()) return true;
    if (vectorized.size() < kVectorEntrySize) {
        error = "0x34C03 is empty";
        return false;
    }
    const size_t records = std::min(kVectorEntryCount, vectorized.size() / kVectorEntrySize);
    const size_t entries = cull.size() / 32;
    if (precull.size() / 32 != entries) {
        error = "0x3410E and 0x3410F do not hold the same number of records";
        return false;
    }

    // Group by the target range, not just the type: aliases share one range.
    std::map<std::pair<u32, u32>, std::vector<size_t>> groups;
    for (size_t i = 0; i < pending.size(); ++i) {
        VectorRange range;
        u32 type = pending[i].type;
        bool found = ReadVectorRange({ vectorized.data(), vectorized.size() }, type, range) &&
                     range.count != 0;
        if (!found) {
            type = 1;
            found = ReadVectorRange({ vectorized.data(), vectorized.size() }, type, range) &&
                    range.count != 0;
        }
        if (!found) {
            for (size_t candidate = 0; candidate < records; ++candidate) {
                if (ReadVectorRange({ vectorized.data(), vectorized.size() },
                                    candidate, range) && range.count) {
                    found = true;
                    type = (u32)candidate;
                    break;
                }
            }
        }
        if (!found) {
            error = "0x34C03 has no non-empty cull range";
            return false;
        }
        if ((u64)range.first + range.count > entries) {
            error = "0x34C03 range extends past 0x3410F";
            return false;
        }
        groups[{ range.first, range.count }].push_back(i);
    }

    // The ranges are contiguous and ordered; append each group after its last
    // entry while rebuilding the two arrays.
    std::map<u32, std::vector<size_t>> atEnd;
    for (const auto& group : groups) {
        const u32 end = group.first.first + group.first.second;
        atEnd[end].insert(atEnd[end].end(), group.second.begin(), group.second.end());
    }
    std::vector<u8> newPrecull, newCull;
    newPrecull.reserve(precull.size() + pending.size() * 32);
    newCull.reserve(cull.size() + pending.size() * 32);
    for (size_t i = 0; i < entries; ++i) {
        newPrecull.insert(newPrecull.end(), precull.begin() + (ptrdiff_t)(i * 32),
                          precull.begin() + (ptrdiff_t)((i + 1) * 32));
        newCull.insert(newCull.end(), cull.begin() + (ptrdiff_t)(i * 32),
                       cull.begin() + (ptrdiff_t)((i + 1) * 32));
        const auto found = atEnd.find((u32)(i + 1));
        if (found == atEnd.end()) continue;
        for (size_t index : found->second) {
            newPrecull.insert(newPrecull.end(), pending[index].box.begin(),
                              pending[index].box.end());
            newCull.insert(newCull.end(), pending[index].node.begin(),
                           pending[index].node.end());
        }
    }

    // The ranges are contiguous and ordered; appending inside one range moves
    // every later range, including ranges that received no entry. Shift every
    // record by the number of entries inserted before its first record.
    std::map<std::pair<u32, u32>, u32> inserted;
    for (const auto& group : groups)
        inserted[group.first] += (u32)group.second.size();

    for (size_t type = 0; type < records; ++type) {
        VectorRange range;
        if (!ReadVectorRange({ vectorized.data(), vectorized.size() },
                             type, range) || !range.count)
            continue;
        u32 shift = 0;
        for (const auto& entry : inserted) {
            const u64 end = (u64)entry.first.first + entry.first.second;
            if (end <= range.first) shift += entry.second;
        }
        u32 added = 0;
        const auto own = inserted.find({ range.first, range.count });
        if (own != inserted.end()) added = own->second;
        const u32 first = range.first + shift;
        PutU32(vectorized, type * kVectorEntrySize + 4, range.count + added);
        PutU32(vectorized, type * kVectorEntrySize + 8, first);
        if (ByteSpan{ vectorized.data(), vectorized.size() }.read<u32>(type * kVectorEntrySize + 12) == range.first)
            PutU32(vectorized, type * kVectorEntrySize + 12, first);
    }
    precull.swap(newPrecull);
    cull.swap(newCull);
    return true;
}
} // namespace

bool NfswParseSolidList(const Chunk& pack, SolidList& out) {
    ChunkReader reader = ChunkReader::Children(pack);
    Chunk child;
    while (reader.Next(child)) {
        if (child.id == kChunkSolidListHeader) {
            const Chunk info = FindChunk(child, kChunkSolidListPadding);
            if (info.data.size > 0x10)
                out.path = ReadName(info.data, 0x10, info.data.size - 0x10);
        } else if (child.id == kChunkSolidObject) {
            SolidObject object;
            if (ParseObject(child, object))
                out.objects.push_back(std::move(object));
        }
    }
    return !out.objects.empty();
}

bool NfswAppendRoadModels(std::vector<u8>& section, const std::vector<SolidObject>& models,
                          std::string& error) {
    if (models.empty()) return true;
    std::vector<u8> bytes;
    const auto begin = [&](u32 id, size_t alignment = 1) {
        const size_t at = bytes.size();
        bytes.resize(at + 8);
        PutU32(bytes, at, id);
        bytes.resize((bytes.size() + alignment - 1) & ~(alignment - 1), 0x11);
        return at;
    };
    const auto end = [&](size_t at) { PutU32(bytes, at + 4, (u32)(bytes.size() - at - 8)); };
    const auto add = [&](const void* data, size_t size) {
        const auto* first = static_cast<const u8*>(data);
        if (size) bytes.insert(bytes.end(), first, first + size);
    };
    const auto addName = [&](const std::string& name) {
        add(name.c_str(), name.size() + 1);
        bytes.resize((bytes.size() + 3) & ~size_t(3));
    };
    const auto putVector = [&](size_t at, const Vec3& p) { std::memcpy(bytes.data() + at, &p, sizeof(p)); };

    const size_t pack = begin(kChunkSolidListPack);
    const size_t list = begin(kChunkSolidListHeader);
    const size_t info = begin(kChunkSolidListPadding);
    bytes.resize(bytes.size() + 144);
    PutU32(bytes, info + 8 + 8, 0x1D);
    PutU32(bytes, info + 8 + 12, (u32)models.size());
    constexpr char source[] = "FRS_ROADS";
    std::memcpy(bytes.data() + info + 8 + 16, source, sizeof(source));
    end(info);
    const size_t hashes = begin(kChunkSolidListHashes);
    std::vector<u32> ids;
    for (const auto& model : models) ids.push_back(model.hash);
    std::sort(ids.begin(), ids.end());
    const u32 zero = 0;
    for (u32 id : ids) { add(&id, 4); add(&zero, 4); }
    end(hashes);
    end(begin(0x80134008));
    end(list);

    for (const auto& model : models) {
        if (model.vertices.empty() || model.vertices.size() >= UINT16_MAX || model.indices.empty() ||
            model.indices.size() % 3 || model.textureHashes.empty() || model.textureHashes.size() > 254 ||
            model.materials.empty() || !model.bounds.Valid()) {
            error = model.name + ": invalid road mesh or too many vertices or textures";
            return false;
        }
        const size_t object = begin(kChunkSolidObject);
        const size_t header = begin(kChunkSolidObjectHeader, 16), hdr = bytes.size();
        bytes.resize(hdr + kHdrName);
        PutU32(bytes, hdr + kHdrVersion, 0x16);
        PutU32(bytes, hdr + kHdrHash, model.hash);
        PutU32(bytes, hdr + 0x14, model.TriangleCount());
        bytes[hdr + 0x19] = (u8)model.textureHashes.size();
        putVector(hdr + kHdrBoundsMin, model.bounds.min);
        putVector(hdr + kHdrBoundsMax, model.bounds.max);
        const Mat4 identity = Mat4::Identity();
        std::memcpy(bytes.data() + hdr + kHdrTransform, &identity, sizeof(identity));
        addName(model.name);
        end(header);
        const size_t textures = begin(kChunkSolidObjectTexRefs, 8);
        for (u32 hash : model.textureHashes) { add(&hash, 4); add(&zero, 4); }
        end(textures);

        struct RoadVertexRun {
            bool road;
            u32 first, end;
        };
        std::vector<RoadVertexRun> runs;
        std::vector<u16> localIndices(model.indices.size());
        const size_t mesh = begin(kChunkMeshContainer);
        const size_t descriptor = begin(kChunkMeshDescriptor, 16), desc = bytes.size();
        bytes.resize(desc + 52);
        PutU32(bytes, desc + 0x08, 0x12);
        PutU32(bytes, desc + 0x0C, 0x4000);
        PutU32(bytes, desc + 0x10, (u32)model.materials.size());
        PutU32(bytes, desc + 0x2C, (u32)model.indices.size());
        end(descriptor);
        const size_t materials = begin(kChunkMeshMaterials, 16);
        u32 materialVerts = 0;
        for (size_t i = 0; i < model.materials.size(); ++i) {
            const auto& material = model.materials[i];
            if (material.textureIndex >= model.textureHashes.size() || !material.indexCount ||
                (u64)material.firstIndex + material.indexCount > model.indices.size()) {
                error = model.name + ": invalid road material";
                return false;
            }
            // Builder groups have their own consecutive vertex blocks.
            const u16* first = model.indices.data() + material.firstIndex;
            const auto range = std::minmax_element(first, first + material.indexCount);
            if (*range.first != materialVerts || *range.second >= model.vertices.size()) {
                error = model.name + ": road material vertices are not consecutive";
                return false;
            }
            const u32 count = *range.second + 1u - materialVerts;
            if (runs.empty() || runs.back().road != material.nfswRoadSurface)
                runs.push_back({ material.nfswRoadSurface, materialVerts, materialVerts });
            materialVerts += count;
            runs.back().end = materialVerts;
            for (u32 k = material.firstIndex; k < material.firstIndex + material.indexCount; ++k)
                localIndices[k] = (u16)(model.indices[k] - runs.back().first);
            const size_t mat = bytes.size();
            bytes.resize(mat + 116);
            PutU32(bytes, mat, 0x4000);
            PutU32(bytes, mat + 4, model.textureHashes[material.textureIndex]);
            PutU32(bytes, mat + kMatEffect, material.nfswRoadSurface ? 3 : 0);
            putVector(mat + 0x10, material.bounds.min);
            putVector(mat + 0x1C, material.bounds.max);
            for (size_t k = 0; k < 5; ++k)
                bytes[mat + kMatMapIds + k] = material.nfswRoadSurface && k > 0
                    ? material.nfswMapIds[k] : (u8)material.textureIndex;
            bytes[mat + 0x2D] = 0xFF;
            PutU32(bytes, mat + 0x40, count);
            PutU32(bytes, mat + 0x44, material.indexCount);
            PutU32(bytes, mat + 0x48, material.indexCount / 3);
            PutU32(bytes, mat + 0x4C, material.firstIndex);
        }
        if (materialVerts != model.vertices.size()) {
            error = model.name + ": the material ranges do not cover the vertex buffer";
            return false;
        }
        end(materials);
        PutU32(bytes, desc + 0x18, (u32)runs.size());
        const size_t indices = begin(kChunkMeshIndices, 16);
        add(localIndices.data(), localIndices.size() * sizeof(u16));
        bytes.resize((bytes.size() + 3) & ~size_t(3));
        end(indices);
        std::vector<Vec3> tangents(model.vertices.size()), bitangents(model.vertices.size());
        for (const auto& material : model.materials) {
            if (!material.nfswRoadSurface) continue;
            for (u32 k = material.firstIndex; k + 2 < material.firstIndex + material.indexCount; k += 3) {
                const auto& a = model.vertices[model.indices[k]];
                const auto& b = model.vertices[model.indices[k + 1]];
                const auto& c = model.vertices[model.indices[k + 2]];
                const Vec3 ab = b.position - a.position, ac = c.position - a.position;
                const float u1 = b.uv.x - a.uv.x, v1 = b.uv.y - a.uv.y;
                const float u2 = c.uv.x - a.uv.x, v2 = c.uv.y - a.uv.y;
                const float det = u1 * v2 - u2 * v1;
                if (std::abs(det) < 1e-8f) continue;
                for (u32 i = k; i < k + 3; ++i) {
                    tangents[model.indices[i]] += (ab * v2 - ac * v1) * (1.0f / det);
                    bitangents[model.indices[i]] += (ac * u1 - ab * u2) * (1.0f / det);
                }
            }
        }
        const auto snorm = [](float f) { return (i16)std::lround(std::clamp(f, -1.0f, 1.0f) * 32767.0f); };
        for (const auto& run : runs) {
            const size_t vertices = begin(kChunkMeshVertices, 16);
            for (u32 i = run.first; i < run.end; ++i) {
                const auto& vertex = model.vertices[i];
                add(&vertex.position, 12);
                const i16 n[]{ snorm(vertex.normal.x), snorm(vertex.normal.y), snorm(vertex.normal.z), 0 };
                add(n, 8);
                add(&vertex.uv, 8);
                add(&vertex.color, 4);
                add(&vertex.color, 4);
                if (run.road) {
                    const Vec3 normal = Normalize(vertex.normal);
                    const Vec3 vector = tangents[i] - normal * Dot(normal, tangents[i]);
                    const Vec3 tangent = Length(vector) > 1e-5f ? Normalize(vector)
                        : Normalize(Cross(std::abs(normal.z) > 0.9f ? Vec3{ 0, 1, 0 } : Vec3{ 0, 0, 1 }, normal));
                    const float sign = Dot(Cross(normal, tangent), bitangents[i]) < 0.0f ? -1.0f : 1.0f;
                    const i16 t[]{ snorm(tangent.x), snorm(tangent.y), snorm(tangent.z), snorm(sign) };
                    add(t, 8);
                }
            }
            end(vertices);
        }
        for (size_t i = 0; i < model.materials.size(); ++i) {
            const size_t name = begin(0x00134C02);
            addName("FRS_ROAD_" + std::to_string(i));
            end(name);
        }
        end(mesh);
        end(object);
    }
    end(pack);
    // A null chunk consumes the padding even when the original section has
    // ended at a two-byte index boundary.
    const size_t pad = (128 - (section.size() & 127)) & 127;
    const size_t extra = pad && pad < 8 ? pad + 128 : pad;
    if (extra) {
        const size_t start = section.size();
        section.resize(start + extra);
        PutU32(section, start + 4, (u32)extra - 8);
    }
    section.insert(section.end(), bytes.begin(), bytes.end());
    return true;
}

bool NfswPatchSolidMeshes(std::vector<u8>& section, const std::vector<const NfswMeshUpdate*>& updates,
                          std::string& error) {
    std::unordered_map<u32, const NfswMeshUpdate*> pending;
    for (const auto* update : updates) pending.emplace(update->hash, update);
    std::unordered_set<u32> written;
    ChunkReader reader({ section.data(), section.size() }, 0);
    Chunk pack;
    while (reader.Next(pack)) {
        if (pack.id != kChunkSolidListPack) continue;
        ChunkReader objects = ChunkReader::Children(pack);
        Chunk object;
        while (objects.Next(object)) {
            if (object.id != kChunkSolidObject) continue;
            const Chunk header = FindChunk(object, kChunkSolidObjectHeader);
            const auto found = pending.find(header.data.read<u32>(kHdrHash));
            if (header.id == 0 || found == pending.end()) continue;
            if (!PatchMeshObject(section, object, *found->second, error)) return false;
            written.insert(found->first);
        }
    }
    if (written.size() == pending.size()) return true;
    error = "the mesh object was not found in its World section";
    return false;
}

bool NfswParseScenery(const Chunk& sceneryChunk, Scenery& out,
                      std::vector<u32>* recordToInstance) {
    ByteSpan infoData, instanceData, overrideData;
    ChunkReader reader = ChunkReader::Children(sceneryChunk);
    Chunk child;
    while (reader.Next(child)) {
        if (child.id == kChunkSceneryInfos)           infoData     = child.data;
        else if (child.id == kChunkSceneryInstances)  instanceData = child.data;
        else if (child.id == kChunkSceneryOverrides) overrideData = child.data;
    }
    if (infoData.empty() || instanceData.empty()) return false;

    const u32 infoBase = (u32)out.infos.size();
    const size_t infoCount = infoData.size / kInfoEntrySize;
    out.infos.reserve(out.infos.size() + infoCount);
    for (size_t i = 0; i < infoCount; ++i) {
        const ByteSpan entry = infoData.sub(i * kInfoEntrySize, kInfoEntrySize);
        SceneryInfo info;
        info.name      = ReadName(entry, 0, 24);
        info.solidHash = entry.read<u32>(0x18);
        info.radius    = entry.read<float>(0x3C);
        info.raw.assign(entry.data, entry.data + entry.size);
        out.infos.push_back(std::move(info));
    }

    const size_t instanceCount = instanceData.size / kInstEntrySize;
    std::vector<u32> mapping(instanceCount, ~0u);
    out.instances.reserve(out.instances.size() + instanceCount);
    for (size_t i = 0; i < instanceCount; ++i) {
        const ByteSpan entry = instanceData.sub(i * kInstEntrySize, kInstEntrySize);
        SceneryInstance instance;
        instance.infoIndex = infoBase + entry.read<u16>(kInstInfoIndex);
        instance.flags     = entry.read<u32>(kInstFlags);
        instance.guid      = entry.read<u32>(kInstGuid);
        instance.record    = (u32)i;
        instance.worldBounds.min = ReadVec3(entry, kInstBoundsMin);
        instance.worldBounds.max = ReadVec3(entry, kInstBoundsMax);

        instance.transform = Mat4::Identity();
        for (int row = 0; row < 3; ++row)
            for (int col = 0; col < 3; ++col)
                instance.transform.m[row][col] =
                    entry.read<float>(kInstRotation + (size_t)(row * 3 + col) * 4);
        const Vec3 translation = ReadVec3(entry, kInstTranslation);
        instance.transform.m[3][0] = translation.x;
        instance.transform.m[3][1] = translation.y;
        instance.transform.m[3][2] = translation.z;

        if (instance.infoIndex >= out.infos.size()) continue;
        mapping[i] = (u32)out.instances.size();
        out.instances.push_back(instance);
    }

    // 0x34106: {u16 table entry, u16 instance index}. The high half indexes
    // this scenery chunk's records; map it through the records that survived
    // the info-index check so a skipped record does not shift later overrides.
    for (size_t at = 0; at + 4 <= overrideData.size; at += 4) {
        const u32 value = overrideData.read<u32>(at);
        const u32 record = value >> 16;
        if (record >= mapping.size() || mapping[record] == ~0u) continue;
        out.overrides.push_back({ (u16)(value & 0xFFFFu), mapping[record] });
    }
    if (recordToInstance) *recordToInstance = std::move(mapping);
    return !out.instances.empty();
}

Scenery NfswReadSectionScenery(ByteSpan bytes) {
    Scenery scenery;
    ChunkReader reader(bytes, 0);
    Chunk chunk;
    while (reader.Next(chunk)) {
        if (chunk.id != kChunkTrackScenery) continue;
        const u32 number = FindChunk(chunk, kChunkSceneryHeader).data.read<u32>(0x0C);
        const size_t first = scenery.instances.size();
        if (!NfswParseScenery(chunk, scenery)) continue;
        for (size_t i = first; i < scenery.instances.size(); ++i)
            scenery.instances[i].sceneryNumber = number;
    }
    return scenery;
}

std::vector<size_t> NfswChangedPlacements(const Scenery& stock, const Scenery& current) {
    std::multimap<std::pair<u32, u32>, const SceneryInstance*> originals;
    for (const SceneryInstance& instance : stock.instances)
        originals.emplace(std::make_pair(instance.sceneryNumber, instance.guid), &instance);
    std::vector<size_t> changed;
    for (size_t i = 0; i < current.instances.size(); ++i) {
        const SceneryInstance& instance = current.instances[i];
        if (instance.flags & 0x10) continue;  // event/holiday scenery owns its collision
        const u32 hash = current.ModelHash(instance);
        const auto [first, last] = originals.equal_range({ instance.sceneryNumber, instance.guid });
        if (std::none_of(first, last, [&](const auto& record) {
                return stock.ModelHash(*record.second) == hash &&
                       std::memcmp(&instance.transform, &record.second->transform, sizeof(Mat4)) == 0;
            }))
            changed.push_back(i);
    }
    return changed;
}

bool NfswReadSceneryChunk(ByteSpan wholeChunk, u64 fileOffset, NfswSceneryChunkInfo& out) {
    ChunkReader reader(wholeChunk, fileOffset);
    Chunk chunk;
    if (!reader.Next(chunk) || chunk.id != kChunkTrackScenery) return false;

    ChunkReader children = ChunkReader::Children(chunk);
    Chunk child;
    bool haveHeader = false;
    while (children.Next(child)) {
        if (child.id == kChunkSceneryHeader) {
            // NFS World's 0x34101 header puts the section number at +0x0C.
            out.number = child.data.read<u32>(0x0C);
            haveHeader = true;
        } else if (child.id == kChunkSceneryInfos) {
            out.infoCount = child.data.size / kInfoEntrySize;
        } else if (child.id == kChunkSceneryInstances) {
            out.instanceCount = child.data.size / kInstEntrySize;
        }
    }
    return haveHeader;
}

bool NfswPatchScenery(ByteSpan section, const std::vector<NfswSceneryPatch>& patches,
                      std::vector<u8>& out, std::string& error) {
    if (section.empty()) {
        error = "empty section";
        return false;
    }
    out.assign(section.data, section.data + section.size);

    for (const NfswSceneryPatch& patch : patches) {
        bool found = false;
        ChunkReader top(section, 0);
        Chunk chunk;
        while (!found && top.Next(chunk)) {
            if (chunk.id != kChunkTrackScenery) continue;

            Chunk header;
            ByteSpan instances, boxes, nodes;
            u64 instanceAt = 0;
            u64 boxAt = 0;
            Chunk cullTree;
            ChunkReader children = ChunkReader::Children(chunk);
            Chunk child;
            while (children.Next(child)) {
                if (child.id == kChunkSceneryHeader) {
                    header = child;
                } else if (child.id == kChunkSceneryInstances) {
                    instances = child.data;
                    instanceAt = child.dataOffset;
                } else if (child.id == kChunkSceneryPrecullBoxes) {
                    boxes = child.data;
                    boxAt = child.dataOffset;
                } else if (child.id == kChunkSceneryPrecullNodes) {
                    nodes = child.data;
                } else if (child.id == kChunkSceneryCullTree) {
                    cullTree = child;
                }
            }
            if (header.id == 0 || header.data.size < 0x10 ||
                header.data.read<u32>(0x0C) != patch.sceneryNumber)
                continue;

            if (instances.empty() || patch.record >= instances.size / kInstEntrySize) {
                error = "scenery record outside its instance chunk";
                return false;
            }
            const u64 recordAt = instanceAt + (u64)patch.record * kInstEntrySize;
            if (recordAt + kInstEntrySize > out.size()) {
                error = "scenery record outside the section";
                return false;
            }

            const auto put = [&out](size_t at, float value) {
                std::memcpy(out.data() + at, &value, sizeof(value));
            };
            for (int row = 0; row < 3; ++row)
                for (int col = 0; col < 3; ++col)
                    put((size_t)recordAt + kInstRotation + (size_t)(row * 3 + col) * 4,
                        patch.transform.m[row][col]);
            put((size_t)recordAt + kInstTranslation + 0, patch.transform.m[3][0]);
            put((size_t)recordAt + kInstTranslation + 4, patch.transform.m[3][1]);
            put((size_t)recordAt + kInstTranslation + 8, patch.transform.m[3][2]);

            if (patch.bounds.Valid()) {
                if (patch.expandCullTree) {
                    for (size_t at = 0; at + kCullNodeSize <= cullTree.data.size; at += kCullNodeSize)
                        CullExpandNode(out.data() + cullTree.dataOffset + at, patch.bounds);
                }
                put((size_t)recordAt + kInstBoundsMin + 0, patch.bounds.min.x);
                put((size_t)recordAt + kInstBoundsMin + 4, patch.bounds.min.y);
                put((size_t)recordAt + kInstBoundsMin + 8, patch.bounds.min.z);
                put((size_t)recordAt + kInstBoundsMax + 0, patch.bounds.max.x);
                put((size_t)recordAt + kInstBoundsMax + 4, patch.bounds.max.y);
                put((size_t)recordAt + kInstBoundsMax + 8, patch.bounds.max.z);

                const float radius = Length((patch.bounds.max - patch.bounds.min) * 0.5f);
                const auto patchBox = [&](size_t index) {
                    if (index >= boxes.size / 32) return;
                    const size_t at = (size_t)boxAt + index * 32;
                    put(at + 0x00, patch.bounds.min.x);
                    put(at + 0x04, patch.bounds.min.y);
                    put(at + 0x08, patch.bounds.min.z);
                    put(at + 0x0C, radius);
                    put(at + 0x10, patch.bounds.max.x);
                    put(at + 0x14, patch.bounds.max.y);
                    put(at + 0x18, patch.bounds.max.z);
                    put(at + 0x1C, radius);
                };

                // The boxes are grouped by model LOD, so the box at the instance
                // index is not necessarily this instance's. Follow 0x3410F, which
                // stores the instance index at +0x10.
                bool matched = false;
                const size_t records = std::min(boxes.size / 32, nodes.size / 32);
                for (size_t i = 0; i < records; ++i) {
                    const u8* entry = nodes.data + i * 32;
                    bool nonzero = false;
                    for (size_t b = 0; b < 32 && !nonzero; ++b) nonzero = entry[b] != 0;
                    if (!nonzero) continue;
                    u32 instance = 0;
                    std::memcpy(&instance, entry + 0x10, sizeof(instance));
                    if (instance != patch.record) continue;
                    patchBox(i);
                    matched = true;
                }
                // Older sections can lack 0x3410F, and then the record index is
                // the only mapping; when the table is present but names no box for
                // this record, leave the unrelated LOD box alone.
                if (!matched && nodes.empty()) {
                    patchBox(patch.record);
                } else if (!matched) {
                    LOG_WARN("scenery %u record %u: no 0x3410F entry; cull box left as shipped",
                             patch.sceneryNumber, patch.record);
                }
            }
            found = true;
        }
        if (!found) {
            error = "scenery chunk not found for the patch";
            return false;
        }
    }
    return true;
}
bool NfswRebuildScenery(ByteSpan section, u32 sceneryNumber,
                        const std::vector<NfswSceneryUpdate>& updates,
                        const std::vector<u32>& removals,
                        const std::vector<NfswSceneryAppend>& appends,
                        std::vector<u8>& out, std::string& error,
                        const std::vector<u32>& registeredRemovals,
                        size_t* firstAppendedRecord) {
    if (section.empty()) {
        error = "empty section";
        return false;
    }
    Chunk scenery;
    if (!FindSceneryChunk(section, sceneryNumber, scenery)) {
        error = "scenery chunk not found in the section";
        return false;
    }
    SceneryParts parts;
    if (!ParseSceneryParts(scenery, parts) || parts.instances.empty()) {
        error = "scenery chunk has no instance array";
        return false;
    }

    const size_t oldInstances = parts.instances.size / kInstEntrySize;
    const bool validBindings = parts.bindings.size % 32 == 0 &&
                               parts.bindings.size / 32 <= oldInstances;
    const bool havePrecull = parts.precull.size % 32 == 0 &&
                             parts.precull.size == parts.cull.size;
    if ((!removals.empty() || !appends.empty()) && !validBindings) {
        error = "0x3410D extends past the instance array or has a partial record";
        return false;
    }
    if (!appends.empty() && !havePrecull) {
        error = "the section's 0x3410E/0x3410F arrays are incomplete";
        return false;
    }

    std::vector<u8> infos(parts.infos.data, parts.infos.data + parts.infos.size);
    // The info reader divides the chunk size by 72 and ignores trailing bytes;
    // a previous append can leave up to 15 alignment bytes behind. Appending
    // after them would put the next info at a non-multiple of 72.
    infos.resize((parts.infos.size / kInfoEntrySize) * kInfoEntrySize);
    std::vector<u8> instances(parts.instances.data,
                              parts.instances.data + parts.instances.size);
    std::vector<u8> bindings;
    if (!parts.bindings.empty())
        bindings.assign(parts.bindings.data, parts.bindings.data + parts.bindings.size);
    std::vector<u8> precull(parts.precull.data, parts.precull.data + parts.precull.size);
    std::vector<u8> cull(parts.cull.data, parts.cull.data + parts.cull.size);
    std::vector<u8> vectorized(parts.vectorized.data,
                               parts.vectorized.data + parts.vectorized.size);
    std::vector<u8> overrides(parts.overrides.data,
                              parts.overrides.data + parts.overrides.size);
    std::vector<u8> tree(parts.tree.data, parts.tree.data + parts.tree.size);
    bool infosChanged = false, instancesChanged = false, bindingsChanged = false;
    bool precullChanged = false, cullChanged = false, overridesChanged = false;
    bool treeChanged = false, vectorizedChanged = false;

    // --- updates -------------------------------------------------------------
    for (const NfswSceneryUpdate& update : updates) {
        if (update.record >= oldInstances) {
            error = "scenery update outside the instance array";
            return false;
        }
        if (!update.bounds.Valid()) {
            error = "scenery update has no bounds";
            return false;
        }
        WriteInstanceTransform(instances, (size_t)update.record * kInstEntrySize,
                               update.transform, update.bounds);
        instancesChanged = true;

        if (!precull.empty() && havePrecull) {
            bool matched = false;
            for (size_t i = 0; i < cull.size() / 32; ++i) {
                const u8* entry = cull.data() + i * 32;
                if (!NonZeroRecord(entry, 32)) continue;
                u32 instance = 0;
                std::memcpy(&instance, entry + 0x10, sizeof(instance));
                if (instance != update.record) continue;
                WritePrecullBounds(precull, i * 32, update.bounds);
                matched = true;
                precullChanged = true;
            }
            // Without a 0x3410F table the record index is the only mapping. When
            // the table is present but names no box for this record, the box at
            // the same index belongs to another LOD; leave it as shipped.
            if (!matched && cull.empty() && update.record < precull.size() / 32) {
                WritePrecullBounds(precull, (size_t)update.record * 32, update.bounds);
                precullChanged = true;
            }
        }
    }

    // --- removals ------------------------------------------------------------
    std::vector<u32> remove = removals;
    std::sort(remove.begin(), remove.end());
    remove.erase(std::unique(remove.begin(), remove.end()), remove.end());
    if (remove.size() > oldInstances) {
        error = "more removals than instances";
        return false;
    }
    for (u32 record : remove)
        if (record >= oldInstances) {
            error = "scenery removal outside the instance array";
            return false;
        }
    if (!overrides.empty()) {
        for (u32 record : remove) {
            if (std::find(registeredRemovals.begin(), registeredRemovals.end(), record) !=
                registeredRemovals.end()) continue;
            for (size_t at = 0; at + 4 <= overrides.size(); at += 4) {
                u32 value = 0;
                std::memcpy(&value, overrides.data() + at, sizeof(value));
                if ((value >> 16) == record) {
                    error = "a shipped record with a holiday override cannot be removed yet";
                    return false;
                }
            }
        }
    }

    for (auto it = remove.rbegin(); it != remove.rend(); ++it) {
        const u32 record = *it;

        if (havePrecull && (!precull.empty() || !cull.empty())) {
            std::vector<size_t> drop;
            for (size_t i = 0; i < cull.size() / 32; ++i) {
                u8* entry = cull.data() + i * 32;
                if (!NonZeroRecord(entry, 32)) continue;
                u32 instance = 0;
                std::memcpy(&instance, entry + 0x10, sizeof(instance));
                if (instance == record) {
                    drop.push_back(i);
                    continue;
                }
                if (instance > record) {
                    PutU32(cull, i * 32 + 0x10, instance - 1);
                    u32 duplicate = 0;
                    std::memcpy(&duplicate, entry + 0x18, sizeof(duplicate));
                    if (duplicate == instance) PutU32(cull, i * 32 + 0x18, instance - 1);
                    cullChanged = true;
                }
            }
            for (auto dropIt = drop.rbegin(); dropIt != drop.rend(); ++dropIt) {
                const size_t at = *dropIt * 32;
                if (!vectorized.empty()) {
                    VectorRemoveEntry(vectorized, *dropIt);
                    vectorizedChanged = true;
                }
                precull.erase(precull.begin() + (ptrdiff_t)at,
                              precull.begin() + (ptrdiff_t)(at + 32));
                cull.erase(cull.begin() + (ptrdiff_t)at,
                           cull.begin() + (ptrdiff_t)(at + 32));
            }
            if (!drop.empty()) {
                precullChanged = true;
                cullChanged = true;
            }
        }

        // Overrides that name a later record keep naming it after the shift.
        if (!overrides.empty()) {
            for (size_t at = 0; at + 4 <= overrides.size();) {
                u32 value = 0;
                std::memcpy(&value, overrides.data() + at, sizeof(value));
                const u32 instance = value >> 16;
                if (instance == record) {
                    overrides.erase(overrides.begin() + at, overrides.begin() + at + 4);
                    overridesChanged = true;
                    continue;
                }
                if (instance > record) {
                    value = (value & 0xFFFFu) | ((instance - 1) << 16);
                    PutU32(overrides, at, value);
                    overridesChanged = true;
                }
                at += 4;
            }
        }

        instances.erase(instances.begin() + (ptrdiff_t)(record * kInstEntrySize),
                        instances.begin() + (ptrdiff_t)((record + 1) * kInstEntrySize));
        if (record < bindings.size() / 32) {
            bindings.erase(bindings.begin() + (ptrdiff_t)(record * 32),
                           bindings.begin() + (ptrdiff_t)((record + 1) * 32));
            bindingsChanged = true;
        }
        instancesChanged = true;
    }

    if (!remove.empty() && !tree.empty()) {
        if (!CullRemoveInstances(tree, remove, oldInstances, error)) return false;
        treeChanged = true;
    }

    // --- appends -------------------------------------------------------------
    size_t instanceCount = instances.size() / kInstEntrySize;
    const size_t firstAppend = instanceCount;
    u32 nextGuid = 1;
    for (size_t at = 0; at + kInstEntrySize <= instances.size(); at += kInstEntrySize) {
        u32 guid = 0;
        std::memcpy(&guid, instances.data() + at + 0x50, sizeof(guid));
        if (guid >= nextGuid) nextGuid = guid + 1;
    }

    std::vector<PendingCullEntry> pendingCulls;
    for (const NfswSceneryAppend& append : appends) {
        if (append.sceneryNumber != sceneryNumber) {
            error = "append targets a different scenery chunk";
            return false;
        }
        if (!append.bounds.Valid()) {
            error = "append has no bounds";
            return false;
        }
        Chunk templateChunk;
        if (!FindSceneryChunk(append.templateSection, append.templateSceneryNumber,
                              templateChunk)) {
            error = "template scenery chunk not found";
            return false;
        }
        SceneryParts source;
        if (!ParseSceneryParts(templateChunk, source) || source.instances.empty() ||
            source.infos.empty()) {
            error = "template scenery chunk has no infos or instances";
            return false;
        }
        if (source.precull.size != source.cull.size || source.precull.size % 32 != 0) {
            error = "template 0x3410E/0x3410F arrays are incomplete";
            return false;
        }
        if (append.templateRecord >= source.instances.size / kInstEntrySize) {
            error = "template record outside its instance array";
            return false;
        }
        const u8* templateInstance =
            source.instances.data + (size_t)append.templateRecord * kInstEntrySize;
        u16 templateInfo = 0;
        std::memcpy(&templateInfo, templateInstance + kInstInfoIndex, sizeof(templateInfo));
        if ((templateInfo + 1) * kInfoEntrySize > source.infos.size) {
            error = "template info index outside its info array";
            return false;
        }
        const u8* templateInfoData = source.infos.data + (size_t)templateInfo * kInfoEntrySize;
        std::vector<u8> createdInfo;
        if (append.newModelHash) {
            if (append.newModelName.empty() || append.newModelName.size() >= 24) {
                error = "the new World road model name does not fit the scenery info";
                return false;
            }
            createdInfo.assign(templateInfoData, templateInfoData + kInfoEntrySize);
            std::fill(createdInfo.begin(), createdInfo.begin() + 24, 0);
            std::memcpy(createdInfo.data(), append.newModelName.data(), append.newModelName.size());
            for (size_t lod = 0x18; lod < 0x28; lod += 4) PutU32(createdInfo, lod, append.newModelHash);
            PutF(createdInfo, 0x3C, append.bounds.Radius());
            templateInfoData = createdInfo.data();
        }
        bool registered = false;
        for (size_t at = 0; at + 4 <= source.overrides.size; at += 4) {
            if ((source.overrides.read<u32>(at) >> 16) != append.templateRecord) continue;
            registered = true;
            break;
        }
        if (registered && (!append.plainCopyFlags || (*append.plainCopyFlags & 0x18))) {
            error = "the template has a scenery registration that cannot be transferred";
            return false;
        }

        if (append.modelHash && !append.newModelHash) {
            // The editor may resolve a later LOD when the first hash is absent.
            const ByteSpan info{ templateInfoData, kInfoEntrySize };
            bool matched = false;
            for (size_t at = 0x18; at < 0x28; at += 4)
                if (info.read<u32>(at) == append.modelHash) matched = true;
            if (!matched) {
                error = "append model hash does not match the template info";
                return false;
            }
        }

        u32 targetInfo = ~0u;
        for (size_t i = 0; i < infos.size() / kInfoEntrySize; ++i) {
            const u8* info = infos.data() + i * kInfoEntrySize;
            if (std::memcmp(info, templateInfoData, 0x28) == 0 &&
                std::memcmp(info + 0x38, templateInfoData + 0x38, kInfoEntrySize - 0x38) == 0) {
                targetInfo = (u32)i;
                break;
            }
        }
        if (targetInfo == ~0u) {
            targetInfo = (u32)(infos.size() / kInfoEntrySize);
            infos.insert(infos.end(), templateInfoData, templateInfoData + kInfoEntrySize);
            // These are runtime pointers the loader fills in; the shipped
            // infos keep them zero.
            for (size_t at = 0x28; at <= 0x34; at += 4)
                std::memset(infos.data() + (size_t)targetInfo * kInfoEntrySize + at, 0, 4);
            infosChanged = true;
        }
        if (targetInfo > 0xFFFFu) {
            error = "too many scenery infos for a 16-bit info index";
            return false;
        }

        const size_t newIndex = instanceCount;
        instances.insert(instances.end(), templateInstance,
                         templateInstance + kInstEntrySize);
        if (registered) {
            const ByteSpan sourceRecord{ templateInstance, kInstEntrySize };
            // The registration supplies the low flags at load time and
            // may override the transform. This copy has a baked transform.
            constexpr u32 kRegistrationFlags = 0x40000000u | 0x00800000u;
            const u32 flags = (sourceRecord.read<u32>(kInstFlags) & 0xFFFF0000u & ~kRegistrationFlags) |
                              *append.plainCopyFlags;
            PutU32(instances, newIndex * kInstEntrySize + kInstFlags, flags);
        }
        WriteInstanceTransform(instances, newIndex * kInstEntrySize, append.transform,
                               append.bounds);
        PutU32(instances, newIndex * kInstEntrySize + 0x50, nextGuid++);
        std::memcpy(instances.data() + newIndex * kInstEntrySize + kInstInfoIndex,
                    &targetInfo, sizeof(u16));
        // The loader binds a prefix of the instance array. Keep a trailing
        // unbound instance unbound rather than assigning it the next binding.
        PutU32(instances, newIndex * kInstEntrySize + 0x5C, 0);
        if (!append.newModelHash && !bindings.empty() && bindings.size() / 32 == newIndex &&
            append.templateRecord < source.bindings.size / 32) {
            const u8* binding = source.bindings.data + (size_t)append.templateRecord * 32;
            bindings.insert(bindings.end(), binding, binding + 32);
            bindingsChanged = true;
        }
        if (!CullAddInstance(tree, (u32)newIndex, append.bounds, error)) return false;
        treeChanged = true;
        instancesChanged = true;

        size_t copied = 0;
        for (size_t i = 0; i < source.cull.size / 32; ++i) {
            const u8* entry = source.cull.data + i * 32;
            if (!NonZeroRecord(entry, 32)) continue;
            u32 instance = 0;
            std::memcpy(&instance, entry + 0x10, sizeof(instance));
            if (instance != append.templateRecord) continue;

            PendingCullEntry addition;
            addition.type = VectorTypeForEntry(source.vectorized, i);
            addition.box.assign(source.precull.data + i * 32,
                                source.precull.data + (i + 1) * 32);
            WritePrecullBounds(addition.box, 0, append.bounds);
            addition.node.assign(entry, entry + 32);
            PutU32(addition.node, 0x10, (u32)newIndex);
            PutU32(addition.node, 0x14, targetInfo);
            PutU32(addition.node, 0x18, (u32)newIndex);
            pendingCulls.push_back(std::move(addition));
            ++copied;
        }
        if (!copied) {
            error = "the template record has no 0x3410F precull entry";
            return false;
        }
        ++instanceCount;
    }

    if (!pendingCulls.empty()) {
        if (!vectorized.empty()) {
            if (!InsertVectorizedCulls(vectorized, precull, cull, pendingCulls, error))
                return false;
            vectorizedChanged = true;
        } else {
            // Very old sections have no 0x34C03 and fall back to 0x34105, so
            // appending at the end is enough there.
            for (const PendingCullEntry& addition : pendingCulls) {
                precull.insert(precull.end(), addition.box.begin(), addition.box.end());
                cull.insert(cull.end(), addition.node.begin(), addition.node.end());
            }
        }
        precullChanged = true;
        cullChanged = true;
    }

    // --- rebuild the 0x80034100 chunk ---------------------------------------
    std::unordered_map<u32, std::vector<u8>> changed;
    if (infosChanged) {
        // The info reader divides the chunk size by 72; trailing zeros are
        // ignored. Pad so the chunk grows by a multiple of 16 -- the alignment
        // every following chunk was written with -- not to a 16-byte size.
        const size_t pad = (parts.infos.size % 16 + 16 - (infos.size() % 16)) % 16;
        infos.insert(infos.end(), pad, 0);
        changed[kChunkSceneryInfos] = std::move(infos);
    }
    if (instancesChanged) changed[kChunkSceneryInstances] = std::move(instances);
    if (bindingsChanged) changed[kChunkSceneryInstanceBindings] = std::move(bindings);
    if (precullChanged) changed[kChunkSceneryPrecullBoxes] = std::move(precull);
    if (cullChanged) changed[kChunkSceneryPrecullNodes] = std::move(cull);
    if (vectorizedChanged) changed[kChunkSceneryVectorized] = std::move(vectorized);
    if (overridesChanged) changed[kChunkSceneryOverrides] = std::move(overrides);
    if (treeChanged) changed[kChunkSceneryCullTree] = std::move(tree);

    std::vector<u8> rebuilt;
    rebuilt.reserve(scenery.raw.size + 4096);
    rebuilt.resize(8);
    std::memcpy(rebuilt.data(), &scenery.id, sizeof(scenery.id));
    u64 cursor = scenery.headerOffset + 8;

    ChunkReader children = ChunkReader::Children(scenery);
    Chunk child;
    while (children.Next(child)) {
        if (child.headerOffset > cursor) {
            rebuilt.insert(rebuilt.end(), section.data + cursor,
                           section.data + child.headerOffset);
        }
        const auto found = changed.find(child.id);
        if (found == changed.end()) {
            rebuilt.insert(rebuilt.end(), section.data + child.headerOffset,
                           section.data + child.headerOffset + 8 + child.raw.size);
        } else {
            const std::vector<u8>& body = found->second;
            const size_t prefix = child.id == kChunkSceneryInstanceBindings ? 0 : child.FillerBytes();
            const ptrdiff_t delta = (ptrdiff_t)(prefix + body.size()) - (ptrdiff_t)child.raw.size;
            size_t padding = 0;
            if (child.id == kChunkSceneryOverrides && delta < 0) {
                // The game treats every 4 bytes in this child as an override.
                // Put alignment in a separate zero-id chunk.
                padding = (size_t)(-delta % 128);
                if (padding && padding < 8) padding += 128;
            } else if (delta % 16 != 0) {
                error = "a changed scenery child would shift the next chunk off its alignment";
                return false;
            }
            const size_t at = rebuilt.size();
            rebuilt.resize(at + 8);
            std::memcpy(rebuilt.data() + at, &child.id, sizeof(child.id));
            rebuilt.insert(rebuilt.end(), child.raw.data, child.raw.data + prefix);
            rebuilt.insert(rebuilt.end(), body.begin(), body.end());
            const u32 size = (u32)(prefix + body.size());
            std::memcpy(rebuilt.data() + at + 4, &size, sizeof(size));
            if (padding) {
                const size_t padHeader = rebuilt.size();
                rebuilt.resize(padHeader + padding, 0);
                PutU32(rebuilt, padHeader + 4, (u32)(padding - 8));
            }
        }
        cursor = child.headerOffset + 8 + child.raw.size;
    }
    const u64 chunkEnd = scenery.headerOffset + 8 + scenery.raw.size;
    if (cursor < chunkEnd) {
        rebuilt.insert(rebuilt.end(), section.data + cursor, section.data + chunkEnd);
    }
    const u32 chunkSize = (u32)(rebuilt.size() - 8);
    std::memcpy(rebuilt.data() + 4, &chunkSize, sizeof(chunkSize));

    const ptrdiff_t chunkDelta = (ptrdiff_t)rebuilt.size() -
                                 (ptrdiff_t)(8 + scenery.raw.size);
    if (chunkDelta % 16 != 0) {
        error = "the rebuilt scenery chunk would shift the next chunk off its alignment";
        return false;
    }

    out.assign(section.data, section.data + scenery.headerOffset);
    out.insert(out.end(), rebuilt.begin(), rebuilt.end());
    out.insert(out.end(), section.data + chunkEnd, section.data + section.size);
    if (firstAppendedRecord) *firstAppendedRecord = firstAppend;
    return true;
}

void NfswParseAnimations(const std::vector<Chunk>& chunks,
                         std::vector<NfswObjectAnimation>& out) {
    NfswObjectAnimation pending;
    bool haveNode = false;
    for (const Chunk& chunk : chunks) {
        if (chunk.id == kChunkSceneryAnimNode) {
            const ByteSpan node = chunk.data;
            if (node.size < 0x70) { haveNode = false; continue; }
            pending = NfswObjectAnimation{};
            pending.guid    = node.read<u32>(0x58);
            pending.section = node.read<u16>(0x62);
            const u32 count = node.read<u32>(0x64);
            const u8 speed  = node.read<u8>(0x45);
            const u8 high   = speed >> 4, low = speed & 0x0F;
            pending.speed   = (float)(high ? high : 1) + (float)low / 15.0f;
            // The loader leaves the keyframe pointer empty for these nodes.
            haveNode = count > 0 && (node.read<u8>(0x46) & 6) == 0;
        } else if (chunk.id == kChunkSceneryAnimFrames && haveNode) {
            const size_t count = chunk.data.size / 64;
            pending.frames.reserve(count);
            for (size_t i = 0; i < count; ++i) {
                Mat4 frame = Mat4::Identity();
                for (int row = 0; row < 4; ++row)
                    for (int col = 0; col < 4; ++col)
                        frame.m[row][col] =
                            chunk.data.read<float>((i * 16 + (size_t)row * 4 + (size_t)col) * 4);
                pending.frames.push_back(frame);
            }
            if (!pending.frames.empty()) out.push_back(std::move(pending));
            haveNode = false;
        } else if (chunk.id == kChunkSceneryAnimEnd) {
            haveNode = false;
        }
    }
}

} // namespace nfs
