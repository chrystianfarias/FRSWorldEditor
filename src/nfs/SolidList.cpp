#include "nfs/SolidList.h"

#include "core/Log.h"

#include <algorithm>
#include <cstring>

namespace nfs {

namespace {

// --- Solid object header, chunk 0x00134011 ---------------------------------
constexpr size_t kHdrVersion   = 0x0C;
constexpr size_t kHdrHash      = 0x10;
constexpr size_t kHdrNumTris   = 0x14;
constexpr size_t kHdrBoundsMin = 0x20;
constexpr size_t kHdrBoundsMax = 0x30;
constexpr size_t kHdrTransform = 0x40;  // 4x4 floats, row major, translation in row 3
constexpr size_t kHdrName      = 0xA4;  // zero-terminated ASCII
// Underground 1 (header version 0x13 in the low word -- the crowd's skinned
// people carry 0x40 in the high one -- Underground 2's is 0x16) has the same
// header sixteen bytes shorter before the name.
constexpr u32    kHdrVersionU1 = 0x13;
constexpr size_t kHdrNameU1    = 0x94;

// Most Wanted: the same header, with the name at 0xA0.
constexpr size_t kHdrNameMW    = 0xA0;

// --- Mesh descriptor, chunk 0x00134900 -------------------------------------
constexpr size_t kDescNumMaterials = 0x10;
constexpr size_t kDescNumTriangles = 0x24;
constexpr size_t kDescNumVertices  = 0x34;

// --- Material entry, chunk 0x00134B02 --------------------------------------
constexpr size_t kMaterialEntrySize = 60;
constexpr size_t kMatBoundsMin      = 0x00;
constexpr size_t kMatIndexCount     = 0x0C;
constexpr size_t kMatBoundsMax      = 0x10;
constexpr size_t kMatTextureIndex   = 0x1C;
constexpr size_t kMatFlags          = 0x20;
constexpr size_t kMatFirstIndex     = 0x34;
constexpr size_t kMatStripFlags     = 0x38;

// --- Vertex layouts --------------------------------------------------------
// Two strides occur across every shipped Underground 2 track stream.
constexpr u32 kStridePositionColorUv       = 24;  // pos, color, uv
constexpr u32 kStridePositionNormalColorUv = 36;  // pos, normal, color, uv
// Underground 1's crowd: the 36-byte layout, then bone weights and indices.
constexpr u32 kStrideSkinned               = 60;

Vec3 ReadVec3(ByteSpan span, size_t offset) {
    return { span.read<float>(offset),
             span.read<float>(offset + 4),
             span.read<float>(offset + 8) };
}

Mat4 ReadMat4(ByteSpan span, size_t offset) {
    Mat4 m;
    for (int row = 0; row < 4; ++row)
        for (int col = 0; col < 4; ++col)
            m.m[row][col] = span.read<float>(offset + (size_t)(row * 4 + col) * 4);
    return m;
}

std::string ReadName(ByteSpan span, size_t offset) {
    if (offset >= span.size) return {};
    const char* text = (const char*)span.data + offset;
    const size_t available = span.size - offset;
    const size_t length = ::strnlen(text, available);
    return std::string(text, length);
}

void ParseHeader(ByteSpan data, SolidObject& out, Game game) {
    out.version    = data.read<u32>(kHdrVersion);
    out.hash       = data.read<u32>(kHdrHash);
    out.bounds.min = ReadVec3(data, kHdrBoundsMin);
    out.bounds.max = ReadVec3(data, kHdrBoundsMax);
    out.transform  = ReadMat4(data, kHdrTransform);
    out.name       = ReadName(data, game == Game::MostWanted || game == Game::Carbon ? kHdrNameMW
                                    : (out.version & 0xFFFFu) <= kHdrVersionU1 ? kHdrNameU1 : kHdrName);
    out.kind       = ClassifyObject(out.name);
}

void ParseTextureRefs(ByteSpan data, SolidObject& out) {
    // Eight bytes per entry: a texture hash followed by four reserved bytes.
    const size_t count = data.size / 8;
    out.textureHashes.reserve(count);
    for (size_t i = 0; i < count; ++i)
        out.textureHashes.push_back(data.read<u32>(i * 8));
}

void ParseMaterials(ByteSpan data, SolidObject& out) {
    const size_t count = data.size / kMaterialEntrySize;
    out.materials.reserve(count);

    for (size_t i = 0; i < count; ++i) {
        const ByteSpan entry = data.sub(i * kMaterialEntrySize, kMaterialEntrySize);

        SolidMaterial material;
        material.indexCount   = entry.read<u32>(kMatIndexCount);
        material.firstIndex   = entry.read<u32>(kMatFirstIndex);
        material.textureIndex = entry.read<u32>(kMatTextureIndex);
        material.flags        = entry.read<u32>(kMatFlags);
        material.stripFlags   = entry.read<u32>(kMatStripFlags);
        material.bounds.min   = ReadVec3(entry, kMatBoundsMin);
        material.bounds.max   = ReadVec3(entry, kMatBoundsMax);

        out.materials.push_back(material);
    }
}

// Widens the packed source vertices into SolidVertex.
bool ParseVertices(ByteSpan data, u32 vertexCount, SolidObject& out) {
    if (vertexCount == 0) return false;
    if (data.size % vertexCount != 0) {
        LOG_WARN("object '%s': vertex buffer %zu bytes is not a multiple of %u vertices",
                 out.name.c_str(), data.size, vertexCount);
        return false;
    }

    const u32 stride = (u32)(data.size / vertexCount);
    size_t colorOffset, uvOffset;
    bool hasNormal;

    switch (stride) {
        case kStridePositionColorUv:
            hasNormal = false; colorOffset = 12; uvOffset = 16;
            break;
        case kStridePositionNormalColorUv:
        case kStrideSkinned:
            hasNormal = true;  colorOffset = 24; uvOffset = 28;
            break;
        default:
            LOG_WARN("object '%s': unsupported vertex stride %u", out.name.c_str(), stride);
            return false;
    }

    out.hasNormals   = hasNormal;
    out.vertexStride = stride;
    out.vertices.resize(vertexCount);
    for (u32 i = 0; i < vertexCount; ++i) {
        const ByteSpan source = data.sub((size_t)i * stride, stride);
        SolidVertex& vertex = out.vertices[i];

        vertex.position = ReadVec3(source, 0);
        vertex.normal   = hasNormal ? ReadVec3(source, 12) : Vec3{ 0.0f, 0.0f, 1.0f };
        vertex.color    = source.read<u32>(colorOffset);
        vertex.uv       = { source.read<float>(uvOffset), source.read<float>(uvOffset + 4) };
    }
    return true;
}

void ParseIndices(ByteSpan data, u32 triangleCount, SolidObject& out) {
    // The buffer is padded out to an alignment boundary, so trust the triangle
    // count from the descriptor over the raw byte size.
    size_t count = (size_t)triangleCount * 3;
    if (count * 2 > data.size) count = data.size / 2;

    out.indices.resize(count);
    if (count) std::memcpy(out.indices.data(), data.data, count * 2);
}

// --- Most Wanted's mesh ------------------------------------------------------
// Material, chunk 0x00134B02, 0x68 bytes: bounds min/max at 0x00/0x0C, u8
// texture slots[5] at 0x18 (diffuse, normal, height, specular, opacity; each
// an index into the object's texture list), then at 0x30 u32 effect, ?,
// flags, vertex count, triangle count, first index; index count at 0x5C.
// The vertex buffers (0x00134B01, one chunk each) follow the effect: a new one
// starts each time the effect changes from one material to the next, and the
// indices of a material count from the start of its own buffer.
//
// Carbon keeps all of it with a 0x90-byte material: the vertex count at 0x40,
// the first index at 0x64, the index count at 0x7C (NFSC.exe; the ProStreet
// FreeRoam converter's c2ps_solid.py reads them the same way).
struct MaterialLayout {
    size_t size, textures, effect, flags, vertexCount, firstIndex, indexCount;
};
constexpr MaterialLayout kMaterialMW     { 0x68, 0x18, 0x30, 0x38, 0x3C, 0x44, 0x5C };
constexpr MaterialLayout kMaterialCarbon { 0x90, 0x18, 0x30, 0x38, 0x40, 0x64, 0x7C };

// Where a vertex keeps what the editor draws with; normal < 0: none (it
// faces up).
struct VertexLayout {
    u32 stride;
    int normal;
    u32 color, uv;
};

// speed.exe's effect table (0x8F9B60): the vertex size of each effect the
// world uses. 0 WORLD, 1 WORLDREFLECT, 3 WORLDNORMALMAP, 5 GLOSSYWINDOW,
// 6 TREE all start pos, normal, colour, uv; 19 the sky adds a second uv.
u32 EffectStrideMW(u32 effect) {
    switch (effect) {
        case 0: case 4: case 5: case 6: case 8: return 36;
        case 1: case 2: case 3:                 return 60;
        case 7:                                 return 24;
        case 19:                                return 44;
        default:                                return 0;
    }
}

VertexLayout EffectLayoutMW(u32 effect) {
    const u32 stride = EffectStrideMW(effect);
    if (stride && stride < 36) return { stride, -1, 12, 16 };
    return { stride, 12, 24, 28 };
}

// NFSC.exe's effect table (0xA64008, 0x90 bytes an effect: its name, then the
// vertex elements in buffer order).
VertexLayout EffectLayoutCarbon(u32 effect) {
    switch (effect) {
        case 0: case 17: case 18: return { 36, 12, 24, 28 };   // WorldShader: pos normal colour uv
        case 1: case 3:           return { 52, 12, 24, 28 };   // WorldReflect, WorldNormalMap: + tangent
        case 20:                  return { 60, 12, 24, 28 };   // NormalMapNoFog: + uv1, tangent
        case 2:                   return { 72, 12, 24, 28 };   // WorldBone: + bones
        case 4: case 28:          return { 48, 12, 24, 28 };   // CarShader, GHOSTCAR: + tangent
        case 5:                   return { 48, 24, 20, 12 };   // CARNORMALMAP: pos uv colour normal tangent
        case 25:                  return { 36, 24, 20, 12 };   // GLASS_REFLECT: pos uv colour normal
        case 26:                  return { 36, 24, 12, 16 };   // WATER: pos colour uv normal
        case 6: case 13:          return { 24, -1, 12, 16 };   // WorldMin, WorldPrelit: pos colour uv
        case 15:                  return { 44, 12, 24, 28 };   // skyshader: + uv1
        default:                  return { 0, 12, 24, 28 };
    }
}

bool ParseMeshMW(const Chunk& mesh, SolidObject& out, Game game) {
    const bool carbon = game == Game::Carbon;
    const MaterialLayout& layout = carbon ? kMaterialCarbon : kMaterialMW;
    const size_t kMaterialEntrySizeMW = layout.size;
    const size_t kMatTexturesMW = layout.textures, kMatEffectMW = layout.effect, kMatFlagsMW = layout.flags,
                 kMatVertexCountMW = layout.vertexCount, kMatFirstIndexMW = layout.firstIndex,
                 kMatIndexCountMW = layout.indexCount;
    ByteSpan materialData, indexData;
    std::vector<ByteSpan> buffers;
    ChunkReader reader = ChunkReader::Children(mesh);
    Chunk part;
    while (reader.Next(part)) {
        switch (part.id) {
            case kChunkMeshMaterials: materialData = part.data; break;
            case kChunkMeshIndices:   indexData = part.data; break;
            // Carbon aligns some buffers to 16 bytes, not 128: the filler the
            // reader looked for is not there, and is cut below instead.
            case kChunkMeshVertices:  buffers.push_back(carbon ? part.raw : part.data); break;
            default: break;
        }
    }

    struct Run { u32 effect, vertices; };
    std::vector<Run> runs;
    std::vector<u32> materialRun;
    const size_t count = materialData.size / kMaterialEntrySizeMW;
    for (size_t i = 0; i < count; ++i) {
        const ByteSpan entry = materialData.sub(i * kMaterialEntrySizeMW, kMaterialEntrySizeMW);
        const u32 effect = entry.read<u32>(kMatEffectMW);
        if (runs.empty() || runs.back().effect != effect) runs.push_back({ effect, 0 });
        runs.back().vertices += entry.read<u32>(kMatVertexCountMW);
        materialRun.push_back((u32)runs.size() - 1);
    }
    if (runs.size() > buffers.size()) {
        LOG_WARN("object '%s': %zu vertex buffers for %zu effect runs", out.name.c_str(), buffers.size(), runs.size());
        return false;
    }

    // Every buffer widened into the one vertex array, each run's start kept.
    std::vector<u32> runBase;
    for (size_t r = 0; r < runs.size(); ++r) {
        VertexLayout vertexLayout = carbon ? EffectLayoutCarbon(runs[r].effect) : EffectLayoutMW(runs[r].effect);
        u32& stride = vertexLayout.stride;
        if (carbon && runs[r].vertices && stride && (size_t)stride * runs[r].vertices <= buffers[r].size) {
            // The data ends the chunk; what is before it must be filler.
            const size_t lead = buffers[r].size - (size_t)stride * runs[r].vertices;
            bool filler = true;
            for (size_t k = 0; k < lead && filler; ++k) filler = buffers[r].data[k] == 0x11;
            if (!filler) {
                LOG_WARN("object '%s': %zu bytes before vertex buffer %zu are not filler", out.name.c_str(), lead, r);
                return false;
            }
            buffers[r] = buffers[r].sub(lead, buffers[r].size - lead);
        }
        if (runs[r].vertices && (!stride || (size_t)stride * runs[r].vertices > buffers[r].size)) {
            stride = (u32)(buffers[r].size / runs[r].vertices);
            vertexLayout = stride >= 36 ? VertexLayout{ stride, 12, 24, 28 } : VertexLayout{ stride, -1, 12, 16 };
        }
        if (runs[r].vertices && stride < 24) {
            LOG_WARN("object '%s': effect %u with a %u-byte vertex", out.name.c_str(), runs[r].effect, stride);
            return false;
        }
        const bool hasNormal = vertexLayout.normal >= 0;
        const size_t colorOffset = vertexLayout.color, uvOffset = vertexLayout.uv;
        runBase.push_back((u32)out.vertices.size());
        for (u32 v = 0; v < runs[r].vertices; ++v) {
            const ByteSpan source = buffers[r].sub((size_t)v * stride, stride);
            SolidVertex vertex;
            vertex.position = ReadVec3(source, 0);
            vertex.normal   = hasNormal ? ReadVec3(source, (size_t)vertexLayout.normal) : Vec3{ 0.0f, 0.0f, 1.0f };
            vertex.color    = source.read<u32>(colorOffset);
            vertex.uv       = { source.read<float>(uvOffset), source.read<float>(uvOffset + 4) };
            out.vertices.push_back(vertex);
        }
        if (hasNormal) out.hasNormals = true;
    }
    if (out.vertices.size() > 0x10000) {
        LOG_WARN("object '%s': %zu vertices do not fit 16-bit indices", out.name.c_str(), out.vertices.size());
        out.vertices.clear();
        return false;
    }

    const size_t indexCount = indexData.size / 2;
    std::vector<u16> source(indexCount);
    if (indexCount) std::memcpy(source.data(), indexData.data, indexCount * 2);
    out.indices.clear();
    out.materials.clear();
    for (size_t i = 0; i < count; ++i) {
        const ByteSpan entry = materialData.sub(i * kMaterialEntrySizeMW, kMaterialEntrySizeMW);
        const u32 first = entry.read<u32>(kMatFirstIndexMW), number = entry.read<u32>(kMatIndexCountMW);
        if (!number || (size_t)first + number > indexCount) continue;
        SolidMaterial material;
        material.firstIndex   = (u32)out.indices.size();
        material.indexCount   = number;
        material.textureIndex = entry.read<u8>(kMatTexturesMW);
        material.flags        = entry.read<u32>(kMatFlagsMW);
        // The editor's effect pick (render/GpuResources.cpp): 0x40 draws the
        // road reflection, which Most Wanted gives effect 1 (WORLDREFLECT).
        material.stripFlags   = entry.read<u32>(kMatEffectMW) == 1 ? 0x40u : 0u;
        material.bounds.min   = ReadVec3(entry, 0x00);
        material.bounds.max   = ReadVec3(entry, 0x0C);
        const u32 base = runBase[materialRun[i]];
        for (u32 k = 0; k < number; ++k) {
            const u32 index = source[first + k] + base;
            out.indices.push_back((u16)std::min<u32>(index, (u32)out.vertices.size() - 1));
        }
        out.materials.push_back(material);
    }
    return !out.vertices.empty() && !out.indices.empty();
}

} // namespace

ObjectKind ClassifyObject(const std::string& name) {
    if (name.rfind("PAN_", 0) == 0)    return ObjectKind::Backdrop;  // panorama pieces
    if (name.rfind("SKYDOME", 0) == 0) return ObjectKind::Backdrop;
    if (name.rfind("TRN_", 0) == 0)    return ObjectKind::Terrain;
    return ObjectKind::Prop;
}

bool ParseSolidObject(const Chunk& objectChunk, SolidObject& out, Game game) {
    ChunkReader reader = ChunkReader::Children(objectChunk);
    Chunk child;

    u32 vertexCount = 0;
    u32 triangleCount = 0;
    ByteSpan vertexData, indexData, materialData;

    while (reader.Next(child)) {
        switch (child.id) {
            case kChunkSolidObjectHeader:
                ParseHeader(child.data, out, game);
                triangleCount = child.data.read<u32>(kHdrNumTris);
                break;

            case kChunkSolidObjectTexRefs:
                ParseTextureRefs(child.data, out);
                break;

            case kChunkMeshContainer: {
                if (game == Game::MostWanted || game == Game::Carbon) {
                    if (!ParseMeshMW(child, out, game)) return false;
                    break;
                }
                ChunkReader mesh = ChunkReader::Children(child);
                Chunk part;
                while (mesh.Next(part)) {
                    switch (part.id) {
                        case kChunkMeshDescriptor:
                            vertexCount = part.data.read<u32>(kDescNumVertices);
                            if (u32 tris = part.data.read<u32>(kDescNumTriangles))
                                triangleCount = tris;
                            break;
                        case kChunkMeshVertices:
                            vertexData = part.data;
                            out.vertexFileOffset = part.dataOffset;
                            break;
                        case kChunkMeshIndices:   indexData    = part.data; break;
                        case kChunkMeshMaterials: materialData = part.data; break;
                        default: break;
                    }
                }
                break;
            }

            default:
                break;
        }
    }

    if (game == Game::MostWanted || game == Game::Carbon) return !out.vertices.empty() && !out.indices.empty();

    if (!ParseVertices(vertexData, vertexCount, out)) return false;
    ParseIndices(indexData, triangleCount, out);
    ParseMaterials(materialData, out);

    // Guard the draw calls: a corrupt range would otherwise read past the buffer.
    for (SolidMaterial& material : out.materials) {
        if (material.firstIndex > out.indices.size()) {
            material.firstIndex = 0;
            material.indexCount = 0;
        } else if (material.firstIndex + material.indexCount > out.indices.size()) {
            material.indexCount = (u32)(out.indices.size() - material.firstIndex);
        }
    }

    return !out.vertices.empty() && !out.indices.empty();
}

bool ParseSolidList(const Chunk& pack, SolidList& out, Game game) {
    ChunkReader reader = ChunkReader::Children(pack);
    Chunk child;

    while (reader.Next(child)) {
        if (child.id == kChunkSolidListHeader) {
            // The pack path sits in the padding chunk right after the header.
            const Chunk info = FindChunk(child, kChunkSolidListPadding);
            if (info.data.size > 0x10)
                out.path = ReadName(info.data, 0x10);
        } else if (child.id == kChunkSolidObject) {
            SolidObject object;
            if (ParseSolidObject(child, object, game))
                out.objects.push_back(std::move(object));
        }
    }

    return !out.objects.empty();
}

void ParseAllSolidLists(ByteSpan buffer, u64 fileOffset, std::vector<SolidList>& out) {
    ChunkReader reader(buffer, fileOffset);
    Chunk chunk;

    while (reader.Next(chunk)) {
        if (chunk.id != kChunkSolidListPack) continue;

        SolidList list;
        if (ParseSolidList(chunk, list))
            out.push_back(std::move(list));
    }
}

} // namespace nfs
