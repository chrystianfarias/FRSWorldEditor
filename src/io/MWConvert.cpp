#include "io/MWConvert.h"

#include "core/Log.h"
#include "io/ModelCopy.h"
#include "io/TextureImport.h"
#include "nfs/ChunkReader.h"
#include "nfs/SolidList.h"
#include "nfs/World.h"

#include <algorithm>
#include <cstring>
#include <unordered_set>

namespace {

constexpr u32 kObject     = 0x80134010;
constexpr u32 kHeader     = 0x00134011;
constexpr u32 kTextures   = 0x00134012;
constexpr u32 kMesh       = 0x80134100;
constexpr u32 kMeshHeader = 0x00134900;
constexpr u32 kVertices   = 0x00134B01;
constexpr u32 kMaterials  = 0x00134B02;
constexpr u32 kIndices    = 0x00134B03;

constexpr size_t kHeaderMW      = 0xA0;    // up to the name
constexpr size_t kHeaderU2      = 192;
constexpr size_t kNameU2        = 0xA4;
constexpr size_t kMeshHeaderU2  = 68;
constexpr size_t kMaterialU2    = 60;
constexpr u32    kHeaderConstantU2 = 0x000EE580;   // +0x88 and +0x8C of every Underground 2 header
constexpr u32    kMeshVersionU2    = 0x10;

void Put32(std::vector<u8>& b, size_t at, u32 v) { std::memcpy(b.data() + at, &v, 4); }
void PutF(std::vector<u8>& b, size_t at, float v) { std::memcpy(b.data() + at, &v, 4); }
void PutVec3(std::vector<u8>& b, size_t at, const Vec3& v) {
    PutF(b, at, v.x); PutF(b, at + 4, v.y); PutF(b, at + 8, v.z);
}

// Chunks written for a buffer that starts on a 128-byte boundary, each
// aligned payload preceded by the 0x11 filler that brings it to its boundary.
class Writer {
public:
    explicit Writer(std::vector<u8>& out) : b_(out) {}
    size_t Begin(u32 id) {
        const size_t at = b_.size();
        b_.resize(at + 8);
        Put32(b_, at, id);
        return at;
    }
    void End(size_t at) { Put32(b_, at + 4, (u32)(b_.size() - at - 8)); }
    void Leaf(u32 id, const std::vector<u8>& data, u32 align) {
        const size_t at = Begin(id);
        if (align) b_.insert(b_.end(), (align - b_.size() % align) % align, 0x11);
        b_.insert(b_.end(), data.begin(), data.end());
        End(at);
    }
private:
    std::vector<u8>& b_;
};

} // namespace

bool ConvertMWObject(ByteSpan object, u64 fileOffset, std::vector<u8>& out, std::wstring& error) {
    out.clear();
    nfs::ChunkReader reader(object, fileOffset);
    nfs::Chunk chunk;
    if (!reader.Next(chunk) || chunk.id != kObject) { error = L"That is not a model chunk."; return false; }

    nfs::SolidObject model;
    if (!nfs::ParseSolidObject(chunk, model, nfs::Game::MostWanted)) {
        error = L"The model could not be read as Most Wanted's.";
        return false;
    }
    const ByteSpan source = nfs::FindChunk(chunk, kHeader).data;
    if (source.size < kHeaderMW) { error = L"The model's header is not Most Wanted's."; return false; }

    // Only the textures the materials draw with, in the order first used.
    std::vector<u32> textures;
    std::vector<u32> textureOf(model.materials.size(), 0);
    for (size_t m = 0; m < model.materials.size(); ++m) {
        const u32 slot = model.materials[m].textureIndex;
        if (slot >= model.textureHashes.size()) continue;
        const u32 hash = model.textureHashes[slot];
        auto found = std::find(textures.begin(), textures.end(), hash);
        if (found == textures.end()) { textures.push_back(hash); found = textures.end() - 1; }
        textureOf[m] = (u32)(found - textures.begin());
    }
    if (textures.size() > 0xFF) { error = L"The model draws with more textures than Underground 2 can name."; return false; }

    const u32 triangles = model.TriangleCount();

    // --- header
    std::vector<u8> header(kHeaderU2, 0);
    std::memcpy(header.data(), source.data, 0x90);
    header[0x0C] = 0x16;                          // version
    header[0x0D] = 0;
    header[0x0E] = header[0x0F] = 0;              // object flags: Most Wanted's mean wind, mesh effects
    Put32(header, 0x14, triangles);
    Put32(header, 0x18, 0);
    header[0x19] = (u8)textures.size();
    Put32(header, 0x2C, 0);                       // run-time words between the bounds
    Put32(header, 0x3C, 0);
    Put32(header, 0x80, 0);
    Put32(header, 0x84, 0);
    Put32(header, 0x88, kHeaderConstantU2);
    Put32(header, 0x8C, kHeaderConstantU2);
    const Vec3 size = model.bounds.max - model.bounds.min;
    const float volume = size.x * size.y * size.z;
    PutF(header, 0x94, volume);
    PutF(header, 0x98, (float)triangles / volume);
    const size_t nameLength = std::min<size_t>(model.name.size(), kHeaderU2 - kNameU2 - 1);
    std::memcpy(header.data() + kNameU2, model.name.data(), nameLength);

    std::vector<u8> textureList(textures.size() * 8, 0);
    for (size_t t = 0; t < textures.size(); ++t) Put32(textureList, t * 8, textures[t]);

    // --- mesh
    std::vector<u8> meshHeader(kMeshHeaderU2, 0);
    Put32(meshHeader, 0x08, kMeshVersionU2);
    Put32(meshHeader, 0x10, (u32)model.materials.size());
    Put32(meshHeader, 0x24, triangles);
    Put32(meshHeader, 0x34, (u32)model.vertices.size());

    std::vector<u8> vertices(model.vertices.size() * sizeof(nfs::SolidVertex));
    std::memcpy(vertices.data(), model.vertices.data(), vertices.size());

    std::vector<u8> materials(model.materials.size() * kMaterialU2, 0);
    for (size_t m = 0; m < model.materials.size(); ++m) {
        const nfs::SolidMaterial& material = model.materials[m];
        const size_t at = m * kMaterialU2;
        PutVec3(materials, at + 0x00, material.bounds.min);
        Put32(materials, at + 0x0C, material.indexCount);
        PutVec3(materials, at + 0x10, material.bounds.max);
        Put32(materials, at + 0x1C, textureOf[m]);
        Put32(materials, at + 0x20, 0xFF);
        Put32(materials, at + 0x34, material.firstIndex);
        Put32(materials, at + 0x38, 0);           // plain WORLD
    }

    std::vector<u8> indices(model.indices.size() * 2);
    std::memcpy(indices.data(), model.indices.data(), indices.size());
    indices.resize((indices.size() + 3) & ~size_t(3), 0);

    Writer writer(out);
    const size_t root = writer.Begin(kObject);
    writer.Leaf(kHeader, header, 16);
    writer.Leaf(kTextures, textureList, 8);
    const size_t mesh = writer.Begin(kMesh);
    writer.Leaf(kMeshHeader, meshHeader, 16);
    writer.Leaf(kVertices, vertices, 0x80);
    writer.Leaf(kMaterials, materials, 16);
    writer.Leaf(kIndices, indices, 16);
    writer.End(mesh);
    writer.End(root);
    return true;
}

// ---------------------------------------------------------------------------

namespace {

bool SameVertex(const nfs::SolidVertex& a, const nfs::SolidVertex& b) {
    return std::memcmp(&a, &b, sizeof(nfs::SolidVertex)) == 0;
}

} // namespace

int RunMWSelfTest(const std::wstring& folder) {
    nfs::World world;
    if (!world.Open(folder + L"\\TRACKS", L"L2RA") || !world.IsMostWanted()) {
        LOG_ERROR("MW self-test: no Most Wanted L2RA in %S", folder.c_str());
        return 2;
    }
    world.LoadAllSections();

    size_t models = 0, good = 0, bad = 0, unreadable = 0;
    std::unordered_set<u32> seen, textures;
    for (const nfs::Section& section : world.Table().Sections()) {
        const ByteSpan bytes = world.SectionBytes(section);
        nfs::ChunkReader top(bytes, section.streamOffset);
        nfs::Chunk pack;
        while (top.Next(pack)) {
            if (pack.id != nfs::kChunkSolidListPack) continue;
            nfs::ChunkReader objects = nfs::ChunkReader::Children(pack);
            nfs::Chunk object;
            while (objects.Next(object)) {
                if (object.id != kObject) continue;
                const ByteSpan whole{ object.raw.data - 8, object.raw.size + 8 };
                nfs::SolidObject original;
                if (!nfs::ParseSolidObject(object, original, nfs::Game::MostWanted)) { ++unreadable; continue; }
                if (!seen.insert(original.hash).second) continue;
                ++models;

                std::vector<u8> converted;
                std::wstring error;
                if (!ConvertMWObject(whole, object.headerOffset, converted, error)) {
                    if (bad++ < 30) LOG_WARN("MW self-test %s: %S", original.name.c_str(), error.c_str());
                    continue;
                }
                nfs::ChunkReader again(ByteSpan{ converted.data(), converted.size() }, 0);
                nfs::Chunk chunk;
                nfs::SolidObject back;
                const bool parsed = again.Next(chunk) && nfs::ParseSolidObject(chunk, back) &&
                                    !again.Next(chunk);
                bool same = parsed && back.hash == original.hash && (back.version & 0xFF) == 0x16 &&
                            back.vertexStride == 36 && back.vertices.size() == original.vertices.size() &&
                            back.indices == original.indices && back.materials.size() == original.materials.size() &&
                            converted.size() % 4 == 0 &&
                            original.name.compare(0, back.name.size(), back.name) == 0;
                for (size_t v = 0; same && v < back.vertices.size(); ++v)
                    same = SameVertex(back.vertices[v], original.vertices[v]);
                for (size_t m = 0; same && m < back.materials.size(); ++m) {
                    const nfs::SolidMaterial& a = back.materials[m];
                    const nfs::SolidMaterial& b = original.materials[m];
                    same = a.firstIndex == b.firstIndex && a.indexCount == b.indexCount &&
                           a.textureIndex < back.textureHashes.size() && b.textureIndex < original.textureHashes.size() &&
                           back.textureHashes[a.textureIndex] == original.textureHashes[b.textureIndex];
                }
                if (same) {
                    ++good;
                    for (u32 hash : back.textureHashes) textures.insert(hash);
                } else if (bad++ < 30) {
                    LOG_WARN("MW self-test %s: %s (vertices %zu/%zu, triangles %u/%u, materials %zu/%zu, stride %u)",
                             original.name.c_str(), parsed ? "DIFFERENT" : "does not parse", back.vertices.size(),
                             original.vertices.size(), back.TriangleCount(), original.TriangleCount(),
                             back.materials.size(), original.materials.size(), back.vertexStride);
                }
            }
        }
    }

    size_t copied = 0, missing = 0, failed = 0;
    for (u32 hash : textures) {
        const nfs::Texture* texture = world.FindDisplayTexture(hash);
        if (!texture) { ++missing; continue; }
        NewTexture copy;
        if (CopyGameTexture(*texture, copy)) ++copied;
        else if (failed++ < 30)
            LOG_WARN("MW self-test texture %s (%ux%u, %s) cannot be copied", texture->name.c_str(), texture->width,
                     texture->height, nfs::ToString(texture->format));
    }

    LOG_INFO("MW self-test: %zu models, %zu converted and read back the same, %zu failed, %zu unreadable as MW; "
             "textures: %zu copied, %zu failed, %zu not in the region's or the global packs",
             models, good, bad, unreadable, copied, failed, missing);
    return bad == 0 && failed == 0 ? 0 : 1;
}
