#pragma once

#include "nfs/ChunkReader.h"
#include "nfs/Scenery.h"
#include "nfs/SolidList.h"

#include <optional>

namespace nfs {

// NFS World's solid list. The container is the same 0x80134000 family as
// Underground 2, but the material entry is 116 bytes and the mesh is split
// into effect runs, each with its own vertex layout (28/36/44 bytes). The
// parser widens every run to SolidVertex and rewrites the indices so the
// existing renderer draws it with one buffer.
bool NfswParseSolidList(const Chunk& pack, SolidList& out);

// Appends an ordinary World solid list at 128-byte stream alignment.
bool NfswAppendRoadModels(std::vector<u8>& section, const std::vector<SolidObject>& models,
                          std::string& error);

struct NfswMeshUpdate {
    std::string name;
    u32 hash = 0;
    std::vector<Vec3> positions;
    std::vector<u16> indices;
};

// Writes positions into the original effect buffers and widens the bounds.
// The caller works on an in-memory copy of the complete section.
bool NfswPatchSolidMeshes(std::vector<u8>& section, const std::vector<const NfswMeshUpdate*>& updates,
                          std::string& error);

// NFS World's scenery: 72-byte infos and 96-byte instances with float
// rotation matrices. `recordToInstance`, when given, receives this chunk's
// record index -> index into `out.instances` (or ~0u for a record dropped
// because its info index was out of range): the layout's 0x34108 table names
// records, the renderer names instances.
bool NfswParseScenery(const Chunk& sceneryChunk, Scenery& out,
                      std::vector<u32>* recordToInstance = nullptr);
Scenery NfswReadSectionScenery(ByteSpan section);
std::vector<size_t> NfswChangedPlacements(const Scenery& stock, const Scenery& current);

struct NfswSceneryChunkInfo {
    u32    number = 0;
    size_t infoCount = 0;
    size_t instanceCount = 0;
};

// `wholeChunk` includes the 0x80034100 header and sits at `fileOffset`.
bool NfswReadSceneryChunk(ByteSpan wholeChunk, u64 fileOffset, NfswSceneryChunkInfo& out);

// Replaces the transform and stored world bounds of existing 0x34103 records.
// Every chunk header, size and offset stays as it was: this is the equal-size
// writeback path. A patch is matched by (scenery number, record). The stored
// 0x3410E cull boxes are updated through 0x3410F, whose +0x10 names the
// instance; the boxes are grouped by LOD, not in instance order.
struct NfswSceneryPatch {
    u32  sceneryNumber = 0;   // the scenery chunk's own 0x34101 number
    u32  record = 0;          // record index inside that chunk
    Mat4 transform;
    Aabb bounds;
    bool expandCullTree = false;
};

bool NfswPatchScenery(ByteSpan section, const std::vector<NfswSceneryPatch>& patches,
                      std::vector<u8>& out, std::string& error);

// An existing record whose transform and stored bounds change. Update indices
// are the shipped record indices.
struct NfswSceneryUpdate {
    u32  record = 0;
    Mat4 transform;
    Aabb bounds;
};

// A new placement cloned from a shipped record. The rebuild copies the
// template's 96-byte instance, a 32-byte 0x3410D binding where applicable and every
// 0x3410E/0x3410F precull record that names it, then rewrites the transform,
// bounds, info index and GUID. It reuses a matching 72-byte info in the
// target chunk, or copies the template's info when absent.
struct NfswSceneryAppend {
    u32      sceneryNumber = 0;          // target chunk (its 0x34101 number)
    ByteSpan templateSection;            // whole BUN that holds the template
    u32      templateSceneryNumber = 0;  // chunk number in the template section
    u32      templateRecord = 0;
    u32      modelHash = 0;
    Mat4     transform;
    Aabb     bounds;
    // A standalone copy of a source with a verified non-event 0x34106
    // registration. The new placement has these flags and no registration.
    std::optional<u16> plainCopyFlags;
    std::string newModelName;
    u32 newModelHash = 0;
};

// Rewrites one 0x80034100 chunk: updates existing records, removes records
// and appends copies from their template sections. Update and removal indices
// are the shipped record indices; removals are applied before appends. Every
// byte outside the target scenery chunk is copied unchanged. Changed payloads
// are padded so the chunk grows or shrinks by a multiple of 16, keeping the
// chunks after it on their original alignment.
//
// A 0x34106-linked removal must be in registeredRemovals; its caller must also
// retire the layout entry and remove its spawn. A section whose size changes
// needs its table entry (layout +0x60/+0x68) patched.
bool NfswRebuildScenery(ByteSpan section, u32 sceneryNumber,
                        const std::vector<NfswSceneryUpdate>& updates,
                        const std::vector<u32>& removals,
                        const std::vector<NfswSceneryAppend>& appends,
                        std::vector<u8>& out, std::string& error,
                        const std::vector<u32>& registeredRemovals = {},
                        size_t* firstAppendedRecord = nullptr);

// One baked object track of NFS World's BBG animation (chunks 0x37220..0x37270,
// stored beside the scenery container). `guid` matches SceneryInstance::guid;
// each frame is the instance's full 4x4 row-major transform.
struct NfswObjectAnimation {
    u32   guid = 0;
    u32   section = 0;
    float speed = 1.0f;
    std::vector<Mat4> frames;
};

// Parses the animation chunks of one section, in file order. Chunks of other
// ids are ignored, so the whole section chunk list can be passed in.
void NfswParseAnimations(const std::vector<Chunk>& chunks,
                         std::vector<NfswObjectAnimation>& out);

} // namespace nfs
