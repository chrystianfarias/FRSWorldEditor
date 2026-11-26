#pragma once

#include "core/Math.h"
#include "core/Types.h"
#include "nfs/Collision.h"

#include <array>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace nfs {

struct SolidObject;

constexpr u32 kNfswCollisionSection = 25000;

// The two surface hashes the static collision is known to use. Strips trail
// their hash at the end of a 'ca' article; boundary records carry one at +12.
inline constexpr u32 kRoadSurface = 0x6CA26F9B;   // the A100 road strips
inline constexpr u32 kSolidSurface = 0x19DB2F1E;  // the F115 wall strips

struct NfswCollisionPlacement {
    const SolidObject* model = nullptr;
    std::vector<CollisionPolygon> polygons;
};

// Adds static faces for edited placements to Z0's collision grid. The original
// collision articles and grid references remain in place. An earlier set of
// generated faces in the input is replaced.
bool NfswBuildPlacementCollision(ByteSpan layout, ByteSpan section,
                                 const std::vector<NfswCollisionPlacement>& placements,
                                 const std::vector<CollisionPolygon>& walls,
                                 std::vector<u8>& out, std::string& error,
                                 const std::vector<const CollisionPack*>& editedPacks = {});

// Reads the editable walls, excluding the original game collision and the
// automatically generated scenery faces.
bool NfswReadCollisionWalls(ByteSpan section, std::vector<CollisionPolygon>& walls);
bool NfswReadNativeCollision(ByteSpan section, std::vector<CollisionPack>& packs);
// Every surface hash the section's native packs use, shared instances included.
void NfswSurveyNativeSurfaces(ByteSpan section, std::map<u32, size_t>& strips,
                              std::map<u32, size_t>& boundaries);
// Saved generated road faces, excluding the editable faces already in memory.
void NfswReadGeneratedRoadGround(ByteSpan section, CollisionPack& out);

struct NfswNativeRoads {
    u64 network = 0;
    std::vector<std::array<Vec3, 2>> segments;
};
bool NfswReadNativeRoads(ByteSpan layout, NfswNativeRoads& roads);
struct NfswRoadEnd {
    Vec3 position, heading;
    int forward = 0, backward = 0;
    float laneWidth = 0, medianWidth = 0;
    float leftShoulder = 0, rightShoulder = 0;
    int nativeProfile = -1;
};
// Finds a node on the selected mesh whose stock lane layout can be joined by an authored road.
std::optional<NfswRoadEnd> NfswFindRoadEnd(ByteSpan layout, const Vec3& point, const Aabb& roadBounds,
                                          const Vec3& toward);

struct NfswRoadSegmentBuild {
    size_t a, b;                         // shared authored node indices
    std::array<Vec3, 4> cubic;            // editor-space control points
    int forward = 1, backward = 1;
    float width = 4.0f, median = 0.0f;
    int nativeProfile = -1;
};

struct NfswRoadBuildReport {
    size_t nodes = 0, segments = 0, stockJoins = 0;
};

// Extends the native network while preserving the shipped node/segment IDs.
bool NfswAppendRoadGraph(std::vector<u8>& layout, const std::vector<NfswRoadSegmentBuild>& segments,
                          NfswRoadBuildReport& report, std::string& error);


struct NfswSmackableBounds {
    Vec3 center, halfSize;
    std::array<float, 4> rotation;
    bool singleBox = false;
};

using NfswSmackableTypes = std::unordered_map<u32, NfswSmackableBounds>;
using NfswSmackableBox = std::array<Vec3, 8>;
using NfswSmackableHalfSizes = std::array<i16, 3>; // local X,Y,Z, in millimetres
inline constexpr u32 kSmackableGeometryList = 0x8003B900;
inline constexpr u32 kSmackableGeometry = 0x3B901;
inline constexpr size_t kSmackableAttributeOffset = 0x24;

struct NfswSmackableLink {
    u32 entry = 0;
    u32 geometry = 0;
    u32 physics = 0;
    std::array<u8, 64> spawn{}; // original bytes, used to locate and patch this spawn on save
    size_t box = SIZE_MAX; // collision type may have no displayable bound
    bool singleBox = false;
    std::optional<NfswSmackableHalfSizes> halfSizes;

    NfswSmackableBox EditedBox(const NfswSmackableBox& original) const;

    bool BindingChanged() const {
        return physics != ByteSpan{ spawn.data(), spawn.size() }.read<u32>(kSmackableAttributeOffset);
    }
};

struct NfswSmackablePlacement {
    NfswSmackableLink link;
    NfswSmackableBox box;
    Mat4 sourceTransform;
    u16 flags = 0;

    NfswSmackableBox BoxAt(const Mat4& transform) const;
};

void NfswReadSmackableTypes(ByteSpan layout, NfswSmackableTypes& types);
bool NfswIsSingleSmackableBox(ByteSpan geometry);
void NfswReadSmackableBoxes(const Chunk& chunk, const NfswSmackableTypes& types,
                          std::vector<NfswSmackableBox>& boxes, std::vector<NfswSmackableLink>& links);

} // namespace nfs
