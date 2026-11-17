#pragma once

#include "core/FileMap.h"
#include "core/Math.h"
#include "io/MeshOverlay.h"
#include "nfs/WorldAnimation.h"
#include "nfs/Collision.h"
#include "nfs/Preculler.h"
#include "nfs/SectionTable.h"
#include "nfs/Scenery.h"
#include "nfs/SceneryEdit.h"
#include "nfs/SolidList.h"
#include "nfs/TexturePack.h"

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>

namespace nfs {

// Fingerprint of "this mesh, drawn here". Used to make sure a model that is
// stored twice, or placed both by its own header and by a scenery instance,
// still ends up on screen exactly once.
u64 PlacementKey(u32 modelHash, const Mat4& transform);

// The parsed contents of one streaming section.
struct LoadedSection {
    const Section*          info = nullptr;
    std::vector<SolidList>  solidLists;
    std::vector<std::unique_ptr<TexturePack>> texturePacks;
    Scenery                 scenery;
    std::vector<CollisionPack> collision;

    // Scenery the editor placed, kept apart from the shipped instances so
    // those stay exactly as they were and a save knows what to append. A
    // section read back from an earlier save has its extra records moved here.
    struct UserPlacement {
        u32            sceneryNumber = 0;   // which of the section's scenery chunks
        PlacedInstance instance;
    };
    std::vector<UserPlacement> placements;
    std::vector<u32> sceneryNumbers;        // each scenery chunk's number, in file order
    bool placementsChanged = false;         // differs from what is on disk

    // Shipped scenery records marked for removal and not written yet:
    // (scenery number, record index). Written ones come back as records with
    // their matrix zeroed.
    std::vector<std::pair<u32, u32>> removedNatives;

    // An empty region (World::EmptyCity): a section written earlier that still
    // carries some of the source's city on disk -- shipped objects not zeroed,
    // a collision pack as shipped. Saving it again takes the city out.
    bool emptyPending = false;

    // Written by an older version with a broken object list (World.cpp,
    // BrokenObjectTable): the game crashes on it until it is saved again.
    bool brokenObjectList = false;

    // Collision edited in the editor and not written yet: the packs (indices
    // into `collision`) whose polygons differ from what is on disk.
    bool collisionChanged = false;
    std::vector<u32> changedCollisionPacks;
    void MarkCollisionChanged(u32 pack) {
        collisionChanged = true;
        for (u32 p : changedCollisionPacks) if (p == pack) return;
        changedCollisionPacks.push_back(pack);
    }

    // Models from other positioned sections to be copied into this one when
    // the placements are written, so they can be placed here. Hashes.
    std::vector<u32> importedModels;

    bool HoldsModel(u32 hash) const;
    Aabb                    bounds;        // everything, backdrop included
    Aabb                    trackBounds;   // drivable world only
    // Bounds of the terrain alone. Props are frequently shared between
    // neighbouring sections and can sit well outside the footprint of the
    // section that stores them, so the terrain is what marks where it lies.
    Aabb                    terrainBounds;

    // The box the camera should frame when this section is selected.
    const Aabb& FocusBounds() const;

    size_t ObjectCount() const;
    size_t TriangleCount() const;
};

// A texture that cycles through frames: water, flashing signs, blinking
// lights. Materials name the first frame's hash; the game swaps in the frame
// of the moment (chunk 0xB0300100, beside a section's texture pack).
struct TextureAnimation {
    std::string      name;
    std::vector<u32> frames;     // texture hashes, the first is the one materials use
    float            fps = 0.0f;
};

// The games whose worlds the editor reads.
enum class Game { Underground2, Underground1 };

// One track region: the L4Rx.BUN layout file plus its STREAML4Rx.BUN payload.
//
// Underground 1 has no layout file per region: each track has a bundle,
// TRACKS\TRACKB<n>.LZC (JDLZ), holding the section table, the visibility
// zones and the whole track's collision, over a stream shared by every track
// of the region. The region opens with one of those bundles in the layout's
// place (see OpenUnderground1).
//
// Sections are parsed on demand. Because the stream stays memory-mapped, a
// section that is never visited costs nothing beyond its table entry.
class World {
public:
    // Region identifiers as they appear in the file names: "L4RA", "L4RB", ...
    static std::vector<std::wstring> AvailableRegions(const std::wstring& tracksDirectory);

    // Where the edited meshes live, usually <game>\models. Set it before
    // opening: overlays are applied as each section is parsed, so the editor
    // shows the same geometry the game will be handed.
    void SetOverlayFolder(const std::wstring& folder) { overlayFolder_ = folder; }
    const std::wstring& OverlayFolder() const { return overlayFolder_; }

    size_t OverlayCount() const { return overlays_.size(); }

    // Opens <tracksDirectory>\<region>.BUN and STREAM<region>.BUN.
    bool Open(const std::wstring& tracksDirectory, const std::wstring& region);
    void Close();

    bool IsOpen() const { return stream_.IsOpen(); }

    Game GameKind() const { return game_; }
    bool IsUnderground1() const { return game_ == Game::Underground1; }
    // An empty region the editor made (nfs/CustomRegions.h): its source's city
    // hidden, and taken out of every section it saves.
    bool EmptyCity() const { return emptyCity_; }
    // Underground 1: the track whose bundle the region was opened with.
    u32  TrackNumber() const { return trackNumber_; }
    // Underground 1 has collision only along each race's course, and each
    // track's bundle has its own: the ground under (x, y), no higher than
    // `ceiling`, on the open track if it has some there, else on any other
    // track of the region -- and which track that is.
    bool Underground1GroundBelow(float x, float y, float ceiling, float& height, u32& track) const;
    // The region's tracks (their bundles name this stream), the open one first.
    std::vector<u32> Underground1Tracks() const;
    // Every track the region lists (a STREAM<region>_<n>.BUN part each), and
    // the open one: the bundles a table correction has to reach.
    const std::vector<u32>& Underground1AllTracks() const { return allTracks_; }
    // Underground 1's track path zones (TRACKB chunk 0x3414A, 0xB4 bytes each:
    // type +0, the point a reset puts the car on +4, its facing +0x0C, height
    // +0x14, direction +0x18 (0xFF both), course distance +0x1C, the stretch of
    // course covered +0x20/+0x24, box +0x28, point count +0x38, outline +0x3C).
    // Types as the game names them: 2 RESET, 3 RESET_TO_POINT, 4 GUIDED_RESET,
    // 5 TUNNEL... A RESET zone puts a car that enters it back on the course --
    // the "invisible walls" of the races. Every track's, each zone once, with
    // the tracks that have it.
    struct Underground1Zone {
        u32 type = 0;
        std::vector<Vec3> ring;      // z = 0
        Vec3 resetPoint, facing;
        float height = 0.0f;
        std::vector<u32> tracks;
    };
    const std::vector<Underground1Zone>& Underground1Zones() const { return zones1_; }

    // The open track's collision as it shipped, before the editor's edit: what
    // a save compares the sections' collision with (io/U1Collision.h).
    const std::vector<CollisionPolygon>& ShippedTrackCollision() const { return shippedTrackPolygons_; }
    // The open track's collision as the editor holds it now: every loaded
    // section's packs, and the packs of sections it did not load as split.
    std::vector<CollisionPolygon> CurrentTrackCollision() const;

    const std::wstring& Region() const { return region_; }
    const SectionTable& Table()  const { return table_; }

    // The mapped stream, for anything that has to work in file bytes rather
    // than in parsed geometry -- rebuilding a section, for one.
    ByteSpan StreamBytes() const { return stream_.Span(); }

    // A section's bytes as the editor reads them: the rebuilt span when an
    // earlier import replaced the section, the stream file otherwise. Building
    // a new edit on these keeps the edits already in force.
    ByteSpan SectionBytes(const Section& section) const;
    const Preculler&    Visibility() const { return preculler_; }
    // The layout file (L4Rx.BUN) as shipped, and the spans the models folder
    // lays over it and the stream.
    ByteSpan LayoutBytes() const { return LayoutSpan(); }
    const std::vector<SpanOverlay>& Spans() const { return spans_; }

    // Parses a section if it has not been parsed yet. Returns nullptr when the
    // section holds no geometry.
    const LoadedSection* LoadSection(const Section& section);

    // Parses every section. Reports progress through the optional callback,
    // which receives (sectionsDone, sectionsTotal).
    void LoadAllSections(void (*progress)(size_t, size_t, void*) = nullptr,
                         void* userData = nullptr);

    const std::vector<std::unique_ptr<LoadedSection>>& Loaded() const { return loaded_; }

    // Bounds of everything parsed so far, including the distant backdrop.
    Aabb Bounds() const { return bounds_; }

    // Bounds of the drivable world only.
    //
    // The first sections of a region hold shared assets and the painted
    // backdrop, which stretch tens of thousands of units in every direction and
    // would swamp any camera framed on them. Those sections carry no world
    // position in the table, so positioned sections alone give the real extent.
    Aabb TrackBounds() const;

    // The box the camera should frame for one section.
    Aabb SectionBounds(const LoadedSection& section) const;

    // Looks a texture up across every pack loaded so far.
    const Texture* FindTexture(u32 hash) const;

    // The same, falling back to the packs the game keeps loaded for every
    // track (GLOBAL\InGameCommon.bun, GLOBAL\GLOBALB.BUN). The race barriers,
    // their tape and neon, and a few effect sprites draw from those. Only for
    // looking at: a texture found there is not the region's to copy or export.
    const Texture* FindDisplayTexture(u32 hash) const;

    // Every distinct texture in the region, deduplicated by hash. A region
    // repeats textures across its packs, so the raw packs are not a useful list.
    std::vector<const Texture*> AllTextures() const;

    // How many material slots draw with this texture.
    u32 TextureUsage(u32 hash) const;

    // Looks a model up by hash across every section loaded so far. Scenery
    // instances reference models that live in a different section, usually the
    // shared library the region loads first. A hash the region does not
    // hold is asked of the models from outside it (nfs/ForeignModels.h).
    const SolidObject* FindModel(u32 hash) const;

    // Whether lookups fall back to the models from outside the region. Off
    // for the worlds that outside source opens itself.
    void SetForeignLookup(bool on) { foreignLookup_ = on; }

    size_t InstanceCount() const { return instanceCount_; }

    // Every texture animation of the sections loaded, by the hash materials use.
    const std::unordered_map<u32, TextureAnimation>& TextureAnimations() const { return textureAnimations_; }

    // The moving scenery of the layout file: doors, planes, cranes, trams.
    const WorldAnimations& ObjectAnimations() const { return objectAnimations_; }
    // Moving scenery shown but not part of the region (WorldAnimations::SetExtraPlacements).
    void SetPreviewAnimations(std::vector<WorldAnimationPlacement> extra) {
        objectAnimations_.SetExtraPlacements(std::move(extra));
    }

    // The race barrier groups the layout file names (chunk 0x3412F): byte 0
    // of a collision polygon indexes this list, entry 0 being "NONE". A group
    // exists for the game only during the races that name it -- in free roam
    // its polygons are not there at all.
    const std::vector<std::string>& RaceBarrierGroups() const { return raceGroups_; }
    std::string RaceBarrierGroupName(u8 group) const;

    // --- placing scenery -----------------------------------------------------

    LoadedSection* MutableLoaded(size_t index) {
        return index < loaded_.size() ? loaded_[index].get() : nullptr;
    }

    // The loaded section a model is stored in, or SIZE_MAX.
    size_t ModelHome(u32 hash) const;

    // Sections without a world position hold the region's shared library,
    // which the game keeps loaded everywhere. Only a model stored there can be
    // placed anywhere: no shipped instance refers to a model that lives in a
    // different positioned section, which would only exist while that section
    // happened to be streamed in.
    bool IsLibrary(const LoadedSection& section) const;

    // A section of the second set (Section::IsAlternate) that no visibility
    // list of the game loads: a true alternative, hidden unless asked for.
    // Without visibility lists, the second set as a whole.
    bool IsUnusedAlternate(const Section& section) const {
        return section.IsAlternate() && (preculler_.IsEmpty() || !preculler_.Listed(section.number));
    }

    // Loaded sections (indices into Loaded()) that every point's visibility
    // zone lists: what the game has loaded wherever along these points the
    // car is. A model or collision placed in one of them never vanishes on
    // the way. Empty when the points cross zones that share no section, or
    // when the region has no visibility lists.
    std::vector<size_t> SectionsLoadedThroughout(const std::vector<Vec3>& points) const;

    // The positioned section a placement at `point` belongs in: the one whose
    // ground covers it, else the nearest. SIZE_MAX when none can take scenery.
    size_t SectionForPlacement(const Vec3& point) const;

    // The section for a model or placement spanning `points`: one that can
    // take scenery and that every point's visibility zone loads, so it never
    // vanishes along them -- SectionForPlacement's choice for their middle
    // when it is one of those, else the nearest of them. Without visibility
    // lists, SectionForPlacement. SIZE_MAX when no section qualifies.
    size_t SectionForPlacementAlong(const std::vector<Vec3>& points) const;

    // Every model in the region, one per hash.
    std::vector<const SolidObject*> AllModels() const;

private:
    std::wstring region_;
    bool emptyCity_ = false;
    bool foreignLookup_ = true;
    FileMap      layout_;   // L4Rx.BUN
    std::vector<u8> layoutUnpacked_;   // Underground 1: the unpacked TRACKB<n>.LZC
    ByteSpan LayoutSpan() const {
        return layoutUnpacked_.empty() ? layout_.Span() : ByteSpan{ layoutUnpacked_.data(), layoutUnpacked_.size() };
    }
    Game         game_ = Game::Underground2;
    u32          trackNumber_ = 0;
    bool OpenUnderground1(const std::wstring& tracksDirectory, const std::wstring& region);
    // Underground 1's collision, one pack per drivable section (by zone).
    std::unordered_map<u32, CollisionPack> trackCollision_;
    void SplitTrackCollision(const std::wstring& region);
    std::vector<CollisionPolygon> shippedTrackPolygons_;   // the open track's own, before any edit
    struct TrackCollision {
        u32 track = 0;
        std::vector<CollisionPolygon> polygons;
        Aabb bounds;
    };
    std::vector<TrackCollision> regionTracks_;   // the open track first
    std::vector<u32> allTracks_;
    std::vector<Underground1Zone> zones1_;
    void ReadUnderground1Zones(u32 track, ByteSpan bundle);
    void NameUnderground1Scenery(LoadedSection& section) const;
    FileMap      stream_;   // STREAML4Rx.BUN
    std::vector<std::unique_ptr<FileMap>>     globalFiles_;
    std::vector<std::unique_ptr<TexturePack>> globalPacks_;
    void OpenGlobalPacks(const std::wstring& tracksDirectory);
    SectionTable table_;
    std::vector<std::string> raceGroups_;
    std::unordered_map<u32, TextureAnimation> textureAnimations_;
    WorldAnimations objectAnimations_;
    void ReadTextureAnimations(const Chunk& chunk);
    void ReadRaceBarrierGroups();

    std::wstring overlayFolder_;
    std::unordered_map<u32, std::vector<Vec3>> overlays_;   // mesh hash -> positions

    // Rebuilt sections and the table corrections that go with them. A mesh that
    // gained or lost vertices cannot be patched a vertex at a time, so its whole
    // section arrives as one span -- and the editor has to read it the same way
    // the game will, or the viewport shows the shipped geometry while the game
    // shows the edited one.
    std::vector<SpanOverlay> spans_;
    Preculler    preculler_;

    std::vector<std::unique_ptr<LoadedSection>> loaded_;
    std::unordered_map<u32, const SolidObject*>  modelsByHash_;
    std::unordered_map<u32, size_t>              modelHome_;
    std::unordered_set<u32>                      instancedHashes_;
    std::unordered_set<u64>                      placements_;
    size_t                                       instanceCount_ = 0;
    std::unordered_map<u32, u32>                 textureUsage_;
    std::vector<int>                            sectionToLoaded_;  // -1 when not parsed
    Aabb                                        bounds_;

    // Half the typical distance between neighbouring section centres.
    float SectionPadding() const;
};

} // namespace nfs
