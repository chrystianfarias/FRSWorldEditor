#pragma once

#include "core/FileMap.h"
#include "core/Math.h"
#include "io/MeshOverlay.h"
#include "nfs/WorldAnimation.h"
#include "nfs/Collision.h"
#include "nfs/NfswCollision.h"
#include "nfs/NfswFormats.h"
#include "nfs/NfswSmackableAttributes.h"
#include "nfs/Preculler.h"
#include "nfs/SectionTable.h"
#include "nfs/Scenery.h"
#include "nfs/SceneryEdit.h"
#include "nfs/SolidList.h"
#include "nfs/TexturePack.h"

#include <memory>
#include <optional>
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
    std::vector<NfswSmackableBox> smackableBoxes;
    std::vector<NfswSmackableLink> smackableLinks;
    std::unordered_map<size_t, size_t> smackableScenery;  // scenery index -> spawn link index
    std::vector<size_t> nfswEditedScenery;    // saved transforms/additions that need placement collision

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
    std::vector<u32> frames;     // texture hashes, in cycle order
    float            fps = 0.0f;
    u8               startFrame = 0;   // material names this frame; cycle from here
};

// The games whose worlds the editor reads.
enum class Game { Underground2, Underground1, World };

// NFS World's server-driven scenery groups. Normal hides every holiday's own
// props, the Top Gear event billboards and the track barrier groups; the
// others show that group and hide its "disable" group. All shows everything.
enum class Holiday { Normal, Halloween, Oktoberfest, Christmas, Valentines, NewYears,
                     TopGear, All };

// One track region: the L4Rx.BUN layout file plus its STREAML4Rx.BUN payload.
//
// Underground 1 has no layout file per region: each track has a bundle,
// TRACKS\TRACKB<n>.LZC (JDLZ), holding the section table, the visibility
// zones and the whole track's collision, over a stream shared by every track
// of the region. The region opens with one of those bundles in the layout's
// place (see OpenUnderground1).
//
// NFS World has the layout file but no single stream: every section is its
// own STREAM<region>_<number>.BUN, read whole on first use (see OpenNfsw).
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

    bool IsOpen() const {
        return IsWorld() ? (layout_.IsOpen() || !layoutUnpacked_.empty())
                         : (stream_.IsOpen() || !layoutUnpacked_.empty());
    }

    Game GameKind() const { return game_; }
    bool IsUnderground1() const { return game_ == Game::Underground1; }
    bool IsWorld() const { return game_ == Game::World; }
    // An empty region the editor made (nfs/CustomRegions.h): its source's city
    // hidden, and taken out of every section it saves.
    bool EmptyCity() const { return emptyCity_; }
    // Underground 1: the track whose bundle the region was opened with.
    u32  TrackNumber() const { return trackNumber_; }

    const std::wstring& Region() const { return region_; }
    const std::wstring& TracksDirectory() const { return tracksDirectory_; }
    const SectionTable& Table()  const { return table_; }

    // NFS World's section BUN file name, e.g. STREAML5RA_436.BUN.
    std::wstring NfswSectionFileName(const Section& section) const;

    // The mapped stream, for anything that has to work in file bytes rather
    // than in parsed geometry -- rebuilding a section, for one.
    ByteSpan StreamBytes() const { return stream_.Span(); }

    // A section's bytes as the editor reads them: the rebuilt span when an
    // earlier import replaced the section, the stream file otherwise. Building
    // a new edit on these keeps the edits already in force.
    ByteSpan SectionBytes(const Section& section) const;
    // Drops the cached section files so the next read sees a fresh overlay.
    void ForgetSectionBytes() { sectionFiles_.assign(table_.Sections().size(), {}); }
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

    // World equivalents of the procedural road textures. Zero leaves the texture unchanged.
    u32 NativeRoadTexture(const std::string& texture) const;

    // A point on a road surface suitable for placing the mesh, including geometry stored in map coordinates.
    std::optional<Vec3> RoadPlacementAnchor(const PlacedInstance& placement) const;

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

    // NFS World's baked scenery animation, by scenery instance GUID.
    const std::unordered_map<u32, NfswObjectAnimation>& SceneryAnimations() const {
        return sceneryAnimations_;
    }

    // NFS World holiday/event scenery (server's SCENERY_GROUP_* lists).
    // Selecting a group set only changes the visibility filter; the renderer
    // asks HolidayHides() for each shipped instance.
    void SetHoliday(Holiday holiday);
    Holiday CurrentHoliday() const { return holiday_; }
    bool HolidayHides(u32 sectionNumber, int instance) const;
    size_t SceneryGroupCount() const { return sceneryGroups_.size(); }
    size_t SceneryOverrideCount() const;
    // Some 0x34106 records register an ordinary placement rather than
    // attaching it to an event. Return the registered instance's low flags.
    bool PlainSceneryFlags(u16 entry, u32 chunk, u32 record, u16& flags) const;
    std::optional<NfswSmackablePlacement> SmackablePlacement(const LoadedSection& section, size_t index) const;
    const NfswSmackableLink* FindSmackableLink(const LoadedSection& section, size_t index) const;
    bool RebindSmackable(LoadedSection& section, size_t index, u32 key) const;
    bool RebindSmackable(LoadedSection& section, PlacedInstance& instance, u32 key);
    bool ResizeSmackableBox(LoadedSection& section, size_t index, const NfswSmackableHalfSizes& sizes);
    bool ResizeSmackableBox(LoadedSection& section, PlacedInstance& instance, const NfswSmackableHalfSizes& sizes);
    struct SmackableState {
        LoadedSection* section = nullptr;
        size_t index = 0; // placement index or native link index
        bool placement = false;
        bool placementsChanged = false;
        u32 physics = 0;
        std::optional<NfswSmackableHalfSizes> halfSizes;

        bool Changed() const;
        void Restore() const;

    private:
        friend class World;
        NfswSmackableLink* Link() const;
    };
    std::vector<SmackableState> CaptureSmackable(LoadedSection& section, size_t index, bool placement);
    const NfswSmackableAttributes* FindSmackableAttributes(u32 key) const;
    bool ReloadSmackableAttributes();
    std::wstring NfswAttributesFile(bool* overlay = nullptr) const;
    bool UsesSmackableAttributeOverlay() const { return smackableAttributeOverlay_; }
    size_t HolidayHiddenCount() const { return holidayHidden_.size(); }
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
    // Without visibility lists, the second set as a whole. NFS World marks no
    // section IsAlternate: its +400 siblings are additional layers.
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
    std::wstring tracksDirectory_;
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
    bool OpenNfsw(const std::wstring& tracksDirectory, const std::wstring& region);
    std::pair<LoadedSection*, size_t> SmackableSource(const PlacedInstance& instance);
    // NFS World: bytes of each section file, read on first use. mutable because
    // SectionBytes and the const lookups that reach it load lazily.
    mutable std::vector<std::vector<u8>> sectionFiles_;
    NfswSmackableTypes smackableTypes_;
    NfswSmackableCatalog smackableAttributes_;
    bool smackableAttributeOverlay_ = false;
    std::wstring NfswSectionPath(size_t index) const;
    // Underground 1's collision, one pack per drivable section (by zone).
    std::unordered_map<u32, CollisionPack> trackCollision_;
    void SplitTrackCollision();
    void NameUnderground1Scenery(LoadedSection& section) const;
    FileMap      stream_;   // STREAML4Rx.BUN
    std::vector<std::unique_ptr<FileMap>>     globalFiles_;
    std::vector<std::vector<u8>>              globalData_;   // decompressed .lzc payloads
    std::vector<std::unique_ptr<TexturePack>> globalPacks_;
    void OpenGlobalPacks(const std::wstring& tracksDirectory);
    SectionTable table_;
    std::vector<std::string> raceGroups_;
    std::unordered_map<u32, TextureAnimation> textureAnimations_;
    // NFS World layout 0x34109: group hash -> 0x34108 table entries.
    struct SceneryGroup {
        u16              id = 0;
        std::vector<u16> entries;
    };
    std::unordered_map<u32, SceneryGroup> sceneryGroups_;
    // The 0x34108 table, dense by entry: which scenery chunk's which record
    // the entry names, with the entry's own flags. This is the complete
    // source -- the per-section 0x34106 lists only cache part of it.
    struct SceneryEntry {
        u32 chunk = 0;     // the scenery chunk's own 0x34101 number
        u16 instance = 0;  // record index inside that chunk
        u16 flags = 0;
    };
    std::vector<SceneryEntry> sceneryEntries_;
    // One scenery chunk's records as the renderer sees them: the file section
    // the chunk lives in and each record's index into that section's
    // Scenery::instances (~0u when the record was dropped).
    struct ChunkInstances {
        u32 section = 0;
        std::vector<u32> recordToInstance;
    };
    std::unordered_map<u32, ChunkInstances> chunkInstances_;
    std::unordered_set<u64> holidayHidden_;
    Holiday holiday_ = Holiday::Normal;
    void ReadSceneryGroups(ByteSpan layout);
    // Entry flags 0x10 marks a group-driven placement (DisableAllSceneryGroups
    // sets 0x18 on every member when it turns the groups off, and every
    // holiday / Top Gear / barrier entry carries it; the 22,705-entry
    // bookkeeping group does not).
    void HideEntry(size_t entry);
    void HideSceneryGroup(u32 hash, bool hide);
    WorldAnimations objectAnimations_;
    std::unordered_map<u32, NfswObjectAnimation> sceneryAnimations_;
    void ReadTextureAnimations(const Chunk& chunk);
    // NFS World's 0xB3312004 frame animations, keyed by their first frame.
    void RegisterNfswAnimations(const TexturePack& pack);
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
