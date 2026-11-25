#include "io/NfswPlacementWrite.h"
#include "io/CollisionGenerate.h"
#include "io/NfswSmackableWrite.h"

#include "core/Log.h"
#include "nfs/NfswCollision.h"
#include "nfs/NfswFormats.h"
#include "nfs/World.h"

#include <windows.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <map>
#include <set>
#include <sstream>
#include <tuple>
#include <utility>
#include <vector>

namespace {

bool ReadWhole(const std::wstring& path, std::vector<u8>& bytes) {
    FILE* file = nullptr;
    if (::_wfopen_s(&file, path.c_str(), L"rb") != 0 || !file) return false;
    std::fseek(file, 0, SEEK_END);
    const long size = std::ftell(file);
    std::fseek(file, 0, SEEK_SET);
    if (size <= 0) { std::fclose(file); return false; }
    bytes.resize((size_t)size);
    const bool ok = std::fread(bytes.data(), 1, bytes.size(), file) == bytes.size();
    std::fclose(file);
    return ok;
}

bool WriteWhole(const std::wstring& path, const std::vector<u8>& bytes) {
    FILE* file = nullptr;
    if (::_wfopen_s(&file, path.c_str(), L"wb") != 0 || !file) return false;
    const bool ok = std::fwrite(bytes.data(), 1, bytes.size(), file) == bytes.size();
    return std::fclose(file) == 0 && ok;
}

void CreateOverlayFolders(const std::wstring& modelsFolder) {
    for (const wchar_t* part : { L"", L"\\files", L"\\files\\TRACKS" })
        ::CreateDirectoryW((modelsFolder + part).c_str(), nullptr);
}

struct SectionPlan {
    std::vector<nfs::NfswSceneryUpdate> updates;
    std::vector<u32>                    removals;
    std::vector<u32>                    registeredRemovals;
    std::vector<nfs::NfswSceneryAppend> appends;
    std::map<size_t, const nfs::NfswSmackablePlacement*> smackableAppends;
};

struct Overlay {
    const nfs::Section* section = nullptr;
    std::wstring        target;
    std::vector<u8>     bytes;
    bool                writeFile = true;
    NfswSmackableWrite::Copies smackableCopies;
};

bool PatchLayoutSizes(const nfs::World& world, const std::vector<Overlay>& overlays,
                      std::vector<u8>& layout, std::wostringstream& log) {
    std::vector<u8> stockLayout;
    nfs::SectionTable stockTable;
    for (const Overlay& overlay : overlays) {
        const nfs::Section& section = *overlay.section;
        const ByteSpan data{ overlay.bytes.data(), overlay.bytes.size() };
        const size_t entry = (size_t)section.tableEntryOffset;
        const ByteSpan table{ layout.data(), layout.size() };
        const u32 parts = table.read<u32>(entry + 0x5C);
        if (!entry || parts < 1 || parts > 5 || entry + 0x60 + parts * 12 > layout.size())
            return false;

        if (parts == 1) {
            const u32 size = (u32)data.size;
            if (size == section.fileSize) continue;
            for (size_t field = 0x60; field <= 0x68; field += 4)
                std::memcpy(layout.data() + entry + field, &size, sizeof(size));
            continue;
        }

        // Each triple describes a cumulative stream endpoint and the base
        // allocation. Collision grows the first part, ahead of the four
        // streamed texture chunks. Rebase on shipped endpoints to repair
        // layouts written by the older single-part writer.
        if (stockLayout.empty()) {
            const auto path = world.TracksDirectory() + L"\\" + world.Region() + L".BUN";
            if (!ReadWhole(path, stockLayout) ||
                !stockTable.ParseNfsw({ stockLayout.data(), stockLayout.size() })) return false;
        }
        const nfs::Section* original = stockTable.FindByNumber(section.number);
        if (!original) return false;
        const ByteSpan stock{ stockLayout.data(), stockLayout.size() };
        const size_t first = (size_t)original->tableEntryOffset;
        if (stock.read<u32>(first + 0x5C) != parts) return false;
        const u32 stockSize = stock.read<u32>(first + 0x60 + (parts - 1) * 12);
        const i64 delta = (i64)data.size - stockSize;
        if (!stockSize || delta % 128) return false;
        for (size_t part = 0; part < parts; ++part) {
            const size_t field = 0x60 + part * 12;
            const i64 endpoint = (i64)stock.read<u32>(first + field) + delta;
            if (endpoint <= 0 || endpoint > (i64)data.size) return false;
            if (part + 1 < parts) {
                const size_t texture = ((size_t)endpoint + 127) & ~size_t(127);
                const u32 bytes = data.read<u32>(texture + 4);
                if (data.read<u32>(texture) != 0xB3300011 ||
                    texture + 8 + bytes != (size_t)((i64)stock.read<u32>(first + field + 12) + delta)) {
                    log << L"  " << section.name.c_str() << L": streamed texture ranges do not match the layout.\n";
                    return false;
                }
            }
            for (size_t word = 0; word < 12; word += 4) {
                const i64 adjusted = (i64)stock.read<u32>(first + field + word) + delta;
                if (adjusted <= 0 || adjusted > UINT32_MAX) return false;
                const u32 value = (u32)adjusted;
                std::memcpy(layout.data() + entry + field + word, &value, sizeof(value));
            }
        }
    }
    return true;
}

void CollectCollisionPlacements(const nfs::World& world, const nfs::Scenery& updated,
                                const std::vector<size_t>& changed, const std::set<size_t>& dynamic,
                                std::vector<nfs::NfswCollisionPlacement>& placements) {
    for (size_t index : changed) {
        if (dynamic.count(index)) continue;
        const auto& instance = updated.instances[index];
        const nfs::SolidObject* model = world.FindModel(updated.ModelHash(instance));
        if (!model) {
            const auto& info = updated.infos[instance.infoIndex].raw;
            for (size_t at = 0x1C; at < 0x28 && !model; at += 4)
                model = world.FindModel(ByteSpan{ info.data(), info.size() }.read<u32>(at));
        }
        if (!model || model->name.rfind("FRS_ROAD_", 0) == 0) continue;
        nfs::NfswCollisionPlacement placement{ model, {} };
        collision::GenerateNfswPlacementCollision(*model, instance.transform, placement.polygons);
        if (!placement.polygons.empty()) placements.push_back(std::move(placement));
    }
}

constexpr char kRoadCollisionMagic[] = "FRSROAD1";
constexpr size_t kRoadFaceBytes = sizeof(Vec3) * 4 + 4;

std::vector<u8> EncodeRoadCollision(const std::vector<nfs::NfswCollisionPlacement>& roads) {
    std::vector<u8> data(kRoadCollisionMagic, kRoadCollisionMagic + 8);
    for (const auto& piece : roads) {
        const u32 count = (u32)piece.polygons.size();
        const u32 wall = piece.model ? 1 : 0;
        const size_t header = data.size();
        data.resize(header + 8 + count * kRoadFaceBytes);
        std::memcpy(data.data() + header, &count, 4);
        std::memcpy(data.data() + header + 4, &wall, 4);
        size_t at = header + 8;
        for (const auto& polygon : piece.polygons) {
            std::memcpy(data.data() + at, polygon.corners, sizeof(polygon.corners));
            std::memcpy(data.data() + at + sizeof(polygon.corners), polygon.attributes, 4);
            at += kRoadFaceBytes;
        }
    }
    return data;
}

bool LoadRoadCollision(const nfs::World& world, std::vector<nfs::NfswCollisionPlacement>& placements) {
    if (world.OverlayFolder().empty()) return true;
    const auto name = world.OverlayFolder() + L"\\" + NfswBakedRoadCollisionFile(world.Region());
    if (::GetFileAttributesW(name.c_str()) == INVALID_FILE_ATTRIBUTES) {
        const DWORD why = ::GetLastError();
        return why == ERROR_FILE_NOT_FOUND || why == ERROR_PATH_NOT_FOUND;
    }
    std::vector<u8> data;
    if (!ReadWhole(name, data) || data.size() < 8 || std::memcmp(data.data(), kRoadCollisionMagic, 8)) return false;
    const ByteSpan bytes{ data.data(), data.size() };
    static const nfs::SolidObject wallModel = [] {
        nfs::SolidObject object;
        object.name = "FRS road wall";
        return object;
    }();
    for (size_t at = 8; at < bytes.size;) {
        if (bytes.size - at < 8) return false;
        const u32 count = bytes.read<u32>(at), wall = bytes.read<u32>(at + 4);
        at += 8;
        if (wall > 1 || count > (bytes.size - at) / kRoadFaceBytes) return false;
        nfs::NfswCollisionPlacement piece{ wall ? &wallModel : nullptr, {} };
        piece.polygons.resize(count);
        for (auto& polygon : piece.polygons) {
            std::memcpy(polygon.corners, data.data() + at, sizeof(polygon.corners));
            std::memcpy(polygon.attributes, data.data() + at + sizeof(polygon.corners), 4);
            polygon.SyncFromAttributes();
            polygon.quad = (polygon.flags & 0x10) != 0;
            polygon.barrier = (polygon.flags & 0x04) != 0;
            at += kRoadFaceBytes;
        }
        placements.push_back(std::move(piece));
    }
    return true;
}

// Compare edited scenery with shipped records so a reopened region can
// regenerate collision for earlier saves and discard deleted custom faces.
bool PlanCollision(const nfs::World& world, const std::wstring& output,
                   std::vector<Overlay>& overlays, NfswSmackableWrite& smackables,
                   std::wostringstream& log, const std::vector<nfs::NfswCollisionPlacement>& roads = {}) {
    const nfs::Section* home = world.Table().FindByNumber(nfs::kNfswCollisionSection);
    if (!home) return true;

    std::map<u32, size_t> changed;
    for (size_t i = 0; i < overlays.size(); ++i)
        changed[overlays[i].section->number] = i;
    std::map<u32, const nfs::Section*> positioned;
    for (const auto& loaded : world.Loaded())
        if (loaded->info && loaded->info != home && !loaded->sceneryNumbers.empty() &&
            !world.IsLibrary(*loaded))
            positioned.emplace(loaded->info->number, loaded->info);
    std::vector<nfs::NfswCollisionPlacement> placements;
    for (const auto& [number, section] : positioned) {
        const auto found = changed.find(number);
        const std::wstring name = world.NfswSectionFileName(*section);
        if (found == changed.end() && (world.OverlayFolder().empty() ||
            ::GetFileAttributesW((world.OverlayFolder() + L"\\files\\TRACKS\\" + name).c_str())
                == INVALID_FILE_ATTRIBUTES))
            continue;

        std::vector<u8> original;
        if (!ReadWhole(world.TracksDirectory() + L"\\" + name, original)) {
            log << L"  " << section->name.c_str() << L": original collision source section could not be read.\n";
            return false;
        }
        const ByteSpan baseline = world.SectionBytes(*section);
        ByteSpan edited = baseline;
        if (found != changed.end()) {
            const auto& bytes = overlays[found->second].bytes;
            edited = { bytes.data(), bytes.size() };
        }
        const nfs::Scenery stock = nfs::NfswReadSectionScenery({ original.data(), original.size() });
        const nfs::Scenery updated = nfs::NfswReadSectionScenery(edited);
        const auto editedRecords = nfs::NfswChangedPlacements(stock, updated);
        std::set<size_t> dynamic;
        std::vector<u8> rebuilt;
        std::string error;
        const NfswSmackableWrite::Copies empty;
        const auto& copies = found == changed.end() ? empty : overlays[found->second].smackableCopies;
        if (!smackables.RewriteSection(edited, stock, nfs::NfswReadSectionScenery(baseline),
                                       updated, copies, editedRecords, dynamic, rebuilt, error)) {
            log << L"  " << section->name.c_str() << L": "
                << (error.empty() ? "dynamic scenery could not be rebuilt" : error.c_str())
                << L". Nothing was written.\n";
            return false;
        }
        if (!rebuilt.empty() && (rebuilt.size() != edited.size ||
            std::memcmp(rebuilt.data(), edited.data, edited.size) != 0)) {
            if (found == changed.end()) {
                overlays.push_back({ section, output + L"\\files\\TRACKS\\" + name, std::move(rebuilt) });
            } else {
                overlays[found->second].bytes = std::move(rebuilt);
                overlays[found->second].writeFile = true;
            }
        }
        const auto dynamicChanges = std::count_if(editedRecords.begin(), editedRecords.end(),
            [&](size_t index) { return dynamic.count(index) != 0; });
        if (dynamicChanges)
            log << L"  " << section->name.c_str() << L": " << dynamicChanges << L" dynamic scenery edits\n";
        CollectCollisionPlacements(world, updated, editedRecords, dynamic, placements);
    }

    if (!roads.empty()) placements.insert(placements.end(), roads.begin(), roads.end());
    else if (!LoadRoadCollision(world, placements)) {
        log << L"The saved road collision source is unreadable.";
        return false;
    }
    const std::wstring target = output + L"\\files\\TRACKS\\" + world.NfswSectionFileName(*home);
    const auto editedHome = changed.find(home->number);
    ByteSpan base = world.SectionBytes(*home);
    if (editedHome != changed.end()) {
        const auto& bytes = overlays[editedHome->second].bytes;
        base = { bytes.data(), bytes.size() };
    }
    if (base.empty() && placements.empty()) return true;
    std::vector<nfs::CollisionPolygon> reopenedWalls;
    const std::vector<nfs::CollisionPolygon>* walls = nullptr;
    std::vector<const nfs::CollisionPack*> collision;
    for (const auto& loaded : world.Loaded()) {
        if (loaded->info == home && !loaded->collision.empty())
            walls = &loaded->collision.front().polygons;
        for (const nfs::CollisionPack& pack : loaded->collision)
            if (pack.edited && pack.sectionNumber != nfs::kNfswCollisionSection)
                collision.push_back(&pack);
    }
    if (!walls) {
        if (!nfs::NfswReadCollisionWalls(base, reopenedWalls)) {
            log << L"  Z0 collision: the saved wall data could not be read.\n";
            return false;
        }
        walls = &reopenedWalls;
    }
    std::vector<u8> result;
    std::string error;
    if (!nfs::NfswBuildPlacementCollision(world.LayoutBytes(), base, placements, *walls,
                                         result, error, collision)) {
        log << L"  Z0 collision: " << error.c_str() << L". Nothing was written.\n";
        return false;
    }

    const bool onDisk = ::GetFileAttributesW(target.c_str()) != INVALID_FILE_ATTRIBUTES;
    bool needsWrite = result.size() != base.size ||
                      std::memcmp(result.data(), base.data, base.size) != 0;
    if (!needsWrite && onDisk && editedHome == changed.end()) {
        // A previous save in this session can differ from the baseline Z0.
        std::vector<u8> previous;
        needsWrite = ReadWhole(target, previous) && previous != result;
    }
    if (needsWrite || onDisk) {
        if (editedHome == changed.end()) overlays.push_back({ home, target, std::move(result), needsWrite });
        else overlays[editedHome->second].bytes = std::move(result);
        log << L"  Z0 collision: " << placements.size() << L" edited objects, "
            << walls->size() << L" wall polygons\n";
    }
    return true;
}

using RemovedKey = std::tuple<u32, u32, u32>;   // section, scenery chunk, record
using Providers = std::set<u16>;
using Dependencies = std::map<u32, std::set<Providers>>;

// Reuse the original asset packs. A STREAM file can contain geometry, textures
// and animation data together, and the game's streamer owns their lifetime.
void AddDependencies(const nfs::World& world, const nfs::SceneryInfo& info,
                     std::set<Providers>& required) {
    std::set<u32> textures;
    const ByteSpan raw{ info.raw.data(), info.raw.size() };
    for (size_t at = 0x18; at < 0x28; at += 4) {
        const u32 hash = raw.read<u32>(at);
        if (!hash) continue;
        Providers models;
        for (const auto& section : world.Loaded()) {
            if (!section->info || !section->HoldsModel(hash)) continue;
            models.insert((u16)section->info->number);
        }
        if (!models.empty()) required.insert(std::move(models));
        if (const nfs::SolidObject* model = world.FindModel(hash))
            textures.insert(model->textureHashes.begin(), model->textureHashes.end());
    }
    for (auto it = textures.begin(); it != textures.end(); ++it) {
        const auto animation = world.TextureAnimations().find(*it);
        if (animation != world.TextureAnimations().end())
            textures.insert(animation->second.frames.begin(), animation->second.frames.end());
    }
    for (u32 hash : textures) {
        Providers packs;
        for (const auto& section : world.Loaded()) {
            if (!section->info) continue;
            for (const auto& pack : section->texturePacks) {
                if (pack->Find(hash)) {
                    packs.insert((u16)section->info->number);
                    break;
                }
            }
        }
        if (!packs.empty()) required.insert(std::move(packs));
    }
}

bool PatchStreamingLists(std::vector<u8>& layout, const Dependencies& dependencies) {
    nfs::ChunkReader top({ layout.data(), layout.size() }, 0);
    nfs::Chunk container;
    while (top.Next(container) && container.id != 0x80034150) {}
    if (container.id != 0x80034150) return false;
    const nfs::Chunk lists = nfs::FindChunk(container, 0x34153);
    if (lists.raw.empty()) return false;

    std::vector<u8> rebuilt;
    for (size_t at = 0; at < lists.raw.size;) {
        const ByteSpan record = lists.raw.sub(at, lists.raw.size - at);
        if (record.size < 20) return false;
        const u16 capacity = record.read<u16>(0x10);
        const u16 count = record.read<u16>(0x12);
        const size_t length = 20 + (size_t)capacity * 2;
        if (count > capacity || length > record.size) return false;
        at += length;

        Providers present{ 25000, 25001, 25002 }; // always requested by TrackStreamer
        for (size_t i = 0; i < count; ++i) present.insert(record.read<u16>(20 + i * 2));
        std::set<Providers> required;
        for (u16 section : present) {
            const auto found = dependencies.find(section);
            if (found != dependencies.end())
                required.insert(found->second.begin(), found->second.end());
        }
        std::vector<u16> additions;
        for (const Providers& providers : required) {
            if (std::any_of(providers.begin(), providers.end(),
                            [&](u16 number) { return present.count(number) != 0; }))
                continue;
            // Prefer a library provider when no copy is already streamed, to
            // avoid bringing in the scenery belonging to a positioned section.
            const auto library = providers.lower_bound(20000);
            const u16 provider = library != providers.end() ? *library : *providers.begin();
            additions.push_back(provider);
            present.insert(provider);
        }

        const size_t start = rebuilt.size();
        rebuilt.insert(rebuilt.end(), record.data, record.data + length);
        if (additions.empty()) continue;
        const size_t used = count + additions.size();
        // Leave later chunk payloads at the same alignment, including 128-byte
        // payloads elsewhere in the layout. Preserve the stock order.
        const size_t slots = used <= capacity ? capacity : capacity + ((used - capacity + 63) & ~size_t(63));
        if (slots > UINT16_MAX || used > 4224) return false;
        rebuilt.resize(start + 20 + slots * 2, 0);
        const u16 size = (u16)slots;
        const u16 written = (u16)used;
        std::memcpy(rebuilt.data() + start + 0x10, &size, sizeof(size));
        std::memcpy(rebuilt.data() + start + 0x12, &written, sizeof(written));
        if (record.read<u16>(0x0E) == count)
            std::memcpy(rebuilt.data() + start + 0x0E, &written, sizeof(written));
        std::memcpy(rebuilt.data() + start + 20 + count * 2,
                    additions.data(), additions.size() * sizeof(u16));
    }

    const size_t payload = (size_t)lists.headerOffset + 8;
    const u32 listSize = (u32)rebuilt.size();
    const u32 containerSize = (u32)(container.raw.size - lists.raw.size + rebuilt.size());
    layout.erase(layout.begin() + payload, layout.begin() + payload + lists.raw.size);
    layout.insert(layout.begin() + payload, rebuilt.begin(), rebuilt.end());
    std::memcpy(layout.data() + lists.headerOffset + 4, &listSize, sizeof(listSize));
    std::memcpy(layout.data() + container.headerOffset + 4, &containerSize, sizeof(containerSize));
    return true;
}

// Both placement files and layout indices are based on the version opened
// by this session. Reapplying index shifts to a previous save would shift twice.
void RemapSceneryEntries(std::vector<u8>& layout,
                        const std::map<std::pair<u32, u32>, SectionPlan>& plans) {
    std::map<u32, std::set<u32>> removed;
    for (const auto& [key, plan] : plans)
        if (!plan.removals.empty())
            removed[key.second].insert(plan.removals.begin(), plan.removals.end());
    if (removed.empty()) return;

    nfs::ChunkReader reader({ layout.data(), layout.size() }, 0);
    nfs::Chunk chunk;
    while (reader.Next(chunk)) {
        if (chunk.id != 0x34108) continue;
        for (size_t at = 0; at + 6 <= chunk.raw.size; at += 6) {
            const u16 number = chunk.raw.read<u16>(at);
            const u16 record = chunk.raw.read<u16>(at + 2);
            const auto found = removed.find(number);
            if (found == removed.end()) continue;
            const auto& records = found->second;
            const auto before = records.lower_bound(record);
            if (before == records.begin() && !records.count(record)) continue;
            u16 entry[3] = { number, (u16)(record - std::distance(records.begin(), before)),
                             chunk.raw.read<u16>(at + 4) };
            if (before != records.end() && *before == record) {
                entry[0] = entry[1] = 0;
                entry[2] |= 0x10;
            }
            std::memcpy(layout.data() + chunk.headerOffset + 8 + at, entry, sizeof(entry));
        }
    }
}

} // namespace

bool WriteNfswPlacementOverlays(const nfs::World& world, const std::wstring& modelsFolder,
                                std::wstring& report) {
    if (!world.IsWorld()) return true;
    if (modelsFolder.empty()) {
        report = L"The game's models folder is not known.";
        return false;
    }

    std::wostringstream log;
    std::map<u32, const nfs::LoadedSection*> byNumber;
    std::set<RemovedKey> removedKeys;
    for (const auto& loaded : world.Loaded()) {
        if (!loaded->info) continue;
        byNumber[loaded->info->number] = loaded.get();
        for (const auto& key : loaded->removedNatives)
            removedKeys.insert({ loaded->info->number, key.first, key.second });
    }

    const auto chunkValid = [](const nfs::LoadedSection& section, u32 chunk) {
        return std::find(section.sceneryNumbers.begin(), section.sceneryNumbers.end(), chunk) !=
               section.sceneryNumbers.end();
    };
    const auto templateHash = [](const nfs::LoadedSection& section,
                                 const nfs::SceneryInstance& instance) {
        return section.scenery.ModelHash(instance);
    };
    const auto sourceInstance = [](const nfs::LoadedSection& section, u32 sceneryNumber, u32 record)
        -> const nfs::SceneryInstance* {
        for (const nfs::SceneryInstance& instance : section.scenery.instances)
            if (instance.sceneryNumber == sceneryNumber && instance.record == record) return &instance;
        return nullptr;
    };

    const auto overridden = [&](const nfs::LoadedSection& section, u32 sceneryNumber, u32 record) {
        const nfs::SceneryInstance* instance = sourceInstance(section, sceneryNumber, record);
        if (!instance) return false;
        const size_t index = (size_t)(instance - section.scenery.instances.data());
        for (const nfs::SceneryOverride& entry : section.scenery.overrides)
            if (entry.instance == index) return true;
        return false;
    };
    const auto removableSmackable = [&](const nfs::LoadedSection& section, u32 number, u32 record) {
        const auto* instance = sourceInstance(section, number, record);
        if (!instance) return false;
        const size_t index = (size_t)(instance - section.scenery.instances.data());
        if (!section.smackableScenery.count(index)) return false;
        for (const auto& entry : section.scenery.overrides) {
            u16 flags = 0;
            if (entry.instance == index && !world.PlainSceneryFlags(entry.entry, number, record, flags))
                return false;
        }
        return true;
    };

    struct CopyTemplate {
        const nfs::LoadedSection* section = nullptr;
        const nfs::SceneryInstance* instance = nullptr;
        std::optional<u16> plainFlags;
    };
    // Prefer a standalone template and fall back to an ordinary registration.
    // That registration belongs to the source and never follows the copy.
    const auto findTemplate = [&](u32 hash) -> CopyTemplate {
        CopyTemplate registered;
        for (const auto& candidate : world.Loaded()) {
            if (!candidate->info || candidate->scenery.instances.empty()) continue;
            const nfs::Scenery& scenery = candidate->scenery;
            for (size_t i = 0; i < scenery.instances.size(); ++i) {
                const nfs::SceneryInstance& item = scenery.instances[i];
                if (!item.sceneryNumber || scenery.ModelHash(item) != hash) continue;
                // A group-owned placement belongs to a holiday/event variant; it
                // is not a safe template for a plain new object.
                if (item.flags & 0x10) continue;
                CopyTemplate found{ candidate.get(), &item, {} };
                bool eligible = true;
                for (const nfs::SceneryOverride& entry : scenery.overrides) {
                    if (entry.instance != i) continue;
                    u16 flags = 0;
                    if (!world.PlainSceneryFlags(entry.entry, item.sceneryNumber, item.record, flags) ||
                        (found.plainFlags && *found.plainFlags != flags)) {
                        eligible = false;
                        break;
                    }
                    found.plainFlags = flags;
                }
                if (!eligible) continue;
                if (!found.plainFlags) return found;
                if (!registered.instance) registered = found;
            }
        }
        return registered;
    };

    std::map<std::pair<u32, u32>, SectionPlan> plans;
    Dependencies dependencies;

    // Plan first: an unsupported edit must leave every overlay untouched.
    for (const auto& loaded : world.Loaded()) {
        if (!loaded->info) continue;
        const nfs::LoadedSection& section = *loaded;
        for (const auto& placement : section.placements) {
            const nfs::PlacedInstance& instance = placement.instance;
            if (instance.smackable && instance.smackable->link.BindingChanged() &&
                !world.FindSmackableAttributes(instance.smackable->link.physics)) {
                report = L"A changed Smackable hash is missing from the active attribute pack. Nothing was written.";
                return false;
            }
            const u32 targetChunk = placement.sceneryNumber ? placement.sceneryNumber
                                    : (section.sceneryNumbers.empty() ? 0 : section.sceneryNumbers[0]);
            if (!chunkValid(section, targetChunk)) {
                log << L"  " << section.info->name.c_str()
                    << L": its target scenery chunk is not in the section; nothing was written.\n";
                report = log.str();
                return false;
            }
            auto& plan = plans[{ section.info->number, targetChunk }];

            if (instance.originalRecord == ~0u) {
                const CopyTemplate source = findTemplate(instance.modelHash);
                if (!source.instance) {
                    log << L"  " << section.info->name.c_str() << L": no shipped placement of "
                        << instance.modelName.c_str() << L" to clone; nothing was written.\n";
                    report = log.str();
                    return false;
                }
                nfs::NfswSceneryAppend append;
                append.sceneryNumber = targetChunk;
                append.templateSection = world.SectionBytes(*source.section->info);
                append.templateSceneryNumber = source.instance->sceneryNumber;
                append.templateRecord = source.instance->record;
                append.modelHash = templateHash(*source.section, *source.instance);
                append.transform = instance.transform;
                append.bounds = instance.WorldBounds();
                append.plainCopyFlags = source.plainFlags;
                if (instance.smackable)
                    plan.smackableAppends.emplace(plan.appends.size(), &*instance.smackable);
                plan.appends.push_back(append);
                AddDependencies(world, source.section->scenery.infos[source.instance->infoIndex],
                                dependencies[section.info->number]);
                continue;
            }

            const auto sourceAt = byNumber.find(instance.originalSection);
            if (sourceAt == byNumber.end()) {
                log << L"  " << section.info->name.c_str() << L": its source section is not loaded; "
                    << L"nothing was written.\n";
                report = log.str();
                return false;
            }
            const nfs::LoadedSection& source = *sourceAt->second;
            const RemovedKey key{ instance.originalSection, instance.originalSceneryNumber,
                                  instance.originalRecord };
            if (!removedKeys.count(key)) {
                log << L"  " << section.info->name.c_str() << L": its source record is not marked "
                    << L"removed; nothing was written.\n";
                report = log.str();
                return false;
            }

            if (instance.originalSection == section.info->number &&
                instance.originalSceneryNumber == targetChunk) {
                plan.updates.push_back({ instance.originalRecord, instance.transform,
                                         instance.WorldBounds() });
            } else {
                if (overridden(source, instance.originalSceneryNumber, instance.originalRecord)) {
                    log << L"  " << section.info->name.c_str()
                        << L": its source record is a holiday/event variant (0x34106); "
                        << L"moving it across sections is not supported yet. Nothing was written.\n";
                    report = log.str();
                    return false;
                }
                auto& sourcePlan = plans[{ instance.originalSection, instance.originalSceneryNumber }];
                sourcePlan.removals.push_back(instance.originalRecord);

                const nfs::SceneryInstance* templateInstance =
                    sourceInstance(source, instance.originalSceneryNumber, instance.originalRecord);
                if (!templateInstance) {
                    log << L"  " << section.info->name.c_str()
                        << L": its source record is missing; nothing was written.\n";
                    report = log.str();
                    return false;
                }
                if (templateInstance->flags & 0x10) {
                    report = L"A holiday/event placement cannot be moved across sections.";
                    return false;
                }
                nfs::NfswSceneryAppend append;
                append.sceneryNumber = targetChunk;
                append.templateSection = world.SectionBytes(*source.info);
                append.templateSceneryNumber = instance.originalSceneryNumber;
                append.templateRecord = instance.originalRecord;
                append.modelHash = templateHash(source, *templateInstance);
                append.transform = instance.transform;
                append.bounds = instance.WorldBounds();
                if (instance.smackable)
                    plan.smackableAppends.emplace(plan.appends.size(), &*instance.smackable);
                plan.appends.push_back(append);
                AddDependencies(world, source.scenery.infos[templateInstance->infoIndex],
                                dependencies[section.info->number]);
            }
        }
    }

    // A removed native with no placement that names it is a delete.
    for (const auto& loaded : world.Loaded()) {
        if (!loaded->info) continue;
        for (const auto& key : loaded->removedNatives) {
            bool moved = false;
            for (const auto& placementSection : world.Loaded()) {
                if (!placementSection->info) continue;
                for (const auto& placement : placementSection->placements) {
                    const nfs::PlacedInstance& instance = placement.instance;
                    if (instance.originalSection == loaded->info->number &&
                        instance.originalSceneryNumber == key.first &&
                        instance.originalRecord == key.second) {
                        moved = true;
                        break;
                    }
                }
                if (moved) break;
            }
            if (moved) continue;
            if (!chunkValid(*loaded, key.first)) {
                log << L"  " << loaded->info->name.c_str()
                    << L": a removed record names a chunk that is not in the section.\n";
                report = log.str();
                return false;
            }
            if (overridden(*loaded, key.first, key.second)) {
                if (!removableSmackable(*loaded, key.first, key.second)) {
                    log << L"  " << loaded->info->name.c_str()
                        << L": this scenery registration cannot be removed. Nothing was written.\n";
                    report = log.str();
                    return false;
                }
                plans[{ loaded->info->number, key.first }].registeredRemovals.push_back(key.second);
            }
            auto& plan = plans[{ loaded->info->number, key.first }];
            plan.removals.push_back(key.second);
        }
    }

    // Build every overlay in memory before writing anything.
    std::map<u32, std::vector<u32>> bySection;
    NfswSmackableWrite smackables;
    smackables.ReadGeometry(world.LayoutBytes());
    for (const auto& loaded : world.Loaded()) {
        if (!loaded->info) continue;
        const bool rebound = std::any_of(loaded->smackableLinks.begin(), loaded->smackableLinks.end(),
            [](const auto& link) { return link.BindingChanged() || link.halfSizes.has_value(); });
        if (loaded->placementsChanged || rebound) bySection[loaded->info->number];
    }
    for (const auto& [key, plan] : plans)
        bySection[key.first].push_back(key.second);

    std::vector<Overlay> overlays;
    for (auto& entry : bySection) {
        const nfs::LoadedSection& section = *byNumber.at(entry.first);
        const ByteSpan base = world.SectionBytes(*section.info);
        if (base.empty()) {
            log << L"  " << section.info->name.c_str() << L": its shipped file could not be read.\n";
            report = log.str();
            return false;
        }
        std::vector<u8> current(base.data, base.data + base.size);
        Overlay overlay;

        for (u32 chunk : entry.second) {
            const SectionPlan& plan = plans.at({ entry.first, chunk });
            std::vector<u8> next;
            std::string error;
            if (!plan.removals.empty() || !plan.appends.empty()) {
                size_t firstAppendedRecord = 0;
                if (!nfs::NfswRebuildScenery({ current.data(), current.size() }, chunk,
                                             plan.updates, plan.removals, plan.appends, next, error,
                                             plan.registeredRemovals, &firstAppendedRecord)) {
                    log << L"  " << section.info->name.c_str() << L": " << error.c_str() << L".\n";
                    report = log.str();
                    return false;
                }
                for (const auto& [index, source] : plan.smackableAppends)
                    overlay.smackableCopies.emplace(std::make_pair(chunk, (u32)(firstAppendedRecord + index)), source);
            } else {
                std::vector<nfs::NfswSceneryPatch> patches;
                for (const nfs::NfswSceneryUpdate& update : plan.updates)
                    patches.push_back({ chunk, update.record, update.transform, update.bounds });
                if (!nfs::NfswPatchScenery({ current.data(), current.size() }, patches, next, error)) {
                    log << L"  " << section.info->name.c_str() << L": " << error.c_str() << L".\n";
                    report = log.str();
                    return false;
                }
            }
            current.swap(next);
        }

        for (const auto& link : section.smackableLinks) {
            if (link.BindingChanged() && !world.FindSmackableAttributes(link.physics)) {
                report = L"A changed Smackable hash is missing from the active attribute pack. Nothing was written.";
                return false;
            }
        }
        if (!smackables.PatchBindings(current, section.smackableLinks)) {
            log << L"  " << section.info->name.c_str() << L": the Smackable spawn or collision box could not be written.\n";
            report = log.str();
            return false;
        }

        overlay.section = section.info;
        overlay.target = modelsFolder + L"\\files\\TRACKS\\" + world.NfswSectionFileName(*section.info);
        if (current.size() == base.size && std::memcmp(current.data(), base.data, base.size) == 0) {
            // If there was no output for this section, there is nothing to
            // restore. The session's baseline may itself be a saved overlay.
            if (::GetFileAttributesW(overlay.target.c_str()) == INVALID_FILE_ATTRIBUTES) continue;
        }
        overlay.bytes = std::move(current);
        overlays.push_back(std::move(overlay));
    }

    const ByteSpan baseline = world.LayoutBytes();
    std::vector<u8> layout(baseline.data, baseline.data + baseline.size);
    RemapSceneryEntries(layout, plans);
    std::vector<u8> stockLayout;
    if (!ReadWhole(world.TracksDirectory() + L"\\" + world.Region() + L".BUN", stockLayout) ||
        !smackables.Start(world, { stockLayout.data(), stockLayout.size() },
                          { layout.data(), layout.size() })) {
        report = L"The World scenery registration table could not be read.";
        return false;
    }
    if (!PlanCollision(world, modelsFolder, overlays, smackables, log)) {
        report = log.str();
        return false;
    }

    if (!PatchLayoutSizes(world, overlays, layout, log)) {
        report = log.str() + L"The section's stream sizes could not be updated. Nothing was written.";
        return false;
    }
    if (!smackables.WriteLayout(layout)) {
        report = L"The layout's dynamic scenery entries could not be updated. Nothing was written.";
        return false;
    }
    if (!dependencies.empty() && !PatchStreamingLists(layout, dependencies)) {
        report = L"The layout's scenery streaming lists could not be updated.";
        return false;
    }

    const std::wstring layoutPath = modelsFolder + L"\\files\\TRACKS\\" + world.Region() + L".BUN";
    bool writeLayout = layout.size() != baseline.size ||
                       std::memcmp(layout.data(), baseline.data, baseline.size) != 0;
    if (!writeLayout && ::GetFileAttributesW(layoutPath.c_str()) != INVALID_FILE_ATTRIBUTES) {
        std::vector<u8> previous;
        writeLayout = !ReadWhole(layoutPath, previous) || previous != layout;
    }

    CreateOverlayFolders(modelsFolder);
    for (const Overlay& overlay : overlays) {
        if (!overlay.writeFile) continue;
        if (!WriteWhole(overlay.target, overlay.bytes)) {
            report = L"Could not write " + overlay.target;
            return false;
        }
        log << L"  " << overlay.section->name.c_str() << L": " << overlay.bytes.size() << L" bytes\n";
    }
    if (writeLayout) {
        if (!WriteWhole(layoutPath, layout)) {
            report = L"Could not write " + layoutPath;
            return false;
        }
        log << L"  layout " << world.Region().c_str() << L".BUN updated\n";
    }

    report = log.str();
    LOG_INFO("NFSW placements: %zu section overlays written", overlays.size());
    return true;
}

bool PlanNfswMeshOverlays(const nfs::World& world, const std::vector<nfs::NfswMeshUpdate>& meshes,
                          std::vector<NfswMeshOverlay>& overlays, std::wstring& error) {
    overlays.clear();
    if (meshes.empty()) return true;
    std::map<u32, Aabb> modelBounds;
    for (const auto& mesh : meshes)
        for (const Vec3& point : mesh.positions)
            modelBounds[mesh.hash].Add(point);

    const auto referencesEditedModel = [&](const nfs::SceneryInfo& info) {
        const ByteSpan raw{ info.raw.data(), info.raw.size() };
        for (size_t lod = 0x18; lod < 0x28; lod += 4)
            if (modelBounds.count(raw.read<u32>(lod))) return true;
        return false;
    };
    std::set<u32> found;
    std::vector<NfswMeshOverlay> planned;
    for (const auto& loaded : world.Loaded()) {
        if (!loaded->info) continue;
        std::vector<const nfs::NfswMeshUpdate*> localMeshes;
        for (const auto& mesh : meshes) {
            if (!loaded->HoldsModel(mesh.hash)) continue;
            localMeshes.push_back(&mesh);
            found.insert(mesh.hash);
        }
        const std::wstring file = world.NfswSectionFileName(*loaded->info);
        const std::wstring overlayPath = world.OverlayFolder() + L"\\files\\TRACKS\\" + file;
        const bool hasOverlay = !world.OverlayFolder().empty() &&
            ::GetFileAttributesW(overlayPath.c_str()) != INVALID_FILE_ATTRIBUTES;
        const bool hasReferences = std::any_of(loaded->scenery.infos.begin(), loaded->scenery.infos.end(),
                                               referencesEditedModel);
        // A just-saved placement may exist only in the BUN on disk. Check the
        // existing overlays as well as the scenery the viewport has indexed.
        if (localMeshes.empty() && !hasReferences && !hasOverlay) continue;
        std::vector<u8> bytes;
        if (!ReadWhole(hasOverlay ? overlayPath : world.TracksDirectory() + L"\\" + file, bytes)) {
            error = L"Could not read " + file;
            return false;
        }
        const ByteSpan base{ bytes.data(), bytes.size() };
        const auto scenery = nfs::NfswReadSectionScenery(base);
        std::vector<nfs::NfswSceneryPatch> boxes;
        for (const auto& instance : scenery.instances) {
            if (instance.infoIndex >= scenery.infos.size()) continue;
            const auto& info = scenery.infos[instance.infoIndex];
            const ByteSpan raw{ info.raw.data(), info.raw.size() };
            Aabb bounds = instance.worldBounds;
            for (size_t lod = 0x18; lod < 0x28; lod += 4) {
                const auto edited = modelBounds.find(raw.read<u32>(lod));
                if (edited != modelBounds.end()) bounds.Add(Transform(edited->second, instance.transform));
            }
            if (!bounds.Valid() || (Length(bounds.min - instance.worldBounds.min) <= 0.001f &&
                                    Length(bounds.max - instance.worldBounds.max) <= 0.001f)) continue;
            boxes.push_back({ instance.sceneryNumber, instance.record, instance.transform, bounds, true });
        }
        if (localMeshes.empty() && boxes.empty()) continue;
        NfswMeshOverlay overlay;
        overlay.file = file;
        std::string why;
        if (boxes.empty()) overlay.bytes.assign(base.begin(), base.end());
        else if (!nfs::NfswPatchScenery(base, boxes, overlay.bytes, why)) {
            error = overlay.file + L": " + std::wstring(why.begin(), why.end());
            return false;
        }
        if (!localMeshes.empty() && !nfs::NfswPatchSolidMeshes(overlay.bytes, localMeshes, why)) {
            error = overlay.file + L": " + std::wstring(why.begin(), why.end());
            return false;
        }
        if (std::memcmp(overlay.bytes.data(), base.data, base.size))
            planned.push_back(std::move(overlay));
    }
    if (found.size() != meshes.size()) {
        error = L"A mesh is missing from its loaded World section.";
        return false;
    }
    overlays = std::move(planned);
    return true;
}

bool WriteNfswMeshOverlays(const std::vector<NfswMeshOverlay>& overlays,
                           const std::wstring& modelsFolder, std::wstring& error) {
    if (overlays.empty()) return true;
    if (modelsFolder.empty()) {
        error = L"The game's models folder is not known.";
        return false;
    }
    const std::filesystem::path models(modelsFolder);
    const auto tracks = models / L"files" / L"TRACKS";
    std::error_code directoryError;
    std::filesystem::create_directories(tracks, directoryError);
    if (directoryError || std::filesystem::equivalent(tracks, models.parent_path() / L"TRACKS", directoryError)) {
        error = L"Could not prepare the external World section folder.";
        return false;
    }

    struct Staged {
        std::wstring target, incoming, backup;
        bool remove = false;
    };
    std::vector<Staged> staged;
    const auto temporary = [&]() -> std::wstring {
        wchar_t name[MAX_PATH]{};
        if (!::GetTempFileNameW(modelsFolder.c_str(), L"fwm", 0, name)) return {};
        return name;
    };
    size_t installed = 0;
    bool ready = true;
    for (const auto& overlay : overlays) {
        auto& file = staged.emplace_back();
        const std::filesystem::path name(overlay.file);
        if (name.empty() || name != name.filename() || name == L"." || name == L"..") {
            error = L"Invalid external file name.";
            ready = false;
            break;
        }
        file.target = ((overlay.modelsRoot ? models : tracks) / name).wstring();
        file.remove = overlay.remove;
        if (!file.remove) file.incoming = temporary();
        if (!file.remove && (file.incoming.empty() || !WriteWhole(file.incoming, overlay.bytes))) {
            error = L"Could not stage " + overlay.file;
            ready = false;
            break;
        }
        if (::GetFileAttributesW(file.target.c_str()) != INVALID_FILE_ATTRIBUTES) {
            file.backup = temporary();
            if (file.backup.empty() || !::CopyFileW(file.target.c_str(), file.backup.c_str(), FALSE)) {
                error = L"Could not keep a recovery copy of " + overlay.file;
                ready = false;
                break;
            }
        }
    }
    if (ready) {
        for (const auto& file : staged) {
            const bool done = file.remove ? (file.backup.empty() || ::DeleteFileW(file.target.c_str()))
                : ::MoveFileExW(file.incoming.c_str(), file.target.c_str(),
                                MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH);
            if (!done) {
                error = L"Could not replace " + file.target;
                break;
            }
            ++installed;
        }
    }
    const bool complete = installed == overlays.size();
    if (!complete) {
        while (installed > 0) {
            auto& file = staged[--installed];
            const bool restored = file.backup.empty() ? (file.remove || ::DeleteFileW(file.target.c_str()))
                : ::MoveFileExW(file.backup.c_str(), file.target.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH);
            if (!restored) {
                error += L"\nCould not restore " + file.target;
                if (!file.backup.empty()) error += L". Recovery copy: " + file.backup;
                file.backup.clear(); // retain the recovery copy named in the error
            }
        }
    }
    for (const auto& file : staged) {
        if (!file.incoming.empty()) ::DeleteFileW(file.incoming.c_str());
        if (!file.backup.empty()) ::DeleteFileW(file.backup.c_str());
    }
    return complete;
}

bool PlanNfswRoadOverlays(const nfs::World& world, const std::vector<NfswRoadPlacement>& roads,
                          const std::vector<nfs::NfswRoadSegmentBuild>& routes,
                          const std::vector<nfs::NfswCollisionPlacement>& collision,
                          std::vector<NfswMeshOverlay>& output, nfs::NfswRoadBuildReport& nav,
                          std::wstring& error) {
    if (!world.IsWorld() || roads.empty()) { error = L"No World road mesh is ready to bake."; return false; }
    const auto& loaded = world.Loaded();
    const auto baseline = world.LayoutBytes();
    std::map<size_t, std::vector<const NfswRoadPlacement*>> sections;
    std::set<u32> hashes;
    for (const auto& road : roads) {
        if (road.section >= loaded.size() || !loaded[road.section]->info || world.FindModel(road.model.hash) ||
            !hashes.insert(road.model.hash).second) {
            error = L"A road has no section or its generated model name is in use.";
            return false;
        }
        sections[road.section].push_back(&road);
    }
    struct Template { const nfs::LoadedSection* section = nullptr; const nfs::SceneryInstance* instance = nullptr; };
    const auto findTemplate = [&](const nfs::LoadedSection& section) -> Template {
        Template fallback;
        for (size_t i = 0; i < section.scenery.instances.size(); ++i) {
            const auto& item = section.scenery.instances[i];
            if (item.flags & (0x10 | 0x40000000 | 0x00800000) || !item.sceneryNumber ||
                section.smackableScenery.count(i)) continue;
            if (std::any_of(section.scenery.overrides.begin(), section.scenery.overrides.end(),
                            [&](const auto& entry) { return entry.instance == i; })) continue;
            const auto* model = world.FindModel(section.scenery.ModelHash(item));
            if (!model) continue;
            if (model->HasNfswRoadSurface()) return { &section, &item };
            if (!fallback.instance) fallback = { &section, &item };
        }
        return fallback;
    };

    Dependencies dependencies;
    std::map<u32, Providers> textures;
    std::vector<Overlay> overlays;
    std::string why;
    for (const auto& [index, pieces] : sections) {
        const auto& target = *loaded[index];
        if (baseline.read<u32>((size_t)target.info->tableEntryOffset + 0x5C) != 1) {
            error = L"A road piece falls in a multipart streaming section.";
            return false;
        }
        Template seed = findTemplate(target);
        if (!seed.instance)
            for (const auto& candidate : loaded) {
                seed = findTemplate(*candidate);
                if (seed.instance) break;
            }
        if (!seed.instance) {
            error = L"The World region has no plain scenery record for a road.";
            return false;
        }
        std::vector<nfs::NfswSceneryAppend> inserts;
        std::vector<nfs::SolidObject> models;
        for (const auto* piece : pieces) {
            for (u32 hash : piece->model.textureHashes) {
                auto [entry, first] = textures.try_emplace(hash);
                if (first)
                    for (const auto& provider : loaded) {
                        if (!provider->info) continue;
                        if (std::any_of(provider->texturePacks.begin(), provider->texturePacks.end(),
                                        [&](const auto& pack) { return pack->Find(hash) != nullptr; }))
                            entry->second.insert((u16)provider->info->number);
                    }
                if (entry->second.empty()) {
                    error = L"A road texture has no provider in the World section stream.";
                    return false;
                }
                dependencies[target.info->number].insert(entry->second);
            }
            const auto& source = *seed.instance;
            nfs::NfswSceneryAppend append;
            append.sceneryNumber = target.info->number;
            append.templateSection = world.SectionBytes(*seed.section->info);
            append.templateSceneryNumber = source.sceneryNumber;
            append.templateRecord = source.record;
            append.transform = Mat4::Translation(piece->position);
            append.bounds = Transform(piece->model.bounds, append.transform);
            append.newModelName = piece->model.name;
            append.newModelHash = piece->model.hash;
            inserts.push_back(std::move(append));
            models.push_back(piece->model);
        }
        Overlay section;
        section.section = target.info;
        if (!nfs::NfswRebuildScenery(world.SectionBytes(*target.info), target.info->number,
                                    {}, {}, inserts, section.bytes, why) ||
            !nfs::NfswAppendRoadModels(section.bytes, models, why)) {
            error.assign(why.begin(), why.end());
            return false;
        }
        overlays.push_back(std::move(section));
    }

    std::vector<u8> layout(baseline.begin(), baseline.end());
    std::vector<u8> stock;
    NfswSmackableWrite smackables;
    std::wostringstream details;
    if (!ReadWhole(world.TracksDirectory() + L"\\" + world.Region() + L".BUN", stock) ||
        !smackables.Start(world, { stock.data(), stock.size() }, baseline) ||
        !PlanCollision(world, world.OverlayFolder(), overlays, smackables, details, collision) ||
        !PatchLayoutSizes(world, overlays, layout, details) || !smackables.WriteLayout(layout) ||
        !PatchStreamingLists(layout, dependencies)) {
        error = details.str() + L"The road collision, layout or streaming dependencies could not be built.";
        return false;
    }
    if (!nfs::NfswAppendRoadGraph(layout, routes, nav, why)) {
        error.assign(why.begin(), why.end());
        return false;
    }

    std::vector<NfswMeshOverlay> planned;
    for (auto& section : overlays)
        if (section.writeFile) planned.push_back({ world.NfswSectionFileName(*section.section), std::move(section.bytes) });
    planned.push_back({ world.Region() + L".BUN", std::move(layout) });
    planned.push_back({ NfswBakedRoadCollisionFile(world.Region()), EncodeRoadCollision(collision), false, true });
    output = std::move(planned);
    return true;
}
