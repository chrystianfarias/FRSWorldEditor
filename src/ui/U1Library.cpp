#include "ui/U1Library.h"

#include "core/Log.h"
#include "io/ModelCopy.h"
#include "io/PlacementWrite.h"
#include "io/SectionEdits.h"
#include "nfs/SceneryEdit.h"
#include "io/TextureImport.h"
#include "io/U1Convert.h"
#include "nfs/ChunkReader.h"
#include "nfs/ForeignModels.h"
#include "nfs/SolidList.h"
#include "nfs/TexturePack.h"
#include "nfs/World.h"
#include "render/D3D9Context.h"
#include "render/GpuResources.h"
#include "render/RenderExtension.h"
#include "render/WorldRenderer.h"
#include "ui/MainWindow.h"
#include "ui/ModelCatalog.h"
#include "ui/Resource.h"
#include "ui/SetupWizard.h"
#include "ui/Viewport.h"

#include <windows.h>

#include <atomic>
#include <cstring>
#include <map>
#include <mutex>
#include <thread>
#include <unordered_map>

// How this extension reaches the window's parts (a friend of MainWindow).
struct U1LibraryAccess {
    explicit U1LibraryAccess(MainWindow& window) : w(window) {}
    MainWindow& w;

    HWND Window() const { return w.window_; }
    HMENU Menu() const { return w.menu_; }
    WorldRenderer& Renderer() const { return w.viewport_.GetRenderer(); }
    ModelCatalog& Catalog() const { return w.catalog_; }
    nfs::World* World() const { return w.world_.get(); }
    void Status(const std::wstring& text) const { w.SetStatusText(2, text); }
};

namespace {


const wchar_t kGame[] = L"Underground 1";

// The regions read, in this order: a model in more than one is taken from the first.
const wchar_t* const kRegions[] = { L"L1RA", L"L1RB", L"L1RC", L"L1RE", L"L1RX" };

// One other game whose models the library offers.
struct GameSpec {
    const wchar_t* game;          // the Library's Game filter
    const wchar_t* title;         // in the status bar
    const wchar_t* folderKey;     // OtherGameFolder(...)
    std::wstring (*detect)();
    bool (*valid)(const std::wstring&);
    std::vector<const wchar_t*> regions;
    // To Underground 2's layout; none when the game's own is that.
    bool (*toU2)(ByteSpan, u64, std::vector<u8>&, std::wstring&);
    // Whether the models are wanted with this world open (null: always).
    bool (*wantedWith)(const nfs::World*);
};

bool IsUnderground2Folder(const std::wstring& folder) {
    return IsGameFolder(folder) && ::GetFileAttributesW((folder + L"\\SPEED2.EXE").c_str()) != INVALID_FILE_ATTRIBUTES;
}

bool OnlyWithUnderground1(const nfs::World* world) { return world && world->IsUnderground1(); }

const GameSpec kUnderground1{ kGame, L"Need for Speed Underground", L"Underground1", &DetectUnderground1Folder,
                              &IsUnderground1Folder, { L"L1RA", L"L1RB", L"L1RC", L"L1RE", L"L1RX" },
                              &ConvertU1Object, nullptr };
// Underground 2's city, for the first game's regions: read only while one of
// those is open (an Underground 2 region has these models as its own).
const GameSpec kUnderground2{ L"Underground 2", L"Need for Speed Underground 2", L"Underground2", &DetectGameFolder,
                              &IsUnderground2Folder, { L"L4RA" }, nullptr, &OnlyWithUnderground1 };

std::wstring Widen(const std::string& text) { return std::wstring(text.begin(), text.end()); }

// The extension itself.
class U1Library final : public EditorExtension, public RenderExtension, public nfs::ForeignModels {
public:
    U1Library(MainWindow& window, const GameSpec& spec) : app_(window), spec_(spec) {}

    ~U1Library() override {
        nfs::RemoveForeignModels(this);
        if (loader_.joinable()) loader_.join();
    }

    // ---- EditorExtension

    void OnCreate() override {
        app_.Renderer().AddExtension(this);
        Reconfigure();
    }

    // The folder is set in File > Game Folder and Plugins (ui/SetupWizard.h),
    // which sends this to every extension once it is closed.
    bool OnCommand(int id) override {
        if (id == ID_LIBRARY_FOLDERS_CHANGED) Reconfigure();
        return false;   // the other games' libraries hear it too
    }

    void Tick(float) override {
        if (!loading_ || !done_.load()) return;
        loader_.join();
        loading_ = false;
        {
            std::lock_guard<std::mutex> lock(handover_);
            worlds_ = std::move(read_);
            regions_ = std::move(readRegions_);
        }
        Index();
        nfs::AddForeignModels(this);
        app_.Catalog().Refresh();
        if (nfs::World* world = app_.World()) app_.Renderer().RebuildInstances(*world);
        app_.Status(models_.empty()
                        ? std::wstring(L"No models were read from ") + spec_.title + L"'s folder; the log has why."
                        : std::wstring(spec_.title) + L": " + std::to_wstring(models_.size()) +
                              L" models in the library (Game filter: " + spec_.game + L").");
        if (spec_.wantedWith && _wgetenv(L"FRS_U1_FOREIGN_SELFTEST")) ForeignSelfTest();
    }

    // FRS_U1_FOREIGN_SELFTEST=1: one of these models (with textures) placed in
    // the open Underground 1 region, the save planned into a scratch folder
    // (nothing reaches the game's), the sections read back: the model in the
    // first game's layout, its textures and the placement; then undone.
    void ForeignSelfTest() {
        nfs::World* world = app_.World();
        if (!world || !world->IsUnderground1()) return;
        const nfs::SolidObject* model = nullptr;
        for (const auto& [hash, entry] : models_)
            if (!entry.model->textureHashes.empty() && entry.model->TriangleCount() > 20 &&
                world->ModelHome(hash) >= world->Loaded().size()) { model = entry.model; break; }
        size_t target = SIZE_MAX;
        for (size_t s = 0; s < world->Loaded().size(); ++s) {
            const nfs::LoadedSection& section = *world->Loaded()[s];
            if (section.info && !world->IsLibrary(section) && !section.scenery.instances.empty()) { target = s; break; }
        }
        if (!model || target == SIZE_MAX) { LOG_WARN("u1 foreign self-test: no model or section to try"); return; }
        nfs::LoadedSection* section = world->MutableLoaded(target);
        nfs::PlacedInstance placed;
        placed.modelHash = model->hash;
        placed.modelName = model->name;
        placed.modelBounds = model->bounds;
        placed.triangles = model->TriangleCount();
        if (model->bounds.Valid()) placed.radius = Length(model->bounds.max - model->bounds.min) * 0.5f;
        placed.transform = section->scenery.instances.front().transform;
        section->placements.push_back({ section->info->number, placed });
        section->placementsChanged = true;

        wchar_t temp[MAX_PATH] = {};
        ::GetTempPathW(MAX_PATH, temp);
        const std::wstring scratch = std::wstring(temp) + L"frs-u1-foreign-selftest";
        ::CreateDirectoryW(scratch.c_str(), nullptr);
        PlacementSavePlan plan;
        const bool planned = PlanPlacementSave(*world, scratch, plan);
        size_t models = 0, textures = 0, placements = 0, layoutU1 = 0, unended = 0;
        for (const SectionSpan& span : plan.spans) {
            if (span.layoutFile) continue;
            size_t walked = 0;
            while (walked + 8 <= span.bytes.size()) {
                u32 size = 0;
                std::memcpy(&size, span.bytes.data() + walked + 4, 4);
                walked += 8 + (size_t)size;
            }
            if (walked != span.bytes.size()) {
                ++unended;
                LOG_WARN("u1 foreign self-test: %s does not end on its chunks", span.name.c_str());
            }
            for (const auto& loaded : world->Loaded()) {
                if (!loaded->info || span.name.find("(section " + loaded->info->name + ")") == std::string::npos) continue;
                std::vector<SectionEdit> found;
                std::wstring why;
                DiffSectionBytes(*world, *loaded->info, ByteSpan{ span.bytes.data(), span.bytes.size() },
                                 span.fileOffset, found, why);
                for (const SectionEdit& e : found) {
                    if (e.kind == SectionEdit::Kind::TextureAdded) ++textures;
                    if (e.kind == SectionEdit::Kind::Placement && e.placement.modelHash == model->hash) ++placements;
                    if (e.kind != SectionEdit::Kind::ModelAdded || e.hash != model->hash) continue;
                    ++models;
                    nfs::ChunkReader reader(ByteSpan{ e.chunk.data(), e.chunk.size() }, e.mod128);
                    nfs::Chunk chunk;
                    nfs::SolidObject back;
                    if (reader.Next(chunk) && nfs::ParseSolidObject(chunk, back) && (back.version & 0xFFFF) == 0x13 &&
                        back.name == model->name && back.vertices.size() == model->vertices.size())
                        ++layoutU1;
                }
                LOG_INFO("u1 foreign self-test %s: %zu edits %S (span '%s', %zu bytes at %llu; shipped %u at %llu)",
                         loaded->info->name.c_str(), found.size(), why.c_str(), span.name.c_str(), span.bytes.size(),
                         (unsigned long long)span.fileOffset, loaded->info->originalStreamSize,
                         (unsigned long long)loaded->info->originalStreamOffset);
            }
        }
        LOG_INFO("u1 foreign self-test: %s placed in %s, planned %s: model added %zu (first game's layout %zu), "
                 "%zu textures added, %zu placements, %zu spans not ending on a chunk; report:\n%S",
                 model->name.c_str(), section->info->name.c_str(), planned ? "yes" : "NO", models, layoutU1, textures,
                 placements, unended, plan.report.c_str());
        section->placements.pop_back();
        section->placementsChanged = false;
    }

    void OnRegionLoaded() override {
        // A game wanted with some worlds only: read when one opens.
        if (spec_.wantedWith && !enabled_ && !loading_) Reconfigure();
        // A new region holds models of its own: the list is asked again.
        if (enabled_ && !loading_) app_.Catalog().Refresh();
    }

    // ---- RenderExtension

    bool FindModel(u32 hash, const GpuSection*& gpu, const GpuObject*& object) override {
        const auto found = models_.find(hash);
        if (found == models_.end()) return false;
        const Entry& entry = found->second;
        const nfs::World& world = *worlds_[entry.world];
        const size_t home = world.ModelHome(hash);
        if (home >= world.Loaded().size()) return false;
        D3D9Context* context = app_.Renderer().Context();
        if (!context || !context->IsValid()) return false;

        auto& slot = gpu_[{ entry.world, home }];
        if (!slot) {
            slot = std::make_unique<GpuSection>();
            if (!slot->Upload(context->Device(), *world.Loaded()[home])) {
                slot.reset();
                return false;
            }
        }
        for (const GpuObject& candidate : slot->Objects())
            if (candidate.source && candidate.source->hash == hash) {
                gpu = slot.get();
                object = &candidate;
                return true;
            }
        return false;
    }

    void ReleaseGpu() override { gpu_.clear(); }

    // ---- nfs::ForeignModels

    void ListModels(std::vector<const nfs::SolidObject*>& out) const override {
        out.reserve(out.size() + models_.size());
        for (const auto& [hash, entry] : models_) out.push_back(entry.model);
    }

    std::wstring Game() const override { return spec_.game; }

    const nfs::SolidObject* FindModel(u32 hash) const override {
        const auto found = models_.find(hash);
        return found == models_.end() ? nullptr : found->second.model;
    }

    const nfs::Texture* FindTexture(u32 hash) const override {
        for (const auto& world : worlds_)
            if (const nfs::Texture* texture = world->FindDisplayTexture(hash)) return texture;
        return nullptr;
    }

    bool ModelChunk(u32 hash, ByteSpan& chunk, u32& mod128, std::wstring& error) const override {
        const auto found = models_.find(hash);
        if (found == models_.end()) { error = std::wstring(L"not one of ") + spec_.title + L"'s models"; return false; }
        auto converted = converted_.find(hash);
        if (converted == converted_.end()) {
            ByteSpan object;
            u64 offset = 0;
            if (!::FindModelChunk(*worlds_[found->second.world], hash, object, offset)) {
                error = L"its bytes were not found in " + found->second.origin;
                return false;
            }
            if (!spec_.toU2) {   // already this layout
                chunk = object;
                mod128 = (u32)(offset % 128);
                return true;
            }
            std::vector<u8> bytes;
            if (!spec_.toU2(object, offset, bytes, error)) return false;
            converted = converted_.emplace(hash, std::move(bytes)).first;
            LOG_INFO("%S library: %s converted (%zu -> %zu bytes)", spec_.game, found->second.model->name.c_str(),
                     object.size, converted->second.size());
        }
        chunk = ByteSpan{ converted->second.data(), converted->second.size() };
        mod128 = 0;
        return true;
    }

    bool CopyTexture(u32 hash, NewTexture& out) const override {
        const nfs::Texture* texture = FindTexture(hash);
        if (!texture) return false;
        if (texture->format == nfs::TextureFormat::Argb8888) {
            const u32 w = texture->width, h = texture->height;
            if (!w || !h || w % 4 || h % 4 || texture->pixels.size < (size_t)w * h * 4) return false;
            std::vector<u32> bgra((size_t)w * h);
            std::memcpy(bgra.data(), texture->pixels.data, bgra.size() * 4);
            TextureFromPixels(texture->name, bgra, w, h, out);
            out.name = texture->name;
            out.hash = texture->hash;
            if (texture->header.size >= 124) out.header.assign(texture->header.data, texture->header.data + 124);
            return !out.pixels.empty();
        }
        return CopyGameTexture(*texture, out);
    }

    std::wstring Origin(u32 hash) const override {
        const auto found = models_.find(hash);
        return found == models_.end() ? std::wstring() : found->second.origin;
    }

private:
    struct Entry {
        size_t world = 0;
        const nfs::SolidObject* model = nullptr;
        std::wstring origin;   // "Underground 1, L1RA X0"
    };

    // Reads the folder from the settings: its models in the library when it
    // is the game's, none when it is not set.
    void Reconfigure() {
        // Never set: the game is taken where it is installed, and kept --
        // File > Game Folder and Plugins changes it, or empties it for none.
        if (!HasOtherGameFolder(spec_.folderKey)) {
            const std::wstring found = spec_.detect();
            StoreOtherGameFolder(spec_.folderKey, found);
            if (!found.empty()) LOG_INFO("%S library: %S found in %S", spec_.game, spec_.title, found.c_str());
        }
        const std::wstring folder = OtherGameFolder(spec_.folderKey);
        // Once read, kept while the folder stays: its models may be placed already.
        const bool wanted = spec_.valid(folder) && (!spec_.wantedWith || enabled_ || spec_.wantedWith(app_.World()));
        if (wanted && folder == folder_ && (enabled_ || loading_)) return;
        if (!wanted && !enabled_) { folder_ = folder; return; }
        if (enabled_) {
            if (PlacedButNotSaved()) {
                app_.Status(std::wstring(L"Objects placed from ") + spec_.title +
                            L" are not saved yet: its models stay in the library until the next start.");
                return;
            }
            Unload();
            if (!wanted) app_.Status(std::wstring(spec_.title) + L"'s models are out of the library.");
        }
        folder_ = folder;
        if (wanted) StartLoading();
    }

    void StartLoading() {
        if (loading_) return;
        enabled_ = true;
        loading_ = true;
        done_ = false;
        app_.Status(std::wstring(L"Reading ") + spec_.title + L"'s models from " + folder_ + L"...");
        const std::wstring tracks = folder_ + L"\\TRACKS";
        loader_ = std::thread([this, tracks] {
            std::vector<std::unique_ptr<nfs::World>> worlds;
            std::vector<std::wstring> regions;
            for (const wchar_t* region : spec_.regions) {
                if (::GetFileAttributesW((tracks + L"\\STREAM" + region + L".BUN").c_str()) == INVALID_FILE_ATTRIBUTES)
                    continue;
                auto world = std::make_unique<nfs::World>();
                world->SetForeignLookup(false);
                if (!world->Open(tracks, region)) {
                    LOG_WARN("%S library: %S could not be opened", spec_.game, region);
                    continue;
                }
                world->LoadAllSections();
                worlds.push_back(std::move(world));
                regions.push_back(region);
            }
            std::lock_guard<std::mutex> lock(handover_);
            read_ = std::move(worlds);
            readRegions_ = std::move(regions);
            done_ = true;
        });
    }

    void Index() {
        models_.clear();
        size_t backdrops = 0;
        for (size_t w = 0; w < worlds_.size(); ++w) {
            const nfs::World& world = *worlds_[w];
            for (const nfs::SolidObject* model : world.AllModels()) {
                if (!model || models_.count(model->hash)) continue;
                if (model->IsBackdrop()) { ++backdrops; continue; }
                const size_t home = world.ModelHome(model->hash);
                const nfs::LoadedSection* section = home < world.Loaded().size() ? world.Loaded()[home].get() : nullptr;
                Entry entry;
                entry.world = w;
                entry.model = model;
                entry.origin = std::wstring(spec_.game) + L", " + regions_[w] +
                               (section && section->info ? L" " + Widen(section->info->name) : std::wstring());
                models_.emplace(model->hash, std::move(entry));
            }
        }
        LOG_INFO("%S library: %zu models from %zu regions (%zu sky and backdrop pieces left out)", spec_.game,
                 models_.size(), worlds_.size(), backdrops);
    }

    void Unload() {
        if (loader_.joinable()) loader_.join();
        loading_ = false;
        enabled_ = false;
        nfs::RemoveForeignModels(this);
        // Nothing may still draw from the copies about to go.
        models_.clear();
        if (nfs::World* world = app_.World()) app_.Renderer().RebuildInstances(*world);
        gpu_.clear();
        converted_.clear();
        worlds_.clear();
        regions_.clear();
        app_.Catalog().Refresh();
    }

    // An object placed from these models and not written yet: it would be
    // lost with them.
    bool PlacedButNotSaved() const {
        const nfs::World* world = app_.World();
        if (!world) return false;
        for (const auto& section : world->Loaded())
            for (const auto& placement : section->placements) {
                const u32 hash = placement.instance.modelHash;
                if (world->ModelHome(hash) >= world->Loaded().size() && models_.count(hash)) return true;
            }
        return false;
    }

    U1LibraryAccess app_;
    const GameSpec& spec_;
    std::wstring folder_;
    bool enabled_ = false, loading_ = false;

    std::thread loader_;
    std::atomic<bool> done_{ false };
    std::mutex handover_;
    std::vector<std::unique_ptr<nfs::World>> read_;
    std::vector<std::wstring> readRegions_;

    std::vector<std::unique_ptr<nfs::World>> worlds_;
    std::vector<std::wstring> regions_;
    std::unordered_map<u32, Entry> models_;
    std::map<std::pair<size_t, size_t>, std::unique_ptr<GpuSection>> gpu_;
    mutable std::unordered_map<u32, std::vector<u8>> converted_;
};

} // namespace

std::unique_ptr<EditorExtension> CreateU1Library(MainWindow& window) {
    return std::make_unique<U1Library>(window, kUnderground1);
}

std::unique_ptr<EditorExtension> CreateU2Library(MainWindow& window) {
    return std::make_unique<U1Library>(window, kUnderground2);
}

int RunU1SelfTest(const std::wstring& folder) {
    const std::wstring tracks = folder + L"\\TRACKS";
    size_t models = 0, good = 0, bad = 0, textures = 0, texturesBad = 0;
    std::unordered_map<u32, bool> seen;
    for (const wchar_t* region : kRegions) {
        nfs::World world;
        world.SetForeignLookup(false);
        if (!world.Open(tracks, region)) continue;
        world.LoadAllSections();
        for (const nfs::SolidObject* model : world.AllModels()) {
            if (!model || seen.count(model->hash)) continue;
            seen[model->hash] = true;
            ++models;
            ByteSpan object;
            u64 offset = 0;
            std::vector<u8> bytes;
            std::wstring error;
            if (!FindModelChunk(world, model->hash, object, offset) || !ConvertU1Object(object, offset, bytes, error)) {
                if (bad++ < 30) LOG_WARN("U1 self-test %S %s: %S", region, model->name.c_str(), error.c_str());
                continue;
            }
            nfs::ChunkReader reader(ByteSpan{ bytes.data(), bytes.size() }, 0);
            nfs::Chunk chunk;
            nfs::SolidObject back;
            const bool parsed = reader.Next(chunk) && nfs::ParseSolidObject(chunk, back);
            const bool same = parsed && back.hash == model->hash && back.name == model->name &&
                              back.vertices.size() == model->vertices.size() && back.indices == model->indices &&
                              back.textureHashes == model->textureHashes &&
                              back.materials.size() == model->materials.size() && (back.version & 0xFF) == 0x16 &&
                              bytes.size() % 4 == 0;
            bool positions = same;
            for (size_t v = 0; positions && v < back.vertices.size(); ++v)
                positions = std::memcmp(&back.vertices[v].position, &model->vertices[v].position, 12) == 0 &&
                            std::memcmp(&back.vertices[v].uv, &model->vertices[v].uv, 8) == 0 &&
                            back.vertices[v].color == model->vertices[v].color;
            if (positions) {
                ++good;
            } else if (bad++ < 30) {
                LOG_WARN("U1 self-test %S %s: %s (vertices %zu/%zu, triangles %u/%u, textures %zu/%zu, version %X, "
                         "stride %u/%u)", region, model->name.c_str(), !parsed ? "does not parse" : same ? "vertices differ" : "DIFFERENT",
                         back.vertices.size(), model->vertices.size(), back.TriangleCount(), model->TriangleCount(),
                         back.textureHashes.size(), model->textureHashes.size(), back.version, back.vertexStride,
                         model->vertexStride);
            }
            for (u32 hash : model->textureHashes) {
                if (seen.count(hash ^ 0x80000000u)) continue;
                seen[hash ^ 0x80000000u] = true;
                const nfs::Texture* texture = world.FindDisplayTexture(hash);
                if (!texture) continue;   // a texture of the race tracks' own packs (STREAM<region>_<race>.BUN)
                ++textures;
                NewTexture copy;
                bool ok = false;
                if (texture->format == nfs::TextureFormat::Argb8888) ok = texture->width % 4 == 0 && texture->height % 4 == 0;
                else ok = CopyGameTexture(*texture, copy);
                if (!ok && texturesBad++ < 30)
                    LOG_WARN("U1 self-test texture %s (%ux%u, format %d) cannot be copied", texture->name.c_str(),
                             texture->width, texture->height, (int)texture->format);
            }
        }
        LOG_INFO("U1 self-test: %S done", region);
    }
    LOG_INFO("U1 self-test: %zu models, %zu convert and read back the same, %zu do not; %zu textures, %zu not copyable",
             models, good, bad, textures, texturesBad);
    return bad == 0 ? 0 : 2;
}

int RunU2ToU1SelfTest(const std::wstring& folder) {
    nfs::World world;
    world.SetForeignLookup(false);
    if (!world.Open(folder + L"\\TRACKS", L"L4RA")) {
        LOG_WARN("U2 to U1 self-test: L4RA could not be opened in %S", folder.c_str());
        return 1;
    }
    world.LoadAllSections();
    size_t models = 0, good = 0, bad = 0;
    for (const nfs::SolidObject* model : world.AllModels()) {
        if (!model) continue;
        ++models;
        ByteSpan object;
        u64 offset = 0;
        std::vector<u8> bytes;
        std::wstring error;
        if (!FindModelChunk(world, model->hash, object, offset) || !ConvertU2ObjectToU1(object, offset, bytes, error)) {
            if (bad++ < 30) LOG_WARN("U2 to U1 self-test %s: %S", model->name.c_str(), error.c_str());
            continue;
        }
        nfs::ChunkReader reader(ByteSpan{ bytes.data(), bytes.size() }, 0);
        nfs::Chunk chunk;
        nfs::SolidObject back;
        const bool parsed = reader.Next(chunk) && nfs::ParseSolidObject(chunk, back);
        bool same = parsed && back.hash == model->hash && back.name == model->name &&
                    back.vertices.size() == model->vertices.size() && back.indices == model->indices &&
                    back.textureHashes == model->textureHashes && back.materials.size() == model->materials.size() &&
                    (back.version & 0xFFFF) == 0x13 && bytes.size() % 4 == 0;
        for (size_t v = 0; same && v < back.vertices.size(); ++v)
            same = std::memcmp(&back.vertices[v].position, &model->vertices[v].position, 12) == 0 &&
                   std::memcmp(&back.vertices[v].uv, &model->vertices[v].uv, 8) == 0;
        // And on to Underground 2 again: the same bytes' meaning.
        std::vector<u8> again;
        if (same && !ConvertU1Object(ByteSpan{ bytes.data(), bytes.size() }, 0, again, error)) same = false;
        if (same) ++good;
        else if (bad++ < 30)
            LOG_WARN("U2 to U1 self-test %s: %s (vertices %zu/%zu, version %X, stride %u/%u) %S", model->name.c_str(),
                     parsed ? "DIFFERENT" : "does not parse", back.vertices.size(), model->vertices.size(), back.version,
                     back.vertexStride, model->vertexStride, error.c_str());
    }
    LOG_INFO("U2 to U1 self-test: %zu models, %zu convert and read back the same, %zu do not", models, good, bad);
    return bad == 0 ? 0 : 2;
}
