#include "ui/MWLibrary.h"

#include "core/Log.h"
#include "io/ModelCopy.h"
#include "io/TextureImport.h"
#include "io/MWConvert.h"
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
struct MWLibraryAccess {
    explicit MWLibraryAccess(MainWindow& window) : w(window) {}
    MainWindow& w;

    HWND Window() const { return w.window_; }
    HMENU Menu() const { return w.menu_; }
    WorldRenderer& Renderer() const { return w.viewport_.GetRenderer(); }
    ModelCatalog& Catalog() const { return w.catalog_; }
    nfs::World* World() const { return w.world_.get(); }
    void Status(const std::wstring& text) const { w.SetStatusText(2, text); }
};

namespace {


// What tells the games read this way apart.
struct GameSpec {
    const wchar_t* game;        // the Library's Game filter: "Most Wanted"
    const wchar_t* fullName;    // "Need for Speed Most Wanted"
    const wchar_t* folderKey;   // OtherGameFolder's name: "MostWanted"
    const char* log;            // "MW library"
    const wchar_t* region;      // the city's region
    nfs::Game kind;
    std::wstring (*detect)();
    bool (*valid)(const std::wstring&);
};

// Most Wanted has one region, Rockport; Carbon's city is Palmont (its L5RB is
// the front end).
const GameSpec kMostWanted{ L"Most Wanted", L"Need for Speed Most Wanted", L"MostWanted", "MW library", L"L2RA",
                            nfs::Game::MostWanted, DetectMostWantedFolder, IsMostWantedFolder };
const GameSpec kCarbon{ L"Carbon", L"Need for Speed Carbon", L"Carbon", "Carbon library", L"L5RA",
                        nfs::Game::Carbon, DetectCarbonFolder, IsCarbonFolder };

std::wstring Widen(const std::string& text) { return std::wstring(text.begin(), text.end()); }

// The extension itself.
class MWLibrary final : public EditorExtension, public RenderExtension, public nfs::ForeignModels {
public:
    MWLibrary(MainWindow& window, const GameSpec& spec) : app_(window), spec_(spec) {}

    ~MWLibrary() override {
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
                        ? L"No models were read from " + std::wstring(spec_.fullName) + L"'s folder; the log has why."
                        : std::wstring(spec_.fullName) + L": " + std::to_wstring(models_.size()) +
                              L" models in the library (Game filter: " + spec_.game + L").");
    }

    void OnRegionLoaded() override {
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
        if (found == models_.end()) { error = L"not one of " + std::wstring(spec_.fullName) + L"'s models"; return false; }
        auto converted = converted_.find(hash);
        if (converted == converted_.end()) {
            ByteSpan object;
            u64 offset = 0;
            if (!::FindModelChunk(*worlds_[found->second.world], hash, object, offset)) {
                error = L"its bytes were not found in " + found->second.origin;
                return false;
            }
            std::vector<u8> bytes;
            if (!ConvertMWObject(object, offset, bytes, error, spec_.kind)) return false;
            converted = converted_.emplace(hash, std::move(bytes)).first;
            LOG_INFO("%s: %s converted (%zu -> %zu bytes)", spec_.log, found->second.model->name.c_str(), object.size,
                     converted->second.size());
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
        std::wstring origin;   // "Most Wanted, L2RA X0"
    };

    // Reads the folder from the settings: its models in the library when it
    // is the game's, none when it is not set.
    void Reconfigure() {
        // Never set: the game is taken where it is installed, and kept --
        // File > Game Folder and Plugins changes it, or empties it for none.
        if (!HasOtherGameFolder(spec_.folderKey)) {
            const std::wstring found = spec_.detect();
            StoreOtherGameFolder(spec_.folderKey, found);
            if (!found.empty()) LOG_INFO("%s: %S found in %S", spec_.log, spec_.fullName, found.c_str());
        }
        const std::wstring folder = OtherGameFolder(spec_.folderKey);
        const bool wanted = spec_.valid(folder);
        if (wanted && folder == folder_ && (enabled_ || loading_)) return;
        if (!wanted && !enabled_) { folder_ = folder; return; }
        if (enabled_) {
            if (PlacedButNotSaved()) {
                app_.Status(L"Objects placed from " + std::wstring(spec_.fullName) +
                            L" are not saved yet: its models stay in the library until the next start.");
                return;
            }
            Unload();
            if (!wanted) app_.Status(std::wstring(spec_.fullName) + L"'s models are out of the library.");
        }
        folder_ = folder;
        if (wanted) StartLoading();
    }

    void StartLoading() {
        if (loading_) return;
        enabled_ = true;
        loading_ = true;
        done_ = false;
        app_.Status(L"Reading " + std::wstring(spec_.fullName) + L"'s models from " + folder_ + L"...");
        const std::wstring tracks = folder_ + L"\\TRACKS";
        loader_ = std::thread([this, tracks] {
            std::vector<std::unique_ptr<nfs::World>> worlds;
            std::vector<std::wstring> regions;
            for (const wchar_t* region : { spec_.region }) {
                if (::GetFileAttributesW((tracks + L"\\STREAM" + region + L".BUN").c_str()) == INVALID_FILE_ATTRIBUTES)
                    continue;
                auto world = std::make_unique<nfs::World>();
                world->SetForeignLookup(false);
                if (!world->Open(tracks, region)) {
                    LOG_WARN("%s: %S could not be opened", spec_.log, region);
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
        LOG_INFO("%s: %zu models from %zu regions (%zu sky and backdrop pieces left out)", spec_.log, models_.size(),
                 worlds_.size(), backdrops);
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

    MWLibraryAccess app_;
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

std::unique_ptr<EditorExtension> CreateMWLibrary(MainWindow& window) {
    return std::make_unique<MWLibrary>(window, kMostWanted);
}

std::unique_ptr<EditorExtension> CreateCarbonLibrary(MainWindow& window) {
    return std::make_unique<MWLibrary>(window, kCarbon);
}
