#include "roads/NfswRoadTemplate.h"

#include "io/TextureImport.h"
#include "nfs/World.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <string_view>

namespace roads {
namespace {

struct Slice {
    const nfs::Texture* texture;
    float left, right, leftHeight, rightHeight;
    Vec2 leftUv, rightUv, advance;
};

struct Sample {
    std::vector<RoadProfile::SurfaceBand> bands;
    float width = 0.0f;
    float sidewalkWidth = 0.0f, sidewalkHeight = 0.0f;
    u32 ambient = 0x777777;
};

Vec3 Flat(Vec3 v) { return { v.x, v.y, 0.0f }; }
Vec2 Difference(const Vec2& a, const Vec2& b) { return { a.x - b.x, a.y - b.y }; }

// Slice the original paved triangles across their texture's longitudinal
// direction. A cut through a triangle also yields its UV change per metre.
std::optional<Sample> SamplePavement(const nfs::World& world, const nfs::SolidObject& model,
                                     size_t courses) {
    struct Triangle {
        const nfs::Texture* texture;
        std::array<const nfs::SolidVertex*, 3> vertices;
        bool sidewalk;
    };
    std::vector<Triangle> triangles;
    std::vector<size_t> anchors;
    std::vector<unsigned> brightness;
    for (const auto& material : model.materials) {
        if (!material.nfswRoadSurface || material.textureIndex >= model.textureHashes.size()) continue;
        const auto* texture = world.FindTexture(model.textureHashes[material.textureIndex]);
        if (!texture) continue;
        const bool sidewalk = texture->name.find("SIDEWALK") != std::string::npos;
        if (!sidewalk && (!IsWorldLaneTexture(texture->name) ||
            texture->name.find("_PED_") != std::string::npos ||
            texture->name.find("_INT_") != std::string::npos)) continue;
        const size_t end = std::min<size_t>((size_t)material.firstIndex + material.indexCount, model.indices.size());
        for (size_t i = material.firstIndex; i + 2 < end; i += 3) {
            Triangle triangle{ texture, { &model.vertices[model.indices[i]], &model.vertices[model.indices[i + 1]],
                                          &model.vertices[model.indices[i + 2]] }, sidewalk };
            const auto& v = triangle.vertices;
            const Vec3 normal = Cross(v[1]->position - v[0]->position, v[2]->position - v[0]->position);
            if (Length(Flat(normal)) > std::abs(normal.z)) continue;
            if (!sidewalk && texture->name.find("_YL_") != std::string::npos) anchors.push_back(triangles.size());
            triangles.push_back(triangle);
            if (!sidewalk)
                for (const auto* corner : triangle.vertices) brightness.push_back((corner->color >> 8) & 255);
        }
    }
    std::optional<Sample> best;
    float bestScore = 1e30f;
    for (size_t seed : anchors) {
        const auto& v = triangles[seed].vertices;
        const Vec2 duv1 = Difference(v[1]->uv, v[0]->uv), duv2 = Difference(v[2]->uv, v[0]->uv);
        const float uvArea = duv1.x * duv2.y - duv1.y * duv2.x;
        if (std::abs(uvArea) < 1e-5f) continue;
        const Vec3 forward = Flat((v[2]->position - v[0]->position) * duv1.x -
                                  (v[1]->position - v[0]->position) * duv2.x) * (1.0f / uvArea);
        if (Length(forward) < 5.0f || Length(forward) > 150.0f) continue;
        const Vec3 along = Normalize(forward), across{ along.y, -along.x, 0.0f };
        const Vec3 origin = (v[0]->position + v[1]->position + v[2]->position) * (1.0f / 3.0f);
        std::vector<Slice> slices, sidewalks;
        for (const auto& triangle : triangles) {
            const auto& corner = triangle.vertices;
            float s[3], x[3];
            for (int i = 0; i < 3; ++i) {
                const Vec3 offset = corner[i]->position - origin;
                s[i] = Dot(offset, along);
                x[i] = Dot(offset, across);
            }
            if (std::min({ s[0], s[1], s[2] }) >= 0 || std::max({ s[0], s[1], s[2] }) <= 0) continue;
            const float area = (s[1] - s[0]) * (x[2] - x[0]) - (s[2] - s[0]) * (x[1] - x[0]);
            if (std::abs(area) < 1e-4f) continue;
            const Vec2 uv1 = Difference(corner[1]->uv, corner[0]->uv), uv2 = Difference(corner[2]->uv, corner[0]->uv);
            Slice slice{ triangle.texture, 1e30f, -1e30f, 0, 0, {}, {},
                { (uv1.x * (x[2] - x[0]) - uv2.x * (x[1] - x[0])) / area,
                  (uv1.y * (x[2] - x[0]) - uv2.y * (x[1] - x[0])) / area } };
            for (int i = 0; i < 3; ++i) {
                const int j = (i + 1) % 3;
                if ((s[i] < 0) == (s[j] < 0)) continue;
                const float t = -s[i] / (s[j] - s[i]);
                const float at = x[i] + (x[j] - x[i]) * t;
                const float height = corner[i]->position.z + (corner[j]->position.z - corner[i]->position.z) * t;
                const Vec2 uv{ corner[i]->uv.x + (corner[j]->uv.x - corner[i]->uv.x) * t,
                               corner[i]->uv.y + (corner[j]->uv.y - corner[i]->uv.y) * t };
                if (at < slice.left) { slice.left = at; slice.leftHeight = height; slice.leftUv = uv; }
                if (at > slice.right) { slice.right = at; slice.rightHeight = height; slice.rightUv = uv; }
            }
            if (slice.right - slice.left > 0.002f)
                (triangle.sidewalk ? sidewalks : slices).push_back(slice);
        }
        std::sort(slices.begin(), slices.end(), [](const auto& a, const auto& b) { return a.left < b.left; });
        std::vector<Slice> bands;
        bool contiguous = true;
        for (const auto& slice : slices) {
            if (!bands.empty()) {
                auto& last = bands.back();
                if (std::abs(last.right - slice.left) > 0.08f) { contiguous = false; break; }
                const Vec2 seam = Difference(last.rightUv, slice.leftUv);
                const Vec2 slope = Difference(last.advance, slice.advance);
                if (last.texture == slice.texture && std::abs(seam.x) < 0.035f && std::abs(seam.y) < 0.035f &&
                    std::abs(slope.x) < 0.005f && std::abs(slope.y) < 0.005f) {
                    last.right = slice.right;
                    last.rightHeight = slice.rightHeight;
                    last.rightUv = slice.rightUv;
                    continue;
                }
            }
            bands.push_back(slice);
        }
        if (!contiguous || bands.size() != courses) continue;
        const float width = bands.back().right - bands.front().left;
        if (width < (float)courses * 1.5f || width > (float)courses * 8.5f) continue;
        const float middle = bands.front().left + width * 0.5f;
        float reference = origin.z;
        float score = 0;
        for (const auto& band : bands) {
            if (middle >= band.left && middle <= band.right)
                reference = band.leftHeight + (band.rightHeight - band.leftHeight) *
                            ((middle - band.left) / (band.right - band.left));
            score += std::abs(band.advance.x) + std::abs(std::abs(band.advance.y) - 1.0f / 40.0f);
        }
        if (score >= bestScore) continue;
        bestScore = score;
        Sample sample;
        sample.width = width;
        const float leftEdge = bands.front().left, rightEdge = bands.back().right;
        float walkLeft = leftEdge, walkRight = rightEdge;
        float kerbSum = 0.0f;
        unsigned kerbs = 0;
        for (const auto& walk : sidewalks) {
            if (walk.right < leftEdge + 0.2f && walk.left > leftEdge - 10.0f)
                walkLeft = std::min(walkLeft, walk.left);
            if (walk.left > rightEdge - 0.2f && walk.right < rightEdge + 10.0f)
                walkRight = std::max(walkRight, walk.right);
            if (std::abs(walk.right - leftEdge) < 0.15f) {
                kerbSum += std::max(0.0f, walk.rightHeight - bands.front().leftHeight);
                ++kerbs;
            }
            if (std::abs(walk.left - rightEdge) < 0.15f) {
                kerbSum += std::max(0.0f, walk.leftHeight - bands.back().rightHeight);
                ++kerbs;
            }
        }
        if (kerbs) {
            sample.sidewalkWidth = (leftEdge - walkLeft + walkRight - rightEdge) / kerbs;
            sample.sidewalkHeight = kerbSum / kerbs;
        }
        for (size_t i = 0; i < bands.size(); ++i) {
            const auto& band = bands[i];
            const Vec2 phase{ std::floor(band.leftUv.x), std::floor(band.leftUv.y) };
            const float left = i == 0 ? -0.5f : sample.bands.back().right;
            sample.bands.push_back({ band.texture->name, left, (band.right - middle) / width,
                band.leftHeight - reference, band.rightHeight - reference,
                Difference(band.leftUv, phase), Difference(band.rightUv, phase), band.advance });
        }
        best = std::move(sample);
    }
    if (best && !brightness.empty()) {
        auto mid = brightness.begin() + brightness.size() / 2;
        std::nth_element(brightness.begin(), mid, brightness.end());
        best->ambient = *mid * 0x010101;
    }
    return best;
}

struct NativeSource {
    const char* name;
    size_t courses;
};

constexpr std::array<NativeSource, (size_t)WorldRoadType::Count> kSources{{
    { "RD_CT_ROAD_52_CHOP_A74_R0", 2 },
    { "RD_CT_ROADCONNECT_04_CHOP_A123_R0", 4 },
    { "RD_CT_ROAD_04_CHOP_A44_R0", 5 },
    { "RD_CT_ROAD_07_CHOP_A34_R0", 7 },
    { "RD_FWY_ROAD_06_CHOP_B97_R0", 4 },
    { "RD_FWY_ROAD_43_CHOP_D89_R0", 5 },
    { "RD_FWY_ROAD_06_CHOP_B97_R0", 4 },
}};

bool ApplyNativeSurface(const nfs::World& world, WorldRoadType type, RoadProfile& profile, bool resize) {
    const auto& source = kSources[(size_t)type];
    const auto* model = world.FindModel(TextureNameHash(source.name));
    if (!model) return false;
    auto sampled = SamplePavement(world, *model, source.courses);
    if (!sampled) return false;
    if (type == WorldRoadType::Tunnel) {
        const auto painted = std::find_if(sampled->bands.begin(), sampled->bands.end(), [](const auto& band) {
            return band.texture.find("_YL_") != std::string::npos;
        });
        if (painted == sampled->bands.end()) return false;
        auto right = *painted;
        sampled->width *= (right.right - right.left) * 2.0f;
        if (right.leftUv.x > right.rightUv.x) {
            std::swap(right.leftUv, right.rightUv);
            std::swap(right.leftHeight, right.rightHeight);
        }
        right.rightHeight -= right.leftHeight;
        right.leftHeight = 0.0f;
        right.left = 0; right.right = 0.5f;
        auto left = right;
        left.left = -0.5f; left.right = 0;
        std::swap(left.leftUv, left.rightUv);
        std::swap(left.leftHeight, left.rightHeight);
        sampled->bands = { std::move(left), std::move(right) };
    }
    if (resize && profile.worldLaneProfile < 0 && profile.Lanes()) {
        const auto width = [&](const auto& band) { return (band.right - band.left) * sampled->width; };
        if (type == WorldRoadType::OneWay || type == WorldRoadType::Highway) {
            profile.shoulderType = 2;
            profile.shoulderWidth = (width(sampled->bands.front()) + width(sampled->bands.back())) * 0.5f;
            profile.shoulderSides = 0;
            profile.shoulderLine = false;
        }
        if (type == WorldRoadType::Avenue || type == WorldRoadType::SixLane) {
            const auto& median = sampled->bands[sampled->bands.size() / 2];
            if (median.texture.find("_YDIAL_") != std::string::npos) {
                profile.medianType = 1;
                profile.medianWidth = width(median);
                profile.medianTexture = median.texture;
            }
        }
        if (type <= WorldRoadType::SixLane && sampled->sidewalkWidth > 0.1f) {
            profile.sidewalkWidth = sampled->sidewalkWidth;
            profile.sidewalkHeight = sampled->sidewalkHeight;
        }
        profile.laneWidth = std::max(1.0f, (sampled->width - profile.MedianWidth() -
            profile.ShoulderAt(-1) - profile.ShoulderAt(1)) / profile.Lanes());
    }
    profile.surfaceTemplate = model->name;
    profile.surfaceBands = std::move(sampled->bands);
    if (profile.ambient == 0x383C42) profile.ambient = sampled->ambient;
    if (profile.textureLength == 32.0f) profile.textureLength = 40.0f;
    return true;
}

void ApplySurfaceDefaults(const nfs::World& world, WorldRoadType type, RoadProfile& profile, bool resize) {
    if (!world.IsWorld()) return;
    struct Materials {
        const char* centre;
        const char* inner;
        const char* sidewalk;
    };
    constexpr std::array<Materials, (size_t)WorldRoadType::Count> presets{{
        { "TRN_PAVA_YL_01_D",            "TRN_PAVA_WD_01_D",            "TRN_SIDEWALK_CLEAN_01_D" },
        { "TRN_PAVB_YL_01_D",            "TRN_PAVB_WD_01_D",            "TRN_SIDEWALKB_01_D" },
        { "TRN_PAVA_YL_01_D",            "TRN_PAVA_WD_01_D",            "TRN_SIDEWALKB_01_D" },
        { "TRN_PAVB_YL_01_D",            "TRN_PAVB_WD_01_D",            "TRN_SIDEWALK_CLEAN_01_D" },
        { "TRN_PAVA_YL_01_D",            "TRN_PAVA_WD_01_D",            "TRN_SIDEWALK_CLEAN_01_D" },
        { "TRN_PAVB_YL_01_D",            "TRN_PAVB_WD_01_D",            "TRN_SIDEWALKB_01_D" },
        { "TRN_CONCRETEHORIZCUT_YL_01_D", "TRN_CONCRETEHORIZCUT_WD_01_D", "TRN_SIDEWALK_CLEAN_01_D" },
    }};
    const size_t index = (size_t)type;
    if (index >= presets.size()) return;
    const auto& materials = presets[index];
    const auto use = [&](std::string& field, const char* name) {
        if ((resize || field.empty() || !world.FindTexture(TextureNameHash(field))) &&
            world.FindTexture(TextureNameHash(name))) field = name;
    };
    use(profile.laneCentre, materials.centre);
    use(profile.laneInner, materials.inner);
    use(profile.sidewalkTexture, materials.sidewalk);
    if (profile.medianType == 3) use(profile.medianTexture, "TRN_GRASS_CLEANCUT_01_NG_D");
    if (profile.shoulderType == 2) use(profile.shoulderTexture, "TRN_CONCRETEHORIZCUT_WDIAL_01_D");
    if (profile.wallLeft || profile.wallRight) use(profile.wallTexture, "ARC_CONCRETEWALL_02_D");
    if (type == WorldRoadType::Tunnel) {
        use(profile.tunnelWall, "ARC_CONCRETEWALL_02_D");
        use(profile.tunnelRoof, "ARC_CONCRETEWALL_02_D");
        use(profile.tunnelPortal, "ARC_CONCRETEWALL_02_D");
    }
    if (!ApplyNativeSurface(world, type, profile, resize) && profile.ambient == 0x383C42)
        profile.ambient = 0x777777;
}

} // namespace

void ApplyWorldRoadMaterials(const nfs::World& world, WorldRoadType type, RoadProfile& profile) {
    ApplySurfaceDefaults(world, type, profile, true);
}

void ApplyWorldRoadSurface(const nfs::World& world, RoadProfile& profile) {
    if (!world.IsWorld() || !profile.surfaceBands.empty()) return;
    const auto type = profile.tunnel ? WorldRoadType::Tunnel :
        profile.lanesForward == 0 || profile.lanesBackward == 0
            ? (profile.Lanes() >= 3 ? WorldRoadType::Highway : WorldRoadType::OneWay) :
        profile.Lanes() >= 6 ? WorldRoadType::SixLane :
        profile.MedianWidth() > 0.1f ? WorldRoadType::Avenue :
        profile.Lanes() >= 4 ? WorldRoadType::FourLane : WorldRoadType::TwoLane;
    ApplySurfaceDefaults(world, type, profile, false);
}

bool IsWorldLaneTexture(const std::string& name) {
    const std::string_view texture = name;
    if (texture.rfind("TRN_PAVA_", 0) == 0 || texture.rfind("TRN_PAVB_", 0) == 0 ||
        texture.rfind("TRN_CONCRETEHORIZCUT_", 0) == 0)
        return texture.size() >= 2 && texture.substr(texture.size() - 2) == "_D";
    if (texture == "TRS_PAVA_01_D" || texture == "TRS_PAVB_INT_01_D") return true;
    if (texture.rfind("TRNS_", 0) != 0 || texture.find("_PAVEMENT") == std::string_view::npos) return false;
    return texture.find("_NORMAL") == std::string_view::npos && texture.find("_SPEC") == std::string_view::npos;
}

u32 ResolveWorldRoadTexture(const nfs::World& world, const std::string& name, u32 hash,
                            bool preferWorldTextures) {
    if (preferWorldTextures) {
        if (const u32 stock = world.NativeRoadTexture(name)) return stock;
    }
    if (world.FindTexture(hash)) return hash;
    return preferWorldTextures ? 0 : world.NativeRoadTexture(name);
}

} // namespace roads
