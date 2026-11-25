#pragma once

#include "roads/RoadNetwork.h"

namespace nfs { class World; }

namespace roads {

enum class WorldRoadType { TwoLane, FourLane, Avenue, SixLane, OneWay, Highway, Tunnel, Count };

// Material defaults for the road types in the shared UG2 drawing tool.
void ApplyWorldRoadMaterials(const nfs::World& world, WorldRoadType type, RoadProfile& profile);
// Refit an existing road without changing its authored lane count, width or elevation.
void ApplyWorldRoadSurface(const nfs::World& world, RoadProfile& profile);
bool IsWorldLaneTexture(const std::string& texture);
u32 ResolveWorldRoadTexture(const nfs::World& world, const std::string& name, u32 hash,
                            bool preferWorldTextures);

} // namespace roads
