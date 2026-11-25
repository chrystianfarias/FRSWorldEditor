#pragma once

#include "io/NfswPlacementWrite.h"
#include "io/WorldFile.h"

namespace roads {

class Network;

struct WorldRoadBake {
    io::WorldFile journal;
    std::vector<NfswMeshOverlay> installed;
};

bool PlanWorldRoadBake(const nfs::World& world, const Network& network,
                       std::vector<NfswMeshOverlay>& files, nfs::NfswRoadBuildReport& report,
                       std::wstring& error);
bool InstallWorldRoadBake(const nfs::World& world, const std::vector<NfswMeshOverlay>& files,
                          std::wstring& error);
bool RemoveWorldRoadBake(const nfs::World& world, WorldRoadBake& previous, std::wstring& error);
bool ReplayWorldRoadBake(const nfs::World& world, const WorldRoadBake& previous, std::wstring& error);

} // namespace roads
