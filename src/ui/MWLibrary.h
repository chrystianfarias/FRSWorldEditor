#pragma once

#include "ui/EditorExtension.h"

#include <memory>

class MainWindow;

// Need for Speed Most Wanted's models in the model library.
//
// The Most Wanted folder set in File > Game Folder and Plugins (the "Your
// games" page) is read in the background, read only: every model of Rockport
// (L2RA) that the open region does not hold is listed in the Library beside
// the region's own -- the "Game" filter tells them apart -- with its picture,
// and can be placed like any other. The editor shows it from Most Wanted's
// copy; saving converts it to Underground 2's layout (io/MWConvert.h: diffuse
// texture only, no normal maps or wind) and puts it, with its textures, in
// the region's library (io/PlacementWrite.cpp), from where every section can
// place it. The same shape as ui/U1Library.h.
std::unique_ptr<EditorExtension> CreateMWLibrary(MainWindow& window);
