#pragma once

#include "ui/EditorExtension.h"

#include <memory>

class MainWindow;

// Need for Speed ProStreet's models in the model library.
//
// The ProStreet folder (found where the game is installed on first start, or
// set in File > Game Folder and Plugins) is read in the background, read only:
// every model of its tracks (TRACKS\L6R_*) that the open region does not hold
// is listed in the Library beside the region's own -- the "Game" filter tells
// them apart -- and can be placed like any other. Saving converts it to
// Underground 2's layout (io/MWConvert.h: diffuse texture only) and puts it,
// with its textures, in the region's library. The same shape as
// ui/MWLibrary.h.
std::unique_ptr<EditorExtension> CreatePSLibrary(MainWindow& window);
