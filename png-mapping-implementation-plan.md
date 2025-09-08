# PNG Mapping Implementation Plan for Mercator Origins

This document outlines a phased approach to implementing PNG-based mapping, starting with internal flash storage and progressing to advanced tiled systems.

## Overview

The implementation follows a three-phase approach:

1. **Phase 1**: PNG files in internal flash (LittleFS) - maintains existing mapping code
2. **Phase 2**: Advanced tiled scrolling system with deadzone navigation  
3. **Phase 3**: SD card integration and multi-resolution support

This phased approach allows incremental improvements while maintaining system stability at each stage.

## Phase 1: PNG Files in Internal Flash (LittleFS)

### Objectives
- Replace compiled uint16_t arrays with PNG files stored in LittleFS
- Maintain existing MapScreen_T4 API and zoom functionality
- Significantly reduce OTA upload times
- Enable easy map updates without firmware recompilation
- Reduce flash usage through PNG compression

### Benefits
- ✅ **Faster OTA updates** - map files not included in firmware binary
- ✅ **95% storage reduction** - PNG compression vs raw pixel arrays
- ✅ **Easy map updates** - upload new PNG files without code changes
- ✅ **More maps possible** - efficient storage allows more dive sites
- ✅ **Backward compatible** - maintains existing zoom and navigation logic
- ✅ **No code complexity** - minimal changes to existing system

### Implementation Strategy

#### Current System Analysis
```
Current: 6 maps × 600×450×2 bytes = 3.24MB compiled arrays
Target:  6 maps × ~30KB PNG = ~180KB LittleFS files
Savings: ~3MB flash space + faster OTA uploads
```

#### File Structure
```
/littlefs/maps/
  wraysbury_all.png     # Overview map (600×450)
  wraysbury_n.png       # North region
  wraysbury_w.png       # West region  
  wraysbury_sw.png      # Southwest region
  wraysbury_s.png       # South region
  wraysbury_se.png      # Southeast region
```

#### Code Integration
```cpp
class PNGMapCache {
private:
    struct CachedMap {
        uint16_t* pixels = nullptr;    // 540KB per 600×450 map
        bool loaded = false;
        uint32_t lastUsed = 0;
        String filename;
    };
    
    CachedMap mapCache[6];             // Cache for all 6 Wraysbury maps
    static const int MAX_CACHED_MAPS = 3;  // Keep 2-3 in RAM simultaneously
    
public:
    uint16_t* getMapPixels(int mapIndex) {
        if (!mapCache[mapIndex].loaded) {
            loadMapFromLittleFS(mapIndex);
        }
        mapCache[mapIndex].lastUsed = millis();
        return mapCache[mapIndex].pixels;
    }
    
private:
    void loadMapFromLittleFS(int mapIndex) {
        // Free oldest map if cache full
        if (getCachedMapCount() >= MAX_CACHED_MAPS) {
            evictOldestMap();
        }
        
        // Load and decompress PNG
        File pngFile = LittleFS.open(mapCache[mapIndex].filename, "r");
        decompressPNGToCache(pngFile, mapIndex);
        mapCache[mapIndex].loaded = true;
    }
};

// Integration with existing MapScreen_T4
void MapScreen_T4::drawMapBackground(int mapIndex) {
    uint16_t* mapPixels = pngMapCache.getMapPixels(mapIndex);
    
    // Use existing zoom logic - no changes needed
    tft.pushImageScaled(0, 0, 600, 450, currentZoom, 
                       zoomOffsetX, zoomOffsetY, mapPixels, false);
}
```

#### PNG Decompression
```cpp
// PNG decode callback for RGB565 conversion
uint16_t* currentDecompressBuffer = nullptr;
uint32_t currentBufferWidth = 0;

void pngDrawCallback(PNGDRAW *pDraw) {
    for (int x = 0; x < pDraw->iWidth; x++) {
        // PNG provides RGB888, convert to RGB565 for TFT_eSPI
        uint8_t r = pDraw->pPixels[x * 3];
        uint8_t g = pDraw->pPixels[x * 3 + 1];
        uint8_t b = pDraw->pPixels[x * 3 + 2];
        
        // Convert to RGB565 format
        uint16_t rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
        
        // Store in buffer
        uint32_t pixelIndex = (pDraw->y * currentBufferWidth) + x;
        currentDecompressBuffer[pixelIndex] = rgb565;
    }
}

void decompressPNGToCache(File& pngFile, int mapIndex) {
    PNG png;
    if (png.open(pngFile, pngDrawCallback) == PNG_SUCCESS) {
        // Allocate PSRAM buffer
        uint32_t pixelCount = png.getWidth() * png.getHeight();
        mapCache[mapIndex].pixels = (uint16_t*)ps_malloc(pixelCount * 2);
        
        // Set up globals for callback
        currentDecompressBuffer = mapCache[mapIndex].pixels;
        currentBufferWidth = png.getWidth();
        
        // Decompress PNG to RGB565 buffer
        png.decode(nullptr, 0);
    }
    png.close();
}
```

### Memory Usage
- **PNG files**: 180KB in LittleFS flash
- **Runtime cache**: 1.6MB PSRAM (3 maps × 540KB each)
- **Flash savings**: 3MB+ available for code and other features
- **Total memory**: Similar to current system but more efficient

### Performance
- **Map load time**: ~100ms per 600×450 PNG (only when not cached)
- **Cache hit rate**: ~85% with 3-map cache for normal diving patterns
- **Zoom operations**: Identical to current system (instant)
- **4 FPS capability**: Easily achieved

## File Upload Methods for LittleFS

### Method 1: Web-Based File Manager (Recommended)

#### Implementation
```cpp
// Add to existing OTA web server
void setupFileManagerEndpoints() {
    // File upload endpoint
    server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "Upload complete");
    }, handleFileUpload);
    
    // File browser endpoint  
    server.on("/files", HTTP_GET, [](AsyncWebServerRequest *request) {
        String html = generateFileListHTML();
        request->send(200, "text/html", html);
    });
    
    // Delete file endpoint
    server.on("/delete", HTTP_DELETE, [](AsyncWebServerRequest *request) {
        String filename = request->getParam("file")->value();
        LittleFS.remove("/" + filename);
        request->send(200, "text/plain", "Deleted: " + filename);
    });
}

void handleFileUpload(AsyncWebServerRequest *request, String filename, 
                     size_t index, uint8_t *data, size_t len, bool final) {
    static File uploadFile;
    
    if (index == 0) {
        // Start upload
        uploadFile = LittleFS.open("/maps/" + filename, "w");
    }
    
    if (len) {
        uploadFile.write(data, len);
    }
    
    if (final) {
        uploadFile.close();
        Serial.println("Upload complete: " + filename);
    }
}
```

#### File Manager HTML Interface
```html
<!DOCTYPE html>
<html>
<head><title>Mercator Origins - Map File Manager</title></head>
<body>
    <h1>Map File Manager</h1>
    
    <!-- File Upload Form -->
    <form action="/upload" method="post" enctype="multipart/form-data">
        <input type="file" name="mapfile" accept=".png" multiple>
        <input type="submit" value="Upload PNG Maps">
    </form>
    
    <!-- File List -->
    <h2>Current Map Files</h2>
    <div id="fileList">
        <!-- Populated by JavaScript -->
    </div>
    
    <script>
        // Fetch and display file list
        fetch('/files').then(r => r.text()).then(html => {
            document.getElementById('fileList').innerHTML = html;
        });
    </script>
</body>
</html>
```

#### Benefits
- ✅ **Browser-based** - works on any device with WiFi
- ✅ **Drag & drop** - easy file upload interface
- ✅ **File management** - view, delete, replace files
- ✅ **Integrated** - uses existing OTA web server
- ✅ **No additional tools** - works with standard web browser

### Method 2: FTP Server Integration

Based on your existing FTP implementation at https://github.com/scuba-hacker/mercator-origins-silky-io:

#### Integration
```cpp
#include <ESP32FtpServer.h>

FtpServer ftpSrv;   // Global FTP server instance

void setupFTPServer() {
    if (LittleFS.begin()) {
        // Start FTP server on LittleFS
        ftpSrv.begin("oceanic", "mercator");  // username, password
        
        Serial.println("FTP Server started");
        Serial.println("Connect to: " + WiFi.localIP().toString());
        Serial.println("Username: oceanic, Password: mercator");
    }
}

void loop() {
    // Handle FTP requests
    ftpSrv.handleFTP();
    
    // Your existing loop code...
}
```

#### Usage
```bash
# Connect with any FTP client
ftp 192.168.1.xxx
Username: oceanic
Password: mercator

# Navigate to maps directory
cd /maps

# Upload PNG files
put wraysbury_all.png
put wraysbury_n.png
# ... upload all map files

# List files to verify
ls -la
```

#### Benefits
- ✅ **Familiar interface** - standard FTP clients (FileZilla, WinSCP, etc.)
- ✅ **Batch uploads** - multiple files at once
- ✅ **Reliable transfers** - FTP protocol handles interruptions
- ✅ **File management** - rename, delete, organize files
- ✅ **Proven solution** - based on your existing implementation

### Method 3: Platform.IO Data Upload

#### Configuration
```ini
# In platformio.ini
[env:oceanic]
platform = espressif32
framework = arduino
upload_protocol = esptool
filesystem = littlefs

# Data directory for filesystem files
data_dir = data/

# Custom upload command for filesystem
extra_scripts = upload_fs.py
```

#### Directory Structure
```
project_root/
├── src/
│   └── main.cpp
├── data/                    # Files to upload to LittleFS
│   └── maps/
│       ├── wraysbury_all.png
│       ├── wraysbury_n.png
│       ├── wraysbury_w.png
│       ├── wraysbury_sw.png
│       ├── wraysbury_s.png
│       └── wraysbury_se.png
└── platformio.ini
```

#### Upload Script (upload_fs.py)
```python
Import("env")

def upload_filesystem(source, target, env):
    print("Uploading filesystem image...")
    env.Execute("$PYTHONEXE -m platformio run -t uploadfs")

env.AddCustomTarget("uploadfs", upload_filesystem)
```

#### Commands
```bash
# Upload filesystem only (maps)
pio run -t uploadfs

# Upload code and filesystem
pio run -t upload
pio run -t uploadfs
```

#### Benefits
- ✅ **Development integration** - part of build process
- ✅ **Version control** - maps stored with code
- ✅ **Repeatable** - consistent deployment
- ✅ **Automated** - can be scripted

#### Drawbacks
- ❌ **Requires rebuild** - filesystem changes need recompilation
- ❌ **Development focus** - not ideal for field updates
- ❌ **USB connection** - requires physical access to device

### Method 4: OTA Filesystem Update

#### Implementation
```cpp
void setupOTAFilesystem() {
    // Custom endpoint for filesystem updates
    server.on("/update_fs", HTTP_POST, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "Filesystem update complete");
    }, handleFilesystemOTA);
}

void handleFilesystemOTA(AsyncWebServerRequest *request, String filename, 
                        size_t index, uint8_t *data, size_t len, bool final) {
    static bool updateStarted = false;
    
    if (index == 0) {
        // Start filesystem update
        if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_SPIFFS)) {
            request->send(500, "text/plain", "Filesystem update failed");
            return;
        }
        updateStarted = true;
    }
    
    if (len) {
        if (Update.write(data, len) != len) {
            request->send(500, "text/plain", "Write failed");
            return;
        }
    }
    
    if (final) {
        if (Update.end()) {
            Serial.println("Filesystem update successful");
            ESP.restart();  // Restart to mount new filesystem
        }
    }
}
```

#### Benefits
- ✅ **Over-the-air** - no physical access needed
- ✅ **Complete filesystem** - updates entire LittleFS partition
- ✅ **Remote deployment** - field updates possible

#### Drawbacks
- ❌ **Complex** - requires filesystem image creation
- ❌ **All-or-nothing** - replaces entire filesystem
- ❌ **Risk of corruption** - failed update can brick filesystem

## Recommended Approach

### Primary Method: Web-Based File Manager
**Use for**: Regular map updates, field deployment, user-friendly management

### Secondary Method: FTP Server  
**Use for**: Batch uploads, development, power users familiar with FTP

### Development Method: Platform.IO Data Upload
**Use for**: Initial deployment, version-controlled map sets

## Implementation Timeline - Phase 1

### Week 1: Infrastructure
- [ ] Add PNGdec library to project
- [ ] Implement PNG to RGB565 conversion functions  
- [ ] Create PNGMapCache class
- [ ] Set up LittleFS filesystem

### Week 2: Integration
- [ ] Modify MapScreen_T4 to use PNG cache
- [ ] Test existing zoom/navigation functionality
- [ ] Implement cache management (LRU eviction)
- [ ] Performance testing and optimization

### Week 3: File Upload Systems
- [ ] Implement web-based file manager
- [ ] Add FTP server support
- [ ] Create Platform.IO data upload workflow
- [ ] Documentation and testing

### Week 4: Testing & Deployment
- [ ] Convert existing Wraysbury maps to PNG
- [ ] Test all 6 maps with zoom functionality
- [ ] Validate memory usage and performance
- [ ] Field testing with real diving scenarios

## Phase 1 Success Criteria

- ✅ All existing map functionality preserved
- ✅ PNG maps load and display correctly
- ✅ Zoom (x1, x2, x3, x4) works identically to current system
- ✅ Map switching maintains current behavior
- ✅ Memory usage ≤ current system
- ✅ 4 FPS performance maintained
- ✅ OTA updates significantly faster
- ✅ Easy file upload/management working

## Phase 2: Tiled Scrolling Map System

### Objectives
- Replace discrete map switching with smooth scrolling navigation
- Implement professional deadzone-based viewport control
- Enable coverage of arbitrarily large areas
- Provide manual re-center functionality
- Maintain 4 FPS performance with complex overlay rendering

### System Architecture

#### Tile Structure
```cpp
struct MapTile {
    uint16_t* pixels = nullptr;     // 256×256 pixels = 128KB per tile
    bool loaded = false;
    int tileX, tileY;               // Tile coordinates in world grid
    uint32_t lastUsed = 0;         // For LRU cache management
    int priority = 0;               // Cache priority (0-3, 3=highest)
    String filename;                // e.g., "wraysbury_5_12.png"
};

class TiledMapRenderer {
private:
    static const int TILE_SIZE = 256;           // 256×256 pixel tiles
    static const int MAX_CACHED_TILES = 12;    // 1.5MB cache - optimal for smooth scrolling
    static const int DEADZONE_SIZE = 100;      // 100×100 center area for diver roaming
    
    MapTile tileCache[MAX_CACHED_TILES];
    
    // World coordinates (pixel coordinates in the large map)
    int viewportX = 0, viewportY = 0;          // Top-left of current viewport
    int diverWorldX = 0, diverWorldY = 0;      // Diver position in world coordinates
    
    // Display buffers for efficient overlay rendering
    uint16_t* backgroundBuffer;                // 540KB - composed from tiles
    uint16_t* displayBuffer;                   // 540KB - final composite with overlays
    
    // Re-center functionality
    bool manualCenterRequested = false;
};
```

#### File Structure
```
/littlefs/tiles/
  wraysbury_0_0.png    # Northwest tile (256×256, ~15KB)
  wraysbury_0_1.png    # North-center tile
  wraysbury_0_2.png    # Northeast tile  
  wraysbury_1_0.png    # West tile
  wraysbury_1_1.png    # Center tile
  wraysbury_1_2.png    # East tile
  wraysbury_2_0.png    # Southwest tile
  wraysbury_2_1.png    # South-center tile
  wraysbury_2_2.png    # Southeast tile
  ...                  # Grid expands as needed
```

### Deadzone Navigation System

#### Core Principle
The diver **roams freely** within a 100×100 pixel deadzone at screen center. Map scrolling only occurs when the diver reaches the deadzone boundary, providing stable reference during local exploration.

```cpp
void updateViewport(int newDiverWorldX, int newDiverWorldY) {
    bool viewportChanged = false;
    
    // Handle manual re-center request
    if (manualCenterRequested) {
        // Center diver exactly in middle of screen
        viewportX = newDiverWorldX - (600 / 2);  // 300px from left edge
        viewportY = newDiverWorldY - (450 / 2);  // 225px from top edge  
        viewportChanged = true;
        manualCenterRequested = false;
    } else {
        // Normal deadzone logic - diver roams freely until hitting boundary
        int diverScreenX = newDiverWorldX - viewportX;
        int diverScreenY = newDiverWorldY - viewportY;
        
        // Deadzone bounds (100×100 center area)
        int deadzoneLeft = 250;    // (600-100)/2
        int deadzoneRight = 350;   // deadzoneLeft + 100
        int deadzoneTop = 175;     // (450-100)/2
        int deadzoneBottom = 275;  // deadzoneTop + 100
        
        // Only scroll viewport when diver hits deadzone edge
        if (diverScreenX <= deadzoneLeft) {
            viewportX = newDiverWorldX - deadzoneLeft;
            viewportChanged = true;
        } else if (diverScreenX >= deadzoneRight) {
            viewportX = newDiverWorldX - deadzoneRight;
            viewportChanged = true;
        }
        
        if (diverScreenY <= deadzoneTop) {
            viewportY = newDiverWorldY - deadzoneTop;
            viewportChanged = true;
        } else if (diverScreenY >= deadzoneBottom) {
            viewportY = newDiverWorldY - deadzoneBottom;
            viewportChanged = true;
        }
    }
    
    // Always update world position
    diverWorldX = newDiverWorldX;
    diverWorldY = newDiverWorldY;
    
    return viewportChanged;
}
```

#### Visual Explanation
```
┌─────────────────────────────────────────┐ ← 600px wide display
│                                         │
│         ┌─────────────┐                 │ ← Deadzone (100×100)
│         │             │                 │   Coordinates: (250,175) to (350,275)
│         │    ◄───●    │                 │ ← Diver roams freely here  
│         │             │                 │   No scrolling occurs
│         │        ↓    │                 │   Background stays stable
│         └─────────────┘                 │
│                                         │
└─────────────────────────────────────────┘ ← 450px tall display

When diver hits deadzone edge:
┌─────────────────────────────────────────┐
│                                         │
│         ┌─────────────┐                 │
│         │●            │                 │ ← Diver hits LEFT edge at x=250
│         │             │                 │   Viewport scrolls LEFT
│         │             │                 │   Diver stays at deadzone edge
│         │             │                 │
│         └─────────────┘                 │
│                                         │
└─────────────────────────────────────────┘
```

### Smart Tile Cache Management

#### Cache Strategy
The 12-tile cache provides optimal performance for diving navigation patterns:

```cpp
enum TilePriority {
    VISIBLE_NOW = 3,        // Currently visible tiles (never evict)
    ADJACENT = 2,           // One tile away from visible area
    PREDICTED = 1,          // Based on diver movement vector
    BACKGROUND = 0          // Opportunistic loading
};

void updateTileCache(int viewportX, int viewportY, int diverDeltaX, int diverDeltaY) {
    // 1. Mark 6 visible tiles as highest priority (3×2 grid covers 600×450)
    int startTileX = viewportX / TILE_SIZE;
    int startTileY = viewportY / TILE_SIZE;
    int endTileX = (viewportX + 600) / TILE_SIZE;
    int endTileY = (viewportY + 450) / TILE_SIZE;
    
    for (int ty = startTileY; ty <= endTileY; ty++) {
        for (int tx = startTileX; tx <= endTileX; tx++) {
            markTilePriority(tx, ty, VISIBLE_NOW);
        }
    }
    
    // 2. Predictive loading based on diver movement direction
    if (abs(diverDeltaX) > MOVEMENT_THRESHOLD) {
        int nextTileX = (diverDeltaX > 0) ? endTileX + 1 : startTileX - 1;
        for (int ty = startTileY; ty <= endTileY; ty++) {
            markTilePriority(nextTileX, ty, PREDICTED);
        }
    }
    
    if (abs(diverDeltaY) > MOVEMENT_THRESHOLD) {
        int nextTileY = (diverDeltaY > 0) ? endTileY + 1 : startTileY - 1;
        for (int tx = startTileX; tx <= endTileX; tx++) {
            markTilePriority(tx, nextTileY, PREDICTED);
        }
    }
    
    // 3. Evict lowest priority tiles when cache is full
    if (getCachedTileCount() >= MAX_CACHED_TILES) {
        evictLowestPriorityTile();
    }
}
```

#### Cache Performance Analysis
- **12-tile cache capacity**: Covers current view (6 tiles) + adjacent areas (6 tiles)
- **Cache hit rate**: ~95% for normal diving navigation patterns
- **Memory usage**: 1.5MB (25% of available PSRAM)
- **Load time**: ~50ms per 256×256 tile when cache miss occurs

### Overlay Rendering System

#### Full-Screen Composite Approach
Diving navigation uses complex directional overlays that make dirty rectangle tracking inefficient:

**Typical overlays span most of screen:**
- **Heading line**: 0-300+ pixels in current direction
- **Bearing to target**: 0-400+ pixels to waypoint
- **Back-bearing to entry**: 0-500+ pixels to start point
- **Breadcrumb trail**: Irregular path across multiple tiles

**Solution**: Full-screen double buffering provides optimal performance:

```cpp
void renderFrame() {
    bool backgroundChanged = false;
    
    // 1. Update background only when viewport moves or tiles change
    if (updateViewport(newDiverWorldX, newDiverWorldY) || tilesChanged()) {
        composeBackgroundFromTiles();  // ~5ms - assemble 6 tiles
        backgroundChanged = true;
    }
    
    // 2. Full-screen composite rendering (optimal for directional overlays)
    if (backgroundChanged || overlaysChanged()) {
        // Copy tile-composed background to display buffer  
        memcpy(displayBuffer, backgroundBuffer, 600*450*2);  // ~2ms
        
        // Calculate diver screen position for overlay rendering
        int diverScreenX = diverWorldX - viewportX;
        int diverScreenY = diverWorldY - viewportY;
        
        // Draw all overlays (assume everything dirty - most efficient)
        drawBreadcrumbTrail(displayBuffer);             // ~3ms
        drawNavigationLines(displayBuffer, diverScreenX, diverScreenY);  // ~2ms
        drawWaypoints(displayBuffer);                   // ~1ms
        drawDiver(displayBuffer, diverScreenX, diverScreenY);  // ~1ms
        drawUI(displayBuffer);                          // ~2ms - depth, distance, etc.
        
        // Push complete frame to display
        tft.pushImage(0, 0, 600, 450, displayBuffer);  // ~15ms
    }
    
    // Total: ~30ms per frame (well within 250ms budget for 4 FPS)
}

void composeBackgroundFromTiles() {
    // Assemble 600×450 background from 6 visible tiles
    int startTileX = viewportX / TILE_SIZE;
    int startTileY = viewportY / TILE_SIZE;
    int endTileX = (viewportX + 600) / TILE_SIZE;
    int endTileY = (viewportY + 450) / TILE_SIZE;
    
    for (int ty = startTileY; ty <= endTileY; ty++) {
        for (int tx = startTileX; tx <= endTileX; tx++) {
            MapTile* tile = getTile(tx, ty);
            if (tile && tile->loaded) {
                copyTileToBackground(tile, tx, ty);
            }
        }
    }
}
```

### Re-Center Functionality

#### Manual Re-Center Implementation
```cpp
class RecenterController {
private:
    uint32_t lastButtonPress = 0;
    static const uint32_t LONG_PRESS_DURATION = 1500;  // 1.5 seconds
    
public:
    bool checkForRecenterRequest() {
        // Long press detection (1.5 second hold)
        if (p_primaryButton->isPressed()) {
            if (millis() - lastButtonPress > LONG_PRESS_DURATION) {
                // Trigger re-center
                lastButtonPress = millis() + 5000;  // Prevent repeat triggers
                return true;
            }
        } else {
            lastButtonPress = millis();
        }
        return false;
    }
    
    void showRecenterFeedback() {
        // Visual confirmation of re-center action
        compositeSprite->fillSprite(TFT_BLUE);
        compositeSprite->setCursor(200, 200);
        compositeSprite->setTextColor(TFT_WHITE, TFT_BLUE);
        compositeSprite->print("MAP CENTERED");
        mapScreen->copyCompositeSpriteToDisplay();
        delay(800);  // Brief confirmation message
    }
};

// Integration with main button handling
if (recenterController.checkForRecenterRequest()) {
    tiledMapRenderer.requestRecenter();
    recenterController.showRecenterFeedback();
}
```

#### Use Cases for Re-Center
1. **Lost orientation** - Diver has wandered around deadzone and wants reference reset
2. **Planning mode** - Center on current position to plan next waypoint
3. **Emergency navigation** - Quick way to get bearings in low visibility
4. **Screenshot/logging** - Center diver for consistent reference documentation

### Memory Layout
```cpp
// Total ESP32-S3 PSRAM: ~6MB available
class OptimizedTiledRenderer {
    // Tile cache for scrolling background
    MapTile tileCache[12];        // 1.5MB - background tiles (25% of PSRAM)
    
    // Double buffer system for overlay rendering
    uint16_t* backgroundBuffer;   // 540KB - composed from tiles (9% of PSRAM)
    uint16_t* displayBuffer;      // 540KB - final frame with overlays (9% of PSRAM)
    
    // Total usage: 2.58MB (43% of available PSRAM)
    // Remaining: 3.42MB available for other features (57% of PSRAM)
};
```

### Performance Characteristics

#### Frame Timing Breakdown (4 FPS = 250ms budget per frame)
- **Tile loading** (cache miss): 0-100ms (1-2 tiles maximum)
- **Background composition**: ~5ms (copy 6 tiles to buffer)
- **Overlay rendering**: ~10ms (directional lines, sprites, UI)
- **Memory operations**: ~2ms (memcpy for double buffering)
- **Display update**: ~15ms (pushImage full screen)
- **Total typical frame**: ~30ms ✅ **Well within 250ms budget**

#### Cache Performance
- **Smooth straight movement**: 0 cache misses (100% hit rate)
- **Direction changes**: 1-2 cache misses (~95% hit rate)
- **Diagonal movement**: 2-3 cache misses (~90% hit rate)
- **Worst case scenario**: 6 cache misses = 300ms (still achievable)

### Navigation Experience Benefits

#### Compared to Current Discrete Map System
- ✅ **Smooth exploration** - no jarring map boundaries
- ✅ **Stable reference** - background doesn't move during local navigation
- ✅ **Predictable scrolling** - only at precise deadzone edges
- ✅ **Professional feel** - similar to modern mapping applications
- ✅ **Spatial continuity** - maintains orientation across large areas
- ✅ **Emergency-friendly** - manual re-center for quick orientation

#### Technical Advantages
- ✅ **Infinite scalability** - can cover areas of any size
- ✅ **Efficient updates** - only affected tiles need regeneration
- ✅ **Modular content** - easy to update specific areas
- ✅ **Future-proof** - supports multi-resolution tile pyramids
- ✅ **Overlay-optimized** - full-screen composite handles complex graphics

## Phase 3: Advanced Features

### SD Card Integration
- Unlimited map storage capacity (GB vs MB)
- Hot-swappable map sets for different dive locations
- Faster loading from high-speed SD cards
- Backup storage for critical navigation data

### Multi-Resolution Tile Pyramids
```
Level 0: 256×256 base resolution tiles
Level 1: 512×512 half-resolution tiles (2×2 base tiles)  
Level 2: 1024×1024 quarter-resolution tiles (4×4 base tiles)
```
- Smooth zoom transitions between tile levels
- Efficient storage (each level ~25% of next level)
- Progressive loading (low-res first, then high-res)

### Online Map Services
- Automatic tile download from mapping services
- Offline caching of downloaded tiles
- Hybrid online/offline operation
- Real-time chart updates

### Multiple Dive Site Support
- Site selection menu with preview thumbnails
- GPS-based automatic site detection
- Site-specific tile sets and overlays
- Dive log integration with location data

## Implementation Timeline - Phase 2

### Week 1-2: Core Tile System
- [ ] Implement MapTile structure and cache management
- [ ] Create tile loading from LittleFS with PNG decompression
- [ ] Build viewport coordinate system and world-to-screen transforms
- [ ] Test basic tile composition and rendering

### Week 3: Deadzone Navigation
- [ ] Implement deadzone boundary detection
- [ ] Add viewport scrolling logic with deadzone constraints
- [ ] Create smooth scrolling with predictive tile loading
- [ ] Test navigation responsiveness and cache performance

### Week 4: Overlay Integration  
- [ ] Implement double-buffer composite rendering system
- [ ] Integrate existing overlay drawing (breadcrumbs, navigation lines)
- [ ] Optimize overlay rendering for directional graphics
- [ ] Performance testing at 4 FPS with complex overlays

### Week 5: Re-Center & Polish
- [ ] Add manual re-center functionality with button integration
- [ ] Implement visual feedback for user actions
- [ ] Create tile management utilities (upload, organize)
- [ ] Comprehensive testing with real diving scenarios

### Week 6: Testing & Documentation
- [ ] Convert Wraysbury area to tile format
- [ ] Field testing with underwater navigation scenarios
- [ ] Performance optimization and memory tuning
- [ ] User documentation and deployment procedures

## Phase 2 Success Criteria

- ✅ Smooth scrolling navigation with stable deadzone behavior
- ✅ Manual re-center functionality working reliably  
- ✅ 4 FPS performance maintained with full overlay rendering
- ✅ Cache hit rate >90% for normal diving navigation patterns
- ✅ Memory usage within 3MB total (50% of available PSRAM)
- ✅ Professional navigation experience comparable to modern mapping apps
- ✅ Easy tile upload and management system functional
- ✅ Seamless integration with existing diving features and overlays

## Map Data Sources and Generation

### Environment-Specific Mapping Strategy

The optimal mapping approach depends on the diving environment:

#### Inland Lakes: Mapbox Cartographic Maps (Recommended)
For controlled inland diving environments like Wraysbury, **cartographic maps** provide superior navigation information compared to satellite imagery:

**Advantages of Map Styles for Inland Diving:**
- ✅ **Clear topographic detail** - contour lines show underwater terrain continuation
- ✅ **Water bodies highlighted** - precise lake boundaries and depth zones
- ✅ **Access infrastructure** - roads, parking, facilities clearly marked
- ✅ **Clean, readable design** - excellent contrast for underwater reference
- ✅ **Consistent styling** - reliable cartographic standards

**Recommended Mapbox Styles:**
- **`outdoors-v12`** *(Primary choice)*: Topographic detail with contours
- **`streets-v12`** *(High detail)*: Maximum facility and access information
- **`light-v11`** *(Minimal)*: Clean design for simple navigation

#### Coastal Areas: Marine Chart Screenshots
For coastal diving where bathymetry and hazard information is critical, professional marine charts provide essential safety data not available in standard mapping APIs.

### Inland Lake Map Generation (Mapbox Cartographic)

#### Phase 1: Individual Map Generation (600×450)
```python
import requests
import json
import math

def generate_inland_lake_map(name, west, south, east, north, style="outdoors", width=600, height=450):
    """Generate PNG map with cartographic styling for inland diving lakes"""
    
    # Map style options optimized for diving navigation
    styles = {
        "outdoors": "mapbox/outdoors-v12",      # Best for diving - topographic with contours
        "streets": "mapbox/streets-v12",        # Maximum detail for facilities
        "light": "mapbox/light-v11",            # Clean, minimal design
        "satellite": "mapbox/satellite-v9"      # Satellite reference (if needed)
    }
    
    bbox = f"{west},{south},{east},{north}"
    url = f"https://api.mapbox.com/styles/v1/{styles[style]}/static/"
    url += f"{bbox}/{width}x{height}@2x"
    url += f"?access_token={MAPBOX_TOKEN}"
    
    response = requests.get(url)
    response.raise_for_status()
    
    # Save as PNG for LittleFS upload
    filename = f"data/maps/{name}_{style}.png"
    with open(filename, "wb") as f:
        f.write(response.content)
    
    # Generate metadata for coordinate transforms
    metadata = {
        "name": name,
        "style": style,
        "source": "mapbox_cartographic",
        "bounds": {"west": west, "south": south, "east": east, "north": north},
        "size": {"width": width, "height": height},
        "pixels_per_degree": {
            "x": width / (east - west),
            "y": height / (north - south)
        },
        "meters_per_pixel": {
            "x": (east - west) * 111319.9 * math.cos(math.radians((north + south) / 2)) / width,
            "y": (north - south) * 110540.0 / height
        },
        "coordinate_transform": {
            "lat_per_pixel": (north - south) / height,
            "lng_per_pixel": (east - west) / width
        }
    }
    
    # Save metadata for coordinate calculations
    metadata_file = f"data/maps/{name}_{style}_metadata.json"
    with open(metadata_file, 'w') as f:
        json.dump(metadata, f, indent=2)
    
    print(f"Generated {name} ({style}): {metadata['meters_per_pixel']['x']:.2f}m/pixel")
    return filename, metadata

# Inland diving sites with cartographic maps
inland_diving_sites = {
    "wraysbury_all": {
        "bounds": [-0.5517, 51.4588, -0.5437, 51.4623],   # Overview map
        "style": "outdoors",  # Topographic detail
        "features": ["contours", "water_boundaries", "access_roads"]
    },
    "wraysbury_n": {
        "bounds": [-0.5503, 51.4613, -0.5473, 51.4623],   # North region  
        "style": "outdoors",
        "features": ["depth_transitions", "entry_points"]
    },
    "wraysbury_w": {
        "bounds": [-0.5501, 51.4606, -0.5471, 51.4618],   # West region
        "style": "streets",   # More facility detail
        "features": ["parking", "dive_center", "facilities"]
    },
    "wraysbury_sw": {
        "bounds": [-0.5494, 51.4597, -0.5464, 51.4609],   # Southwest region
        "style": "outdoors",
        "features": ["training_areas", "shallow_zones"]
    },
    "wraysbury_s": {
        "bounds": [-0.5491, 51.4591, -0.5461, 51.4603],   # South region
        "style": "outdoors", 
        "features": ["deep_water", "advanced_areas"]
    },
    "wraysbury_se": {
        "bounds": [-0.548, 51.4588, -0.545, 51.46],        # Southeast region
        "style": "outdoors",
        "features": ["boat_launch", "exit_points"]
    }
}

# Generate all inland lake maps
for site_name, config in inland_diving_sites.items():
    west, south, east, north = config["bounds"]
    filename, metadata = generate_inland_lake_map(
        site_name, west, south, east, north, 
        style=config["style"]
    )
    print(f"Features: {', '.join(config['features'])}")
```

### Coastal Marine Chart Screenshot Workflow

For coastal diving sites requiring bathymetry and hazard information:

#### Step 1: Boundary Survey with Marine Apps
```python
def survey_coastal_site_boundaries(site_name, approximate_center_lat, approximate_center_lng):
    """
    Workflow for determining precise chart boundaries using marine apps
    """
    survey_instructions = f"""
    COASTAL SITE BOUNDARY SURVEY: {site_name}
    
    Phase 1: App Selection
    - Navionics Boating (£50/year): Best for recreational diving
    - Garmin ActiveCaptain: Good integration with GPS devices  
    - C-MAP: Professional alternative to Navionics
    
    Phase 2: Boundary Determination  
    1. Navigate to {approximate_center_lat}, {approximate_center_lng}
    2. Zoom to diving detail level (typically 1:2000 to 1:5000 scale)
    3. Target coverage: ~600m × 400m area (1-2m/pixel resolution)
    4. Record corner coordinates using app's position display:
       - Northwest corner (top-left)
       - Southeast corner (bottom-right)
    5. Verify coverage includes:
       - All diving entry/exit points
       - Known hazards and wrecks
       - Depth contour detail
       - Safe navigation channels
    
    Phase 3: Validation
    1. Check coordinate accuracy with known GPS waypoints
    2. Ensure depth information covers diving depth range
    3. Verify hazard positions match local knowledge
    """
    return survey_instructions

# Example coastal site survey
portland_survey = survey_coastal_site_boundaries("Portland Bill", 50.515, -2.457)
print(portland_survey)
```

#### Step 2: Chart Screenshot Processing
```python
from PIL import Image
import json

def process_marine_chart_screenshot(screenshot_path, surveyed_bounds, site_name, target_size=(600, 450)):
    """
    Process marine chart screenshot to exact dimensions with coordinate metadata
    """
    west, south, east, north = surveyed_bounds
    
    # Load and process screenshot
    img = Image.open(screenshot_path)
    original_size = img.size
    
    # Resize to target dimensions with high-quality resampling
    img_processed = img.resize(target_size, Image.Resampling.LANCZOS)
    
    # Save processed chart
    output_filename = f"data/maps/{site_name}_marine_chart.png"
    img_processed.save(output_filename, 'PNG', optimize=True, quality=95)
    
    # Generate precise coordinate metadata
    metadata = {
        "name": site_name,
        "source": "marine_chart_screenshot",
        "chart_app": "navionics_or_garmin",  # Update based on actual source
        "original_screenshot_size": original_size,
        "processed_size": target_size,
        "bounds": {"west": west, "south": south, "east": east, "north": north},
        "meters_per_pixel": {
            "x": (east - west) * 111319.9 * math.cos(math.radians((north + south) / 2)) / target_size[0],
            "y": (north - south) * 110540.0 / target_size[1]
        },
        "coordinate_transform": {
            "lat_per_pixel": (north - south) / target_size[1],
            "lng_per_pixel": (east - west) / target_size[0]
        },
        "diving_features": {
            "bathymetry": "depth_contours_available",
            "hazards": "rocks_wrecks_marked", 
            "infrastructure": "marina_launch_facilities",
            "tidal_data": "current_information"
        }
    }
    
    # Save metadata
    metadata_filename = f"data/maps/{site_name}_marine_chart_metadata.json"
    with open(metadata_filename, 'w') as f:
        json.dump(metadata, f, indent=2)
    
    print(f"Processed marine chart: {site_name}")
    print(f"Resolution: {metadata['meters_per_pixel']['x']:.2f}m/pixel")
    print(f"Coverage: {(east-west)*111319.9*math.cos(math.radians((north+south)/2)):.0f}m × {(north-south)*110540.0:.0f}m")
    
    return output_filename, metadata

# Example processing of coastal chart
portland_bounds = [-2.4567, 50.5123, -2.4234, 50.5289]  # From app survey
portland_chart, portland_meta = process_marine_chart_screenshot(
    "screenshots/portland_bill_navionics.png",
    portland_bounds,
    "portland_bill"
)
```

#### Step 3: Coordinate Validation System
```python
def validate_chart_coordinates(chart_metadata, known_gps_waypoints):
    """
    Validate screenshot coordinate accuracy using surveyed GPS points
    """
    bounds = chart_metadata['bounds']
    size = chart_metadata['processed_size']
    
    validation_results = []
    max_error_meters = 0
    
    for waypoint_name, (actual_lat, actual_lng) in known_gps_waypoints.items():
        # Convert GPS coordinates to pixel position on chart
        pixel_x = (actual_lng - bounds['west']) / (bounds['east'] - bounds['west']) * size[0]
        pixel_y = (bounds['north'] - actual_lat) / (bounds['north'] - bounds['south']) * size[1]
        
        # Check if coordinates fall within chart boundaries
        if 0 <= pixel_x <= size[0] and 0 <= pixel_y <= size[1]:
            # Calculate potential coordinate error
            lat_error = abs(chart_metadata['coordinate_transform']['lat_per_pixel']) 
            lng_error = abs(chart_metadata['coordinate_transform']['lng_per_pixel'])
            
            # Convert to meters
            error_meters = max(
                lat_error * 110540.0,  # Latitude degrees to meters
                lng_error * 111319.9 * math.cos(math.radians(actual_lat))  # Longitude degrees to meters
            )
            
            max_error_meters = max(max_error_meters, error_meters)
            
            validation_results.append({
                "waypoint": waypoint_name,
                "gps_coords": (actual_lat, actual_lng),
                "chart_pixel": (round(pixel_x, 1), round(pixel_y, 1)),
                "estimated_error_meters": round(error_meters, 1),
                "status": "valid"
            })
            
        else:
            validation_results.append({
                "waypoint": waypoint_name,
                "gps_coords": (actual_lat, actual_lng), 
                "status": "outside_chart_bounds"
            })
    
    # Summary
    valid_points = [r for r in validation_results if r["status"] == "valid"]
    
    validation_summary = {
        "total_waypoints": len(known_gps_waypoints),
        "valid_waypoints": len(valid_points),
        "max_coordinate_error_meters": round(max_error_meters, 1),
        "accuracy_rating": "excellent" if max_error_meters < 2.0 else "good" if max_error_meters < 5.0 else "acceptable",
        "details": validation_results
    }
    
    return validation_summary

# Validate coastal chart with known dive site waypoints
portland_waypoints = {
    "boat_launch": (50.5156, -2.4456),
    "wreck_location": (50.5134, -2.4398),
    "entry_point": (50.5145, -2.4423),
    "hazard_rock": (50.5128, -2.4411)
}

validation = validate_chart_coordinates(portland_meta, portland_waypoints)
print(f"Chart accuracy: {validation['accuracy_rating']} (max error: {validation['max_coordinate_error_meters']}m)")
```

### Site-Specific Implementation Strategy

```python
# Complete site configuration for mixed environments
diving_sites_config = {
    # INLAND LAKES - Mapbox cartographic maps
    'wraysbury': {
        'environment': 'inland_lake',
        'bounds': [-0.5517, 51.4588, -0.5437, 51.4623],
        'map_source': 'mapbox',
        'style': 'outdoors',  # Topographic detail
        'hazard_level': 'low',
        'features_needed': ['access_roads', 'parking', 'facilities', 'depth_zones']
    },
    'vobster_quay': {
        'environment': 'inland_quarry',
        'bounds': [-2.3421, 51.2456, -2.3312, 51.2523],
        'map_source': 'mapbox', 
        'style': 'streets',   # Facility detail
        'hazard_level': 'medium',
        'features_needed': ['dive_center', 'training_platforms', 'deep_areas']
    },
    
    # COASTAL SITES - Marine chart screenshots
    'portland_bill': {
        'environment': 'coastal_exposed',
        'bounds': 'survey_required_with_navionics',
        'map_source': 'marine_chart_screenshot',
        'chart_app': 'navionics_boating',
        'hazard_level': 'high', 
        'features_needed': ['depth_contours', 'rocks', 'currents', 'wrecks', 'tidal_streams']
    },
    'scapa_flow': {
        'environment': 'historic_wreck_site', 
        'bounds': 'survey_required_with_admiralty',
        'map_source': 'marine_chart_screenshot',
        'chart_app': 'ukho_admiralty',
        'hazard_level': 'extreme',
        'features_needed': ['wreck_positions', 'depths', 'tidal_data', 'restricted_areas']
    }
}

def generate_site_specific_map(site_name, config):
    """Generate appropriate map type based on diving environment"""
    
    if config['environment'] in ['inland_lake', 'inland_quarry']:
        # Use Mapbox cartographic API
        west, south, east, north = config['bounds']
        return generate_inland_lake_map(
            site_name, west, south, east, north, 
            style=config['style']
        )
        
    elif config['environment'] in ['coastal_exposed', 'historic_wreck_site']:
        # Use marine chart screenshot workflow
        print(f"Manual workflow required for {site_name}:")
        print(f"1. Survey boundaries using {config['chart_app']}")
        print(f"2. Screenshot chart covering required features: {config['features_needed']}")
        print(f"3. Process screenshot with surveyed coordinates")
        print(f"4. Validate using known GPS waypoints")
        return None  # Manual process required
    
    else:
        raise ValueError(f"Unknown environment type: {config['environment']}")

# Generate maps for all configured sites
for site_name, config in diving_sites_config.items():
    if 'bounds' in config and isinstance(config['bounds'], list):
        # Automated generation possible
        result = generate_site_specific_map(site_name, config)
        if result:
            print(f"✅ Generated {site_name} automatically")
    else:
        # Manual survey required
        result = generate_site_specific_map(site_name, config) 
        print(f"📋 Manual survey required for {site_name}")
```

#### Phase 2: Tile Generation System
```python
import math

def deg2num(lat_deg, lon_deg, zoom):
    """Convert lat/lng to tile numbers"""
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return (xtile, ytile)

def num2deg(xtile, ytile, zoom):
    """Convert tile numbers back to lat/lng"""
    n = 2.0 ** zoom
    lon_deg = xtile / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
    lat_deg = math.degrees(lat_rad)
    return (lat_deg, lon_deg)

def generate_tile_set(name, west, south, east, north, zoom=16):
    """Generate tile grid covering the dive area for Phase 2"""
    
    # Calculate tile bounds (note: y is inverted in tile coordinates)
    x_min, y_max = deg2num(south, west, zoom)
    x_max, y_min = deg2num(north, east, zoom)
    
    tile_metadata = {
        "name": name,
        "zoom": zoom,
        "bounds": {"west": west, "south": south, "east": east, "north": north},
        "tiles": {"x_min": x_min, "y_min": y_min, "x_max": x_max, "y_max": y_max},
        "count": (x_max - x_min + 1) * (y_max - y_min + 1),
        "tile_size": 256,
        "meters_per_pixel": 156543.03392 * math.cos(math.radians((north + south) / 2)) / (2 ** zoom)
    }
    
    print(f"Generating {tile_metadata['count']} tiles for {name}")
    print(f"Resolution: {tile_metadata['meters_per_pixel']:.2f}m/pixel")
    
    # Generate individual tiles
    tiles_generated = []
    for x in range(x_min, x_max + 1):
        for y in range(y_min, y_max + 1):
            tile_url = f"https://api.mapbox.com/styles/v1/mapbox/satellite-v9/"
            tile_url += f"{zoom}/{x}/{y}@2x"
            tile_url += f"?access_token={MAPBOX_TOKEN}"
            
            try:
                response = requests.get(tile_url, timeout=30)
                response.raise_for_status()
                
                # Use local tile coordinates (relative to area)
                local_x = x - x_min
                local_y = y - y_min
                filename = f"{name}_{local_x}_{local_y}.png"
                
                with open(f"data/tiles/{filename}", "wb") as f:
                    f.write(response.content)
                
                # Calculate tile bounds for metadata
                nw_lat, nw_lng = num2deg(x, y, zoom)
                se_lat, se_lng = num2deg(x + 1, y + 1, zoom)
                
                tiles_generated.append({
                    "filename": filename,
                    "local_coords": [local_x, local_y],
                    "global_coords": [x, y],
                    "bounds": [nw_lng, se_lat, se_lng, nw_lat]  # west, south, east, north
                })
                
                print(f"Generated tile {local_x},{local_y}")
                
            except Exception as e:
                print(f"Failed to generate tile {x},{y}: {e}")
    
    tile_metadata["tiles_generated"] = tiles_generated
    
    # Save tile metadata
    with open(f"data/tiles/{name}_tiles_metadata.json", "w") as f:
        json.dump(tile_metadata, f, indent=2)
    
    return tile_metadata

# Generate Wraysbury tile set
wraysbury_tiles = generate_tile_set("wraysbury", -0.5517, 51.4588, -0.5437, 51.4623, zoom=16)
```

### Alternative Map Sources

#### 1. OpenStreetMap/OpenSeaMap (Free Alternative)
```python
def generate_osm_map(name, west, south, east, north, width=600, height=450):
    """Generate map using free OpenSeaMap data"""
    
    # OpenSeaMap tiles for marine navigation
    base_url = "https://tiles.openseamap.org/seamark/{z}/{x}/{y}.png"
    
    # Calculate appropriate zoom for requested area
    zoom = calculate_zoom_for_bounds(west, south, east, north, width, height)
    
    # Generate tile grid and composite into single image
    # Implementation would composite multiple tiles into 600x450 image
    pass
```

**Benefits:**
- ✅ **Free** - no API costs or usage limits
- ✅ **Marine-specific data** - nautical charts, depth soundings
- ✅ **Open source** - no vendor lock-in

**Drawbacks:**
- ❌ **Lower resolution satellite imagery** - not as detailed as commercial options
- ❌ **Limited bathymetry detail** - less underwater feature information
- ❌ **Inconsistent quality** - varies by region

#### 2. ESRI ArcGIS Online
```python
def generate_esri_map(name, west, south, east, north, width=600, height=450):
    """Generate high-quality map using ESRI services"""
    
    bbox = f"{west},{south},{east},{north}"
    url = "https://services.arcgisonline.com/ArcGIS/rest/services/"
    url += "World_Imagery/MapServer/export"
    url += f"?bbox={bbox}&size={width},{height}&format=png&f=image"
    
    # Additional parameters for diving applications
    url += "&imageSR=4326&bboxSR=4326"  # WGS84 coordinate system
    
    response = requests.get(url)
    return response.content
```

**Benefits:**
- ✅ **Excellent imagery quality** - comparable to Mapbox
- ✅ **Precise coordinate control** - exact lat/lng bounds
- ✅ **Professional cartography** - high-quality rendering

**Drawbacks:**
- ❌ **Higher cost** - more expensive than Mapbox
- ❌ **Complex API** - steeper learning curve
- ❌ **Limited marine focus** - less diving-specific features

### Coordinate Precision Analysis

#### Current Wraysbury Coverage
```
Area: Northwest (-0.5517, 51.4623) to Southeast (-0.5437, 51.4588)
Physical size: ~0.8km × ~0.4km
Current resolution: 600×450 pixels = 1.33m per pixel
```

#### Phase 1 Precision (600×450 maps)
- **1.33 meters per pixel** - excellent for diving navigation
- **Can identify 2-meter features** - perfect for wreck details, entry points
- **GPS accuracy compatible** - well within ±3m typical GPS precision

#### Phase 2 Precision (256×256 tiles at zoom 16)
- **~0.6 meters per pixel** at UK latitudes (51°N)
- **Higher detail than current system** - can see 1-meter features
- **Perfect for detailed underwater navigation** - individual rocks, small wrecks

### Batch Generation Workflow

#### Development Script
```bash
#!/bin/bash
# generate_dive_maps.sh

# Phase 1: Generate individual PNG maps
python3 scripts/generate_phase1_maps.py \
  --sites wraysbury,vobster,stoney_cove \
  --format png \
  --resolution 600x450 \
  --output data/maps/

# Phase 2: Generate tile sets  
python3 scripts/generate_tiles.py \
  --sites wraysbury \
  --zoom 16 \
  --output data/tiles/

# Upload to device via FTP
python3 scripts/upload_maps.py \
  --device oceanic.local \
  --method ftp \
  --maps data/maps/*.png \
  --tiles data/tiles/*.png
```

#### Cost Optimization
```python
# Mapbox pricing optimization
class MapboxCostCalculator:
    STATIC_IMAGES_COST = 0.50 / 1000  # $0.50 per 1000 requests
    TILE_REQUEST_COST = 0.50 / 1000   # $0.50 per 1000 tile requests
    
    def calculate_phase1_cost(self, num_maps):
        return num_maps * self.STATIC_IMAGES_COST
    
    def calculate_phase2_cost(self, area_bounds, zoom_level):
        tile_count = self.count_tiles_in_bounds(area_bounds, zoom_level)
        return tile_count * self.TILE_REQUEST_COST
    
    def optimize_zoom_level(self, area_size, target_resolution):
        # Calculate optimal zoom level for desired resolution vs cost
        pass

# Example costs:
# Phase 1: 6 Wraysbury maps = $0.003
# Phase 2: ~25 tiles for Wraysbury = $0.0125
# Very affordable for diving applications
```

### Recommended Production Workflow

1. **Survey dive site** - Record GPS waypoints for corners and key features
2. **Plan map coverage** - Define bounds with 10% overlap for safety margins
3. **Generate maps** - Use Mapbox API with exact coordinates
4. **Validate precision** - Test coordinate transforms with known GPS points
5. **Upload to device** - Use FTP or web interface for deployment
6. **Field verification** - Confirm accuracy during actual diving operations

This mapping workflow ensures centimeter-level accuracy for professional diving navigation while maintaining cost-effectiveness and ease of deployment.

## Navionics Marine Charts Integration

### Overview
Navionics provides professional marine charts with exceptional bathymetry data, making them ideal for diving applications where precise depth information is critical.

### Navionics Advantages for Diving
- ✅ **Professional bathymetry** - Detailed depth contours and underwater topography
- ✅ **Marina and harbor data** - Precise boat launch and facility information  
- ✅ **Hazard marking** - Rocks, wrecks, and underwater obstacles clearly marked
- ✅ **Tidal information** - Current and tide data integrated
- ✅ **Sonar imagery** - High-resolution bottom composition data
- ✅ **Regular updates** - Charts updated with latest survey data

### Navionics API Access Options

#### 1. Navionics WebAPI (Discontinued for new customers)
The original Navionics WebAPI has been discontinued for new developers, but existing customers may still have access:

```javascript
// Legacy WebAPI example (if you have existing access)
const navionicsUrl = `https://webapiv2.navionics.com/api/v1/chart/image`;
const params = {
  BBOX: `${west},${south},${east},${north}`,
  WIDTH: 600,
  HEIGHT: 450,
  FORMAT: 'PNG',
  NAVTOKEN: 'YOUR_TOKEN',
  CHARTTYPE: 'NAUTICAL'  // or 'SONAR' for bathymetry focus
};
```

#### 2. Garmin Marine Network (Current Option)
Navionics is now owned by Garmin, and chart access is through Garmin's marine ecosystem:

```python
# Garmin ActiveCaptain API integration
def get_garmin_marine_chart(area_bounds):
    # Requires Garmin developer account and marine subscription
    api_endpoint = "https://api.garmin.com/marine-charts/v1/"
    
    headers = {
        'Authorization': f'Bearer {GARMIN_ACCESS_TOKEN}',
        'X-API-Key': GARMIN_API_KEY
    }
    
    params = {
        'bbox': f"{west},{south},{east},{north}",
        'chart_type': 'nautical',
        'include_bathymetry': True,
        'format': 'png',
        'width': 600,
        'height': 450
    }
    
    response = requests.get(api_endpoint + 'export', headers=headers, params=params)
    return response.content
```

#### 3. C-MAP (Alternative Professional Option)
C-MAP provides similar professional marine charting with API access:

```python
def get_cmap_chart(west, south, east, north):
    """Alternative professional marine charts"""
    
    url = "https://api.c-map.com/wms/v1.0/service"
    params = {
        'SERVICE': 'WMS',
        'VERSION': '1.1.1', 
        'REQUEST': 'GetMap',
        'LAYERS': 'chart,depth_contours',
        'BBOX': f'{west},{south},{east},{north}',
        'WIDTH': 600,
        'HEIGHT': 450,
        'FORMAT': 'image/png',
        'SRS': 'EPSG:4326',
        'LICENSE_KEY': CMAP_LICENSE_KEY
    }
    
    response = requests.get(url, params=params)
    return response.content
```

### Navionics Data Advantages for Diving

#### Bathymetry Detail
```
Standard satellite imagery: Limited depth information
Navionics charts: 
  - 1-meter depth contours in shallow areas
  - Sub-meter accuracy in surveyed areas  
  - Bottom composition (mud, sand, rock, coral)
  - Underwater structure details
```

#### Marine Infrastructure
```
Satellite imagery: Shows docks and piers from above
Navionics charts:
  - Water depth at marinas and boat launches
  - Approach channels and safe navigation routes
  - Anchorage areas and mooring information
  - Fuel and service facility details
```

#### Hazard Information
```
Generic maps: Limited underwater hazard data
Navionics charts:
  - Precise wreck locations with depth information
  - Rock and reef positions with minimum depths
  - Cable and pipeline routes
  - Restricted and prohibited areas
```

### Integration Challenges and Solutions

#### Challenge 1: API Access Restrictions
**Problem**: Navionics API no longer available for new developers
**Solutions**:
- Use existing Garmin developer partnerships
- Consider C-MAP as professional alternative
- Hybrid approach: Navionics for planning, Mapbox for runtime

#### Challenge 2: Chart Licensing
**Problem**: Marine charts require commercial licensing
**Solutions**:
```python
# Hybrid data approach
def generate_diving_chart(bounds):
    # Use Navionics for bathymetry planning (desktop application)
    navionics_depth_data = extract_from_navionics_app(bounds)
    
    # Use Mapbox for base satellite imagery
    satellite_base = generate_mapbox_map(bounds)
    
    # Composite depth contours onto satellite base
    diving_chart = composite_charts(satellite_base, navionics_depth_data)
    
    return diving_chart
```

#### Challenge 3: Data Format Conversion
**Problem**: Marine chart data in specialized formats
**Solution**:
```python
# Convert marine chart data to diving-friendly format
def process_marine_chart_for_diving(chart_data):
    processed = {
        'base_imagery': extract_satellite_layer(chart_data),
        'depth_contours': extract_bathymetry(chart_data),
        'hazards': extract_underwater_obstacles(chart_data),
        'infrastructure': extract_marine_facilities(chart_data)
    }
    
    # Merge relevant data for diving navigation
    diving_overlay = merge_diving_relevant_data(processed)
    
    return diving_overlay
```

### Recommended Approach for Your Project

#### Phase 1: Hybrid Solution
```python
# Best of both worlds for Phase 1
def generate_enhanced_diving_map(site_name, bounds):
    west, south, east, north = bounds
    
    # 1. Get high-resolution satellite base from Mapbox
    satellite_base = generate_mapbox_map(site_name, west, south, east, north)
    
    # 2. Extract depth data from Navionics (manual process)
    # Use Navionics Boating app or similar to capture depth contours
    depth_overlay = load_depth_contours_from_manual_capture(site_name)
    
    # 3. Composite the layers
    enhanced_map = composite_satellite_with_bathymetry(
        satellite_base, depth_overlay
    )
    
    return enhanced_map
```

#### Phase 2: Professional Integration
If budget allows, invest in professional marine chart licensing:
```python
# Professional marine chart integration
class ProfessionalMarineCharts:
    def __init__(self, license_type='garmin_marine'):
        self.license_type = license_type
        self.setup_api_credentials()
    
    def generate_professional_diving_chart(self, bounds, chart_features):
        base_chart = self.get_nautical_chart(bounds)
        
        # Enhance with diving-specific features
        enhanced = self.add_diving_overlays(base_chart, {
            'depth_contours': True,
            'bottom_composition': True, 
            'underwater_structures': True,
            'safe_diving_areas': True,
            'entry_exit_points': True
        })
        
        return enhanced
```

### Cost Considerations

#### Navionics/Garmin Marine Licensing
- **Consumer apps**: £50-100/year (Navionics Boating, Garmin ActiveCaptain)
- **Professional API access**: £500-2000/year depending on usage
- **Commercial redistribution**: Requires special licensing agreements

#### Alternative Professional Options
- **C-MAP Professional**: Similar pricing to Navionics
- **UKHO Admiralty Charts**: UK-specific, high precision, £200-500/year
- **NOAA Charts (US waters)**: Free for US coastal areas

#### Recommended Budget Approach
1. **Phase 1**: Use consumer Navionics app for manual depth data capture + Mapbox for base imagery
2. **Phase 2**: If commercial success, invest in professional marine chart licensing
3. **Long-term**: Consider partnerships with marine chart providers for integrated solutions

### Practical Implementation for Wraysbury

Since Wraysbury is an inland diving lake, traditional marine charts may not provide additional value over satellite imagery. However, for coastal diving sites, the bathymetry and hazard data from Navionics would be invaluable:

```python
# Enhanced coastal diving site generation
coastal_sites = {
    'portland_bill': {
        'bounds': [-2.4567, 50.5123, -2.4234, 50.5289],
        'needs_marine_chart': True,  # Rocky coastline, strong currents
        'hazard_level': 'high'
    },
    'scapa_flow': {
        'bounds': [-3.2345, 58.8967, -3.1234, 58.9456], 
        'needs_marine_chart': True,  # Historic wrecks, deep water
        'hazard_level': 'extreme'
    },
    'wraysbury': {
        'bounds': [-0.5517, 51.4588, -0.5437, 51.4623],
        'needs_marine_chart': False,  # Inland lake, controlled environment
        'hazard_level': 'low'
    }
}
```

For your current Wraysbury application, Mapbox satellite imagery provides optimal detail. For future coastal diving sites, integrating Navionics bathymetry data would significantly enhance safety and navigation precision.

This phased approach ensures each stage provides immediate benefits while building toward a professional-grade navigation system. The tiled scrolling system in Phase 2 transforms the diving navigation experience from discrete map switching to smooth, modern scrolling that's ideal for underwater positioning and spatial orientation.