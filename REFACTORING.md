# SLAM2REF Refactoring Documentation

## Overview
This document tracks the refactoring progress of the SLAM2REF codebase to improve readability, maintainability, and organization.

## Refactoring Goals
- Break down large files (1000+ lines) into manageable sizes (200-400 lines)
- Improve directory structure and naming conventions
- Better separation of concerns
- Reduce code duplication
- Follow modern C++17 best practices

---

## ✅ Completed Refactorings

### 1. Directory Restructuring (Priority 1) ✅
**Status**: COMPLETED

**Changes**:
- Removed numeric prefixes from directories (`0.Utils`, `01.Slam2ref`)
- Created organized subdirectory structure:

```
include/slam2ref/
├── core/                   # Main orchestrator and session management
├── optimization/           # GTSAM factor graph and optimization
├── registration/           # ICP/GICP algorithms
├── place_recognition/      # Loop detection and scan context
├── io/                     # File I/O operations
├── geometry/               # Geometric transformations and conversions
├── visualization/          # Visualization utilities
├── config/                 # Configuration management
├── utils/                  # General utilities
└── types/                  # Type definitions

src/slam2ref/
├── (same structure as include)
└── main.cpp
```

**Impact**:
- ✅ Clearer organization by responsibility
- ✅ Easier to locate relevant code
- ✅ Professional directory naming

**Files Affected**: All headers and sources

---

### 2. Include Path Updates (Priority 1) ✅
**Status**: COMPLETED

**Changes**:
- Updated all `#include` statements from old paths to new paths:
  - `"0.Utils/ParamServer.h"` → `"slam2ref/config/ParamServer.h"`
  - `"01.Slam2ref/Slam2ref.h"` → `"slam2ref/core/Slam2ref.h"`
  - And 10+ more systematic replacements

**Impact**:
- ✅ All includes now reflect new directory structure
- ✅ Consistent naming throughout codebase

**Files Affected**: 31 source and header files

---

### 3. CMakeLists.txt Modernization (Priority 1) ✅
**Status**: COMPLETED

**Changes**:
- Replaced `GLOB_RECURSE` with explicit file listing
- Organized sources by module:
  - Core module (4 files)
  - Configuration module (2 files)
  - I/O module (2 files)
  - Optimization module (1 file)
  - Place recognition module (4 files)
  - Registration module (3 files)
  - Geometry module (2 files)
  - Utils module (2 files)

**Benefits**:
- ✅ Clear module boundaries
- ✅ Easier to enable/disable modules
- ✅ Better compile-time dependency tracking
- ✅ Prevents accidental inclusion of old files

**File**: `CMakeLists.txt`

---

### 4. Remove Using Namespace from Headers (Priority 1) ✅
**Status**: COMPLETED

**Changes**:
- Removed all `using namespace` statements from `Slam2ref.h`:
  - `using namespace open3d;`
  - `using namespace std;`
  - `using namespace Eigen;`
  - `using namespace gtsam;`
  - `using namespace Slam3refParam;`

- Fully qualified GTSAM types:
  - `NonlinearFactorGraph` → `gtsam::NonlinearFactorGraph`
  - `Values` → `gtsam::Values`
  - `ISAM2` → `gtsam::ISAM2`
  - `noiseModel::` → `gtsam::noiseModel::`
  - `Key` → `gtsam::Key`
  - `SharedNoiseModel` → `gtsam::SharedNoiseModel`

**Impact**:
- ✅ Prevents namespace pollution
- ✅ Clearer type origins
- ✅ Follows C++ best practices
- ✅ Safer for header-only usage

**Files Affected**: `include/slam2ref/core/Slam2ref.h`

---

### 5. utility.cpp Partial Split (Priority 2) ✅
**Status**: PARTIALLY COMPLETED

**New Files Created**:

#### `src/slam2ref/utils/MathUtils.cpp` (80 lines)
Functions extracted:
- `genGlobalNodeIdx()` - Generate global node indices
- `ungenGlobalNodeIdx()` - Reverse index generation
- `genAnchorNodeIdx()` - Generate anchor node indices
- `sortVecWithIdx()` - Sort vector with index tracking
- `linspace()` - Linear space generation
- `collect_digits()` - Digit collection utility
- `poseDistance()` - Calculate distance between poses

#### `src/slam2ref/geometry/Conversions.cpp` (95 lines)
Functions extracted:
- `pclPointToAffine3f()` - PCL point to Eigen affine
- `pclPointToAffine3fOnlyTranslation()` - Translation only
- `pclPointTogtsamPose3()` - PCL to GTSAM conversion
- `getPoseTransformation()` - Extract pose from matrix
- `pclToOpen3d()` / `pclToOpen3d_v2()` - PCL to Open3D
- `open3dToPcl()` - Open3D to PCL
- `transformPoint()` - Point transformation
- `convertPointCloud()` - Point cloud type conversion

**Remaining in utility.cpp** (~570 lines):
- Binary I/O functions (readBin)
- Point cloud transformations
- File I/O (writePose3ToStream variants, saveSCD, readSCD)
- Color utilities
- Point cloud filters
- File system utilities

**Next Steps for utility.cpp**:
- [ ] Extract BinaryIO.cpp (~80 lines)
- [ ] Extract Transformations.cpp (~200 lines)
- [ ] Extract PoseIO.cpp (~120 lines)
- [ ] Extract ColorUtils.cpp (~80 lines)
- [ ] Extract PointCloudFilters.cpp (~80 lines)

---

## 📋 Planned Refactorings

### Priority 2: Large File Splitting

#### Slam2ref_05_ICP_MV.cpp (1119 lines) → 6 files
**Status**: PENDING

Planned split:
1. `ICPHelpers.cpp` (~150 lines) - Helper functions
2. `ICPRangeSearch.cpp` (~200 lines) - RS loop ICP
3. `ICPYawProjection.cpp` (~200 lines) - Yaw-based ICP
4. `ICPScanContext.cpp` (~200 lines) - SC loop ICP
5. `ICPKNN.cpp` (~150 lines) - KNN loop ICP
6. `FinalICP.cpp` (~250 lines) - Final refinement ICP

**Benefits**:
- Clearer separation of ICP variants
- Easier to maintain individual algorithms
- Better testability

#### Session.cpp (780 lines) → 4 files
**Status**: PENDING

Planned split:
1. `Session.cpp` (~200 lines) - Core class
2. `SessionGraphLoader.cpp` (~250 lines) - G2O/KITTI loading
3. `SessionPointCloudLoader.cpp` (~200 lines) - PCD loading
4. `SessionQueries.cpp` (~150 lines) - Query operations

**Benefits**:
- Clearer I/O vs. processing separation
- Easier to add new file format support

#### Scancontext.cpp (588 lines) → 3 files
**Status**: PENDING

Planned split:
1. `ScanContext.cpp` (~250 lines) - Core implementation
2. `ScanContextIndoor.cpp` (~200 lines) - Indoor variants
3. `LoopDetector.cpp` (~150 lines) - Loop closure detection

**Benefits**:
- Separates algorithm variants
- Easier to experiment with new variants

#### PointCloudRegistration.cpp (489 lines) → 3 files
**Status**: PENDING

Planned split:
1. `GICPOpen3D.cpp` (~200 lines) - Open3D GICP variants
2. `YawGICP.cpp` (~200 lines) - Yaw-constrained GICP
3. `RegistrationUtils.cpp` (~100 lines) - Utilities

**Benefits**:
- Clear algorithm separation
- Easier to add new registration methods

---

### Priority 3: Code Quality Improvements

#### Refactor Slam2ref.h Member Variables
**Status**: PENDING

**Current Issue**:
- ~100 member variables
- Duplication from ParamServer

**Proposed Solution**:
```cpp
class Slam2ref {
private:
    ParamServer* params;  // Single source of truth

    // Only state variables (not config)
    Sessions sessions_;
    gtsam::NonlinearFactorGraph gtSAMgraph;
    gtsam::Values initialEstimate;
    gtsam::ISAM2* isam;

    std::vector<std::pair<int, int>> SCLoopIdxPairs_;
    std::vector<std::pair<int, int>> RSLoopIdxPairs_;

    std::mutex mtx;
};
```

**Benefits**:
- Reduces from ~100 to ~15 member variables
- Single source of truth for configuration
- Easier to maintain

---

## 📊 Metrics

### File Size Reduction Progress

| File | Original | Target | Status |
|------|----------|--------|--------|
| utility.cpp | 744 | 7 files @ ~100 ea | 25% complete |
| Slam2ref_05_ICP_MV.cpp | 1119 | 6 files @ ~200 ea | 0% |
| Session.cpp | 780 | 4 files @ ~200 ea | 0% |
| Scancontext.cpp | 588 | 3 files @ ~250 ea | 0% |
| PointCloudRegistration.cpp | 489 | 3 files @ ~160 ea | 0% |

### Overall Progress

- ✅ Directory restructuring: 100%
- ✅ Include path updates: 100%
- ✅ CMakeLists modernization: 100%
- ✅ Namespace cleanup: 100%
- 🔄 File splitting: 10% (2 of ~20 planned splits)
- ⏳ Member variable refactoring: 0%
- ⏳ Function size reduction: 0%

**Total Refactoring Completion: ~40%**

---

## 🎯 Next Steps

### Immediate (Week 1-2):
1. ✅ Directory restructuring - DONE
2. ✅ CMakeLists.txt updates - DONE
3. ✅ Remove using namespace from headers - DONE
4. 🔄 Complete utility.cpp split - 25% DONE
5. ⏳ Split Slam2ref_05_ICP_MV.cpp

### Short-term (Week 3-4):
6. Split Session.cpp
7. Split Scancontext.cpp
8. Split PointCloudRegistration.cpp
9. Refactor Slam2ref.h member variables

### Medium-term (Week 5-6):
10. Break down large functions (>100 lines)
11. Standardize naming conventions
12. Add comprehensive Doxygen documentation

---

## 🔧 Build Instructions

The refactored code maintains the same build process:

```bash
mkdir build && cd build
cmake ..
make -j5
./slam2ref
```

**Note**: Requires dependencies:
- GTSAM 4.0.3+
- Open3D
- PCL
- OpenCV 4.2.0+
- yaml-cpp
- OpenMP

---

## 📝 Migration Guide

### For Developers Using Old Include Paths

Update your includes:
```cpp
// Old
#include "0.Utils/ParamServer.h"
#include "01.Slam2ref/Slam2ref.h"

// New
#include "slam2ref/config/ParamServer.h"
#include "slam2ref/core/Slam2ref.h"
```

### For Code Using utility.h Functions

Some functions moved to new headers:
```cpp
// Math utilities
#include "slam2ref/utils/MathUtils.h"
// Now use: Slam3refParam::genGlobalNodeIdx()

// Conversions
#include "slam2ref/geometry/Conversions.h"
// Now use: pclToOpen3d(), pclPointTogtsamPose3(), etc.
```

---

## 📚 Additional Resources

- Original refactoring plan: See initial analysis
- GTSAM documentation: https://gtsam.org/
- Open3D documentation: http://www.open3d.org/
- PCL documentation: https://pointclouds.org/

---

## 👥 Contributors

- Original codebase: Giseop Kim (LT-SLAM), Miguel A. Vega-Torres
- Refactoring: 2024

---

## 📄 License

GPLv3 - Same as original SLAM2REF project
