# MCTB-PDT Structure Migration Guide

Your project structure has been reorganized to follow best practices. Here's what to do:

## ✅ Completed Automatically
- ✓ Created `include/` directory with subdirectories
- ✓ Created `apps/` directory  
- ✓ Updated Makefile with new include path `-I./include`

## 📋 Manual Steps Required

### Step 1: Move Header Files to `include/`

**From `Environment/` → Move to `include/Environment/`:**
- Point.h
- GridWorld.h
- Environment.h
- gridvis.h
- dstar.hpp

**From `LTLFormula/` → Move to `include/LTLFormula/`:**
- BatchAtomicProposition.h
- LTLFormula.h

**From `MultiRobotSystem/` → Move to `include/MultiRobotSystem/`:**
- Robot.h
- MultiRobotSystem.h

**From `Tree/` → Move to `include/Tree/`:**
- Tree_Node.h
- PlanningDecisionTree.h

**From root `MCTB-PDT/` → Move to `include/`:**
- TaskAllocationAlgorithms.h
- AlgorithmMetrics.h

### Step 2: Move Test Files to `apps/`

**From `Testing/` → Move to `apps/`:**
- TestIntegration.cpp
- TreeBuildingTests.cpp
- TestTaskAllocationAlgorithms.cpp
- TestPlanningTreeComprehensive.cpp
- Optimal_Path_Test.cpp
- AlgorithmMetrics_Example.cpp (optional, if you want to keep it)

### Step 3: Update `#include` Statements

All `.cpp` files now need to update their includes. Change:
```cpp
// OLD (files were in same directory)
#include "Point.h"
#include "LTLFormula.h"

// NEW (headers are now in include/)
#include "Environment/Point.h"
#include "LTLFormula/LTLFormula.h"
#include "TaskAllocationAlgorithms.h"
#include "AlgorithmMetrics.h"
```

### Step 4: Keep `.cpp` Files Where They Are

The implementation files stay organized:
- `Environment/*.cpp` → stays in `Environment/`
- `LTLFormula/*.cpp` → stays in `LTLFormula/`
- `MultiRobotSystem/*.cpp` → stays in `MultiRobotSystem/`
- `Tree/*.cpp` → stays in `Tree/`
- `TaskAllocationAlgorithms.cpp` → stays in root
- `AlgorithmMetrics.cpp` → stays in root

### Step 5: Verify the Build

```bash
cd MCTB-PDT
make clean
make all
```

## 📁 Final Structure

```
MCTB-PDT/
├── include/                    ← All headers
│   ├── TaskAllocationAlgorithms.h
│   ├── AlgorithmMetrics.h
│   ├── Environment/
│   │   ├── Point.h
│   │   ├── GridWorld.h
│   │   ├── Environment.h
│   │   ├── gridvis.h
│   │   └── dstar.hpp
│   ├── LTLFormula/
│   │   ├── BatchAtomicProposition.h
│   │   └── LTLFormula.h
│   ├── MultiRobotSystem/
│   │   ├── Robot.h
│   │   └── MultiRobotSystem.h
│   └── Tree/
│       ├── Tree_Node.h
│       └── PlanningDecisionTree.h
│
├── apps/                       ← Test programs with main()
│   ├── TestIntegration.cpp
│   ├── TreeBuildingTests.cpp
│   ├── TestTaskAllocationAlgorithms.cpp
│   ├── TestPlanningTreeComprehensive.cpp
│   └── Optimal_Path_Test.cpp
│
├── Environment/                ← .cpp implementation files
│   ├── Point.cpp
│   ├── GridWorld.cpp
│   ├── Environment.cpp
│   ├── dstar.cpp
│   ├── gridvis.cpp
│   └── README.md
│
├── LTLFormula/
│   ├── BatchAtomicProposition.cpp
│   ├── LTLFormula.cpp
│   └── ...
│
├── MultiRobotSystem/
│   ├── Robot.cpp
│   ├── MultiRobotSystem.cpp
│   └── ...
│
├── Tree/
│   ├── Tree_Node.cpp
│   ├── PlanningDecisionTree.cpp
│   └── ...
│
├── Testing/                    ← Keep for reference (deprecated)
├── output/                     ← Keep for outputs
├── TaskAllocationAlgorithms.cpp
├── TaskAllocationAlgorithms.h
├── AlgorithmMetrics.cpp
├── AlgorithmMetrics.h
├── AlgorithmMetrics_Example.cpp
├── Makefile                    ← Updated!
└── README.md
```

## 🔧 If You Want to Automate

Once you've manually moved files, run:
```bash
cd MCTB-PDT
make clean
make all
```

The Makefile will now find headers in `include/` thanks to `-I./include`.

---

**Need help with a specific file?** Let me know the `.cpp` filename and I can help update its `#include` statements!
