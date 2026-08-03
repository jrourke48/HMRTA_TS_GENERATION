# Makefile for Planning Tree Integration Tests
# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++20 -Wall -Wextra -g3
INCLUDES = -I. -I./include -I..
LIBS = -lspot -lbddx
RAYLIB_FLAGS = $(shell pkg-config --cflags --libs raylib 2>/dev/null || echo "-lraylib -lGL -lm -lpthread -ldl -lrt -lX11")

# Output directory
OUT_DIR = Testing
OUT_BIN = $(OUT_DIR)/TestIntegration
TREE_BIN = $(OUT_DIR)/TreeBuildingTests
TASK_ALLOC_BIN = $(OUT_DIR)/TestTaskAllocation
PLANNING_TREE_BIN = $(OUT_DIR)/TestPlanningTreeComprehensive
OPTIMAL_PATH_BIN = $(OUT_DIR)/OptimalPathTest
DSTAR_BIN = $(OUT_DIR)/Test_DStarGridWorld
VIS_BIN = $(OUT_DIR)/Visualization

# Source files
TEST_SRC = apps/TestIntegration.cpp
TREE_TEST_SRC = apps/TreeBuildingTests.cpp
TASK_ALLOC_SRC = apps/TestTaskAllocationAlgorithms.cpp
PLANNING_TREE_TEST_SRC = apps/TestPlanningTreeComprehensive.cpp
OPTIMAL_PATH_SRC = apps/Optimal_Path_Test.cpp
DSTAR_SRC = apps/Test_DStarGridWorld.cpp
VIS_SRC = apps/VisualizationApp.cpp
# TaskAllocationAlgorithms - algorithms for task allocation
TASK_ALLOC_SRCS = src/TaskAllocationAlgorithms.cpp

COMMON_SRCS = \
	src/Environment/Point.cpp \
	src/Environment/GridWorld.cpp \
	src/Environment/Environment.cpp \
	src/Environment/DStarGridWorld.cpp \
	src/Environment/gridvis.cpp \
	src/Tree/Tree_Node.cpp \
	src/Tree/PlanningDecisionTree.cpp \
	src/MultiRobotSystem/Robot.cpp \
	src/MultiRobotSystem/MultiRobotSystem.cpp \
	src/LTLFormula/BatchAtomicProposition.cpp \
	src/LTLFormula/LTLFormula.cpp \
	../Automatons/BuchiAutomaton.cpp \
	../Automatons/TS.cpp \
	../Transition_Systems/GeneralTransitionSystem.cpp \
	src/AlgorithmMetrics.cpp

# Object files
TEST_OBJ = $(TEST_SRC:.cpp=.o)
TREE_TEST_OBJ = $(TREE_TEST_SRC:.cpp=.o)
TASK_ALLOC_OBJ = $(TASK_ALLOC_SRC:.cpp=.o)
PLANNING_TREE_TEST_OBJ = $(PLANNING_TREE_TEST_SRC:.cpp=.o)
OPTIMAL_PATH_OBJ = $(OPTIMAL_PATH_SRC:.cpp=.o)
DSTAR_OBJ = $(DSTAR_SRC:.cpp=.o)
VIS_OBJ = $(VIS_SRC:.cpp=.o)
COMMON_OBJS = $(COMMON_SRCS:.cpp=.o)
TASK_ALLOC_OBJS = $(TASK_ALLOC_SRCS:.cpp=.o)
ALL_OBJS = $(TEST_OBJ) $(TREE_TEST_OBJ) $(TASK_ALLOC_OBJ) $(PLANNING_TREE_TEST_OBJ) $(OPTIMAL_PATH_OBJ) $(DSTAR_OBJ) $(VIS_OBJ) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)

# Targets
.PHONY: all clean run rebuild help test test-tree test-task test-planning-tree test-optimal-path test-dstar test-all tree-only task-only planning-tree-only optimal-path-only dstar-only vis vis-only

all: $(OUT_BIN) $(TREE_BIN) $(TASK_ALLOC_BIN) $(PLANNING_TREE_BIN) $(OPTIMAL_PATH_BIN) $(DSTAR_BIN) $(VIS_BIN)

# Build the test executable
$(OUT_BIN): $(TEST_OBJ) $(COMMON_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(OUT_BIN)"

# Build the tree building tests executable
$(TREE_BIN): $(TREE_TEST_OBJ) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(TREE_BIN)"

# Build the task allocation tests executable
$(TASK_ALLOC_BIN): $(TASK_ALLOC_OBJ) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(TASK_ALLOC_BIN)"

# Build the planning tree comprehensive tests executable
$(PLANNING_TREE_BIN): $(PLANNING_TREE_TEST_OBJ) $(COMMON_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(PLANNING_TREE_BIN)"

# Build the optimal path tests executable
$(OPTIMAL_PATH_BIN): $(OPTIMAL_PATH_OBJ) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(OPTIMAL_PATH_BIN)"

# Build the D* GridWorld tests executable
$(DSTAR_BIN): $(DSTAR_OBJ) $(COMMON_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(DSTAR_BIN)"

# Build the visualization executable
$(VIS_BIN): $(VIS_OBJ) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS) $(RAYLIB_FLAGS)
	@echo "✓ Build successful: $(VIS_BIN)"

# Compile object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Run the test
run: $(OUT_BIN)
	@echo "Running integration tests..."
	@$(OUT_BIN)

# Run tree building tests
test-tree: $(TASK_ALLOC_BIN)
	@echo "Running task allocation tests..."
	@$(TASK_ALLOC_BIN)

# Run task allocation tests
test-task: $(TREE_BIN)
	@echo "Running tree building tests..."
	@$(TREE_BIN)

# Run planning tree comprehensive tests
test-planning-tree: $(PLANNING_TREE_BIN)
	@echo "Running planning tree comprehensive tests..."
	@$(PLANNING_TREE_BIN)

# Run optimal path tests
test-optimal-path: $(OPTIMAL_PATH_BIN)
	@echo "Running optimal path tests..."
	@$(OPTIMAL_PATH_BIN)

# Run D* GridWorld tests
test-dstar: $(DSTAR_BIN)
	@echo "Running D* GridWorld tests..."
	@$(DSTAR_BIN)

# Shortcut to run only tree tests
tree-only: test-tree

# Shortcut to run only task allocation tests
task-only: test-task

# Shortcut to run only planning tree tests
planning-tree-only: test-planning-tree

# Shortcut to run only optimal path tests
optimal-path-only: test-optimal-path

# Shortcut to run only D* tests
dstar-only: test-dstar

# Run visualization
vis: $(VIS_BIN)
	@echo "Running visualization..."
	@$(VIS_BIN)

# Shortcut to build only visualization
vis-only: $(VIS_BIN)
	@echo "✓ Visualization built. Run 'make vis' to execute."

# Run all tests
test-all: run test-tree test-task test-planning-tree test-optimal-path test-dstar

# Run tests and show output
test: test-all

# Rebuild from scratch
rebuild: clean all
	@echo "✓ Rebuild complete"

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts..."
	@rm -f $(ALL_OBJS)
	@rm -f $(OUT_BIN)
	@echo "✓ Cleaned"

# Help target
help:
	@echo "Planning Tree Integration Test Makefile"
	@echo "========================================"
	@echo "Targets:"
	@echo "  make all                 - Build all test executables (default)"
	@echo "  make run                 - Build and run the integration test"
	@echo "  make test-tree           - Build and run the tree building tests"
	@echo "  make test-task           - Build and run the task allocation tests"
	@echo "  make test-planning-tree  - Build and run planning tree tests"
	@echo "  make test-optimal-path   - Build and run optimal path visualization tests"
	@echo "  make test-dstar          - Build and run D* GridWorld algorithm tests"
	@echo "  make vis                 - Build and run visualization app (requires raylib)"
	@echo "  make vis-only            - Build only visualization executable"
	@echo "  make tree-only           - Shortcut for 'make test-tree'"
	@echo "  make task-only           - Shortcut for 'make test-task'"
	@echo "  make planning-tree-only  - Shortcut for 'make test-planning-tree'"
	@echo "  make optimal-path-only   - Shortcut for 'make test-optimal-path'"
	@echo "  make dstar-only          - Shortcut for 'make test-dstar'"
	@echo "  make test                - Run all tests"
	@echo "  make test-all            - Run all tests (explicit)"
	@echo "  make rebuild             - Clean and rebuild all"
	@echo "  make clean               - Remove build artifacts"
	@echo "  make help                - Show this help message"
	@echo ""
	@echo "Examples:"
	@echo "  make                        # Build all tests"
	@echo "  make run                    # Build and run integration test"
	@echo "  make test-tree              # Build and run tree tests"
	@echo "  make vis                    # Build and run visualization (D* pathfinding)"
	@echo "  make test-optimal-path      # Build and run optimal path tests"
	@echo "  make test                   # Build and run all tests"
	@echo "  make clean && make          # Rebuild from scratch"
