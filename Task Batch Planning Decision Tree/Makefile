# Makefile for Planning Tree Integration Tests
# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++20 -Wall -Wextra -g3
INCLUDES = -I. -I..
LIBS = -lspot -lbddx

# Output directory
OUT_DIR = Testing
OUT_BIN = $(OUT_DIR)/TestIntegration
TREE_BIN = $(OUT_DIR)/TreeBuildingTests
TASK_ALLOC_BIN = $(OUT_DIR)/TestTaskAllocation

# Source files
TEST_SRC = Testing/TestIntegration.cpp
TREE_TEST_SRC = Testing/TreeBuildingTests.cpp
TASK_ALLOC_SRC = Testing/TestTaskAllocationAlgorithms_new.cpp
# TaskAllocationAlgorithms - algorithms for task allocation
TASK_ALLOC_SRCS = TaskAllocationAlgorithms.cpp

COMMON_SRCS = \
	Environment/Point.cpp \
	Environment/GridWorld.cpp \
	Environment/Environment.cpp \
	Tree/Tree_Node.cpp \
	Tree/PlanningDecisionTree.cpp \
	MultiRobotSystem/Robot.cpp \
	MultiRobotSystem/MultiRobotSystem.cpp \
	LTLFormula/BatchAtomicProposition.cpp \
	LTLFormula/LTLFormula.cpp \
	../Automatons/BuchiAutomaton.cpp \
	../Automatons/TS.cpp \
	../Transition_Systems/GeneralTransitionSystem.cpp

# Object files
TEST_OBJ = $(TEST_SRC:.cpp=.o)
TREE_TEST_OBJ = $(TREE_TEST_SRC:.cpp=.o)
TASK_ALLOC_OBJ = $(TASK_ALLOC_SRC:.cpp=.o)
COMMON_OBJS = $(COMMON_SRCS:.cpp=.o)
TASK_ALLOC_OBJS = $(TASK_ALLOC_SRCS:.cpp=.o)
ALL_OBJS = $(TEST_OBJ) $(TREE_TEST_OBJ) $(TASK_ALLOC_OBJ) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)

# Targets
.PHONY: all clean run rebuild help test test-tree test-task test-all tree-only task-only

all: $(OUT_BIN) $(TREE_BIN) $(TASK_ALLOC_BIN)

# Build the test executable
$(OUT_BIN): $(TEST_OBJ) $(COMMON_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(OUT_BIN)"

# Build the tree building tests executable
$(TREE_BIN): $(TREE_TEST_OBJ) $(COMMON_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(TREE_BIN)"

# Build the task allocation tests executable
$(TASK_ALLOC_BIN): $(TASK_ALLOC_OBJ) $(COMMON_OBJS) $(TASK_ALLOC_OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $^ -o $@ $(LIBS)
	@echo "✓ Build successful: $(TASK_ALLOC_BIN)"

# Compile object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Run the test
run: $(OUT_BIN)
	@echo "Running integration tests..."
	@$(OUT_BIN)

# Run tree building tests
test-tree: $(TREE_BIN)
	@echo "Running tree building tests..."
	@$(TREE_BIN)

# Run task allocation tests
test-task: $(TASK_ALLOC_BIN)
	@echo "Running task allocation tests..."
	@$(TASK_ALLOC_BIN)

# Shortcut to run only tree tests
tree-only: test-tree

# Shortcut to run only task allocation tests
task-only: test-task

# Run all tests
test-all: run test-tree test-task

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
	@echo "  make all              - Build all test executables (default)"
	@echo "  make run              - Build and run the integration test"
	@echo "  make test-tree        - Build and run the tree building tests"
	@echo "  make test-task        - Build and run the task allocation tests"
	@echo "  make tree-only        - Shortcut for 'make test-tree'"
	@echo "  make task-only        - Shortcut for 'make test-task'"
	@echo "  make test             - Run all tests (integration, tree, task)"
	@echo "  make test-all         - Run all tests (explicit)"
	@echo "  make rebuild          - Clean and rebuild all"
	@echo "  make clean            - Remove build artifacts"
	@echo "  make help             - Show this help message"
	@echo ""
	@echo "Examples:"
	@echo "  make                  # Build all tests"
	@echo "  make run              # Build and run integration test"
	@echo "  make test-tree        # Build and run tree tests"
	@echo "  make test-task        # Build and run task allocation tests"
	@echo "  make test             # Build and run all tests"
	@echo "  make clean && make    # Rebuild from scratch"
