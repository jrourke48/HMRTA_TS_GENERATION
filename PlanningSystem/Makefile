# Makefile for Planning System Module
# Compile with: make demo  or  make all

CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -g
LDFLAGS = -lspot -lbddx -pthread

# Search path for source files
VPATH = .:..

# Include paths
INCLUDE_DIRS = -I.. \
               -I../Transition_Systems \
               -I/usr/include \
               -I/usr/include/spot \
               -I/usr/include/eigen3

# Library paths
LIB_DIRS = -L/usr/lib

# Source files
CORE_SRC = VoronoiPruningPipeline.cpp \
           IntegratedPlanningSystem.cpp \
           PlanningUtilities.cpp \
           ProductAutomaton.cpp \
           TS.cpp \
           BuchiAutomaton.cpp \
           GridWorldTransitionSystem.cpp

EXAMPLES = VoronoiPruningExample.cpp CompleteWorkflowExample.cpp

# Object files
CORE_OBJ = $(CORE_SRC:.cpp=.o)

# Targets
all: complete_workflow voronoi_example

complete_workflow: $(CORE_OBJ) CompleteWorkflowExample.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIB_DIRS) $(LDFLAGS)
	@echo "✓ Built complete_workflow"

voronoi_example: $(CORE_OBJ) VoronoiPruningExample.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIB_DIRS) $(LDFLAGS)
	@echo "✓ Built voronoi_example"

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE_DIRS) -c $< -o $@

TS.o: ../TS.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE_DIRS) -c $< -o $@

BuchiAutomaton.o: ../BuchiAutomaton.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE_DIRS) -c $< -o $@

GridWorldTransitionSystem.o: ../Transition_Systems/GridWorldTransitionSystem.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE_DIRS) -c $< -o $@

clean:
	rm -f *.o complete_workflow voronoi_example
	@echo "✓ Cleaned build artifacts"

run: all
	@echo "\n===== Running Complete Workflow Example =====" && \
	./complete_workflow && \
	echo "\n===== Running Voronoi Pruning Example =====" && \
	./voronoi_example

help:
	@echo "Make targets:"
	@echo "  make all                - Build all examples"
	@echo "  make complete_workflow  - Build warehouse scenario"
	@echo "  make voronoi_example    - Build basic pruning example"
	@echo "  make clean              - Remove build artifacts"
	@echo "  make run                - Build and run all examples"
	@echo "  make help               - Show this message"

.PHONY: all clean run help
