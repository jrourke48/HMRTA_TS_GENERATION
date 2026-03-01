# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++20 -Wall -Wextra -g3
INCLUDES = -I"C:/Users/johnn/Spot Library/spot-2.14.4.dev" -I"C:/Users/johnn/Spot Library/spot-2.14.4.dev/buddy/src"
LDFLAGS = -L"C:/Users/johnn/Spot Library/spot-2.14.4.dev/lib"
LIBS = -lspot -lbddx

# Output directory
OUTDIR = output

# Default target
all: LTLtoBuchiAutomaton

# Individual targets
LTLtoBuchiAutomaton: LTLtoBuchiAutomaton.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LDFLAGS) $< -o $(OUTDIR)/$@.exe $(LIBS)

GlobalState: GlobalState.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LDFLAGS) $< -o $(OUTDIR)/$@.exe $(LIBS)

ParsingandPrintingLTL: ParsingandPrintingLTL.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LDFLAGS) $< -o $(OUTDIR)/$@.exe $(LIBS)

RelablingFormulas: RelablingFormulas.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LDFLAGS) $< -o $(OUTDIR)/$@.exe $(LIBS)

TestingEquivalance: TestingEquivalance.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LDFLAGS) $< -o $(OUTDIR)/$@.exe $(LIBS)

BasicTransitionSystem: Transition_Systems/BasicTransitionSystem.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LDFLAGS) $< -o $(OUTDIR)/$@.exe $(LIBS)

GeneralTransitionSystem: Transition_Systems/GeneralTransitionSystem.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LDFLAGS) $< -o $(OUTDIR)/$@.exe $(LIBS)

GridWorldTransitionSystem: Transition_Systems/GridWorldTransitionSystem.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LDFLAGS) $< -o $(OUTDIR)/$@.exe $(LIBS)

SpotGridWorldTS: Transition_Systems/SpotGridWorldTS.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LDFLAGS) $< -o $(OUTDIR)/$@.exe $(LIBS)

# Run targets (build and execute)
run-LTLtoBuchiAutomaton: LTLtoBuchiAutomaton
	./$(OUTDIR)/LTLtoBuchiAutomaton.exe

run-GlobalState: GlobalState
	./$(OUTDIR)/GlobalState.exe

run-ParsingandPrintingLTL: ParsingandPrintingLTL
	./$(OUTDIR)/ParsingandPrintingLTL.exe

run-RelablingFormulas: RelablingFormulas
	./$(OUTDIR)/RelablingFormulas.exe

run-TestingEquivalance: TestingEquivalance
	./$(OUTDIR)/TestingEquivalance.exe

run-BasicTransitionSystem: BasicTransitionSystem
	./$(OUTDIR)/BasicTransitionSystem.exe

run-GeneralTransitionSystem: GeneralTransitionSystem
	./$(OUTDIR)/GeneralTransitionSystem.exe

run-GridWorldTransitionSystem: GridWorldTransitionSystem
	./$(OUTDIR)/GridWorldTransitionSystem.exe

run-SpotGridWorldTS: SpotGridWorldTS
	./$(OUTDIR)/SpotGridWorldTS.exe

# Clean build artifacts
clean:
	rm -f $(OUTDIR)/*.exe

.PHONY: all clean run-LTLtoBuchiAutomaton run-GlobalState run-ParsingandPrintingLTL run-RelablingFormulas run-TestingEquivalance run-BasicTransitionSystem run-GeneralTransitionSystem run-GridWorldTransitionSystem run-SpotGridWorldTS
