# LTL Formula Module

## Overview

The LTL Formula module provides task specification and semantic representation for multi-robot planning. It bridges human-readable Linear Temporal Logic (LTL) specifications with formal automata-theoretic representations suitable for planning algorithms.

### Key Responsibilities

1. **LTL Parsing** - Convert textual formulas to internal representation
2. **Atomic Proposition Management** - Define and track task requirements
3. **Formula Validation** - Ensure specifications are well-formed
4. **Spot Integration** - Interface with Spot library for automata generation
5. **Semantic Representation** - Maintain task structure for planning

## Components

### LTLFormula

**Files**: `LTLFormula.h`, `LTLFormula.cpp`

Main class for LTL formula handling and Spot integration.

#### Key Members

```cpp
spot::formula formula;                                  // Spot internal representation
std::vector<BatchAtomicProposition>* batchAPs;         // Task requirements
spot::twa_graph_ptr spotTWA;                           // Generated Büchi automaton
```

#### Key Methods

```cpp
// Constructor
LTLFormula(std::string formulaStr, const std::vector<BatchAtomicProposition>& batchAPs);

// Parsing and representation
void parse(const std::string& formula);
std::string toString() const;
bool isValid() const;

// Getters
spot::formula getSpotFormula() const;
const std::vector<BatchAtomicProposition>& getBatchAPs() const;
std::vector<BatchAtomicProposition>& getBatchAtomicPropositions();

// Atomic proposition queries
BatchAtomicProposition getBatchAP(uint16_t id) const;
uint16_t getTSState(uint16_t id) const;

// Evaluation
bool evaluate();
```

#### LTL Operator Reference

| Operator | Name | Meaning | Example |
|----------|------|---------|---------|
| `F` | Eventually | Becomes true at some future point | `F"at_A"` - eventually reach A |
| `G` | Globally | Always true throughout execution | `G"safe"` - always stay safe |
| `X` | Next | True in the immediate next state | `X"done"` - done after one step |
| `U` | Until | Left side until right becomes true | `"go" U "arrived"` - go until arrive |
| `R` | Release | Right holds unless left occurs | `"safe" R "stopped"` - safe until stop |
| `W` | Weak Until | Weak version of until (no guarantee) | `"move" W "tired"` - move weakly |
| `M` | Strong Release | Strong version of release | `"alert" M "danger"` - alert unless danger |

#### Usage Example

```cpp
#include "LTLFormula.h"

// 1. Define batch atomic propositions
std::vector<BatchAtomicProposition> batchAPs;
batchAPs.push_back(BatchAtomicProposition(0, "location_A", 1));
batchAPs.push_back(BatchAtomicProposition(1, "location_B", 2));

// 2. Create LTL formula
std::string taskSpec = "F(\"location_A\") & F(\"location_B\")";
LTLFormula task(taskSpec, batchAPs);

// 3. Validate
if (task.isValid()) {
    std::cout << "Valid task specification" << std::endl;
}

// 4. Get Spot representation
spot::formula spotFormula = task.getSpotFormula();
```

---

### BatchAtomicProposition

**Files**: `BatchAtomicProposition.h`, `BatchAtomicProposition.cpp`

Represents a single atomic proposition (task requirement).

#### Key Members

```cpp
uint16_t id;                    // Unique identifier
std::string name;               // Human-readable name (e.g., "location_A")
uint32_t associatedTSState;     // TS state where this AP is true
```

#### Key Methods

```cpp
// Constructor
BatchAtomicProposition(uint16_t id, const std::string& name, uint32_t tsState);

// Accessors
uint16_t getId() const;
const std::string& getName() const;
uint32_t getAssociatedTSState() const;

// Mutators
void setName(const std::string& newName);
void setAssociatedTSState(uint32_t tsState);

// Queries
bool isValidProposition() const;
std::string getDescription() const;
```

#### Usage Example

```cpp
#include "BatchAtomicProposition.h"

// Create propositions
BatchAtomicProposition p0(0, "pick_up_object", 5);
BatchAtomicProposition p1(1, "deliver_object", 12);
BatchAtomicProposition p2(2, "return_home", 0);

// Use in task specification
std::cout << "Proposition: " << p0.getName() << " at state " 
          << p0.getAssociatedTSState() << std::endl;
```

---

## LTL Syntax Guide

### Basic Syntax Rules

1. **Atomic Propositions**: Enclosed in double quotes
   ```
   "location_A"
   "holding_package"
   "battery_low"
   ```

2. **Boolean Operations**: Standard operators
   ```
   "p0" & "p1"          // AND
   "p0" | "p1"          // OR
   !"p0"                // NOT
   ```

3. **Temporal Operators**: Prefix notation
   ```
   F "p0"               // Eventually p0
   G "p0"               // Always p0
   X "p0"               // Next p0
   "p0" U "p1"          // p0 until p1
   ```

### Operator Precedence (High to Low)

1. Negation: `!`
2. AND: `&`
3. OR: `|`
4. Temporal operators: `F`, `G`, `X`, `U`, `R`, etc.

### Common Patterns

#### One-Time Tasks (Finite)

```ltl
F("p0") & F("p1")
  → Visit location p0, then visit location p1
  → Mission completes after both visited

F("p0" & "p1")
  → Eventually reach state where both p0 and p1 true
  → Must satisfy simultaneously in same state
```

#### Repeating Tasks (Infinite)

```ltl
G(F("p0"))
  → Infinitely often visit p0
  → Must repeatedly return to location p0
  → Cannot terminate

G(F("p0" & X("p1")))
  → Infinitely often: at p0, then immediately p1 next
  → Creates repeating pattern
```

#### Conditional Tasks

```ltl
G(("need_recharge" -> F("charging_station")))
  → Whenever needing recharge, eventually reach charging station
  → Reactive specification

F("reach_goal" & !("failed"))
  → Reach goal state without ever failing
  → Safety constraint embedded in goal
```

#### Sequencing

```ltl
"p0" U "p1"
  → Start at p0, maintain p0 until reaching p1
  → Enforces ordering

F("p0" & X(F("p1" & X(F("p2")))))
  → Visit p0, then p1, then p2 in sequence
  → Complex nesting creates specific order
```

---

## Integration with Planning

### Task to Automaton Pipeline

```
LTL Formula (text)
    ↓ [parse()]
Internal Representation (spot::formula)
    ↓ [Spot library conversion]
Büchi Automaton (states, edges, acceptance)
    ↓ [Product construction]
Planning State Space (TS × Büchi)
    ↓ [Task allocation algorithm]
Robot Task Sequence
```

### Atomic Proposition Binding

```cpp
// 1. Define APs tied to TS states
std::vector<BatchAtomicProposition> batchAPs;
batchAPs.push_back(BatchAtomicProposition(0, "at_A", 5));   // TS state 5
batchAPs.push_back(BatchAtomicProposition(1, "at_B", 12));  // TS state 12

// 2. Create LTL using AP names
LTLFormula task("F(\"at_A\") & F(\"at_B\")", batchAPs);

// 3. During planning, evaluation checks current TS state
// If current TS state == 5: proposition 0 ("at_A") is TRUE
// This allows planning algorithm to know which edges are executable
```

---

## Common Workflows

### Creating a Simple Task

```cpp
#include "LTLFormula.h"
#include "BatchAtomicProposition.h"

// Step 1: Define locations (APs)
std::vector<BatchAtomicProposition> locations;
locations.push_back(BatchAtomicProposition(0, "warehouse", 1));
locations.push_back(BatchAtomicProposition(1, "customer_A", 5));
locations.push_back(BatchAtomicProposition(2, "customer_B", 9));
locations.push_back(BatchAtomicProposition(3, "depot", 0));

// Step 2: Create task spec
std::string deliveryTask = 
    "F(\"warehouse\") & F(\"customer_A\") & F(\"customer_B\") & F(\"depot\")";

// Step 3: Instantiate formula
LTLFormula formula(deliveryTask, locations);

// Step 4: Verify
if (formula.isValid()) {
    std::cout << "Delivery task valid" << std::endl;
    
    // Get automaton
    spot::formula spotFormula = formula.getSpotFormula();
    // ... convert to Büchi automaton ...
}
```

### Creating a Patrol Task

```cpp
// Continuous patrol task (infinite)
std::vector<BatchAtomicProposition> patrolPoints;
patrolPoints.push_back(BatchAtomicProposition(0, "north", 10));
patrolPoints.push_back(BatchAtomicProposition(1, "south", 20));
patrolPoints.push_back(BatchAtomicProposition(2, "east", 15));
patrolPoints.push_back(BatchAtomicProposition(3, "west", 25));

// Infinite patrol: revisit all points forever
std::string patrolSpec = "G(F(\"north\") & F(\"south\") & F(\"east\") & F(\"west\"))";
LTLFormula patrol(patrolSpec, patrolPoints);

// This creates an infinite automaton requiring cyclic execution
```

### Creating a Conditional Task

```cpp
// Reactive task: if blocked at north, go around via east
std::vector<BatchAtomicProposition> points;
points.push_back(BatchAtomicProposition(0, "north", 10));
points.push_back(BatchAtomicProposition(1, "east", 15));
points.push_back(BatchAtomicProposition(2, "goal", 30));
points.push_back(BatchAtomicProposition(3, "blocked", 11));

// "If not blocked at north, go north to goal; otherwise go east then goal"
std::string reactiveSpec = 
    "(!(\"blocked\") U \"goal\") | (\"blocked\" U (\"east\" & F(\"goal\")))";

LTLFormula reactive(reactiveSpec, points);
```

---

## Finite vs. Infinite Classification

### Automatic Detection

The LTLFormula class automatically classifies formulas:

```cpp
bool isFinite = formula.isFinite();  // true = finite task, false = infinite
```

### Classification Rules

**Finite Formulas** (may terminate):
- No globally (G) operators
- All requirements are eventuality-based (F operations)
- Example: `F("p0") & F("p1") & F("p2")`

**Infinite Formulas** (must cycle):
- Contains globally (G) operators
- Creates repeating patterns
- Example: `G(F("p0"))`, `G(F("p0") & F("p1"))`

### Planning Impact

**Finite Planning**:
- Search terminates when all APs visited
- Aggressive pruning removes duplicate states
- Success = reaching state where all APs satisfied

**Infinite Planning**:
- Must discover cycles that revisit all APs
- Relaxed pruning allows multiple traversals
- Success = finding infinite path satisfying all APs infinitely often

---

## Performance Characteristics

### Parse Time

- Simple formula: O(|formula| × |APs|) - milliseconds
- Complex nested formula: O(2^depth) worst case
- Spot conversion: Depends on automaton complexity

### Memory Usage

- Formula representation: O(|APs|)
- Spot automaton: O(|states| + |edges|) typically manageable
- Batch APs vector: O(|APs|)

### Optimization Tips

1. **Reuse Formulas**: Cache parsed formulas to avoid reparsing
2. **Simplify Early**: Use logical minimization before Spot conversion
3. **Batch AP Definitions**: Define all APs once, reuse across formulas
4. **Profile Spot**: Use Spot profiling tools for large automata

---

## Common Errors and Troubleshooting

### Syntax Errors

**Error**: `"Invalid proposition format"`
- **Cause**: Missing quotes around AP names
- **Fix**: Use `F("location_A")` not `F(location_A)`

**Error**: `"Operator not recognized"`
- **Cause**: Wrong operator symbol
- **Fix**: Use `&` for AND, not `&&`; use `|` for OR, not `||`

### Semantic Errors

**Error**: `"Atomic proposition not found"`
- **Cause**: AP name in formula doesn't match defined APs
- **Fix**: Check exact spelling and capitalization

**Error**: `"Cyclic dependency detected"`
- **Cause**: APs reference non-existent TS states
- **Fix**: Verify all AP TS state IDs exist in TS

### Performance Issues

**Error**: `"Automaton explosion" (too many states)`
- **Cause**: Complex formula creates large automaton
- **Fix**: Simplify formula, break into subtasks

**Error**: `"Timeout during formula parsing"`
- **Cause**: Deeply nested operators
- **Fix**: Use more concise formula syntax

---

## Spot Library Integration

### Spot Formula Creation

```cpp
// Direct Spot formula creation (advanced)
auto f = spot::parse_infix_psl("F(\"p0\") & F(\"p1\")", false);

// Convert to automaton
auto aut = spot::translate(f, spot::postprocess::BUCHI);

// Export to DOT
std::ofstream out("automaton.dot");
spot::print_dot(out, aut);
out.close();
```

### Debugging Spot Issues

```cpp
// Print formula
std::cout << "Formula: " << formula.getSpotFormula() << std::endl;

// Check satisfiability
auto aut = spot::translate(formula.getSpotFormula());
if (aut->num_states() == 0) {
    std::cerr << "Warning: Formula may be unsatisfiable" << std::endl;
}
```

---

## Advanced Topics

### Custom Atomic Propositions

```cpp
// APs can represent complex conditions
BatchAtomicProposition safe_range(
    0, 
    "within_safe_range",
    42  // TS state representing safe zone
);

// Use in formula
std::string safeTask = "G(\"within_safe_range\")";
LTLFormula formula(safeTask, {safe_range});
```

### Formula Composition

```cpp
// Combine multiple formulas
LTLFormula task1("F(\"p0\")", aps1);
LTLFormula task2("F(\"p1\")", aps2);

// Create combined specification
// Note: Direct formula combination requires string manipulation
std::string combined = "(" + task1.toString() + ") & (" + task2.toString() + ")";
LTLFormula combined_task(combined, all_aps);
```

### Handling Assumptions and Guarantees

```cpp
// Assumption-Guarantee formalism
// Assumption: environment always provides required signals
// Guarantee: robot achieves goal

std::string assumption = "G(\"signal_available\")";
std::string guarantee = "F(\"goal_reached\")";

std::string spec = "(" + assumption + ") i (" + guarantee + ")";
LTLFormula agFormula(spec, aps);
```

---

## See Also

- [../README.md](../README.md) - Main module documentation
- [../Environment/README.md](../Environment/README.md) - AP binding to TS states
- [../../Automatons/README.md](../../Automatons/README.md) - Automata from LTL
- [https://spot.lrde.epita.fr/](https://spot.lrde.epita.fr/) - Spot library documentation

---

**Last Updated**: 2026-07-21  
**Status**: Complete with Spot integration
