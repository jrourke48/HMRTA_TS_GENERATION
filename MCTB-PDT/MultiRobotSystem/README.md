# MultiRobotSystem Module

## Overview

The MultiRobotSystem module manages heterogeneous robot teams with varying capabilities, constraints, and configurations. It provides abstractions for individual robots and system-level coordination.

### Key Responsibilities

1. **Robot Management** - Track individual robots and their properties
2. **Capability Modeling** - Define and query robot capabilities
3. **Multi-Robot Coordination** - Manage team-level planning decisions
4. **Constraint Handling** - Enforce robot and system limitations
5. **Configuration Management** - Track robot setup and parameters

## Components

### Robot

**Files**: `Robot.h`, `Robot.cpp`

Represents a single robot in the system.

#### Key Members

```cpp
uint32_t robotId;                   // Unique identifier
std::string name;                   // Robot name (e.g., "Robot_1")
Point startPosition;                // Initial position
Point currentPosition;              // Current location
uint8_t velo;                       // Velocity (grid units per second)
std::vector<bool> capabilities;     // Capability flags
```

#### Key Methods

```cpp
// Identification
uint32_t getRobotId() const;
const std::string& getName() const;
void setName(const std::string& name);

// Position tracking
const Point& getStartPosition() const;
const Point& getCurrentPosition() const;
void setCurrentPosition(const Point& position);

// Motion properties
uint8_t getVelocity() const;
void setVelocity(uint8_t v);

// Capabilities
const std::vector<bool>& getCapabilities() const;
bool hasCapability(RobotCapability cap) const;
void addCapability(RobotCapability cap);
void removeCapability(RobotCapability cap);
```

#### Usage Example

```cpp
#include "Robot.h"
#include "RobotCapabilities.h"

// Create robot
Robot robot1(0, "Robot_1", Point(0, 0));

// Configure capabilities
robot1.addCapability(ROBOT_CAPABILITY::MOVE);
robot1.addCapability(ROBOT_CAPABILITY::SENSE);
robot1.addCapability(ROBOT_CAPABILITY::MANIPULATE);

// Set velocity
robot1.setVelocity(2);  // 2 units per second

// Query
if (robot1.hasCapability(ROBOT_CAPABILITY::MANIPULATE)) {
    cout << "Robot can manipulate objects" << endl;
}

// Update state
robot1.setCurrentPosition(Point(50, 50));
```

---

### RobotCapabilities

**File**: `RobotCapabilities.h`

Defines the set of available robot capabilities.

#### Capability Enum

```cpp
enum class ROBOT_CAPABILITY {
    MOVE,           // Basic locomotion
    SENSE,          // Sensor capability
    MANIPULATE,     // Grasping/manipulation
    COMMUNICATE,    // Inter-robot communication
    COMPUTE,        // On-board computation
    CARRY,          // Cargo transportation
    DEPLOY,         // Equipment deployment
    DEFUSE,         // Explosive handling (specialized)
    // Add more as needed
};
```

#### Usage

```cpp
// Check capability
if (robot.hasCapability(ROBOT_CAPABILITY::MANIPULATE)) {
    // Can execute manipulation tasks
}

// Add multiple capabilities
robot.addCapability(ROBOT_CAPABILITY::MOVE);
robot.addCapability(ROBOT_CAPABILITY::MANIPULATE);
robot.addCapability(ROBOT_CAPABILITY::CARRY);
```

---

### MultiRobotSystem

**Files**: `MultiRobotSystem.h`, `MultiRobotSystem.cpp`

Manages a team of robots and system-level operations.

#### Key Members

```cpp
std::vector<Robot*> robots;         // Team of robots
uint32_t numRobots;                 // Team size
std::map<uint32_t, Robot*> robotMap; // ID-to-Robot mapping
```

#### Key Methods

```cpp
// Constructor
MultiRobotSystem(uint32_t numRobots = 0);

// Robot management
void addRobot(Robot* robot);
Robot* getRobot(uint32_t robotId) const;
const std::vector<Robot*>& getAllRobots() const;
uint32_t getNumRobots() const;

// Team queries
std::vector<Robot*> getRobotsWithCapability(RobotCapability cap) const;
bool hasCapabilityInTeam(RobotCapability cap) const;
uint32_t countCapabilityInTeam(RobotCapability cap) const;

// Coordination
bool allocateTask(uint32_t robotId, const std::string& taskDescription);
bool checkTeamCapabilities(const std::vector<RobotCapability>& required) const;

// System state
void printSystemState() const;
double getTeamCost() const;
```

#### Usage Example

```cpp
#include "MultiRobotSystem.h"

// Create team with 3 robots
MultiRobotSystem team(3);

// Configure each robot
Robot* r1 = new Robot(0, "Robot_1", Point(0, 0));
r1->addCapability(ROBOT_CAPABILITY::MOVE);
r1->addCapability(ROBOT_CAPABILITY::SENSE);
team.addRobot(r1);

Robot* r2 = new Robot(1, "Robot_2", Point(10, 10));
r2->addCapability(ROBOT_CAPABILITY::MOVE);
r2->addCapability(ROBOT_CAPABILITY::MANIPULATE);
team.addRobot(r2);

Robot* r3 = new Robot(2, "Robot_3", Point(20, 20));
r3->addCapability(ROBOT_CAPABILITY::MOVE);
r3->addCapability(ROBOT_CAPABILITY::COMMUNICATE);
team.addRobot(r3);

// Query team capabilities
vector<Robot*> movers = team.getRobotsWithCapability(ROBOT_CAPABILITY::MOVE);
cout << "Robots that can move: " << movers.size() << endl;

// Check if team has required capabilities
vector<RobotCapability> needed = {
    ROBOT_CAPABILITY::MOVE,
    ROBOT_CAPABILITY::MANIPULATE
};
if (team.checkTeamCapabilities(needed)) {
    cout << "Team can handle task" << endl;
}
```

---

## Robot Configuration Patterns

### Homogeneous Team (All Same)

```cpp
// Create 5 identical robots
MultiRobotSystem team(5);
for (int i = 0; i < 5; i++) {
    Robot* r = new Robot(i, "Robot_" + to_string(i), Point(0, 0));
    r->addCapability(ROBOT_CAPABILITY::MOVE);
    r->addCapability(ROBOT_CAPABILITY::SENSE);
    r->setVelocity(2);
    team.addRobot(r);
}
```

### Heterogeneous Team (Mixed)

```cpp
// Scouts: fast, sensing only
Robot* scout = new Robot(0, "Scout", Point(0, 0));
scout->addCapability(ROBOT_CAPABILITY::MOVE);
scout->addCapability(ROBOT_CAPABILITY::SENSE);
scout->setVelocity(4);  // Fast
team.addRobot(scout);

// Manipulators: slow, can grab
Robot* arm = new Robot(1, "Arm", Point(0, 0));
arm->addCapability(ROBOT_CAPABILITY::MOVE);
arm->addCapability(ROBOT_CAPABILITY::MANIPULATE);
arm->addCapability(ROBOT_CAPABILITY::CARRY);
arm->setVelocity(1);   // Slow
team.addRobot(arm);

// Communicators: medium-speed relay
Robot* relay = new Robot(2, "Relay", Point(0, 0));
relay->addCapability(ROBOT_CAPABILITY::MOVE);
relay->addCapability(ROBOT_CAPABILITY::COMMUNICATE);
relay->setVelocity(3);
team.addRobot(relay);
```

### Specialized Roles

```cpp
// Define specialized robots for specific scenarios

// Demolitions expert
Robot* expert = new Robot(0, "Demolition_Expert", Point(0, 0));
expert->addCapability(ROBOT_CAPABILITY::MOVE);
expert->addCapability(ROBOT_CAPABILITY::MANIPULATE);
expert->addCapability(ROBOT_CAPABILITY::DEFUSE);
team.addRobot(expert);

// Support robot
Robot* support = new Robot(1, "Support", Point(0, 0));
support->addCapability(ROBOT_CAPABILITY::MOVE);
support->addCapability(ROBOT_CAPABILITY::COMMUNICATE);
support->addCapability(ROBOT_CAPABILITY::CARRY);
team.addRobot(support);
```

---

## Multi-Robot Planning Integration

### Capability-Based Task Assignment

```cpp
// LTL formula specifies required capabilities
std::vector<RobotCapability> requiredCaps = {
    ROBOT_CAPABILITY::MOVE,
    ROBOT_CAPABILITY::MANIPULATE
};

// Check which robots can perform task
vector<Robot*> qualified = team.getRobotsWithCapability(
    ROBOT_CAPABILITY::MANIPULATE);

if (!qualified.empty()) {
    // Assign to first available robot
    Robot* assigned = qualified[0];
    cout << "Task assigned to " << assigned->getName() << endl;
}
```

### Coordinated Motion Planning

```cpp
// Plan paths for multiple robots simultaneously
vector<Robot*> robots = team.getAllRobots();
map<uint32_t, vector<Point>> paths;

for (Robot* r : robots) {
    if (r->hasCapability(ROBOT_CAPABILITY::MOVE)) {
        // Plan path for this robot
        vector<Point> path = planPath(
            r->getCurrentPosition(),
            goal_position,
            obstacles
        );
        paths[r->getRobotId()] = path;
    }
}

// Check for collisions between paths
bool collision_free = checkPathCollisions(paths);
```

### Capability-Aware Task Decomposition

```cpp
// Break complex task into subtasks matching robot capabilities

struct SubTask {
    std::string description;
    RobotCapability required_capability;
};

vector<SubTask> decompose_task(const string& complex_task) {
    // Parse task and determine required capabilities
    vector<SubTask> subtasks;
    
    // Example: "move to location and manipulate"
    subtasks.push_back({"Move to location", ROBOT_CAPABILITY::MOVE});
    subtasks.push_back({"Manipulate object", ROBOT_CAPABILITY::MANIPULATE});
    
    return subtasks;
}

// Assign subtasks to team
vector<SubTask> subtasks = decompose_task("move and manipulate");
for (const auto& st : subtasks) {
    auto robots = team.getRobotsWithCapability(st.required_capability);
    if (!robots.empty()) {
        // Assign to robot
    }
}
```

---

## System Configuration Examples

### Search and Rescue Team

```cpp
MultiRobotSystem sar_team(4);

// Scout: Fast reconnaissance
Robot* scout = new Robot(0, "Scout", Point(0, 0));
scout->addCapability(ROBOT_CAPABILITY::MOVE);
scout->addCapability(ROBOT_CAPABILITY::SENSE);
scout->setVelocity(3);
sar_team.addRobot(scout);

// Rescue: Strong, careful movement
Robot* rescue = new Robot(1, "Rescue", Point(0, 0));
rescue->addCapability(ROBOT_CAPABILITY::MOVE);
rescue->addCapability(ROBOT_CAPABILITY::MANIPULATE);
rescue->addCapability(ROBOT_CAPABILITY::CARRY);
rescue->setVelocity(1);
sar_team.addRobot(rescue);

// Medic: Sensing and communication
Robot* medic = new Robot(2, "Medic", Point(0, 0));
medic->addCapability(ROBOT_CAPABILITY::SENSE);
medic->addCapability(ROBOT_CAPABILITY::COMMUNICATE);
medic->setVelocity(2);
sar_team.addRobot(medic);

// Relay: Communication hub
Robot* relay = new Robot(3, "Relay", Point(0, 0));
relay->addCapability(ROBOT_CAPABILITY::MOVE);
relay->addCapability(ROBOT_CAPABILITY::COMMUNICATE);
relay->setVelocity(2);
sar_team.addRobot(relay);
```

### Factory Automation Team

```cpp
MultiRobotSystem factory_team(3);

// Material handler
Robot* handler = new Robot(0, "Handler", Point(0, 0));
handler->addCapability(ROBOT_CAPABILITY::MOVE);
handler->addCapability(ROBOT_CAPABILITY::CARRY);
handler->setVelocity(2);
factory_team.addRobot(handler);

// Precision assembler
Robot* assembler = new Robot(1, "Assembler", Point(10, 0));
assembler->addCapability(ROBOT_CAPABILITY::MANIPULATE);
assembler->addCapability(ROBOT_CAPABILITY::COMPUTE);
assembler->setVelocity(1);
factory_team.addRobot(assembler);

// Quality inspector
Robot* inspector = new Robot(2, "Inspector", Point(20, 0));
inspector->addCapability(ROBOT_CAPABILITY::SENSE);
inspector->addCapability(ROBOT_CAPABILITY::COMPUTE);
inspector->setVelocity(1);
factory_team.addRobot(inspector);
```

---

## Performance Characteristics

### Time Complexity

- **Robot lookup by ID**: O(log n) with map, O(n) with vector
- **Find robots with capability**: O(n × m) where m = capabilities
- **Team capability check**: O(n) 

### Space Complexity

- **Robot storage**: O(n × capabilities)
- **Robot mappings**: O(n)
- **Total**: O(n × (m + coordinates)) = O(n) typically

### Optimization Tips

1. **Cache Capability Queries**: Store robot capability sets for frequent queries
2. **Use Spatial Hashing**: Organize robots by position for neighbor queries
3. **Lazy Capability Computation**: Only recompute when robot changes
4. **Batch Assignments**: Process multiple task assignments together

---

## Multi-Robot Coordination Strategies

### Sequential Assignment

```cpp
// Assign tasks one by one to available robots
for (const auto& task : task_queue) {
    for (Robot* robot : team.getAllRobots()) {
        if (robot->hasCapability(task.required_capability)) {
            assignTaskToRobot(robot, task);
            break;
        }
    }
}
```

### Capability-Based Clustering

```cpp
// Group robots by complementary capabilities
map<set<RobotCapability>, vector<Robot*>> clusters;

for (Robot* robot : team.getAllRobots()) {
    set<RobotCapability> caps;
    for (int i = 0; i < robot->getCapabilities().size(); i++) {
        if (robot->getCapabilities()[i]) {
            caps.insert(static_cast<RobotCapability>(i));
        }
    }
    clusters[caps].push_back(robot);
}

// Form teams from clusters with complementary capabilities
```

### Load Balancing

```cpp
// Distribute workload evenly across team
map<uint32_t, int> task_count;

for (Robot* robot : team.getAllRobots()) {
    task_count[robot->getRobotId()] = 0;
}

for (const auto& task : tasks) {
    // Find robot with least tasks
    auto min_robot = min_element(
        team.getAllRobots().begin(),
        team.getAllRobots().end(),
        [&](Robot* a, Robot* b) {
            return task_count[a->getRobotId()] < task_count[b->getRobotId()];
        }
    );
    
    assignTaskToRobot(*min_robot, task);
    task_count[(*min_robot)->getRobotId()]++;
}
```

---

## Debugging and Monitoring

### System State Visualization

```cpp
void printMultiRobotState(const MultiRobotSystem& team) {
    cout << "=== Multi-Robot System State ===" << endl;
    cout << "Total Robots: " << team.getNumRobots() << endl;
    
    for (Robot* robot : team.getAllRobots()) {
        cout << "\nRobot: " << robot->getName() << endl;
        cout << "  ID: " << robot->getRobotId() << endl;
        cout << "  Position: (" << robot->getCurrentPosition().getX() 
             << ", " << robot->getCurrentPosition().getY() << ")" << endl;
        cout << "  Velocity: " << (int)robot->getVelocity() << " units/sec" << endl;
        cout << "  Capabilities: ";
        
        if (robot->hasCapability(ROBOT_CAPABILITY::MOVE)) cout << "Move ";
        if (robot->hasCapability(ROBOT_CAPABILITY::SENSE)) cout << "Sense ";
        if (robot->hasCapability(ROBOT_CAPABILITY::MANIPULATE)) cout << "Manipulate ";
        cout << endl;
    }
}
```

### Capability Analysis

```cpp
void analyzeTeamCapabilities(const MultiRobotSystem& team) {
    cout << "Team Capability Analysis:" << endl;
    
    map<string, int> capability_count = {
        {"MOVE", 0},
        {"SENSE", 0},
        {"MANIPULATE", 0},
        {"COMMUNICATE", 0},
        {"COMPUTE", 0}
    };
    
    for (Robot* r : team.getAllRobots()) {
        if (r->hasCapability(ROBOT_CAPABILITY::MOVE)) capability_count["MOVE"]++;
        if (r->hasCapability(ROBOT_CAPABILITY::SENSE)) capability_count["SENSE"]++;
        if (r->hasCapability(ROBOT_CAPABILITY::MANIPULATE)) capability_count["MANIPULATE"]++;
        if (r->hasCapability(ROBOT_CAPABILITY::COMMUNICATE)) capability_count["COMMUNICATE"]++;
        if (r->hasCapability(ROBOT_CAPABILITY::COMPUTE)) capability_count["COMPUTE"]++;
    }
    
    for (const auto& [cap, count] : capability_count) {
        cout << "  " << cap << ": " << count << " robots" << endl;
    }
}
```

---

## See Also

- [../README.md](../README.md) - Main module documentation
- [../LTLFormula/README.md](../LTLFormula/README.md) - Task specifications
- [../../Automatons/README.md](../../Automatons/README.md) - Capability constraints
- [../Environment/README.md](../Environment/README.md) - Robot motion in environment

---

**Last Updated**: 2026-07-21  
**Status**: Complete with heterogeneous team support
