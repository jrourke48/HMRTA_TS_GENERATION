#include "../MultiRobotSystem/MultiRobotSystem.h"
#include "../MultiRobotSystem/Robot.h"
#include "../MultiRobotSystem/RobotCapabilities.h"
#include <iostream>
#include <cassert>

/**
 * Test 1: Basic robot creation and management
 */
void testRobotCreation() {
    std::cout << "\n=== Test 1: Robot Creation ===" << std::endl;
    
    Robot robot1(1, "Rover_1", Point(0, 0));
    assert(robot1.getRobotId() == 1);
    assert(robot1.getName() == "Rover_1");
    
    robot1.initializeCapabilities(13);
    robot1.enableCapability(RobotCapability::MOVEMENT_GROUND);
    robot1.enableCapability(RobotCapability::SENSOR_GPS);
    
    assert(robot1.hasCapability(RobotCapability::MOVEMENT_GROUND));
    assert(robot1.hasCapability(RobotCapability::SENSOR_GPS));
    assert(!robot1.hasCapability(RobotCapability::MOVEMENT_AERIAL));
    
    std::cout << "✓ Robot creation and capability management works" << std::endl;
}

/**
 * Test 2: MultiRobotSystem creation and robot addition
 */
void testMultiRobotSystemCreation() {
    std::cout << "\n=== Test 2: MultiRobotSystem Creation ===" << std::endl;
    
    MultiRobotSystem system(3);
    assert(system.getNumRobots() == 3);
    
    Robot* rover = new Robot(1, "Rover_1", Point(0, 0));
    rover->initializeCapabilities(13);
    rover->enableCapability(RobotCapability::MOVEMENT_GROUND);
    rover->enableCapability(RobotCapability::SENSOR_CAMERA);
    rover->enableCapability(RobotCapability::SENSOR_GPS);
    
    system.addRobot(rover);
    assert(system.getRobot(1) == rover);
    
    std::cout << "✓ MultiRobotSystem creation and robot addition works" << std::endl;
}

/**
 * Test 3: Capability queries
 */
void testCapabilityQueries() {
    std::cout << "\n=== Test 3: Capability Queries ===" << std::endl;
    
    MultiRobotSystem system;
    
    // Create ground robot
    Robot* rover = new Robot(1, "Rover_1");
    rover->initializeCapabilities(13);
    rover->enableCapability(RobotCapability::MOVEMENT_GROUND);
    rover->enableCapability(RobotCapability::SENSOR_CAMERA);
    system.addRobot(rover);
    
    // Create aerial robot
    Robot* drone = new Robot(2, "Drone_1");
    drone->initializeCapabilities(13);
    drone->enableCapability(RobotCapability::MOVEMENT_AERIAL);
    drone->enableCapability(RobotCapability::SENSOR_LIDAR);
    system.addRobot(drone);
    
    // Test single capability queries
    assert(system.countRobotsWithCapability(RobotCapability::MOVEMENT_GROUND) == 1);
    assert(system.countRobotsWithCapability(RobotCapability::MOVEMENT_AERIAL) == 1);
    assert(system.hasRobotWithCapability(RobotCapability::MOVEMENT_GROUND));
    assert(system.hasRobotWithCapability(RobotCapability::SENSOR_CAMERA));
    
    auto groundRobots = system.getRobotsWithCapability(RobotCapability::MOVEMENT_GROUND);
    assert(groundRobots.size() == 1);
    assert(groundRobots[0]->getRobotId() == 1);
    
    std::cout << "✓ Capability queries work correctly" << std::endl;
}

/**
 * Test 4: Multiple capability queries
 */
void testMultipleCapabilityQueries() {
    std::cout << "\n=== Test 4: Multiple Capability Queries ===" << std::endl;
    
    MultiRobotSystem system;
    
    // Create robots with different capability combinations
    Robot* r1 = new Robot(1, "Robot_A");
    r1->initializeCapabilities(13);
    r1->enableCapability(RobotCapability::MOVEMENT_GROUND);
    r1->enableCapability(RobotCapability::SENSOR_GPS);
    system.addRobot(r1);
    
    Robot* r2 = new Robot(2, "Robot_B");
    r2->initializeCapabilities(13);
    r2->enableCapability(RobotCapability::MOVEMENT_GROUND);
    r2->enableCapability(RobotCapability::SENSOR_CAMERA);
    system.addRobot(r2);
    
    Robot* r3 = new Robot(3, "Robot_C");
    r3->initializeCapabilities(13);
    r3->enableCapability(RobotCapability::MOVEMENT_AERIAL);
    system.addRobot(r3);
    
    // Test getRobotsWithAllCapabilities
    std::vector<RobotCapability> allCaps = {RobotCapability::MOVEMENT_GROUND, RobotCapability::SENSOR_GPS};
    auto withAll = system.getRobotsWithAllCapabilities(allCaps);
    assert(withAll.size() == 1);
    assert(withAll[0]->getRobotId() == 1);
    
    // Test getRobotsWithAnyCapability
    std::vector<RobotCapability> anyCaps = {RobotCapability::SENSOR_GPS, RobotCapability::SENSOR_CAMERA};
    auto withAny = system.getRobotsWithAnyCapability(anyCaps);
    assert(withAny.size() == 2);
    
    std::cout << "✓ Multiple capability queries work correctly" << std::endl;
}

/**
 * Test 5: to_string() - Formatted table output
 */
void testToString() {
    std::cout << "\n=== Test 5: Formatted Table Output (to_string) ===" << std::endl;
    
    MultiRobotSystem system;
    
    // Create diverse robots
    Robot* rover = new Robot(1, "Rover_1");
    rover->initializeCapabilities(13);
    rover->enableCapability(RobotCapability::MOVEMENT_GROUND);
    rover->enableCapability(RobotCapability::SENSOR_CAMERA);
    rover->enableCapability(RobotCapability::SENSOR_GPS);
    system.addRobot(rover);
    
    Robot* drone = new Robot(2, "Drone_1");
    drone->initializeCapabilities(13);
    drone->enableCapability(RobotCapability::MOVEMENT_AERIAL);
    drone->enableCapability(RobotCapability::SENSOR_LIDAR);
    drone->enableCapability(RobotCapability::COMMUNICATION_WIFI);
    system.addRobot(drone);
    
    Robot* aquabot = new Robot(3, "AquaBot_1");
    aquabot->initializeCapabilities(13);
    aquabot->enableCapability(RobotCapability::MOVEMENT_AQUATIC);
    aquabot->enableCapability(RobotCapability::SENSOR_PROXIMITY);
    system.addRobot(aquabot);
    
    // Print the formatted table
    std::cout << system.to_string() << std::endl;
    
    std::cout << "✓ Formatted table output generated successfully" << std::endl;
}

/**
 * Main test runner
 */
int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  MultiRobotSystem Test Suite" << std::endl;
    std::cout << "========================================" << std::endl;
    
    try {
        testRobotCreation();
        testMultiRobotSystemCreation();
        testCapabilityQueries();
        testMultipleCapabilityQueries();
        testToString();
        
        std::cout << "\n========================================" << std::endl;
        std::cout << "  All tests passed! ✓" << std::endl;
        std::cout << "========================================" << std::endl;
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
}
