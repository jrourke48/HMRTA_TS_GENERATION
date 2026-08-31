# Organized Tests

## Important independent variables for the thesis:
1. Number of automaton states
2. Number of automaton edges
3. Number of Atomic propositions
4. Number of robots
5. Total robot capabilities in fleet
6. Number of regions in transition system
7. (total robot capabilities) / Number of robots - average number of capabilities for a robot in your fleet
8. (Total independent capabilities)/ Number of robots - how homogeneous your fleet is
9. Number of inter-task constraints: unrelated, exclusive, complementary

Want to try to get as comprehensive of an idea of the algorithms behavior without running an astronomical amount of tests

## variable ranges and number of tests
We are going to run tests for 3, 6, 15 and 45 robot configuration 6 regions and maybe 3 different buchis for every scenario where those variables are not involved
### Number of automaton states
try 15 different state numbers: 5-150 states
### Number of robots
try 8 different numbers of robots: 3-20
### regions of transition system
try 10 different transition system partitions: 5-40 states
Other possible tests 
### average capabilities
try 10 different average capabilities: 1-5
### robot Homogenity
try 10 different robot Homogenity: 0.2-3

Need a way to store all the data and plot it in simueltaneusly



