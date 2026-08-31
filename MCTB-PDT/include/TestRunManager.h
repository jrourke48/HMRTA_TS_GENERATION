#ifndef TEST_RUN_MANAGER_H
#define TEST_RUN_MANAGER_H

#include <string>
#include <vector>
#include <map>
#include <memory>
#include "AlgorithmMetrics.h"

/**
 * @class TestRunManager
 * @brief Manages and organizes test runs by category for comprehensive thesis evaluation
 * 
 * Separates test results into distinct categories to enable independent analysis:
 * - Automaton State Scaling
 * - Robot Count Scaling
 * - Transition System Regions Scaling
 * - Average Capabilities Variation
 * - Robot Homogeneity Variation
 */
class TestRunManager {
public:
    // ==================== TEST CATEGORIES ====================
    enum class TestCategory {
        AUTOMATON_STATES,           // 5-150 states: 15 tests
        NUM_ROBOTS,                 // 3-20 robots: 8 tests
        TS_REGIONS,                 // 5-40 regions: 10 tests
        AVG_CAPABILITIES,           // 1-5 avg capabilities: 10 tests
        ROBOT_HOMOGENEITY           // 0.2-3 homogeneity: 10 tests
    };
    
    // ==================== TEST RUN DATA ====================
    struct TestRun {
        int run_id;
        int trial_number;           // Multiple runs for same config
        AlgorithmMetrics metrics;
        std::map<std::string, std::string> parameters;  // Variable config
        std::string timestamp;
        std::string notes;          // Optional: error notes, special observations
    };
    
    struct TestCategoryData {
        TestCategory category;
        std::string category_name;
        std::vector<TestRun> runs;
        std::string description;
    };
    
    // ==================== PUBLIC INTERFACE ====================
public:
    TestRunManager(const std::string& data_dir = "./test_results");
    ~TestRunManager();
    
    // Initialize directory structure for all test categories
    void initialize();
    
    // Store a single run in appropriate category
    void storeRun(TestCategory category, 
                  const AlgorithmMetrics& metrics,
                  const std::map<std::string, std::string>& parameters,
                  int trial_number = 1,
                  const std::string& notes = "");
    
    // Retrieve runs by category
    std::vector<TestRun> getRuns(TestCategory category) const;
    
    // Retrieve runs by category and filter parameters
    std::vector<TestRun> getRuns(TestCategory category,
                                 const std::map<std::string, std::string>& filters) const;
    
    // ==================== STATISTICS & AGGREGATION ====================
    
    struct RunStatistics {
        std::map<std::string, std::string> parameter_config;
        int num_runs;
        
        // Time statistics
        double mean_time_ms;
        double stddev_time_ms;
        double min_time_ms;
        double max_time_ms;
        
        // Efficiency statistics
        double mean_pruning_ratio;
        double stddev_pruning_ratio;
        double mean_explored_ratio;
        double stddev_explored_ratio;
        
        // Quality statistics
        double mean_makespan_seconds;
        double stddev_makespan_seconds;
        double mean_robot_util;
    };
    
    // Get aggregated statistics for a category
    std::vector<RunStatistics> getStatistics(TestCategory category) const;
    
    // ==================== EXPORT FOR ANALYSIS ====================
    
    // Export single category to CSV for plotting
    void exportCategoryToCSV(TestCategory category,
                            const std::string& output_filename) const;
    
    // Export all categories
    void exportAllToCSV() const;
    
    // Export summary report
    void exportSummaryReport(const std::string& output_filename) const;
    
    // Export aggregated statistics
    void exportStatisticsToCSV(TestCategory category,
                              const std::string& output_filename) const;
    
    // ==================== ANALYSIS HELPERS ====================
    
    // Get metadata about a category
    int getExpectedNumberOfRuns(TestCategory category) const;
    int getCurrentNumberOfRuns(TestCategory category) const;
    double getCompletionPercentage(TestCategory category) const;
    
    // Print summary of all test progress
    void printTestProgress() const;
    
    // Get category name as string
    static std::string getCategoryName(TestCategory category);
    
    // ==================== INTERNAL DATA ====================
    
private:
    std::string data_dir_;
    std::map<TestCategory, TestCategoryData> category_data_;
    
    // Helper methods
    std::string getCategoryDirPath(TestCategory category) const;
    std::string getRunFilename(TestCategory category, int run_id) const;
    
    void createCategoryDirectory(TestCategory category);
    void loadCategoryData(TestCategory category);
    void saveCategoryData(TestCategory category) const;
    
    // Serialization
    std::string serializeTestRun(const TestRun& run) const;
    TestRun deserializeTestRun(const std::string& json_str) const;
};

#endif // TEST_RUN_MANAGER_H
