#include "TestRunManager.h"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <ctime>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <set>

namespace fs = std::filesystem;

// ==================== CONSTRUCTOR & INITIALIZATION ====================

TestRunManager::TestRunManager(const std::string& data_dir)
    : data_dir_(data_dir) {
    // Initialize all categories with expected run counts
    category_data_[TestCategory::AUTOMATON_STATES] = {
        TestCategory::AUTOMATON_STATES,
        "Automaton State Scaling",
        {},
        "Testing algorithm performance with 5-150 automaton states (15 tests)"
    };
    category_data_[TestCategory::NUM_ROBOTS] = {
        TestCategory::NUM_ROBOTS,
        "Robot Count Scaling",
        {},
        "Testing algorithm performance with 3-20 robots (8 tests)"
    };
    category_data_[TestCategory::TS_REGIONS] = {
        TestCategory::TS_REGIONS,
        "Transition System Regions",
        {},
        "Testing algorithm performance with 5-40 TS regions (10 tests)"
    };
    category_data_[TestCategory::AVG_CAPABILITIES] = {
        TestCategory::AVG_CAPABILITIES,
        "Average Capabilities Variation",
        {},
        "Testing algorithm performance with 1-5 average capabilities (10 tests)"
    };
    category_data_[TestCategory::ROBOT_HOMOGENEITY] = {
        TestCategory::ROBOT_HOMOGENEITY,
        "Robot Homogeneity Variation",
        {},
        "Testing algorithm performance with 0.2-3 homogeneity index (10 tests)"
    };
}

TestRunManager::~TestRunManager() {
    // Cleanup if needed
}

void TestRunManager::initialize() {
    // Create main data directory
    if (!fs::exists(data_dir_)) {
        fs::create_directories(data_dir_);
        std::cout << "✓ Created results directory: " << data_dir_ << std::endl;
    }
    
    // Create subdirectories for each test category
    for (const auto& [category, data] : category_data_) {
        createCategoryDirectory(category);
    }
    
    // Create metadata file (simple text format, no JSON dependency)
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    
    std::ofstream meta_file(data_dir_ + "/metadata.txt");
    meta_file << "Test Manager Initialized\n";
    meta_file << "Created: " << oss.str() << "\n";
    meta_file << "Data Version: 1.0\n";
    meta_file << "Categories: " << category_data_.size() << "\n";
    meta_file.close();
    
    std::cout << "✓ Test manager initialized with " << category_data_.size() 
              << " test categories" << std::endl;
}

// ==================== STORE & RETRIEVE ====================

void TestRunManager::storeRun(TestCategory category,
                              const AlgorithmMetrics& metrics,
                              const std::map<std::string, std::string>& parameters,
                              int trial_number,
                              const std::string& notes) {
    // Create TestRun object
    TestRun run;
    run.run_id = category_data_[category].runs.size() + 1;
    run.trial_number = trial_number;
    run.metrics = metrics;
    run.parameters = parameters;
    run.notes = notes;
    
    // Get current timestamp
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    run.timestamp = oss.str();
    
    // Store in memory
    category_data_[category].runs.push_back(run);
    
    // Persist to file
    saveCategoryData(category);
    
    std::cout << "✓ Stored run " << run.run_id << " in category \"" 
              << getCategoryName(category) << "\" (trial " << trial_number << ")"
              << std::endl;
}

std::vector<TestRunManager::TestRun> TestRunManager::getRuns(TestCategory category) const {
    auto it = category_data_.find(category);
    if (it != category_data_.end()) {
        return it->second.runs;
    }
    return {};
}

std::vector<TestRunManager::TestRun> TestRunManager::getRuns(
    TestCategory category,
    const std::map<std::string, std::string>& filters) const {
    
    std::vector<TestRun> filtered;
    auto runs = getRuns(category);
    
    for (const auto& run : runs) {
        bool matches = true;
        for (const auto& [key, value] : filters) {
            auto it = run.parameters.find(key);
            if (it == run.parameters.end() || it->second != value) {
                matches = false;
                break;
            }
        }
        if (matches) {
            filtered.push_back(run);
        }
    }
    
    return filtered;
}

// ==================== STATISTICS & AGGREGATION ====================

std::vector<TestRunManager::RunStatistics> TestRunManager::getStatistics(
    TestCategory category) const {
    
    auto runs = getRuns(category);
    std::map<std::map<std::string, std::string>, std::vector<TestRun>> grouped;
    
    // Group runs by parameter configuration
    for (const auto& run : runs) {
        grouped[run.parameters].push_back(run);
    }
    
    std::vector<RunStatistics> stats;
    
    for (const auto& [params, group_runs] : grouped) {
        RunStatistics stat;
        stat.parameter_config = params;
        stat.num_runs = group_runs.size();
        
        // Calculate time statistics
        std::vector<double> times;
        std::vector<double> pruning_ratios;
        std::vector<double> explored_ratios;
        std::vector<double> makespans;
        std::vector<double> robot_utils;
        
        for (const auto& run : group_runs) {
            times.push_back(run.metrics.total_computation_time_ms);
            pruning_ratios.push_back(run.metrics.getSubtreeEfficiency().pruning_ratio);
            explored_ratios.push_back(run.metrics.getSubtreeEfficiency().explored_product_ratio);
            makespans.push_back(run.metrics.getSolutionQuality().tree_makespan_seconds);
            robot_utils.push_back(run.metrics.getSolutionQuality().robot_utilization_ratio);
        }
        
        // Calculate means
        stat.mean_time_ms = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
        stat.mean_pruning_ratio = std::accumulate(pruning_ratios.begin(), pruning_ratios.end(), 0.0) / pruning_ratios.size();
        stat.mean_explored_ratio = std::accumulate(explored_ratios.begin(), explored_ratios.end(), 0.0) / explored_ratios.size();
        stat.mean_makespan_seconds = std::accumulate(makespans.begin(), makespans.end(), 0.0) / makespans.size();
        stat.mean_robot_util = std::accumulate(robot_utils.begin(), robot_utils.end(), 0.0) / robot_utils.size();
        
        // Calculate standard deviations
        double time_sum_sq = 0.0;
        double pruning_sum_sq = 0.0;
        double explored_sum_sq = 0.0;
        double makespan_sum_sq = 0.0;
        
        for (size_t i = 0; i < times.size(); ++i) {
            time_sum_sq += std::pow(times[i] - stat.mean_time_ms, 2);
            pruning_sum_sq += std::pow(pruning_ratios[i] - stat.mean_pruning_ratio, 2);
            explored_sum_sq += std::pow(explored_ratios[i] - stat.mean_explored_ratio, 2);
            makespan_sum_sq += std::pow(makespans[i] - stat.mean_makespan_seconds, 2);
        }
        
        stat.stddev_time_ms = std::sqrt(time_sum_sq / times.size());
        stat.stddev_pruning_ratio = std::sqrt(pruning_sum_sq / pruning_ratios.size());
        stat.stddev_explored_ratio = std::sqrt(explored_sum_sq / explored_ratios.size());
        stat.stddev_makespan_seconds = std::sqrt(makespan_sum_sq / makespans.size());
        
        // Min/Max times
        stat.min_time_ms = *std::min_element(times.begin(), times.end());
        stat.max_time_ms = *std::max_element(times.begin(), times.end());
        
        stats.push_back(stat);
    }
    
    return stats;
}

// ==================== EXPORT FOR ANALYSIS ====================

void TestRunManager::exportCategoryToCSV(TestCategory category,
                                        const std::string& output_filename) const {
    auto runs = getRuns(category);
    if (runs.empty()) {
        std::cerr << "⚠ No runs to export for category: " << getCategoryName(category) << std::endl;
        return;
    }
    
    std::ofstream out(output_filename);
    
    // Write header: dynamic parameters + standard metrics
    out << "run_id,trial_number,timestamp";
    
    // Get all unique parameter keys
    std::set<std::string> all_keys;
    for (const auto& run : runs) {
        for (const auto& [key, value] : run.parameters) {
            all_keys.insert(key);
        }
    }
    
    for (const auto& key : all_keys) {
        out << "," << key;
    }
    
    // Metrics header
    out << ",num_automaton_states,num_automaton_edges,num_robots,num_ts_regions,"
        << "total_computation_time_ms,total_nodes_planning,total_nodes_traversed,"
        << "total_nodes_pruned,nodes_satisfying_ltl,task_allocation_memory_bytes,"
        << "product_automaton_memory_bytes,product_automaton_nodes,product_automaton_edges,"
        << "pruning_ratio,explored_product_ratio,tree_makespan_seconds,robots_utilized,"
        << "robot_utilization_ratio,load_balance_variance\n";
    
    // Write data
    for (const auto& run : runs) {
        out << run.run_id << "," << run.trial_number << ",\"" << run.timestamp << "\"";
        
        for (const auto& key : all_keys) {
            out << ",";
            auto it = run.parameters.find(key);
            if (it != run.parameters.end()) {
                out << it->second;
            }
        }
        
        const auto& metrics = run.metrics;
        const auto& efficiency = metrics.getSubtreeEfficiency();
        const auto& quality = metrics.getSolutionQuality();
        const auto& iv = metrics.getIndependentVariables();
        
        out << "," << iv.num_automaton_states
            << "," << iv.num_automaton_edges
            << "," << iv.num_robots
            << "," << iv.num_ts_regions
            << "," << metrics.total_computation_time_ms
            << "," << efficiency.total_nodes_planning
            << "," << efficiency.total_nodes_traversed
            << "," << efficiency.total_nodes_pruned
            << "," << efficiency.nodes_satisfying_ltl
            << "," << efficiency.task_allocation_algorithm_memory_bytes
            << "," << efficiency.full_product_automaton_memory_bytes
            << "," << efficiency.full_product_automaton_nodes
            << "," << efficiency.full_product_automaton_edges
            << "," << efficiency.pruning_ratio
            << "," << efficiency.explored_product_ratio
            << "," << quality.tree_makespan_seconds
            << "," << quality.robots_utilized
            << "," << quality.robot_utilization_ratio
            << "," << quality.load_balance_variance
            << "\n";
    }
    
    out.close();
    std::cout << "✓ Exported " << runs.size() << " runs to: " << output_filename << std::endl;
}

void TestRunManager::exportAllToCSV() const {
    std::string export_dir = data_dir_ + "/exports";
    fs::create_directories(export_dir);
    
    for (const auto& [category, data] : category_data_) {
        std::string filename = export_dir + "/" + getCategoryName(category) + ".csv";
        exportCategoryToCSV(category, filename);
    }
    
    std::cout << "✓ Exported all test categories to: " << export_dir << std::endl;
}

void TestRunManager::exportStatisticsToCSV(TestCategory category,
                                          const std::string& output_filename) const {
    auto stats = getStatistics(category);
    if (stats.empty()) {
        std::cerr << "⚠ No statistics to export for category: " << getCategoryName(category) << std::endl;
        return;
    }
    
    std::ofstream out(output_filename);
    
    // Header
    out << "config_id";
    
    // Get all unique parameter keys
    std::set<std::string> all_keys;
    for (const auto& stat : stats) {
        for (const auto& [key, value] : stat.parameter_config) {
            all_keys.insert(key);
        }
    }
    
    for (const auto& key : all_keys) {
        out << "," << key;
    }
    
    out << ",num_runs,mean_time_ms,stddev_time_ms,min_time_ms,max_time_ms,"
        << "mean_pruning_ratio,stddev_pruning_ratio,mean_explored_ratio,"
        << "stddev_explored_ratio,mean_makespan_seconds,stddev_makespan_seconds,"
        << "mean_robot_utilization\n";
    
    // Write data
    int config_id = 0;
    for (const auto& stat : stats) {
        out << config_id++;
        
        for (const auto& key : all_keys) {
            out << ",";
            auto it = stat.parameter_config.find(key);
            if (it != stat.parameter_config.end()) {
                out << it->second;
            }
        }
        
        out << "," << stat.num_runs
            << "," << stat.mean_time_ms
            << "," << stat.stddev_time_ms
            << "," << stat.min_time_ms
            << "," << stat.max_time_ms
            << "," << stat.mean_pruning_ratio
            << "," << stat.stddev_pruning_ratio
            << "," << stat.mean_explored_ratio
            << "," << stat.stddev_explored_ratio
            << "," << stat.mean_makespan_seconds
            << "," << stat.stddev_makespan_seconds
            << "," << stat.mean_robot_util
            << "\n";
    }
    
    out.close();
    std::cout << "✓ Exported statistics to: " << output_filename << std::endl;
}

void TestRunManager::exportSummaryReport(const std::string& output_filename) const {
    std::ofstream out(output_filename);
    
    out << "# Test Run Summary Report\n\n";
    
    for (const auto& [category, data] : category_data_) {
        out << "## " << data.category_name << "\n";
        out << data.description << "\n\n";
        out << "**Status**: " << getCurrentNumberOfRuns(category) << " / " 
            << getExpectedNumberOfRuns(category) << " runs ("
            << std::fixed << std::setprecision(1) 
            << getCompletionPercentage(category) << "%)\n\n";
        
        if (!data.runs.empty()) {
            out << "| Metric | Value |\n";
            out << "|--------|-------|\n";
            
            double total_time = 0.0;
            for (const auto& run : data.runs) {
                total_time += run.metrics.total_computation_time_ms;
            }
            
            out << "| Average Computation Time | " 
                << (total_time / data.runs.size()) << " ms |\n";
            out << "| Last Run Timestamp | " << data.runs.back().timestamp << " |\n";
        }
        
        out << "\n";
    }
    
    out.close();
    std::cout << "✓ Exported summary report to: " << output_filename << std::endl;
}

// ==================== ANALYSIS HELPERS ====================

int TestRunManager::getExpectedNumberOfRuns(TestCategory category) const {
    switch (category) {
        case TestCategory::AUTOMATON_STATES:   return 15;  // 5-150 states
        case TestCategory::NUM_ROBOTS:         return 8;   // 3-20 robots
        case TestCategory::TS_REGIONS:         return 10;  // 5-40 regions
        case TestCategory::AVG_CAPABILITIES:   return 10;  // 1-5 avg cap
        case TestCategory::ROBOT_HOMOGENEITY:  return 10;  // 0.2-3 homogeneity
        default: return 0;
    }
}

int TestRunManager::getCurrentNumberOfRuns(TestCategory category) const {
    auto it = category_data_.find(category);
    if (it != category_data_.end()) {
        return it->second.runs.size();
    }
    return 0;
}

double TestRunManager::getCompletionPercentage(TestCategory category) const {
    int expected = getExpectedNumberOfRuns(category);
    if (expected == 0) return 0.0;
    return (getCurrentNumberOfRuns(category) / static_cast<double>(expected)) * 100.0;
}

void TestRunManager::printTestProgress() const {
    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "TEST PROGRESS SUMMARY" << std::endl;
    std::cout << std::string(70, '=') << std::endl;
    
    for (const auto& [category, data] : category_data_) {
        int current = getCurrentNumberOfRuns(category);
        int expected = getExpectedNumberOfRuns(category);
        double completion = getCompletionPercentage(category);
        
        std::cout << "\n" << std::left << std::setw(30) << data.category_name
                  << " [" << std::setfill('=') << std::setw((int)(completion / 5)) 
                  << "" << std::setfill(' ') << std::setw(20 - (int)(completion / 5))
                  << "] " << current << "/" << expected 
                  << " (" << std::fixed << std::setprecision(1) << completion << "%)"
                  << std::endl;
    }
    
    std::cout << "\n" << std::string(70, '=') << std::endl << std::endl;
}

std::string TestRunManager::getCategoryName(TestCategory category) {
    switch (category) {
        case TestCategory::AUTOMATON_STATES:   return "automaton_states";
        case TestCategory::NUM_ROBOTS:         return "num_robots";
        case TestCategory::TS_REGIONS:         return "ts_regions";
        case TestCategory::AVG_CAPABILITIES:   return "avg_capabilities";
        case TestCategory::ROBOT_HOMOGENEITY:  return "robot_homogeneity";
        default: return "unknown";
    }
}

// ==================== INTERNAL HELPERS ====================

std::string TestRunManager::getCategoryDirPath(TestCategory category) const {
    return data_dir_ + "/" + getCategoryName(category);
}

std::string TestRunManager::getRunFilename(TestCategory category, int run_id) const {
    std::ostringstream oss;
    oss << getCategoryDirPath(category) << "/run_" << std::setfill('0') 
        << std::setw(4) << run_id << ".txt";
    return oss.str();
}

void TestRunManager::createCategoryDirectory(TestCategory category) {
    std::string dir = getCategoryDirPath(category);
    if (!fs::exists(dir)) {
        fs::create_directories(dir);
        std::cout << "✓ Created category directory: " << getCategoryName(category) << std::endl;
    }
}

void TestRunManager::loadCategoryData(TestCategory category) {
    // TODO: Load runs from JSON files in category directory
    std::cout << "Loading category: " << getCategoryName(category) << std::endl;
}

void TestRunManager::saveCategoryData(TestCategory category) const {
    (void)category;  // Unused parameter
    // TODO: Save all runs in category to individual text files
}

std::string TestRunManager::serializeTestRun(const TestRun& run) const {
    (void)run;  // Unused parameter
    // TODO: Serialize TestRun to text format
    return "";
}

TestRunManager::TestRun TestRunManager::deserializeTestRun(const std::string& json_str) const {
    (void)json_str;  // Unused parameter
    // TODO: Deserialize text to TestRun
    return TestRun{};
}
