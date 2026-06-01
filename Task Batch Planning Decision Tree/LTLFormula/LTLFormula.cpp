#include "LTLFormula.h"
#include <spot/tl/parse.hh>
#include <sstream>

// Constructor
LTLFormula::LTLFormula(std::string formulaStr, const std::vector<BatchAtomicProposition>& batchAPsVec) 
    : batchAPs(nullptr) {
    try {
        spot::parsed_formula pf = spot::parse_infix_psl(formulaStr);
        formula = pf.f;
        batchAPs = new std::vector<BatchAtomicProposition>(batchAPsVec);
    } catch (const std::exception& e) {
        delete batchAPs;
        batchAPs = nullptr;
        throw std::runtime_error("Failed to parse LTL formula: " + std::string(e.what()));
    }
}

// Destructor
LTLFormula::~LTLFormula() {
    delete batchAPs;
    batchAPs = nullptr;
}

// Parse the LTL formula
void LTLFormula::parse(const std::string& formulaStr) {
    try {
        spot::parsed_formula pf = spot::parse_infix_psl(formulaStr);
        formula = pf.f;
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to parse LTL formula: " + std::string(e.what()));
    }
}

// Convert to string representation
std::string LTLFormula::toString() const {
    std::ostringstream oss;
    oss << formula;
    return oss.str();
}

// Evaluate the formula
bool LTLFormula::evaluate() {
    // TODO: Implement evaluation logic
    return false;
}

// Setter for formula
void LTLFormula::setFormula(const std::string& formulaStr) {
    try {
        spot::parsed_formula pf = spot::parse_infix_psl(formulaStr);
        formula = pf.f;
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to parse LTL formula: " + std::string(e.what()));
    }
}

// Getter for formula string
std::string LTLFormula::getFormula() const {
    std::ostringstream oss;
    oss << formula;
    return oss.str();
}

// Get the Spot formula object
spot::formula LTLFormula::getSpotFormula() const {
    return formula;
}

// Get all batch atomic propositions
const std::vector<BatchAtomicProposition>& LTLFormula::getBatchAPs() const {
    if (!batchAPs) throw std::runtime_error("BatchAPs vector is null");
    return *batchAPs;
}

// Get a specific batch atomic proposition by ID
BatchAtomicProposition LTLFormula::getAP(uint16_t id) const {
    if (!batchAPs) throw std::runtime_error("BatchAPs vector is null");
    for (const auto& batchAP : *batchAPs) {
        if (batchAP.getAP() == id) {
            return batchAP;
        }
    }
    throw std::out_of_range("Atomic proposition ID not found");
}

// Add a batch atomic proposition
void LTLFormula::addBatchAtomicProposition(const BatchAtomicProposition& ap) {
    if (!batchAPs) batchAPs = new std::vector<BatchAtomicProposition>();
    batchAPs->push_back(ap);
}

// Get batch atomic propositions vector
std::vector<BatchAtomicProposition>& LTLFormula::getBatchAtomicPropositions() {
    if (!batchAPs) batchAPs = new std::vector<BatchAtomicProposition>();
    return *batchAPs;
}

// Get atomic propositions (alias for getBatchAtomicPropositions)
std::vector<BatchAtomicProposition>& LTLFormula::getAtomicPropositions() {
    if (!batchAPs) batchAPs = new std::vector<BatchAtomicProposition>();
    return *batchAPs;
}

// Get the batch value for a specific AP ID
int8_t LTLFormula::getBatchVal(uint16_t apId) const {
    if (!batchAPs) throw std::runtime_error("BatchAPs vector is null");
    for (const auto& batchAP : *batchAPs) {
        if (batchAP.getAP() == apId) {
            return batchAP.getBatch();
        }
    }
    throw std::out_of_range("Atomic proposition ID not found");
}

// Check if formula is valid
bool LTLFormula::isValid() const {
    // TODO: Implement validation logic
    return true;
}

// Build the formula tree
void LTLFormula::buildTree() {
    // TODO: Implement tree building logic
}
