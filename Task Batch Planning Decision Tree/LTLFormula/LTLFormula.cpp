#include "LTLFormula.h"
#include <spot/tl/parse.hh>
#include <sstream>

// Constructor
LTLFormula::LTLFormula(std::string formulaStr, const std::vector<BatchAtomicProposition>& batchAPs) 
    : batchAPs(batchAPs) {
    try {
        spot::parsed_formula pf = spot::parse_infix_psl(formulaStr);
        this->formula = pf.f;
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to parse LTL formula: " + std::string(e.what()));
    }
}

// Destructor
LTLFormula::~LTLFormula() {
}

// Parse the LTL formula
void LTLFormula::parse(const std::string& formulaStr) {
    try {
        spot::parsed_formula pf = spot::parse_infix_psl(formulaStr);
        this->formula = pf.f;
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
        this->formula = pf.f;
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
    return batchAPs;
}

// Get a specific batch atomic proposition by ID
BatchAtomicProposition LTLFormula::getAP(uint16_t id) const {
    if (id < batchAPs.size()) {
        return batchAPs[id];
    }
    throw std::out_of_range("Atomic proposition ID out of range");
}

// Add a batch atomic proposition
void LTLFormula::addBatchAtomicProposition(const BatchAtomicProposition& ap) {
    batchAPs.push_back(ap);
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
