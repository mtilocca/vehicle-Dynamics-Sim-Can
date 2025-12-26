// src/plant/plant_state_visitor.hpp
#pragma once

#include "plant/plant_state.hpp"
#include <functional>
#include <string>

namespace plant {

/**
 * FieldVisitor - Infrastructure for automatic field enumeration
 * 
 * This enables PlantState to expose its fields to external systems
 * (CAN encoding, CSV logging, debugging, serialization) without
 * manual field-by-field mapping.
 * 
 * Usage:
 *   auto visitor = [](const char* name, double value) {
 *       std::cout << name << " = " << value << "\n";
 *   };
 *   state.accept_fields(visitor);
 */

/**
 * Type-erasing visitor that converts all numeric types to double
 * This allows the visitor to accept int, float, double, etc.
 */
class FieldVisitor {
public:
    using Callback = std::function<void(const char*, double)>;
    
    explicit FieldVisitor(Callback cb) : callback_(cb) {}
    
    // Visit methods for different types - all convert to double
    void visit(const char* name, double value) {
        callback_(name, value);
    }
    
    void visit(const char* name, float value) {
        callback_(name, static_cast<double>(value));
    }
    
    void visit(const char* name, int value) {
        callback_(name, static_cast<double>(value));
    }
    
    void visit(const char* name, uint32_t value) {
        callback_(name, static_cast<double>(value));
    }
    
    void visit(const char* name, bool value) {
        callback_(name, value ? 1.0 : 0.0);
    }
    
private:
    Callback callback_;
};

/**
 * Lambda-based visitor - allows direct lambda usage
 */
template<typename Lambda>
class LambdaVisitor {
public:
    explicit LambdaVisitor(Lambda&& lambda) : lambda_(std::forward<Lambda>(lambda)) {}
    
    template<typename T>
    void visit(const char* name, T value) {
        lambda_(name, static_cast<double>(value));
    }
    
private:
    Lambda lambda_;
};

// Helper to create lambda visitor
template<typename Lambda>
LambdaVisitor<Lambda> make_visitor(Lambda&& lambda) {
    return LambdaVisitor<Lambda>(std::forward<Lambda>(lambda));
}

} // namespace plant
