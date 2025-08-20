#
# Winyunq Style Makefile for a C++ Project v1.3
#

# --- Compiler and Flags ---
# @brief The C++ compiler to be used.
CXX := g++
# @brief Compiler flags for compilation.
CXXFLAGS := -std=c++17 -Wall -Wextra -g
# @brief Include paths. We now add the system path for Eigen3.
# The compiler will now search in our project's src, and the system's Eigen3 directory.
INCLUDES := -I./src -I/usr/include/eigen3
# @brief Linker flags.
LDFLAGS :=

# --- Directories ---
# @brief The directory for the final executable.
BIN_DIR := bin
# @brief The directory containing all source files (.cpp).
SRC_ROOT_DIR := src
# @brief The directory for object files (.o).
OBJ_DIR := obj

# --- Files and Executable ---
# @brief The name of the final executable file.
TARGET := $(BIN_DIR)/UKF
# @brief Automatically find all .cpp source files in all subdirectories of SRC_ROOT_DIR.
SRCS := $(shell find $(SRC_ROOT_DIR) -name '*.cpp')
# @brief Generate object file names from source file names, placing them in OBJ_DIR.
OBJS := $(patsubst $(SRC_ROOT_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRCS))

# --- Recipes ---

# @brief The default target, which is 'all'.
.PHONY: all
all: $(TARGET)

# @brief The rule to link the final executable.
$(TARGET): $(OBJS)
	@mkdir -p $(BIN_DIR)
	@echo "Linking executable: $@"
	$(CXX) $(LDFLAGS) -o $@ $^

# @brief The rule to compile a single .cpp source file into an .o object file.
$(OBJ_DIR)/%.o: $(SRC_ROOT_DIR)/%.cpp
	@mkdir -p $(dir $@)
	@echo "Compiling: $<"
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# @brief The rule to clean up generated files.
.PHONY: clean
clean:
	@echo "Cleaning up generated files..."
	@rm -rf $(BIN_DIR) $(OBJ_DIR)
	@echo "Cleanup complete."