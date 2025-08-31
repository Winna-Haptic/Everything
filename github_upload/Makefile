# Makefile for Haptic Glove MVP - Location-Specific Biomechanical Feedback
# Standard C++ compilation for Visual Studio Code

CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
INCLUDES = -Isrc
TARGET = haptic_simulation
SRCDIR = src
SOURCES = $(wildcard $(SRCDIR)/*.cpp)
OBJECTS = $(SOURCES:.cpp=.o)

# Default target
all: $(TARGET)

# Build simulation (matches README instructions)
simulation: $(TARGET)
	@echo "🎯 Haptic simulation ready!"
	@echo "Run: ./$(TARGET)"

# Build the main executable
$(TARGET): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $(TARGET)
	@echo "Build complete: $(TARGET)"

# Compile source files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Clean build files
clean:
	rm -f $(OBJECTS) $(TARGET)
	@echo "Cleaned build files"

# Run the program
run: $(TARGET)
	./$(TARGET)

# Debug build
debug: CXXFLAGS += -g -DDEBUG
debug: $(TARGET)

# Install dependencies (placeholder)
install-deps:
	@echo "No external dependencies required for C++ version"
	@echo "For ESP32 hardware integration, install ESP-IDF separately"

# Format code (requires clang-format)
format:
	find $(SRCDIR) -name "*.cpp" -o -name "*.h" | xargs clang-format -i

# Static analysis (requires cppcheck)
analyze:
	cppcheck --enable=all --std=c++17 $(SRCDIR)/

# Create documentation (requires doxygen)
docs:
	doxygen Doxyfile

# Help
help:
	@echo "Available targets:"
	@echo "  simulation - Build haptic simulation (recommended)"
	@echo "  all        - Build the project (default)"
	@echo "  clean      - Remove build files"
	@echo "  run        - Build and run the program"
	@echo "  debug      - Build with debug symbols"
	@echo "  format     - Format source code"
	@echo "  analyze    - Run static analysis"
	@echo "  docs       - Generate documentation"
	@echo "  help       - Show this help message"

.PHONY: all simulation clean run debug install-deps format analyze docs help
