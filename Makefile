# Define the compiler and flags
CXX = g++
CXXFLAGS = -Wall -g

# The target executable name
TARGET = dykstra_ecmp

# Default rule to build the program
all: main.cc
	$(CXX) $(CXXFLAGS) -o $(TARGET) main.cc

# Rule to remove the executable
clean:
	rm -f $(TARGET)
