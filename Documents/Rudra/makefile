CXX = g++
CXXFLAGS = -std=c++17 -Og -g -Wall

# default target
all: main

# build main executable
main: src/*.cpp
	$(CXX) $(CXXFLAGS) src/*.cpp -Ilib -lm -o main

check: main
	@echo "--------------------------------------------"
	@echo "Checking..."
	@echo "Test-1: "
	./main test/testcase1.txt test/result1.txt
	diff test/result1.txt test/expected1.txt
	@echo "Test-2: "
	./main test/testcase2.txt test/result2.txt
	diff test/result2.txt test/expected2.txt
	@echo "Test-3: "
	./main test/testcase3.txt test/result3.txt
	diff test/result3.txt test/expected3.txt
	@echo "**** Success: ***"
	@echo "--------------------------------------------"

clean:
	rm -f main test/result*

.PHONY: all clean check

