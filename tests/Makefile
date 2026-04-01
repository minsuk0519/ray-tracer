CXX      = g++
CXXFLAGS = -std=c++2b -I.. -I../libs/glm -g

SRCS_COMMON = ../AABB.cpp

TESTS = test_aabb

all: $(TESTS)

test_aabb: test_aabb.cpp $(SRCS_COMMON)
	$(CXX) $(CXXFLAGS) -o $@ $^

run: all
	@for t in $(TESTS); do \
		echo "--- $$t ---"; \
		./$$t; \
	done

clean:
	rm -f $(TESTS)
