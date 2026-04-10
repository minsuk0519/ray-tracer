CXX      = g++
CXXFLAGS = -std=c++2b -I../src/bvh -I../libs/glm -g

SRCS_COMMON = ../src/bvh/AABB.cpp

TESTS = test_aabb

# ── Platform detection ────────────────────────────────────────────────
ifeq ($(OS),Windows_NT)
    BINS = $(addsuffix .exe, $(TESTS))
else
    BINS = $(TESTS)
endif

# ── Targets ───────────────────────────────────────────────────────────
all: $(BINS)

ifeq ($(OS),Windows_NT)
%.exe: %.cpp $(SRCS_COMMON)
	$(CXX) $(CXXFLAGS) -o $@ $^
else
%: %.cpp $(SRCS_COMMON)
	$(CXX) $(CXXFLAGS) -o $@ $^
endif

run: all
	@for t in $(BINS); do \
		echo "--- $$t ---"; \
		./$$t; \
	done

clean:
	rm -f $(BINS)

.PHONY: all run clean
