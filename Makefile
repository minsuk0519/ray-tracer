
########################################################################
# Makefile — cross-platform (Linux / Windows MinGW-MSYS2)
########################################################################

CXX = g++

LIBDIR    = libs
ASSIMPDIR = $(LIBDIR)/assimp

OPTIMIZE  = -g -O2

CXXFLAGS = $(OPTIMIZE) -std=c++2b \
           -I. -Isrc -Isrc/bvh \
           -I$(LIBDIR)/glm \
           -I$(LIBDIR)/stb \
           -I$(ASSIMPDIR)/include \
           -DGLM_ENABLE_EXPERIMENTAL \
           -Wnarrowing

# Use locally built assimp if libs/assimp/lib exists, otherwise fall back
# to system-installed assimp (Linux: apt/pacman; Windows MSYS2: pacman -S mingw-w64-x86_64-assimp)
ifneq (,$(wildcard $(ASSIMPDIR)/lib))
    LIBS = -L$(ASSIMPDIR)/lib -lassimp
else
    LIBS = -lassimp
endif

SRCS = src/main.cpp \
       src/bvh/bvh_bake.cpp \
       src/bvh/bvh_bfs.cpp \
       src/bvh/bvh_IO.cpp \
       src/bvh/bvh_morton.cpp \
       src/bvh/bvh_sah.cpp \
       src/bvh/bvh_geo.cpp \
       src/bvh/AABB.cpp \
       src/bvh/bvh_traverse.cpp

OBJS = $(patsubst %.cpp,%.o,$(SRCS))

# ── Platform detection ────────────────────────────────────────────────
ifeq ($(OS),Windows_NT)
    TARGET = raytrace.exe
    RUN    = ./$(TARGET)
else
    TARGET = raytrace
    RUN    = LD_LIBRARY_PATH="$(ASSIMPDIR)/lib:$(LD_LIBRARY_PATH)" ./$(TARGET)
endif

# ── Targets ───────────────────────────────────────────────────────────
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

run: $(TARGET)
	$(RUN)

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: run clean
