
########################################################################
# Makefile for Linux
########################################################################

CXX = g++

ifneq (,$(wildcard libs))
    LIBDIR := libs
else
    ifneq (,$(wildcard ../libs))
        LIBDIR := ../libs
    else
        ifneq (,$(wildcard ../../libs))
            LIBDIR := ../../libs
        else
            LIBDIR := ../../../libs
        endif
    endif
endif

OPTIMIZE = -g -O4

BROKENCXXFLAGS = $(OPTIMIZE) -std=c++11 -I$(LIBDIR)/glm -I$(LIBDIR) -I/usr/include -I$(LIBDIR)/assimp/include -Wnarrowing -I.  -fopenmp -msse3 

ASSIMPDIR = $(LIBDIR)/assimp
CXXFLAGS = $(OPTIMIZE) -std=c++2b -I. -I$(LIBDIR)/glm -I$(LIBDIR) -I$(LIBDIR)/stb -I/usr/include -Wnarrowing -I$(ASSIMPDIR)/include

LIBS = -L$(ASSIMPDIR)/lib -lassimp


target = raytrace.exe

headers = geom.h raytrace.h acceleration.h
src = geom.cpp main.cpp raytrace.cpp acceleration.cpp
extras = Makefile

scenes = testscene.scn letterX.ply letterY.ply bunny.ply dwarf.x

pkgFiles = $(src) $(headers) $(extras) $(scenes)
pkgName = raytrace

objects = $(patsubst %.cpp,%.o,$(src))


$(target): $(objects)
	g++  $(CXXFLAGS) -o $@  $(objects) $(LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

run: $(target)
	#python3 driver.py
	LD_LIBRARY_PATH="$(LIBDIR); $(LD_LIBRARY_PATH)" ./raytrace.exe testscene.scn

zip:
	rm -rf $(pkgDir)/$(pkgName) $(pkgDir)/$(pkgName).zip
	mkdir $(pkgDir)/$(pkgName)
	cp $(pkgFiles) $(pkgDir)/$(pkgName)
	cp -r ../libs $(pkgDir)/$(pkgName)
	rm -rf $(pkgDir)/$(pkgName)/libs/.hg 
	cd $(pkgDir);  zip -r $(pkgName).zip $(pkgName); rm -rf $(pkgName)


pyzip: $(pyFiles)
	rm -rf ../$(pkgName) ../$(pkgName)-py.zip
	mkdir ../$(pkgName)
	cp -r $(pyFiles) ../$(pkgName)
	rm -f `find ../$(pkgName) -name '*.pyc'`
	cd ..;  zip -r $(pkgName)-py.zip $(pkgName); rm -rf $(pkgName)

clean:
	rm -rf *.suo *.sdf *.orig Release Debug ipch *.o *~ raytrace dependencies *13*scn  *13*ppm 

dependencies: 
	g++ -MM $(CXXFLAGS)  $(src) > dependencies

-include dependencies
