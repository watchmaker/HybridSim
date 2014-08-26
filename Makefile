# HybridSim build system

###################################################

CXXFLAGS=-m64 -DNO_STORAGE -Wall -DDEBUG_BUILD -std=c++0x
OPTFLAGS=-m64 -O3


ifdef DEBUG
ifeq ($(DEBUG), 1)
OPTFLAGS= -O0 -g
endif
endif
CXXFLAGS+=$(OPTFLAGS)

CUR_DIRECTORY=$(shell pwd)
#DRAM_LIB=$(CUR_DIRECTORY)/../DRAMSim2
#NV_LIB=$(CUR_DIRECTORY)/../NVDIMMSim/src
#NV_LIB=$(CUR_DIRECTORY)/../FNVSim

#INCLUDES=-I$(DRAM_LIB) -I$(NV_LIB)
#LIBS=-L${DRAM_LIB} -L${NV_LIB} -ldramsim -lnvdsim -Wl,-rpath ${DRAM_LIB} -Wl,-rpath ${NV_LIB}

EXE_NAME=HybridSim
LIB_NAME=libhybridsim.so
LIB_NAME_MACOS=libhybridsim.dylib

SRC = $(wildcard *.cpp)
OBJ = $(addsuffix .o, $(basename $(SRC)))
POBJ = $(addsuffix .po, $(basename $(SRC)))
REBUILDABLES=$(OBJ) ${POBJ} $(EXE_NAME) $(LIB_NAME)

all: ${EXE_NAME} 

lib: ${LIB_NAME} 

#   $@ target name, $^ target deps, $< matched pattern
$(EXE_NAME): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ ${LIBS}
	@echo "Built $@ successfully" 

${LIB_NAME}: ${POBJ}
	$(CXX) -g -shared -Wl,-soname,$@ -o $@ $^ ${LIBS}
	@echo "Built $@ successfully"

${LIB_NAME_MACOS}: ${POBJ}
	$(CXX) -g -dynamiclib -o $@ $^ ${LIBS}
	@echo "Built $@ successfully"

#include the autogenerated dependency files for each .o file
-include $(OBJ:.o=.dep)
-include $(POBJ:.po=.deppo)

# build dependency list via gcc -M and save to a .dep file
%.dep : %.cpp
	@$(CXX) $(INCLUDES) -std=c++0x -M $(CXXFLAGS) $< > $@
%.deppo : %.cpp
	@$(CXX) $(INCLUDES) -std=c++0x -M $(CXXFLAGS) -MT"$*.po" $< > $@

# build all .cpp files to .o files
%.o : %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

%.po : %.cpp
	$(CXX) $(INCLUDES) -std=c++0x -O3 -g -ffast-math -fPIC -DNO_OUTPUT -DNO_STORAGE -o $@ -c $<

clean: 
	rm -rf ${REBUILDABLES} *.dep *.deppo out results *.log callgrind*
