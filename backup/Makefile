
#----------------------------------
# Dependencies:
# - GLFW (http://www.glfw.org)
#----------------------------------

# Recursively expanded varaibles: evaluation of the variable
# is deferred until it is used. Benefit: assignments can be performed 
# our of order.
CXX = g++
CC = gcc
EXE = dyno_gui
IMPLOT_DIR = lib/implot
IMGUI_DIR = lib/imgui
LCM_DIR = lib/lcm
MCC_DIR = lib/mcc
TEST_DIR = test
SOURCES = src/main.cpp
SOURCES += $(IMGUI_DIR)/imgui.cpp $(IMGUI_DIR)/imgui_demo.cpp $(IMGUI_DIR)/imgui_draw.cpp $(IMGUI_DIR)/imgui_tables.cpp $(IMGUI_DIR)/imgui_widgets.cpp
SOURCES += $(IMGUI_DIR)/backends/imgui_impl_glfw.cpp $(IMGUI_DIR)/backends/imgui_impl_opengl3.cpp
SOURCES += $(IMPLOT_DIR)/implot.cpp $(IMPLOT_DIR)/implot_demo.cpp $(IMPLOT_DIR)/implot_items.cpp 
#SOURCES += $(LCM_DIR)/mypackage_Position.c
# === Built-in functions ===
# $(addsuffix suffix,name...): appends suffix with given name
# $(basename name...): returns filename w/o suffix
# $(notdir name...): returns the filename
# $(shell command) - misc function
OBJS = $(addsuffix .o, $(basename $(notdir $(SOURCES))))
UNAME_S := $(shell uname -s)
LINUX_GL_LIBS = -lGL

CXXFLAGS = -std=c++11 -I$(IMGUI_DIR) -I$(IMGUI_DIR)/backends -I$(IMPLOT_DIR) -I$(LCM_DIR)/mypackage -I$(LCM_DIR)/FOC
#CXXFLAGS += -g -Wall -Wformat
C_FLAGS = -O0 -g -fPIC -O -I$(LCM_DIR) -I$(MCC_DIR) -I/usr/local/include/libusb-1.0 -L/usr/local/lib -lusb-1.0 `pkg-config --cflags lcm`
C_LIBS = `pkg-config --libs lcm` -L. -lpthread -lrt -lm -lmccusb -lusb-1.0 -L/usr/local/lib -lhidapi-libusb 
C_OBJS = pmd.o nist.o usb-1608FS.o itimer.o
LIBS = -pthread

#----------------------------------
# BUILD FLAGS FOR LINUX PLATFORM
#----------------------------------

ifeq ($(UNAME_S), Linux) #LINUX
	ECHO_MESSAGE = "Linux"
	LIBS += $(LINUX_GL_LIBS) `pkg-config --static --libs glfw3 lcm`

	CXXFLAGS +=`pkg-config --cflags glfw3 lcm`
	CFLAGS = $(CXXFLAGS)
endif

ifeq ($(shell uname), Darwin)
	SONAME_FLAGS = -install_name
	SHARED_EXT = dylib
else
	SONAME_FLAGS = -soname
	SHARED_EXT = so
endif 

# test-usb1608FS
TARGETS=libmccusb.$(SHARED_EXT) libmccusb.a itmer
ID=MCCLIBUSB
DIST_NAME=$(ID).$(VERSION).tgz
# test-usb1608FS.c
DIST_FILES={Makefile,nist.c,pmd.c,pmd.h,usb-1608FS.h,usb-1608FS.c,itimer.c}

#----------------------------------
# BUILD RULES
# target: prerequisite1 prerequisite2 ...
# 	rule
# === Automatic variables: ===
# $@: target
# $<: prerequisite1 (i.e. only first prerequisite)
# $^: prerequisite1 prerequisite2 (i.e. all prerequisites)
# === gcc commands ===
# -c: compile but don't assemble (compile-only mode). Creates object file
# -o output_filename: will compile 
# *Note: we use -c -o together on a single rule when we need to compile
# many source files separately.
#----------------------------------

#	$(CC) -O -shared -Wall $(OBJS) -o $@

#all: $(EXE) itimer
all: $(EXE) $(TRAGETS)
	@echo Build complete for $(ECHO_MESSAGE)


%.d: %.c
	set -e; $(CC) -I. -M $(CPPFLAGS) $< \
	| sed 's/\($*\)\.o[ :]*/\1.o $@ : /g' > $@; \
	[ -s $@ ] || rm -f $@
ifneq ($(MAKECMDGOALS),clean)
include $(SRCS:.c=.d)
endif

# -shared: creates a shared library
libmccusb.$(SHARED_EXT): $(C_OBJS)
#	$(CC) -O -shared -Wall $(OBJS) -o $@
	$(CC) -shared -Wl,$(SONAME_FLAGS),$@ -o $@ $(C_OBJS) -lc -lm

libmccusb.a: $(C_OBJS)
	ar -r libmccusb.a $(C_OBJS)
	ranlib libmccusb.a

#%.o:$(SOURCES)/%.cpp $(LCM_DIR)/mypackage/%.hpp $(LCM_DIR)/%.h
%.o:$(SOURCES)/%.cpp $(LCM_DIR)/mypackage/%.hpp $(LCM_DIR)/FOC/%.hpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.o:$(IMGUI_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.o:$(IMGUI_DIR)/backends/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.o:$(IMPLOT_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

#itimer.o: src/itimer.c
#	$(CC) $(C_FLAGS) -c -o $@ $<

itimer.o: src/itimer.c
	$(CC) $(C_FLAGS) -c $@ $<

%.o:$(MCC_DIR)/%.c $(MCC_DIR)/%.h
	$(CC) -fPIC $(C_FLAGS) -c $<

itimer:	itimer.o $(LCM_DIR)/mypackage_Position.o $(LCM_DIR)/FOC_motor_t.o usb-1608FS.o libmccusb.a
	$(CC) $(C_FLAGS) -o $@ $^ $(C_LIBS)

$(EXE): $(OBJS)
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LIBS)


vpath %.h $(IMPLOT_DIR) $(IMPLOT_DIR)/backends $(IMGUI_DIR) $(LCM_DIR) $(MCC_DIR)
vpath %.hpp $(LCM_DIR)/mypackage
vpath %.cpp $(IMPLOT_DIR) $(IMPLOT_DIR)/backends $(IMGUI_DIR) $(LCM_DIR)
vpath %.c $(LCM_DIR) $(MCC_DIR)
vpath %.o $(LCM_DIR) $(MCC_DIR)
vpath %.cpp src
vpath %.c src

clean:
	rm -f $(EXE) $(OBJS) itimer itimer.o