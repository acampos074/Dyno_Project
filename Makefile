#--------------------------------------------------
# FOC_GUI_V4 Makefile
# Builds two targets:
#   dyno_gui   — ImGui/ImPlot GUI process
#   seqgen     — Real-time sequencer process
#
# All object files go into build/
#--------------------------------------------------

CXX        = g++
CC         = gcc
BUILD_DIR  = build

IMGUI_DIR  = lib/imgui
IMPLOT_DIR = lib/implot
LCM_DIR    = lib/lcm
MCC_DIR    = lib/mcc
INC_DIR    = include

#--------------------------------------------------
# GUI target (dyno_gui)
#--------------------------------------------------
GUI_EXE    = dyno_gui
GUI_SRC    = src/gui/main.cpp \
             $(IMGUI_DIR)/imgui.cpp \
             $(IMGUI_DIR)/imgui_demo.cpp \
             $(IMGUI_DIR)/imgui_draw.cpp \
             $(IMGUI_DIR)/imgui_tables.cpp \
             $(IMGUI_DIR)/imgui_widgets.cpp \
             $(IMGUI_DIR)/backends/imgui_impl_glfw.cpp \
             $(IMGUI_DIR)/backends/imgui_impl_opengl3.cpp \
             $(IMPLOT_DIR)/implot.cpp \
             $(IMPLOT_DIR)/implot_demo.cpp \
             $(IMPLOT_DIR)/implot_items.cpp

GUI_OBJS   = $(addprefix $(BUILD_DIR)/, $(addsuffix .o, $(basename $(notdir $(GUI_SRC)))))

GUI_CXXFLAGS = -std=c++11 \
               -I$(IMGUI_DIR) \
               -I$(IMGUI_DIR)/backends \
               -I$(IMPLOT_DIR) \
               -I$(LCM_DIR)/FOC \
               -I$(INC_DIR) \
               `pkg-config --cflags glfw3 lcm`

GUI_LIBS   = -pthread -lGL \
             `pkg-config --static --libs glfw3 lcm`

#--------------------------------------------------
# Sequencer target (seqgen)
#--------------------------------------------------
SEQ_EXE    = seqgen
SEQ_SRC    = src/sequencer/seqgen_main.c \
             src/sequencer/sequencer.c \
             src/sequencer/lcm_interface.c \
             src/sequencer/can_driver.c \
             src/sequencer/mcc_driver.c \
             src/sequencer/logger.c \
             src/sequencer/encoding.c \
             src/sequencer/motor_sim.c \
             $(LCM_DIR)/FOC_motor_t.c \
             $(MCC_DIR)/usb-1608FS.c \
             $(MCC_DIR)/pmd.c \
             $(MCC_DIR)/nist.c

SEQ_OBJS   = $(addprefix $(BUILD_DIR)/, $(addsuffix .o, $(basename $(notdir $(SEQ_SRC)))))

SEQ_CFLAGS = -O0 -g -fPIC \
             -I$(LCM_DIR) \
             -I$(MCC_DIR) \
             -I$(INC_DIR) \
             -I/usr/local/include/libusb-1.0 \
             -L/usr/local/lib \
             `pkg-config --cflags lcm`

SEQ_LIBS   = `pkg-config --libs lcm` \
             -lpthread -lrt -lm \
             -lusb-1.0 -L/usr/local/lib \
             -lhidapi-libusb

#--------------------------------------------------
# Build rules
#--------------------------------------------------

all: $(BUILD_DIR) $(GUI_EXE) $(SEQ_EXE)
	@echo "Build complete."

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# GUI objects
$(BUILD_DIR)/%.o: $(IMGUI_DIR)/%.cpp | $(BUILD_DIR)
	$(CXX) $(GUI_CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: $(IMGUI_DIR)/backends/%.cpp | $(BUILD_DIR)
	$(CXX) $(GUI_CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: $(IMPLOT_DIR)/%.cpp | $(BUILD_DIR)
	$(CXX) $(GUI_CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: src/gui/%.cpp | $(BUILD_DIR)
	$(CXX) $(GUI_CXXFLAGS) -c -o $@ $<

# Sequencer objects
$(BUILD_DIR)/%.o: src/sequencer/%.c | $(BUILD_DIR)
	$(CC) $(SEQ_CFLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: $(LCM_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(SEQ_CFLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: $(MCC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(SEQ_CFLAGS) -c -o $@ $<

# Link
$(GUI_EXE): $(GUI_OBJS)
	$(CXX) -o $@ $^ $(GUI_CXXFLAGS) $(GUI_LIBS)

$(SEQ_EXE): $(SEQ_OBJS)
	$(CC) $(SEQ_CFLAGS) -o $@ $^ $(SEQ_LIBS)

clean:
	rm -rf $(BUILD_DIR) $(GUI_EXE) $(SEQ_EXE)

.PHONY: all clean
