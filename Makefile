# Columbia — Apollo flight simulator for Haiku
# Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
#
#   make            build the Haiku application (on Haiku)
#   make test       build and run the headless tests (any POSIX host)

NAME     := Columbia
CXX      ?= g++
CXXFLAGS ?= -O2
CXXFLAGS += -std=c++17 -Wall -Wextra -Icore -Iapp -Iui -pthread

CORE_SRCS := \
	core/Mission.cpp \
	core/agc/AGC.cpp \
	core/agc/DSKY.cpp \
	core/agc/Comanche055Rope.cpp \
	core/sim/Flight.cpp \
	core/sim/Planner.cpp

APP_SRCS := $(wildcard app/*.cpp) $(wildcard ui/*.cpp)

OBJDIR := objects
CORE_OBJS := $(CORE_SRCS:%.cpp=$(OBJDIR)/%.o)
APP_OBJS  := $(APP_SRCS:%.cpp=$(OBJDIR)/%.o)
# The simulation the app runs on (defined before any rule that lists it:
# make expands a rule's prerequisites as it reads the rule).
GNC_SRCS := core/agc/AGC.cpp core/agc/DSKY.cpp $(wildcard core/gnc/*.cpp) \
	core/sim/World.cpp core/sim/Vehicle.cpp core/sim/Spacecraft.cpp \
	core/sim/Aero.cpp core/sim/MoonScene.cpp
FLIGHT_SRCS := $(GNC_SRCS) $(wildcard core/flight/*.cpp)
FLIGHT_OBJS := $(FLIGHT_SRCS:%.cpp=$(OBJDIR)/%.o)
APP_CORE_SRCS = $(FLIGHT_SRCS)
APP_CORE_OBJS = $(APP_CORE_SRCS:%.cpp=$(OBJDIR)/%.o)
LIBS      := -lbe -ltracker -ldevice

.PHONY: all test preview clean saver install-saver

all: $(NAME)

$(NAME): $(APP_CORE_OBJS) $(APP_OBJS) Columbia.rdef
	$(CXX) -o $@ $(APP_CORE_OBJS) $(APP_OBJS) $(LIBS)
	@if command -v rc >/dev/null 2>&1 && command -v xres >/dev/null 2>&1; then \
		rc -o $(OBJDIR)/Columbia.rsrc Columbia.rdef && \
		xres -o $@ $(OBJDIR)/Columbia.rsrc; \
	fi
	@if command -v mimeset >/dev/null 2>&1; then mimeset -f $@; fi

$(OBJDIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -MMD -MP -c -o $@ $<

-include $(shell find $(OBJDIR) -name '*.d' 2>/dev/null)

# --- Headless tests (no Haiku needed) ---------------------------------------

TESTDIR := $(OBJDIR)/tests

$(TESTDIR)/mission_test: tests/mission_test.cpp core/sim/Flight.cpp core/sim/Planner.cpp
	@mkdir -p $(TESTDIR)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(TESTDIR)/controller_test: tests/controller_test.cpp $(CORE_SRCS)
	@mkdir -p $(TESTDIR)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(TESTDIR)/agc_boot: tests/agc_boot.cpp core/agc/AGC.cpp core/agc/DSKY.cpp
	@mkdir -p $(TESTDIR)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(TESTDIR)/agc_selftest: tests/agc_selftest.cpp core/agc/AGC.cpp core/agc/DSKY.cpp
	@mkdir -p $(TESTDIR)
	$(CXX) $(CXXFLAGS) -o $@ $^

test: $(TESTDIR)/mission_test $(TESTDIR)/controller_test $(TESTDIR)/agc_boot $(TESTDIR)/agc_selftest
	$(TESTDIR)/mission_test
	$(TESTDIR)/controller_test
	$(TESTDIR)/agc_boot rope/Comanche055.bin "V36E  V05N09E  V16N65E   " 3
	@if [ -f "$(VALIDATION)" ]; then \
		$(TESTDIR)/agc_selftest "$(VALIDATION)" 120; \
	else \
		echo "(set VALIDATION=path/to/Validation.agc.bin to run the AGC self-test;"; \
		echo " tools/verify_rope.sh builds it from Virtual AGC)"; \
	fi

# --- Flight tests: the real flight software flies real mission phases -------
# (each takes a minute or more; they need the rope/ directory)


$(TESTDIR)/lm_landing: tests/lm_landing_test.cpp $(GNC_SRCS)
	@mkdir -p $(TESTDIR)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(TESTDIR)/csm_burn: tests/csm_burn_test.cpp $(GNC_SRCS)
	@mkdir -p $(TESTDIR)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(TESTDIR)/cm_entry: tests/cm_entry_test.cpp $(GNC_SRCS)
	@mkdir -p $(TESTDIR)
	$(CXX) $(CXXFLAGS) -o $@ $^

.PHONY: flighttest
flighttest: $(TESTDIR)/lm_landing $(TESTDIR)/csm_burn $(TESTDIR)/cm_entry $(TESTDIR)/flight_test $(TESTDIR)/rtcc_test $(TESTDIR)/launch_test $(TESTDIR)/contingency_test $(TESTDIR)/autocrew_test
	$(TESTDIR)/lm_landing rope Luminary099 | tail -3
	$(TESTDIR)/csm_burn rope Comanche055 | tail -2
	$(TESTDIR)/cm_entry rope Comanche055 | tail -2
	$(TESTDIR)/rtcc_test
	$(TESTDIR)/flight_test rope 11 | tail -1
	$(TESTDIR)/launch_test rope 11 1 | tail -3
	$(TESTDIR)/launch_test rope 7 | tail -2
	$(TESTDIR)/contingency_test rope | tail -1
	$(TESTDIR)/autocrew_test rope all | tail -1

clean:
	rm -rf $(OBJDIR) $(NAME)


FLIGHT_TESTS := flight_test rtcc_test launch_test contingency_test \
	autocrew_test mission_test_full recording_test

$(FLIGHT_TESTS:%=$(TESTDIR)/%): $(TESTDIR)/%: $(OBJDIR)/tests/%.o $(FLIGHT_OBJS)
	@mkdir -p $(TESTDIR)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(OBJDIR)/tests/%.o: tests/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -MMD -MP -c -o $@ $<

$(TESTDIR)/session_test: $(OBJDIR)/tests/session_test.o $(OBJDIR)/app/FlightSession.o $(FLIGHT_OBJS)
	@mkdir -p $(TESTDIR)
	$(CXX) $(CXXFLAGS) -pthread -o $@ $^

$(TESTDIR)/decision_test: $(OBJDIR)/tests/decision_test.o $(OBJDIR)/app/FlightSession.o $(FLIGHT_OBJS)
	@mkdir -p $(TESTDIR)
	$(CXX) $(CXXFLAGS) -pthread -o $@ $^

$(TESTDIR)/drill_test: $(OBJDIR)/tests/drill_test.o $(OBJDIR)/app/FlightSession.o $(FLIGHT_OBJS)
	@mkdir -p $(TESTDIR)
	$(CXX) $(CXXFLAGS) -pthread -o $@ $^

# --- Flight recordings (for the screensaver and replays) ---------------------

$(OBJDIR)/record_mission: $(OBJDIR)/tools/record_mission.o $(FLIGHT_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

RECORDINGS := recordings
MISSIONS_TO_RECORD ?= 11

.PHONY: recordings
recordings: $(OBJDIR)/record_mission
	@mkdir -p $(RECORDINGS)
	@for n in $(MISSIONS_TO_RECORD); do \
		$(OBJDIR)/record_mission rope $$n $(RECORDINGS)/apollo$$n.colrec --quiet \
			--difficulty $${DIFFICULTY:-asflown}; \
	done

# Renders replay frames (the screensaver's drawing) to PNGs off-Haiku.
REPLAY_SRCS := ui/ReplayRenderer.cpp ui/ReplayPacer.cpp ui/DSKYPainter.cpp ui/Drawing.cpp \
	ui/Coastline.cpp core/flight/Recording.cpp core/sim/World.cpp

$(OBJDIR)/preview/replay: tests/preview/replay_preview.cpp tests/preview/HaikuShim.cpp $(REPLAY_SRCS)
	@mkdir -p $(OBJDIR)/preview
	$(CXX) $(CXXFLAGS) -Wno-deprecated-copy -Wno-multichar -Itests/preview/shim -Iui -Icore \
		$$(pkg-config --cflags cairo) -o $@ $^ $$(pkg-config --libs cairo)

# --- The screensaver (Haiku) --------------------------------------------------
# A Screen Saver add-on that replays flight recordings. `make saver` builds
# it; `make install-saver` installs it with the recordings for the ScreenSaver
# preferences to find.

SAVER_SRCS := screensaver/ColumbiaSaver.cpp $(REPLAY_SRCS)

saver: $(OBJDIR)/saver/Columbia

$(OBJDIR)/saver/Columbia: $(SAVER_SRCS)
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -Iui -fPIC -shared -o $@ $^ -lbe -lscreensaver
	@if command -v mimeset >/dev/null 2>&1; then mimeset -f $@; fi

SAVER_DIR := $(shell finddir B_USER_NONPACKAGED_ADDONS_DIRECTORY 2>/dev/null)/Screen Savers
DATA_DIR := $(shell finddir B_USER_NONPACKAGED_DATA_DIRECTORY 2>/dev/null)/Columbia

install-saver: saver
	mkdir -p "$(SAVER_DIR)" "$(DATA_DIR)/recordings"
	cp $(OBJDIR)/saver/Columbia "$(SAVER_DIR)/Columbia"
	-cp $(RECORDINGS)/*.colrec "$(DATA_DIR)/recordings/"

$(OBJDIR)/preview/panel: tests/preview/panel_preview.cpp tests/preview/HaikuShim.cpp app/PanelView.cpp app/WindowView.cpp app/FlightSession.cpp ui/DSKYPainter.cpp ui/Drawing.cpp ui/Coastline.cpp $(FLIGHT_SRCS)
	@mkdir -p $(OBJDIR)/preview
	$(CXX) $(CXXFLAGS) -Wno-deprecated-copy -Wno-multichar -Itests/preview/shim -Iui -Icore -Iapp \
		$$(pkg-config --cflags cairo) -o $@ $^ $$(pkg-config --libs cairo)
