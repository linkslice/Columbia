# Columbia — Apollo flight simulator for Haiku
# Copyright (c) 2026 Bryan Irvine. MIT License (see LICENSE).
#
#   make            build the Haiku application (on Haiku)
#   make test       build and run the headless tests (any POSIX host)

NAME     := Columbia
CXX      ?= g++
CXXFLAGS ?= -O2
CXXFLAGS += -std=c++17 -Wall -Wextra -Icore -Iapp

CORE_SRCS := \
	core/Mission.cpp \
	core/agc/AGC.cpp \
	core/agc/DSKY.cpp \
	core/agc/Comanche055Rope.cpp \
	core/sim/Flight.cpp \
	core/sim/Planner.cpp

APP_SRCS := \
	app/App.cpp \
	app/BurnWindow.cpp \
	app/DSKYView.cpp \
	app/MainWindow.cpp \
	app/TrajectoryView.cpp

OBJDIR := objects
CORE_OBJS := $(CORE_SRCS:%.cpp=$(OBJDIR)/%.o)
APP_OBJS  := $(APP_SRCS:%.cpp=$(OBJDIR)/%.o)
LIBS      := -lbe -ltracker

.PHONY: all test preview clean

all: $(NAME)

$(NAME): $(CORE_OBJS) $(APP_OBJS) Columbia.rdef
	$(CXX) -o $@ $(CORE_OBJS) $(APP_OBJS) $(LIBS)
	@if command -v rc >/dev/null 2>&1 && command -v xres >/dev/null 2>&1; then \
		rc -o $(OBJDIR)/Columbia.rsrc Columbia.rdef && \
		xres -o $@ $(OBJDIR)/Columbia.rsrc; \
	fi
	@if command -v mimeset >/dev/null 2>&1; then mimeset -f $@; fi

$(OBJDIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

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

GNC_SRCS := core/agc/AGC.cpp core/agc/DSKY.cpp $(wildcard core/gnc/*.cpp) \
	core/sim/World.cpp core/sim/Vehicle.cpp core/sim/Spacecraft.cpp \
	core/sim/Aero.cpp core/sim/MoonScene.cpp

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

# Renders the real DSKY and trajectory views to PNGs through a small Cairo
# stand-in for the Interface Kit (needs cairo; useful off-Haiku).
preview: tests/preview/preview.cpp tests/preview/HaikuShim.cpp app/DSKYView.cpp app/TrajectoryView.cpp $(CORE_SRCS)
	@mkdir -p $(OBJDIR)/preview
	$(CXX) $(CXXFLAGS) -Wno-deprecated-copy -Itests/preview/shim \
		$$(pkg-config --cflags cairo) -o $(OBJDIR)/preview/render $^ \
		$$(pkg-config --libs cairo)
	$(OBJDIR)/preview/render $(OBJDIR)/preview

clean:
	rm -rf $(OBJDIR) $(NAME)

FLIGHT_SRCS := $(GNC_SRCS) $(wildcard core/flight/*.cpp)

$(TESTDIR)/flight_test: tests/flight_test.cpp $(FLIGHT_SRCS)
	@mkdir -p $(TESTDIR)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(TESTDIR)/rtcc_test: tests/rtcc_test.cpp $(FLIGHT_SRCS)
	@mkdir -p $(TESTDIR)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(TESTDIR)/launch_test: tests/launch_test.cpp $(FLIGHT_SRCS)
	@mkdir -p $(TESTDIR)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(TESTDIR)/contingency_test: tests/contingency_test.cpp $(FLIGHT_SRCS)
	@mkdir -p $(TESTDIR)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(TESTDIR)/autocrew_test: tests/autocrew_test.cpp $(FLIGHT_SRCS)
	@mkdir -p $(TESTDIR)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(TESTDIR)/mission_test_full: tests/mission_test_full.cpp $(FLIGHT_SRCS)
	@mkdir -p $(TESTDIR)
	$(CXX) $(CXXFLAGS) -o $@ $^
