# Makefile
# See https://docs.cocotb.org/en/stable/quickstart.html for more info

# defaults
SIM ?= icarus
FST ?= -fst # Use more efficient FST format
TOPLEVEL_LANG ?= verilog
SRC_DIR = $(PWD)/../src
PROJECT_SOURCES = project.v tinyqv.v cpu/core.v cpu/decode.v cpu/alu.v cpu/register.v cpu/counter.v cpu/cpu.v cpu/mem_ctrl.v cpu/qspi_ctrl.v cpu/spi_mem_ctrl.v peri/uart/uart_tx.v

ifneq ($(GATES),yes)

# RTL simulation:
SIM_BUILD				= sim_build/rtl
VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))

else

# Gate level simulation:
# SDF-annotated timed sim (LOCAL ONLY, EXPERIMENTAL): GATES=yes SDF=1 make
# Default (CI path) is unchanged: functional, no annotate.
# KNOWN LIMIT (verified Sep 2026): icarus SDF support is too partial for the
# OpenSTA SDF — it ignores args beyond the first two, aborts parsing on COND
# path delays / port-level INTERCONNECT / empty-typ triplets ("Too many
# errors: syntax error"), and silently drops ifnone timing checks. The result
# is half-annotated timing that fails tests for TOOL reasons (X storm), while
# the identical netlist passes functional GL and STA is clean (+3.02/-0.11ns).
# Timing signoff = STA + functional GL, NOT this target.
ifeq ($(SDF),1)
SIM_BUILD				= sim_build/sdf
else
SIM_BUILD				= sim_build/gl
endif
COMPILE_ARGS    += -DGL_TEST
COMPILE_ARGS    += -DFUNCTIONAL
COMPILE_ARGS    += -DSIM
# -gspecify/-ginterconnect are REQUIRED for SDF=1 (else $sdf_annotate is
# silently omitted / INTERCONNECT entries can't attach); keep them out of
# the default/CI path to avoid the ifnone sorry-spam.
# -Tmax selects the defined SDF value (OpenSTA writes min::max with empty typ).
ifeq ($(SDF),1)
COMPILE_ARGS    += -gspecify -ginterconnect -Tmax
endif
VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13g2/libs.ref/sg13g2_io/verilog/sg13g2_io.v
VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13g2/libs.ref/sg13g2_stdcell/verilog/sg13g2_stdcell.v

# this gets copied in by the GDS action workflow
VERILOG_SOURCES += $(PWD)/gate_level_netlist.v

endif

# Allow sharing configuration between design and testbench via `include`:
COMPILE_ARGS 		+= -I$(SRC_DIR)

# Include the testbench sources:
# (SDF=1 selects the annotate variant; default tb.v is the CI path)
ifeq ($(SDF),1)
VERILOG_SOURCES += $(PWD)/tb_sdf.v
else
VERILOG_SOURCES += $(PWD)/tb.v
endif
TOPLEVEL = tb

# List test modules to run, separated by commas and without the .py suffix:
COCOTB_TEST_MODULES = test

# include cocotb's make rules to take care of the simulator setup
include $(shell cocotb-config --makefiles)/Makefile.sim
