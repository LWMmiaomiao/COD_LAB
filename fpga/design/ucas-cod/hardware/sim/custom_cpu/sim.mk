SIM_TOP := custom_cpu_test
SIM_TOP_IV := custom_cpu_test_iverilog

SIM_BIN := $(SIM_OBJ_LOC)/sim.vvp
SIM_DUMP := $(SIM_OBJ_LOC)/dump.fst

SIM_SRCS := $(wildcard $(RTL_SRC_LOC)/alu/*.v)
SIM_SRCS += $(wildcard $(RTL_SRC_LOC)/reg_file/*.v)
SIM_SRCS += $(wildcard $(RTL_SRC_LOC)/shifter/*.v)
SIM_SRCS += $(wildcard $(RTL_SRC_LOC)/$(SIM_TARGET)/$(DUT_ISA)/*.v)
SIM_SRCS += $(wildcard $(RTL_SRC_LOC)/$(SIM_TARGET)/cache/*.v)
SIM_SRCS += $(wildcard $(RTL_SRC_LOC)/$(SIM_TARGET)/dma/*.v)
SIM_SRCS += $(wildcard $(RTL_SRC_LOC)/../wrapper/$(SIM_TARGET)/*.v)
SIM_SRCS += $(wildcard $(SIM_SRC_LOC)/$(SIM_TARGET)/$(DUT_ARCH)/*.v)
SIM_SRCS += $(wildcard $(SIM_SRC_LOC)/$(SIM_TARGET)/common/*.v)

SIM_SRCS_IV := $(wildcard $(SIM_SRC_LOC)/$(SIM_TARGET)/$(DUT_ARCH)/golden/$(DUT_ISA)/*.v)

SIM_SRCS_VL := $(wildcard $(SIM_SRC_LOC)/$(SIM_TARGET)/$(DUT_ARCH)/golden/$(DUT_ISA)/*.sv)
SIM_SRCS_VL += $(realpath $(wildcard $(SIM_SRC_LOC)/$(SIM_TARGET)/$(DUT_ARCH)/*.cpp))
SIM_SRCS_VL += $(realpath $(wildcard $(SIM_SRC_LOC)/$(SIM_TARGET)/$(DUT_ARCH)/golden/$(DUT_ISA)/*.a))

IV_FLAGS := -I ../
IV_FLAGS += -I $(SIM_SRC_LOC)/$(SIM_TARGET)
IV_FLAGS += -I $(RTL_SRC_LOC)/$(SIM_TARGET)/$(DUT_ISA)/include
IV_FLAGS += -DTRACE_FILE=\"$(TRACE_FILE)\"
IV_FLAGS += -DAXI_RAM_ADDR_WIDTH=14
IV_FLAGS += -grelative-include
IV_FLAGS += -g2012

VL_FLAGS := --trace-fst
VL_FLAGS += +incdir+$(SIM_SRC_LOC)/$(SIM_TARGET)
VL_FLAGS += +incdir+$(RTL_SRC_LOC)/$(SIM_TARGET)/$(DUT_ISA)/include
VL_FLAGS += --relative-includes
VL_FLAGS += -DTRACE_FILE=\"$(TRACE_FILE)\"	# unnecessary, just to get custom_cpu_test_iverilog compiled in verilator

# Parsing user-defined architectural options
ARCH_OPTION_TCL := $(RTL_SRC_LOC)/$(SIM_TARGET)/arch_options.tcl

USE_ICACHE     := $(shell cat $(ARCH_OPTION_TCL) | grep "icache" | awk '{print $$3}')
USE_DCACHE     := $(shell cat $(ARCH_OPTION_TCL) | grep "dcache" | awk '{print $$3}')

ifeq ($(USE_ICACHE),1)
IV_FLAGS += -DUSE_ICACHE
VL_FLAGS += -DUSE_ICACHE
endif

ifeq ($(USE_DCACHE),1)
IV_FLAGS += -DUSE_DCACHE
VL_FLAGS += -DUSE_DCACHE
endif

# Use FST format for waveform to provide better compression ratio
# Waveform file size would be reduced by about 10x.
PLUSARGS += -fst
PLUSARGS += +INITMEM="$(MEM_FILE)" +TRACE_FILE="$(TRACE_FILE)"
