PIN_DEF := ./constraints/ULX3S.lpf
DEVICE ?= 85k
BUILDDIR = bin
YOSYS_OPTIONS=

ZTACHIP = $(wildcard ./components/ztachip/HW/src/*.vhd) $(wildcard ./components/ztachip/HW/src/*/*.vhd)
AXI_CROSSBAR = $(wildcard ./components/axi-crossbar/rtl/*.sv)
TOP_MODULE = main
VHDL_TOP_MODULE = sdram
VERILOG = $(wildcard ./top/*.v) $(wildcard ./components/*.v) $(wildcard ./components/*.sv) $(AXI_CROSSBAR)
VHDL = $(wildcard ./components/*.vhd) $(ZTACHIP)

BUILDDIR = bin

compile: $(BUILDDIR)/toplevel.bit

prog: $(BUILDDIR)/toplevel.bit
	fujprog $^

flash: $(BUILDDIR)/toplevel.bit
	fujprog -j flash $^

$(BUILDDIR)/toplevel.json: $(VERILOG) $(VHDL)
	mkdir -p $(BUILDDIR)
	yosys \
	-p "read_verilog -sv $(VERILOG)" \
	-m ghdl \
	-p "ghdl --ieee=synopsys --std=08 -fexplicit -frelaxed-rules $(VHDL) -e $(VHDL_TOP_MODULE)" \
	-p "hierarchy -top ${TOP_MODULE}" \
	-p "synth_ecp5 ${YOSYS_OPTIONS} -json $(BUILDDIR)/toplevel.json"


$(BUILDDIR)/%.config: $(PIN_DEF) $(BUILDDIR)/toplevel.json
	nextpnr-ecp5 --${DEVICE} --package CABGA381 --freq 25 --timing-allow-fail --textcfg  $@ --json $(filter-out $<,$^) --lpf $<

$(BUILDDIR)/toplevel.bit: $(BUILDDIR)/toplevel.config
	ecppack --compress $^ $@

clean:
	rm -rf ${BUILDDIR}

.SECONDARY:
.PHONY: compile clean prog flash
