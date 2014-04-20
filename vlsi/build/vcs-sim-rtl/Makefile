#=======================================================================
# UCB VLSI FLOW: Makefile for vcs-sim-rtl
#-----------------------------------------------------------------------
# Yunsup Lee (yunsup@cs.berkeley.edu)
#
# This makefile will build a rtl simulator and run various tests to
# verify proper functionality.
#

include ../Makefrag

default : all

basedir  = ..

#--------------------------------------------------------------------
# Sources
#--------------------------------------------------------------------

# Library components

vclibdir = $(UCB_VLSI_HOME)/install/vclib
#vclibsrcs = \
#	$(vclibdir)/vcQueues.v \
#	$(vclibdir)/vcStateElements.v \
#	$(vclibdir)/vcTest.v \
#	$(vclibdir)/vcTestSource.v \
#	$(vclibdir)/vcTestSink.v \

# Verilog sources

srcdir = $(basedir)/generated-src
vsrcs = \
	$(srcdir)/FSM.v \
#	$(srcdir)/Cpu.v \


#--------------------------------------------------------------------
# Build rules
#--------------------------------------------------------------------

VCS      = vcs -full64
VCS_OPTS = -notice -PP -line +lint=all,noVCDE +v2k -timescale=1ns/10ps

#--------------------------------------------------------------------
# Build the simulator
#--------------------------------------------------------------------

vcs_sim = simv
$(vcs_sim) : Makefile $(vsrcs)
	$(VCS) $(VCS_OPTS) +incdir+$(srcdir) -o $(vcs_sim) \
	       +define+CLOCK_PERIOD=$(vcs_clock_period) \
	        $(vsrcs)
$(srcdir)/FSM.v:
	cd ../../.. && sbt "project fsm" "run -vbuild --backend Chisel.Fame5VerilogBackend --targetDir vlsi/build/generated-src"
$(srcdir)/Cpu.v:
	cd ../../.. && sbt "project cpu" "run -vbuild --backend Chisel.Fame5VerilogBackend --targetDir vlsi/build/generated-src"
	
#--------------------------------------------------------------------
# Run
#--------------------------------------------------------------------

vpd = vcdplus.vpd
$(vpd): $(vcs_sim)
	./simv +verbose=1
	date > timestamp

run: $(vpd)

#--------------------------------------------------------------------
# Default make target
#--------------------------------------------------------------------

.PHONY: run

all : $(vcs_sim)

#--------------------------------------------------------------------
# Clean up
#--------------------------------------------------------------------

junk += simv* csrc *.vpd *.key DVE* .vcs* timestamp

clean :
	rm -rf $(junk) *~ \#* *.log *.cmd *.daidir
	rm -f ../generated-src/*.v
 
