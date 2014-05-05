#=======================================================================
# UCB VLSI FLOW: Makefile for sram generation
#-----------------------------------------------------------------------
# Stephen Twigg (sdtwigg@eecs.berkeley.edu)
#
# This makefile fragment will build a memory for each specified configuration
#

default: srams
all: srams

base_dir = $(abspath ../..)
sram_dir = $(abspath .)

sram_confs = $(wildcard $(sram_dir)/*.conf)
sram_names = $(notdir $(basename $(sram_confs)))
pathed_srams = $(addprefix $(sram_dir)/generated/, $(sram_names))

cacti = $(base_dir)/ucbsc/cacti65/cacti
$(cacti):
	cd $(base_dir)/ucbsc/cacti65; make -j

$(pathed_srams): $(sram_dir)/generated/%: $(sram_dir)/%.conf $(cacti)
	mkdir -p $(sram_dir)/generated; cd $(sram_dir)/generated/; $(base_dir)/ucbsc/ucbsc $(sram_dir)/$*.conf

srams: $(pathed_srams)

clean-srams:
	rm -rf $(pathed_srams)
