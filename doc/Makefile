SHELL=/bin/bash
DOC_DIR=source
CONDA_CONFIG_DIR=tools/conda

environment:
	conda config --add channels conda-forge
	conda config --set channel_priority strict
	conda env create --file $(CONDA_CONFIG_DIR)/environment.yml

# all the following targets shall be executed in the opm conda environment

.PHONY: doc
doc:
	make -C $(DOC_DIR) html

doc-pdf:
	make -C $(DOC_DIR) latexpdf

doc-clean:
	# clean sphinx build
	make -C $(DOC_DIR) clean
