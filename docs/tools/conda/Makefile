SHELL=/bin/bash
CONDA_CONFIG_DIR=.

environment:
	conda config --add channels conda-forge
	conda config --set channel_priority strict
	conda env create --file $(CONDA_CONFIG_DIR)/environment.yml
