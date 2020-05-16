SHELL=/bin/bash

NAME=uav_model
ENV_NAME=$(file < env-name)

CONDA_ACTIVATE=set -e; \
	       source $$(conda info --base)/etc/profile.d/conda.sh; \
	       conda activate $(ENV_NAME)

# configure conda
config:
	conda config --add channels conda-forge
	conda config --set channel_priority strict

# create the anaconda environment
create: config
	conda create --yes --name $(ENV_NAME) python=3.7 pip conda-build

# install the development requirements
requirements-dev:
	$(CONDA_ACTIVATE); \
	pip install --requirement requirements-dev.txt

# prepare the anaconda environment with the dev requirements
environment: create requirements-dev

# all the following targets shall be executed after having activated an environment

# install lbpost with its requirements
requirements:
	conda install --yes $$(cat requirements.txt)

# install for development
develop: requirements
	pip install --force-reinstall --no-deps --editable .

.PHONY: doc
doc:
	cd doc && make html

flake8: ci-dir
	flake8 --tee --output-file .ci/flake8.log $(NAME) tests

check: mypy flake8

black:
	black -t py37 --exclude _version.py tests $(NAME)

style: isort black
