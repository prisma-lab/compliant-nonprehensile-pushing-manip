#! /usr/bin/bash

export ACADOS_INSTALL_DIR=~/acados/
MODEL_FOLDER=${MODEL_FOLDER:-"./build"}
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ACADOS_INSTALL_DIR/lib:$MODEL_FOLDER
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ACADOS_INSTALL_DIR/interfaces/acados_template/tera_renderer/t_renderer/target/release

