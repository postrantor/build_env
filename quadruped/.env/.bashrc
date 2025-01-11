export MACHINE=vmware

# set workdir
export QUAD_WORKDIR=${HOME}/project/build_env/quadruped/
source ${QUAD_WORKDIR}/.env/bashrc
source ${QUAD_WORKDIR}/.env/system
source ${QUAD_WORKDIR}/.env/robot

export http_proxy=http://192.168.31.201:7891
export https_proxy=http://192.168.31.201:7891

export PATH=.:${HOME}/.local/bin:$PATH
