export MACHINE=vmware

# set workdir
export QUAD_WORKDIR=${HOME}/project/model_ws/
source ${QUAD_WORKDIR}/.env/bashrc
source ${QUAD_WORKDIR}/.env/robot
source ${QUAD_WORKDIR}/.env/custom

export http_proxy=http://192.168.31.201:7891
export https_proxy=http://192.168.31.201:7891

export PATH=.:${HOME}/.local/bin:$PATH
