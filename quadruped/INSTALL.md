## for ubuntu24.04

### python

需要降级 python3 到 3.10 避免 matplotlibcpp.h 包含弃用的 c API

```bash
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.10

sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1
sudo update-alternatives --config python3
```

### g++

```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt update
sudo apt install gcc-11 g++-11

sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 100
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 100
```

## detect cpu temperature

```bash
sudo apt-get install lm-sensors
sudo sensors-detect
sensors
```

## conda

> ref: [miniconda](https://docs.anaconda.com/miniconda/)

These four commands download the latest 64-bit version of the Linux installer, rename it to a shorter file name, silently install, and then delete the installer:

```bash
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm ~/miniconda3/miniconda.sh
```

After installing, initialize your newly-installed Miniconda. The following commands initialize for bash and zsh shells:

```bash
~/miniconda3/bin/conda init bash
~/miniconda3/bin/conda init zsh
```

## code lint

### clang

[clangd](https://clangd.llvm.org/installation.html)

```bash
apt install \
  clang-15 \
  clang-format-15 \
  clangd-15 \
  clang-tools-15 \
  clang-tidy-15

update-alternatives --install /usr/bin/clangd clangd /usr/bin/clangd-15 100
```
