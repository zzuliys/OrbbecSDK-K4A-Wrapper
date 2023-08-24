# OrbbecSDK-K4A-Wrapper

![stability](https://img.shields.io/badge/stability-unstable-yellow)

This branch is contain the K4A wrapper for OrbbecSDK. It's mean that user can use this library to develop the application with K4A API, but use to control the Orbbec camera.
Also user can use this library to replace native K4A library in your application to control the Orbbec camera without any code change.

*This repo is forked from [Azure Kinect Sensor SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK)*

*This branch is base on release/1.4.x, and use new impl code base on [OrbbecSDK](https://github.com/orbbec/OrbbecSDK) to replace the k4a implementation.*

*The usage of this library is same as [Native K4A](./README_K4A.md)*

## Supported camera and platform

Orbbec Femto Mega: Win10+、Ubuntu20.04+, x64

*Other Orbbec cameras and platforms without test, don't use it in production environment.*

## How to build

### Get source code

```bash
git clone https://github.com/orbbec/OrbbecSDK-K4A-Wrapper.git
# Please make sure you have switched to the branch with the prefix "orb/", for example: git checkout orb/feature/1.1.x.
git submodule update --init --recursive
```

### Build && install

* Windows: it's recommend to use the Ninja to build the project

```powershell
cd OrbbecSDK-K4A-Wrapper
mkdir build && cd build
cmake .. -G Ninja
cmake --build .
cmake --install .
```

* Linux

```bash
cd OrbbecSDK-K4A-Wrapper
mkdir build && cd build
cmake ..
cmake --build .
cmake --install
```

### Enjoy it!

Connect the Orbbec camera to your PC, and run the k4aviewer.

## Attention

1. The library of this branch is not support the K4A device, please use the [Native K4A](https://github.com/microsoft/Azure-Kinect-Sensor-SDK) library to control the K4A device.
