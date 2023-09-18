# OrbbecSDK-K4A-Wrapper

![stability](https://img.shields.io/badge/stability-unstable-yellow)

This branch is contain the K4A wrapper for OrbbecSDK. It's mean that user can use this library to develop the application with K4A API, but use to access the Orbbec camera.
Also user can use this library to replace native K4A library in your application to access the Orbbec camera without any code change.

*This repo is forked from [Azure Kinect Sensor SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK)*

*This branch is base on release/1.4.x, and use new impl code base on [OrbbecSDK](https://github.com/orbbec/OrbbecSDK) to replace the k4a implementation.*

*The usage of this library is same as [Native K4A](./README_K4A.md)*

## What we did?

The [k4a.h](./include/k4a/k4a.h) is the header file of K4A API, and the source code in [k4a.c](./src/sdk/k4a.c) is the implementation of K4A API. We have reimplemented the K4A API in [ob_k4a_impl.c](./src/orbbec/ob_k4a_impl.c) with OrbbecSDK, and keep the same effect as the original K4A API. Therefore, all functions called on user's side will be redirected to the OrbbecSDK, and user can access the Orbbec camera like the K4A device.

![OrbbecSDK-K4A-Wrapper](src/orbbec/docs/resource/OrbbecSDK-K4A-Wrapper.png)

## Supported camera and platform

Orbbec Femto Mega: Windows10+, Ubuntu20.04+; x64

*Other Orbbec cameras and platforms without test, don't use it in production environment.*

## How to build

### Get source code

```bash
git clone https://github.com/orbbec/OrbbecSDK-K4A-Wrapper.git
# Please make sure you have switched to the branch with the prefix "ob/", for example: git checkout ob/feature/1.1.x.
git submodule update --init --recursive
```

### Build && install

**It`s is same as the Native K4A [build](docs/building.md)**

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

### Test it!

Connect the Orbbec camera to your PC, and run the k4aviewer.

## Attention

1. The library of this branch is not support the K4A device, please use the [Native K4A](https://github.com/microsoft/Azure-Kinect-Sensor-SDK) library to access the K4A device.
2. The OrbbecSDK K4A Wrapper is aim to provide the same API as the K4A, but it's not full API for OrbbecSDK and feature for Orbbec camera. If you want to use the full feature of Orbbec camera, please use the [OrbbecSDK](https://github.com/orbbec/OrbbecSDK) directly.
