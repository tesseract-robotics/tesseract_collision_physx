# Tesseract Collision PhysX Implementation

This package supports both CPU and GPU functionality of PhysX.

## CPU

Currently there are no known limitation when using the CPU.

## GPU
Currently when using the GPU there are some limitation because it must pre-allocate space.

* Convex hulls with more than 64 vertices or with more than 32 vertices per-face will have their contacts processed by the CPU rather than the GPU.
  * Note: Because of this if GPU is enabled Tesseract PhysX will convert the convex shape to be GPU compatible. This will change the shape but it is unclear how and if it is conservative so all original vertices lie within the new convex shape.

## PhysX Documentation

Please see [Release Notes](http://gameworksdocs.nvidia.com/PhysX/4.1/release_notes.html) for updates pertaining to the latest version.

The full set of documentation can also be found in the repository under physx/documentation or online at http://gameworksdocs.nvidia.com/simulation.html

Platform specific and Install information can be found here:
* [Microsoft Windows](http://gameworksdocs.nvidia.com/PhysX/4.1/documentation/platformreadme/windows/readme_windows.html)
* [Linux](http://gameworksdocs.nvidia.com/PhysX/4.1/documentation/platformreadme/linux/readme_linux.html)
* [Google Android ARM](http://gameworksdocs.nvidia.com/PhysX/4.1/documentation/platformreadme/android/readme_android.html)
* [Apple macOS](http://gameworksdocs.nvidia.com/PhysX/4.1/documentation/platformreadme/mac/readme_mac.html)
* [Apple iOS](http://gameworksdocs.nvidia.com/PhysX/4.1/documentation/platformreadme/ios/readme_ios.html)

## PhysX Installation

This is optional for CPU and GPU since the static and dynamic libraries are included in this repository for linux.

* Install CMake 3.12
  * Clone CMake
  * mdkir build && cd build && cmake .. && make
  * sudo checkinstall --pkgname=cmake312
  * cmake --version
* Install PhysX
  * Clone PhysX
  * Run setup script: ./Physx/physx/generate_setup.sh
  * cd PhysX/physx/compiler/linux-release && make
  * Change tesseract_collision_physx cmake line 11 to my path:
    * `set(PHYSX_ROOT_DIR /home/ros-industrial/workspaces/realtime_ros2/src/PhysX/physx/install/linux)`
  * Changeline 14 to release. I must have built it in release.
    * `link_directories(${PHYSX_ROOT_DIR}/PhysX/bin/linux.clang/release)`

## Developers Links
[GPU Rigid Bodies](https://gameworksdocs.nvidia.com/PhysX/4.1/documentation/physxguide/Manual/GPURigidBodies.html#gpu-rigid-bodies)
[What is GPU Accelerated?](https://gameworksdocs.nvidia.com/PhysX/4.1/documentation/physxguide/Manual/GPURigidBodies.html#what-is-gpu-accelerated)
[Best Practices GPU](https://gameworksdocs.nvidia.com/PhysX/4.1/documentation/physxguide/Manual/BestPractices.html?highlight=gpu#gpu-rigid-bodies)

## Known Issues
* Kinematic objects are only guaranteed to be awake for one simulations timestep after setKinematicTarget() is called
  and then they may go to sleep even if in collision. You must always set the active links transforms even if they have
  not moved. Through testing it appears you get two more simulation timesteps before it will go to sleep but it is not
  guaranteed to always be true.


## TODO
* For Mesh and Octomap types we should create PxBVHStructure when adding the actor to the scene or aggregate.
  * See https://github.com/NVIDIAGameWorks/PhysX/issues/203
  * https://gameworksdocs.nvidia.com/PhysX/4.1/documentation/physxguide/Manual/SceneQueries.html#pxbvhstructure
* Update Mesh types to be a single actor with triangles as a convex shape using a PxBVHStructure


