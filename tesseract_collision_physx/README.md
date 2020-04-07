# Tesseract Collision PhysX Implementation

This package supports both CPU and GPU functionality of PhysX.

## CPU

Currently there are known know limitation when using the CPU.

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

## Developers Links
[GPU Rigid Bodies](https://gameworksdocs.nvidia.com/PhysX/4.1/documentation/physxguide/Manual/GPURigidBodies.html#gpu-rigid-bodies)
[What is GPU Accelerated?](https://gameworksdocs.nvidia.com/PhysX/4.1/documentation/physxguide/Manual/GPURigidBodies.html#what-is-gpu-accelerated)
[Best Practices GPU](https://gameworksdocs.nvidia.com/PhysX/4.1/documentation/physxguide/Manual/BestPractices.html?highlight=gpu#gpu-rigid-bodies)

