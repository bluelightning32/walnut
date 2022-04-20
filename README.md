## Build the project on Linux ##
  mkdir build
  cd build
  cmake ../
  make -j4

## Building and installing VTK in Windows ##
These directions were tested with Visual Studio Community Edition 2022.

There is no VTK installer for Windows. VTK must be built from source, or Walnut
can be configured to skip components that depend on VTK. To build VTK, start by
downloading a zip copy of a release tag from the VTK Github page, such as
https://github.com/Kitware/VTK/archive/refs/tags/v9.1.0.zip. Extract it into
your Visual Studio repos directory.

Open the folder in Visual Studio. It should start running the CMake
configuration for x64-Debug automatically.

When it is done configuring, select "Install VTK" from the build menu. It will
start building everything, then install it in the out/install/x64-Debug
subfolder of the VTK-9.1.0 source folder.

## Building Walnut on Windows ##
Clone https://github.com/bluelightning32/walnut.git. After opening the cloned
folder, Visual Studio will try to run CMake, but fail saying it can't find VTK.

Select CMake Settings from the Project menu. Either set VTK_DIR to the VTK
source directory plus "out/install/x64-Debug/lib/cmake/vtk-9.1", or to skip
using VTK, uncheck WALNUT_USE_VTK. Save the CMakeSettings.json file, which will
automatically rerun CMake.

Select Build All. There may be a few minor warnings, but all build steps
should succeed. The Walnut unit tests will run automatically as part of
building Walnut.

The examples can be run by right clicking on the corresponding cpp file in the
examples folder in Visual Studio, then selecting Debug.

## Overview ##
Walnut is a library for performing 3D boolean operations. It also contains an
optional library that wraps the algorithm in a VTK filter, along with some
examples of how to use the VTK filter. These parts can be excluded from the
build with the -DUSE_VTK=OFF option to cmake.

## More build options ##
The -L option lists other common options.
  cmake ../ -L

For regular development, a debug release type is recommended. The debug release
has the asserts enabled, and it has full debug symbols.
  cmake ../ -DCMAKE_BUILD_TYPE=Debug

## Walnut VTK filter ##
To get started using it, take a look at the examples/union.cpp file. The
example uses the VTK adapter to Walnut, which is not required to use Walnut,
but it is the simplest way to use Walnut.

These are the key parts of the program:
```
  // Instantiate the boolean filter adapter
  auto bool_filter = vtkSmartPointer<walnut::vtkWalnutBooleanFilter>::New();
  // Give the boolean filter one or more other objects to perform the boolean
  // operation on.
  bool_filter->AddInputConnection(cu->GetOutputPort());
  bool_filter->AddInputConnection(cyl->GetOutputPort());
  bool_filter->AddInputConnection(cu2->GetOutputPort());
  // Enable the optional input quantization to speed up the Walnut algorithm.
  // Setting -8 means that the input coordinates are rounded to the nearest
  // multiple of 2^-8.
  bool_filter->SetMinExponent(-8);

  // Call one of the following functions to select the boolean operation.
  bool_filter->SetOperationToUnion();
  bool_filter->SetOperationToIntersection();
  bool_filter->SetOperationToDifference();
```

The example performs boolean operations between 2 rectangular prisms and a
cylinder. The cylinder and one rectangular prism overlap. The second
rectangular prism is disconnected to show how Walnut can handle disconnected
meshes. For illustrative purposes, the example program shows the facet normals,
and the facet edges (even of coplanar facets).

When running the example, use the ‘m’ key to cycle between 3 different boolean
operations: union, intersection, and set difference. When the program first
starts up, it shows a union of the 3 objects.

There are several other programs in the examples directory. Most of the example
programs illustrate some internal part of how Walnut works.

https://github.com/bluelightning32/vtk_print_examples has an example of how to
use Walnut from an external project.

## Design ##
A design doc for how the internals of Walnut work is available at:
https://docs.google.com/document/d/1G9616Bs4OURC1fKEUgY0b25A_Wra7Mn9mVxW1SmjVRY/edit?usp=sharing
