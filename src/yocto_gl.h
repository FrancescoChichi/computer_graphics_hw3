///
/// # Yocto/GL: Single-file C++ Library for Physically-based Graphics
///
/// Yocto/GL is a collection utiliies for building physically-based graphics
/// algorithms implemented as a single-file header-only library `yocto_gl.h`
/// and released under the MIT license. Features include:
///
/// - convenience math functions for graphics
/// - static length vectors, with specialization for 2, 3, 4 length
/// - static length matrices, with specialization for 2x2, 3x3, 4x4
/// - static length rigid transforms (frames), specialized for 2d and 3d space
/// - static length growable vectors)
/// - linear algebra operations and transforms for fixed length matrices/vecs
/// - axis aligned bounding boxes
/// - rays and ray-primitive intersection
/// - point-primitive distance and overlap tests
/// - normal and tangent computation for meshes and lines
/// - generation of tesselated meshes
/// - mesh refinement with linear tesselation and Catmull-Cark subdivision
/// - random number generation via PCG32
/// - trivial image data structure and a few image operations
/// - BVH for intersection and closest point query
/// - Python-like iterators, string, path and container operations
/// - utilities to load and save entire text and binary files
/// - immediate mode command line parser
/// - simple logger and thread pool
/// - a fully featured path tracer supporting surfaces and hairs, GGX and MIS
/// - a simple scene format
/// - support for loading and saving Wavefront OBJ and Khronos glTF
/// - OpenGL utilities to manage textures, buffers and prograrms
/// - OpenGL shader for image viewing
/// - OpenGL shader for GGX microfacet and hair rendering with skinning
///
///
/// ## Compilation
///
/// Yocto/GL is written in C++14, with compilation supported on C++11, and
/// compiles on OSX (clang/gcc), Linux (gcc) and Windows (MSVC 2017).
///
/// For image loading and saving, this library depends on `stb_image.h`,
/// `stb_image_write.h`, `stb_image_resize.h` and `tinyexr.h`. These features
/// can be disabled by defining YGL_IMAGEIO to 0 before including this file.
/// If these features are useful, then the implementation files need to
/// included in the manner described by the respecitive libraries. To simplify
/// the build process, you can alternatively define YGL_IMAGEIO_IMPLEMENTATION
/// before including this, but note that this should be done for only one
/// file in your aopplication or library.
///
/// The library includes parsers for Wavefront OBJ and glTF and a small scene
/// for quickly creating demos. Since the code these libraries is quite heavy
/// you can disable them by setting YGL_SCENEIO to 0. glTF also depends
/// on `json.hpp`.
///
/// OpenGL utilities include the OpenGL libaries and use GLEW on Windows/Linux.
/// Since OpenGL is quite onerous and hard to link, its support is disabled by
/// default. You can enable it by defining YGL_OPENGL to 1 before including
/// this file. If you use any of the OpenGL calls, make sure to properly link to
/// the OpenGL libraries on your system.
///
/// OpenGL windowing and widgets uses GLFW and ImGui and is diabled by default.
/// You can enable it by defning YGL_GLFW and YGL_IMGUI to 1 before
/// including this file. If you use the OpenGL windowing functionality, please
/// make sure to link to GLFW and the ImGui implementation files. To simplify
/// the build process, you can alternatively define YGL_IMGUI_IMPLEMENTATION
/// before including this, but note that this should be done for only one
/// file in your aopplication or library.
///
/// This library includes code from the PCG random number generator,
/// the LLVM thread pool, boost hash_combine, Pixar multijittered sampling,
/// code from "Real-Time Collision Detection" by Christer Ericson, base64
/// encode/decode by René Nyffenegger and public domain code from
/// github.com/sgorsten/linalg and gist.github.com/badboy/6267743.
///
/// This library imports many symbols from std for three reasons: avoid
/// verbosity , esnuring better conventions when calling math functions and
/// allowing easy overriding of std containers if desired. Just do not
/// flatten this namespace into yours if this is a concern.
///
/// For most components of the library, the use should be relatively easy to
/// understand if you are familiar with 3d computer graphics. For more complex
/// components, we follow the usage below.
///
///
/// ## Example Applications
///
/// You can see Yocto/GL in action in the following applications written to
/// test the library:
///
/// - `yview.cpp`: simple OpenGL viewer for OBJ and glTF scenes
/// - `ytrace.cpp`: offline path-tracer
/// - `yitrace.cpp.cpp`: interactive path-tracer
/// - `yscnproc.cpp`: scene manipulation and conversion to/from OBJ and glTF
/// - `ytestgen.cpp`: creates test cases for the path tracer and GL viewer
/// - `yimview.cpp`: HDR/PNG/JPG image viewer with exposure/gamma tone mapping
/// - `yimproc.cpp`: offline image manipulation.
///
/// You can build the example applications using CMake with
///     `mkdir build; cd build; cmake ..; cmake --build`
///
/// Here are two images rendered with the buildin path tracer, where the
/// scenes are crated with the test generator.
///
/// ![Yocto/GL](images/shapes.png)
///
/// ![Yocto/GL](images/lines.png)
///
/// ## Usage
///
/// To use the library simply include this file and setup the compilation
/// option as described above.
/// All library features are documented at the definition and should be
/// relatively easy to use if you are familiar with writing graphics code.
/// You can find the extracted documentation at `yocto_gl.md`.
/// Here we give an overview of some of the main features.
///
/// ### Command Line Parsing
///
/// The library includes a simple command line parser that parses commands in
/// immediate mode, i.e. when an option is declared. The parser supports options
/// and unnamed arguments with generic types parsed using C++ stream. The
/// parser autogenerates its own documentation. This allows to write complex
/// command lines with a tiny amount of implementation code on both the library
/// and user end.
///
/// 1. create a `cmdline` parser object by passing `argc, argv, name, help`
///     - an option for printing help is automatically added
/// 2. for each option, parse it calling the functions `parse_opt()`
///     - options are parsed on the fly and a comprehensive help is
///       automatically generated
///     - supports bool (flags), int, float, double, string, enums
///     - options names are "--longname" for longname and "-s" for short
///     - command line format is "--longname value", "-s v" for all but flags
///     - values are parsed with `iostream <<` operators
///     - for general use `opt = parse_opt<type>()`
///     - for boolean flags is `parse_flag()`
///     - for enums use `parse_opte()`
/// 3. for each unnamed argument, parse it calling the functions parse_arg()
///     - names are only used for help
///     - supports types as above
///     - for general use `arg = parse_arg<type>()`
///     - to parse all remaining values use `args = parse_arga<type>(...)`
/// 4. end cmdline parsing with `check_parsing()` to check for unsued values,
///    missing arguments
/// 5. to check for error use `should_exit()` and to print the message use
///    `get_message()`
/// 6. since arguments are parsed immediately, one can easily implement
///    subcommands by just branching the command line code based on a read
///    argument without any need for complex syntax
///
///
/// ### Logging
///
/// We include a simple logger for keeping track of application progress.
/// The logger optionally outputs to console or disk, and support different
/// levels of verbosity
///
/// 1. create a `logger`
/// 2. add more streams with `add_console_stream()` or `add_file_stream()`
/// 3. write log messages with `log_msg()` and its variants.
/// 4. you can also use a global default logger with the free functions
///    `log_XXX()`
///
///
/// ### Concurrent Execution
///
/// Since the C++14 standard does not yet include an easy way to run large
/// tasks in parallel, we include a thread pool from the LLVM codebase.
///
/// 1. either create a `thread_pool` or use the global one
/// 2. run tasks in parallel `parallel_for()`
/// 3. run tasks asynchronously `async()`
///
///
/// ### Timer
///
/// To time application code we use a wrapper over the `std::chrono`
/// functionality to avoid its verbosity.
///
/// 1. create a `timer`
/// 2. start and stop the clock with `start()` and `stop()`
/// 3. get time with `elapsed_time()`
///
///
/// ### Simple scene
///
/// We support a simple scene model used to quickly write demos that lets you
/// load/save Wavefront OBJ and Khronos glTF and perform several simple scene
/// manipulation including ray-scene intersection and closest point queries.
///
/// The geometry model is comprised of a set of shapes, which are indexed
/// collections of points, lines, triangles and quads. Each shape may contain
/// only one element type. Shapes are organized into a scene by creating shape
/// instances, each its own transform. Materials are specified like in glTF and
/// include emission, base-metallic and diffuse-specular parametrization,
/// normal, occlusion and displacement mapping. Finally, the scene containes
/// caemras and environement maps. Quad support in shapes is experimental and
/// mostly supported for loading and saving.
///
/// For low-level access to OBJ/glTF formats, you are best accssing the formats
/// directly with Yocto/Obj and Yocto/glTF. This components provides a
/// simplified high-level access to each format which is sufficient for most
/// applications and tuned for quick creating viewers, renderers and simulators.
///
/// 1. load a scene with `load_scene()` and save it with `save_scene()`.
/// 2. add missing data with `add_elements()`
/// 3. use `compute_bounds()` to compute element bounds
/// 4. can merge scene together with `merge_into()`
///
/// Ray-intersection and closet-point routines supporting points,
/// lines and triangles accelerated by a two-level bounding volume
/// hierarchy (BVH). Quad support is experimental.
///
/// 1. build the bvh with `build_bvh()`
/// 2. perform ray-interseciton tests with `intersect_ray()`
///     - use early_exit=false if you want to know the closest hit point
///     - use early_exit=false if you only need to know whether there is a hit
///     - for points and lines, a radius is required
///     - for triangles, the radius is ignored
/// 2. perform point overlap tests with `overlap_point()` to check whether
///    a point overlaps with an element within a maximum distance
///     - use early_exit as above
///     - for all primitives, a radius is used if defined, but should
///       be very small compared to the size of the primitive since the radius
///       overlap is approximate
/// 3. perform instance overlap queries with `overlap_instance_bounds()`
/// 4. use `refit_bvh()` to recompute the bvh bounds if transforms or vertices
///    are changed (you should rebuild the bvh for large changes)
///
/// Notes: Quads are internally handled as a pair of two triangles v0,v1,v3 and
/// v2,v3,v1, with the u/v coordinates of the second triangle corrected as 1-u
/// and 1-v to produce a quad parametrization where u and v go from 0 to 1. This
/// is equivalent to Intel's Embree.
///
///
/// ### Pathtracing
///
/// We supply a path tracer implementation with support for textured mesh
/// lights, GGX/Phong materials, environment mapping. The interface supports
/// progressive parallel execution. The path tracer takes as input a scene
/// and update pixels in image with traced samples. We use a straightfoward
/// path tracer with MIS and also a few simpler shaders for debugging or
/// quick image generation.
///
/// Materials are represented as sums of an emission term, a diffuse term and
/// a specular microfacet term (GGX or Phong). Only opaque for now. We pick
/// a proper material type for each shape element type (points, lines,
/// triangles).
///
/// Lights are defined as any shape with a material emission term. Additionally
/// one can also add environment maps. But even if you can, you might want to
/// add a large triangle mesh with inward normals instead. The latter is more
/// general (you can even more an arbitrary shape sun). For now only the first
/// env is used.
///
/// 1. build the ray-tracing acceleration structure with `build_bvh()`
/// 2. prepare lights for rendering `update_lights()`
/// 3. define rendering params with the `trace_params` structure
/// 4. render blocks of samples with `trace_block()`
///
/// The code can also run in fully asynchronous mode to preview images in a
/// window.
///
/// 1. build the ray-tracing acceleration structure with `build_bvh()`
/// 2. prepare lights for rendering `update_lights()`
/// 3. define rendering params with the `trace_params` structure
/// 4. initialize the prograssive rendering buffers
/// 5. start the progressive renderer with `trace_async_start()`
/// 7. stop the progressive renderer with `trace_async_stop()`
///
///
/// ### Wavefront OBJ
///
/// Wavefront OBJ/MTL loader and writer with support for points,
/// lines, triangles and general polygons and all materials properties.
/// Contains also a few extensions to easily create demos such as per-vertex
/// color and radius, cameras, environment maps and instances.
/// Can use either a low-level OBJ representation, from this files,
/// or a high level flattened representation included in Yocto/Scn.
///
/// Both in reading and writing, OBJ has no clear convention on the orientation
/// of textures Y axis. So in many cases textures appears flipped. To handle
/// that, use the option to flip textures coordinates on either saving or
/// loading. By default texture coordinates are flipped since this seems
/// the convention found on test cases collected on the web. The value Tr
/// has similar problems, since its relation to opacity is software specific.
/// Again we let the user chose the convension and set the default to the
/// one found on the web.
///
/// In the high level interface, shapes are indexed meshes and are described
/// by arrays of vertex indices for points/lines/triangles and arrays for vertex
/// positions, normals, texcoords, color and radius. The latter two as
/// extensions. Since OBJ is a complex formats that does not match well with
/// current GPU rendering / path tracing algorithms, we adopt a simplification
/// similar to other single file libraries:
/// 1. vertex indices are unique, as in OpenGL and al standard indexed triangle
///   meshes data structures, and not OBJ triplets; YOCTO_OBJ ensures that no
///   vertex dusplication happens thought for same triplets
/// 2. we split shapes on changes to groups and materials, instead of keeping
///   per-face group/material data; this makes the data usable right away in
///   a GPU viewer; this is not a major limitation if we accept the previous
///   point that already changes shapes topology.
///
/// 1. load a obj data with `load_obj()`; can load also textues
/// 2. look at the `obj_XXX` data structures for access to individual elements
/// 3. use obj back to disk with `save_obj()`; can also save textures
/// 4. use get_shape() to get a flattened shape version that contains only
///    triangles, lines or points
///
///
/// ### Khronos glTF
///
/// Khronos GLTF loader and writer for Khronos glTF format. Supports
/// all the glTF spec and the Khronos extensions. All parsing and writing code
/// is autogenerated form the schema. Supports glTF version 2.0 and the
/// following extensions: `KHR_binary_glTF` and `KHR_specular_glossiness`.
///
/// This component depends on `json.hpp` and, for image loading and saving,
/// it depends on `stb_image.h`, `stb_image_write.h`, `stb_image_resize.h` and
/// `tinyexr.h`. This feature can be disabled as before.
///
/// The library provides a low  level interface that is a direct
/// C++ translation of the glTF schemas and should be used if one wants
/// complete control over the fromat or an application wants to have their
/// own scene code added. A higher-level interface is provided by the scene
/// or by `yocto_gltf.h`.
///
/// glTF is a very complex file format and was designed mainly with untyped
/// languages in mind. We attempt to match the glTF low-level interface
/// to C++ as best as it can. Since the code is generated from the schema, we
/// follow glTF naming conventions and typing quite well. To simplify adoption
/// and keep the API relatively simple we use vector as arrays and use
/// pointers to reference to all glTF objects. While this makes it less effcient
/// than it might have been, glTF heavy use of optional values makes this
/// necessary. At the same time, we do not keep track of set/unset values
/// for basic types (int, float, bool) as a compromise for efficieny.
///
/// glTF uses integer indices to access objects.
/// While writing code ourselves we found that we add signiicant problems
/// since we would use an index to access the wriong type of scene objects.
/// For this reasons, we use an explit index `glTFid<T>` that can only access
/// an object of type T. Internally this is just the same old glTF index. But
/// this can used to access the scene data with `glTF::get<T>(index)`.
///
/// 1. load a glTF model with `load_gltf()`
/// 2. look at the `glTFXXX` data structures for access to individual elements
/// 3. save glTF back to disk with `save_gltf()`
///
///
/// ### OpenGL support
///
/// We include a set of utilities to draw on screen with OpenGL 3.3, manage
/// windows with GLFW and draw immediate-mode widgets with ImGui.
///
/// 1. texture and buffer objects with `gl_texture` and `gl_buffer`
///     - create textures/buffers with appropriate constructors
///     - check validity wiht `is_valid()`
///     - update textures/buffers with `update()` functions
///     - delete textures/buffers with `clear()`
///     - bind/unbind textures/buffers with `bind()`/`unbind()`
///     - draw elements with `gl_buffer::draw_elems()`
/// 2. program objects with `gl_program`
///     - program creation with constructor
///     - check validity wiht `is_valid()`
///     - delete with `clear()`
///     - uniforms with `set_program_uniform()`
///     - vertex attrib with `set_program_vertattr()`
///     - draw elements with `gl_buffer::draw_elems()`
/// 3. image viewing with `gl_stdimage_program`, with support for tone mapping.
/// 4. draw surfaces and hair with GGX/Kayjia-Kay with `gl_stdsurface_program`
///     - initialize the program with constructor
///     - check validity wiht `is_valid()`
///     - start/end each frame with `begin_frame()`, `end_frame()`
///     - define lights with `set_lights()`
///     - start/end each shape with `begin_shape()`, `end_shape()`
///     - define material Parameters with `set_material()`
///     - define vertices with `set_vert()`
///     - draw elements with `draw_elems()`
/// 5. also includes other utlities for quick OpenGL
/// 6. GLFW window with `gl_window`
///     - create with constructor
///     - delete with `clear()`
///     - set callbacks with `set_callbacks()`
///     - includes carious utiliies to query window, mouse and keyboard
/// 7. immediate mode widgets
///     - init with `init_widget()`
///     - use the various widget calls to draw the widget and handle events
///
///
/// ## History
///
/// - v 0.42: import std constainers and common functions
/// - v 0.41: added path tracer
/// - v 0.40: added scene support
/// - v 0.39: added Kronos glTF support
/// - v 0.38: added OpenGL utilities
/// - v 0.37: added simple logger, timer and thread pool
/// - v 0.36: added immediate mode command line parser
/// - v 0.35: added path manipulation
/// - v 0.34: added file loading and saving
/// - v 0.33: added stream support to basic types
/// - v 0.32: image load, save and resize
/// - v 0.31: use internal namespaces
/// - v 0.30: updated pcg32 generator
/// - v 0.29: bezier curves and better barycentric interpolation
/// - v 0.28: experimental support for quad intersect and sampling
/// - v 0.27: BVH for intersection and closest point queries
/// - v 0.26: fix small compiler bugs
/// - v 0.25: geodesic sphere and surface faceting
/// - v 0.24: tesselation function
/// - v 0.23: more camera navigation
/// - v 0.22: removed image lookup with arbitrary channels
/// - v 0.21: added more functions
/// - v 0.20: remove unused bbox overlap tests
/// - v 0.19: remove indexing from specializations
/// - v 0.18: bump to normal mapping convertion
/// - v 0.17: added example image geneation
/// - v 0.16: sampling
/// - v 0.15: enable specialization always
/// - v 0.14: move timer to Yocto/Utils
/// - v 0.13: more shape functions
/// - v 0.12: documentation update
/// - v 0.11: added more matrix and quaternion operations
/// - v 0.10: specialize some type and functions
/// - v 0.9: bbox containment tests
/// - v 0.8: remove std:array as base class for better control
/// - v 0.7: doxygen comments
/// - v 0.6: uniformed internal names
/// - v 0.5: simplification of constructors, raname bbox -> bbox
/// - v 0.4: overall type simplification
/// - v 0.3: internal C++ refactoring
/// - v 0.2: use of STL containers; removal of yocto containers
/// - v 0.1: C++ only implementation
/// - v 0.0: initial release in C99
///
namespace ygl {}

//
// LICENSE:
//
// Copyright (c) 2016 -- 2017 Fabio Pellacini
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//
// LICENSE OF INCLUDED SOFTWARE for Pcg random number generator
//
// This code also includes a small exerpt from http://www.pcg-random.org/
// licensed as follows
// *Really* minimal PCG32 code / (c) 2014 M.E. O'Neill / pcg-random.org
// Licensed under Apache License 2.0 (NO WARRANTY, etc. see website)
//
//
// LICENSE OF INCLUDED SOFTWARE for ThreadPool code from LLVM code base
//
// Copyright (c) 2003-2016 University of Illinois at Urbana-Champaign.
// All rights reserved.
//
// Developed by:
//
//     LLVM Team
//
//     University of Illinois at Urbana-Champaign
//
//     http://llvm.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// with the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
//     * Redistributions of source code must retain the above copyright notice,
//       this list of conditions and the following disclaimers.
//
//     * Redistributions in binary form must reproduce the above copyright
//     notice,
//       this list of conditions and the following disclaimers in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the names of the LLVM Team, University of Illinois at
//       Urbana-Champaign, nor the names of its contributors may be used to
//       endorse or promote products derived from this Software without specific
//       prior written permission.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
// CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS WITH
// THE SOFTWARE.
//
//
// LICENSE OF INCLUDED CODE FOR BASE64 (base64.h, base64.cpp)
//
// Copyright (C) 2004-2008 René Nyffenegger
//
// This source code is provided 'as-is', without any express or implied
// warranty. In no event will the author be held liable for any damages
// arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
//
// 1. The origin of this source code must not be misrepresented; you must not
// claim that you wrote the original source code. If you use this source code
// in a product, an acknowledgment in the product documentation would be
// appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such, and must not be
// misrepresented as being the original source code.
//
// 3. This notice may not be removed or altered from any source distribution.
//
// René Nyffenegger rene.nyffenegger@adp-gmbh.ch

#ifndef _YMATH_H_
#define _YMATH_H_

// -----------------------------------------------------------------------------
// COMPILATION OPTIONS
// -----------------------------------------------------------------------------

// enable image io
#ifndef YGL_IMAGEIO
#define YGL_IMAGEIO 1
#endif

// inlude image io implementation
#ifndef YGL_IMAGEIO_IMPLEMENTATION
#define YGL_IMAGEIO_IMPLEMENTATION 0
#endif

// enable scene io
#ifndef YGL_SCENEIO
#define YGL_SCENEIO 1
#endif

// enable OpenGL
#ifndef YGL_OPENGL
#define YGL_OPENGL 0
#endif

// enable GLFW
#ifndef YGL_GLFW
#define YGL_GLFW 0
#endif

// enable ImGui
#ifndef YGL_IMGUI
#define YGL_IMGUI 0
#endif

// inlude image io implementation
#ifndef YGL_IMGUI_IMPLEMENTATION
#define YGL_IMGUI_IMPLEMENTATION 0
#endif

// enable scene ui support
#ifndef YGL_SCENEUI
#define YGL_SCENEUI 0
#endif

// -----------------------------------------------------------------------------
// INCLUDES
// -----------------------------------------------------------------------------

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <functional>
#include <future>
#include <initializer_list>
#include <limits>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// stb image
#if YGL_IMAGEIO
#include "stb_image.h"
#include "stb_image_resize.h"
#include "stb_image_write.h"
#include "tinyexr.h"
#endif

// json
#if YGL_SCENEIO
#include "json.hpp"
#endif

#if YGL_OPENGL

#ifdef __APPLE__
#include <OpenGL/gl3.h>
#else
#include <GL/glew.h>
#endif

#if YGL_GLFW
#define GLFW_INCLUDE_GLCOREARB
#include <GLFW/glfw3.h>
#if YGL_IMGUI
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw_gl3.h"
#endif
#endif

#endif

// HACK to avoid compilation with MSVC2015 and C++11 without dirtying code
#if defined(_WIN32) || __cplusplus < 201402L
#define constexpr
#endif

// Compilation option
#define YGL_FAST_RANDFLOAT 1

// -----------------------------------------------------------------------------
// IMPORTED MATH FUNCTIONS
// -----------------------------------------------------------------------------
namespace ygl {

/// sqrt
using std::sqrt;
/// pow
using std::pow;
/// pow
using std::exp;
/// log
using std::log;
/// log10
using std::log10;
/// sin
using std::sin;
/// cos
using std::cos;
/// tan
using std::tan;
/// asin
using std::asin;
/// acos
using std::acos;
/// atan
using std::atan;
/// atan2
using std::atan2;
/// absolute value
using std::abs;
/// floating point absolute value
using std::fabs;
/// floor
using std::floor;
/// ceil
using std::ceil;
/// round
using std::round;
/// isfinate
using std::isfinite;

}  // namespace ygl

// -----------------------------------------------------------------------------
// IMPORTED CONTAINERS AND RELATED FUNCTIONS
// -----------------------------------------------------------------------------
namespace ygl {

/// string
using std::string;
/// vector
using std::vector;
/// map
using std::map;
/// unordered map
using std::unordered_map;
/// unordered set
using std::unordered_set;
/// pair
using std::pair;
/// tuple
using std::tuple;
/// unique pointer
using std::unique_ptr;
/// function
using std::function;
/// string literals
using namespace std::string_literals;
/// numeric limits
using std::numeric_limits;
/// initializer list
using std::initializer_list;
/// output stream
using std::ostream;
/// input stream
using std::istream;
/// string stream
using std::stringstream;
/// file stream
using std::fstream;
/// runtime error
using std::runtime_error;
/// exception
using std::exception;
/// ios base
using std::ios_base;
/// find algorithms
using std::find;
/// swap algorithms
using std::swap;

// makes literals available
using namespace std::literals;

}  // namespace ygl

// -----------------------------------------------------------------------------
// BASIC TYPEDEFS, MATH CONSTANTS AND FUNCTIONS
// -----------------------------------------------------------------------------
namespace ygl {

/// convenient typedef for bytes
using byte = unsigned char;

/// convenient typedef for bytes
using uint = unsigned int;

/// pi (float)
constexpr const auto pif = 3.14159265f;
/// pi (double)
constexpr const auto pi = 3.1415926535897932384626433832795;

/// shortcat for float max value
constexpr const auto flt_max = numeric_limits<float>::max();
/// shortcat for float min value
constexpr const auto flt_min = numeric_limits<float>::lowest();
/// shortcat for int max value
constexpr const auto int_max = numeric_limits<int>::max();
/// shortcat for int min value
constexpr const auto int_min = numeric_limits<int>::min();

/// Safe minimum value.
template <typename T>
constexpr inline T min(T x, T y) {
    return (x < y) ? x : y;
}

/// Safe maximum value.
template <typename T>
constexpr inline T max(T x, T y) {
    return (x > y) ? x : y;
}

/// Clamp a value between a minimum and a maximum.
template <typename T>
constexpr inline T clamp(T x, T min_, T max_) {
    return min(max(x, min_), max_);
}

/// Linear interpolation.
template <typename T>
constexpr inline T lerp(T a, T b, T t) {
    return a * (1 - t) + b * t;
}

/// bilinear interpolation
template <typename T>
constexpr inline T bilerp(T aa, T ba, T ab, T bb, T s, T t) {
    return aa * (1 - s) * (1 - t) + ba * s * (1 - t) + ab * (1 - s) * t +
           bb * s * t;
}

/// Integer power of two
constexpr inline int pow2(int x) { return 1 << x; }

/// Safe float to byte conversion
constexpr inline byte float_to_byte(float x) {
    return (byte)max(0, min(int(x * 256), 255));
}

/// Safe byte to float conversion
constexpr inline float byte_to_float(byte x) { return (float)x / 255.0f; }

}  // namespace ygl

// -----------------------------------------------------------------------------
// VECTORS
// -----------------------------------------------------------------------------
namespace ygl {

/// Vector of elements of compile time dimension with default initializer.
template <typename T, int N>
struct vec {
    /// default constructor
    constexpr vec() {
        for (auto i = 0; i < N; i++) v[i] = 0;
    }
    /// element constructor
    constexpr explicit vec(T vv) {
        for (auto i = 0; i < N; i++) v[i] = vv;
    }
    /// list constructor
    constexpr vec(const initializer_list<T>& vv) {
        assert(N == vv.size());
        auto i = 0;
        for (auto&& e : vv) v[i++] = e;
    }

    /// element access
    constexpr T& operator[](int i) { return v[i]; }
    /// element access
    constexpr const T& operator[](int i) const { return v[i]; }

    /// data access
    constexpr T* data() { return v; }
    /// data access
    constexpr const T* data() const { return v; }

    /// element data
    T v[N];
};

/// Specialization of vectors for 1 component and float coordinates.
template <typename T>
struct vec<T, 1> {
    /// size
    constexpr static const int N = 1;

    /// default constructor
    constexpr vec() : x{0} {}
    /// element constructor
    constexpr vec(T x) : x{x} {}

    /// element access
    constexpr T& operator[](int i) { return (&x)[i]; }
    /// element access
    constexpr const T& operator[](int i) const { return (&x)[i]; }

    /// data access
    constexpr T* data() { return &x; }
    /// data access
    constexpr const T* data() const { return &x; }

    /// element data
    T x;
};

/// Specialization of vectors for 2 components and float coordinates.
template <typename T>
struct vec<T, 2> {
    /// size
    constexpr static const int N = 2;

    /// default constructor
    constexpr vec() : x{0}, y{0} {}
    /// element constructor
    constexpr explicit vec(T vv) : x(vv), y(vv) {}
    /// element constructor
    constexpr vec(T x, T y) : x{x}, y{y} {}

    /// element access
    constexpr T& operator[](int i) { return (&x)[i]; }
    /// element access
    constexpr const T& operator[](int i) const { return (&x)[i]; }

    /// data access
    constexpr T* data() { return &x; }
    /// data access
    constexpr const T* data() const { return &x; }

    /// element data
    T x;
    /// element data
    T y;
};

/// Specialization of vectors for 3 components and float coordinates.
template <typename T>
struct vec<T, 3> {
    /// size
    constexpr static const int N = 3;

    /// default constructor
    constexpr vec() : x{0}, y{0}, z{0} {}
    /// element constructor
    constexpr explicit vec(T vv) : x(vv), y(vv), z(vv) {}
    /// element constructor
    constexpr vec(T x, T y, T z) : x{x}, y{y}, z{z} {}

    /// element access
    constexpr T& operator[](int i) { return (&x)[i]; }
    /// element access
    constexpr const T& operator[](int i) const { return (&x)[i]; }

    /// data access
    constexpr T* data() { return &x; }
    /// data access
    constexpr const T* data() const { return &x; }

    /// element data
    T x;
    /// element data
    T y;
    /// element data
    T z;
};

/// Specialization of vectors for 4 components and float coordinates.
template <typename T>
struct vec<T, 4> {
    /// size
    constexpr static const int N = 4;

    /// default constructor
    constexpr vec() : x{0}, y{0}, z{0}, w{0} {}
    /// element constructor
    constexpr explicit vec(T vv) : x(vv), y(vv), z(vv), w(vv) {}
    /// element constructor
    constexpr vec(T x, T y, T z, T w) : x{x}, y{y}, z{z}, w{w} {}
    /// constructor from smaller vector
    constexpr vec(const vec<T, 3>& xyz, T w)
        : x{xyz.x}, y{xyz.y}, z{xyz.z}, w{w} {}

    /// element access
    constexpr T& operator[](int i) { return (&x)[i]; }
    /// element access
    constexpr const T& operator[](int i) const { return (&x)[i]; }

    /// data access
    constexpr T* data() { return &x; }
    /// data access
    constexpr const T* data() const { return &x; }

    /// access xyz components
    constexpr vec<T, 3>& xyz() { return *(vec<T, 3>*)&x; }
    /// access xyz components
    constexpr const vec<T, 3>& xyz() const { return *(vec<T, 3>*)&x; }

    /// element data
    T x;
    /// element data
    T y;
    /// element data
    T z;
    /// element data
    T w;
};

/// 1-dimensional float vector
using vec1f = vec<float, 1>;
/// 2-dimensional float vector
using vec2f = vec<float, 2>;
/// 3-dimensional float vector
using vec3f = vec<float, 3>;
/// 4-dimensional float vector
using vec4f = vec<float, 4>;

/// 1-dimensional int vector
using vec1i = vec<int, 1>;
/// 2-dimensional int vector
using vec2i = vec<int, 2>;
/// 3-dimensional int vector
using vec3i = vec<int, 3>;
/// 4-dimensional int vector
using vec4i = vec<int, 4>;

/// 1-dimensional byte vector
using vec1b = vec<byte, 1>;
/// 2-dimensional byte vector
using vec2b = vec<byte, 2>;
/// 3-dimensional byte vector
using vec3b = vec<byte, 3>;
/// 4-dimensional byte vector
using vec4b = vec<byte, 4>;

/// 1-dimensional float zero vector
const auto zero1f = vec<float, 1>();
/// 2-dimensional float zero vector
const auto zero2f = vec<float, 2>();
/// 3-dimensional float zero vector
const auto zero3f = vec<float, 3>();
/// 4-dimensional float zero vector
const auto zero4f = vec<float, 4>();

/// 1-dimensional float one vector
const auto one1f = vec<float, 1>(1);
/// 2-dimensional float one vector
const auto one2f = vec<float, 2>(1);
/// 3-dimensional float one vector
const auto one3f = vec<float, 3>(1);
/// 4-dimensional float one vector
const auto one4f = vec<float, 4>(1);

/// 1-dimensional int zero vector
const auto zero1i = vec<int, 1>();
/// 2-dimensional int zero vector
const auto zero2i = vec<int, 2>();
/// 3-dimensional int zero vector
const auto zero3i = vec<int, 3>();
/// 4-dimensional int zero vector
const auto zero4i = vec<int, 4>();

/// 1-dimensional byte zero vector
const auto zero1b = vec<byte, 1>();
/// 2-dimensional byte zero vector
const auto zero2b = vec<byte, 2>();
/// 3-dimensional byte zero vector
const auto zero3b = vec<byte, 3>();
/// 4-dimensional byte zero vector
const auto zero4b = vec<byte, 4>();

/// iteration support
template <typename T, int N>
constexpr inline T* begin(vec<T, N>& a) {
    return &a[0];
}

/// iteration support
template <typename T, int N>
constexpr inline const T* begin(const vec<T, N>& a) {
    return &a[0];
}

/// iteration support
template <typename T, int N>
constexpr inline T* end(vec<T, N>& a) {
    return &a[0] + N;
}

/// iteration support
template <typename T, int N>
constexpr inline const T* end(const vec<T, N>& a) {
    return &a[0] + N;
}

/// vector operator ==
template <typename T, int N>
constexpr inline bool operator==(const vec<T, N>& a, const vec<T, N>& b) {
    for (auto i = 0; i < N; i++)
        if (a[i] != b[i]) return false;
    return true;
}

/// vector operator !=
template <typename T, int N>
constexpr inline bool operator!=(const vec<T, N>& a, const vec<T, N>& b) {
    return !(a == b);
}

/// vector operator < (lexicographic order - useful for map)
template <typename T, int N>
constexpr inline bool operator<(const vec<T, N>& a, const vec<T, N>& b) {
    for (auto i = 0; i < N; i++) {
        if (a[i] < b[i]) return true;
        if (a[i] > b[i]) return false;
    }
    return false;
}

/// vector operator ==
template <>
constexpr inline bool operator==(const vec2f& a, const vec2f& b) {
    return a.x == b.x && a.y == b.y;
}

/// vector operator !=
template <>
constexpr inline bool operator!=(const vec2f& a, const vec2f& b) {
    return a.x != b.x || a.y != b.y;
}

/// vector operator ==
template <>
constexpr inline bool operator==(const vec3f& a, const vec3f& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

/// vector operator !=
template <>
constexpr inline bool operator!=(const vec3f& a, const vec3f& b) {
    return a.x != b.x || a.y != b.y || a.z != b.z;
}

/// vector operator ==
template <>
constexpr inline bool operator==(const vec4f& a, const vec4f& b) {
    return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w;
}

/// vector operator !=
template <>
constexpr inline bool operator!=(const vec4f& a, const vec4f& b) {
    return a.x != b.x || a.y != b.y || a.z != b.z || a.w != b.w;
}

/// vector operator +
template <typename T, int N>
constexpr inline vec<T, N> operator+(const vec<T, N>& a) {
    return a;
}

/// vector operator -
template <typename T, int N>
constexpr inline vec<T, N> operator-(const vec<T, N>& a) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = -a[i];
    return c;
}

/// vector operator +
template <typename T, int N>
constexpr inline vec<T, N> operator+(const vec<T, N>& a, const vec<T, N>& b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a[i] + b[i];
    return c;
}

/// vector operator -
template <typename T, int N>
constexpr inline vec<T, N> operator-(const vec<T, N>& a, const vec<T, N>& b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a[i] - b[i];
    return c;
}

/// vector operator +
template <typename T, int N>
constexpr inline vec<T, N> operator+(const vec<T, N>& a, const T b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a[i] + b;
    return c;
}

/// vector operator -
template <typename T, int N>
constexpr inline vec<T, N> operator-(const vec<T, N>& a, const T b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a[i] - b;
    return c;
}

/// vector operator +
template <typename T, int N>
constexpr inline vec<T, N> operator+(T a, const vec<T, N>& b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a + b[i];
    return c;
}

/// vector operator -
template <typename T, int N>
constexpr inline vec<T, N> operator-(T a, const vec<T, N>& b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a - b[i];
    return c;
}

/// vector operator *
template <typename T, int N>
constexpr inline vec<T, N> operator*(const vec<T, N>& a, const vec<T, N>& b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a[i] * b[i];
    return c;
}

/// vector operator *
template <typename T, int N>
constexpr inline vec<T, N> operator*(const vec<T, N>& a, const T b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a[i] * b;
    return c;
}

/// vector operator *
template <typename T, int N>
constexpr inline vec<T, N> operator*(const T a, const vec<T, N>& b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a * b[i];
    return c;
}

/// vector operator /
template <typename T, int N>
constexpr inline vec<T, N> operator/(const vec<T, N>& a, const vec<T, N>& b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a[i] / b[i];
    return c;
}

/// vector operator /
template <typename T, int N>
constexpr inline vec<T, N> operator/(const vec<T, N>& a, const T b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a[i] / b;
    return c;
}

/// vector operator /
template <typename T, int N>
constexpr inline vec<T, N> operator/(const T a, const vec<T, N>& b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = a / b[i];
    return c;
}

/// vector operator +
template <>
constexpr inline vec2f operator+(const vec2f& a) {
    return a;
}

/// vector operator -
template <>
constexpr inline vec2f operator-(const vec2f& a) {
    return {-a.x, -a.y};
}

/// vector operator +
template <>
constexpr inline vec2f operator+(const vec2f& a, const vec2f& b) {
    return {a.x + b.x, a.y + b.y};
}

/// vector operator -
template <>
constexpr inline vec2f operator-(const vec2f& a, const vec2f& b) {
    return {a.x - b.x, a.y - b.y};
}

/// vector operator *
template <>
constexpr inline vec2f operator*(const vec2f& a, const vec2f& b) {
    return {a.x * b.x, a.y * b.y};
}

/// vector operator *
template <>
constexpr inline vec2f operator*(const vec2f& a, const float b) {
    return {a.x * b, a.y * b};
}

/// vector operator *
template <>
constexpr inline vec2f operator*(const float a, const vec2f& b) {
    return {a * b.x, a * b.y};
}

/// vector operator /
template <>
constexpr inline vec2f operator/(const vec2f& a, const vec2f& b) {
    return {a.x / b.x, a.y / b.y};
}

/// vector operator /
template <>
constexpr inline vec2f operator/(const vec2f& a, const float b) {
    return {a.x / b, a.y / b};
}

/// vector operator /
template <>
constexpr inline vec2f operator/(const float a, const vec2f& b) {
    return {a / b.x, a / b.y};
}

/// vector operator +
template <>
constexpr inline vec3f operator+(const vec3f& a) {
    return a;
}

/// vector operator -
template <>
constexpr inline vec3f operator-(const vec3f& a) {
    return {-a.x, -a.y, -a.z};
}

/// vector operator +
template <>
constexpr inline vec3f operator+(const vec3f& a, const vec3f& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

/// vector operator -
template <>
constexpr inline vec3f operator-(const vec3f& a, const vec3f& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

/// vector operator *
template <>
constexpr inline vec3f operator*(const vec3f& a, const vec3f& b) {
    return {a.x * b.x, a.y * b.y, a.z * b.z};
}

/// vector operator *
template <>
constexpr inline vec3f operator*(const vec3f& a, const float b) {
    return {a.x * b, a.y * b, a.z * b};
}

/// vector operator *
template <>
constexpr inline vec3f operator*(const float a, const vec3f& b) {
    return {a * b.x, a * b.y, a * b.z};
}

/// vector operator /
template <>
constexpr inline vec3f operator/(const vec3f& a, const vec3f& b) {
    return {a.x / b.x, a.y / b.y, a.z / b.z};
}

/// vector operator /
template <>
constexpr inline vec3f operator/(const vec3f& a, const float b) {
    return {a.x / b, a.y / b, a.z / b};
}

/// vector operator /
template <>
constexpr inline vec3f operator/(const float a, const vec3f& b) {
    return {a / b.x, a / b.y, a / b.z};
}

/// vector operator +
template <>
constexpr inline vec4f operator+(const vec4f& a) {
    return a;
}

/// vector operator -
template <>
constexpr inline vec4f operator-(const vec4f& a) {
    return {-a.x, -a.y, -a.z, -a.w};
}

/// vector operator +
template <>
constexpr inline vec4f operator+(const vec4f& a, const vec4f& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
}

/// vector operator -
template <>
constexpr inline vec4f operator-(const vec4f& a, const vec4f& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w};
}

/// vector operator *
template <>
constexpr inline vec4f operator*(const vec4f& a, const vec4f& b) {
    return {a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w};
}

/// vector operator *
template <>
constexpr inline vec4f operator*(const vec4f& a, const float b) {
    return {a.x * b, a.y * b, a.z * b, a.w * b};
}

/// vector operator *
template <>
constexpr inline vec4f operator*(const float a, const vec4f& b) {
    return {a * b.x, a * b.y, a * b.z, a * b.w};
}

/// vector operator /
template <>
constexpr inline vec4f operator/(const vec4f& a, const vec4f& b) {
    return {a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w};
}

/// vector operator /
template <>
constexpr inline vec4f operator/(const vec4f& a, const float b) {
    return {a.x / b, a.y / b, a.z / b, a.w / b};
}

/// vector operator /
template <>
constexpr inline vec4f operator/(const float a, const vec4f& b) {
    return {a / b.x, a / b.y, a / b.z, a / b.w};
}

/// vector operator +=
template <typename T, int N>
constexpr inline vec<T, N>& operator+=(vec<T, N>& a, const vec<T, N>& b) {
    return a = a + b;
}

/// vector operator -=
template <typename T, int N>
constexpr inline vec<T, N>& operator-=(vec<T, N>& a, const vec<T, N>& b) {
    return a = a - b;
}

/// vector operator *=
template <typename T, int N>
constexpr inline vec<T, N>& operator*=(vec<T, N>& a, const vec<T, N>& b) {
    return a = a * b;
}

/// vector operator *=
template <typename T, int N>
constexpr inline vec<T, N>& operator*=(vec<T, N>& a, const T b) {
    return a = a * b;
}

/// vector operator /=
template <typename T, int N>
constexpr inline vec<T, N>& operator/=(vec<T, N>& a, const vec<T, N>& b) {
    return a = a / b;
}

/// vector operator /=
template <typename T, int N>
constexpr inline vec<T, N>& operator/=(vec<T, N>& a, const T b) {
    return a = a / b;
}

/// vector dot product
template <typename T, int N>
constexpr inline T dot(const vec<T, N>& a, const vec<T, N>& b) {
    auto c = T(0);
    for (auto i = 0; i < N; i++) c += a[i] * b[i];
    return c;
}

/// vector cross product (2d)
template <typename T>
constexpr inline T cross(const vec<T, 2>& a, const vec<T, 2>& b) {
    return a.x * b.y - a.y * b.x;
}

/// vector cross product (3d)
template <typename T>
constexpr inline vec<T, 3> cross(const vec<T, 3>& a, const vec<T, 3>& b) {
    return {
        a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

/// vector dot product
template <>
constexpr inline float dot(const vec2f& a, const vec2f& b) {
    return a.x * b.x + a.y * b.y;
}

/// vector dot product
template <>
constexpr inline float dot(const vec3f& a, const vec3f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// vector dot product
template <>
constexpr inline float dot(const vec4f& a, const vec4f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

/// vector cross product (2d)
template <>
constexpr inline float cross(const vec2f& a, const vec2f& b) {
    return a.x * b.y - a.y * b.x;
}

/// vector cross product (3d)
template <>
constexpr inline vec3f cross(const vec3f& a, const vec3f& b) {
    return {
        a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

/// vector length
template <typename T, int N>
constexpr inline T length(const vec<T, N>& a) {
    return sqrt(dot(a, a));
}

/// vector length squared
template <typename T, int N>
constexpr inline T lengthsqr(const vec<T, N>& a) {
    return dot(a, a);
}

/// vector normalization
template <typename T, int N>
constexpr inline vec<T, N> normalize(const vec<T, N>& a) {
    auto l = length(a);
    if (l == 0) return a;
    return a * (1 / l);
}

/// point distance
template <typename T, int N>
constexpr inline T dist(const vec<T, N>& a, const vec<T, N>& b) {
    return length(a - b);
}

/// point distance squared
template <typename T, int N>
constexpr inline T distsqr(const vec<T, N>& a, const vec<T, N>& b) {
    return lengthsqr(a - b);
}

/// angle between normalized vectors
template <typename T, int N>
constexpr inline T uangle(const vec<T, N>& a, const vec<T, N>& b) {
    auto d = dot(a, b);
    return d > 1 ? 0 : acos(d < -1 ? -1 : d);
}

/// angle between vectors
template <typename T, int N>
constexpr inline T angle(const vec<T, N>& a, const vec<T, N>& b) {
    return uangle(normalize(a), normalize(b));
}

/// vector linear interpolation
template <typename T, int N>
constexpr inline vec<T, N> lerp(const vec<T, N>& a, const vec<T, N>& b, T t) {
    return a * (1 - t) + b * t;
}

/// vector bilinear interpolation
template <typename T, int N>
constexpr inline vec<T, N> bilerp(const vec<T, N>& aa, const vec<T, N>& ba,
    const vec<T, N>& ab, const vec<T, N>& bb, T s, T t) {
    return aa * (1 - s) * (1 - t) + ba * s * (1 - t) + ab * (1 - s) * t +
           bb * s * t;
}

/// vector normalized linear interpolation
template <typename T, int N>
constexpr inline vec<T, N> nlerp(const vec<T, N>& a, const vec<T, N>& b, T t) {
    return normalize(lerp(a, b, t));
}

/// vector spherical linear interpolation (vectors have to be normalized)
template <typename T, int N>
constexpr inline vec<T, N> slerp(const vec<T, N>& a, const vec<T, N>& b, T t) {
    auto th = uangle(a, b);
    return th == 0 ?
               a :
               a * (sin(th * (1 - t)) / sin(th)) + b * (sin(th * t) / sin(th));
}

/// orthogonal vector
// http://lolengine.net/blog/2013/09/21/picking-orthogonal-vector-combing-coconuts)
template <typename T>
constexpr inline vec<T, 3> orthogonal(const vec<T, 3>& v) {
    return abs(v.x) > abs(v.z) ? vec<T, 3>{-v.y, v.x, 0} :
                                 vec<T, 3>{0, -v.z, v.y};
}

/// orthonormalize two vectors
template <typename T>
constexpr inline vec<T, 3> orthonormalize(
    const vec<T, 3>& a, const vec<T, 3>& b) {
    return normalize(a - b * dot(a, b));
}

/// vector component-wise min
template <typename T, int N>
constexpr inline vec<T, N> min(
    const vec<T, N>& x, const vec<T, N>& a, const vec<T, N>& b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = min(x[i], a[i], b[i]);
    return c;
}

/// vector component-wise max
template <typename T, int N>
constexpr inline vec<T, N> max(
    const vec<T, N>& x, const vec<T, N>& a, const vec<T, N>& b) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = max(x[i], a[i], b[i]);
    return c;
}

/// vector component-wise clamp
template <typename T, int N>
constexpr inline vec<T, N> clamp(
    const vec<T, N>& x, const T& min, const T& max) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = clamp(x[i], min, max);
    return c;
}

/// vector component-wise clamp
template <typename T, int N>
constexpr inline vec<T, N> clamp(
    const vec<T, N>& x, const vec<T, N>& min, const vec<T, N>& max) {
    vec<T, N> c;
    for (auto i = 0; i < N; i++) c[i] = clamp(x[i], min[i], max[i]);
    return c;
}

/// clamp the length of a vector
template <typename T, int N, typename T1>
constexpr inline vec<T, N> clamplen(const vec<T, N> x, T1 max) {
    auto l = length(x);
    return (l > (T)max) ? x * (T)max / l : x;
}

/// index of the min vector element
template <typename T, int N>
constexpr inline int min_element_idx(const vec<T, N>& a) {
    auto v = numeric_limits<T>::max();
    auto pos = -1;
    for (auto i = 0; i < N; i++) {
        if (v > a[i]) {
            v = a[i];
            pos = i;
        }
    }
    return pos;
}

/// index of the max vector element
template <typename T, int N>
constexpr inline int max_element_idx(const vec<T, N>& a) {
    auto v = -numeric_limits<T>::max();
    auto pos = -1;
    for (auto i = 0; i < N; i++) {
        if (v < a[i]) {
            v = a[i];
            pos = i;
        }
    }
    return pos;
}

/// index of the min vector element
template <typename T, int N>
constexpr inline T min_element_val(const vec<T, N>& a) {
    auto v = numeric_limits<T>::max();
    for (auto i = 0; i < N; i++) {
        if (v > a[i]) v = a[i];
    }
    return v;
}

/// index of the max vector element
template <typename T, int N>
constexpr inline T max_element_val(const vec<T, N>& a) {
    auto v = -numeric_limits<T>::max();
    for (auto i = 0; i < N; i++) {
        if (v < a[i]) v = a[i];
    }
    return v;
}

/// Element-wise sqrt
template <typename T, int N>
constexpr inline vec<T, N> sqrt(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = sqrt(a[i]);
    return c;
}

/// Element-wise pow
template <typename T, int N>
constexpr inline vec<T, N> pow(const vec<T, N>& a, const T b) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = pow(a[i], b);
    return c;
}

/// Element-wise exp
template <typename T, int N>
constexpr inline vec<T, N> exp(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = exp(a[i]);
    return c;
}

/// Element-wise log
template <typename T, int N>
constexpr inline vec<T, N> log(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = log(a[i]);
    return c;
}

/// Element-wise log10
template <typename T, int N>
constexpr inline vec<T, N> log10(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = log10(a[i]);
    return c;
}

/// Element-wise sin
template <typename T, int N>
constexpr inline vec<T, N> sin(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = sin(a[i]);
    return c;
}

/// Element-wise cos
template <typename T, int N>
constexpr inline vec<T, N> cos(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = cos(a[i]);
    return c;
}

/// Element-wise tan
template <typename T, int N>
constexpr inline vec<T, N> tan(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = tan(a[i]);
    return c;
}

/// Element-wise asin
template <typename T, int N>
constexpr inline vec<T, N> asin(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = asin(a[i]);
    return c;
}

/// Element-wise acos
template <typename T, int N>
constexpr inline vec<T, N> acos(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = acos(a[i]);
    return c;
}

/// Element-wise atan
template <typename T, int N>
constexpr inline vec<T, N> atan(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = atan(a[i]);
    return c;
}

/// Element-wise abs
template <typename T, int N>
constexpr inline vec<T, N> abs(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = abs(a[i]);
    return c;
}

/// Element-wise floor
template <typename T, int N>
constexpr inline vec<T, N> floor(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = floor(a[i]);
    return c;
}

/// Element-wise ceil
template <typename T, int N>
constexpr inline vec<T, N> ceil(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = ceil(a[i]);
    return c;
}

/// Element-wise round
template <typename T, int N>
constexpr inline vec<T, N> round(const vec<T, N>& a) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = round(a[i]);
    return c;
}

/// Element-wise round
template <typename T, int N>
constexpr inline vec<T, N> atan2(const vec<T, N>& a, const vec<T, N>& b) {
    auto c = vec<T, N>();
    for (auto i = 0; i < N; i++) c[i] = atan2(a[i], b[i]);
    return c;
}

/// Check if finite
template <typename T, int N>
constexpr inline bool isfinite(const vec<T, N>& a) {
    for (auto i = 0; i < N; i++)
        if (!isfinite(a[i])) return false;
    return true;
}

/// Element-wise conversion
template <int N>
constexpr inline vec<byte, N> float_to_byte(const vec<float, N>& a) {
    auto c = vec<byte, N>();
    for (auto i = 0; i < N; i++) c[i] = float_to_byte(a[i]);
    return c;
}

/// Element-wise conversion
template <int N>
constexpr inline vec<float, N> byte_to_float(const vec<byte, N>& a) {
    auto c = vec<float, N>();
    for (auto i = 0; i < N; i++) c[i] = byte_to_float(a[i]);
    return c;
}

/// stream write
template <typename T, int N>
inline ostream& operator<<(ostream& os, const vec<T, N>& a) {
    for (auto i = 0; i < N; i++) {
        if (i) os << ' ';
        os << a[i];
    }
    return os;
}

/// stream read
template <typename T, int N>
inline istream& operator>>(istream& is, vec<T, N>& a) {
    for (auto i = 0; i < N; i++) is >> a[i];
    return is;
}

}  // namespace ygl

namespace std {
/// Hash functor for vec<T,N> for use with unordered_map
template <typename T, int N>
struct hash<ygl::vec<T, N>> {
    // from boost::hash_combine
    constexpr static size_t hash_combine(size_t h, size_t h1) {
        h ^= h1 + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
    constexpr size_t operator()(const ygl::vec<T, N>& v) const {
        auto vh = hash<T>();
        auto h = (size_t)0;
        for (auto i = 0; i < N; i++) h = hash_combine(h, vh(v[i]));
        return h;
    }
};
}  // namespace std

// -----------------------------------------------------------------------------
// SMALL GROWABLE VECTORS
// -----------------------------------------------------------------------------
namespace ygl {

/// Vector of elements of compile time dimension with default initializer.
template <typename T, int N>
struct svec {
    /// default constructor
    constexpr svec() {
        for (auto i = 0; i < N; i++) v[i] = 0;
        num = 0;
    }
    /// element constructor
    constexpr svec(int n, T vv) {
        num = n;
        for (auto i = 0; i < num; i++) v[i] = vv;
        for (auto i = num; i < N; i++) v[i] = 0;
    }
    /// list constructor
    constexpr svec(const initializer_list<T>& vv) {
        assert(N >= vv.size());
        num = vv.size();
        auto i = 0;
        for (auto&& e : vv) v[i++] = e;
    }

    /// empty
    bool empty() const { return !num; }
    /// vector size
    int size() const { return num; }

    /// add an element at the end of the vector
    void push_back(T vv) { v[num++] = vv; }

    /// element access
    constexpr T& operator[](int i) { return v[i]; }
    /// element access
    constexpr const T& operator[](int i) const { return v[i]; }

    /// data access
    constexpr T* data() { return v; }
    /// data access
    constexpr const T* data() const { return v; }

    /// element data
    T v[N];
    int num = 0;
};

/// 7-dimensional growable float vector
using svec7f = svec<float, 7>;
/// 7-dimensional growable int vector
using svec7i = svec<int, 7>;
/// 15-dimensional growable float vector
using svec15f = svec<float, 15>;
/// 15-dimensional growable int vector
using svec15i = svec<int, 15>;

/// 7-dimensional growable empty float vector
using empty_svec7f = svec<float, 7>();
/// 7-dimensional growable empty int vector
using empty_svec7i = svec<int, 7>();
/// 15-dimensional growable empty float vector
using empty_svec15f = svec<float, 15>();
/// 15-dimensional growable empty int vector
using empty_svec15i = svec<int, 15>();

/// iteration support
template <typename T, int N>
constexpr inline T* begin(svec<T, N>& a) {
    return &a[0];
}

/// iteration support
template <typename T, int N>
constexpr inline const T* begin(const svec<T, N>& a) {
    return &a[0];
}

/// iteration support
template <typename T, int N>
constexpr inline T* end(svec<T, N>& a) {
    return &a[0] + a.size();
}

/// iteration support
template <typename T, int N>
constexpr inline const T* end(const svec<T, N>& a) {
    return &a[0] + a.size();
}

/// vector operator ==
template <typename T, int N>
constexpr inline bool operator==(const svec<T, N>& a, const svec<T, N>& b) {
    if (a.size() != b.size()) return false;
    for (auto i = 0; i < a.size(); i++)
        if (a[i] != b[i]) return false;
    return true;
}

/// vector operator !=
template <typename T, int N>
constexpr inline bool operator!=(const svec<T, N>& a, const svec<T, N>& b) {
    return !(a == b);
}

/// vector operator < (lexicographic order - useful for map)
template <typename T, int N>
constexpr inline bool operator<(const svec<T, N>& a, const svec<T, N>& b) {
    for (auto i = 0; i < min(a.size(), b.size()); i++) {
        if (a[i] < b[i]) return true;
        if (a[i] > b[i]) return false;
    }
    if (a.size() < b.size()) return true;
    return false;
}

}  // namespace ygl

namespace std {
/// Hash functor for vec<T,N> for use with unordered_map
template <typename T, int N>
struct hash<ygl::svec<T, N>> {
    // from boost::hash_combine
    constexpr static size_t hash_combine(size_t h, size_t h1) {
        h ^= h1 + 0x9e3779b9 + (h << 6) + (h >> 2);
        return h;
    }
    constexpr size_t operator()(const ygl::vec<T, N>& v) const {
        auto vh = hash<T>();
        auto h = (size_t)0;
        h = hash_combine(h, vh(v.size()));
        for (auto i = 0; i < v.size(); i++) h = hash_combine(h, vh(v[i]));
        return h;
    }
};
}  // namespace std

// -----------------------------------------------------------------------------
// MATRICES
// -----------------------------------------------------------------------------
namespace ygl {

/// Matrix of elements of compile time dimensions, stored in column major
/// format, with default initializer.
/// Colums access via operator[].
template <typename T, int N, int M>
struct mat {
    /// column data type
    using V = vec<T, N>;

    /// default constructor
    constexpr mat() {
        for (auto j = 0; j < M; j++) v[j] = V{};
    }
    /// diagonal constructor
    constexpr explicit mat(T vv) {
        for (auto j = 0; j < M; j++)
            for (auto i = 0; i < N; i++) v[j][i] = (i == j) ? vv : T{};
    }
    /// list constructor
    constexpr mat(const initializer_list<V>& vv) {
        assert(M == vv.size());
        auto i = 0;
        for (auto&& e : vv) v[i++] = e;
    }

    /// element access
    constexpr V& operator[](int i) { return v[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return v[i]; }

    /// data access
    constexpr V* data() { return v; }
    /// data access
    constexpr const V* data() const { return v; }

    /// element data
    V v[M];
};

/// Specialization for 2x2 float matrices.
template <>
struct mat<float, 2, 2> {
    /// size
    constexpr static const int N = 2, M = 2;
    /// type
    using T = float;

    /// column data type
    using V = vec<T, N>;

    /// default constructor
    constexpr mat() : x{0, 0}, y{0, 0} {}
    /// diagonal constructor
    constexpr explicit mat(T vv) : x{vv, 0}, y{0, vv} {}
    /// list constructor
    constexpr mat(const V& x, const V& y) : x(x), y(y) {}

    /// element access
    constexpr V& operator[](int i) { return (&x)[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return (&x)[i]; }

    /// data access
    constexpr V* data() { return &x; }
    /// data access
    constexpr const V* data() const { return &x; }

    /// element data
    V x;
    /// element data
    V y;
};

/// Specialization for 3x3 float matrices.
template <>
struct mat<float, 3, 3> {
    /// size
    constexpr static const int N = 3, M = 3;
    /// type
    using T = float;

    /// column data type
    using V = vec<T, N>;

    /// default constructor
    constexpr mat() : x{0, 0, 0}, y{0, 0, 0}, z{0, 0, 0} {}
    /// diagonal constructor
    constexpr explicit mat(T vv) : x{vv, 0, 0}, y{0, vv, 0}, z{0, 0, vv} {}
    /// list constructor
    constexpr mat(const V& x, const V& y, const V& z) : x(x), y(y), z(z) {}

    /// element access
    constexpr V& operator[](int i) { return (&x)[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return (&x)[i]; }

    /// data access
    constexpr V* data() { return &x; }
    /// data access
    constexpr const V* data() const { return &x; }

    /// element data
    V x;
    /// element data
    V y;
    /// element data
    V z;
};

/// Specialization for 4x4 float matrices.
template <>
struct mat<float, 4, 4> {
    /// size
    constexpr static const int N = 4, M = 4;
    /// type
    using T = float;

    /// column data type
    using V = vec<T, N>;

    /// default constructor
    constexpr mat()
        : x{0, 0, 0, 0}, y{0, 0, 0, 0}, z{0, 0, 0, 0}, w{0, 0, 0, 0} {}
    /// diagonal constructor
    constexpr explicit mat(T vv)
        : x{vv, 0, 0, 0}, y{0, vv, 0, 0}, z{0, 0, vv, 0}, w{0, 0, 0, vv} {}
    /// list constructor
    constexpr mat(const V& x, const V& y, const V& z, const V& w)
        : x(x), y(y), z(z), w(w) {}

    /// element access
    constexpr V& operator[](int i) { return (&x)[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return (&x)[i]; }

    /// data access
    constexpr V* data() { return &x; }
    /// data access
    constexpr const V* data() const { return &x; }

    /// element data
    V x;
    /// element data
    V y;
    /// element data
    V z;
    /// element data
    V w;
};

/// 1-dimensional float matrix
using mat1f = mat<float, 1, 1>;
/// 2-dimensional float matrix
using mat2f = mat<float, 2, 2>;
/// 3-dimensional float matrix
using mat3f = mat<float, 3, 3>;
/// 4-dimensional float matrix
using mat4f = mat<float, 4, 4>;

/// Initialize an identity matrix.
template <typename T, int N>
constexpr inline mat<T, N, N> identity_mat() {
    mat<T, N, N> c;
    for (auto j = 0; j < N; j++)
        for (auto i = 0; i < N; i++) c[j][i] = (i == j) ? 1 : 0;
    return c;
}

/// Specialization for Initialize an identity matrix.
template <>
constexpr inline mat3f identity_mat() {
    return {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
}

/// Specialization for Initialize an identity matrix.
template <>
constexpr inline mat4f identity_mat() {
    return {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
}

/// 1-dimensional float identity matrix
const auto identity_mat1f = identity_mat<float, 1>();
/// 2-dimensional float identity matrix
const auto identity_mat2f = identity_mat<float, 2>();
/// 3-dimensional float identity matrix
const auto identity_mat3f = identity_mat<float, 3>();
/// 4-dimensional float identity matrix
const auto identity_mat4f = identity_mat<float, 4>();

/// iteration support
template <typename T, int N, int M>
constexpr inline vec<T, N>* begin(mat<T, N, M>& a) {
    return a.v;
}

/// iteration support
template <typename T, int N, int M>
constexpr inline const vec<T, N>* begin(const mat<T, N, M>& a) {
    return a.v;
}

/// iteration support
template <typename T, int N, int M>
constexpr inline vec<T, N>* end(mat<T, N, M>& a) {
    return a.v + M;
}

/// iteration support
template <typename T, int N, int M>
constexpr inline const vec<T, N>* end(const mat<T, N, M>& a) {
    return a.v + M;
}

/// vector operator ==
template <typename T, int N, int M>
constexpr inline bool operator==(const mat<T, N, M>& a, const mat<T, N, M>& b) {
    for (auto i = 0; i < M; i++)
        if (a[i] != b[i]) return false;
    return true;
}

/// vector operator !=
template <typename T, int N, int M>
constexpr inline bool operator!=(const mat<T, N, M>& a, const mat<T, N, M>& b) {
    return !(a == b);
}

/// matrix operator -
template <typename T, int N, int M>
constexpr inline mat<T, M, N> operator-(const mat<T, N, M>& a) {
    mat<T, N, M> c;
    for (auto i = 0; i < M; i++) c[i] = -a[i];
    return c;
}

/// matrix operator +
template <typename T, int N, int M>
constexpr inline mat<T, M, N> operator+(
    const mat<T, N, M>& a, const mat<T, N, M>& b) {
    mat<T, N, M> c;
    for (auto i = 0; i < M; i++) c[i] = a[i] + b[i];
    return c;
}

/// matrix scalar multiply
template <typename T, int N, int M>
constexpr inline mat<T, M, N> operator*(const mat<T, N, M>& a, T b) {
    mat<T, N, M> c;
    for (auto i = 0; i < M; i++) c[i] = a[i] * b;
    return c;
}

/// matrix scalar division
template <typename T, int N, int M>
constexpr inline mat<T, M, N> operator/(const mat<T, N, M>& a, T b) {
    mat<T, N, M> c;
    for (auto i = 0; i < M; i++) c[i] = a[i] / b;
    return c;
}

/// matrix-vector right multiply
template <typename T, int N, int M>
constexpr inline vec<T, N> operator*(
    const mat<T, N, M>& a, const vec<T, M>& b) {
    auto c = vec<T, N>();
    for (auto j = 0; j < M; j++) c += a[j] * b[j];
    return c;
}

/// matrix-vector left multiply
template <typename T, int N, int M>
constexpr inline vec<T, M> operator*(
    const vec<T, N>& a, const mat<T, N, M>& b) {
    vec<T, M> c;
    for (auto j = 0; j < M; j++) c[j] = dot(a, b[j]);
    return c;
}

/// matrix-matrix multiply
template <typename T, int N, int M, int K>
constexpr inline mat<T, N, M> operator*(
    const mat<T, N, K>& a, const mat<T, K, M>& b) {
    mat<T, N, M> c;
    for (auto j = 0; j < M; j++) c[j] = a * b[j];
    return c;
}

/// matrix-vector right multiply
template <>
constexpr inline vec2f operator*(const mat2f& a, const vec2f& b) {
    return a.x * b.x + a.y * b.y;
}

/// matrix-vector left multiply
template <>
constexpr inline vec2f operator*(const vec2f& a, const mat2f& b) {
    return {dot(a, b.x), dot(a, b.y)};
}

/// matrix-matrix multiply
template <>
constexpr inline mat2f operator*(const mat2f& a, const mat2f& b) {
    return {a * b.x, a * b.y};
}

/// matrix-vector right multiply
template <>
constexpr inline vec3f operator*(const mat3f& a, const vec3f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// matrix-vector left multiply
template <>
constexpr inline vec3f operator*(const vec3f& a, const mat3f& b) {
    return {dot(a, b.x), dot(a, b.y), dot(a, b.z)};
}

/// matrix-matrix multiply
template <>
constexpr inline mat3f operator*(const mat3f& a, const mat3f& b) {
    return {a * b.x, a * b.y, a * b.z};
}

/// matrix-vector right multiply
template <>
constexpr inline vec4f operator*(const mat4f& a, const vec4f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

/// matrix-vector left multiply
template <>
constexpr inline vec4f operator*(const vec4f& a, const mat4f& b) {
    return {dot(a, b.x), dot(a, b.y), dot(a, b.z), dot(a, b.w)};
}

/// matrix-matrix multiply
template <>
constexpr inline mat4f operator*(const mat4f& a, const mat4f& b) {
    return {a * b.x, a * b.y, a * b.z, a * b.w};
}

/// matrix sum assignment
template <typename T, int N, int M>
constexpr inline mat<T, M, N>& operator+=(
    mat<T, N, M>& a, const mat<T, N, M>& b) {
    return a = a + b;
}

/// matrix-matrix multiply assignment
template <typename T, int N, int M>
constexpr inline mat<T, M, N>& operator*=(
    mat<T, N, M>& a, const mat<T, N, M>& b) {
    return a = a * b;
}

/// matrix scaling assignment
template <typename T, int N, int M>
constexpr inline mat<T, M, N>& operator*=(mat<T, N, M>& a, const T& b) {
    return a = a * b;
}

/// matrix scaling assignment
template <typename T, int N, int M>
constexpr inline mat<T, M, N>& operator/=(mat<T, N, M>& a, const T& b) {
    return a = a / b;
}

/// matrix diagonal
template <typename T, int N>
constexpr vec<T, N> mat_diagonal(const mat<T, N, N>& a) {
    vec<T, N> d;
    for (auto i = 0; i < N; i++) d[i] = a[i][i];
    return d;
}

/// matrix transpose
template <typename T, int N, int M>
constexpr inline mat<T, M, N> transpose(const mat<T, N, M>& a) {
    mat<T, M, N> c;
    for (auto j = 0; j < M; j++) {
        for (auto i = 0; i < N; i++) { c[i][j] = a[j][i]; }
    }
    return c;
}

/// matrix adjugate (2x2)
template <typename T>
constexpr inline mat<T, 2, 2> adjugate(const mat<T, 2, 2>& a) {
    return {{a.y.y, -a.x.y}, {-a.y.x, a.x.x}};
}

/// matrix adjugate (3x3)
template <typename T>
constexpr inline mat<T, 3, 3> adjugate(const mat<T, 3, 3>& a) {
    return {{a.y.y * a.z.z - a.z.y * a.y.z, a.z.y * a.x.z - a.x.y * a.z.z,
                a.x.y * a.y.z - a.y.y * a.x.z},
        {a.y.z * a.z.x - a.z.z * a.y.x, a.z.z * a.x.x - a.x.z * a.z.x,
            a.x.z * a.y.x - a.y.z * a.x.x},
        {a.y.x * a.z.y - a.z.x * a.y.y, a.z.x * a.x.y - a.x.x * a.z.y,
            a.x.x * a.y.y - a.y.x * a.x.y}};
}

/// matrix adjugate (4x4)
template <typename T>
constexpr inline mat<T, 4, 4> adjugate(const mat<T, 4, 4>& a) {
    return {{a.y.y * a.z.z * a.w.w + a.w.y * a.y.z * a.z.w +
                    a.z.y * a.w.z * a.y.w - a.y.y * a.w.z * a.z.w -
                    a.z.y * a.y.z * a.w.w - a.w.y * a.z.z * a.y.w,
                a.x.y * a.w.z * a.z.w + a.z.y * a.x.z * a.w.w +
                    a.w.y * a.z.z * a.x.w - a.w.y * a.x.z * a.z.w -
                    a.z.y * a.w.z * a.x.w - a.x.y * a.z.z * a.w.w,
                a.x.y * a.y.z * a.w.w + a.w.y * a.x.z * a.y.w +
                    a.y.y * a.w.z * a.x.w - a.x.y * a.w.z * a.y.w -
                    a.y.y * a.x.z * a.w.w - a.w.y * a.y.z * a.x.w,
                a.x.y * a.z.z * a.y.w + a.y.y * a.x.z * a.z.w +
                    a.z.y * a.y.z * a.x.w - a.x.y * a.y.z * a.z.w -
                    a.z.y * a.x.z * a.y.w - a.y.y * a.z.z * a.x.w},
        {a.y.z * a.w.w * a.z.x + a.z.z * a.y.w * a.w.x + a.w.z * a.z.w * a.y.x -
                a.y.z * a.z.w * a.w.x - a.w.z * a.y.w * a.z.x -
                a.z.z * a.w.w * a.y.x,
            a.x.z * a.z.w * a.w.x + a.w.z * a.x.w * a.z.x +
                a.z.z * a.w.w * a.x.x - a.x.z * a.w.w * a.z.x -
                a.z.z * a.x.w * a.w.x - a.w.z * a.z.w * a.x.x,
            a.x.z * a.w.w * a.y.x + a.y.z * a.x.w * a.w.x +
                a.w.z * a.y.w * a.x.x - a.x.z * a.y.w * a.w.x -
                a.w.z * a.x.w * a.y.x - a.y.z * a.w.w * a.x.x,
            a.x.z * a.y.w * a.z.x + a.z.z * a.x.w * a.y.x +
                a.y.z * a.z.w * a.x.x - a.x.z * a.z.w * a.y.x -
                a.y.z * a.x.w * a.z.x - a.z.z * a.y.w * a.x.x},
        {a.y.w * a.z.x * a.w.y + a.w.w * a.y.x * a.z.y + a.z.w * a.w.x * a.y.y -
                a.y.w * a.w.x * a.z.y - a.z.w * a.y.x * a.w.y -
                a.w.w * a.z.x * a.y.y,
            a.x.w * a.w.x * a.z.y + a.z.w * a.x.x * a.w.y +
                a.w.w * a.z.x * a.x.y - a.x.w * a.z.x * a.w.y -
                a.w.w * a.x.x * a.z.y - a.z.w * a.w.x * a.x.y,
            a.x.w * a.y.x * a.w.y + a.w.w * a.x.x * a.y.y +
                a.y.w * a.w.x * a.x.y - a.x.w * a.w.x * a.y.y -
                a.y.w * a.x.x * a.w.y - a.w.w * a.y.x * a.x.y,
            a.x.w * a.z.x * a.y.y + a.y.w * a.x.x * a.z.y +
                a.z.w * a.y.x * a.x.y - a.x.w * a.y.x * a.z.y -
                a.z.w * a.x.x * a.y.y - a.y.w * a.z.x * a.x.y},
        {a.y.x * a.w.y * a.z.z + a.z.x * a.y.y * a.w.z + a.w.x * a.z.y * a.y.z -
                a.y.x * a.z.y * a.w.z - a.w.x * a.y.y * a.z.z -
                a.z.x * a.w.y * a.y.z,
            a.x.x * a.z.y * a.w.z + a.w.x * a.x.y * a.z.z +
                a.z.x * a.w.y * a.x.z - a.x.x * a.w.y * a.z.z -
                a.z.x * a.x.y * a.w.z - a.w.x * a.z.y * a.x.z,
            a.x.x * a.w.y * a.y.z + a.y.x * a.x.y * a.w.z +
                a.w.x * a.y.y * a.x.z - a.x.x * a.y.y * a.w.z -
                a.w.x * a.x.y * a.y.z - a.y.x * a.w.y * a.x.z,
            a.x.x * a.y.y * a.z.z + a.z.x * a.x.y * a.y.z +
                a.y.x * a.z.y * a.x.z - a.x.x * a.z.y * a.y.z -
                a.y.x * a.x.y * a.z.z - a.z.x * a.y.y * a.x.z}};
}

/// matrix determinant (2x2)
template <typename T>
constexpr inline T determinant(const mat<T, 2, 2>& a) {
    return a.x.x * a.y.y - a.x.y * a.y.x;
}

/// matrix determinant (3x3)
template <typename T>
constexpr inline T determinant(const mat<T, 3, 3>& a) {
    return a.x.x * (a.y.y * a.z.z - a.z.y * a.y.z) +
           a.x.y * (a.y.z * a.z.x - a.z.z * a.y.x) +
           a.x.z * (a.y.x * a.z.y - a.z.x * a.y.y);
}

/// matrix determinant (4x4)
template <typename T>
constexpr inline T determinant(const mat<T, 4, 4>& a) {
    return a.x.x * (a.y.y * a.z.z * a.w.w + a.w.y * a.y.z * a.z.w +
                       a.z.y * a.w.z * a.y.w - a.y.y * a.w.z * a.z.w -
                       a.z.y * a.y.z * a.w.w - a.w.y * a.z.z * a.y.w) +
           a.x.y * (a.y.z * a.w.w * a.z.x + a.z.z * a.y.w * a.w.x +
                       a.w.z * a.z.w * a.y.x - a.y.z * a.z.w * a.w.x -
                       a.w.z * a.y.w * a.z.x - a.z.z * a.w.w * a.y.x) +
           a.x.z * (a.y.w * a.z.x * a.w.y + a.w.w * a.y.x * a.z.y +
                       a.z.w * a.w.x * a.y.y - a.y.w * a.w.x * a.z.y -
                       a.z.w * a.y.x * a.w.y - a.w.w * a.z.x * a.y.y) +
           a.x.w * (a.y.x * a.w.y * a.z.z + a.z.x * a.y.y * a.w.z +
                       a.w.x * a.z.y * a.y.z - a.y.x * a.z.y * a.w.z -
                       a.w.x * a.y.y * a.z.z - a.z.x * a.w.y * a.y.z);
}

/// matrix inverse (uses adjugate and determinant)
template <typename T, int N>
constexpr inline mat<T, N, N> inverse(const mat<T, N, N>& a) {
    return adjugate(a) / determinant(a);
}

/// stream write
template <typename T, int N, int M>
inline ostream& operator<<(ostream& os, const mat<T, N, M>& a) {
    for (auto i = 0; i < M; i++) {
        if (i) os << ' ';
        os << a[i];
    }
    return os;
}

/// stream read
template <typename T, int N, int M>
inline istream& operator>>(istream& is, mat<T, N, M>& a) {
    for (auto i = 0; i < M; i++) is >> a[i];
    return is;
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// RIGID BODY TRANSFORMS/FRAMES
// -----------------------------------------------------------------------------
namespace ygl {

/// Rigid transforms stored as a column-major affine matrix Nx(N+1).
/// In memory, this representation is equivalent to storing an NxN rotation
/// followed by a Nx1 translation. Viewed this way, the representation allows
/// also to retrive the axis of the coordinate frame as the first N column and
/// the translation as the N+1 column.
/// Colums access via operator[]. Access rotation and position with pos() and
/// rot().
template <typename T, int N>
struct frame {
    /// column data type
    using V = vec<T, N>;
    /// rotation data type
    using M = mat<T, N, N>;

    /// default constructor
    constexpr frame() {
        for (auto i = 0; i < N + 1; i++) v[i] = V{};
    }
    /// element constructor
    constexpr frame(const initializer_list<vec<T, N>>& vv) {
        assert(N + 1 == vv.size());
        auto i = 0;
        for (auto&& e : vv) v[i++] = e;
    }
    /// element constructor
    constexpr frame(const M& m, const V& t) {
        for (auto i = 0; i < N; i++) v[i] = m[i];
        v[N + 1] = t;
    }

    /// element access
    constexpr V& operator[](int i) { return v[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return v[i]; }

    /// data access
    constexpr V* data() { return v; }
    /// data access
    constexpr const V* data() const { return v; }

    /// access position
    constexpr V& pos() { return v[N]; }
    /// access position
    constexpr const V& pos() const { return v[N]; }

    /// access rotation
    constexpr M& rot() { return *(M*)v; }
    /// access rotation
    constexpr const M& rot() const { return *(M*)v; }

    /// element data
    V v[N + 1];
};

/// Specialization for 3D float frames.
template <>
struct frame<float, 2> {
    /// size
    constexpr static const int N = 2;
    /// type
    using T = float;

    /// column data type
    using V = vec<T, N>;
    /// rotation data type
    using M = mat<T, N, N>;

    /// default constructor
    constexpr frame() : x{0, 0}, y{0, 0}, o{0, 0} {}
    /// element constructor
    constexpr frame(const V& x, const V& y, const V& o) : x(x), y(y), o(o) {}
    /// element constructor
    constexpr frame(const M& m, const V& t) : x(m.x), y(m.y), o(t) {}

    /// element access
    constexpr V& operator[](int i) { return (&x)[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return (&x)[i]; }

    /// data access
    constexpr V* data() { return &x; }
    /// data access
    constexpr const V* data() const { return &x; }

    /// access position
    constexpr V& pos() { return o; }
    /// access position
    constexpr const V& pos() const { return o; }

    /// access rotation
    constexpr M& rot() { return *(M*)(&x); }
    /// access rotation
    constexpr const M& rot() const { return *(M*)(&x); }

    /// element data
    V x;
    /// element data
    V y;
    /// element data
    V o;
};

/// Specialization for 3D float frames.
template <>
struct frame<float, 3> {
    /// size
    constexpr static const int N = 3;
    /// type
    using T = float;

    /// column data type
    using V = vec<T, N>;
    /// rotation data type
    using M = mat<T, N, N>;

    /// default constructor
    constexpr frame() : x{0, 0, 0}, y{0, 0, 0}, z{0, 0, 0}, o{0, 0, 0} {}

    /// element constructor
    constexpr frame(const V& x, const V& y, const V& z, const V& o)
        : x(x), y(y), z(z), o(o) {}

    /// element constructor
    constexpr frame(const M& m, const V& t) : x(m.x), y(m.y), z(m.z), o(t) {}

    /// conversion from matrix (assumes the matrix is a frame, so dangerous!)
    constexpr frame(const mat<T, 4, 4>& m)
        : x(m.x.x, m.x.y, m.x.z)
        , y(m.y.x, m.y.y, m.y.z)
        , z(m.z.x, m.z.y, m.z.z)
        , o(m.w.x, m.w.y, m.w.z) {}

    /// conversion to matrix
    constexpr explicit operator mat<T, 4, 4>() const {
        return {{x.x, x.y, x.z, 0}, {y.x, y.y, y.z, 0}, {z.x, z.y, z.z, 0},
            {o.x, o.y, o.z, 1}};
    }

    /// element access
    constexpr V& operator[](int i) { return (&x)[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return (&x)[i]; }

    /// data access
    constexpr V* data() { return &x; }
    /// data access
    constexpr const V* data() const { return &x; }

    /// access position
    constexpr V& pos() { return o; }
    /// access position
    constexpr const V& pos() const { return o; }

    /// access rotation
    constexpr M& rot() { return *(M*)(&x); }
    /// access rotation
    constexpr const M& rot() const { return *(M*)(&x); }

    /// element data
    V x;
    /// element data
    V y;
    /// element data
    V z;
    /// element data
    V o;
};

/// 1-dimensional float frame
using frame1f = frame<float, 1>;
/// 2-dimensional float frame
using frame2f = frame<float, 2>;
/// 3-dimensional float frame
using frame3f = frame<float, 3>;
/// 4-dimensional float frame
using frame4f = frame<float, 4>;

/// Initialize an identity frame.
template <typename T, int N>
constexpr inline frame<T, N> identity_frame() {
    frame<T, N> c;
    for (auto j = 0; j < N; j++)
        for (auto i = 0; i < N; i++) c[j][i] = (i == j) ? 1 : 0;
    for (auto i = 0; i < N; i++) c[N][i] = 0;
    return c;
}

// initializes a frame3 from origin and z.
template <typename T>
constexpr inline frame<T, 3> make_frame3_fromz(
    const vec<T, 3>& o, const vec<T, 3>& z_) {
    auto z = normalize(z_);
    auto x = normalize(orthogonal(z));
    auto y = normalize(cross(z, x));
    return {x, y, z, o};
}

// initializes a frame3 from origin, z and x.
template <typename T>
constexpr inline frame<T, 3> make_frame3_fromzx(
    const vec<T, 3>& o, const vec<T, 3>& z_, const vec<T, 3>& x_) {
    auto z = normalize(z_);
    auto x = orthonormalize(x_, z);
    auto y = normalize(cross(z, x));
    return {x, y, z, o};
}

/// Initialize an identity frame.
template <>
constexpr inline frame2f identity_frame() {
    return {{1, 0}, {0, 1}, {0, 0}};
}

/// Initialize an identity frame.
template <>
constexpr inline frame3f identity_frame() {
    return {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 0}};
}

// initializes a frame3 from origin and z.
template <>
constexpr inline frame3f make_frame3_fromz(const vec3f& o, const vec3f& z_) {
    auto z = normalize(z_);
    auto x = normalize(orthogonal(z));
    auto y = normalize(cross(z, x));
    return {x, y, z, o};
}

// initializes a frame3 from origin, z and x.
template <typename T>
constexpr inline frame3f make_frame3_fromzx(
    const vec3f& o, const vec3f& z_, const vec3f& x_) {
    auto z = normalize(z_);
    auto x = orthonormalize(x_, z);
    auto y = normalize(cross(z, x));
    return {x, y, z, o};
}

/// 1-dimensional float identity frame
const auto identity_frame1f = identity_frame<float, 1>();
/// 2-dimensional float identity frame
const auto identity_frame2f = identity_frame<float, 2>();
/// 3-dimensional float identity frame
const auto identity_frame3f = identity_frame<float, 3>();
/// 4-dimensional float identity frame
const auto identity_frame4f = identity_frame<float, 4>();

/// frame position const access
template <typename T, int N>
constexpr inline const vec<T, N>& pos(const frame<T, N>& f) {
    return f[N];
}

/// frame rotation const access
template <typename T, int N>
constexpr inline const mat<T, N, N>& rot(const frame<T, N>& f) {
    return *(mat<T, N, N>*)&f;
}

/// frame position reference
template <typename T, int N>
constexpr inline vec<T, N>& pos(frame<T, N>& f) {
    return f[N];
}

/// frame rotation reference
template <typename T, int N>
constexpr inline mat<T, N, N>& rot(frame<T, N>& f) {
    return *(mat<T, N, N>*)&f;
}

/// iteration support
template <typename T, int N>
constexpr inline vec<T, N>* begin(frame<T, N>& a) {
    return a.v;
}

/// iteration support
template <typename T, int N>
constexpr inline const vec<T, N>* begin(const frame<T, N>& a) {
    return a.v;
}

/// iteration support
template <typename T, int N>
constexpr inline vec<T, N>* end(frame<T, N>& a) {
    return a.v + N + 1;
}

/// iteration support
template <typename T, int N>
constexpr inline const vec<T, N>* end(const frame<T, N>& a) {
    return a.v + N + 1;
}

/// frame to matrix conversion
template <typename T, int N>
constexpr inline mat<T, N + 1, N + 1> to_mat(const frame<T, N>& a) {
    auto m = mat<T, N + 1, N + 1>();
    for (auto j = 0; j < N; j++) {
        (vec<T, N>&)m[j] = a[j];
        m[j][N] = 0;
    }
    (vec<T, N>&)m[N] = a[N];
    m[N][N] = 1;
    return m;
}

/// matrix to frame conversion
template <typename T, int N>
constexpr inline frame<T, N - 1> to_frame(const mat<T, N, N>& a) {
    auto f = frame<T, N - 1>();
    for (auto j = 0; j < N; j++) {
        for (auto i = 0; i < N - 1; i++) { f[j][i] = a[j][i]; }
    }
    return f;
}

/// vector operator ==
template <typename T, int N>
constexpr inline bool operator==(const frame<T, N>& a, const frame<T, N>& b) {
    for (auto i = 0; i < N + 1; i++)
        if (a[i] != b[i]) return false;
    return true;
}

/// vector operator !=
template <typename T, int N>
constexpr inline bool operator!=(const frame<T, N>& a, const frame<T, N>& b) {
    return !(a == b);
}

/// frame composition (equivalent to affine matrix multiply)
template <typename T, int N>
constexpr inline frame<T, N> operator*(
    const frame<T, N>& a, const frame<T, N>& b) {
    return {a.rot() * b.rot(), a.rot() * b.pos() + a.pos()};
}

/// frame inverse (equivalent to rigid affine inverse)
template <typename T, int N>
constexpr inline frame<T, N> inverse(const frame<T, N>& a) {
    auto minv = transpose(rot(a));
    return {minv, -(minv * pos(a))};
}

/// frame composition (equivalent to affine matrix multiply)
template <>
constexpr inline frame3f operator*(const frame3f& a, const frame3f& b) {
    return {a.rot() * b.rot(), a.rot() * b.pos() + a.pos()};
}

/// frame inverse (equivalent to rigid affine inverse)
template <>
constexpr inline frame3f inverse(const frame3f& a) {
    auto minv = transpose(a.rot());
    return {minv, -(minv * a.pos())};
}

/// stream write
template <typename T, int N>
inline ostream& operator<<(ostream& os, const frame<T, N>& a) {
    for (auto i = 0; i < N + 1; i++) {
        if (i) os << ' ';
        os << a[i];
    }
    return os;
}

/// stream read
template <typename T, int N>
inline istream& operator>>(istream& is, frame<T, N>& a) {
    for (auto i = 0; i < N + 1; i++) is >> a[i];
    return is;
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// QUATERNIONS
// -----------------------------------------------------------------------------
namespace ygl {

/// Quaternion placeholder. Only helpful in the specialization.
template <typename T, int N>
struct quat;

/// Quaternions implemented as a vec<T,4>. Data access via operator[].
/// Quaterions are xi + yj + zk + w.
template <typename T>
struct quat<T, 4> {
    /// size
    constexpr static const int N = 4;

    /// default constructor
    constexpr quat() : x{0}, y{0}, z{0}, w{1} {}

    // list constructor
    constexpr quat(const T& x, const T& y, const T& z, const T& w)
        : x{x}, y{y}, z{z}, w{w} {}

    /// conversion from vec
    constexpr explicit quat(const vec<T, N>& vv)
        : x{vv.x}, y{vv.y}, z{vv.z}, w{vv.w} {}
    /// conversion to vec
    constexpr explicit operator vec<T, N>() const { return {x, y, z, w}; }

    /// element access
    constexpr T& operator[](int i) { return (&x)[i]; }
    /// element access
    constexpr const T& operator[](int i) const { return (&x)[i]; }

    /// data access
    constexpr T* data() { return &x; }
    /// data access
    constexpr const T* data() const { return &x; }

    /// data
    T x;
    /// data
    T y;
    /// data
    T z;
    /// data
    T w;
};

/// float quaterion
using quat4f = quat<float, 4>;

/// float identity quaterion
const auto identity_quat4f = quat<float, 4>{0, 0, 0, 1};

/// vector operator ==
template <typename T, int N>
constexpr inline bool operator==(const quat<T, N>& a, const quat<T, N>& b) {
    for (auto i = 0; i < N; i++)
        if (a[i] != b[i]) return false;
    return true;
}

/// vector operator !=
template <typename T, int N>
constexpr inline bool operator!=(const quat<T, N>& a, const quat<T, N>& b) {
    return !(a == b);
}

/// quaterion multiply
template <typename T>
constexpr quat<T, 4> operator*(const quat<T, 4>& a, const quat<T, 4>& b) {
    return {a.x * b.w + a.w * b.x + a.y * b.w - a.z * b.y,
        a.y * b.w + a.w * b.y + a.z * b.x - a.x * b.z,
        a.z * b.w + a.w * b.z + a.x * b.y - a.y * b.x,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

/// quaterion conjugate
template <typename T>
constexpr quat<T, 4> conjugate(const quat<T, 4>& v) {
    return {-v.x, -v.y, -v.z, v.w};
}

/// quaterion inverse
template <typename T>
constexpr quat<T, 4> inverse(const quat<T, 4>& v) {
    return conjugate(v) / lengthsqr(vec<T, 4>(v));
}

/// quaterion inverse
template <typename T>
constexpr quat<T, 4> normalize(const quat<T, 4>& v) {
    auto l = length(vec<T, 4>{v.x, v.y, v.z, v.w});
    if (!l) return {0, 0, 0, 1};
    return {v.x / l, v.y / l, v.z / l, v.w / l};
}

/// quaterion normalized linear interpolation
template <typename T>
constexpr quat<T, 4> nlerp(const quat<T, 4>& a, const quat<T, 4>& b, T t) {
    return (quat<T, 4>)nlerp(vec<T, 4>(a),
        dot(vec<T, 4>(a), vec<T, 4>(b)) < 0 ? -vec<T, 4>(b) : vec<T, 4>(b), t);
}

/// quaterion spherical linear interpolation
template <typename T>
constexpr quat<T, 4> slerp(const quat<T, 4>& a, const quat<T, 4>& b, T t) {
    return (quat<T, 4>)slerp(vec<T, 4>(a),
        dot(vec<T, 4>(a), vec<T, 4>(b)) < 0 ? -vec<T, 4>(b) : vec<T, 4>(b), t);
}

/// stream write
template <typename T>
inline ostream& operator<<(ostream& os, const quat<T, 4>& a) {
    for (auto i = 0; i < 4; i++) {
        if (i) os << ' ';
        os << a[i];
    }
    return os;
}

/// stream read
template <typename T>
inline istream& operator>>(istream& is, quat<T, 4>& a) {
    for (auto i = 0; i < 4; i++) is >> a[i];
    return is;
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// GEOMETRY UTILITIES
// -----------------------------------------------------------------------------
namespace ygl {

/// line tangent
template <typename T>
constexpr inline vec<T, 3> line_tangent(
    const vec<T, 3>& v0, const vec<T, 3>& v1) {
    return normalize(v1 - v0);
}

/// line length
template <typename T>
constexpr inline T line_length(const vec<T, 3>& v0, const vec<T, 3>& v1) {
    return length(v1 - v0);
}

/// triangle normal
template <typename T>
constexpr inline vec<T, 3> triangle_normal(
    const vec<T, 3>& v0, const vec<T, 3>& v1, const vec<T, 3>& v2) {
    return normalize(cross(v1 - v0, v2 - v0));
}

/// triangle area
template <typename T>
constexpr inline T triangle_area(
    const vec<T, 3>& v0, const vec<T, 3>& v1, const vec<T, 3>& v2) {
    return length(cross(v1 - v0, v2 - v0)) / 2;
}

/// quad area
template <typename T>
constexpr inline T quad_area(const vec<T, 3>& v0, const vec<T, 3>& v1,
    const vec<T, 3>& v2, const vec<T, 3>& v3) {
    return triangle_area(v0, v1, v3) + triangle_area(v3, v2, v1);
}

/// tetrahedron volume
template <typename T>
constexpr inline T tetrahedron_volume(const vec<T, 3>& v0, const vec<T, 3>& v1,
    const vec<T, 3>& v2, const vec<T, 3>& v3) {
    return dot(cross(v1 - v0, v2 - v0), v3 - v0) / 6;
}

// Triangle tangent and bitangent from uv (not othornormalized with themselfves
// not the normal). Follows the definition in
// http://www.terathon.com/code/tangent.html and
// https://gist.github.com/aras-p/2843984
template <typename T>
constexpr inline pair<vec<T, 3>, vec<T, 3>> triangle_tangents_fromuv(
    const vec<T, 3>& v0, const vec<T, 3>& v1, const vec<T, 3>& v2,
    const vec<T, 2>& uv0, const vec<T, 2>& uv1, const vec<T, 2>& uv2) {
    // normal points up from texture space
    auto p = v1 - v0;
    auto q = v2 - v0;
    auto s = vec<T, 2>{uv1.x - uv0.x, uv2.x - uv0.x};
    auto t = vec<T, 2>{uv1.y - uv0.y, uv2.y - uv0.y};
    auto div = s.x * t.y - s.y * t.x;

    if (div != 0) {
        auto tu = vec<T, 3>{t.y * p.x - t.x * q.x, t.y * p.y - t.x * q.y,
                      t.y * p.z - t.x * q.z} /
                  div;
        auto tv = vec<T, 3>{s.x * q.x - s.y * p.x, s.x * q.y - s.y * p.y,
                      s.x * q.z - s.y * p.z} /
                  div;
        return {tu, tv};
    } else {
        return {{1, 0, 0}, {0, 1, 0}};
    }
}

/// line barycentric interpolation
template <typename T, typename T1>
constexpr inline T eval_barycentric_point(const T& a, const T1& w) {
    return a * w.x;
}

/// line barycentric interpolation
template <typename T, typename T1>
constexpr inline T eval_barycentric_point(
    const vector<T>& vals, const int& p, const T1& w) {
    if (vals.empty()) return T();
    return vals[p] * w.x;
}

/// line barycentric interpolation
template <typename T, typename T1>
constexpr inline T eval_barycentric_line(const T& a, const T& b, const T1& w) {
    return a * w.x + b * w.y;
}

/// line barycentric interpolation
template <typename T, typename T1>
constexpr inline T eval_barycentric_line(
    const vector<T>& vals, const vec2i& l, const T1& w) {
    if (vals.empty()) return T();
    return vals[l.x] * w.x + vals[l.y] * w.y;
}

/// triangle barycentric interpolation
template <typename T, typename T1>
constexpr inline T eval_barycentric_triangle(
    const T& a, const T& b, const T& c, const T1& w) {
    return a * w.x + b * w.y + c * w.z;
}

/// triangle barycentric interpolation
template <typename T, typename T1>
constexpr inline T eval_barycentric_triangle(
    const vector<T>& vals, const vec3i& t, const T1& w) {
    if (vals.empty()) return T();
    return vals[t.x] * w.x + vals[t.y] * w.y + vals[t.z] * w.z;
}

/// tetrahedra barycentric interpolation
template <typename T, typename T1>
constexpr inline T eval_barycentric_tetra(
    const T& a, const T& b, const T& c, const T& d, const T1& w) {
    return a * w.x + b * w.y + c * w.z + d * w.w;
}

/// tetrahedron barycentric interpolation
template <typename T, typename T1>
constexpr inline T eval_barycentric_tetra(
    const vector<T>& vals, const vec4i& t, const T1& w) {
    if (vals.empty()) return T();
    return vals[t.x] * w.x + vals[t.y] * w.y + vals[t.z] * w.z +
           vals[t.w] * w.w;
}

/// quad interpolation based on the two-triangle representation
template <typename T, typename T1>
constexpr inline T eval_barycentric_quad(
    const T& a, const T& b, const T& c, const T& d, const T1& w) {
    return a * w.x + b * w.y + c * w.z + d * w.w;
}

/// quad interpolation based on the two-triangle representation
template <typename T, typename T1>
constexpr inline T eval_barycentric_quad(
    const vector<T>& vals, const vec4i& t, const T1& w) {
    if (vals.empty()) return T();
    return vals[t.x] * w.x + vals[t.y] * w.y + vals[t.z] * w.z +
           vals[t.w] * w.w;
}

/// bernstein polynomials (for Bezier)
template <typename T>
inline T eval_bernstein(T u, int i, int degree) {
    if (i < 0 or i > degree) return 0;
    if (degree == 0)
        return 1;
    else if (degree == 1) {
        if (i == 0)
            return 1 - u;
        else if (i == 1)
            return u;
    } else if (degree == 2) {
        if (i == 0)
            return (1 - u) * (1 - u);
        else if (i == 1)
            return 2 * u * (1 - u);
        else if (i == 2)
            return u * u;
    } else if (degree == 3) {
        if (i == 0)
            return (1 - u) * (1 - u) * (1 - u);
        else if (i == 1)
            return 3 * u * (1 - u) * (1 - u);
        else if (i == 2)
            return 3 * u * u * (1 - u);
        else if (i == 3)
            return u * u * u;
    } else {
        return (1 - u) * eval_bernstein(u, i, degree - 1) +
               u * eval_bernstein(u, i - 1, degree - 1);
    }
    return 0;
}

/// bernstein polynomials (for Bezier)
template <typename T>
inline T eval_bernstein_derivative(T u, int i, int degree) {
    return degree * (eval_bernstein(u, i - 1, degree - 1) -
                        eval_bernstein(u, i, degree - 1));
}

/// eval bezier
template <typename T, typename T1>
inline T eval_bezier_cubic(
    const T& v0, const T& v1, const T& v2, const T& v3, T1 t) {
    return v0 * eval_bernstein(t, 0, 3) + v1 * eval_bernstein(t, 1, 3) +
           v2 * eval_bernstein(t, 2, 3) + v3 * eval_bernstein(t, 3, 3);
}

/// eval bezier
template <typename T, typename T1>
inline T eval_bezier_cubic(const vector<T>& vals, const vec4i& b, T1 t) {
    if (vals.empty()) return T();
    return eval_bezier_cubic(vals[b.x], vals[b.y], vals[b.z], vals[b.w], t);
}

/// eval bezier derivative
template <typename T, typename T1>
inline T eval_bezier_cubic_derivative(
    const T& v0, const T& v1, const T& v2, const T& v3, T1 t) {
    return v0 * eval_bernstein_derivative(t, 0, 3) +
           v1 * eval_bernstein_derivative(t, 1, 3) +
           v2 * eval_bernstein_derivative(t, 2, 3) +
           v3 * eval_bernstein_derivative(t, 3, 3);
}

/// eval bezier derivative
template <typename T, typename T1>
inline T eval_bezier_cubic_derivative(
    const vector<T>& vals, const vec4i& b, T1 t) {
    if (vals.empty()) return T();
    return eval_bezier_cubic_derivative(
        vals[b.x], vals[b.y], vals[b.z], vals[b.w], t);
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// AXIS ALIGNED BOUNDING BOXES
// -----------------------------------------------------------------------------
namespace ygl {

/// Axis aligned bounding box represented as a min/max vector pair.
/// Access min/max with operator[].
template <typename T, int N>
struct bbox {
    /// column data type
    using V = vec<T, N>;

    /// initializes an invalid bbox
    constexpr bbox() {
        for (auto i = 0; i < N; i++) {
            min[i] = numeric_limits<T>::max();
            max[i] = numeric_limits<T>::lowest();
        }
    }

    /// list constructor
    constexpr bbox(const vec<T, N>& m, const vec<T, N>& M) : min{m}, max{M} {}

    /// element access
    constexpr V& operator[](int i) { return (&min)[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return (&min)[i]; }

    /// element data
    V min;
    /// element data
    V max;
};

/// Specialization for float 3D bounding boxes.
template <>
struct bbox<float, 1> {
    /// size
    constexpr static const int N = 1;
    /// type
    using T = float;

    /// column data type
    using V = vec<T, N>;

    /// initializes an invalid bbox
    constexpr bbox() : min{flt_max}, max{flt_min} {}
    /// list constructor
    constexpr bbox(const vec<T, N>& m, const vec<T, N>& M) : min{m}, max{M} {}

    /// element access
    constexpr V& operator[](int i) { return (&min)[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return (&min)[i]; }

    /// element data
    V min;
    /// element data
    V max;
};

/// Specialization for float 3D bounding boxes.
template <>
struct bbox<float, 2> {
    /// size
    constexpr static const int N = 2;
    /// type
    using T = float;

    /// column data type
    using V = vec<T, N>;

    /// initializes an invalid bbox
    constexpr bbox() : min{flt_max, flt_max}, max{flt_min, flt_min} {}
    /// list constructor
    constexpr bbox(const vec<T, N>& m, const vec<T, N>& M) : min{m}, max{M} {}

    /// element access
    constexpr V& operator[](int i) { return (&min)[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return (&min)[i]; }

    /// element data
    V min;
    /// element data
    V max;
};

/// Specialization for float 3D bounding boxes.
template <>
struct bbox<float, 3> {
    /// size
    constexpr static const int N = 3;
    /// type
    using T = float;

    /// column data type
    using V = vec<T, N>;

    /// initializes an invalid bbox
    constexpr bbox()
        : min{flt_max, flt_max, flt_max}, max{flt_min, flt_min, flt_min} {}
    /// list constructor
    constexpr bbox(const vec<T, N>& m, const vec<T, N>& M) : min{m}, max{M} {}

    /// element access
    constexpr V& operator[](int i) { return (&min)[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return (&min)[i]; }

    /// element data
    V min;
    /// element data
    V max;
};

/// Specialization for float 3D bounding boxes.
template <>
struct bbox<float, 4> {
    /// size
    constexpr static const int N = 4;
    /// type
    using T = float;

    /// column data type
    using V = vec<T, N>;

    /// initializes an invalid bbox
    constexpr bbox()
        : min{flt_max, flt_max, flt_max, flt_max}
        , max{flt_min, flt_min, flt_min, flt_min} {}
    /// list constructor
    constexpr bbox(const vec<T, N>& m, const vec<T, N>& M) : min{m}, max{M} {}

    /// element access
    constexpr V& operator[](int i) { return (&min)[i]; }
    /// element access
    constexpr const V& operator[](int i) const { return (&min)[i]; }

    /// element data
    V min;
    /// element data
    V max;
};

/// 1-dimensional float bbox
using bbox1f = bbox<float, 1>;
/// 2-dimensional float bbox
using bbox2f = bbox<float, 2>;
/// 3-dimensional float bbox
using bbox3f = bbox<float, 3>;
/// 4-dimensional float bbox
using bbox4f = bbox<float, 4>;

/// 1-dimensional float bbox
using bbox1i = bbox<int, 1>;
/// 2-dimensional float bbox
using bbox2i = bbox<int, 2>;
/// 3-dimensional float bbox
using bbox3i = bbox<int, 3>;
/// 4-dimensional float bbox
using bbox4i = bbox<int, 4>;

/// initializes an empty bbox
template <typename T, int N>
constexpr inline bbox<T, N> invalid_bbox() {
    auto a = bbox<T, N>();
    for (auto i = 0; i < N; i++) {
        a.min[i] = numeric_limits<T>::max();
        a.max[i] = numeric_limits<T>::lowest();
    }
    return a;
}

/// initialize a bonding box from a list of points
template <typename T, int N>
constexpr inline bbox<T, N> make_bbox(int count, const vec<T, N>* v) {
    auto a = invalid_bbox<T, N>();
    for (auto j = 0; j < count; j++) {
        auto&& vv = v[j];
        for (auto i = 0; i < N; i++) {
            a.min[i] = min(a.min[i], vv[i]);
            a.max[i] = max(a.max[i], vv[i]);
        }
    }
    return a;
}

/// initialize a bonding box from a list of points
template <typename T, int N>
constexpr inline bbox<T, N> make_bbox(const initializer_list<vec<T, N>>& v) {
    auto a = invalid_bbox<T, N>();
    for (auto&& vv : v) {
        for (auto i = 0; i < N; i++) {
            a.min[i] = min(a.min[i], vv[i]);
            a.max[i] = max(a.max[i], vv[i]);
        }
    }
    return a;
}

/// 1-dimensional float empty bbox
const auto invalid_bbox1f = bbox1f();
/// 2-dimensional float empty bbox
const auto invalid_bbox2f = bbox2f();
/// 3-dimensional float empty bbox
const auto invalid_bbox3f = bbox3f();
/// 4-dimensional float empty bbox
const auto invalid_bbox4f = bbox4f();

/// bbox operator ==
template <typename T, int N>
constexpr inline bool operator==(const bbox<T, N>& a, const bbox<T, N>& b) {
    return a.min == b.min && a.max == b.max;
}

/// bbox operator !=
template <typename T, int N>
constexpr inline bool operator!=(const bbox<T, N>& a, const bbox<T, N>& b) {
    return a.min != b.min || a.max != b.max;
}

/// computes the center of a bbox
template <typename T, int N>
constexpr inline vec<T, N> center(const bbox<T, N>& a) {
    return (a.min + a.max) / (T)2;
}

/// computes the diagonal of a bbox
template <typename T, int N>
constexpr inline vec<T, N> diagonal(const bbox<T, N>& a) {
    return a.max - a.min;
}

/// iteration support
template <typename T, int N>
constexpr inline vec<T, N>* begin(bbox<T, N>& a) {
    return a.v;
}

/// iteration support
template <typename T, int N>
constexpr inline const vec<T, N>* begin(const bbox<T, N>& a) {
    return a.v;
}

/// iteration support
template <typename T, int N>
constexpr inline vec<T, N>* end(bbox<T, N>& a) {
    return a.v + 2;
}

/// iteration support
template <typename T, int N>
constexpr inline const vec<T, N>* end(const bbox<T, N>& a) {
    return a.v + 2;
}

/// expands a bounding box with a point
template <typename T, int N>
constexpr inline bbox<T, N> expand(const bbox<T, N>& a, const vec<T, N>& b) {
    bbox<T, N> c;
    for (auto i = 0; i < N; i++) {
        c.x[i] = min(a.x[i], b[i]);
        c.y[i] = max(a.y[i], b[i]);
    }
    return c;
}

/// expands a bounding box with a bounding box
template <typename T, int N>
constexpr inline bbox<T, N> expand(const bbox<T, N>& a, const bbox<T, N>& b) {
    bbox<T, N> c;
    for (auto i = 0; i < N; i++) {
        c.x[i] = min(a.x[i], b.x[i]);
        c.y[i] = max(a.y[i], b.y[i]);
    }
    return c;
}

/// check if a bounding box contains a point
template <typename T, int N>
constexpr inline bool contains(const bbox<T, N>& a, const vec<T, N>& b) {
    for (auto i = 0; i < N; i++) {
        if (a.min[i] > b[i] || a.max[i] < b[i]) return false;
    }
    return true;
}

/// check if a bounding box contains a bounding box
template <typename T, int N>
constexpr inline bool contains(const bbox<T, N>& a, const bbox<T, N>& b) {
    for (auto i = 0; i < N; i++) {
        if (a.min[i] > b.max[i] || a.max[i] < b.min[i]) return false;
    }
    return true;
}

/// expands a bounding box with a point
template <>
constexpr inline bbox3f expand(const bbox3f& a, const vec3f& b) {
    return {{min(a.min.x, b.x), min(a.min.y, b.y), min(a.min.z, b.z)},
        {max(a.max.x, b.x), max(a.max.y, b.y), max(a.max.z, b.z)}};
}

/// expands a bounding box with a bounding box
template <>
constexpr inline bbox3f expand(const bbox3f& a, const bbox3f& b) {
    return {
        {min(a.min.x, b.min.x), min(a.min.y, b.min.y), min(a.min.z, b.min.z)},
        {max(a.max.x, b.max.x), max(a.max.y, b.max.y), max(a.max.z, b.max.z)}};
}

/// check if a bounding box contains a point
template <>
constexpr inline bool contains(const bbox3f& a, const vec3f& b) {
    if (a.min.x > b.x || a.max.x < b.x) return false;
    if (a.min.y > b.y || a.max.y < b.y) return false;
    if (a.min.z > b.z || a.max.z < b.z) return false;
    return true;
}

/// check if a bounding box contains a bounding box
template <>
constexpr inline bool contains(const bbox3f& a, const bbox3f& b) {
    if (a.min.x > b.max.x || a.max.x < b.min.x) return false;
    if (a.min.y > b.max.y || a.max.y < b.min.y) return false;
    if (a.min.z > b.max.z || a.max.z < b.min.z) return false;
    return true;
}

/// same as expand()
template <typename T, int N>
constexpr inline bbox<T, N> operator+(const bbox<T, N>& a, const T& b) {
    return expand(a, b);
}

/// same as expand()
template <typename T, int N>
constexpr inline bbox<T, N> operator+(
    const bbox<T, N>& a, const bbox<T, N>& b) {
    return expand(a, b);
}

/// assign to expand()
template <typename T, int N>
constexpr inline bbox<T, N>& operator+=(bbox<T, N>& a, const vec<T, N>& b) {
    return a = expand(a, b);
}

/// assign to expand()
template <typename T, int N>
constexpr inline bbox<T, N>& operator+=(bbox<T, N>& a, const bbox<T, N>& b) {
    return a = expand(a, b);
}

/// stream write
template <typename T, int N>
inline ostream& operator<<(ostream& os, const bbox<T, N>& a) {
    os << a.min << ',' << a.max;
    return os;
}

/// stream read
template <typename T, int N>
inline istream& operator>>(istream& is, bbox<T, N>& a) {
    is >> a.min >> a.max;
    return is;
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// PRIMITIVE BBOX FUNCTIONS
// -----------------------------------------------------------------------------
namespace ygl {

/// Point bounds
inline bbox3f point_bbox(const vec3f& p, float r = 0) {
    return bbox3f{p - vec3f{r, r, r}, p + vec3f{r, r, r}};
}

/// Line bounds
inline bbox3f line_bbox(
    const vec3f& v0, const vec3f& v1, float r0 = 0, float r1 = 0) {
    return make_bbox({v0 - vec3f{r0, r0, r0}, v0 + vec3f{r0, r0, r0},
        v1 - vec3f{r1, r1, r1}, v1 + vec3f{r1, r1, r1}});
}

/// Triangle bounds
inline bbox3f triangle_bbox(const vec3f& v0, const vec3f& v1, const vec3f& v2) {
    return make_bbox({v0, v1, v2});
}

/// Quad bounds
inline bbox3f quad_bbox(
    const vec3f& v0, const vec3f& v1, const vec3f& v2, const vec3f& v3) {
    return make_bbox({v0, v1, v2, v3});
}

/// Tetrahedron bounds
inline bbox3f tetrahedron_bbox(
    const vec3f& v0, const vec3f& v1, const vec3f& v2, const vec3f& v3) {
    return make_bbox({v0, v1, v2, v3});
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// RAYS
// -----------------------------------------------------------------------------
namespace ygl {

/// Rays with origin, direction and min/max t value.
template <typename T, int N>
struct ray {
    /// origin
    vec<T, N> o;
    /// direction
    vec<T, N> d;
    /// minimum distance
    T tmin;
    /// maximum distance
    T tmax;

    /// default constructor
    constexpr ray()
        : o(), d(0, 0, 1), tmin(0), tmax(numeric_limits<T>::max()) {}
    /// initializes a ray from its elements
    constexpr ray(const vec<T, N>& o, const vec<T, N>& d, T tmin = 0,
        T tmax = numeric_limits<T>::max())
        : o(o), d(d), tmin(tmin), tmax(tmax) {}
};

/// Sepcialization for 3D float rays.
template <>
struct ray<float, 3> {
    /// size
    constexpr static const int N = 3;
    /// type
    using T = float;

    /// origin
    vec<T, N> o;
    /// direction
    vec<T, N> d;
    /// minimum distance
    T tmin;
    /// maximum distance
    T tmax;

    /// default constructor
    constexpr ray() : o{0, 0, 0}, d{0, 0, 1}, tmin{0}, tmax{flt_max} {}
    /// initializes a ray from its elements
    constexpr ray(
        const vec<T, N>& o, const vec<T, N>& d, T tmin = 0, T tmax = flt_max)
        : o(o), d(d), tmin(tmin), tmax(tmax) {}
};

/// 1-dimensional float ray
using ray1f = ray<float, 1>;
/// 2-dimensional float ray
using ray2f = ray<float, 2>;
/// 3-dimensional float ray
using ray3f = ray<float, 3>;
/// 4-dimensional float ray
using ray4f = ray<float, 4>;

/// evalutes the position along the ray
template <typename T, int N>
constexpr inline vec<T, N> eval(const ray<T, N>& ray, T t) {
    return ray.o + t * ray.d;
}

/// stream write
template <typename T, int N>
inline ostream& operator<<(ostream& os, const ray<T, N>& a) {
    os << a.o << ' ' << a.d << ' ' << a.tmin << ' ' << a.tmax;
    return os;
}

/// stream read
template <typename T, int N>
inline istream& operator>>(istream& is, ray<T, N>& a) {
    is >> a.o >> a.d >> a.tmin >> a.tmax;
    return is;
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// RAY-PRIMITIVE INTERSECTION FUNCTIONS
// -----------------------------------------------------------------------------
namespace ygl {

/// Intersect a ray with a point (approximate)
///
/// Parameters:
/// - ray: ray origin and direction, parameter min, max range
/// - p: point position
/// - r: point radius
///
/// Out Parameters:
/// - ray_t: ray parameter at the intersection point
/// - euv: primitive uv ( {0,0} for points )
///
/// Returns:
/// - whether the intersection occurred
///
/// Iplementation Notes:
/// - out Parameters and only writtent o if an intersection occurs
/// - algorithm finds the closest point on the ray segment to the point and
///    test their distance with the point radius
/// - based on http://geomalgorithms.com/a02-lines.html.
inline bool intersect_point(
    const ray3f& ray, const vec3f& p, float r, float& ray_t) {
    // find parameter for line-point minimum distance
    auto w = p - ray.o;
    auto t = dot(w, ray.d) / dot(ray.d, ray.d);

    // exit if not within bounds
    if (t < ray.tmin || t > ray.tmax) return false;

    // test for line-point distance vs point radius
    auto rp = eval(ray, t);
    auto prp = p - rp;
    if (dot(prp, prp) > r * r) return false;

    // intersection occurred: set params and exit
    ray_t = t;

    return true;
}

/// Intersect a ray with a line
///
/// Parameters:
/// - ray: ray origin and direction, parameter min, max range
/// - v0, v1: line segment points
/// - r0, r1: line segment radia
///
/// Out Parameters:
/// - ray_t: ray parameter at the intersection point
/// - euv: euv.x is the line parameter at the intersection ( euv.y is zero )
///
/// Returns:
/// - whether the intersection occurred
///
/// Notes:
/// - out Parameters and only writtent o if an intersection occurs
/// - algorithm find the closest points on line and ray segment and test
///   their distance with the line radius at that location
/// - based on http://geomalgorithms.com/a05-intersect-1.html
/// - based on http://geomalgorithms.com/a07-distance.html#
///     dist3D_Segment_to_Segment
inline bool intersect_line(const ray3f& ray, const vec3f& v0, const vec3f& v1,
    float r0, float r1, float& ray_t, vec2f& euv) {
    // setup intersection params
    auto u = ray.d;
    auto v = v1 - v0;
    auto w = ray.o - v0;

    // compute values to solve a linear system
    auto a = dot(u, u);
    auto b = dot(u, v);
    auto c = dot(v, v);
    auto d = dot(u, w);
    auto e = dot(v, w);
    auto det = a * c - b * b;

    // check determinant and exit if lines are parallel
    // (could use EPSILONS if desired)
    if (det == 0) return false;

    // compute Parameters on both ray and segment
    auto t = (b * e - c * d) / det;
    auto s = (a * e - b * d) / det;

    // exit if not within bounds
    if (t < ray.tmin || t > ray.tmax) return false;

    // clamp segment param to segment corners
    s = clamp(s, (float)0, (float)1);

    // compute segment-segment distance on the closest points
    auto p0 = eval(ray, t);
    auto p1 = eval(ray3f{v0, v1 - v0}, s);
    auto p01 = p0 - p1;

    // check with the line radius at the same point
    auto r = r0 * (1 - s) + r1 * s;
    if (dot(p01, p01) > r * r) return false;

    // intersection occurred: set params and exit
    ray_t = t;
    euv = {1 - s, s};

    return true;
}

/// Intersect a ray with a triangle
///
/// Parameters:
/// - ray: ray origin and direction, parameter min, max range
/// - v0, v1, v2: triangle vertices
///
/// Out Parameters:
/// - ray_t: ray parameter at the intersection point
/// - euv: baricentric coordinates of the intersection
///
/// Returns:
/// - whether the intersection occurred
///
/// Notes:
/// - out Parameters and only writtent o if an intersection occurs
/// - algorithm based on Muller-Trombone intersection test
inline bool intersect_triangle(const ray3f& ray, const vec3f& v0,
    const vec3f& v1, const vec3f& v2, float& ray_t, vec3f& euv) {
    // compute triangle edges
    auto edge1 = v1 - v0;
    auto edge2 = v2 - v0;

    // compute determinant to solve a linear system
    auto pvec = cross(ray.d, edge2);
    auto det = dot(edge1, pvec);

    // check determinant and exit if triangle and ray are parallel
    // (could use EPSILONS if desired)
    if (det == 0) return false;
    auto inv_det = 1.0f / det;

    // compute and check first bricentric coordinated
    auto tvec = ray.o - v0;
    auto u = dot(tvec, pvec) * inv_det;
    if (u < 0 || u > 1) return false;

    // compute and check second bricentric coordinated
    auto qvec = cross(tvec, edge1);
    auto v = dot(ray.d, qvec) * inv_det;
    if (v < 0 || u + v > 1) return false;

    // compute and check ray parameter
    auto t = dot(edge2, qvec) * inv_det;
    if (t < ray.tmin || t > ray.tmax) return false;

    // intersection occurred: set params and exit
    ray_t = t;
    euv = {1 - u - v, u, v};

    return true;
}

/// Intersect a ray with a quad represented as two triangles (0,1,3) and
/// (2,3,1), with the uv coordinates of the second triangle corrected by u =
/// 1-u' and v = 1-v' to produce a quad parametrization where u and v go from 0
/// to 1. This is equivalent to Intel's Embree. The external user does not have
/// to be concerned about the parametrization and can just use the euv as
/// specified.
///
/// Parameters:
/// - ray: ray origin and direction, parameter min, max range
/// - v0, v1, v2, v3: quad vertices
///
/// Out Parameters:
/// - ray_t: ray parameter at the intersection point
/// - euv: baricentric coordinates of the intersection
///
/// Returns:
/// - whether the intersection occurred
inline bool intersect_quad(const ray3f& ray, const vec3f& v0, const vec3f& v1,
    const vec3f& v2, const vec3f& v3, float& ray_t, vec4f& euv) {
    auto hit = false;
    auto tray = ray;
    if (intersect_triangle(tray, v0, v1, v3, ray_t, (vec3f&)euv)) {
        euv = {euv.x, euv.y, 0, euv.z};
        tray.tmax = ray_t;
        hit = true;
    }
    if (intersect_triangle(tray, v2, v3, v1, ray_t, (vec3f&)euv)) {
        euv = {0, 1 - euv.y, euv.y + euv.z - 1, 1 - euv.z};
        tray.tmax = ray_t;
        hit = true;
    }
    return hit;
}

/// Intersect a ray with a tetrahedron. Note that we consider only
/// intersection wiht the tetrahedra surface and discount intersction with
/// the interior.
///
/// Parameters:
/// - ray: ray to intersect with
/// - v0, v1, v2: triangle vertices
///
/// Out Parameters:
/// - ray_t: ray parameter at the intersection point
/// - euv: baricentric coordinates of the intersection
///
/// Returns:
/// - whether the intersection occurred
///
/// TODO: check order
/// TODO: uv
inline bool intersect_tetrahedron(const ray3f& ray_, const vec3f& v0,
    const vec3f& v1, const vec3f& v2, const vec3f& v3, float& ray_t,
    vec4f& euv) {
    // check intersction for each face
    auto hit = false;
    auto ray = ray_;
    auto tuv = zero3f;
    if (intersect_triangle(ray, v0, v1, v2, ray_t, tuv)) {
        hit = true;
        ray.tmax = ray_t;
    }
    if (intersect_triangle(ray, v0, v1, v3, ray_t, tuv)) {
        hit = true;
        ray.tmax = ray_t;
    }
    if (intersect_triangle(ray, v0, v2, v3, ray_t, tuv)) {
        hit = true;
        ray.tmax = ray_t;
    }
    if (intersect_triangle(ray, v1, v2, v3, ray_t, tuv)) {
        hit = true;
        ray.tmax = ray_t;
    }

    return hit;
}

/// Intersect a ray with a axis-aligned bounding box
///
/// Parameters:
/// - ray: ray to intersect with
/// - bbox: bounding box min/max bounds
///
/// Returns:
/// - whether the intersection occurred
inline bool intersect_check_bbox(const ray3f& ray, const bbox3f& bbox) {
    // set up convenient pointers for looping over axes
    auto tmin = ray.tmin, tmax = ray.tmax;

    // for each axis, clip intersection against the bounding planes
    for (int i = 0; i < 3; i++) {
        // determine intersection ranges
        auto invd = 1.0f / ray.d[i];
        auto t0 = (bbox.min[i] - ray.o[i]) * invd;
        auto t1 = (bbox.max[i] - ray.o[i]) * invd;
        // flip based on range directions
        if (invd < 0.0f) {
            float a = t0;
            t0 = t1;
            t1 = a;
        }
        // clip intersection
        tmin = t0 > tmin ? t0 : tmin;
        tmax = t1 < tmax ? t1 : tmax;
        // if intersection is empty, exit
        if (tmin > tmax) return false;
    }

    // passed all planes, then intersection occurred
    return true;
}

/// Min/max used in BVH traversal. Copied here since the traversal code
/// relies on the specific behaviour wrt NaNs.
template <typename T>
static inline const T& _safemin(const T& a, const T& b) {
    return (a < b) ? a : b;
}
/// Min/max used in BVH traversal. Copied here since the traversal code
/// relies on the specific behaviour wrt NaNs.
template <typename T>
static inline const T& _safemax(const T& a, const T& b) {
    return (a > b) ? a : b;
}

/// Intersect a ray with a axis-aligned bounding box
///
/// Parameters:
/// - ray_o, ray_d: ray origin and direction
/// - ray_tmin, ray_tmax: ray parameter min, max range
/// - ray_dinv: ray inverse direction
/// - ray_dsign: ray direction sign
/// - bbox_min, bbox_max: bounding box min/max bounds
///
/// Returns:
/// - whether the intersection occurred
///
/// Implementation Notes:
/// - based on "Robust BVH Ray Traversal" by T. Ize published at
/// http://jcgt.org/published/0002/02/02/paper.pdf
inline bool intersect_check_bbox(const ray3f& ray, const vec3f& ray_dinv,
    const vec3i& ray_dsign, const bbox3f& bbox_) {
    auto bbox = &bbox_.min;
    auto txmin = (bbox[ray_dsign.x].x - ray.o.x) * ray_dinv.x;
    auto txmax = (bbox[1 - ray_dsign.x].x - ray.o.x) * ray_dinv.x;
    auto tymin = (bbox[ray_dsign.y].y - ray.o.y) * ray_dinv.y;
    auto tymax = (bbox[1 - ray_dsign.y].y - ray.o.y) * ray_dinv.y;
    auto tzmin = (bbox[ray_dsign.z].z - ray.o.z) * ray_dinv.z;
    auto tzmax = (bbox[1 - ray_dsign.z].z - ray.o.z) * ray_dinv.z;
    auto tmin = _safemax(tzmin, _safemax(tymin, _safemax(txmin, ray.tmin)));
    auto tmax = _safemin(tzmax, _safemin(tymax, _safemin(txmax, ray.tmax)));
    tmax *= 1.00000024f;  // for double: 1.0000000000000004
    return tmin <= tmax;
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// POINT-PRIMITIVE DISTANCE FUNCTIONS
// -----------------------------------------------------------------------------
namespace ygl {

// TODO: documentation
inline bool overlap_point(
    const vec3f& pos, float dist_max, const vec3f& p, float r, float& dist) {
    auto d2 = distsqr(pos, p);
    if (d2 > (dist_max + r) * (dist_max + r)) return false;
    dist = sqrt(d2);
    return true;
}

// TODO: documentation
inline vec2f closestuv_line(
    const vec3f& pos, const vec3f& v0, const vec3f& v1) {
    auto ab = v1 - v0;
    auto d = dot(ab, ab);
    // Project c onto ab, computing parameterized position d(t) = a + t*(b –
    // a)
    auto u = dot(pos - v0, ab) / d;
    u = clamp(u, (float)0, (float)1);
    return {1 - u, u};
}

// TODO: documentation
inline bool overlap_line(const vec3f& pos, float dist_max, const vec3f& v0,
    const vec3f& v1, float r0, float r1, float& dist, vec2f& euv) {
    auto uv = closestuv_line(pos, v0, v1);
    // Compute projected position from the clamped t d = a + t * ab;
    auto p = lerp(v0, v1, uv.y);
    auto r = lerp(r0, r1, uv.y);
    auto d2 = distsqr(pos, p);
    // check distance
    if (d2 > (dist_max + r) * (dist_max + r)) return false;
    // done
    dist = sqrt(d2);
    euv = uv;
    return true;
}

// TODO: documentation
// this is a complicated test -> I probably prefer to use a sequence of test
// (triangle body, and 3 edges)
inline vec3f closestuv_triangle(
    const vec3f& pos, const vec3f& v0, const vec3f& v1, const vec3f& v2) {
    auto ab = v1 - v0;
    auto ac = v2 - v0;
    auto ap = pos - v0;

    auto d1 = dot(ab, ap);
    auto d2 = dot(ac, ap);

    // corner and edge cases
    if (d1 <= 0 && d2 <= 0) return vec3f{1, 0, 0};

    auto bp = pos - v1;
    auto d3 = dot(ab, bp);
    auto d4 = dot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return vec3f{0, 1, 0};

    auto vc = d1 * d4 - d3 * d2;
    if ((vc <= 0) && (d1 >= 0) && (d3 <= 0))
        return vec3f{1 - d1 / (d1 - d3), d1 / (d1 - d3), 0};

    auto cp = pos - v2;
    auto d5 = dot(ab, cp);
    auto d6 = dot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return vec3f{0, 0, 1};

    auto vb = d5 * d2 - d1 * d6;
    if ((vb <= 0) && (d2 >= 0) && (d6 <= 0))
        return vec3f{1 - d2 / (d2 - d6), 0, d2 / (d2 - d6)};

    auto va = d3 * d6 - d5 * d4;
    if ((va <= 0) && (d4 - d3 >= 0) && (d5 - d6 >= 0)) {
        auto w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return vec3f{0, 1 - w, w};
    }

    // face case
    auto denom = 1 / (va + vb + vc);
    auto v = vb * denom;
    auto w = vc * denom;
    return vec3f{1 - v - w, v, w};
}

// TODO: documentation
inline bool overlap_triangle(const vec3f& pos, float dist_max, const vec3f& v0,
    const vec3f& v1, const vec3f& v2, float r0, float r1, float r2, float& dist,
    vec3f& euv) {
    auto uv = closestuv_triangle(pos, v0, v1, v2);
    auto p = eval_barycentric_triangle(v0, v1, v2, uv);
    auto r = eval_barycentric_triangle(r0, r1, r2, uv);
    auto dd = distsqr(p, pos);
    if (dd > (dist_max + r) * (dist_max + r)) return false;
    dist = sqrt(dd);
    euv = uv;
    return true;
}

// TODO: documentation
inline bool overlap_quad(const vec3f& pos, float dist_max, const vec3f& v0,
    const vec3f& v1, const vec3f& v2, const vec3f& v3, float r0, float r1,
    float r2, float r3, float& dist, vec4f& euv) {
    auto hit = false;
    if (overlap_triangle(
            pos, dist_max, v0, v1, v3, r0, r1, r3, dist, (vec3f&)euv)) {
        euv = {euv.x, euv.y, 0, euv.z};
        dist_max = dist;
        hit = true;
    }
    if (overlap_triangle(
            pos, dist_max, v2, v3, v1, r2, r3, r1, dist, (vec3f&)euv)) {
        // dist_max = dist;
        euv = {0, 1 - euv.y, euv.y + euv.z - 1, 1 - euv.z};
        hit = true;
    }
    return hit;
}

// TODO: documentation
inline bool overlap_tetrahedron(const vec3f& pos, const vec3f& v0,
    const vec3f& v1, const vec3f& v2, const vec3f& v3, vec4f& euv) {
    auto vol = dot(v3 - v0, cross(v3 - v1, v3 - v0));
    if (vol == 0) return false;
    auto u = dot(v3 - v0, cross(v3 - v1, v3 - v0)) / vol;
    if (u < 0 || u > 1) return false;
    auto v = dot(v3 - v0, cross(v3 - v1, v3 - v0)) / vol;
    if (v < 0 || v > 1 || u + v > 1) return false;
    auto w = dot(v3 - v0, cross(v3 - v1, v3 - v0)) / vol;
    if (w < 0 || w > 1 || u + v + w > 1) return false;
    euv = {u, v, w, 1 - u - v - w};
    return true;
}

// TODO: documentation
inline bool overlap_tetrahedron(const vec3f& pos, float dist_max,
    const vec3f& v0, const vec3f& v1, const vec3f& v2, const vec3f& v3,
    float r0, float r1, float r2, float r3, float& dist, vec4f& euv) {
    // check interior
    if (overlap_tetrahedron(pos, v0, v1, v2, v3, euv)) {
        dist = 0;
        return true;
    }

    // check faces
    auto hit = false;
    auto tuv = zero3f;
    if (overlap_triangle(pos, dist_max, v0, v1, v2, r0, r1, r2, dist, tuv)) {
        hit = true;
        dist_max = dist;
    }
    if (overlap_triangle(pos, dist_max, v0, v1, v3, r0, r1, r3, dist, tuv)) {
        hit = true;
        dist_max = dist;
    }
    if (overlap_triangle(pos, dist_max, v0, v2, v3, r0, r2, r3, dist, tuv)) {
        hit = true;
        dist_max = dist;
    }
    if (overlap_triangle(pos, dist_max, v1, v2, v3, r1, r2, r3, dist, tuv)) {
        hit = true;
        // dist_max = dist;
    }

    return hit;
}

// TODO: documentation
inline bool distance_check_bbox(
    const vec3f& pos, float dist_max, const bbox3f& bbox) {
    // computing distance
    auto dd = 0.0f;

    // For each axis count any excess distance outside box extents
    for (int i = 0; i < 3; i++) {
        auto v = pos[i];
        if (v < bbox.min[i]) dd += (bbox.min[i] - v) * (bbox.min[i] - v);
        if (v > bbox.max[i]) dd += (v - bbox.max[i]) * (v - bbox.max[i]);
    }

    // check distance
    return dd < dist_max * dist_max;
}

// TODO: doc
inline bool overlap_bbox(const bbox3f& bbox1, const bbox3f& bbox2) {
    if (bbox1.max.x < bbox2.min.x || bbox1.min.x > bbox2.max.x) return false;
    if (bbox1.max.y < bbox2.min.y || bbox1.min.y > bbox2.max.y) return false;
    if (bbox1.max.z < bbox2.min.z || bbox1.min.z > bbox2.max.z) return false;
    return true;
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// TRANSFORMS
// -----------------------------------------------------------------------------
namespace ygl {

/// transforms a point by a matrix
template <typename T, int N>
constexpr inline vec<T, N> transform_point(
    const mat<T, N + 1, N + 1>& a, const vec<T, N>& b) {
    // make it generic
    auto vb = vec<T, N + 1>();
    (vec<T, N>&)vb = b;
    vb[N] = 1;
    auto tvb = a * vb;
    return *(vec<T, N>*)(&tvb) / tvb[N];
}

/// transforms a vector by a matrix
template <typename T, int N>
constexpr inline vec<T, N> transform_vector(
    const mat<T, N + 1, N + 1>& a, const vec<T, N>& b) {
    // make it generic
    auto vb = vec<T, N + 1>();
    (vec<T, N>&)vb = b;
    vb[N] = 0;
    auto tvb = a * vb;
    return *(vec<T, N>*)(&tvb);
}

/// transforms a direction by a matrix
template <typename T, int N>
constexpr inline vec<T, N> transform_direction(
    const mat<T, N + 1, N + 1>& a, const vec<T, N>& b) {
    return normalize(transform_vector(a, b));
}

/// transforms a point by a frame (rigid affine transform)
template <typename T, int N>
constexpr inline vec<T, N> transform_point(
    const frame<T, N>& a, const vec<T, N>& b) {
    return rot(a) * b + pos(a);
}

/// transforms a vector by a frame (rigid affine transform)
template <typename T, int N>
constexpr inline vec<T, N> transform_vector(
    const frame<T, N>& a, const vec<T, N>& b) {
    return rot(a) * b;
}

/// transforms a direction by a frame (rigid affine transform)
template <typename T, int N>
constexpr inline vec<T, N> transform_direction(
    const frame<T, N>& a, const vec<T, N>& b) {
    return rot(a) * b;
}

/// transforms a frame by a frame (rigid affine transform)
template <typename T, int N>
constexpr inline frame<T, N> transform_frame(
    const frame<T, N>& a, const frame<T, N>& b) {
    return {rot(a) * rot(b), pos(a) * pos(b) + pos(a)};
}

/// inverse transforms a point by a frame (rigid affine transform)
template <typename T, int N>
constexpr inline vec<T, N> transform_point_inverse(
    const frame<T, N>& a, const vec<T, N>& b) {
    return (b - pos(a)) * rot(a);
}

/// inverse transforms a vector by a frame (rigid affine transform)
template <typename T, int N>
constexpr inline vec<T, N> transform_vector_inverse(
    const frame<T, N>& a, const vec<T, N>& b) {
    return b * rot(a);
}

/// inverse transforms a direction by a frame (rigid affine transform)
template <typename T, int N>
constexpr inline vec<T, N> transform_direction_inverse(
    const frame<T, N>& a, const vec<T, N>& b) {
    return b * rot(a);
}

/// transforms a point by a matrix
template <>
constexpr inline vec3f transform_point(const mat4f& a, const vec3f& b) {
    auto vb = vec4f{b.x, b.y, b.z, 1};
    auto tvb = a * vb;
    return vec3f{tvb.x, tvb.y, tvb.z} / tvb.w;
}

/// transforms a vector by a matrix
template <>
constexpr inline vec3f transform_vector(const mat4f& a, const vec3f& b) {
    auto vb = vec4f{b.x, b.y, b.z, 0};
    auto tvb = a * vb;
    return vec3f{tvb.x, tvb.y, tvb.z};
}

/// transforms a direction by a matrix
template <>
constexpr inline vec3f transform_direction(const mat4f& a, const vec3f& b) {
    return normalize(transform_vector(a, b));
}

/// transforms a point by a frame (rigid affine transform)
template <>
constexpr inline vec3f transform_point(const frame3f& a, const vec3f& b) {
    return a.rot() * b + a.pos();
}

/// transforms a vector by a frame (rigid affine transform)
template <>
constexpr inline vec3f transform_vector(const frame3f& a, const vec3f& b) {
    return a.rot() * b;
}

/// transforms a direction by a frame (rigid affine transform)
template <>
constexpr inline vec3f transform_direction(const frame3f& a, const vec3f& b) {
    return a.rot() * b;
}

/// transforms a frame by a frame (rigid affine transform)
template <>
constexpr inline frame3f transform_frame(const frame3f& a, const frame3f& b) {
    return {a.rot() * b.rot(), a.rot() * b.pos() + a.pos()};
}

/// inverse transforms a point by a frame (rigid affine transform)
template <>
constexpr inline vec3f transform_point_inverse(
    const frame3f& a, const vec3f& b) {
    return (b - a.pos()) * a.rot();
}

/// inverse transforms a vector by a frame (rigid affine transform)
template <>
constexpr inline vec3f transform_vector_inverse(
    const frame3f& a, const vec3f& b) {
    return b * a.rot();
}

/// inverse transforms a direction by a frame (rigid affine transform)
template <>
constexpr inline vec3f transform_direction_inverse(
    const frame3f& a, const vec3f& b) {
    return b * a.rot();
}

/// transforms a ray by a matrix (direction is not normalized after)
template <typename T, int N>
constexpr inline ray<T, N> transform_ray(
    const mat<T, N + 1, N + 1>& a, const ray<T, N>& b) {
    return {transform_point(a, b.o), transform_vector(a, b.d), b.tmin, b.tmax};
}

/// transforms a bbox by a matrix
template <typename T>
constexpr inline bbox<T, 3> transform_bbox(
    const mat<T, 4, 4>& a, const bbox<T, 3>& b) {
    vec<T, 3> corners[8] = {
        {b.min.x, b.min.y, b.min.z},
        {b.min.x, b.min.y, b.max.z},
        {b.min.x, b.max.y, b.min.z},
        {b.min.x, b.max.y, b.max.z},
        {b.max.x, b.min.y, b.min.z},
        {b.max.x, b.min.y, b.max.z},
        {b.max.x, b.max.y, b.min.z},
        {b.max.x, b.max.y, b.max.z},
    };
    auto xformed = bbox<T, 3>();
    for (auto j = 0; j < 8; j++) xformed += transform_point(a, corners[j]);
    return xformed;
}

/// transforms a ray by a frame (rigid affine transform)
template <typename T, int N>
constexpr inline ray<T, N> transform_ray(
    const frame<T, N>& a, const ray<T, N>& b) {
    return {
        transform_point(a, b.o), transform_direction(a, b.d), b.tmin, b.tmax};
}

/// transforms a bbox by a frame (rigid affine transform)
template <typename T>
constexpr inline bbox<T, 3> transform_bbox(
    const frame<T, 3>& a, const bbox<T, 3>& b) {
#if 0
    vec<T, 3> corners[8] = {
        {b.min.x, b.min.y, b.min.z}, {b.min.x, b.min.y, b.max.z},
        {b.min.x, b.max.y, b.min.z}, {b.min.x, b.max.y, b.max.z},
        {b.max.x, b.min.y, b.min.z}, {b.max.x, b.min.y, b.max.z},
        {b.max.x, b.max.y, b.min.z}, {b.max.x, b.max.y, b.max.z},
    };
    auto xformed = bbox<T, 3>();
    for (auto j = 0; j < 8; j++) xformed += transform_point(a, corners[j]);
    return xformed;
#else
    // Code from Real-time Collision Detection by Christer Ericson Sect. 4.2.6
    // Transform AABB a by the matrix m and translation t,
    // find maximum extents, and store result into AABB b.
    // start by adding in translation
    auto c = bbox<T, 3>{pos(a), pos(a)};
    // for all three axes
    for (auto i = 0; i < 3; i++) {
        // form extent by summing smaller and larger terms respectively
        for (auto j = 0; j < 3; j++) {
            auto e = a.rot()[j][i] * b.min[j];
            auto f = a.rot()[j][i] * b.max[j];
            if (e < f) {
                c.min[i] += e;
                c.max[i] += f;
            } else {
                c.min[i] += f;
                c.max[i] += e;
            }
        }
    }
    return c;
#endif
}

/// inverse transforms a ray by a frame (rigid affine transform)
template <typename T, int N>
constexpr inline ray<T, N> transform_ray_inverse(
    const frame<T, N>& a, const ray<T, N>& b) {
    return {transform_point_inverse(a, b.o),
        transform_direction_inverse(a, b.d), b.tmin, b.tmax};
}

/// inverse transforms a bbox by a frame (rigid affine transform)
template <typename T>
constexpr inline bbox<T, 3> transform_bbox_inverse(
    const frame<T, 3>& a, const bbox<T, 3>& b) {
    return transform_bbox(inverse(a), b);
}

/// rotation matrix from axis-angle
template <typename T>
constexpr inline mat<T, 3, 3> rotation_mat3(const vec<T, 3>& axis, T angle) {
    auto s = sin(angle), c = cos(angle);
    auto vv = normalize(axis);
    return {{c + (1 - c) * vv.x * vv.x, (1 - c) * vv.x * vv.y + s * vv.z,
                (1 - c) * vv.x * vv.z - s * vv.y},
        {(1 - c) * vv.x * vv.y - s * vv.z, c + (1 - c) * vv.y * vv.y,
            (1 - c) * vv.y * vv.z + s * vv.x},
        {(1 - c) * vv.x * vv.z + s * vv.y, (1 - c) * vv.y * vv.z - s * vv.x,
            c + (1 - c) * vv.z * vv.z}};
}

/// translation frame
template <typename T>
constexpr inline frame<T, 3> translation_frame3(const vec<T, 3>& a) {
    return {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, a};
}

/// translation matrix
template <typename T>
constexpr inline mat<T, 4, 4> translation_mat4(const vec<T, 3>& a) {
    return to_mat(translation_frame3(a));
}

/// scaling frame (this is not rigid and it is only here for symmatry of
/// API)
template <typename T>
constexpr inline frame<T, 3> scaling_frame3(const vec<T, 3>& a) {
    return {{a.x, 0, 0}, {0, a.y, 0}, {0, 0, a.z}, {0, 0, 0}};
}

/// scaling matrix
template <typename T>
constexpr inline mat<T, 4, 4> scaling_mat4(const vec<T, 3>& a) {
    return to_mat(scaling_frame3(a));
}

/// rotation frame
template <typename T>
constexpr inline frame<T, 3> rotation_frame3(const vec<T, 3>& axis, T angle) {
    return {rotation_mat3(axis, angle), {0, 0, 0}};
}

/// rotation matrix
template <typename T>
constexpr inline mat<T, 4, 4> rotation_mat4(const mat<T, 3, 3>& rot) {
    return mat<T, 4, 4>{{rot.x.x, rot.x.y, rot.x.z, 0},
        {rot.y.x, rot.y.y, rot.y.z, 0}, {rot.z.x, rot.z.y, rot.z.z, 0},
        {0, 0, 0, 1}};
}

/// rotation matrix
template <typename T>
constexpr inline mat<T, 4, 4> rotation_mat4(const vec<T, 3>& axis, T angle) {
    return rotation_mat4(rotation_frame3(axis, angle).rot());
}

/// quaternion axis-angle conversion
template <typename T>
constexpr inline vec<T, 4> rotation_axisangle4(const quat<T, 4>& a) {
    auto axis = normalize(vec<T, 3>{a.x, a.y, a.z});
    auto angle = acos(a.w) * 2;
    return {axis.x, axis.y, axis.z, angle};
}

/// axis-angle to quaternion
template <typename T>
constexpr inline quat<T, 4> rotation_quat4(const vec<T, 4>& axis_angle) {
    auto axis = vec<T, 3>{axis_angle.x, axis_angle.y, axis_angle.z};
    auto len = lenght(axis);
    auto angle = atan2(len, axis_angle.w);
    if (len)
        axis /= len;
    else
        axis = {0, 0, 1};
    return {axis.x, axis.y, axis.z, angle};
}

/// quaterion to matrix conversion
template <typename T>
constexpr inline mat<T, 3, 3> rotation_mat3(const quat<T, 4>& v) {
    return {{v.w * v.w + v.x * v.x - v.y * v.y - v.z * v.z,
                (v.x * v.y + v.z * v.w) * 2, (v.z * v.x - v.y * v.w) * 2},
        {(v.x * v.y - v.z * v.w) * 2,
            v.w * v.w - v.x * v.x + v.y * v.y - v.z * v.z,
            (v.y * v.z + v.x * v.w) * 2},
        {(v.z * v.x + v.y * v.w) * 2, (v.y * v.z - v.x * v.w) * 2,
            v.w * v.w - v.x * v.x - v.y * v.y + v.z * v.z}};
}

/// rotation matrix
template <typename T>
constexpr inline mat<T, 4, 4> rotation_mat4(const quat<T, 4>& v) {
    return rotation_mat4(rotation_mat3(v));
}

// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
/// matrix to quaternion
template <typename T>
constexpr inline quat<T, 4> rotation_quat4(const mat<T, 3, 3>& m_) {
    auto q = quat<T, 4>();
    auto m = transpose(m_);
#if 1
    auto trace = m.x.x + m.y.y + m.z.z;
    if (trace > 0) {
        float s = (T)0.5 / sqrt(trace + 1);
        q.w = (T)0.25 / s;
        q.x = (m.z.y - m.y.z) * s;
        q.y = (m.x.z - m.z.x) * s;
        q.z = (m.y.x - m.x.y) * s;
    } else {
        if (m.x.x > m.y.y && m.x.x > m.z.z) {
            float s = 2 * sqrt(max((T)0, 1 + m.x.x - m.y.y - m.z.z));
            q.w = (m.z.y - m.y.z) / s;
            q.x = (T)0.25 * s;
            q.y = (m.x.y + m.y.x) / s;
            q.z = (m.x.z + m.z.x) / s;
        } else if (m.y.y > m.z.z) {
            float s = 2 * sqrt(max((T)0, 1 + m.y.y - m.x.x - m.z.z));
            q.w = (m.x.z - m.z.x) / s;
            q.x = (m.x.y + m.y.x) / s;
            q.y = (T)0.25 * s;
            q.z = (m.y.z + m.z.y) / s;
        } else {
            float s = 2 * sqrt(max((T)0, 1 + m.z.z - m.x.x - m.y.y));
            q.w = (m.y.x - m.x.y) / s;
            q.x = (m.x.z + m.z.x) / s;
            q.y = (m.y.z + m.z.y) / s;
            q.z = (T)0.25 * s;
        }
    }

#else
    q.w = sqrt(max(0, 1 + m.x.x + m.y.y + m.z.z)) / 2;
    q.x = sqrt(max(0, 1 + m.x.x - m.y.y - m.z.z)) / 2;
    q.y = sqrt(max(0, 1 - m.x.x + m.y.y - m.z.z)) / 2;
    q.z = sqrt(max(0, 1 - m.x.x - m.y.y + m.z.z)) / 2;
    Q.x = copysign(q.x, m.z.y - m.y.z);
    Q.y = copysign(q.y, m.x.z - m.z.x);
    Q.z = copysign(q.z, m.y.x - m.x.y);
#endif

    return q;
}

/// OpenGL lookat frame
template <typename T>
constexpr inline frame<T, 3> lookat_frame3(
    const vec<T, 3>& eye, const vec<T, 3>& center, const vec<T, 3>& up) {
    auto w = normalize(eye - center);
    auto u = normalize(cross(up, w));
    auto v = normalize(cross(w, u));
    return {u, v, w, eye};
}

/// OpenGL lookat matrix
template <typename T>
constexpr inline mat<T, 4, 4> lookat_mat4(
    const vec<T, 3>& eye, const vec<T, 3>& center, const vec<T, 3>& up) {
    return to_mat(lookat_frame3(eye, center, up));
}

/// OpenGL frustum matrix
template <typename T>
constexpr inline mat<T, 4, 4> frustum_mat4(T l, T r, T b, T t, T n, T f) {
    return {{2 * n / (r - l), 0, 0, 0}, {0, 2 * n / (t - b), 0, 0},
        {(r + l) / (r - l), (t + b) / (t - b), -(f + n) / (f - n), -1},
        {0, 0, -2 * f * n / (f - n), 0}};
}

/// OpenGL orthographic matrix
template <typename T>
constexpr inline mat<T, 4, 4> ortho_mat4(T l, T r, T b, T t, T n, T f) {
    return {{2 / (r - l), 0, 0, 0}, {0, 2 / (t - b), 0, 0},
        {0, 0, -2 / (f - n), 0},
        {-(r + l) / (r - l), -(t + b) / (t - b), -(f + n) / (f - n), 1}};
}

/// OpenGL orthographic 2D matrix
template <typename T>
constexpr inline mat<T, 4, 4> ortho2d_mat4(T left, T right, T bottom, T top) {
    return ortho_mat4(left, right, bottom, top, -1, 1);
}

/// OpenGL/GLTF orthographic matrix
template <typename T>
constexpr inline mat<T, 4, 4> ortho_mat4(T xmag, T ymag, T near, T far) {
    return {{1 / xmag, 0, 0, 0}, {0, 1 / ymag, 0, 0},
        {0, 0, 2 / (near - far), 0}, {0, 0, (far + near) / (near - far), 1}};
}

/// OpenGL/GLTF perspective matrix
template <typename T>
constexpr inline mat<T, 4, 4> perspective_mat4(
    T fovy, T aspect, T near, T far) {
    auto tg = tan(fovy / 2);
    return {{1 / (aspect * tg), 0, 0, 0}, {0, 1 / tg, 0, 0},
        {0, 0, (far + near) / (near - far), -1},
        {0, 0, 2 * far * near / (near - far), 0}};
}

/// OpenGL/GLTF infinite perspective matrix
template <typename T>
constexpr inline mat<T, 4, 4> perspective_mat4(T fovy, T aspect, T near) {
    auto tg = tan(fovy / 2);
    return {{1 / (aspect * tg), 0, 0, 0}, {0, 1 / tg, 0, 0}, {0, 0, -1, -1},
        {0, 0, 2 * near, 0}};
}

/// Decompose an affine matrix into translation, rotation, scale.
/// Assumes there is no shear and the matrix is affine.
template <typename T>
constexpr inline void decompose_mat4(const mat<T, 4, 4>& m,
    vec<T, 3>& translation, mat<T, 3, 3>& rotation, vec<T, 3>& scale) {
    translation = {m.w.x, m.w.y, m.w.z};
    rotation.x = {m.x.x, m.x.y, m.x.z};
    rotation.y = {m.y.x, m.y.y, m.y.z};
    rotation.z = {m.z.x, m.z.y, m.z.z};
    scale = {length(rotation.x), length(rotation.y), length(rotation.z)};
    rotation = {
        normalize(rotation.x), normalize(rotation.y), normalize(rotation.z)};
}

/// Decompose an affine matrix into translation, rotation, scale.
/// Assumes there is no shear and the matrix is affine.
template <typename T>
constexpr inline void decompose_mat4(const mat<T, 4, 4>& m,
    vec<T, 3>& translation, quat<T, 4>& rotation, vec<T, 3>& scale) {
    auto rot_matrix = mat<T, 3, 3>();
    decompose_mat4(m, translation, rot_matrix, scale);
    rotation = to_quat4(rotation_mat4(rot_matrix));
}

/// Decompose an affine matrix into translation, rotation, scale.
/// Assumes there is no shear and the matrix is affine.
template <typename T>
constexpr inline mat<T, 4, 4> compose_mat4(const vec<T, 3>& translation,
    const mat<T, 3, 3>& rotation, const vec<T, 3>& scale) {
    return translation_mat4(translation) * scaling_mat4(scale) *
           rotation_mat4(rotation);
}

/// Decompose an affine matrix into translation, rotation, scale.
/// Assumes there is no shear and the matrix is affine.
template <typename T>
constexpr inline mat<T, 4, 4> compose_mat4(const vec<T, 3>& translation,
    const quat<T, 4>& rotation, const vec<T, 3>& scale) {
    return translation_mat4(translation) * scaling_mat4(scale) *
           rotation_mat4(rotation);
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// UI UTILITIES
// -----------------------------------------------------------------------------
namespace ygl {

/// Turntable for UI navigation from a from/to/up parametrization of the
/// camera.
constexpr inline void camera_turntable(vec3f& from, vec3f& to, vec3f& up,
    const vec3f& rotate, float dolly, const vec3f& pan) {
    // rotate if necessary
    if (rotate.x || rotate.y) {
        auto z = normalize(to - from);
        auto lz = dist(to, from);
        auto phi = atan2(z.z, z.x) + rotate.x;
        auto theta = acos(z.y) + rotate.y;
        theta = clamp(theta, 0.001f, pif - 0.001f);
        auto nz = vec3f{sin(theta) * cos(phi) * lz, cos(theta) * lz,
            sin(theta) * sin(phi) * lz};
        from = to - nz;
    }

    // dolly if necessary
    if (dolly) {
        auto z = normalize(to - from);
        auto lz = max(0.001f, dist(to, from) * (1 + dolly));
        z *= lz;
        from = to - z;
    }

    // pan if necessary
    if (pan.x || pan.y) {
        auto z = normalize(to - from);
        auto x = normalize(cross(up, z));
        auto y = normalize(cross(z, x));
        auto t = vec3f{pan.x * x.x + pan.y * y.x, pan.x * x.y + pan.y * y.y,
            pan.x * x.z + pan.y * y.z};
        from += t;
        to += t;
    }
}

/// Turntable for UI navigation for a frame/distance parametrization of the
/// camera.
constexpr inline void camera_turntable(frame3f& frame, float& focus,
    const vec2f& rotate, float dolly, const vec2f& pan) {
    // rotate if necessary
    if (rotate.x || rotate.y) {
        auto phi = atan2(frame.z.z, frame.z.x) + rotate.x;
        auto theta = acos(frame.z.y) + rotate.y;
        theta = clamp(theta, 0.001f, pif - 0.001f);
        auto new_z =
            vec3f{sin(theta) * cos(phi), cos(theta), sin(theta) * sin(phi)};
        auto new_center = pos(frame) - frame.z * focus;
        auto new_o = new_center + new_z * focus;
        frame = lookat_frame3(new_o, new_center, {0, 1, 0});
        focus = dist(new_o, new_center);
    }

    // pan if necessary
    if (dolly) {
        auto c = pos(frame) - frame.z * focus;
        focus = max(focus + dolly, 0.001f);
        pos(frame) = c + frame.z * focus;
    }

    // pan if necessary
    if (pan.x || pan.y) { pos(frame) += frame.x * pan.x + frame.y * pan.y; }
}

/// FPS camera for UI navigation for a frame parametrization.
/// https://gamedev.stackexchange.com/questions/30644/how-to-keep-my-quaternion-using-fps-camera-from-tilting-and-messing-up
constexpr inline void camera_fps(
    frame3f& frame, const vec3f& transl, const vec2f& rotate) {
    auto y = vec3f{0, 1, 0};
    auto z = orthonormalize(frame.z, y);
    auto x = cross(y, z);

    frame.rot() = rotation_mat3({1, 0, 0}, rotate.y) * frame.rot() *
                  rotation_mat3({0, 1, 0}, rotate.x);
    frame.pos() += transl.x * x + transl.y * y + transl.z * z;
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// RANDOM NUMBER GENERATION
// -----------------------------------------------------------------------------
namespace ygl {

/// PCG random numbers. A family of random number generators that supports
/// multiple sequences. In our code, we allocate one sequence for each sample.
/// PCG32 from http://www.pcg-random.org/
struct rng_pcg32 {
    /// RNG state.
    uint64_t state = 0x853c49e6748fea9bULL;
    /// RNG sequence. Must be odd.
    uint64_t inc = 0xda3e39cb94b95bdbULL;
};

/// Next random number
constexpr inline uint32_t advance_rng(rng_pcg32& rng) {
    uint64_t oldstate = rng.state;
    rng.state = oldstate * 6364136223846793005ULL + rng.inc;
    uint32_t xorshifted = (uint32_t)(((oldstate >> 18u) ^ oldstate) >> 27u);
    uint32_t rot = (uint32_t)(oldstate >> 59u);
    return (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
    // return (xorshifted >> rot) | (xorshifted << ((~rot + 1u) & 31));
}

/// Multi-step advance function (jump-ahead, jump-back).
constexpr inline void advance_rng(rng_pcg32& rng, uint64_t delta) {
    // The method used here is based on Brown, "Random Number Generation
    // with Arbitrary Stride", Transactions of the American Nuclear
    // Society (Nov. 1994). The algorithm is very similar to fast
    // exponentiation.
    uint64_t cur_mult = 6364136223846793005ULL, cur_plus = rng.inc,
             acc_mult = 1u, acc_plus = 0u;

    while (delta > 0) {
        if (delta & 1) {
            acc_mult *= cur_mult;
            acc_plus = acc_plus * cur_mult + cur_plus;
        }
        cur_plus = (cur_mult + 1) * cur_plus;
        cur_mult *= cur_mult;
        delta /= 2;
    }
    rng.state = acc_mult * rng.state + acc_plus;
}

/// Multi-step advance function (jump-ahead, jump-back).
constexpr inline void advance_rng(rng_pcg32& rng, int64_t delta) {
    // Even though delta is an unsigned integer, we can pass a signed
    // integer to go backwards, it just goes "the long way round".
    advance_rng(rng, (uint64_t)delta);
}

/// Seeds a random number generator with a state state from the sequence seq.
constexpr inline void seed_rng(
    rng_pcg32& rng, uint64_t state, uint64_t seq = 1) {
    rng.state = 0U;
    rng.inc = (seq << 1u) | 1u;
    advance_rng(rng);
    rng.state += state;
    advance_rng(rng);
}

/// Init a random number generator with a state state from the sequence seq.
constexpr inline rng_pcg32 init_rng(uint64_t state, uint64_t seq = 1) {
    auto rng = rng_pcg32();
    seed_rng(rng, state, seq);
    return rng;
}

/// Next random uint in [0,n) range with proper weighting
constexpr inline uint32_t next_rand1i(rng_pcg32& rng, uint32_t n) {
#if YGL_RNG_FASTUINT
    return advance_rng(rng) % n;
#else
    // To avoid bias, we need to make the range of the RNG a multiple of
    // bound, which we do by dropping output less than a threshold.
    // A naive scheme to calculate the threshold would be to do
    //
    //     uint32_t threshold = 0x100000000ull % bound;
    //
    // but 64-bit div/mod is slower than 32-bit div/mod (especially on
    // 32-bit platforms).  In essence, we do
    //
    //     uint32_t threshold = (0x100000000ull-bound) % bound;
    //
    // because this version will calculate the same modulus, but the LHS
    // value is less than 2^32.
    uint32_t threshold = (~n + 1u) % n;

    // Uniformity guarantees that this loop will terminate.  In practice, it
    // should usually terminate quickly; on average (assuming all bounds are
    // equally likely), 82.25% of the time, we can expect it to require just
    // one iteration.  In the worst case, someone passes a bound of 2^31 + 1
    // (i.e., 2147483649), which invalidates almost 50% of the range.  In
    // practice, bounds are typically small and only a tiny amount of the
    // range is eliminated.
    while (true) {
        uint32_t r = advance_rng(rng);
        if (r >= threshold) return r % n;
    }
#endif
}

/// Next random float in [0,1).
inline float next_rand1f(rng_pcg32& rng) {
#if 1
    // Trick from MTGP: generate an uniformly distributed
    // single precision number in [1,2) and subtract 1.
    union {
        uint32_t u;
        float f;
    } x;
    x.u = (advance_rng(rng) >> 9) | 0x3f800000u;
    return x.f - 1.0f;
#else
    constexpr const static auto scale =
        (float)(1.0 / numeric_limits<uint32_t>::max());
    return advance_rng(rng) * scale;
#endif
}

/// Next random double in [0, 1). Only 32 mantissa bits are filled, but still
/// better than float that uses 23.
inline double next_rand1d(rng_pcg32& rng) {
#if 1
    // Trick from MTGP: generate an uniformly distributed
    // double precision number in [1,2) and subtract 1.
    union {
        uint64_t u;
        double d;
    } x;
    x.u = ((uint64_t)advance_rng(rng) << 20) | 0x3ff0000000000000ull;
    return x.d - 1.0;
#else
    constexpr const static auto scale =
        (double)(1.0 / numeric_limits<uint32_t>::max());
    return advance_rng(rng) * scale;
#endif
}

/// Distance between random number generators
constexpr inline int64_t rng_distance(const rng_pcg32& a, const rng_pcg32& b) {
    assert(a.inc == b.inc);

    uint64_t cur_mult = 6364136223846793005ULL, cur_plus = a.inc,
             cur_state = b.state, the_bit = 1u, distance = 0u;

    while (a.state != cur_state) {
        if ((a.state & the_bit) != (cur_state & the_bit)) {
            cur_state = cur_state * cur_mult + cur_plus;
            distance |= the_bit;
        }
        assert((a.state & the_bit) == (cur_state & the_bit));
        the_bit <<= 1;
        cur_plus = (cur_mult + 1ULL) * cur_plus;
        cur_mult *= cur_mult;
    }

    return (int64_t)distance;
}

/// Random shuffle of a sequence.
template <typename Iterator>
constexpr inline void rng_shuffle(
    rng_pcg32& rng, Iterator begin, Iterator end) {
    // Draw uniformly distributed permutation and permute the
    // given STL container. From Knuth, TAoCP Vol.2(3rd 3d),
    // Section 3.4.2
    for (Iterator it = end - 1; it > begin; --it)
        std::iter_swap(
            it, begin + next_rand1i(rng, (uint32_t)(it - begin + 1)));
}

/// Random shuffle of a sequence.
template <typename T>
constexpr inline void rng_shuffle(rng_pcg32& rng, T* vals, int num) {
    // Draw uniformly distributed permutation and permute the
    // given STL container
    for (auto i = num - 1; i > 0; --i)
        swap(vals[i], vals[next_rand1i(rng, (uint32_t)(i - 1))]);
}

/// Random shuffle of a sequence.
template <typename T>
constexpr inline void rng_shuffle(rng_pcg32& rng, vector<T>& vals) {
    shuffle(rng, vals.data(), vals.size());
}

/// Equality operator
constexpr inline bool operator==(const rng_pcg32& a, const rng_pcg32& b) {
    return a.state == b.state && a.inc == b.inc;
}

/// Inequality operator
constexpr inline bool operator!=(const rng_pcg32& a, const rng_pcg32& b) {
    return a.state != b.state || a.inc != b.inc;
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// MONETACARLO SAMPLING FUNCTIONS
// -----------------------------------------------------------------------------
namespace ygl {

/// sample hemispherical direction with uniform distribution
inline vec3f sample_hemisphere(const vec2f& ruv) {
    auto z = ruv.y;
    auto r = sqrt(1 - z * z);
    auto phi = 2 * pif * ruv.x;
    return vec3f(r * cos(phi), r * sin(phi), z);
}

/// pdf for hemispherical direction with uniform distribution
inline float sample_hemisphere_pdf(const vec3f& w) {
    return (w.z <= 0) ? 0 : 1 / (2 * pif);
}

/// spherical direction with uniform distribution
inline vec3f sample_sphere(const vec2f ruv) {
    auto z = 2 * ruv.y - 1;
    auto r = sqrt(1 - z * z);
    auto phi = 2 * pif * ruv.x;
    return vec3f(r * cos(phi), r * sin(phi), z);
}

/// pdf for spherical direction with uniform distribution
inline float sample_sphere_pdf(const vec3f& w) { return 1 / (4 * pif); }

/// hemispherical direction with cosine distribution
inline vec3f sample_hemisphere_cosine(const vec2f& ruv) {
    auto z = sqrt(ruv.y);
    auto r = sqrt(1 - z * z);
    auto phi = 2 * pif * ruv.x;
    return vec3f(r * cos(phi), r * sin(phi), z);
}

/// pdf for hemispherical direction with cosine distribution
inline float sample_hemisphere_cosine_pdf(const vec3f& w) {
    return (w.z <= 0) ? 0 : w.z / pif;
}

/// hemispherical direction with cosine power distribution
inline vec3f sample_hemisphere_cospower(const vec2f& ruv, float n) {
    auto z = pow(ruv.y, 1 / (n + 1));
    auto r = sqrt(1 - z * z);
    auto phi = 2 * pif * ruv.x;
    return vec3f(r * cos(phi), r * sin(phi), z);
}

/// pdf for hemispherical direction with cosine power distribution
inline float sample_hemisphere_cospower_pdf(const vec3f& w, float n) {
    return (w.z <= 0) ? 0 : pow(w.z, n) * (n + 1) / (2 * pif);
}

/// uniform disk
inline vec3f sample_disk(const vec2f& ruv) {
    auto r = sqrt(ruv.y);
    auto phi = 2 * pif * ruv.x;
    return vec3f(cos(phi) * r, sin(phi) * r, 0);
}

/// pdf for uniform disk
inline float sample_disk_pdf() { return 1 / pif; }

/// uniform cylinder
inline vec3f sample_cylinder(const vec2f& ruv) {
    auto phi = 2 * pif * ruv.x;
    return vec3f(sin(phi), cos(phi), ruv.y * 2 - 1);
}

/// pdf for uniform cylinder
inline float sample_cylinder_pdf() { return 1 / pif; }

/// uniform triangle
inline vec2f sample_triangle(const vec2f& ruv) {
    return {1 - sqrt(ruv.x), ruv.y * sqrt(ruv.x)};
}

/// uniform triangle
inline vec3f sample_triangle(
    const vec2f& ruv, const vec3f& v0, const vec3f& v1, const vec3f& v2) {
    auto uv = sample_triangle(ruv);
    return v0 * (1 - uv.x - uv.y) + v1 * uv.x + v2 * uv.y;
}

/// pdf for uniform triangle (triangle area)
inline float sample_triangle_pdf(
    const vec3f& v0, const vec3f& v1, const vec3f& v2) {
    return 2 / length(cross(v1 - v0, v2 - v0));
}

/// index with uniform distribution
inline int sample_index(float r, int size) {
    return clamp((int)(r * size), 0, size - 1);
}

/// pdf for index with uniform distribution
inline float sample_index_pdf(int size) { return 1.0f / size; }

}  // namespace ygl

// -----------------------------------------------------------------------------
// HASHING
// -----------------------------------------------------------------------------
namespace ygl {

/// Computes the i-th term of a permutation of l values keyed by p.
/// From Correlated Multi-Jittered Sampling by Kensler @ Pixar
constexpr inline uint32_t hash_permute(uint32_t i, uint32_t n, uint32_t key) {
    uint32_t w = n - 1;
    w |= w >> 1;
    w |= w >> 2;
    w |= w >> 4;
    w |= w >> 8;
    w |= w >> 16;
    do {
        i ^= key;
        i *= 0xe170893du;
        i ^= key >> 16;
        i ^= (i & w) >> 4;
        i ^= key >> 8;
        i *= 0x0929eb3f;
        i ^= key >> 23;
        i ^= (i & w) >> 1;
        i *= 1 | key >> 27;
        i *= 0x6935fa69;
        i ^= (i & w) >> 11;
        i *= 0x74dcb303;
        i ^= (i & w) >> 2;
        i *= 0x9e501cc3;
        i ^= (i & w) >> 2;
        i *= 0xc860a3df;
        i &= w;
        i ^= i >> 5;
    } while (i >= n);
    return (i + key) % n;
}

/// Computes a float value by hashing i with a key p.
/// From Correlated Multi-Jittered Sampling by Kensler @ Pixar
constexpr inline float hash_randfloat(uint32_t i, uint32_t key) {
    i ^= key;
    i ^= i >> 17;
    i ^= i >> 10;
    i *= 0xb36534e5;
    i ^= i >> 12;
    i ^= i >> 21;
    i *= 0x93fc4795;
    i ^= 0xdf6e307f;
    i ^= i >> 17;
    i *= 1 | key >> 18;
    return i * (1.0f / 4294967808.0f);
}

/// 32 bit integer hash. Public domain code.
constexpr inline uint32_t hash_uint32(uint64_t a) {
    a -= (a << 6);
    a ^= (a >> 17);
    a -= (a << 9);
    a ^= (a << 4);
    a -= (a << 3);
    a ^= (a << 10);
    a ^= (a >> 15);
    return a;
}

/// 64 bit integer hash. Public domain code.
constexpr inline uint64_t hash_uint64(uint64_t a) {
    a = (~a) + (a << 21);  // a = (a << 21) - a - 1;
    a ^= (a >> 24);
    a += (a << 3) + (a << 8);  // a * 265
    a ^= (a >> 14);
    a += (a << 2) + (a << 4);  // a * 21
    a ^= (a >> 28);
    a += (a << 31);
    return a;
}

/// 64-to-32 bit integer hash. Public domain code.
constexpr inline uint32_t hash_uint64_32(uint64_t a) {
    a = (~a) + (a << 18);  // a = (a << 18) - a - 1;
    a ^= (a >> 31);
    a *= 21;  // a = (a + (a << 2)) + (a << 4);
    a ^= (a >> 11);
    a += (a << 6);
    a ^= (a >> 22);
    return (uint32_t)a;
}

/// Combines two 64 bit hashes as in boost::hash_combine
constexpr inline size_t hash_combine(size_t a, size_t b) {
    return a ^ (b + 0x9e3779b9 + (a << 6) + (a >> 2));
}

/// Hash a vector with hash_combine() and std::hash
template <typename T, int N>
constexpr inline int hash_vec(const vec<T, N>& v) {
    std::hash<T> Th;
    int h = 0;
    for (auto i = 0; i < N; i++) {
        h ^= (Th(v[i]) + 0x9e3779b9 + (h << 6) + (h >> 2));
    }
    return h;
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// SHAPE UTILITIES
// -----------------------------------------------------------------------------
namespace ygl {

/// Compute smoothed tangents.
///
/// Parameters:
/// - nverts/pos: array pf vertex positions
/// - nlines/lines: array of point indices
/// - weighted: whether to use area weighting (typically true)
///
/// Out Parameters:
/// - tang: array of computed tangents
inline void compute_tangents(const vector<vec2i>& lines,
    const vector<vec3f>& pos, vector<vec3f>& tang, bool weighted = true) {
    // clear tangents
    tang.assign(pos.size(), zero3f);

    // handle lines
    for (auto& l : lines) {
        auto n = pos[l.y] - pos[l.x];
        if (!weighted) n = normalize(n);
        tang[l.x] += n;
        tang[l.y] += n;
    }

    // normalize result
    for (auto& t : tang) t = normalize(t);
}

/// Compute smoothed normals.
///
/// Parameters:
/// - nverts/pos: array pf vertex positions
/// - triangles: array of triangle indices
/// - weighted: whether to use area weighting (typically true)
///
/// Out Parameters:
/// - norm: array of computed normals
inline void compute_normals(const vector<vec3i>& triangles,
    const vector<vec3f>& pos, vector<vec3f>& norm, bool weighted = true) {
    // clear normals
    norm.assign(pos.size(), zero3f);

    // handle triangles
    for (auto& t : triangles) {
        auto n = cross(pos[t.y] - pos[t.x], pos[t.z] - pos[t.x]);
        if (!weighted) n = normalize(n);
        norm[t.x] += n;
        norm[t.y] += n;
        norm[t.z] += n;
    }

    // normalize result
    for (auto& n : norm) n = normalize(n);
}

/// Compute smoothed normals.
///
/// Parameters:
/// - nverts/pos: array pf vertex positions
/// - nquads/quads: array of point indices
/// - weighted: whether to use area weighting (typically true)
///
/// Out Parameters:
/// - norm: array of computed normals
inline void compute_normals(const vector<vec4i>& quads,
    const vector<vec3f>& pos, vector<vec3f>& norm, bool weighted = true) {
    // clear normals
    norm.assign(pos.size(), zero3f);

    // handle triangles
    for (auto& q : quads) {
        auto n = cross(pos[q.y] - pos[q.x], pos[q.w] - pos[q.x]) +
                 cross(pos[q.w] - pos[q.z], pos[q.x] - pos[q.z]);
        if (!weighted) n = normalize(n);
        norm[q.x] += n;
        norm[q.y] += n;
        norm[q.z] += n;
        norm[q.w] += n;
    }

    // normalize result
    for (auto& n : norm) n = normalize(n);
}

/// Compute tangent frame for triangle mesh. Tangent space is defined by
/// a four component vector. The first three components are the tangent
/// with respect to the U texcoord. The fourth component is the sign of the
/// tangent wrt the V texcoord. Tangent frame is useful in normal mapping.
///
/// Parameters:
/// - nverts/pos: array pf vertex positions
/// - ntriangles/triangles: array of point indices
/// - weighted: whether to use area weighting (typically true)
///
/// Out Parameters:
/// - tangsp: array of computed tangent space
inline void compute_tangent_frame(const vector<vec3i>& triangles,
    const vector<vec3f>& pos, const vector<vec3f>& norm,
    const vector<vec2f>& texcoord, vector<vec4f>& tangsp,
    bool weighted = true) {
    auto tangu = vector<vec3f>(pos.size(), zero3f);
    auto tangv = vector<vec3f>(pos.size(), zero3f);

    for (auto& t : triangles) {
        auto tutv = triangle_tangents_fromuv(pos[t.x], pos[t.y], pos[t.z],
            texcoord[t.x], texcoord[t.y], texcoord[t.z]);
        if (!weighted) tutv = {normalize(tutv.first), normalize(tutv.second)};
        tangu[t.x] += tutv.first;
        tangu[t.y] += tutv.first;
        tangu[t.z] += tutv.first;
        tangv[t.x] += tutv.second;
        tangv[t.y] += tutv.second;
        tangv[t.z] += tutv.second;
    }

    for (auto& t : tangu) t = normalize(t);
    for (auto& t : tangv) t = normalize(t);

    // clear normals
    tangsp.assign(pos.size(), zero4f);

    for (auto i = 0; i < pos.size(); i++) {
        tangu[i] = orthonormalize(tangu[i], norm[i]);
        auto s = (dot(cross(norm[i], tangu[i]), tangv[i]) < 0) ? -1.0f : 1.0f;
        tangsp[i] = {tangu[i].x, tangu[i].y, tangu[i].z, s};
    }
}

/// Apply skinning
inline void compute_skinning(int nverts, const vec3f* pos, const vec3f* norm,
    const vec4f* weights, const vec4i* joints, const mat4f* xforms,
    vec3f* skinned_pos, vec3f* skinned_norm) {}

/// Apply skinning
inline void compute_skinning(const vector<vec3f>& pos,
    const vector<vec3f>& norm, const vector<vec4f>& weights,
    const vector<vec4i>& joints, const vector<mat4f>& xforms,
    vector<vec3f>& skinned_pos, vector<vec3f>& skinned_norm) {
    skinned_pos.resize(pos.size());
    skinned_norm.resize(norm.size());
    for (auto i = 0; i < pos.size(); i++) {
        skinned_pos[i] =
            transform_point(xforms[joints[i].x], pos[i]) * weights[i].x +
            transform_point(xforms[joints[i].y], pos[i]) * weights[i].y +
            transform_point(xforms[joints[i].z], pos[i]) * weights[i].z +
            transform_point(xforms[joints[i].w], pos[i]) * weights[i].w;
    }
    for (auto i = 0; i < pos.size(); i++) {
        skinned_norm[i] = normalize(
            transform_direction(xforms[joints[i].x], norm[i]) * weights[i].x +
            transform_direction(xforms[joints[i].y], norm[i]) * weights[i].y +
            transform_direction(xforms[joints[i].z], norm[i]) * weights[i].z +
            transform_direction(xforms[joints[i].w], norm[i]) * weights[i].w);
    }
}

/// Apply skinning
inline void compute_skinning(const vector<vec3f>& pos,
    const vector<vec3f>& norm, const vector<vec4f>& weights,
    const vector<vec4i>& joints, const vector<frame3f>& xforms,
    vector<vec3f>& skinned_pos, vector<vec3f>& skinned_norm) {
    skinned_pos.resize(pos.size());
    skinned_norm.resize(norm.size());
    for (auto i = 0; i < pos.size(); i++) {
        skinned_pos[i] =
            transform_point(xforms[joints[i].x], pos[i]) * weights[i].x +
            transform_point(xforms[joints[i].y], pos[i]) * weights[i].y +
            transform_point(xforms[joints[i].z], pos[i]) * weights[i].z +
            transform_point(xforms[joints[i].w], pos[i]) * weights[i].w;
    }
    for (auto i = 0; i < pos.size(); i++) {
        skinned_norm[i] = normalize(
            transform_direction(xforms[joints[i].x], norm[i]) * weights[i].x +
            transform_direction(xforms[joints[i].y], norm[i]) * weights[i].y +
            transform_direction(xforms[joints[i].z], norm[i]) * weights[i].z +
            transform_direction(xforms[joints[i].w], norm[i]) * weights[i].w);
    }
}

/// Apply skinning as specified in Khronos glTF
inline void compute_matrix_skinning(const vector<vec3f>& pos,
    const vector<vec3f>& norm, const vector<vec4f>& weights,
    const vector<vec4i>& joints, const vector<mat4f>& xforms,
    vector<vec3f>& skinned_pos, vector<vec3f>& skinned_norm) {
    skinned_pos.resize(pos.size());
    skinned_norm.resize(norm.size());
    for (auto i = 0; i < pos.size(); i++) {
        auto xform = xforms[joints[i].x] * weights[i].x +
                     xforms[joints[i].y] * weights[i].y +
                     xforms[joints[i].z] * weights[i].z +
                     xforms[joints[i].w] * weights[i].w;
        skinned_pos[i] = transform_point(xform, pos[i]);
        skinned_norm[i] = normalize(transform_direction(xform, norm[i]));
    }
}

/// Create an array of edges.
inline vector<vec2i> make_edges(const vector<vec2i>& lines,
    const vector<vec3i>& triangles, const vector<vec4i>& quads) {
    auto edges = vector<vec2i>();
    auto edge_map = unordered_map<vec2i, int>();

    auto add_edge = [&edges, &edge_map](const vec2i& e) {
        auto ee = vec2i{min(e.x, e.y), max(e.x, e.y)};
        if (edge_map.find(ee) != edge_map.end()) return;
        // split in two to avoid undefined behaviour
        auto size = (int)edges.size();
        edge_map[ee] = size;
        edges.push_back(ee);
    };

    for (auto l : lines) add_edge(l);
    for (auto t : triangles) {
        add_edge({t.x, t.y});
        add_edge({t.y, t.z});
        add_edge({t.z, t.x});
    }
    for (auto t : quads) {
        add_edge({t.x, t.y});
        add_edge({t.y, t.z});
        add_edge({t.z, t.w});
        add_edge({t.w, t.x});
    }

    return edges;
}

/// Convert quads to triangles
inline vector<vec3i> convert_quads_to_triangles(const vector<vec4i>& quads) {
    auto triangles = vector<vec3i>();
    triangles.reserve(quads.size() * 2);
    for (auto& q : quads) {
        triangles.push_back({q.x, q.y, q.w});
        triangles.push_back({q.z, q.w, q.y});
    }
    return triangles;
}

/// Convert quads to triangles with a diamond-like topology.
/// Quads have to be consecutive one row after another.
inline vector<vec3i> convert_quads_to_triangles(
    const vector<vec4i>& quads, int row_length) {
    auto triangles = vector<vec3i>(quads.size() * 2);
    for (auto i = 0; i < quads.size(); i++) {
        auto q = quads[i];
        triangles[i * 2 + 0] = {q.x, q.y, q.w};
        triangles[i * 2 + 1] = {q.z, q.w, q.y};
    }
#if 0
        triangles.resize(usteps * vsteps * 2);
        for (auto j = 0; j < vsteps; j++) {
            for (auto i = 0; i < usteps; i++) {
                auto& f1 = triangles[(j * usteps + i) * 2 + 0];
                auto& f2 = triangles[(j * usteps + i) * 2 + 1];
                if ((i + j) % 2) {
                    f1 = {vid(i, j), vid(i + 1, j), vid(i + 1, j + 1)};
                    f2 = {vid(i + 1, j + 1), vid(i, j + 1), vid(i, j)};
                } else {
                    f1 = {vid(i, j), vid(i + 1, j), vid(i, j + 1)};
                    f2 = {vid(i + 1, j + 1), vid(i, j + 1), vid(i + 1, j)};
                }
            }
        }
#endif
    return triangles;
}

/// Convert polys to triangles. Makes no attempt to convert properly and just
/// emits triangles in a fan-like fashion. Works only for planar and convex
/// faces.
template <int N>
inline vector<vec3i> convert_polys_to_triangles(
    const vector<svec<int, N>>& polys) {
    auto triangles = vector<vec3i>();
    triangles.reserve(polys.size() * 2);
    for (auto& p : polys) {
        for (auto i = 2; i < p.size(); i++)
            triangles.push_back({p[0], p[i - 1], p[i]});
    }
    return triangles;
}

/// Convert face varying data to single primitives. Returns the quads indices
/// and filled vectors for pos, norm and texcoord.
inline tuple<vector<vec4i>, vector<vec3f>, vector<vec3f>, vector<vec2f>>
convert_face_varying(const vector<vec4i>& quads_pos,
    const vector<vec4i>& quads_norm, const vector<vec4i>& quads_texcoord,
    const vector<vec3f>& pos, const vector<vec3f>& norm,
    const vector<vec2f>& texcoord) {
    // make faces unique
    unordered_map<vec3i, int> vert_map;
    auto quads = vector<vec4i>(quads_pos.size());
    for (auto fid = 0; fid < quads_pos.size(); fid++) {
        for (auto c = 0; c < 4; c++) {
            auto v = vec3i{
                quads_pos[fid][c],
                (!quads_norm.empty()) ? quads_norm[fid][c] : -1,
                (!quads_texcoord.empty()) ? quads_texcoord[fid][c] : -1,
            };
            if (vert_map.find(v) == vert_map.end()) {
                auto s = (int)vert_map.size();
                vert_map[v] = s;
            }
            quads[fid][c] = vert_map.at(v);
        }
    }

    // fill vert data
    auto qpos = vector<vec3f>();
    if (!pos.empty()) {
        qpos.resize(vert_map.size());
        for (auto& kv : vert_map) { qpos[kv.second] = pos[kv.first.x]; }
    }
    auto qnorm = vector<vec3f>();
    if (!norm.empty()) {
        qnorm.resize(vert_map.size());
        for (auto& kv : vert_map) { qnorm[kv.second] = norm[kv.first.y]; }
    }
    auto qtexcoord = vector<vec2f>();
    if (!texcoord.empty()) {
        qtexcoord.resize(vert_map.size());
        for (auto& kv : vert_map) {
            qtexcoord[kv.second] = texcoord[kv.first.z];
        }
    }

    // done
    return {quads, qpos, qnorm, qtexcoord};
}

/// Edge map structure
struct edge_map {
    /// an empty edge map
    edge_map();

    /// initialize the edge map with triangles
    edge_map(const vector<vec3i>& triangles) {
        for (auto& t : triangles) {
            add_edge({t.x, t.y});
            add_edge({t.y, t.z});
            add_edge({t.z, t.x});
        }
    }

    /// initialize the edge map with quads
    edge_map(const vector<vec4i>& quads) {
        for (auto& t : quads) {
            if (t.z == t.w) {
                add_edge({t.x, t.y});
                add_edge({t.y, t.z});
                add_edge({t.z, t.x});
            } else {
                add_edge({t.x, t.y});
                add_edge({t.y, t.z});
                add_edge({t.z, t.w});
                add_edge({t.w, t.x});
            }
        }
    }

    /// initialize the edge map with polys
    template <int N>
    edge_map(const vector<svec<int, N>>& polys) {
        for (auto& p : polys) {
            for (auto i = 0; i < p.size(); i++)
                add_edge({p[i], p[(i + 1) % p.size()]});
        }
    }

    /// size
    auto size() const { return _map.size(); }

    /// iteration
    auto begin() const { return _map.begin(); }
    /// iteration
    auto end() const { return _map.end(); }
    /// iteration
    auto begin() { return _map.begin(); }
    /// iteration
    auto end() { return _map.end(); }

    /// add an edge to the edge map if not already present
    int add_edge(const vec2i& e) {
        auto ee = vec2i{min(e.x, e.y), max(e.x, e.y)};
        auto it = _map.find(ee);
        if (it != _map.end()) return it->second;
        // split in two to avoid undefined behaviour
        auto eid = (int)size();
        _map[ee] = eid;
        return eid;
    }

    /// add an edge to the edge map if not already present
    int operator[](const vec2i& e) { return add_edge(e); }

    /// get the edge index
    int at(const vec2i& e) const {
        return _map.at({min(e.x, e.y), max(e.x, e.y)});
    }

    /// get all edges
    const vector<vec2i> get_edges() const {
        auto edges = vector<vec2i>(size());
        for (auto& kv : _map) edges[kv.second] = kv.first;
        return edges;
    }

    // implementation -------------------------------------------
   private:
    unordered_map<vec2i, int> _map = {};
};

/// Tesselate lines by splitting them in half, giving two output segments
/// for each input segment.
inline void tesselate_lines(vector<vec2i>& lines, vector<vec3f>& pos,
    vector<vec3f>& tang, vector<vec2f>& texcoord, vector<vec4f>& color,
    vector<float>& radius, bool normalize_tangents = true) {
    auto nverts = (int)pos.size();
    auto add_vertices = [&lines, nverts](auto& vert) {
        if (vert.empty()) return;
        vert.resize(vert.size() + lines.size());
        for (auto eid = 0; eid < (int)lines.size(); eid++) {
            auto e = lines[eid];
            vert[nverts + eid] = (vert[e.x] + vert[e.y]) / 2.0f;
        }
    };

    add_vertices(pos);
    add_vertices(tang);
    add_vertices(texcoord);
    add_vertices(color);
    add_vertices(radius);

    if (normalize_tangents) {
        for (auto& n : tang) n = normalize(n);
    }

    auto edge_id = [nverts](int lid) { return nverts + lid; };

    auto tlines = vector<vec2i>(lines.size() * 2);
    for (auto lid = 0; lid < lines.size(); lid++) {
        auto& l = lines[lid];
        tlines[lid * 2 + 0] = {l.x, edge_id(lid)};
        tlines[lid * 2 + 1] = {edge_id(lid), l.y};
    }
    swap(lines, tlines);
}

/// Tesselate triangles by splitting edges, producing 4 triangles per input
/// triangle.
inline void tesselate_triangles(vector<vec3i>& triangles, vector<vec3f>& pos,
    vector<vec3f>& norm, vector<vec2f>& texcoord, vector<vec4f>& color,
    vector<float>& radius, bool normalize_normals = true) {
    auto nverts = (int)max(pos.size(),
        max(norm.size(),
            max(texcoord.size(), max(color.size(), radius.size()))));

    auto emap = edge_map(triangles);
    for (auto& e_kv : emap) e_kv.second += nverts;

    auto add_vertices = [&emap](auto& vert) {
        if (vert.empty()) return;
        vert.resize(vert.size() + emap.size());
        for (const auto& e_kv : emap) {
            const auto& e = e_kv.first;
            vert[e_kv.second] = (vert[e.x] + vert[e.y]) / 2.0f;
        }
    };

    add_vertices(pos);
    add_vertices(norm);
    add_vertices(texcoord);
    add_vertices(color);
    add_vertices(radius);

    if (normalize_normals) {
        for (auto& n : norm) n = normalize(n);
    }

    auto ttriangles = vector<vec3i>();
    ttriangles.reserve(triangles.size() * 4);
    for (auto& t : triangles) {
        ttriangles.push_back({t.x, emap.at({t.x, t.y}), emap.at({t.z, t.x})});
        ttriangles.push_back({t.y, emap.at({t.y, t.z}), emap.at({t.x, t.y})});
        ttriangles.push_back({t.z, emap.at({t.z, t.x}), emap.at({t.y, t.z})});
        ttriangles.push_back(
            {emap.at({t.x, t.y}), emap.at({t.y, t.z}), emap.at({t.z, t.x})});
    }
    swap(triangles, ttriangles);
}

/// Tesselate quads by splitting faces, producing 4 quads per input quad and
/// 3 quads per input triangle. Handles degenerate quads gracefully.
inline void tesselate_quads(vector<vec4i>& quads, vector<vec3f>& pos,
    vector<vec3f>& norm, vector<vec2f>& texcoord, vector<vec4f>& color,
    vector<float>& radius, bool normalize_normals = true) {
    auto nverts = (int)max(pos.size(),
        max(norm.size(),
            max(texcoord.size(), max(color.size(), radius.size()))));

    auto emap = edge_map(quads);
    for (auto& e_kv : emap) e_kv.second += nverts;

    auto fmap = unordered_map<vec4i, int>();
    for (auto& f : quads) fmap[f] = fmap.size() + nverts + emap.size();

    auto add_vertices = [&emap, &fmap](auto& vert) {
        if (vert.empty()) return;
        vert.resize(vert.size() + emap.size() + fmap.size());
        for (const auto& e_kv : emap) {
            const auto& e = e_kv.first;
            vert[e_kv.second] = (vert[e.x] + vert[e.y]) / 2.0f;
        }
        for (const auto& f_kv : fmap) {
            const auto& f = f_kv.first;
            if (f.z != f.w) {
                vert[f_kv.second] =
                    (vert[f.x] + vert[f.y] + vert[f.z] + vert[f.w]) / 4.0f;
            } else {
                vert[f_kv.second] = (vert[f.x] + vert[f.y] + vert[f.z]) / 3.0f;
            }
        }

    };

    add_vertices(pos);
    add_vertices(norm);
    add_vertices(texcoord);
    add_vertices(color);
    add_vertices(radius);

    if (normalize_normals) {
        for (auto& n : norm) n = normalize(n);
    }

    auto tquads = vector<vec4i>();
    tquads.reserve(quads.size() * 4);
    for (auto& f : quads) {
        if (f.z != f.w) {
            tquads.push_back(
                {f.x, emap.at({f.x, f.y}), fmap.at(f), emap.at({f.w, f.x})});
            tquads.push_back(
                {f.y, emap.at({f.y, f.z}), fmap.at(f), emap.at({f.x, f.y})});
            tquads.push_back(
                {f.z, emap.at({f.z, f.w}), fmap.at(f), emap.at({f.y, f.z})});
            tquads.push_back(
                {f.w, emap.at({f.w, f.x}), fmap.at(f), emap.at({f.z, f.w})});
        } else {
            tquads.push_back(
                {f.x, emap.at({f.x, f.y}), fmap.at(f), emap.at({f.z, f.x})});
            tquads.push_back(
                {f.y, emap.at({f.y, f.z}), fmap.at(f), emap.at({f.x, f.y})});
            tquads.push_back(
                {f.z, emap.at({f.z, f.x}), fmap.at(f), emap.at({f.y, f.z})});
        }
    }
    tquads.shrink_to_fit();
    swap(quads, tquads);
}

/// Tesselate quads by splitting faces, producing 4 quads per input quad and
/// 3 quads per input triangle. Handles degenerate quads gracefully.
inline void tesselate_catmullclark(vector<vec4i>& quads, vector<vec3f>& pos,
    vector<vec3f>& norm, vector<vec2f>& texcoord, vector<vec4f>& color,
    vector<float>& radius, bool normalize_normals = true) {
    auto nverts = (int)max(pos.size(),
        max(norm.size(),
            max(texcoord.size(), max(color.size(), radius.size()))));

    auto emap = edge_map(quads);
    for (auto& e_kv : emap) e_kv.second += nverts;

    auto fmap = unordered_map<vec4i, int>();
    for (auto& f : quads) fmap[f] = fmap.size() + nverts + emap.size();

    auto add_vertices = [&emap, &fmap](auto& vert) {
        if (vert.empty()) return;
        vert.resize(vert.size() + emap.size() + fmap.size());
        for (const auto& e_kv : emap) {
            const auto& e = e_kv.first;
            vert[e_kv.second] = (vert[e.x] + vert[e.y]) / 2.0f;
        }
        for (const auto& f_kv : fmap) {
            const auto& f = f_kv.first;
            if (f.z != f.w) {
                vert[f_kv.second] =
                    (vert[f.x] + vert[f.y] + vert[f.z] + vert[f.w]) / 4.0f;
            } else {
                vert[f_kv.second] = (vert[f.x] + vert[f.y] + vert[f.z]) / 3.0f;
            }
        }

    };

    add_vertices(pos);
    add_vertices(norm);
    add_vertices(texcoord);
    add_vertices(color);
    add_vertices(radius);

    if (normalize_normals) {
        for (auto& n : norm) n = normalize(n);
    }

    auto tquads = vector<vec4i>();
    tquads.reserve(quads.size() * 4);
    for (auto& f : quads) {
        if (f.z != f.w) {
            tquads.push_back(
                {f.x, emap.at({f.x, f.y}), fmap.at(f), emap.at({f.w, f.x})});
            tquads.push_back(
                {f.y, emap.at({f.y, f.z}), fmap.at(f), emap.at({f.x, f.y})});
            tquads.push_back(
                {f.z, emap.at({f.z, f.w}), fmap.at(f), emap.at({f.y, f.z})});
            tquads.push_back(
                {f.w, emap.at({f.w, f.x}), fmap.at(f), emap.at({f.z, f.w})});
        } else {
            tquads.push_back(
                {f.x, emap.at({f.x, f.y}), fmap.at(f), emap.at({f.z, f.x})});
            tquads.push_back(
                {f.y, emap.at({f.y, f.z}), fmap.at(f), emap.at({f.x, f.y})});
            tquads.push_back(
                {f.z, emap.at({f.z, f.x}), fmap.at(f), emap.at({f.y, f.z})});
        }
    }
    tquads.shrink_to_fit();
    swap(quads, tquads);

    // correct vertex data
    auto smooth_vertices = [&quads](auto& vert, auto zero) {
        if (vert.empty()) return;
        // averaging pass ----------------------------------
        // TEMPLATE PROBLEM HERE - FIX IT WITHOUT ELEGANCE
        // auto avg = vector<T>(vert.size(), T());
        auto avg = vert;
        for (auto& v : avg) v = zero;
        auto count = vector<int>(vert.size(), 0);
        for (auto& f : quads) {
            auto fc = (vert[f.x] + vert[f.y] + vert[f.z] + vert[f.w]) / 4.0f;
            for (auto vid : f) avg[vid] += fc;
            for (auto vid : f) count[vid] += 1;
        }
        for (auto i = 0; i < vert.size(); i++) { avg[i] /= (float)count[i]; }

        // correction pass ----------------------------------
        // p = p + (avg_p - p) * (4/avg_count)
        for (auto i = 0; i < vert.size(); i++) {
            vert[i] = vert[i] + (avg[i] - vert[i]) * (4.0f / count[i]);
        }
    };

    smooth_vertices(pos, zero3f);
    smooth_vertices(norm, zero3f);
    smooth_vertices(texcoord, zero2f);
    smooth_vertices(color, zero4f);
    smooth_vertices(radius, 0.0f);

    if (normalize_normals) {
        for (auto& n : norm) n = normalize(n);
    }
}

/// Generate a parametric surface with callbacks.
///
/// Parameters:
/// - usteps: subdivisions in u
/// - vsteps: subdivisions in v
/// - as_triangles: whether to use triangles or quads
/// - pos_fn: pos callbacks (vec2f -> vec3f)
/// - norm_fn: norm callbacks (vec2f -> vec3f)
/// - texcoord_fn: texcoord callbacks (vec2f -> vec2f)
///
/// Out Parameters:
/// - triangles: element array
/// - quads: element array
/// - pos/norm/texcoord: vertex position/normal/texcoords
template <typename PosFunc, typename NormFunc, typename TexcoordFunc>
inline void make_faces(int usteps, int vsteps, vector<vec4i>& quads,
    vector<vec3f>& pos, vector<vec3f>& norm, vector<vec2f>& texcoord,
    const PosFunc& pos_fn, const NormFunc& norm_fn,
    const TexcoordFunc& texcoord_fn) {
    auto vid = [usteps](int i, int j) { return j * (usteps + 1) + i; };
    pos.resize((usteps + 1) * (vsteps + 1));
    norm.resize((usteps + 1) * (vsteps + 1));
    texcoord.resize((usteps + 1) * (vsteps + 1));
    for (auto j = 0; j <= vsteps; j++) {
        for (auto i = 0; i <= usteps; i++) {
            auto uv = vec2f{i / (float)usteps, j / (float)vsteps};
            pos[vid(i, j)] = pos_fn(uv);
            norm[vid(i, j)] = norm_fn(uv);
            texcoord[vid(i, j)] = texcoord_fn(uv);
        }
    }

    quads.resize(usteps * vsteps);
    for (auto j = 0; j < vsteps; j++) {
        for (auto i = 0; i < usteps; i++) {
            quads[j * usteps + i] = {
                vid(i, j), vid(i + 1, j), vid(i + 1, j + 1), vid(i, j + 1)};
        }
    }
}

/// Generate a parametric surface with callbacks.
///
/// Parameters:
/// - usteps: subdivisions in u
/// - vsteps: subdivisions in v
/// - pos_fn: pos callbacks (vec2f -> vec3f)
/// - norm_fn: norm callbacks (vec2f -> vec3f)
/// - texcoord_fn: texcoord callbacks (vec2f -> vec2f)
///
/// Out Parameters:
/// - triangles: element array
/// - pos/norm/texcoord: vertex position/normal/texcoords
template <typename PosFunc, typename NormFunc, typename TexcoordFunc>
inline void make_triangles(int usteps, int vsteps, vector<vec3i>& triangles,
    vector<vec3f>& pos, vector<vec3f>& norm, vector<vec2f>& texcoord,
    const PosFunc& pos_fn, const NormFunc& norm_fn,
    const TexcoordFunc& texcoord_fn) {
    auto quads = vector<vec4i>();
    make_faces(usteps, vsteps, quads, pos, norm, texcoord, pos_fn, norm_fn,
        texcoord_fn);
    triangles = convert_quads_to_triangles(quads, usteps);
}

/// Generate a parametric surface with callbacks.
///
/// Parameters:
/// - usteps: subdivisions in u
/// - vsteps: subdivisions in v
/// - pos_fn: pos callbacks (vec2f -> vec3f)
/// - norm_fn: norm callbacks (vec2f -> vec3f)
/// - texcoord_fn: texcoord callbacks (vec2f -> vec2f)
///
/// Out Parameters:
/// - quads: element array
/// - pos/norm/texcoord: vertex position/normal/texcoords
template <typename PosFunc, typename NormFunc, typename TexcoordFunc>
inline void make_quads(int usteps, int vsteps, vector<vec4i>& quads,
    vector<vec3f>& pos, vector<vec3f>& norm, vector<vec2f>& texcoord,
    const PosFunc& pos_fn, const NormFunc& norm_fn,
    const TexcoordFunc& texcoord_fn) {
    return make_faces(usteps, vsteps, quads, pos, norm, texcoord, pos_fn,
        norm_fn, texcoord_fn);
}

/// Generate parametric lines with callbacks.
///
/// Parameters:
/// - usteps: subdivisions in u
/// - num: number of lines
/// - pos_fn: pos callbacks ((int, float) -> vec3f)
/// - tang_fn: tangent callbacks ((int, float) -> vec3f)
/// - texcoord_fn: texcoord callbacks ((int, float) -> vec2f)
/// - radius_fn: radius callbacks ((int, float) -> float)
///
/// Out Parameters:
/// - lines: element array
/// - pos/tang/texcoord/radius: vertex position/tangent/texcoords/radius
template <typename PosFunc, typename TangFunc, typename TexcoordFunc,
    typename RadiusFunc>
inline void make_lines(int num, int usteps, vector<vec2i>& lines,
    vector<vec3f>& pos, vector<vec3f>& tang, vector<vec2f>& texcoord,
    vector<float>& radius, const PosFunc& pos_fn, const TangFunc& tang_fn,
    const TexcoordFunc& texcoord_fn, const RadiusFunc& radius_fn) {
    auto vid = [usteps](int i, int j) { return j * (usteps + 1) + i; };
    pos.resize((usteps + 1) * num);
    tang.resize((usteps + 1) * num);
    texcoord.resize((usteps + 1) * num);
    radius.resize((usteps + 1) * num);
    for (auto j = 0; j < num; j++) {
        for (auto i = 0; i <= usteps; i++) {
            auto u = i / (float)usteps;
            ;
            pos[vid(i, j)] = pos_fn(j, u);
            tang[vid(i, j)] = tang_fn(j, u);
            texcoord[vid(i, j)] = texcoord_fn(j, u);
            radius[vid(i, j)] = radius_fn(j, u);
        }
    }

    lines.resize(usteps * num);
    for (int j = 0; j < num; j++) {
        for (int i = 0; i < usteps; i++) {
            lines[j * usteps + i] = {vid(i, j), vid(i + 1, j)};
        }
    }
}

/// Generate a parametric point set. Mostly here for completeness.
///
/// Parameters:
/// - num: number of points
/// - pos_fn: pos callbacks (int -> vec3f)
/// - norm_fn: norm callbacks (int -> vec3f)
/// - texcoord_fn: texcoord callbacks (int -> vec2f)
/// - radius_fn: radius callbacks (int -> float)
///
/// Out Parameters:
/// - points: element array
/// - pos/norm/texcoord/radius: vertex position/normal/texcoords/radius
template <typename PosFunc, typename NormFunc, typename TexcoordFunc,
    typename RadiusFunc>
inline void make_points(int num, vector<int>& points, vector<vec3f>& pos,
    vector<vec3f>& norm, vector<vec2f>& texcoord, vector<float>& radius,
    const PosFunc& pos_fn, const NormFunc& norm_fn,
    const TexcoordFunc& texcoord_fn, const RadiusFunc& radius_fn) {
    pos.resize(num);
    norm.resize(num);
    texcoord.resize(num);
    radius.resize(num);
    for (auto i = 0; i < num; i++) {
        pos[i] = pos_fn(i);
        norm[i] = norm_fn(i);
        texcoord[i] = texcoord_fn(i);
        radius[i] = radius_fn(i);
    }

    points.resize(num);
    for (auto i = 0; i < num; i++) points[i] = i;
}

/// Merge a triangle mesh into another.
inline void merge_triangles(vector<vec3i>& triangles, vector<vec3f>& pos,
    vector<vec3f>& norm, vector<vec2f>& texcoord,
    const vector<vec3i>& mtriangles, const vector<vec3f>& mpos,
    const vector<vec3f>& mnorm, const vector<vec2f>& mtexcoord) {
    auto o = (int)pos.size();
    for (auto t : mtriangles) triangles.push_back({t.x + o, t.y + o, t.z + o});
    for (auto p : mpos) pos.push_back(p);
    for (auto n : mnorm) norm.push_back(n);
    for (auto t : mtexcoord) texcoord.push_back(t);
}

/// Merge a quad mesh into another.
inline void merge_quads(vector<vec4i>& quads, vector<vec3f>& pos,
    vector<vec3f>& norm, vector<vec2f>& texcoord, const vector<vec4i>& mquads,
    const vector<vec3f>& mpos, const vector<vec3f>& mnorm,
    const vector<vec2f>& mtexcoord) {
    auto o = (int)pos.size();
    for (auto q : mquads) quads.push_back({q.x + o, q.y + o, q.z + o, q.w + o});
    for (auto p : mpos) pos.push_back(p);
    for (auto n : mnorm) norm.push_back(n);
    for (auto t : mtexcoord) texcoord.push_back(t);
}

/// Unshare shape data by duplicating all vertex data for each element,
/// giving a faceted look. Note that faceted tangents are not computed.
inline void facet_lines(vector<vec2i>& lines, vector<vec3f>& pos,
    vector<vec3f>& norm, vector<vec2f>& texcoord, vector<vec4f>& color,
    vector<float>& radius) {
    auto npos = vector<vec3f>();
    auto nnorm = vector<vec3f>();
    auto ntexcoord = vector<vec2f>();
    auto ncolor = vector<vec4f>();
    auto nradius = vector<float>();

    auto nlines = vector<vec2i>();
    for (auto l : lines) {
        nlines.push_back({(int)npos.size(), (int)npos.size() + 1});
        for (auto v : l) {
            if (!pos.empty()) npos.push_back(pos[v]);
            if (!norm.empty()) nnorm.push_back(norm[v]);
            if (!texcoord.empty()) ntexcoord.push_back(texcoord[v]);
            if (!color.empty()) ncolor.push_back(color[v]);
            if (!radius.empty()) nradius.push_back(radius[v]);
        }
    }

    swap(pos, npos);
    swap(norm, nnorm);
    swap(texcoord, ntexcoord);
    swap(color, ncolor);
    swap(radius, nradius);
    swap(lines, nlines);
}

/// Unshare shape data by duplicating all vertex data for each element,
/// giving a faceted look.
inline void facet_triangles(vector<vec3i>& triangles, vector<vec3f>& pos,
    vector<vec3f>& norm, vector<vec2f>& texcoord, vector<vec4f>& color,
    vector<float>& radius) {
    auto npos = vector<vec3f>();
    auto nnorm = vector<vec3f>();
    auto ntexcoord = vector<vec2f>();
    auto ncolor = vector<vec4f>();
    auto nradius = vector<float>();

    auto ntriangles = vector<vec3i>();

    for (auto t : triangles) {
        ntriangles.push_back(
            {(int)npos.size(), (int)npos.size() + 1, (int)npos.size() + 2});
        for (auto v : t) {
            if (!pos.empty()) npos.push_back(pos[v]);
            if (!norm.empty()) nnorm.push_back(norm[v]);
            if (!texcoord.empty()) ntexcoord.push_back(texcoord[v]);
            if (!color.empty()) ncolor.push_back(color[v]);
            if (!radius.empty()) nradius.push_back(radius[v]);
        }
    }

    swap(pos, npos);
    swap(norm, nnorm);
    swap(texcoord, ntexcoord);
    swap(color, ncolor);
    swap(radius, nradius);
    swap(triangles, ntriangles);
}

/// Unshare shape data by duplicating all vertex data for each element,
/// giving a faceted look.
inline void facet_quads(vector<vec4i>& quads, vector<vec3f>& pos,
    vector<vec3f>& norm, vector<vec2f>& texcoord, vector<vec4f>& color,
    vector<float>& radius) {
    auto npos = vector<vec3f>();
    auto nnorm = vector<vec3f>();
    auto ntexcoord = vector<vec2f>();
    auto ncolor = vector<vec4f>();
    auto nradius = vector<float>();

    auto nquads = vector<vec4i>();

    for (auto q : quads) {
        nquads.push_back({(int)npos.size(), (int)npos.size() + 1,
            (int)npos.size() + 2, (int)npos.size() + 3});
        for (auto v : q) {
            if (!pos.empty()) npos.push_back(pos[v]);
            if (!norm.empty()) nnorm.push_back(norm[v]);
            if (!texcoord.empty()) ntexcoord.push_back(texcoord[v]);
            if (!color.empty()) ncolor.push_back(color[v]);
            if (!radius.empty()) nradius.push_back(radius[v]);
        }
    }

    swap(pos, npos);
    swap(norm, nnorm);
    swap(texcoord, ntexcoord);
    swap(color, ncolor);
    swap(radius, nradius);
    swap(quads, nquads);
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// SHAPE SAMPLING
// -----------------------------------------------------------------------------
namespace ygl {

/// Pick a point
inline int sample_points(int npoints, float re) {
    return clamp(0, npoints - 1, (int)(re * npoints));
}

/// Compute a distribution for sampling points uniformly
inline vector<float> sample_points_cdf(int npoints) {
    auto cdf = vector<float>(npoints);
    for (auto i = 0; i < npoints; i++) cdf[i] = i + 1;
    return cdf;
}

/// Pick a point
inline int sample_points(const vector<float>& cdf, float re) {
    re = clamp(re * cdf.back(), 0.0f, cdf.back() - 0.00001f);
    return (int)(std::upper_bound(cdf.begin(), cdf.end(), re) - cdf.begin());
}

/// Compute a distribution for sampling lines uniformly
inline vector<float> sample_lines_cdf(
    const vector<vec2i>& lines, const vector<vec3f>& pos) {
    auto cdf = vector<float>(lines.size());
    for (auto i = 0; i < lines.size(); i++)
        cdf[i] = length(pos[lines[i].x] - pos[lines[i].y]);
    for (auto i = 1; i < lines.size(); i++) cdf[i] += cdf[i - 1];
    return cdf;
}

/// Pick a point on lines
inline pair<int, vec2f> sample_lines(
    const vector<float>& cdf, float re, float ruv) {
    re = clamp(re * cdf.back(), 0.0f, cdf.back() - 0.00001f);
    auto eid =
        (int)(std::upper_bound(cdf.begin(), cdf.end(), re) - cdf.begin());
    return {eid, {1 - ruv, ruv}};
}

/// Compute a distribution for sampling triangle meshes uniformly
inline vector<float> sample_triangles_cdf(
    const vector<vec3i>& triangles, const vector<vec3f>& pos) {
    auto cdf = vector<float>(triangles.size());
    for (auto i = 0; i < triangles.size(); i++)
        cdf[i] = triangle_area(
            pos[triangles[i].x], pos[triangles[i].y], pos[triangles[i].z]);
    for (auto i = 1; i < triangles.size(); i++) cdf[i] += cdf[i - 1];
    return cdf;
}

/// Pick a point on a triangle mesh
inline pair<int, vec3f> sample_triangles(
    const vector<float>& cdf, float re, const vec2f& ruv) {
    re = clamp(re * cdf.back(), 0.0f, cdf.back() - 0.00001f);
    auto eid =
        (int)(std::upper_bound(cdf.begin(), cdf.end(), re) - cdf.begin());
    return {
        eid, {sqrt(ruv.x) * (1 - ruv.y), 1 - sqrt(ruv.x), ruv.y * sqrt(ruv.x)}};
}

/// Compute a distribution for sampling quad meshes uniformly
inline vector<float> sample_quads_cdf(
    const vector<vec4i>& quads, const vector<vec3f>& pos) {
    auto cdf = vector<float>(quads.size());
    for (auto i = 0; i < quads.size(); i++)
        cdf[i] = quad_area(
            pos[quads[i].x], pos[quads[i].y], pos[quads[i].z], pos[quads[i].w]);
    for (auto i = 1; i < quads.size(); i++) cdf[i] += cdf[i - 1];
    return cdf;
}

/// Pick a point on a quad mesh
inline pair<int, vec4f> sample_quads(
    const vector<float>& cdf, float re, const vec2f& ruv) {
    if (ruv.x < 0.5f) {
        auto eid = 0;
        auto euv = zero3f;
        std::tie(eid, euv) = sample_triangles(cdf, re, {ruv.x * 2, ruv.y});
        return {eid, {euv.x, euv.y, 0, euv.z}};
    } else {
        auto eid = 0;
        auto euv = zero3f;
        std::tie(eid, euv) =
            sample_triangles(cdf, re, {(ruv.x - 0.5f) * 2, ruv.y});
        return {eid, {0, euv.z, euv.x, euv.y}};
    }
}

/// Samples a set of points over a triangle mesh uniformly. The rng function
/// takes the point index and returns vec3f numbers uniform directibuted in
/// [0,1]^3. ù norm and texcoord are optional.
inline void sample_triangles_points(const vector<vec3i>& triangles,
    const vector<vec3f>& pos, const vector<vec3f>& norm,
    const vector<vec2f>& texcoord, int npoints, vector<vec3f>& sampled_pos,
    vector<vec3f>& sampled_norm, vector<vec2f>& sampled_texcoord,
    uint64_t seed) {
    sampled_pos.resize(npoints);
    if (!norm.empty()) sampled_norm.resize(npoints);
    if (!texcoord.empty()) sampled_texcoord.resize(npoints);
    auto cdf = sample_triangles_cdf(triangles, pos);
    auto rng = init_rng(seed);
    for (auto i = 0; i < npoints; i++) {
        auto eid = 0;
        auto euv = zero3f;
        std::tie(eid, euv) = sample_triangles(
            cdf, next_rand1f(rng), {next_rand1f(rng), next_rand1f(rng)});
        auto t = triangles[eid];
        sampled_pos[i] = pos[t.x] * euv.x + pos[t.y] * euv.y + pos[t.z] * euv.z;
        if (!sampled_norm.empty())
            sampled_norm[i] = normalize(
                norm[t.x] * euv.x + norm[t.y] * euv.y + norm[t.z] * euv.z);
        if (!sampled_texcoord.empty())
            sampled_texcoord[i] = texcoord[t.x] * euv.x +
                                  texcoord[t.y] * euv.y + texcoord[t.z] * euv.z;
    }
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// STANDARD SHAPES
// -----------------------------------------------------------------------------
namespace ygl {

/// Make a sphere. This is not watertight.
inline void make_uvsphere(int usteps, int vsteps, vector<vec4i>& quads,
    vector<vec3f>& pos, vector<vec3f>& norm, vector<vec2f>& texcoord) {
    return make_faces(usteps, vsteps, quads, pos, norm, texcoord,
        [](const vec2f& uv) {
            auto a = vec2f{2 * pif * uv.x, pif * (1 - uv.y)};
            return vec3f{cos(a.x) * sin(a.y), sin(a.x) * sin(a.y), cos(a.y)};
        },
        [](const vec2f& uv) {
            auto a = vec2f{2 * pif * uv.x, pif * (1 - uv.y)};
            return vec3f{cos(a.x) * sin(a.y), sin(a.x) * sin(a.y), cos(a.y)};
        },
        [](const vec2f& uv) { return uv; });
}

/// Make a geodesic sphere.
inline void make_geodesicsphere(int level, vector<vec3i>& triangles,
    vector<vec3f>& pos, vector<vec3f>& norm) {
    // https://stackoverflow.com/questions/17705621/algorithm-for-a-geodesic-sphere
    const float X = 0.525731112119133606f;
    const float Z = 0.850650808352039932f;
    pos = {{-X, 0.0, Z}, {X, 0.0, Z}, {-X, 0.0, -Z}, {X, 0.0, -Z}, {0.0, Z, X},
        {0.0, Z, -X}, {0.0, -Z, X}, {0.0, -Z, -X}, {Z, X, 0.0}, {-Z, X, 0.0},
        {Z, -X, 0.0}, {-Z, -X, 0.0}};
    triangles = {{0, 1, 4}, {0, 4, 9}, {9, 4, 5}, {4, 8, 5}, {4, 1, 8},
        {8, 1, 10}, {8, 10, 3}, {5, 8, 3}, {5, 3, 2}, {2, 3, 7}, {7, 3, 10},
        {7, 10, 6}, {7, 6, 11}, {11, 6, 0}, {0, 6, 1}, {6, 10, 1}, {9, 11, 0},
        {9, 2, 11}, {9, 5, 2}, {7, 11, 2}};
    norm = pos;
    vector<vec2f> _aux1;
    vector<vec4f> _aux2;
    vector<float> _aux3;
    for (auto l = 0; l < level - 2; l++) {
        tesselate_triangles(triangles, pos, norm, _aux1, _aux2, _aux3, false);
    }
    for (auto& p : pos) p = normalize(p);
    for (auto& n : norm) n = normalize(n);
}

/// Make a sphere. This is not watertight.
inline void make_uvhemisphere(int usteps, int vsteps, vector<vec4i>& quads,
    vector<vec3f>& pos, vector<vec3f>& norm, vector<vec2f>& texcoord) {
    return make_faces(usteps, vsteps, quads, pos, norm, texcoord,
        [](const vec2f& uv) {
            auto a = vec2f{2 * pif * uv.x, pif * 0.5f * (1 - uv.y)};
            return vec3f{cos(a.x) * sin(a.y), sin(a.x) * sin(a.y), cos(a.y)};
        },
        [](const vec2f& uv) {
            auto a = vec2f{2 * pif * uv.x, pif * 0.5f * (1 - uv.y)};
            return vec3f{cos(a.x) * sin(a.y), sin(a.x) * sin(a.y), cos(a.y)};
        },
        [](const vec2f& uv) { return uv; });
}

/// Make an inside-out sphere. This is not watertight.
inline void make_uvflippedsphere(int usteps, int vsteps, vector<vec4i>& quads,
    vector<vec3f>& pos, vector<vec3f>& norm, vector<vec2f>& texcoord) {
    return make_faces(usteps, vsteps, quads, pos, norm, texcoord,
        [](const vec2f& uv) {
            auto a = vec2f{2 * pif * uv.x, pif * uv.y};
            return vec3f{cos(a.x) * sin(a.y), sin(a.x) * sin(a.y), cos(a.y)};
        },
        [](const vec2f& uv) {
            auto a = vec2f{2 * pif * uv.x, pif * uv.y};
            return vec3f{-cos(a.x) * sin(a.y), -sin(a.x) * sin(a.y), -cos(a.y)};
        },
        [](const vec2f& uv) {
            return vec2f{uv.x, 1 - uv.y};
        });
}

/// Make an inside-out hemisphere. This is not watertight.
inline void make_uvflippedhemisphere(int usteps, int vsteps,
    vector<vec4i>& quads, vector<vec3f>& pos, vector<vec3f>& norm,
    vector<vec2f>& texcoord) {
    return make_faces(usteps, vsteps, quads, pos, norm, texcoord,
        [](const vec2f& uv) {
            auto a = vec2f{2 * pif * uv.x, pif * (0.5f + 0.5f * uv.y)};
            return vec3f{cos(a.x) * sin(a.y), sin(a.x) * sin(a.y), cos(a.y)};
        },
        [](const vec2f& uv) {
            auto a = vec2f{2 * pif * uv.x, pif * uv.y};
            return vec3f{-cos(a.x) * sin(a.y), -sin(a.x) * sin(a.y), -cos(a.y)};
        },
        [](const vec2f& uv) {
            return vec2f{uv.x, 1 - uv.y};
        });
}

/// Make a quad.
inline void make_uvquad(int usteps, int vsteps, vector<vec4i>& quads,
    vector<vec3f>& pos, vector<vec3f>& norm, vector<vec2f>& texcoord) {
    return make_faces(usteps, vsteps, quads, pos, norm, texcoord,
        [](const vec2f& uv) {
            return vec3f{(-1 + uv.x * 2), (-1 + uv.y * 2), 0};
        },
        [](const vec2f& uv) {
            return vec3f{0, 0, 1};
        },
        [](const vec2f& uv) { return uv; });
}

/// Make a cube with unique vertices. This is watertight but has no
/// texture coordinates or normals.
inline void make_cube(vector<vec4i>& quads, vector<vec3f>& pos) {
    static auto cube_pos =
        vector<vec3f>{{-1, -1, -1}, {-1, +1, -1}, {+1, +1, -1}, {+1, -1, -1},
            {-1, -1, +1}, {-1, +1, +1}, {+1, +1, +1}, {+1, -1, +1}};
    static auto cube_quads = vector<vec4i>{{0, 1, 2, 3}, {7, 6, 5, 4},
        {4, 5, 1, 0}, {6, 7, 3, 2}, {2, 1, 5, 6}, {0, 3, 7, 4}};
    static auto cube_quad_uv = vector<vec2f>{{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    pos = cube_pos;
    quads = cube_quads;
}

/// Make a cube with unique vertices. This is watertight but has no
/// texture coordinates or normals.
inline void make_cube(vector<vec3i>& triangles, vector<vec3f>& pos) {
    auto quads = vector<vec4i>();
    make_cube(quads, pos);
    triangles = convert_quads_to_triangles(quads);
}

/// Make a suzanne monkey model for testing. Note that some quads are
/// degenerate.
inline void make_suzanne(vector<vec4i>& quads, vector<vec3f>& pos) {
    static auto suzanne_pos = vector<vec3f>{{0.4375, 0.1640625, 0.765625},
        {-0.4375, 0.1640625, 0.765625}, {0.5, 0.09375, 0.6875},
        {-0.5, 0.09375, 0.6875}, {0.546875, 0.0546875, 0.578125},
        {-0.546875, 0.0546875, 0.578125}, {0.3515625, -0.0234375, 0.6171875},
        {-0.3515625, -0.0234375, 0.6171875}, {0.3515625, 0.03125, 0.71875},
        {-0.3515625, 0.03125, 0.71875}, {0.3515625, 0.1328125, 0.78125},
        {-0.3515625, 0.1328125, 0.78125}, {0.2734375, 0.1640625, 0.796875},
        {-0.2734375, 0.1640625, 0.796875}, {0.203125, 0.09375, 0.7421875},
        {-0.203125, 0.09375, 0.7421875}, {0.15625, 0.0546875, 0.6484375},
        {-0.15625, 0.0546875, 0.6484375}, {0.078125, 0.2421875, 0.65625},
        {-0.078125, 0.2421875, 0.65625}, {0.140625, 0.2421875, 0.7421875},
        {-0.140625, 0.2421875, 0.7421875}, {0.2421875, 0.2421875, 0.796875},
        {-0.2421875, 0.2421875, 0.796875}, {0.2734375, 0.328125, 0.796875},
        {-0.2734375, 0.328125, 0.796875}, {0.203125, 0.390625, 0.7421875},
        {-0.203125, 0.390625, 0.7421875}, {0.15625, 0.4375, 0.6484375},
        {-0.15625, 0.4375, 0.6484375}, {0.3515625, 0.515625, 0.6171875},
        {-0.3515625, 0.515625, 0.6171875}, {0.3515625, 0.453125, 0.71875},
        {-0.3515625, 0.453125, 0.71875}, {0.3515625, 0.359375, 0.78125},
        {-0.3515625, 0.359375, 0.78125}, {0.4375, 0.328125, 0.765625},
        {-0.4375, 0.328125, 0.765625}, {0.5, 0.390625, 0.6875},
        {-0.5, 0.390625, 0.6875}, {0.546875, 0.4375, 0.578125},
        {-0.546875, 0.4375, 0.578125}, {0.625, 0.2421875, 0.5625},
        {-0.625, 0.2421875, 0.5625}, {0.5625, 0.2421875, 0.671875},
        {-0.5625, 0.2421875, 0.671875}, {0.46875, 0.2421875, 0.7578125},
        {-0.46875, 0.2421875, 0.7578125}, {0.4765625, 0.2421875, 0.7734375},
        {-0.4765625, 0.2421875, 0.7734375}, {0.4453125, 0.3359375, 0.78125},
        {-0.4453125, 0.3359375, 0.78125}, {0.3515625, 0.375, 0.8046875},
        {-0.3515625, 0.375, 0.8046875}, {0.265625, 0.3359375, 0.8203125},
        {-0.265625, 0.3359375, 0.8203125}, {0.2265625, 0.2421875, 0.8203125},
        {-0.2265625, 0.2421875, 0.8203125}, {0.265625, 0.15625, 0.8203125},
        {-0.265625, 0.15625, 0.8203125}, {0.3515625, 0.2421875, 0.828125},
        {-0.3515625, 0.2421875, 0.828125}, {0.3515625, 0.1171875, 0.8046875},
        {-0.3515625, 0.1171875, 0.8046875}, {0.4453125, 0.15625, 0.78125},
        {-0.4453125, 0.15625, 0.78125}, {0.0, 0.4296875, 0.7421875},
        {0.0, 0.3515625, 0.8203125}, {0.0, -0.6796875, 0.734375},
        {0.0, -0.3203125, 0.78125}, {0.0, -0.1875, 0.796875},
        {0.0, -0.7734375, 0.71875}, {0.0, 0.40625, 0.6015625},
        {0.0, 0.5703125, 0.5703125}, {0.0, 0.8984375, -0.546875},
        {0.0, 0.5625, -0.8515625}, {0.0, 0.0703125, -0.828125},
        {0.0, -0.3828125, -0.3515625}, {0.203125, -0.1875, 0.5625},
        {-0.203125, -0.1875, 0.5625}, {0.3125, -0.4375, 0.5703125},
        {-0.3125, -0.4375, 0.5703125}, {0.3515625, -0.6953125, 0.5703125},
        {-0.3515625, -0.6953125, 0.5703125}, {0.3671875, -0.890625, 0.53125},
        {-0.3671875, -0.890625, 0.53125}, {0.328125, -0.9453125, 0.5234375},
        {-0.328125, -0.9453125, 0.5234375}, {0.1796875, -0.96875, 0.5546875},
        {-0.1796875, -0.96875, 0.5546875}, {0.0, -0.984375, 0.578125},
        {0.4375, -0.140625, 0.53125}, {-0.4375, -0.140625, 0.53125},
        {0.6328125, -0.0390625, 0.5390625}, {-0.6328125, -0.0390625, 0.5390625},
        {0.828125, 0.1484375, 0.4453125}, {-0.828125, 0.1484375, 0.4453125},
        {0.859375, 0.4296875, 0.59375}, {-0.859375, 0.4296875, 0.59375},
        {0.7109375, 0.484375, 0.625}, {-0.7109375, 0.484375, 0.625},
        {0.4921875, 0.6015625, 0.6875}, {-0.4921875, 0.6015625, 0.6875},
        {0.3203125, 0.7578125, 0.734375}, {-0.3203125, 0.7578125, 0.734375},
        {0.15625, 0.71875, 0.7578125}, {-0.15625, 0.71875, 0.7578125},
        {0.0625, 0.4921875, 0.75}, {-0.0625, 0.4921875, 0.75},
        {0.1640625, 0.4140625, 0.7734375}, {-0.1640625, 0.4140625, 0.7734375},
        {0.125, 0.3046875, 0.765625}, {-0.125, 0.3046875, 0.765625},
        {0.203125, 0.09375, 0.7421875}, {-0.203125, 0.09375, 0.7421875},
        {0.375, 0.015625, 0.703125}, {-0.375, 0.015625, 0.703125},
        {0.4921875, 0.0625, 0.671875}, {-0.4921875, 0.0625, 0.671875},
        {0.625, 0.1875, 0.6484375}, {-0.625, 0.1875, 0.6484375},
        {0.640625, 0.296875, 0.6484375}, {-0.640625, 0.296875, 0.6484375},
        {0.6015625, 0.375, 0.6640625}, {-0.6015625, 0.375, 0.6640625},
        {0.4296875, 0.4375, 0.71875}, {-0.4296875, 0.4375, 0.71875},
        {0.25, 0.46875, 0.7578125}, {-0.25, 0.46875, 0.7578125},
        {0.0, -0.765625, 0.734375}, {0.109375, -0.71875, 0.734375},
        {-0.109375, -0.71875, 0.734375}, {0.1171875, -0.8359375, 0.7109375},
        {-0.1171875, -0.8359375, 0.7109375}, {0.0625, -0.8828125, 0.6953125},
        {-0.0625, -0.8828125, 0.6953125}, {0.0, -0.890625, 0.6875},
        {0.0, -0.1953125, 0.75}, {0.0, -0.140625, 0.7421875},
        {0.1015625, -0.1484375, 0.7421875}, {-0.1015625, -0.1484375, 0.7421875},
        {0.125, -0.2265625, 0.75}, {-0.125, -0.2265625, 0.75},
        {0.0859375, -0.2890625, 0.7421875}, {-0.0859375, -0.2890625, 0.7421875},
        {0.3984375, -0.046875, 0.671875}, {-0.3984375, -0.046875, 0.671875},
        {0.6171875, 0.0546875, 0.625}, {-0.6171875, 0.0546875, 0.625},
        {0.7265625, 0.203125, 0.6015625}, {-0.7265625, 0.203125, 0.6015625},
        {0.7421875, 0.375, 0.65625}, {-0.7421875, 0.375, 0.65625},
        {0.6875, 0.4140625, 0.7265625}, {-0.6875, 0.4140625, 0.7265625},
        {0.4375, 0.546875, 0.796875}, {-0.4375, 0.546875, 0.796875},
        {0.3125, 0.640625, 0.8359375}, {-0.3125, 0.640625, 0.8359375},
        {0.203125, 0.6171875, 0.8515625}, {-0.203125, 0.6171875, 0.8515625},
        {0.1015625, 0.4296875, 0.84375}, {-0.1015625, 0.4296875, 0.84375},
        {0.125, -0.1015625, 0.8125}, {-0.125, -0.1015625, 0.8125},
        {0.2109375, -0.4453125, 0.7109375}, {-0.2109375, -0.4453125, 0.7109375},
        {0.25, -0.703125, 0.6875}, {-0.25, -0.703125, 0.6875},
        {0.265625, -0.8203125, 0.6640625}, {-0.265625, -0.8203125, 0.6640625},
        {0.234375, -0.9140625, 0.6328125}, {-0.234375, -0.9140625, 0.6328125},
        {0.1640625, -0.9296875, 0.6328125}, {-0.1640625, -0.9296875, 0.6328125},
        {0.0, -0.9453125, 0.640625}, {0.0, 0.046875, 0.7265625},
        {0.0, 0.2109375, 0.765625}, {0.328125, 0.4765625, 0.7421875},
        {-0.328125, 0.4765625, 0.7421875}, {0.1640625, 0.140625, 0.75},
        {-0.1640625, 0.140625, 0.75}, {0.1328125, 0.2109375, 0.7578125},
        {-0.1328125, 0.2109375, 0.7578125}, {0.1171875, -0.6875, 0.734375},
        {-0.1171875, -0.6875, 0.734375}, {0.078125, -0.4453125, 0.75},
        {-0.078125, -0.4453125, 0.75}, {0.0, -0.4453125, 0.75},
        {0.0, -0.328125, 0.7421875}, {0.09375, -0.2734375, 0.78125},
        {-0.09375, -0.2734375, 0.78125}, {0.1328125, -0.2265625, 0.796875},
        {-0.1328125, -0.2265625, 0.796875}, {0.109375, -0.1328125, 0.78125},
        {-0.109375, -0.1328125, 0.78125}, {0.0390625, -0.125, 0.78125},
        {-0.0390625, -0.125, 0.78125}, {0.0, -0.203125, 0.828125},
        {0.046875, -0.1484375, 0.8125}, {-0.046875, -0.1484375, 0.8125},
        {0.09375, -0.15625, 0.8125}, {-0.09375, -0.15625, 0.8125},
        {0.109375, -0.2265625, 0.828125}, {-0.109375, -0.2265625, 0.828125},
        {0.078125, -0.25, 0.8046875}, {-0.078125, -0.25, 0.8046875},
        {0.0, -0.2890625, 0.8046875}, {0.2578125, -0.3125, 0.5546875},
        {-0.2578125, -0.3125, 0.5546875}, {0.1640625, -0.2421875, 0.7109375},
        {-0.1640625, -0.2421875, 0.7109375}, {0.1796875, -0.3125, 0.7109375},
        {-0.1796875, -0.3125, 0.7109375}, {0.234375, -0.25, 0.5546875},
        {-0.234375, -0.25, 0.5546875}, {0.0, -0.875, 0.6875},
        {0.046875, -0.8671875, 0.6875}, {-0.046875, -0.8671875, 0.6875},
        {0.09375, -0.8203125, 0.7109375}, {-0.09375, -0.8203125, 0.7109375},
        {0.09375, -0.7421875, 0.7265625}, {-0.09375, -0.7421875, 0.7265625},
        {0.0, -0.78125, 0.65625}, {0.09375, -0.75, 0.6640625},
        {-0.09375, -0.75, 0.6640625}, {0.09375, -0.8125, 0.640625},
        {-0.09375, -0.8125, 0.640625}, {0.046875, -0.8515625, 0.6328125},
        {-0.046875, -0.8515625, 0.6328125}, {0.0, -0.859375, 0.6328125},
        {0.171875, 0.21875, 0.78125}, {-0.171875, 0.21875, 0.78125},
        {0.1875, 0.15625, 0.7734375}, {-0.1875, 0.15625, 0.7734375},
        {0.3359375, 0.4296875, 0.7578125}, {-0.3359375, 0.4296875, 0.7578125},
        {0.2734375, 0.421875, 0.7734375}, {-0.2734375, 0.421875, 0.7734375},
        {0.421875, 0.3984375, 0.7734375}, {-0.421875, 0.3984375, 0.7734375},
        {0.5625, 0.3515625, 0.6953125}, {-0.5625, 0.3515625, 0.6953125},
        {0.5859375, 0.2890625, 0.6875}, {-0.5859375, 0.2890625, 0.6875},
        {0.578125, 0.1953125, 0.6796875}, {-0.578125, 0.1953125, 0.6796875},
        {0.4765625, 0.1015625, 0.71875}, {-0.4765625, 0.1015625, 0.71875},
        {0.375, 0.0625, 0.7421875}, {-0.375, 0.0625, 0.7421875},
        {0.2265625, 0.109375, 0.78125}, {-0.2265625, 0.109375, 0.78125},
        {0.1796875, 0.296875, 0.78125}, {-0.1796875, 0.296875, 0.78125},
        {0.2109375, 0.375, 0.78125}, {-0.2109375, 0.375, 0.78125},
        {0.234375, 0.359375, 0.7578125}, {-0.234375, 0.359375, 0.7578125},
        {0.1953125, 0.296875, 0.7578125}, {-0.1953125, 0.296875, 0.7578125},
        {0.2421875, 0.125, 0.7578125}, {-0.2421875, 0.125, 0.7578125},
        {0.375, 0.0859375, 0.7265625}, {-0.375, 0.0859375, 0.7265625},
        {0.4609375, 0.1171875, 0.703125}, {-0.4609375, 0.1171875, 0.703125},
        {0.546875, 0.2109375, 0.671875}, {-0.546875, 0.2109375, 0.671875},
        {0.5546875, 0.28125, 0.671875}, {-0.5546875, 0.28125, 0.671875},
        {0.53125, 0.3359375, 0.6796875}, {-0.53125, 0.3359375, 0.6796875},
        {0.4140625, 0.390625, 0.75}, {-0.4140625, 0.390625, 0.75},
        {0.28125, 0.3984375, 0.765625}, {-0.28125, 0.3984375, 0.765625},
        {0.3359375, 0.40625, 0.75}, {-0.3359375, 0.40625, 0.75},
        {0.203125, 0.171875, 0.75}, {-0.203125, 0.171875, 0.75},
        {0.1953125, 0.2265625, 0.75}, {-0.1953125, 0.2265625, 0.75},
        {0.109375, 0.4609375, 0.609375}, {-0.109375, 0.4609375, 0.609375},
        {0.1953125, 0.6640625, 0.6171875}, {-0.1953125, 0.6640625, 0.6171875},
        {0.3359375, 0.6875, 0.59375}, {-0.3359375, 0.6875, 0.59375},
        {0.484375, 0.5546875, 0.5546875}, {-0.484375, 0.5546875, 0.5546875},
        {0.6796875, 0.453125, 0.4921875}, {-0.6796875, 0.453125, 0.4921875},
        {0.796875, 0.40625, 0.4609375}, {-0.796875, 0.40625, 0.4609375},
        {0.7734375, 0.1640625, 0.375}, {-0.7734375, 0.1640625, 0.375},
        {0.6015625, 0.0, 0.4140625}, {-0.6015625, 0.0, 0.4140625},
        {0.4375, -0.09375, 0.46875}, {-0.4375, -0.09375, 0.46875},
        {0.0, 0.8984375, 0.2890625}, {0.0, 0.984375, -0.078125},
        {0.0, -0.1953125, -0.671875}, {0.0, -0.4609375, 0.1875},
        {0.0, -0.9765625, 0.4609375}, {0.0, -0.8046875, 0.34375},
        {0.0, -0.5703125, 0.3203125}, {0.0, -0.484375, 0.28125},
        {0.8515625, 0.234375, 0.0546875}, {-0.8515625, 0.234375, 0.0546875},
        {0.859375, 0.3203125, -0.046875}, {-0.859375, 0.3203125, -0.046875},
        {0.7734375, 0.265625, -0.4375}, {-0.7734375, 0.265625, -0.4375},
        {0.4609375, 0.4375, -0.703125}, {-0.4609375, 0.4375, -0.703125},
        {0.734375, -0.046875, 0.0703125}, {-0.734375, -0.046875, 0.0703125},
        {0.59375, -0.125, -0.1640625}, {-0.59375, -0.125, -0.1640625},
        {0.640625, -0.0078125, -0.4296875}, {-0.640625, -0.0078125, -0.4296875},
        {0.3359375, 0.0546875, -0.6640625}, {-0.3359375, 0.0546875, -0.6640625},
        {0.234375, -0.3515625, 0.40625}, {-0.234375, -0.3515625, 0.40625},
        {0.1796875, -0.4140625, 0.2578125}, {-0.1796875, -0.4140625, 0.2578125},
        {0.2890625, -0.7109375, 0.3828125}, {-0.2890625, -0.7109375, 0.3828125},
        {0.25, -0.5, 0.390625}, {-0.25, -0.5, 0.390625},
        {0.328125, -0.9140625, 0.3984375}, {-0.328125, -0.9140625, 0.3984375},
        {0.140625, -0.7578125, 0.3671875}, {-0.140625, -0.7578125, 0.3671875},
        {0.125, -0.5390625, 0.359375}, {-0.125, -0.5390625, 0.359375},
        {0.1640625, -0.9453125, 0.4375}, {-0.1640625, -0.9453125, 0.4375},
        {0.21875, -0.28125, 0.4296875}, {-0.21875, -0.28125, 0.4296875},
        {0.2109375, -0.2265625, 0.46875}, {-0.2109375, -0.2265625, 0.46875},
        {0.203125, -0.171875, 0.5}, {-0.203125, -0.171875, 0.5},
        {0.2109375, -0.390625, 0.1640625}, {-0.2109375, -0.390625, 0.1640625},
        {0.296875, -0.3125, -0.265625}, {-0.296875, -0.3125, -0.265625},
        {0.34375, -0.1484375, -0.5390625}, {-0.34375, -0.1484375, -0.5390625},
        {0.453125, 0.8671875, -0.3828125}, {-0.453125, 0.8671875, -0.3828125},
        {0.453125, 0.9296875, -0.0703125}, {-0.453125, 0.9296875, -0.0703125},
        {0.453125, 0.8515625, 0.234375}, {-0.453125, 0.8515625, 0.234375},
        {0.4609375, 0.5234375, 0.4296875}, {-0.4609375, 0.5234375, 0.4296875},
        {0.7265625, 0.40625, 0.3359375}, {-0.7265625, 0.40625, 0.3359375},
        {0.6328125, 0.453125, 0.28125}, {-0.6328125, 0.453125, 0.28125},
        {0.640625, 0.703125, 0.0546875}, {-0.640625, 0.703125, 0.0546875},
        {0.796875, 0.5625, 0.125}, {-0.796875, 0.5625, 0.125},
        {0.796875, 0.6171875, -0.1171875}, {-0.796875, 0.6171875, -0.1171875},
        {0.640625, 0.75, -0.1953125}, {-0.640625, 0.75, -0.1953125},
        {0.640625, 0.6796875, -0.4453125}, {-0.640625, 0.6796875, -0.4453125},
        {0.796875, 0.5390625, -0.359375}, {-0.796875, 0.5390625, -0.359375},
        {0.6171875, 0.328125, -0.5859375}, {-0.6171875, 0.328125, -0.5859375},
        {0.484375, 0.0234375, -0.546875}, {-0.484375, 0.0234375, -0.546875},
        {0.8203125, 0.328125, -0.203125}, {-0.8203125, 0.328125, -0.203125},
        {0.40625, -0.171875, 0.1484375}, {-0.40625, -0.171875, 0.1484375},
        {0.4296875, -0.1953125, -0.2109375},
        {-0.4296875, -0.1953125, -0.2109375}, {0.890625, 0.40625, -0.234375},
        {-0.890625, 0.40625, -0.234375}, {0.7734375, -0.140625, -0.125},
        {-0.7734375, -0.140625, -0.125}, {1.0390625, -0.1015625, -0.328125},
        {-1.0390625, -0.1015625, -0.328125}, {1.28125, 0.0546875, -0.4296875},
        {-1.28125, 0.0546875, -0.4296875}, {1.3515625, 0.3203125, -0.421875},
        {-1.3515625, 0.3203125, -0.421875}, {1.234375, 0.5078125, -0.421875},
        {-1.234375, 0.5078125, -0.421875}, {1.0234375, 0.4765625, -0.3125},
        {-1.0234375, 0.4765625, -0.3125}, {1.015625, 0.4140625, -0.2890625},
        {-1.015625, 0.4140625, -0.2890625}, {1.1875, 0.4375, -0.390625},
        {-1.1875, 0.4375, -0.390625}, {1.265625, 0.2890625, -0.40625},
        {-1.265625, 0.2890625, -0.40625}, {1.2109375, 0.078125, -0.40625},
        {-1.2109375, 0.078125, -0.40625}, {1.03125, -0.0390625, -0.3046875},
        {-1.03125, -0.0390625, -0.3046875}, {0.828125, -0.0703125, -0.1328125},
        {-0.828125, -0.0703125, -0.1328125}, {0.921875, 0.359375, -0.21875},
        {-0.921875, 0.359375, -0.21875}, {0.9453125, 0.3046875, -0.2890625},
        {-0.9453125, 0.3046875, -0.2890625},
        {0.8828125, -0.0234375, -0.2109375},
        {-0.8828125, -0.0234375, -0.2109375}, {1.0390625, 0.0, -0.3671875},
        {-1.0390625, 0.0, -0.3671875}, {1.1875, 0.09375, -0.4453125},
        {-1.1875, 0.09375, -0.4453125}, {1.234375, 0.25, -0.4453125},
        {-1.234375, 0.25, -0.4453125}, {1.171875, 0.359375, -0.4375},
        {-1.171875, 0.359375, -0.4375}, {1.0234375, 0.34375, -0.359375},
        {-1.0234375, 0.34375, -0.359375}, {0.84375, 0.2890625, -0.2109375},
        {-0.84375, 0.2890625, -0.2109375}, {0.8359375, 0.171875, -0.2734375},
        {-0.8359375, 0.171875, -0.2734375}, {0.7578125, 0.09375, -0.2734375},
        {-0.7578125, 0.09375, -0.2734375}, {0.8203125, 0.0859375, -0.2734375},
        {-0.8203125, 0.0859375, -0.2734375}, {0.84375, 0.015625, -0.2734375},
        {-0.84375, 0.015625, -0.2734375}, {0.8125, -0.015625, -0.2734375},
        {-0.8125, -0.015625, -0.2734375}, {0.7265625, 0.0, -0.0703125},
        {-0.7265625, 0.0, -0.0703125}, {0.71875, -0.0234375, -0.171875},
        {-0.71875, -0.0234375, -0.171875}, {0.71875, 0.0390625, -0.1875},
        {-0.71875, 0.0390625, -0.1875}, {0.796875, 0.203125, -0.2109375},
        {-0.796875, 0.203125, -0.2109375}, {0.890625, 0.2421875, -0.265625},
        {-0.890625, 0.2421875, -0.265625}, {0.890625, 0.234375, -0.3203125},
        {-0.890625, 0.234375, -0.3203125}, {0.8125, -0.015625, -0.3203125},
        {-0.8125, -0.015625, -0.3203125}, {0.8515625, 0.015625, -0.3203125},
        {-0.8515625, 0.015625, -0.3203125}, {0.828125, 0.078125, -0.3203125},
        {-0.828125, 0.078125, -0.3203125}, {0.765625, 0.09375, -0.3203125},
        {-0.765625, 0.09375, -0.3203125}, {0.84375, 0.171875, -0.3203125},
        {-0.84375, 0.171875, -0.3203125}, {1.0390625, 0.328125, -0.4140625},
        {-1.0390625, 0.328125, -0.4140625}, {1.1875, 0.34375, -0.484375},
        {-1.1875, 0.34375, -0.484375}, {1.2578125, 0.2421875, -0.4921875},
        {-1.2578125, 0.2421875, -0.4921875}, {1.2109375, 0.0859375, -0.484375},
        {-1.2109375, 0.0859375, -0.484375}, {1.046875, 0.0, -0.421875},
        {-1.046875, 0.0, -0.421875}, {0.8828125, -0.015625, -0.265625},
        {-0.8828125, -0.015625, -0.265625}, {0.953125, 0.2890625, -0.34375},
        {-0.953125, 0.2890625, -0.34375}, {0.890625, 0.109375, -0.328125},
        {-0.890625, 0.109375, -0.328125}, {0.9375, 0.0625, -0.3359375},
        {-0.9375, 0.0625, -0.3359375}, {1.0, 0.125, -0.3671875},
        {-1.0, 0.125, -0.3671875}, {0.9609375, 0.171875, -0.3515625},
        {-0.9609375, 0.171875, -0.3515625}, {1.015625, 0.234375, -0.375},
        {-1.015625, 0.234375, -0.375}, {1.0546875, 0.1875, -0.3828125},
        {-1.0546875, 0.1875, -0.3828125}, {1.109375, 0.2109375, -0.390625},
        {-1.109375, 0.2109375, -0.390625}, {1.0859375, 0.2734375, -0.390625},
        {-1.0859375, 0.2734375, -0.390625}, {1.0234375, 0.4375, -0.484375},
        {-1.0234375, 0.4375, -0.484375}, {1.25, 0.46875, -0.546875},
        {-1.25, 0.46875, -0.546875}, {1.3671875, 0.296875, -0.5},
        {-1.3671875, 0.296875, -0.5}, {1.3125, 0.0546875, -0.53125},
        {-1.3125, 0.0546875, -0.53125}, {1.0390625, -0.0859375, -0.4921875},
        {-1.0390625, -0.0859375, -0.4921875}, {0.7890625, -0.125, -0.328125},
        {-0.7890625, -0.125, -0.328125}, {0.859375, 0.3828125, -0.3828125},
        {-0.859375, 0.3828125, -0.3828125}};
    static auto suzanne_triangles = vector<vec3i>{{60, 64, 48}, {49, 65, 61},
        {62, 64, 60}, {61, 65, 63}, {60, 58, 62}, {63, 59, 61}, {60, 56, 58},
        {59, 57, 61}, {60, 54, 56}, {57, 55, 61}, {60, 52, 54}, {55, 53, 61},
        {60, 50, 52}, {53, 51, 61}, {60, 48, 50}, {51, 49, 61}, {224, 228, 226},
        {227, 229, 225}, {72, 283, 73}, {73, 284, 72}, {341, 347, 383},
        {384, 348, 342}, {299, 345, 343}, {344, 346, 300}, {323, 379, 351},
        {352, 380, 324}, {441, 443, 445}, {446, 444, 442}, {463, 491, 465},
        {466, 492, 464}, {495, 497, 499}, {500, 498, 496}};
    static auto suzanne_quads = vector<vec4i>{{46, 0, 2, 44}, {3, 1, 47, 45},
        {44, 2, 4, 42}, {5, 3, 45, 43}, {2, 8, 6, 4}, {7, 9, 3, 5},
        {0, 10, 8, 2}, {9, 11, 1, 3}, {10, 12, 14, 8}, {15, 13, 11, 9},
        {8, 14, 16, 6}, {17, 15, 9, 7}, {14, 20, 18, 16}, {19, 21, 15, 17},
        {12, 22, 20, 14}, {21, 23, 13, 15}, {22, 24, 26, 20}, {27, 25, 23, 21},
        {20, 26, 28, 18}, {29, 27, 21, 19}, {26, 32, 30, 28}, {31, 33, 27, 29},
        {24, 34, 32, 26}, {33, 35, 25, 27}, {34, 36, 38, 32}, {39, 37, 35, 33},
        {32, 38, 40, 30}, {41, 39, 33, 31}, {38, 44, 42, 40}, {43, 45, 39, 41},
        {36, 46, 44, 38}, {45, 47, 37, 39}, {46, 36, 50, 48}, {51, 37, 47, 49},
        {36, 34, 52, 50}, {53, 35, 37, 51}, {34, 24, 54, 52}, {55, 25, 35, 53},
        {24, 22, 56, 54}, {57, 23, 25, 55}, {22, 12, 58, 56}, {59, 13, 23, 57},
        {12, 10, 62, 58}, {63, 11, 13, 59}, {10, 0, 64, 62}, {65, 1, 11, 63},
        {0, 46, 48, 64}, {49, 47, 1, 65}, {88, 173, 175, 90},
        {175, 174, 89, 90}, {86, 171, 173, 88}, {174, 172, 87, 89},
        {84, 169, 171, 86}, {172, 170, 85, 87}, {82, 167, 169, 84},
        {170, 168, 83, 85}, {80, 165, 167, 82}, {168, 166, 81, 83},
        {78, 91, 145, 163}, {146, 92, 79, 164}, {91, 93, 147, 145},
        {148, 94, 92, 146}, {93, 95, 149, 147}, {150, 96, 94, 148},
        {95, 97, 151, 149}, {152, 98, 96, 150}, {97, 99, 153, 151},
        {154, 100, 98, 152}, {99, 101, 155, 153}, {156, 102, 100, 154},
        {101, 103, 157, 155}, {158, 104, 102, 156}, {103, 105, 159, 157},
        {160, 106, 104, 158}, {105, 107, 161, 159}, {162, 108, 106, 160},
        {107, 66, 67, 161}, {67, 66, 108, 162}, {109, 127, 159, 161},
        {160, 128, 110, 162}, {127, 178, 157, 159}, {158, 179, 128, 160},
        {125, 155, 157, 178}, {158, 156, 126, 179}, {123, 153, 155, 125},
        {156, 154, 124, 126}, {121, 151, 153, 123}, {154, 152, 122, 124},
        {119, 149, 151, 121}, {152, 150, 120, 122}, {117, 147, 149, 119},
        {150, 148, 118, 120}, {115, 145, 147, 117}, {148, 146, 116, 118},
        {113, 163, 145, 115}, {146, 164, 114, 116}, {113, 180, 176, 163},
        {176, 181, 114, 164}, {109, 161, 67, 111}, {67, 162, 110, 112},
        {111, 67, 177, 182}, {177, 67, 112, 183}, {176, 180, 182, 177},
        {183, 181, 176, 177}, {134, 136, 175, 173}, {175, 136, 135, 174},
        {132, 134, 173, 171}, {174, 135, 133, 172}, {130, 132, 171, 169},
        {172, 133, 131, 170}, {165, 186, 184, 167}, {185, 187, 166, 168},
        {130, 169, 167, 184}, {168, 170, 131, 185}, {143, 189, 188, 186},
        {188, 189, 144, 187}, {184, 186, 188, 68}, {188, 187, 185, 68},
        {129, 130, 184, 68}, {185, 131, 129, 68}, {141, 192, 190, 143},
        {191, 193, 142, 144}, {139, 194, 192, 141}, {193, 195, 140, 142},
        {138, 196, 194, 139}, {195, 197, 138, 140}, {137, 70, 196, 138},
        {197, 70, 137, 138}, {189, 143, 190, 69}, {191, 144, 189, 69},
        {69, 190, 205, 207}, {206, 191, 69, 207}, {70, 198, 199, 196},
        {200, 198, 70, 197}, {196, 199, 201, 194}, {202, 200, 197, 195},
        {194, 201, 203, 192}, {204, 202, 195, 193}, {192, 203, 205, 190},
        {206, 204, 193, 191}, {198, 203, 201, 199}, {202, 204, 198, 200},
        {198, 207, 205, 203}, {206, 207, 198, 204}, {138, 139, 163, 176},
        {164, 140, 138, 176}, {139, 141, 210, 163}, {211, 142, 140, 164},
        {141, 143, 212, 210}, {213, 144, 142, 211}, {143, 186, 165, 212},
        {166, 187, 144, 213}, {80, 208, 212, 165}, {213, 209, 81, 166},
        {208, 214, 210, 212}, {211, 215, 209, 213}, {78, 163, 210, 214},
        {211, 164, 79, 215}, {130, 129, 71, 221}, {71, 129, 131, 222},
        {132, 130, 221, 219}, {222, 131, 133, 220}, {134, 132, 219, 217},
        {220, 133, 135, 218}, {136, 134, 217, 216}, {218, 135, 136, 216},
        {216, 217, 228, 230}, {229, 218, 216, 230}, {217, 219, 226, 228},
        {227, 220, 218, 229}, {219, 221, 224, 226}, {225, 222, 220, 227},
        {221, 71, 223, 224}, {223, 71, 222, 225}, {223, 230, 228, 224},
        {229, 230, 223, 225}, {182, 180, 233, 231}, {234, 181, 183, 232},
        {111, 182, 231, 253}, {232, 183, 112, 254}, {109, 111, 253, 255},
        {254, 112, 110, 256}, {180, 113, 251, 233}, {252, 114, 181, 234},
        {113, 115, 249, 251}, {250, 116, 114, 252}, {115, 117, 247, 249},
        {248, 118, 116, 250}, {117, 119, 245, 247}, {246, 120, 118, 248},
        {119, 121, 243, 245}, {244, 122, 120, 246}, {121, 123, 241, 243},
        {242, 124, 122, 244}, {123, 125, 239, 241}, {240, 126, 124, 242},
        {125, 178, 235, 239}, {236, 179, 126, 240}, {178, 127, 237, 235},
        {238, 128, 179, 236}, {127, 109, 255, 237}, {256, 110, 128, 238},
        {237, 255, 257, 275}, {258, 256, 238, 276}, {235, 237, 275, 277},
        {276, 238, 236, 278}, {239, 235, 277, 273}, {278, 236, 240, 274},
        {241, 239, 273, 271}, {274, 240, 242, 272}, {243, 241, 271, 269},
        {272, 242, 244, 270}, {245, 243, 269, 267}, {270, 244, 246, 268},
        {247, 245, 267, 265}, {268, 246, 248, 266}, {249, 247, 265, 263},
        {266, 248, 250, 264}, {251, 249, 263, 261}, {264, 250, 252, 262},
        {233, 251, 261, 279}, {262, 252, 234, 280}, {255, 253, 259, 257},
        {260, 254, 256, 258}, {253, 231, 281, 259}, {282, 232, 254, 260},
        {231, 233, 279, 281}, {280, 234, 232, 282}, {66, 107, 283, 72},
        {284, 108, 66, 72}, {107, 105, 285, 283}, {286, 106, 108, 284},
        {105, 103, 287, 285}, {288, 104, 106, 286}, {103, 101, 289, 287},
        {290, 102, 104, 288}, {101, 99, 291, 289}, {292, 100, 102, 290},
        {99, 97, 293, 291}, {294, 98, 100, 292}, {97, 95, 295, 293},
        {296, 96, 98, 294}, {95, 93, 297, 295}, {298, 94, 96, 296},
        {93, 91, 299, 297}, {300, 92, 94, 298}, {307, 308, 327, 337},
        {328, 308, 307, 338}, {306, 307, 337, 335}, {338, 307, 306, 336},
        {305, 306, 335, 339}, {336, 306, 305, 340}, {88, 90, 305, 339},
        {305, 90, 89, 340}, {86, 88, 339, 333}, {340, 89, 87, 334},
        {84, 86, 333, 329}, {334, 87, 85, 330}, {82, 84, 329, 331},
        {330, 85, 83, 332}, {329, 335, 337, 331}, {338, 336, 330, 332},
        {329, 333, 339, 335}, {340, 334, 330, 336}, {325, 331, 337, 327},
        {338, 332, 326, 328}, {80, 82, 331, 325}, {332, 83, 81, 326},
        {208, 341, 343, 214}, {344, 342, 209, 215}, {80, 325, 341, 208},
        {342, 326, 81, 209}, {78, 214, 343, 345}, {344, 215, 79, 346},
        {78, 345, 299, 91}, {300, 346, 79, 92}, {76, 323, 351, 303},
        {352, 324, 76, 303}, {303, 351, 349, 77}, {350, 352, 303, 77},
        {77, 349, 347, 304}, {348, 350, 77, 304}, {304, 347, 327, 308},
        {328, 348, 304, 308}, {325, 327, 347, 341}, {348, 328, 326, 342},
        {295, 297, 317, 309}, {318, 298, 296, 310}, {75, 315, 323, 76},
        {324, 316, 75, 76}, {301, 357, 355, 302}, {356, 358, 301, 302},
        {302, 355, 353, 74}, {354, 356, 302, 74}, {74, 353, 315, 75},
        {316, 354, 74, 75}, {291, 293, 361, 363}, {362, 294, 292, 364},
        {363, 361, 367, 365}, {368, 362, 364, 366}, {365, 367, 369, 371},
        {370, 368, 366, 372}, {371, 369, 375, 373}, {376, 370, 372, 374},
        {313, 377, 373, 375}, {374, 378, 314, 376}, {315, 353, 373, 377},
        {374, 354, 316, 378}, {353, 355, 371, 373}, {372, 356, 354, 374},
        {355, 357, 365, 371}, {366, 358, 356, 372}, {357, 359, 363, 365},
        {364, 360, 358, 366}, {289, 291, 363, 359}, {364, 292, 290, 360},
        {73, 359, 357, 301}, {358, 360, 73, 301}, {283, 285, 287, 289},
        {288, 286, 284, 290}, {283, 289, 359, 73}, {360, 290, 284, 73},
        {293, 295, 309, 361}, {310, 296, 294, 362}, {309, 311, 367, 361},
        {368, 312, 310, 362}, {311, 381, 369, 367}, {370, 382, 312, 368},
        {313, 375, 369, 381}, {370, 376, 314, 382}, {347, 349, 385, 383},
        {386, 350, 348, 384}, {317, 383, 385, 319}, {386, 384, 318, 320},
        {297, 299, 383, 317}, {384, 300, 298, 318}, {299, 343, 341, 383},
        {342, 344, 300, 384}, {313, 321, 379, 377}, {380, 322, 314, 378},
        {315, 377, 379, 323}, {380, 378, 316, 324}, {319, 385, 379, 321},
        {380, 386, 320, 322}, {349, 351, 379, 385}, {380, 352, 350, 386},
        {399, 387, 413, 401}, {414, 388, 400, 402}, {399, 401, 403, 397},
        {404, 402, 400, 398}, {397, 403, 405, 395}, {406, 404, 398, 396},
        {395, 405, 407, 393}, {408, 406, 396, 394}, {393, 407, 409, 391},
        {410, 408, 394, 392}, {391, 409, 411, 389}, {412, 410, 392, 390},
        {409, 419, 417, 411}, {418, 420, 410, 412}, {407, 421, 419, 409},
        {420, 422, 408, 410}, {405, 423, 421, 407}, {422, 424, 406, 408},
        {403, 425, 423, 405}, {424, 426, 404, 406}, {401, 427, 425, 403},
        {426, 428, 402, 404}, {401, 413, 415, 427}, {416, 414, 402, 428},
        {317, 319, 443, 441}, {444, 320, 318, 442}, {319, 389, 411, 443},
        {412, 390, 320, 444}, {309, 317, 441, 311}, {442, 318, 310, 312},
        {381, 429, 413, 387}, {414, 430, 382, 388}, {411, 417, 439, 443},
        {440, 418, 412, 444}, {437, 445, 443, 439}, {444, 446, 438, 440},
        {433, 445, 437, 435}, {438, 446, 434, 436}, {431, 447, 445, 433},
        {446, 448, 432, 434}, {429, 447, 431, 449}, {432, 448, 430, 450},
        {413, 429, 449, 415}, {450, 430, 414, 416}, {311, 447, 429, 381},
        {430, 448, 312, 382}, {311, 441, 445, 447}, {446, 442, 312, 448},
        {415, 449, 451, 475}, {452, 450, 416, 476}, {449, 431, 461, 451},
        {462, 432, 450, 452}, {431, 433, 459, 461}, {460, 434, 432, 462},
        {433, 435, 457, 459}, {458, 436, 434, 460}, {435, 437, 455, 457},
        {456, 438, 436, 458}, {437, 439, 453, 455}, {454, 440, 438, 456},
        {439, 417, 473, 453}, {474, 418, 440, 454}, {427, 415, 475, 463},
        {476, 416, 428, 464}, {425, 427, 463, 465}, {464, 428, 426, 466},
        {423, 425, 465, 467}, {466, 426, 424, 468}, {421, 423, 467, 469},
        {468, 424, 422, 470}, {419, 421, 469, 471}, {470, 422, 420, 472},
        {417, 419, 471, 473}, {472, 420, 418, 474}, {457, 455, 479, 477},
        {480, 456, 458, 478}, {477, 479, 481, 483}, {482, 480, 478, 484},
        {483, 481, 487, 485}, {488, 482, 484, 486}, {485, 487, 489, 491},
        {490, 488, 486, 492}, {463, 475, 485, 491}, {486, 476, 464, 492},
        {451, 483, 485, 475}, {486, 484, 452, 476}, {451, 461, 477, 483},
        {478, 462, 452, 484}, {457, 477, 461, 459}, {462, 478, 458, 460},
        {453, 473, 479, 455}, {480, 474, 454, 456}, {471, 481, 479, 473},
        {480, 482, 472, 474}, {469, 487, 481, 471}, {482, 488, 470, 472},
        {467, 489, 487, 469}, {488, 490, 468, 470}, {465, 491, 489, 467},
        {490, 492, 466, 468}, {391, 389, 503, 501}, {504, 390, 392, 502},
        {393, 391, 501, 499}, {502, 392, 394, 500}, {395, 393, 499, 497},
        {500, 394, 396, 498}, {397, 395, 497, 495}, {498, 396, 398, 496},
        {399, 397, 495, 493}, {496, 398, 400, 494}, {387, 399, 493, 505},
        {494, 400, 388, 506}, {493, 501, 503, 505}, {504, 502, 494, 506},
        {493, 495, 499, 501}, {500, 496, 494, 502}, {313, 381, 387, 505},
        {388, 382, 314, 506}, {313, 505, 503, 321}, {504, 506, 314, 322},
        {319, 321, 503, 389}, {504, 322, 320, 390}};
    pos = suzanne_pos;
    quads = suzanne_quads;
    quads.reserve(quads.size() + suzanne_triangles.size());
    for (auto& t : suzanne_triangles) { quads.push_back({t.x, t.y, t.z, t.z}); }
}

/// Make a cube with uv. This is not watertight.
inline void make_uvcube(int usteps, int vsteps, vector<vec4i>& quads,
    vector<vec3f>& pos, vector<vec3f>& norm, vector<vec2f>& texcoord) {
    frame3f frames[6] = {frame3f{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0, 0, 1}},
        frame3f{{-1, 0, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, -1}},
        frame3f{{-1, 0, 0}, {0, 0, 1}, {0, 1, 0}, {0, 1, 0}},
        frame3f{{1, 0, 0}, {0, 0, 1}, {0, -1, 0}, {0, -1, 0}},
        frame3f{{0, 1, 0}, {0, 0, 1}, {1, 0, 0}, {1, 0, 0}},
        frame3f{{0, -1, 0}, {0, 0, 1}, {-1, 0, 0}, {-1, 0, 0}}};
    vector<vec3f> quad_pos, quad_norm;
    vector<vec2f> quad_texcoord;
    vector<vec4i> quad_quads;
    make_uvquad(usteps, vsteps, quad_quads, quad_pos, quad_norm, quad_texcoord);
    for (auto i = 0; i < 6; i++) {
        pos.insert(pos.end(), quad_pos.begin(), quad_pos.end());
        norm.insert(norm.end(), quad_norm.begin(), quad_norm.end());
        texcoord.insert(
            texcoord.end(), quad_texcoord.begin(), quad_texcoord.end());
        quads.insert(quads.end(), quad_quads.begin(), quad_quads.end());
    }
    auto quad_verts = quad_pos.size();
    for (auto i = 0; i < 6; i++) {
        for (auto j = quad_verts * i; j < quad_verts * (i + 1); j++)
            pos[j] = transform_point(frames[i], pos[j]);
        for (auto j = quad_verts * i; j < quad_verts * (i + 1); j++)
            norm[j] = transform_direction(frames[i], norm[j]);
    }
    auto quad_faces = quad_quads.size();
    for (auto i = 0; i < 6; i++) {
        for (auto j = quad_faces * i; j < quad_faces * (i + 1); j++) {
            quads[j].x += quad_verts * i;
            quads[j].y += quad_verts * i;
            quads[j].z += quad_verts * i;
            quads[j].w += quad_verts * i;
        }
    }
}

/// Make a sphere from a cube. This is not watertight.
inline void make_uvspherecube(int usteps, int vsteps, vector<vec4i>& quads,
    vector<vec3f>& pos, vector<vec3f>& norm, vector<vec2f>& texcoord) {
    make_uvcube(usteps, vsteps, quads, pos, norm, texcoord);
    for (auto i = 0; i < pos.size(); i++) {
        pos[i] = normalize(pos[i]);
        norm[i] = normalize(pos[i]);
    }
}

/// Make a cube than stretch it towards a sphere. This is not watertight.
inline void make_uvspherizedcube(int usteps, int vsteps, float radius,
    vector<vec4i>& quads, vector<vec3f>& pos, vector<vec3f>& norm,
    vector<vec2f>& texcoord) {
    make_uvcube(usteps, vsteps, quads, pos, norm, texcoord);
    for (auto i = 0; i < pos.size(); i++) {
        norm[i] = normalize(pos[i]);
        pos[i] *= 1 - radius;
        pos[i] += norm[i] * radius;
    }
    compute_normals(quads, pos, norm, true);
}

/// Make a flipped sphere. This is not watertight.
inline void make_uvflipcapsphere(int usteps, int vsteps, float radius,
    vector<vec4i>& quads, vector<vec3f>& pos, vector<vec3f>& norm,
    vector<vec2f>& texcoord) {
    make_uvsphere(usteps, vsteps, quads, pos, norm, texcoord);
    for (auto i = 0; i < pos.size(); i++) {
        if (pos[i].z > radius) {
            pos[i].z = 2 * radius - pos[i].z;
            norm[i].x = -norm[i].x;
            norm[i].y = -norm[i].y;
        } else if (pos[i].z < -radius) {
            pos[i].z = -2 * radius - pos[i].z;
            norm[i].x = -norm[i].x;
            norm[i].y = -norm[i].y;
        }
    }
}

/// Make a butout sphere. This is not watertight.
inline void make_uvcutsphere(int usteps, int vsteps, float radius,
    vector<vec4i>& quads, vector<vec3f>& pos, vector<vec3f>& norm,
    vector<vec2f>& texcoord) {
    return make_faces(usteps, vsteps, quads, pos, norm, texcoord,
        [radius](const vec2f& uv) {
            auto p = 1 - acos(radius) / pif;
            auto a = vec2f{2 * pif * uv.x, pif * (1 - p * uv.y)};
            return vec3f{cos(a.x) * sin(a.y), sin(a.x) * sin(a.y), cos(a.y)};
        },
        [radius](const vec2f& uv) {
            auto p = 1 - acos(radius) / pif;
            auto a = vec2f{2 * pif * uv.x, pif * (1 - p * uv.y)};
            return vec3f{cos(a.x) * sin(a.y), sin(a.x) * sin(a.y), cos(a.y)};
        },
        [](const vec2f& uv) { return uv; });
}

/// Make a quad. This is not watertight.
inline void make_uvflippedcutsphere(int usteps, int vsteps, float radius,
    vector<vec4i>& quads, vector<vec3f>& pos, vector<vec3f>& norm,
    vector<vec2f>& texcoord) {
    return make_faces(usteps, vsteps, quads, pos, norm, texcoord,
        [radius](const vec2f& uv) {
            auto p = 1 - acos(radius) / pif;
            auto a = vec2f{2 * pif * uv.x, pif * ((1 - p) + p * uv.y)};
            return vec3f{cos(a.x) * sin(a.y), sin(a.x) * sin(a.y), cos(a.y)};
        },
        [radius](const vec2f& uv) {
            auto p = 1 - acos(radius) / pif;
            auto a = vec2f{2 * pif * uv.x, pif * ((1 - p) + p * uv.y)};
            return vec3f{-cos(a.x) * sin(a.y), -sin(a.x) * sin(a.y), -cos(a.y)};
        },
        [](const vec2f& uv) {
            return vec2f{uv.x, (1 - uv.y)};
        });
}

// -----------------------------------------------------------------------------
// FILE LOADING AND SAVING
// -----------------------------------------------------------------------------

/// Loads the contents of a binary file in an in-memory array.
inline vector<unsigned char> load_binfile(const string& filename) {
    fstream fs(filename, ios_base::in | ios_base::binary);
    if (fs.fail()) throw runtime_error("cannot read file " + filename);
    fs.seekg(0, std::ios::end);
    auto buf = vector<unsigned char>(fs.tellg());
    fs.seekg(0);
    fs.read((char*)buf.data(), buf.size());
    if (fs.fail() || fs.bad())
        throw runtime_error("cannot read file " + filename);
    return buf;
}

/// Loads the contents of a text file into a string.
inline string load_txtfile(const string& filename) {
    fstream fs(filename, ios_base::in);
    if (fs.fail()) throw runtime_error("cannot read file " + filename);
    stringstream ss;
    ss << fs.rdbuf();
    if (fs.fail()) throw runtime_error("cannot read file " + filename);
    return ss.str();
}

/// Saves binary data to a file.
inline void save_binfile(
    const string& filename, const vector<unsigned char>& data) {
    fstream fs(filename, ios_base::out | ios_base::binary);
    if (fs.fail()) throw runtime_error("cannot write file " + filename);
    fs.write((const char*)data.data(), data.size());
    if (fs.fail() || fs.bad())
        throw runtime_error("cannot write file " + filename);
}

/// Saves a string to a text file.
inline void save_txtfile(const string& filename, const string& str) {
    fstream fs(filename, ios_base::out);
    if (fs.fail()) throw runtime_error("cannot write file " + filename);
    fs << str;
    if (fs.fail()) throw runtime_error("cannot write file " + filename);
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// BASE64 ENCONDING/DECODING SUPPORT
// -----------------------------------------------------------------------------
namespace ygl {

/// Encode in base64
inline string base64_encode(
    unsigned char const* bytes_to_encode, unsigned int in_len) {
    static const string base64_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";

    string ret;
    int i = 0;
    int j = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];

    while (in_len--) {
        char_array_3[i++] = *(bytes_to_encode++);
        if (i == 3) {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) +
                              ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) +
                              ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;

            for (i = 0; (i < 4); i++) ret += base64_chars[char_array_4[i]];
            i = 0;
        }
    }

    if (i) {
        for (j = i; j < 3; j++) char_array_3[j] = '\0';

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] =
            ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] =
            ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for (j = 0; (j < i + 1); j++) ret += base64_chars[char_array_4[j]];

        while ((i++ < 3)) ret += '=';
    }

    return ret;
}

/// Decode from base64
inline string base64_decode(string const& encoded_string) {
    static const string base64_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";

    auto is_base64 = [](unsigned char c) -> bool {
        return (isalnum(c) || (c == '+') || (c == '/'));
    };

    int in_len = (int)encoded_string.size();
    int i = 0;
    int j = 0;
    int in_ = 0;
    unsigned char char_array_4[4], char_array_3[3];
    string ret;

    while (in_len-- && (encoded_string[in_] != '=') &&
           is_base64(encoded_string[in_])) {
        char_array_4[i++] = encoded_string[in_];
        in_++;
        if (i == 4) {
            for (i = 0; i < 4; i++)
                char_array_4[i] = base64_chars.find(char_array_4[i]);

            char_array_3[0] =
                (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
            char_array_3[1] = ((char_array_4[1] & 0xf) << 4) +
                              ((char_array_4[2] & 0x3c) >> 2);
            char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

            for (i = 0; (i < 3); i++) ret += char_array_3[i];
            i = 0;
        }
    }

    if (i) {
        for (j = i; j < 4; j++) char_array_4[j] = 0;

        for (j = 0; j < 4; j++)
            char_array_4[j] = base64_chars.find(char_array_4[j]);

        char_array_3[0] =
            (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
        char_array_3[1] =
            ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
        char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

        for (j = 0; (j < i - 1); j++) ret += char_array_3[j];
    }

    return ret;
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// PYTHON-LIKE ITERATORS AND CONTAINER OPERATIONS
// -----------------------------------------------------------------------------
namespace ygl {

/// Python-like range. Create it with the function below to use argument
/// deduction. Eventually, this will change to template class deduction
/// when available.
template <typename T>
struct range_generator {
    /// iterator
    struct iterator {
        /// constructor
        constexpr iterator(T pos, T step) : _pos(pos), _step(step) {}

        /// check for equality in range
        constexpr bool operator!=(const iterator& a) const {
            return _pos < a._pos;
        }

        /// increment
        constexpr iterator& operator++() {
            _pos += _step;
            return *this;
        }

        /// value access
        constexpr T& operator*() { return _pos; }
        /// value access
        constexpr const T& operator*() const { return _pos; }

        // implementation ------------
        int _pos, _step;
    };

    /// Iterates from 0 to max-1
    constexpr range_generator(int max) : _min(0), _max(max), _step(1) {}
    /// Iterates from min to max-1 with step step
    constexpr range_generator(int min, int max, int step = 1)
        : _min(min), _max(max), _step(step) {}

    /// Iteration
    constexpr iterator begin() const { return {_min, _step}; }
    /// Iteration
    constexpr iterator end() const { return {_max, _step}; }

    // implementation ----------------
    // min, max and step
    int _min, _max, _step;
};

/// Python-like range
template <typename T>
constexpr inline range_generator<T> range(T max) {
    return {max};
}

/// Python-like range
template <typename T>
constexpr inline range_generator<T> range(T min, T max, T step = 1) {
    return {min, max, step};
}

/// Container operations enable with using directives.
namespace ops {

/// Append an element to a vector
template <typename T>
inline vector<T> operator+(const vector<T>& v, const T& vv) {
    auto vc = vector<T>();
    vc.reserve(v.size() + 1);
    vc.insert(vc.end(), v.begin(), v.end());
    vc.push_back(vv);
    return vc;
}

/// Append an element to a vector
template <typename T>
inline vector<T>& operator+=(vector<T>& v, const T& vv) {
    v.push_back(vv);
    return v;
}

/// Append an element to a vector
template <typename T, typename ET>
inline vector<T> operator+(const vector<T>& v, const ET& vv) {
    auto vc = vector<T>();
    vc.reserve(v.size() + 1);
    vc.insert(vc.end(), v.begin(), v.end());
    vc.push_back(vv);
    return vc;
}

/// Append an element to a vector
template <typename T, typename ET>
inline vector<T>& operator+=(vector<T>& v, const ET& vv) {
    v.push_back(vv);
    return v;
}

/// Append a vector to a vector
template <typename T>
inline vector<T> operator+(const vector<T>& v, const vector<T>& vv) {
    auto vc = vector<T>();
    vc.reserve(v.size() + vv.size());
    vc.insert(vc.end(), v.begin(), v.end());
    vc.insert(vc.end(), vv.begin(), vv.end());
    return vc;
}

/// Append a vector to a vector
template <typename T>
inline vector<T>& operator+=(vector<T>& v, const vector<T>& vv) {
    v.insert(v.end(), vv.begin(), vv.end());
    return v;
}

}  // namespace ops

/// Checks if a containers contains a value
template <typename T>
inline bool contains(const vector<T>& v, const T& vv) {
    return find(v.begin(), v.end(), vv) != v.end();
}

/// Checks if a containers contains a value
template <typename K, typename V>
inline bool contains(const map<K, V>& v, const K& vv) {
    return v.find(vv) != v.end();
}

/// Checks if a containers contains a value
template <typename K, typename V>
inline bool contains(const unordered_map<K, V>& v, const K& vv) {
    return v.find(vv) != v.end();
}

/// Checks if a string starts with a prefix.
inline bool startswith(const string& str, const string& substr) {
    if (str.length() < substr.length()) return false;
    for (auto i = 0; i < substr.length(); i++)
        if (str[i] != substr[i]) return false;
    return true;
}

/// Checks if a string ends with a prefix.
inline bool endswith(const string& str, const string& substr) {
    if (str.length() < substr.length()) return false;
    auto offset = str.length() - substr.length();
    for (auto i = 0; i < substr.length(); i++)
        if (str[i + offset] != substr[i]) return false;
    return true;
}

/// Check is a string contains a substring.
inline bool contains(const string& str, const string& substr) {
    return str.find(substr) != str.npos;
}

/// Splits a string into lines at the '\n' character. The line
/// terminator is kept if keep_newline. This function does not work on
/// Window if keep_newline is true.
inline vector<string> splitlines(const string& str, bool keep_newline = false) {
    if (str.empty()) return {};
    auto lines = vector<string>();
    auto line = vector<char>();
    for (auto c : str) {
        if (c == '\n') {
            if (keep_newline) line.push_back(c);
            lines.push_back(string(line.begin(), line.end()));
            line.clear();
        } else {
            line.push_back(c);
        }
    }
    if (!line.empty()) lines.push_back(string(line.begin(), line.end()));
    return lines;
}

/// Partition the string.
inline vector<string> partition(const string& str, const string& split) {
    auto pos = str.find(split);
    if (pos == str.npos) return {str, "", ""};
    return {str.substr(0, pos), split, str.substr(pos + split.length())};
}

/// Splits the string.
inline vector<string> split(const string& str) {
    if (str.empty()) return {};
    auto ret = vector<string>();
    auto lpos = (size_t)0;
    while (lpos != str.npos) {
        auto pos = str.find_first_of(" \t\n\r", lpos);
        if (pos != str.npos) {
            if (pos > lpos) ret.push_back(str.substr(lpos, pos - lpos));
            lpos = pos + 1;
        } else {
            if (lpos < str.size()) ret.push_back(str.substr(lpos));
            lpos = pos;
        }
    }
    return ret;
}

/// Splits the string.
inline vector<string> split(const string& str, const string& substr) {
    if (str.empty()) return {};
    auto ret = vector<string>();
    auto lpos = (size_t)0;
    while (lpos != str.npos) {
        auto pos = str.find(substr, lpos);
        if (pos != str.npos) {
            ret.push_back(str.substr(lpos, pos - lpos));
            lpos = pos + substr.size();
        } else {
            if (lpos < str.size()) ret.push_back(str.substr(lpos));
            lpos = pos;
        }
    }
    return ret;
}

/// Splits the string.
inline vector<string> split(const string& str, char substr) {
    if (str.empty()) return {};
    auto ret = vector<string>();
    auto lpos = (size_t)0;
    while (lpos != str.npos) {
        auto pos = str.find(substr, lpos);
        if (pos != str.npos) {
            ret.push_back(str.substr(lpos, pos - lpos));
            lpos = pos + 1;
        } else {
            if (lpos < str.size()) ret.push_back(str.substr(lpos));
            lpos = pos;
        }
    }
    return ret;
}

/// Strip the string.
inline string rstrip(const string& str) {
    auto pos = str.find_last_not_of(" \t\r\n");
    if (pos == str.npos) return "";
    return str.substr(0, pos + 1);
}

/// Strip the string.
inline string lstrip(const string& str) {
    auto pos = str.find_first_not_of(" \t\r\n");
    if (pos == str.npos) return "";
    return str.substr(pos);
}

/// Strip the string.
inline string strip(const string& str) { return rstrip(lstrip(str)); }

/// Joins a list of string with a string as separator.
inline string join(const vector<string>& strs, const string& sep) {
    auto ret = string();
    auto first = true;
    for (auto& str : strs) {
        if (!first) ret += sep;
        ret += str;
        first = false;
    }
    return ret;
}

/// Converts an ASCII string to lowercase.
inline string lower(const string& str) {
    auto s = str;
    for (auto& c : s) c = tolower(c);
    return s;
}

/// Converts an ASCII string to uppercase.
inline string upper(const string& str) {
    auto s = str;
    for (auto& c : s) c = toupper(c);
    return s;
}

/// Check if a string is space.
inline bool isspace(const string& str) {
    for (auto c : str) {
        if (c != ' ' && c != '\n' && c != '\t' && c != '\r') return false;
    }
    return true;
}

/// Replace s1 with s2 in str.
inline string replace(const string& str, const string& s1, const string& s2) {
    auto s = string();
    auto last = 0;
    auto pos = (int)str.find(s1);
    while (pos != str.npos) {
        s += str.substr(last, pos - last);
        s += s2;
        last = pos + (int)s1.length();
        pos = (int)str.find(s1, last);
    }
    s += str.substr(last);
    return s;
}

/// Get directory name (including '/').
inline string path_dirname(const string& filename) {
    auto pos = filename.rfind('/');
    if (pos == string::npos) pos = filename.rfind('\\');
    if (pos == string::npos) return "";
    return filename.substr(0, pos + 1);
}

/// Get extension (including '.').
inline string path_extension(const string& filename) {
    auto pos = filename.rfind('.');
    if (pos == string::npos) return "";
    return filename.substr(pos);
}

/// Get file basename.
inline string path_basename(const string& filename) {
    auto dirname = path_dirname(filename);
    auto extension = path_extension(filename);
    return filename.substr(
        dirname.size(), filename.size() - dirname.size() - extension.size());
}

/// Get filename without directory (equiv to get_basename() +
/// get_extension()).
inline string path_filename(const string& filename) {
    return path_basename(filename) + path_extension(filename);
}

/// Replace extension.
inline string replace_path_extension(
    const string& filename, const string& ext) {
    return path_dirname(filename) + path_basename(filename) + ext;
}

/// Prepend a string to the extension.
inline string prepend_path_extension(
    const string& filename, const string& prep) {
    return path_dirname(filename) + path_basename(filename) + prep +
           path_extension(filename);
}

/// Splits a path calling the above functions.
inline void split_path(
    const string& filename, string& dirname, string& basename, string& ext) {
    dirname = path_dirname(filename);
    basename = path_basename(filename);
    ext = path_extension(filename);
}

// Argument conversion for format
inline int format_arg(int v) { return v; }
inline float format_arg(float v) { return v; }
inline double format_arg(double v) { return v; }
inline const char* format_arg(bool v) { return (v) ? "true" : "false"; }
inline const char* format_arg(const char* v) { return v; }
inline const char* format_arg(const string& v) { return v.c_str(); }

/// C-like string formatting. This is only meant for short strings with max
/// length 10000 chars. Memory corruption will happen for longer strings.
template <typename... Args>
inline string formatf(const string& fmt, const Args&... args) {
    char buffer[1024 * 16];
    sprintf(buffer, fmt.c_str(), format_arg(args)...);
    return buffer;
}

/// Really-minimal Python like string format. The implementation is not fast
/// nor memory efficient. But it is good enough for some needs.
inline string format(const string& fmt, const vector<string>& args) {
    auto open = false;
    auto cur = 0;
    auto str = string();
    for (auto c : fmt) {
        if (c == '{') {
            str += args[cur++];
            open = true;
        } else if (c == '}') {
            if (!open) throw runtime_error("bad format");
            open = false;
        } else {
            str += c;
        }
    }
    return str;
}

// Implementation of the function below
inline void _format_one(vector<string>& vals) {}
template <typename Arg, typename... Args>
inline void _format_one(
    vector<string>& vals, const Arg& arg, const Args&... args) {
    auto stream = stringstream();
    stream << arg;
    vals.push_back(stream.str());
    _format_one(vals, args...);
}

/// Really-minimal Python like string format. Internally uses streams for
/// generality and supports for now only the '{}' operator. The implementation
/// is not fast nor memory efficient. But it is good enough for some needs.
template <typename... Args>
inline string format(const string& fmt, const Args&... args) {
    auto vals = vector<string>();
    _format_one(vals, args...);
    return format(fmt, vals);
}

/// Wrapper for the above function that prints to stdout.
template <typename... Args>
inline string print(const string& fmt, const Args&... args) {
    printf("%s", format(fmt, args...).c_str());
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// IMAGE CONTAINERS
// -----------------------------------------------------------------------------
namespace ygl {

/// Image of a specified type
template <typename T>
struct image {
    /// empty image constructor
    constexpr image() : _w{0}, _h{0}, _d{} {}
    /// image constructor
    constexpr image(int w, int h, const T& v = {})
        : _w{w}, _h{h}, _d(size_t(w * h), v) {}
    /// image constructor
    constexpr image(int w, int h, const T* v)
        : _w{w}, _h{h}, _d(v, v + w * h) {}

    /// width
    constexpr int width() const { return _w; }
    /// height
    constexpr int height() const { return _h; }
    /// size
    constexpr vec2i size() const { return {_w, _h}; }
    /// check for empty
    constexpr bool empty() const { return _w == 0 || _h == 0; }
    /// check for empty
    constexpr explicit operator bool() const { return _w != 0 && _h != 0; }

    /// reallocate memory
    void resize(int w, int h, const T& v = {}) {
        _w = w;
        _h = h;
        _d.resize(_w * _h);
    }
    /// reallocate memory
    void assign(int w, int h, const T& v) {
        _w = w;
        _h = h;
        _d.assign(_w * _h, v);
    }

    /// set values
    void set(const T& v) { _d.assign(_w * _h, v); }

    /// element access
    constexpr T& operator[](const vec2i& ij) { return _d[ij.y * _w + ij.x]; }
    /// element access
    constexpr const T& operator[](const vec2i& ij) const {
        return _d[ij.y * _w + ij.x];
    }
    /// element access
    constexpr T& at(const vec2i& ij) { return _d.at(ij.y * _w + ij.x); }
    /// element access
    constexpr const T& at(const vec2i& ij) const {
        return _d.at(ij.y * _w + ij.x);
    }
    /// element access
    constexpr T& at(int i, int j) { return _d.at(j * _w + i); }
    /// element access
    constexpr const T& at(int i, int j) const { return _d.at(j * _w + i); }

    /// data access
    constexpr T* data() { return _d.data(); }
    /// data access
    constexpr const T* data() const { return _d.data(); }

   private:
    int _w, _h;
    vector<T> _d;
};

/// 1-dimensional float image
using image1f = image<vec<float, 1>>;
/// 2-dimensional float image
using image2f = image<vec<float, 2>>;
/// 3-dimensional float image
using image3f = image<vec<float, 3>>;
/// 4-dimensional float image
using image4f = image<vec<float, 4>>;

/// 4-dimensional byte image
using image4b = image<vec<byte, 4>>;

/// float image
using imagef = image<float>;

}  // namespace ygl

// -----------------------------------------------------------------------------
// IMAGE OPERATIONS
// -----------------------------------------------------------------------------
namespace ygl {

/// Approximate conversion from srgb.
inline vec3f srgb_to_linear(const vec3b& srgb) {
    return pow(byte_to_float(srgb), 2.2f);
}

/// Approximate conversion from srgb.
inline vec4f srgb_to_linear(const vec4b& srgb) {
    return {pow(byte_to_float(srgb.x), 2.2f), pow(byte_to_float(srgb.y), 2.2f),
        pow(byte_to_float(srgb.z), 2.2f), byte_to_float(srgb.w)};
}

/// Approximate conversion to srgb.
inline vec3b linear_to_srgb(const vec3f& lin) {
    return float_to_byte(pow(lin, 1 / 2.2f));
}

/// Approximate conversion to srgb.
inline vec4b linear_to_srgb(const vec4f& lin) {
    return {float_to_byte(pow(lin.x, 1 / 2.2f)),
        float_to_byte(pow(lin.y, 1 / 2.2f)),
        float_to_byte(pow(lin.z, 1 / 2.2f)), float_to_byte(lin.w)};
}

/// Tone mapping configurations
enum struct tonemap_type { none = 0, srgb, gamma, filmic };

/// Tone mapping type names
inline const vector<pair<string, tonemap_type>>& tonemap_names() {
    static auto names = vector<pair<string, tonemap_type>>{
        {"none", tonemap_type::none}, {"srgb", tonemap_type::srgb},
        {"gamma", tonemap_type::gamma}, {"filmic", tonemap_type::filmic}};
    return names;
}

#if 1
/// Tone map with a fitted filmic curve.
///
/// Implementation from
/// https://knarkowicz.wordpress.com/2016/01/06/aces-filmic-tone-mapping-curve/
inline vec3f tonemap_filmic(const vec3f& hdr) {
    // rescale
    auto x = hdr * 2.05f;
    // fitted values
    float a = 2.51f, b = 0.03f, c = 2.43f, d = 0.59f, e = 0.14f;
    auto y = ((x * (a * x + b)) / (x * (c * x + d) + e));
    return pow(clamp(y, 0.0f, 1.0f), 1 / 2.2f);
}
#else
inline float tonemap_filmic(float x) {
    auto y =
        (x * (x * (x * (x * 2708.7142 + 6801.1525) + 1079.5474) + 1.1614649) -
            0.00004139375) /
        (x * (x * (x * (x * 983.38937 + 4132.0662) + 2881.6522) + 128.35911) +
            1.0);
    return (float)std::max(y, 0.0);
}
#endif

/// Tone mapping HDR to LDR images.
inline void tonemap_image(const image<vec4f>& hdr, image<vec4b>& ldr,
    tonemap_type tm, float exposure, float gamma) {
    ldr.resize(hdr.width(), hdr.height());
    auto scale = pow(2.0f, exposure);
    for (auto j = 0; j < hdr.height(); j++) {
        for (auto i = 0; i < hdr.width(); i++) {
            auto h = hdr[{i, j}];
            h.xyz() *= scale;
            switch (tm) {
                case tonemap_type::none: break;
                case tonemap_type::srgb:
                    h.xyz() = pow(h.xyz(), 1 / 2.2f);
                    break;
                case tonemap_type::gamma:
                    h.xyz() = pow(h.xyz(), 1 / gamma);
                    break;
                case tonemap_type::filmic:
                    h.xyz() = tonemap_filmic(h.xyz());
                    break;
            }
            ldr[{i, j}] = float_to_byte(h);
        }
    }
}

/// Tone mapping HDR to LDR images.
inline image<vec4b> tonemap_image(
    const image<vec4f>& hdr, tonemap_type tm, float exposure, float gamma) {
    auto ldr = image<vec4b>();
    tonemap_image(hdr, ldr, tm, exposure, gamma);
    return ldr;
}

/// Image over operator
inline void image_over(
    vec4f* img, int width, int height, int nlayers, vec4f** layers) {
    for (auto i = 0; i < width * height; i++) {
        img[i] = {0, 0, 0, 0};
        auto weight = 1.0f;
        for (auto l = 0; l < nlayers; l++) {
            img[i].x += layers[l][i].x * layers[l][i].w * weight;
            img[i].y += layers[l][i].y * layers[l][i].w * weight;
            img[i].z += layers[l][i].z * layers[l][i].w * weight;
            img[i].w += layers[l][i].w * weight;
            weight *= (1 - layers[l][i].w);
        }
        if (img[i].w) {
            img[i].x /= img[i].w;
            img[i].y /= img[i].w;
            img[i].z /= img[i].w;
        }
    }
}

/// Image over operator
inline void image_over(
    vec4b* img, int width, int height, int nlayers, vec4b** layers) {
    for (auto i = 0; i < width * height; i++) {
        auto comp = zero4f;
        auto weight = 1.0f;
        for (auto l = 0; l < nlayers && weight > 0; l++) {
            auto w = byte_to_float(layers[l][i].w);
            comp.x += byte_to_float(layers[l][i].x) * w * weight;
            comp.y += byte_to_float(layers[l][i].y) * w * weight;
            comp.z += byte_to_float(layers[l][i].z) * w * weight;
            comp.w += w * weight;
            weight *= (1 - w);
        }
        if (comp.w) {
            img[i].x = float_to_byte(comp.x / comp.w);
            img[i].y = float_to_byte(comp.y / comp.w);
            img[i].z = float_to_byte(comp.z / comp.w);
            img[i].w = float_to_byte(comp.w);
        } else {
            img[i] = {0, 0, 0, 0};
        }
    }
}

/// Convert HSV to RGB
///
/// Implementatkion from
/// http://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
inline vec4b hsv_to_rgb(const vec4b& hsv) {
    vec4b rgb = {0, 0, 0, hsv.w};
    byte region, remainder, p, q, t;

    byte h = hsv.x, s = hsv.y, v = hsv.z;

    if (s == 0) {
        rgb.x = v;
        rgb.y = v;
        rgb.z = v;
        return rgb;
    }

    region = h / 43;
    remainder = (h - (region * 43)) * 6;

    p = (v * (255 - s)) >> 8;
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    switch (region) {
        case 0:
            rgb.x = v;
            rgb.y = t;
            rgb.z = p;
            break;
        case 1:
            rgb.x = q;
            rgb.y = v;
            rgb.z = p;
            break;
        case 2:
            rgb.x = p;
            rgb.y = v;
            rgb.z = t;
            break;
        case 3:
            rgb.x = p;
            rgb.y = q;
            rgb.z = v;
            break;
        case 4:
            rgb.x = t;
            rgb.y = p;
            rgb.z = v;
            break;
        default:
            rgb.x = v;
            rgb.y = p;
            rgb.z = q;
            break;
    }

    return rgb;
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// EXAMPLE IMAGES
// -----------------------------------------------------------------------------
namespace ygl {

/// Make a grid image
inline image<vec4b> make_grid_image(int width, int height, int tile = 64,
    const vec4b& c0 = {90, 90, 90, 255},
    const vec4b& c1 = {128, 128, 128, 255}) {
    image<vec4b> pixels(width, height);
    for (int j = 0; j < width; j++) {
        for (int i = 0; i < height; i++) {
            auto c = i % tile == 0 || i % tile == tile - 1 || j % tile == 0 ||
                     j % tile == tile - 1;
            pixels.at(i, j) = (c) ? c0 : c1;
        }
    }
    return pixels;
}

/// Make a checkerboard image
inline image<vec4b> make_checker_image(int width, int height, int tile = 64,
    const vec4b& c0 = {90, 90, 90, 255},
    const vec4b& c1 = {128, 128, 128, 255}) {
    image<vec4b> pixels(width, height);
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            auto c = (i / tile + j / tile) % 2 == 0;
            pixels.at(i, j) = (c) ? c0 : c1;
        }
    }
    return pixels;
}

/// Make an image with bumps and dimples.
inline image<vec4b> make_bumpdimple_image(
    int width, int height, int tile = 64) {
    image<vec4b> pixels(width, height);
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            auto c = (i / tile + j / tile) % 2 == 0;
            auto ii = i % tile - tile / 2, jj = j % tile - tile / 2;
            auto r =
                sqrt(float(ii * ii + jj * jj)) / sqrt(float(tile * tile) / 4);
            auto h = 0.5f;
            if (r < 0.5f) { h += (c) ? (0.5f - r) : -(0.5f - r); }
            auto g = float_to_byte(h);
            pixels.at(i, j) = vec4b{g, g, g, 255};
        }
    }
    return pixels;
}

/// Make a uv colored grid
inline image<vec4b> make_ramp_image(int width, int height, const vec4b& c0,
    const vec4b& c1, bool srgb = false) {
    image<vec4b> pixels(width, height);
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            auto u = (float)i / (float)width;
            if (srgb) {
                pixels.at(i, j) = linear_to_srgb(
                    srgb_to_linear(c0) * (1 - u) + srgb_to_linear(c1) * u);
            } else {
                pixels.at(i, j) = float_to_byte(
                    byte_to_float(c0) * (1 - u) + byte_to_float(c1) * u);
            }
        }
    }
    return pixels;
}

/// Make a gamma ramp image
inline image<vec4b> make_gammaramp_image(int width, int height) {
    image<vec4b> pixels(width, height);
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            auto u = j / float(height - 1);
            if (i < width / 3) u = pow(u, 2.2f);
            if (i > (width * 2) / 3) u = pow(u, 1 / 2.2f);
            auto c = (unsigned char)(u * 255);
            pixels.at(i, j) = {c, c, c, 255};
        }
    }
    return pixels;
}

/// Make a gamma ramp image
inline image<vec4f> make_gammaramp_imagef(int width, int height) {
    image<vec4f> pixels(width, height);
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            auto u = j / float(height - 1);
            if (i < width / 3) u = pow(u, 2.2f);
            if (i > (width * 2) / 3) u = pow(u, 1 / 2.2f);
            pixels.at(i, j) = {u, u, u, 1};
        }
    }
    return pixels;
}

/// Make an image color with red/green in the [0,1] range. Helpful to visualize
/// uv texture coordinate application.
inline image<vec4b> make_uv_image(int width, int height) {
    image<vec4b> pixels(width, height);
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            auto r = float_to_byte(i / (float)(width - 1));
            auto g = float_to_byte(j / (float)(height - 1));
            pixels.at(i, j) = vec4b{r, g, 0, 255};
        }
    }
    return pixels;
}

/// Make a uv colored grid
inline image<vec4b> make_uvgrid_image(
    int width, int height, int tile = 64, bool colored = true) {
    image<vec4b> pixels(width, height);
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            byte ph = 32 * (i / (height / 8));
            byte pv = 128;
            byte ps = 64 + 16 * (7 - j / (height / 8));
            if (i % (tile / 2) && j % (tile / 2)) {
                if ((i / tile + j / tile) % 2)
                    pv += 16;
                else
                    pv -= 16;
            } else {
                pv = 196;
                ps = 32;
            }
            pixels.at(i, j) = (colored) ? hsv_to_rgb({ph, ps, pv, 255}) :
                                          vec4b{pv, pv, pv, 255};
        }
    }
    return pixels;
}

/// Make a uv recusive colored grid
inline image<vec4b> make_recuvgrid_image(
    int width, int height, int tile = 64, bool colored = true) {
    image<vec4b> pixels(width, height);
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            byte ph = 32 * (i / (height / 8));
            byte pv = 128;
            byte ps = 64 + 16 * (7 - j / (height / 8));
            if (i % (tile / 2) && j % (tile / 2)) {
                if ((i / tile + j / tile) % 2)
                    pv += 16;
                else
                    pv -= 16;
                if ((i / (tile / 4) + j / (tile / 4)) % 2)
                    pv += 4;
                else
                    pv -= 4;
                if ((i / (tile / 8) + j / (tile / 8)) % 2)
                    pv += 1;
                else
                    pv -= 1;
            } else {
                pv = 196;
                ps = 32;
            }
            pixels.at(i, j) = (colored) ? hsv_to_rgb({ph, ps, pv, 255}) :
                                          vec4b{pv, pv, pv, 255};
        }
    }
    return pixels;
}

/// Comvert a bump map to a normal map.
inline image<vec4b> bump_to_normal_map(
    const image<vec4b>& img, float scale = 1) {
    image<vec4b> norm(img.width(), img.height());
    for (int j = 0; j < img.height(); j++) {
        for (int i = 0; i < img.width(); i++) {
            auto i1 = (i + 1) % img.width(), j1 = (j + 1) % img.height();
            auto p00 = img.at(i, j), p10 = img.at(i1, j), p01 = img.at(i, j1);
            auto g00 = (float(p00.x) + float(p00.y) + float(p00.z)) / (3 * 255);
            auto g01 = (float(p01.x) + float(p01.y) + float(p01.z)) / (3 * 255);
            auto g10 = (float(p10.x) + float(p10.y) + float(p10.z)) / (3 * 255);
            auto n = vec3f{scale * (g00 - g10), scale * (g00 - g01), 1.0f};
            n = normalize(n) * 0.5f + vec3f{0.5f, 0.5f, 0.5f};
            auto c =
                vec4b{byte(n.x * 255), byte(n.y * 255), byte(n.z * 255), 255};
            norm.at(i, j) = c;
        }
    }
    return norm;
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// IMAGE LOADING/SAVING
// -----------------------------------------------------------------------------
namespace ygl {

#if YGL_IMAGEIO

/// Check if an image is HDR based on filename
inline bool is_hdr_filename(const string& filename) {
    auto ext = path_extension(filename);
    return ext == ".hdr" || ext == ".exr";
}

/// Loads an ldr image.
inline image4b load_image4b(const string& filename) {
    auto w = 0, h = 0, c = 0;
    auto pixels = unique_ptr<byte>(stbi_load(filename.c_str(), &w, &h, &c, 4));
    if (!pixels) return {};
    return image4b(w, h, (vec4b*)pixels.get());
}

/// Loads an hdr image.
inline image4f load_image4f(const string& filename) {
    auto ext = path_extension(filename);
    auto w = 0, h = 0, c = 0;
    auto pixels = unique_ptr<float>(nullptr);
    if (ext == ".exr") {
        auto pixels_ = (float*)nullptr;
        if (!LoadEXR(&pixels_, &w, &h, filename.c_str(), nullptr))
            pixels = unique_ptr<float>(pixels_);
    } else {
        pixels = unique_ptr<float>(stbi_loadf(filename.c_str(), &w, &h, &c, 4));
    }
    if (!pixels) return {};
    return image4f(w, h, (vec4f*)pixels.get());
}

/// Saves an ldr image.
inline bool save_image4b(const string& filename, const image4b& img) {
    if (path_extension(filename) == ".png") {
        return stbi_write_png(filename.c_str(), img.width(), img.height(), 4,
            (byte*)img.data(), img.width() * 4);
    } else if (path_extension(filename) == ".jpg") {
        return stbi_write_jpg(filename.c_str(), img.width(), img.height(), 4,
            (byte*)img.data(), 75);
    } else {
        return false;
    }
}

/// Saves an hdr image.
inline bool save_image4f(const string& filename, const image4f& img) {
    if (path_extension(filename) == ".hdr") {
        return stbi_write_hdr(
            filename.c_str(), img.width(), img.height(), 4, (float*)img.data());
    } else if (path_extension(filename) == ".exr") {
        return !SaveEXR(
            (float*)img.data(), img.width(), img.height(), 4, filename.c_str());
    } else {
        return false;
    }
}

/// Loads an image
inline vector<float> load_imagef(
    const string& filename, int& width, int& height, int& ncomp) {
    auto pixels = stbi_loadf(filename.c_str(), &width, &height, &ncomp, 0);
    if (!pixels) return {};
    auto ret = vector<float>(pixels, pixels + width * height * ncomp);
    free(pixels);
    return ret;
}

/// Loads an image
inline vector<byte> load_image(
    const string& filename, int& width, int& height, int& ncomp) {
    auto pixels = stbi_load(filename.c_str(), &width, &height, &ncomp, 0);
    if (!pixels) return {};
    auto ret = vector<byte>(pixels, pixels + width * height * ncomp);
    free(pixels);
    return ret;
}

/// Loads an image from memory.
inline vector<float> load_imagef_from_memory(const string& filename,
    const byte* data, int length, int& width, int& height, int& ncomp) {
    auto pixels =
        stbi_loadf_from_memory(data, length, &width, &height, &ncomp, 0);
    if (!pixels) return {};
    auto ret = vector<float>(pixels, pixels + width * height * ncomp);
    free(pixels);
    return ret;
}

/// Loads an image from memory.
inline vector<byte> load_image_from_memory(const string& filename,
    const byte* data, int length, int& width, int& height, int& ncomp) {
    auto pixels =
        stbi_load_from_memory(data, length, &width, &height, &ncomp, 0);
    if (!pixels) return {};
    auto ret = vector<byte>(pixels, pixels + width * height * ncomp);
    free(pixels);
    return ret;
}

/// Saves an image
inline bool save_imagef(const string& filename, int width, int height,
    int ncomp, const float* hdr) {
    if (path_extension(filename) == ".hdr") {
        return stbi_write_hdr(filename.c_str(), width, height, ncomp, hdr);
    } else {
        return false;
    }
}

/// Saves an image
inline bool save_image(
    const string& filename, int width, int height, int ncomp, const byte* ldr) {
    if (path_extension(filename) == ".png") {
        return stbi_write_png(
            filename.c_str(), width, height, ncomp, ldr, width * ncomp);
    } else if (path_extension(filename) == ".jpg") {
        return stbi_write_jpg(filename.c_str(), width, height, ncomp, ldr, 75);
    } else {
        return false;
    }
}

/// Save an HDR or LDR image with tonemapping based on filename
bool save_image(const string& filename, const image4f& hdr, float exposure,
    tonemap_type tonemap, float gamma) {
    if (is_hdr_filename(filename)) {
        return save_image4f(filename, hdr);
    } else {
        auto ldr = image4b(hdr.width(), hdr.height());
        tonemap_image(hdr, ldr, tonemap, exposure, gamma);
        return save_image4b(filename, ldr);
    }
}

/// Filter for resizing
enum struct resize_filter {
    /// default
    def = 0,
    /// box filter
    box = 1,
    /// triangle filter
    triangle = 2,
    /// cubic spline
    cubic_spline = 3,
    /// Catmull-Rom interpolating sline
    catmull_rom = 4,
    /// Mitchel-Netrevalli filter with B=1/3, C=1/3
    mitchell = 5
};

/// Edge mode for resizing
enum struct resize_edge {
    /// default
    def = 0,
    /// clamp
    clamp = 1,
    /// reflect
    reflect = 2,
    /// wrap
    wrap = 3,
    /// zero
    zero = 4
};

/// Resize image.
inline void resize_image(const image4f& img, image4f& res_img,
    resize_filter filter = resize_filter::def,
    resize_edge edge = resize_edge::def, bool premultiplied_alpha = false) {
    static const auto filter_map = map<resize_filter, stbir_filter>{
        {resize_filter::def, STBIR_FILTER_DEFAULT},
        {resize_filter::box, STBIR_FILTER_BOX},
        {resize_filter::triangle, STBIR_FILTER_TRIANGLE},
        {resize_filter::cubic_spline, STBIR_FILTER_CUBICBSPLINE},
        {resize_filter::catmull_rom, STBIR_FILTER_CATMULLROM},
        {resize_filter::mitchell, STBIR_FILTER_MITCHELL}};

    static const auto edge_map =
        map<resize_edge, stbir_edge>{{resize_edge::def, STBIR_EDGE_CLAMP},
            {resize_edge::clamp, STBIR_EDGE_CLAMP},
            {resize_edge::reflect, STBIR_EDGE_REFLECT},
            {resize_edge::wrap, STBIR_EDGE_WRAP},
            {resize_edge::zero, STBIR_EDGE_ZERO}};

    stbir_resize_float_generic((float*)img.data(), img.width(), img.height(),
        sizeof(vec4f) * img.width(), (float*)res_img.data(), res_img.width(),
        res_img.height(), sizeof(vec4f) * res_img.width(), 4, 3,
        (premultiplied_alpha) ? STBIR_FLAG_ALPHA_PREMULTIPLIED : 0,
        edge_map.at(edge), filter_map.at(filter), STBIR_COLORSPACE_LINEAR,
        nullptr);
}

/// Resize image.
inline void resize_image(const image4b& img, image4b& res_img,
    resize_filter filter = resize_filter::def,
    resize_edge edge = resize_edge::def, bool premultiplied_alpha = false) {
    static const auto filter_map = map<resize_filter, stbir_filter>{
        {resize_filter::def, STBIR_FILTER_DEFAULT},
        {resize_filter::box, STBIR_FILTER_BOX},
        {resize_filter::triangle, STBIR_FILTER_TRIANGLE},
        {resize_filter::cubic_spline, STBIR_FILTER_CUBICBSPLINE},
        {resize_filter::catmull_rom, STBIR_FILTER_CATMULLROM},
        {resize_filter::mitchell, STBIR_FILTER_MITCHELL}};

    static const auto edge_map =
        map<resize_edge, stbir_edge>{{resize_edge::def, STBIR_EDGE_CLAMP},
            {resize_edge::clamp, STBIR_EDGE_CLAMP},
            {resize_edge::reflect, STBIR_EDGE_REFLECT},
            {resize_edge::wrap, STBIR_EDGE_WRAP},
            {resize_edge::zero, STBIR_EDGE_ZERO}};

    stbir_resize_uint8_generic((unsigned char*)img.data(), img.width(),
        img.height(), sizeof(vec4b) * img.width(),
        (unsigned char*)res_img.data(), res_img.width(), res_img.height(),
        sizeof(vec4b) * res_img.width(), 4, 3,
        (premultiplied_alpha) ? STBIR_FLAG_ALPHA_PREMULTIPLIED : 0,
        edge_map.at(edge), filter_map.at(filter), STBIR_COLORSPACE_LINEAR,
        nullptr);
}

#endif

}  // namespace ygl

// -----------------------------------------------------------------------------
// GENERIC IMAGE CONTAINERS AND OPERATIONS
// -----------------------------------------------------------------------------
namespace ygl {

/// Generic image that contains either an HDR or an LDR image, giving access
/// to both. This is helpful when writing viewers or generic image
/// manipulation code
struct gimage {
    /// image path
    string filename;
    /// HDR image content
    image<vec4f> hdr;
    /// LDR image content
    image<vec4b> ldr;

    /// Check if the image is valid
    operator bool() const { return hdr || ldr; }

    /// image width
    int width() const {
        if (hdr) return hdr.width();
        if (ldr) return ldr.width();
        return 0;
    }

    /// image height
    int height() const {
        if (hdr) return hdr.height();
        if (ldr) return ldr.height();
        return 0;
    }

    /// access to pixel values
    vec4f& at4f(const vec2i& ij) { return hdr.at(ij); }
    /// access to pixel values
    const vec4f& at4f(const vec2i& ij) const { return hdr.at(ij); }
    /// access to pixel values
    vec4b& at4b(const vec2i& ij) { return ldr.at(ij); }
    /// access to pixel values
    const vec4b& at4b(const vec2i& ij) const { return ldr.at(ij); }

    /// guarded access to pixel values
    vec4f lookup4f(const vec2i& ij, const vec4f& def = zero4f) const {
        if (ij.x < 0 || ij.x >= width() || ij.y < 0 || ij.y > height())
            return def;
        if (hdr) return hdr.at(ij);
        if (ldr) return srgb_to_linear(ldr.at(ij));
        return def;
    }
    /// guarded access to pixel values
    vec4b lookup4b(const vec2i& ij, const vec4b& def = zero4b) const {
        if (ij.x < 0 || ij.x >= width() || ij.y < 0 || ij.y > height())
            return def;
        if (ldr) return ldr.at(ij);
        if (hdr) return linear_to_srgb(hdr.at(ij));
        return def;
    }
};

#if YGL_IMAGEIO
/// Loads a generic image
gimage load_gimage(const string& filename) {
    auto img = gimage();
    img.filename = filename;
    if (is_hdr_filename(filename)) {
        img.hdr = load_image4f(filename);
    } else {
        img.ldr = load_image4b(filename);
    }
    if (!img) { throw runtime_error("cannot load image " + img.filename); }
    return img;
}
#endif

}  // namespace ygl

// -----------------------------------------------------------------------------
// BVH FOR RAY INTERSECTION AND CLOSEST ELEMENT
// -----------------------------------------------------------------------------
namespace ygl {

// number of primitives to avoid splitting on
constexpr const int bvh_minprims = 4;

/// BVH tree node containing its bounds, indices to the BVH arrays of either
/// sorted primitives or internal nodes, whether its a leaf or an internal node,
/// and the split axis. Leaf and internal nodes are identical, except that
/// indices refer to primitives for leaf nodes or other nodes for internal
/// nodes. See bvh_tree for more details.
///
/// This is an internal data structure.
struct bvh_node {
    /// bounding box
    bbox3f bbox;
    /// index to the first sorted primitive/node
    uint32_t start;
    /// number of primitives/nodes
    uint16_t count;
    /// whether it is a leaf
    uint8_t isleaf;
    /// slit axis
    uint8_t axis;
};

/// BVH tree, stored as a node array. The tree structure is encoded using array
/// indices instead of pointers, both for speed but also to simplify code.
/// BVH nodes indices refer to either the node array, for internal nodes,
/// or a primitive array, for leaf nodes. BVH trees may contain only one type
/// of geometric primitive, like points, lines, triangle or shape other BVHs.
/// To handle multiple primitive types and transformed primitices, build
/// a two-level hierarchy with the outer BVH, the scene BVH, containing inner
/// BVHs, shape BVHs, each of which of a uniform primitive type.
///
/// This is an internal data structure.
struct bvh_tree {
    /// sorted array of internal nodes
    vector<bvh_node> nodes;
    /// sorted elements
    vector<int> sorted_prim;
};

// Struct that pack a bounding box, its associate primitive index, and other
// data for faster hierarchy build.
// This is internal only and should not be used externally.
struct bvh_bound_prim {
    bbox3f bbox;   // bounding box
    vec3f center;  // bounding box center (for faster sort)
    int pid;       // primitive id
};

// Comparison function for each axis
struct bvh_bound_prim_comp {
    int axis;
    float middle;

    bvh_bound_prim_comp(int a, float m = 0) : axis(a), middle(m) {}

    bool operator()(const bvh_bound_prim& a, const bvh_bound_prim& b) const {
        return a.center[axis] < b.center[axis];
    }

    bool operator()(const bvh_bound_prim& a) const {
        return a.center[axis] < middle;
    }
};

// Initializes the BVH node node that contains the primitives sorted_prims
// from start to end, by either splitting it into two other nodes,
// or initializing it as a leaf. When splitting, the heuristic heuristic is
// used and nodes added sequentially in the preallocated nodes array and
// the number of nodes nnodes is updated.
inline void make_bvh_node(bvh_node* node, vector<bvh_node>& nodes,
    bvh_bound_prim* sorted_prims, int start, int end, bool equalsize) {
    // compute node bounds
    node->bbox = invalid_bbox3f;
    for (auto i = start; i < end; i++) node->bbox += sorted_prims[i].bbox;

    // decide whether to create a leaf
    if (end - start <= bvh_minprims) {
        // makes a leaf node
        node->isleaf = true;
        node->start = start;
        node->count = end - start;
    } else {
        // choose the split axis and position
        // init to default values
        auto axis = 0;
        auto mid = (start + end) / 2;

        // compute primintive bounds and size
        auto centroid_bbox = invalid_bbox3f;
        for (auto i = start; i < end; i++)
            centroid_bbox += sorted_prims[i].center;
        auto centroid_size = diagonal(centroid_bbox);

        // check if it is not possible to split
        if (centroid_size == zero3f) {
            // we failed to split for some reasons
            node->isleaf = true;
            node->start = start;
            node->count = end - start;
        } else {
            // split along largest
            auto largest_axis = max_element_idx(centroid_size);

            // check heuristic
            if (equalsize) {
                // split the space in the middle along the largest axis
                axis = largest_axis;
                mid = (int)(std::partition(sorted_prims + start,
                                sorted_prims + end,
                                bvh_bound_prim_comp(largest_axis,
                                    center(centroid_bbox)[largest_axis])) -
                            sorted_prims);
            } else {
                // balanced tree split: find the largest axis of the bounding
                // box and split along this one right in the middle
                axis = largest_axis;
                mid = (start + end) / 2;
                std::nth_element(sorted_prims + start, sorted_prims + mid,
                    sorted_prims + end, bvh_bound_prim_comp(largest_axis));
            }

            // check correctness
            assert(axis >= 0 && mid > 0);
            assert(mid > start && mid < end);

            // makes an internal node
            node->isleaf = false;
            // perform the splits by preallocating the child nodes and recurring
            node->axis = axis;
            node->start = (int)nodes.size();
            node->count = 2;
            nodes.emplace_back();
            nodes.emplace_back();
            // build child nodes
            make_bvh_node(&nodes[node->start], nodes, sorted_prims, start, mid,
                equalsize);
            make_bvh_node(&nodes[node->start + 1], nodes, sorted_prims, mid,
                end, equalsize);
        }
    }
}

/// Build a BVH from a set of primitives.
template <typename ElemBbox>
inline bvh_tree* build_bvh(
    int nprims, bool equalsize, const ElemBbox& elem_bbox) {
    // allocate if needed
    auto bvh = new bvh_tree();

    // prepare prims
    auto bound_prims = vector<bvh_bound_prim>(nprims);
    for (auto i = 0; i < nprims; i++) {
        bound_prims[i].pid = i;
        bound_prims[i].bbox = elem_bbox(i);
        bound_prims[i].center = center(bound_prims[i].bbox);
    }

    // clear bvh
    bvh->nodes.clear();
    bvh->sorted_prim.clear();

    // allocate nodes (over-allocate now then shrink)
    bvh->nodes.reserve(nprims * 2);

    // start recursive splitting
    bvh->nodes.emplace_back();
    make_bvh_node(
        &bvh->nodes[0], bvh->nodes, bound_prims.data(), 0, nprims, equalsize);

    // shrink back
    bvh->nodes.shrink_to_fit();

    // init sorted element arrays
    // for shared memory, stored pointer to the external data
    // store the sorted primitive order for BVH walk
    bvh->sorted_prim.resize(nprims);
    for (int i = 0; i < nprims; i++) {
        bvh->sorted_prim[i] = bound_prims[i].pid;
    }

    // done
    return bvh;
}

/// Build a triangles BVH.
inline bvh_tree* build_triangles_bvh(int ntriangles, const vec3i* triangles,
    const vec3f* pos, bool equal_size = true) {
    return build_bvh(ntriangles, equal_size, [triangles, pos](int eid) {
        auto f = triangles[eid];
        return triangle_bbox(pos[f.x], pos[f.y], pos[f.z]);
    });
}

/// Build a triangles BVH.
inline bvh_tree* build_triangles_bvh(const vector<vec3i>& triangles,
    const vector<vec3f>& pos, bool equal_size = true) {
    return build_triangles_bvh(
        triangles.size(), triangles.data(), pos.data(), equal_size);
}

/// Build a quads BVH.
inline bvh_tree* build_quads_bvh(
    int nquads, const vec4i* quads, const vec3f* pos, bool equal_size = true) {
    return build_bvh(nquads, equal_size, [quads, pos](int eid) {
        auto f = quads[eid];
        return quad_bbox(pos[f.x], pos[f.y], pos[f.z], pos[f.w]);
    });
}

/// Build a quads BVH.
inline bvh_tree* build_quads_bvh(const vector<vec4i>& quads,
    const vector<vec3f>& pos, bool equal_size = true) {
    return build_quads_bvh(quads.size(), quads.data(), pos.data(), equal_size);
}

/// Build a lines BVH.
inline bvh_tree* build_lines_bvh(int nlines, const vec2i* lines,
    const vec3f* pos, const float* radius, bool equal_size = true) {
    return build_bvh(nlines, equal_size, [lines, pos, radius](int eid) {
        auto f = lines[eid];
        return line_bbox(pos[f.x], pos[f.y], radius[f.x], radius[f.y]);
    });
}

/// Build a lines BVH.
inline bvh_tree* build_lines_bvh(const vector<vec2i>& lines,
    const vector<vec3f>& pos, const vector<float>& radius,
    bool equal_size = true) {
    return build_lines_bvh(
        lines.size(), lines.data(), pos.data(), radius.data(), equal_size);
}

/// Build a points BVH.
inline bvh_tree* build_points_bvh(int npoints, const int* points,
    const vec3f* pos, const float* radius, bool equal_size = true) {
    return build_bvh(npoints, equal_size, [points, pos, radius](int eid) {
        auto f = points[eid];
        return point_bbox(pos[f], radius[f]);
    });
}

/// Build a points BVH.
inline bvh_tree* build_points_bvh(const vector<int>& points,
    const vector<vec3f>& pos, const vector<float>& radius,
    bool equal_size = true) {
    return build_points_bvh(
        points.size(), points.data(), pos.data(), radius.data(), equal_size);
}

/// Build a points BVH.
inline bvh_tree* build_points_bvh(int npoints, const vec3f* pos,
    const float* radius, bool equal_size = true) {
    return build_bvh(npoints, equal_size, [pos, radius](int eid) {
        return point_bbox(pos[eid], (radius) ? radius[eid] : 0);
    });
}

/// Build a points BVH.
inline bvh_tree* build_points_bvh(int npoints, const vector<vec3f>& pos,
    const vector<float>& radius, bool equal_size = true) {
    return build_points_bvh(npoints, pos.data(), radius.data(), equal_size);
}

/// Recursively recomputes the node bounds for a shape bvh
template <typename ElemBbox>
inline void refit_bvh(bvh_tree* bvh, int nodeid, const ElemBbox& elem_bbox) {
    // refit
    auto node = &bvh->nodes[nodeid];
    node->bbox = invalid_bbox3f;
    if (node->isleaf) {
        for (auto i = 0; i < node->count; i++) {
            auto idx = bvh->sorted_prim[node->start + i];
            node->bbox += elem_bbox(idx);
        }
    } else {
        for (auto i = 0; i < node->count; i++) {
            auto idx = node->start + i;
            refit_bvh(bvh, idx, elem_bbox);
            node->bbox += bvh->nodes[idx].bbox;
        }
    }
}

/// Refit triangles bvh
inline void refit_triangles_bvh(
    bvh_tree* bvh, const vec3i* triangles, const vec3f* pos) {
    refit_bvh(bvh, 0, [triangles, pos](int eid) {
        auto f = triangles[eid];
        return triangle_bbox(pos[f.x], pos[f.y], pos[f.z]);
    });
}

/// Refit triangles bvh
inline void refit_triangles_bvh(
    bvh_tree* bvh, const vector<vec3i>& triangles, const vector<vec3f>& pos) {
    refit_triangles_bvh(bvh, triangles.data(), pos.data());
}

/// Refit quads bvh
inline void refit_quads_bvh(
    bvh_tree* bvh, const vec4i* quads, const vec3f* pos) {
    refit_bvh(bvh, 0, [quads, pos](int eid) {
        auto f = quads[eid];
        return quad_bbox(pos[f.x], pos[f.y], pos[f.z], pos[f.w]);
    });
}

/// Refit quads bvh
inline void refit_quads_bvh(
    bvh_tree* bvh, const vector<vec4i>& quads, const vector<vec3f>& pos) {
    refit_quads_bvh(bvh, quads.data(), pos.data());
}

/// Refit lines bvh
inline void refit_lines_bvh(
    bvh_tree* bvh, const vec2i* lines, const vec3f* pos, const float* radius) {
    refit_bvh(bvh, 0, [lines, pos, radius](int eid) {
        auto f = lines[eid];
        return line_bbox(pos[f.x], pos[f.y], radius[f.x], radius[f.y]);
    });
}

/// Refit lines bvh
inline void refit_lines_bvh(bvh_tree* bvh, const vector<vec2i>& lines,
    const vector<vec3f>& pos, const vector<float>& radius) {
    refit_lines_bvh(bvh, lines.data(), pos.data(), radius.data());
}

/// Refit points bvh
inline void refit_points_bvh(
    bvh_tree* bvh, const int* points, const vec3f* pos, const float* radius) {
    refit_bvh(bvh, 0, [points, pos, radius](int eid) {
        auto f = points[eid];
        return point_bbox(pos[f], (radius) ? radius[f] : 0);
    });
}

/// Refit points bvh
inline void refit_points_bvh(bvh_tree* bvh, const vector<int>& points,
    const vector<vec3f>& pos, const vector<float>& radius) {
    refit_points_bvh(bvh, points.data(), pos.data(), radius.data());
}
/// Refit points bvh
inline void refit_points_bvh(
    bvh_tree* bvh, const vec3f* pos, const float* radius) {
    refit_bvh(bvh, 0,
        [pos, radius](int eid) { return point_bbox(pos[eid], radius[eid]); });
}

/// Refit lines bvh
inline void refit_points_bvh(
    bvh_tree* bvh, const vector<vec3f>& pos, const vector<float>& radius) {
    refit_points_bvh(bvh, pos.data(), radius.data());
}

/// Intersect ray with a bvh.
template <typename Isec>
inline bool intersect_bvh(const bvh_tree* bvh, const ray3f& ray_,
    bool early_exit, float& ray_t, int& eid, const Isec& intersect_elem) {
    // node stack
    int node_stack[64];
    auto node_cur = 0;
    node_stack[node_cur++] = 0;

    // shared variables
    auto hit = false;

    // copy ray to modify it
    auto ray = ray_;

    // prepare ray for fast queries
    auto ray_dinv = vec3f{1, 1, 1} / ray.d;
    auto ray_dsign = vec3i{(ray_dinv.x < 0) ? 1 : 0, (ray_dinv.y < 0) ? 1 : 0,
        (ray_dinv.z < 0) ? 1 : 0};
    auto ray_reverse = vec<bool, 4>{
        (bool)ray_dsign.x, (bool)ray_dsign.y, (bool)ray_dsign.z, false};

    // walking stack
    while (node_cur) {
        // grab node
        auto node = bvh->nodes[node_stack[--node_cur]];

        // intersect bbox
        if (!intersect_check_bbox(ray, ray_dinv, ray_dsign, node.bbox))
            continue;

        // intersect node, switching based on node type
        // for each type, iterate over the the primitive list
        if (!node.isleaf) {
            // for internal nodes, attempts to proceed along the
            // split axis from smallest to largest nodes
            if (ray_reverse[node.axis]) {
                for (auto i = 0; i < node.count; i++) {
                    auto idx = node.start + i;
                    node_stack[node_cur++] = idx;
                    assert(node_cur < 64);
                }
            } else {
                for (auto i = node.count - 1; i >= 0; i--) {
                    auto idx = node.start + i;
                    node_stack[node_cur++] = idx;
                    assert(node_cur < 64);
                }
            }
        } else {
            for (auto i = 0; i < node.count; i++) {
                auto idx = bvh->sorted_prim[node.start + i];
                if (intersect_elem(idx, ray, ray_t)) {
                    hit = true;
                    ray.tmax = ray_t;
                    eid = idx;
                    if (early_exit) return true;
                }
            }
        }
    }

    return hit;
}

/// Finds the closest element with a bvh.
template <typename OverlapElem>
inline bool overlap_bvh(const bvh_tree* bvh, const vec3f& pos, float max_dist,
    bool early_exit, float& dist, int& eid, const OverlapElem& overlap_elem) {
    // node stack
    int node_stack[64];
    auto node_cur = 0;
    node_stack[node_cur++] = 0;

    // hit
    auto hit = false;

    // walking stack
    while (node_cur) {
        // grab node
        auto node = bvh->nodes[node_stack[--node_cur]];

        // intersect bbox
        if (!distance_check_bbox(pos, max_dist, node.bbox)) continue;

        // intersect node, switching based on node type
        // for each type, iterate over the the primitive list
        if (!node.isleaf) {
            // internal node
            for (auto idx = node.start; idx < node.start + node.count; idx++) {
                node_stack[node_cur++] = idx;
                assert(node_cur < 64);
            }
        } else {
            for (auto i = 0; i < node.count; i++) {
                auto idx = bvh->sorted_prim[node.start + i];
                if (overlap_elem(idx, pos, max_dist, dist)) {
                    hit = true;
                    max_dist = dist;
                    eid = idx;
                    if (early_exit) return true;
                }
            }
        }
    }

    return hit;
}

/// Intersect a triangle BVH
inline bool intersect_triangles_bvh(const bvh_tree* bvh, const vec3i* triangles,
    const vec3f* pos, const ray3f& ray, bool early_exit, float& ray_t, int& eid,
    vec3f& euv) {
    return intersect_bvh(bvh, ray, early_exit, ray_t, eid,
        [&triangles, &pos, &euv](int eid, const ray3f& ray, float& ray_t) {
            const auto& f = triangles[eid];
            return intersect_triangle(
                ray, pos[f.x], pos[f.y], pos[f.z], ray_t, euv);
        });
}

/// Intersect a triangle BVH
inline bool intersect_triangles_bvh(const bvh_tree* bvh,
    const vector<vec3i>& triangles, const vector<vec3f>& pos, const ray3f& ray,
    bool early_exit, float& ray_t, int& eid, vec3f& euv) {
    return intersect_triangles_bvh(
        bvh, triangles.data(), pos.data(), ray, early_exit, ray_t, eid, euv);
}

/// Intersect a quad BVH
inline bool intersect_quads_bvh(const bvh_tree* bvh, const vec4i* quads,
    const vec3f* pos, const ray3f& ray, bool early_exit, float& ray_t, int& eid,
    vec4f& euv) {
    return intersect_bvh(bvh, ray, early_exit, ray_t, eid,
        [&quads, &pos, &euv](int eid, const ray3f& ray, float& ray_t) {
            const auto& f = quads[eid];
            return intersect_quad(
                ray, pos[f.x], pos[f.y], pos[f.z], pos[f.w], ray_t, euv);
        });
}

/// Intersect a quad BVH
inline bool intersect_quads_bvh(const bvh_tree* bvh, const vector<vec4i>& quads,
    const vector<vec3f>& pos, const ray3f& ray, bool early_exit, float& ray_t,
    int& eid, vec4f& euv) {
    return intersect_quads_bvh(
        bvh, quads.data(), pos.data(), ray, early_exit, ray_t, eid, euv);
}

/// Intersect a line BVH
inline bool intersect_lines_bvh(const bvh_tree* bvh, const vec2i* lines,
    const vec3f* pos, const float* radius, const ray3f& ray, bool early_exit,
    float& ray_t, int& eid, vec2f& euv) {
    return intersect_bvh(bvh, ray, early_exit, ray_t, eid,
        [&lines, &pos, &radius, &euv](int eid, const ray3f& ray, float& ray_t) {
            auto f = lines[eid];
            return intersect_line(
                ray, pos[f.x], pos[f.y], radius[f.x], radius[f.y], ray_t, euv);
        });
}

/// Intersect a line BVH
inline bool intersect_lines_bvh(const bvh_tree* bvh, const vector<vec2i>& lines,
    const vector<vec3f>& pos, const vector<float>& radius, const ray3f& ray,
    bool early_exit, float& ray_t, int& eid, vec2f& euv) {
    return intersect_lines_bvh(bvh, lines.data(), pos.data(), radius.data(),
        ray, early_exit, ray_t, eid, euv);
}

/// Intersect a point BVH
inline bool intersect_points_bvh(const bvh_tree* bvh, const int* points,
    const vec3f* pos, const float* radius, const ray3f& ray, bool early_exit,
    float& ray_t, int& eid) {
    return intersect_bvh(bvh, ray, early_exit, ray_t, eid,
        [&points, &pos, &radius](int eid, const ray3f& ray, float& ray_t) {
            auto f = points[eid];
            return intersect_point(ray, pos[f], radius[f], ray_t);
        });
}

/// Intersect a point BVH
inline bool intersect_points_bvh(const bvh_tree* bvh, const vector<int>& points,
    const vector<vec3f>& pos, const vector<float>& radius, const ray3f& ray,
    bool early_exit, float& ray_t, int& eid) {
    return intersect_points_bvh(bvh, points.data(), pos.data(), radius.data(),
        ray, early_exit, ray_t, eid);
}

/// Intersect a point BVH
inline bool intersect_points_bvh(const bvh_tree* bvh, const vec3f* pos,
    const float* radius, const ray3f& ray, bool early_exit, float& ray_t,
    int& eid) {
    return intersect_bvh(bvh, ray, early_exit, ray_t, eid,
        [&pos, &radius](int eid, const ray3f& ray, float& ray_t) {
            return intersect_point(ray, pos[eid], radius[eid], ray_t);
        });
}

/// Intersect a point BVH
inline bool intersect_points_bvh(const bvh_tree* bvh, const vector<vec3f>& pos,
    const vector<float>& radius, const ray3f& ray, bool early_exit,
    float& ray_t, int& eid) {
    return intersect_points_bvh(
        bvh, pos.data(), radius.data(), ray, early_exit, ray_t, eid);
}

/// Intersect a triangle BVH
inline bool overlap_triangles_bvh(const bvh_tree* bvh, const vec3i* triangles,
    const vec3f* pos, const float* radius, const vec3f& pt, float max_dist,
    bool early_exit, float& dist, int& eid, vec3f& euv) {
    return overlap_bvh(bvh, pt, max_dist, early_exit, dist, eid,
        [&triangles, &pos, &radius, &euv](
            int eid, const vec3f& pt, float max_dist, float& dist) {
            auto f = triangles[eid];
            return overlap_triangle(pt, max_dist, pos[f.x], pos[f.y], pos[f.z],
                (radius) ? radius[f.x] : 0, (radius) ? radius[f.y] : 0,
                (radius) ? radius[f.z] : 0, dist, euv);
        });
}

/// Intersect a triangle BVH
inline bool overlap_triangles_bvh(const bvh_tree* bvh,
    const vector<vec3i>& triangles, const vector<vec3f>& pos,
    const vector<float>& radius, const vec3f& pt, float max_dist,
    bool early_exit, float& dist, int& eid, vec3f& euv) {
    return overlap_triangles_bvh(bvh, triangles.data(), pos.data(),
        radius.data(), pt, max_dist, early_exit, dist, eid, euv);
}

/// Intersect a quad BVH
inline bool overlap_quads_bvh(const bvh_tree* bvh, const vec4i* quads,
    const vec3f* pos, const float* radius, const vec3f& pt, float max_dist,
    bool early_exit, float& dist, int& eid, vec4f& euv) {
    return overlap_bvh(bvh, pt, max_dist, early_exit, dist, eid,
        [&quads, &pos, &radius, &euv](
            int eid, const vec3f& pt, float max_dist, float& dist) {
            auto f = quads[eid];
            return overlap_quad(pt, max_dist, pos[f.x], pos[f.y], pos[f.z],
                pos[f.w], (radius) ? radius[f.x] : 0,
                (radius) ? radius[f.y] : 0, (radius) ? radius[f.z] : 0,
                (radius) ? radius[f.w] : 0, dist, euv);
        });
}

/// Intersect a quad BVH
inline bool overlap_quads_bvh(const bvh_tree* bvh, const vector<vec4i>& quads,
    const vector<vec3f>& pos, const vector<float>& radius, const vec3f& pt,
    float max_dist, bool early_exit, float& dist, int& eid, vec4f& euv) {
    return overlap_quads_bvh(bvh, quads.data(), pos.data(), radius.data(), pt,
        max_dist, early_exit, dist, eid, euv);
}

/// Intersect a line BVH
inline bool overlap_lines_bvh(const bvh_tree* bvh, const vec2i* lines,
    const vec3f* pos, const float* radius, const vec3f& pt, float max_dist,
    bool early_exit, float& dist, int& eid, vec2f& euv) {
    return overlap_bvh(bvh, pt, max_dist, early_exit, dist, eid,
        [&lines, &pos, &radius, &euv](
            int eid, const vec3f& pt, float max_dist, float& dist) {
            auto f = lines[eid];
            return overlap_line(pt, max_dist, pos[f.x], pos[f.y],
                (radius) ? radius[f.x] : 0, (radius) ? radius[f.y] : 0, dist,
                euv);
        });
}

/// Intersect a line BVH
inline bool overlap_lines_bvh(const bvh_tree* bvh, const vector<vec2i>& lines,
    const vector<vec3f>& pos, const vector<float>& radius, const vec3f& pt,
    float max_dist, bool early_exit, float& dist, int& eid, vec2f& euv) {
    return overlap_lines_bvh(bvh, lines.data(), pos.data(), radius.data(), pt,
        max_dist, early_exit, dist, eid, euv);
}

/// Intersect a point BVH
inline bool overlap_points_bvh(const bvh_tree* bvh, const int* points,
    const vec3f* pos, const float* radius, const vec3f& pt, float max_dist,
    bool early_exit, float& dist, int& eid) {
    return overlap_bvh(bvh, pt, max_dist, early_exit, dist, eid,
        [&points, &pos, &radius](
            int eid, const vec3f& pt, float max_dist, float& dist) {
            auto f = points[eid];
            return overlap_point(
                pt, max_dist, pos[f], (radius) ? radius[f] : 0, dist);
        });
}

/// Intersect a point BVH
inline bool overlap_points_bvh(const bvh_tree* bvh, const vector<int>& points,
    const vector<vec3f>& pos, const vector<float>& radius, const vec3f& pt,
    float max_dist, bool early_exit, float& dist, int& eid) {
    return overlap_points_bvh(bvh, points.data(), pos.data(), radius.data(), pt,
        max_dist, early_exit, dist, eid);
}

/// Intersect a point BVH
inline bool overlap_points_bvh(const bvh_tree* bvh, const vec3f* pos,
    const float* radius, const vec3f& pt, float max_dist, bool early_exit,
    float& dist, int& eid) {
    return overlap_bvh(bvh, pt, max_dist, early_exit, dist, eid,
        [&pos, &radius](int eid, const vec3f& pt, float max_dist, float& dist) {
            return overlap_point(pt, max_dist, pos[eid], radius[eid], dist);
        });
}

/// Intersect a point BVH
inline bool overlap_points_bvh(const bvh_tree* bvh, const vector<vec3f>& pos,
    const vector<float>& radius, const vec3f& pt, float max_dist,
    bool early_exit, float& dist, int& eid) {
    return overlap_points_bvh(
        bvh, pos.data(), radius.data(), pt, max_dist, early_exit, dist, eid);
}

/// Finds the overlap between BVH leaf nodes.
template <typename OverlapElem>
void overlap_bvh_elems(const bvh_tree* bvh1, const bvh_tree* bvh2,
    bool skip_duplicates, bool skip_self, vector<vec2i>& overlaps,
    const OverlapElem& overlap_elems) {
    // node stack
    vec2i node_stack[128];
    auto node_cur = 0;
    node_stack[node_cur++] = {0, 0};

    // walking stack
    while (node_cur) {
        // grab node
        auto node_idx = node_stack[--node_cur];
        const auto node1 = bvh1->nodes[node_idx.x];
        const auto node2 = bvh2->nodes[node_idx.y];

        // intersect bbox
        if (!overlap_bbox(node1.bbox, node2.bbox)) continue;

        // check for leaves
        if (node1.isleaf && node2.isleaf) {
            // collide primitives
            for (auto i1 = node1.start; i1 < node1.start + node1.count; i1++) {
                for (auto i2 = node2.start; i2 < node2.start + node2.count;
                     i2++) {
                    auto idx1 = bvh1->sorted_prim[i1];
                    auto idx2 = bvh2->sorted_prim[i2];
                    if (skip_duplicates && idx1 > idx2) continue;
                    if (skip_self && idx1 == idx2) continue;
                    if (overlap_elems(idx1, idx2))
                        overlaps.push_back({idx1, idx2});
                }
            }
        } else {
            // descend
            if (node1.isleaf) {
                for (auto idx2 = node2.start; idx2 < node2.start + node2.count;
                     idx2++) {
                    node_stack[node_cur++] = {node_idx.x, (int)idx2};
                    assert(node_cur < 128);
                }
            } else if (node2.isleaf) {
                for (auto idx1 = node1.start; idx1 < node1.start + node1.count;
                     idx1++) {
                    node_stack[node_cur++] = {(int)idx1, node_idx.y};
                    assert(node_cur < 128);
                }
            } else {
                for (auto idx2 = node2.start; idx2 < node2.start + node2.count;
                     idx2++) {
                    for (auto idx1 = node1.start;
                         idx1 < node1.start + node1.count; idx1++) {
                        node_stack[node_cur++] = {(int)idx1, (int)idx2};
                        assert(node_cur < 128);
                    }
                }
            }
        }
    }
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// IMMEDIATE MODE COMMAND LINE PARSER
// -----------------------------------------------------------------------------
namespace ygl {

/// Immediate mode command line parser (opaque type)
struct cmdline_parser;

/// Immediate mode command line parser
struct cmdline_parser {
    // private implementation
    vector<string> _to_parse;    // args left to parse
    vector<string> _used_names;  // used names for check
    string _usage_prog;          // usage prog line
    string _usage_help;          // usage help line
    string _usage_opts;          // usage option lines
    string _usage_args;          // usage argument lines
    bool _usage = false;         // help option triggered
    string _error;               // parse error
};

// cmdline implementation
inline void _check_name(
    cmdline_parser& parser, const string& name, const string& flag, bool opt) {
    if (opt) {
        if (name.size() < 3 || name[0] != '-' || name[1] != '-' ||
            name[2] == '-')
            throw runtime_error("bad name " + name);
    } else {
        if (name.size() < 1 || name[0] == '-')
            throw runtime_error("bad name " + name);
    }
    if (find(parser._used_names.begin(), parser._used_names.end(), name) !=
        parser._used_names.end())
        throw runtime_error("already used " + name);
    parser._used_names.push_back(name);
    if (flag.empty()) return;
    if (flag.size() < 2 || flag[0] != '-' || flag[1] == '-')
        throw runtime_error("bad name " + flag);
    if (find(parser._used_names.begin(), parser._used_names.end(), flag) !=
        parser._used_names.end())
        throw runtime_error("already used " + flag);
    parser._used_names.push_back(flag);
}

// cmdline implementation
template <typename T>
inline void _add_usage_str(cmdline_parser& parser, const string& name,
    const string& flag, bool opt, const string& help, const string& def,
    bool req, const vector<T>& choices) {
    auto stream = stringstream();
    stream << "  " << name;
    if (!flag.empty()) stream << "/" << flag;
    while (stream.str().length() < 32) stream << " ";
    stream << help << " ";
    if (!req) stream << "[" << def << "]";
    stream << "\n";
    if (!choices.empty()) {
        for (auto i = 0; i < 32; i++) stream << " ";
        stream << "(";
        auto first = true;
        for (auto&& c : choices) {
            if (!first) stream << ",";
            stream << c;
            first = false;
        }
        stream << ")";
        stream << "\n";
    }
    if (opt)
        parser._usage_opts += stream.str();
    else
        parser._usage_args += stream.str();
}

// cmdline implementation
template <typename T>
inline void _add_usage(cmdline_parser& parser, const string& name,
    const string& flag, bool opt, const string& help, const T& def, bool req,
    const vector<T>& choices) {
    auto stream = stringstream();
    stream << def;
    _add_usage_str(parser, name, flag, opt, help, stream.str(), req, choices);
}

// cmdline implementation
template <typename T>
inline void _add_usage(cmdline_parser& parser, const string& name,
    const string& flag, bool opt, const string& help, const vector<T>& def,
    bool req, const vector<T>& choices) {
    auto stream = stringstream();
    auto first = true;
    for (auto&& v : def) {
        if (!first) stream << ",";
        stream << v;
        first = false;
    }
    _add_usage_str(parser, name, flag, opt, help, stream.str(), req, choices);
}

// cmdline implementation
inline void _set_error(cmdline_parser& parser, const string& err) {
    if (parser._error.empty()) parser._error = err;
}

/// check unused arguments
inline bool should_exit(cmdline_parser& parser) {
    for (auto&& v : parser._to_parse) {
        if (v[0] == '-')
            _set_error(parser, "unknown option " + v);
        else
            _set_error(parser, "unknown argument " + v);
    }
    return !parser._error.empty();
}

/// returns the usage string
inline string get_usage(const cmdline_parser& parser) {
    auto str = string();
    if (!parser._error.empty()) str += "error: " + parser._error + "\n\n";
    str += parser._usage_prog;
    if (!parser._usage_opts.empty()) str += " [options]";
    if (!parser._usage_args.empty()) str += " <arguments>";
    str += "\n";
    // while (str.size() < 32) str += " ";
    str += parser._usage_help + "\n\n";
    if (!parser._usage_opts.empty())
        str += "options:\n" + parser._usage_opts + "\n";
    if (!parser._usage_args.empty())
        str += "arguments:\n" + parser._usage_args + "\n";
    return str;
}

/// parse a flag from the command line
inline bool parse_flag(cmdline_parser& parser, const string& name,
    const string& flag, const string& help, bool def = false,
    bool req = false) {
    // check names
    _check_name(parser, name, flag, true);
    // skip if error
    if (!parser._error.empty()) return def;
    // find location of option
    auto pos = find(parser._to_parse.begin(), parser._to_parse.end(), name);
    if (pos == parser._to_parse.end())
        pos = find(parser._to_parse.begin(), parser._to_parse.end(), flag);
    if (pos == parser._to_parse.end()) {
        if (req) _set_error(parser, "missing required flag " + name);
        return def;
    }
    // remove parsed arg
    parser._to_parse.erase(pos, pos + 1);
    // done
    return !def;
}

/// parse an option from the command line
template <typename T>
inline T parse_opt(cmdline_parser& parser, const string& name,
    const string& flag, const string& help, const T& def = {}, bool req = false,
    const vector<T>& choices = {}) {
    // check names
    _check_name(parser, name, flag, true);
    // update usage
    _add_usage(parser, name, flag, true, help, def, req, choices);
    // skip if error
    if (!parser._error.empty()) return def;
    // find location of option
    auto pos = find(parser._to_parse.begin(), parser._to_parse.end(), name);
    if (pos == parser._to_parse.end())
        pos = find(parser._to_parse.begin(), parser._to_parse.end(), flag);
    if (pos == parser._to_parse.end()) {
        if (req) _set_error(parser, "missing option " + name);
        return def;
    }
    // check if value exists
    if (pos == parser._to_parse.end() - 1) {
        _set_error(parser, "no value for parameter " + name);
        return def;
    }
    // get value
    auto val = def;
    const auto& arg = *(pos + 1);
    // parse
    auto stream = stringstream(arg);
    stream >> val;
    if (stream.fail()) {
        _set_error(
            parser, "incorrect value \"" + arg + "\" for option " + name);
    }
    // validate if necessary
    if (!choices.empty()) {
        if (find(choices.begin(), choices.end(), val) == choices.end())
            _set_error(
                parser, "incorrect value \"" + arg + "\" for option " + name);
    }
    // remove parsed arg
    parser._to_parse.erase(pos, pos + 2);
    // done
    return val;
}

/// parse an enum option from the command line
template <typename T>
inline T parse_opt(cmdline_parser& parser, const string& name,
    const string& flag, const string& help,
    const vector<pair<string, T>>& key_values, const T& def, bool req = false,
    const vector<T>& choices = {}) {
    auto keys = vector<string>{};
    auto key_def = string();
    for (auto&& kv : key_values) {
        keys.push_back(kv.first);
        if (kv.second == def) key_def = kv.first;
    }
    auto key = parse_opt<string>(parser, name, flag, help, key_def, req, keys);
    if (!parser._error.empty()) return def;
    auto val = def;
    for (auto&& kv : key_values) {
        if (kv.first == key) val = kv.second;
    }
    return val;
}

// parse positional argument from the command line
template <typename T>
inline T parse_arg(cmdline_parser& parser, const string& name,
    const string& help, const T& def = {}, bool req = true,
    const vector<T>& choices = {}) {
    // check names
    _check_name(parser, name, "", false);
    // update usage
    _add_usage(parser, name, "", false, help, def, req, choices);
    // skip if error
    if (!parser._error.empty()) return def;
    // find location of argument
    auto pos = std::find_if(parser._to_parse.begin(), parser._to_parse.end(),
        [](const auto& s) { return s.size() > 0 && s[0] != '-'; });
    if (pos == parser._to_parse.end()) {
        if (req) _set_error(parser, "missing argument " + name);
        return def;
    }
    // get value
    auto val = def;
    const auto& arg = *(pos);
    // parse
    auto stream = stringstream(arg);
    stream >> val;
    if (stream.fail()) {
        _set_error(
            parser, "incorrect value \"" + arg + "\" for argument " + name);
    }
    // validate if necessary
    if (!choices.empty()) {
        if (find(choices.begin(), choices.end(), val) == choices.end())
            _set_error(
                parser, "incorrect value \"" + arg + "\" for argument " + name);
    }
    // remove parsed arg
    parser._to_parse.erase(pos, pos + 1);
    // done
    return val;
}

// parse all remaining positional argument from the command line
template <typename T>
inline vector<T> parse_args(cmdline_parser& parser, const string& name,
    const string& help, const vector<T>& def = {}, bool req = true,
    const vector<T>& choices = {}) {
    // check names
    _check_name(parser, name, "", false);
    // update usage
    _add_usage(parser, name, "", false, help, def, req, choices);
    // skip if error
    if (!parser._error.empty()) return def;
    // search for all params
    auto vals = vector<T>();
    while (true) {
        // find location of argument
        auto pos =
            std::find_if(parser._to_parse.begin(), parser._to_parse.end(),
                [](const auto& s) { return s.size() > 0 && s[0] != '-'; });
        if (pos == parser._to_parse.end()) break;
        // get value
        auto val = T{};
        const auto& arg = *(pos);
        // parse
        auto stream = stringstream(arg);
        stream >> val;
        if (stream.fail()) {
            _set_error(
                parser, "incorrect value \"" + arg + "\" for argument " + name);
        }
        // validate if necessary
        if (!choices.empty()) {
            if (find(choices.begin(), choices.end(), val) == choices.end())
                _set_error(parser,
                    "incorrect value \"" + arg + "\" for argument " + name);
        }
        // remove parsed arg
        parser._to_parse.erase(pos, pos + 1);
        // append value
        vals.push_back(val);
    }
    // check missing
    if (vals.empty()) {
        if (req) _set_error(parser, "missing argument " + name);
        return def;
    }
    // done
    return vals;
}

/// initialize the command line
inline cmdline_parser make_parser(
    int argc, char** argv, const string& prog, const string& help) {
    auto parser = cmdline_parser();
    parser._to_parse = vector<string>(argv + 1, argv + argc);
    parser._usage_prog = (prog.empty()) ? string(argv[0]) : prog;
    parser._usage_help = help;
    parser._usage =
        parse_flag(parser, "--help", "-h", "prints and help message");
    return parser;
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// SIMPLE LOGGER
// -----------------------------------------------------------------------------
namespace ygl {

/// Logger object. A logger can output messages to multiple streams.
/// Use add streams commands for it.
struct logger {
    /// whether to output verbose
    bool _verbose = true;
    /// whether to output to console
    bool _console = true;
    /// file stream for stream output
    FILE* _file = nullptr;

    // cleanup
    ~logger() {
        if (_file) fclose(_file);
    }
};

/// Make a logger with an optional console stream and a verbosity level
inline logger* make_logger(bool console = true, bool verbose = true) {
    auto lgr = new logger();
    lgr->_verbose = verbose;
    lgr->_console = console;
    lgr->_file = nullptr;
    return lgr;
}

/// Add a file stream to a logger.
///
/// - Parameters:
///     - lgr: logger
///     - filename: filename
///     - append: append or write open mode for file logger
///     - short_message: whether to use a short message version
///     - output_level: output level
///     - flush_level: output level
/// - Returns:
///     - true if ok
inline void add_file_stream(logger* lgr, const string& filename, bool append) {
    lgr->_file = fopen(filename.c_str(), (append) ? "at" : "wt");
    if (!lgr->_file) throw runtime_error("could not open file " + filename);
}

/// Get default logger.
/// By default a non-verbose stdout logger is creater.
inline logger* get_default_logger() {
    static auto default_logger = new logger();
    return default_logger;
}

// Log a message. Used internally.
inline void _log_msg(logger* lgr, const string& msg, const char* type) {
    char time_buf[1024];
    auto tm = time(nullptr);
    auto ttm = localtime(&tm);  // TODO: use thread safe version

    // short message for console
    if (lgr->_console) {
        strftime(time_buf, 1024, "%H:%M:%S", ttm);
        printf("%s %s %s\n", time_buf, type, msg.c_str());
        fflush(stdout);
    }

    // long message for file
    if (lgr->_file) {
        strftime(time_buf, 1024, "%Y-%m-%d %H:%M:%S", ttm);
        fprintf(lgr->_file, "%s %s %s\n", time_buf, type, msg.c_str());
    }
}

/// Log a info message
template <typename... Args>
inline void log_info(logger* lgr, const string& msg, const Args&... args) {
    if (!lgr->_verbose) return;
    _log_msg(lgr, format(msg, args...), "INFO ");
}

/// Log a info message
template <typename... Args>
inline void log_warning(logger* lgr, const string& msg, const Args&... args) {
    if (!lgr->_verbose) return;
    _log_msg(lgr, format(msg, args...), "WARN ");
}

/// Log an error message
template <typename... Args>
inline void log_error(logger* lgr, const string& msg, const Args&... args) {
    _log_msg(lgr, format(msg, args...), "ERROR");
}

/// Log a fatal message and exit
template <typename... Args>
inline void log_fatal(logger* lgr, const string& msg, const Args&... args) {
    _log_msg(lgr, format(msg, args...), "FATAL");
    exit(1);
}

/// Adds a file stream to the default logger
inline void add_file_stream(const string& filename, bool append) {
    add_file_stream(get_default_logger(), filename, append);
}

/// Logs a message to the default loggers
template <typename... Args>
inline void log_info(const string& msg, const Args&... args) {
    log_info(get_default_logger(), msg, args...);
}

/// Logs a message to the default loggers
template <typename... Args>
inline void log_error(const string& msg, const Args&... args) {
    log_error(get_default_logger(), msg, args...);
}

/// Logs a message to the default loggers
template <typename... Args>
inline void log_fatal(const string& msg, const Args&... args) {
    log_fatal(get_default_logger(), msg, args...);
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// THREAD POOL
// -----------------------------------------------------------------------------
namespace ygl {

/// Thread pool for concurrency. This code is derived from LLVM ThreadPool
struct thread_pool {
    // initialize the thread pool
    thread_pool(int nthreads = std::thread::hardware_concurrency())
        : _working_threads(0), _stop_flag(false) {
        _threads.reserve(nthreads);
        for (auto tid = 0; tid < nthreads; tid++) {
            _threads.emplace_back([this] { _thread_proc(); });
        }
    }

    // cleanup
    ~thread_pool() {
        {
            std::unique_lock<std::mutex> lock_guard(_queue_lock);
            _stop_flag = true;
        }
        _queue_condition.notify_all();
        for (auto& Worker : _threads) Worker.join();
    }

    // empty the queue
    void _clear_pool() {
        {
            std::unique_lock<std::mutex> lock_guard(_queue_lock);
            _tasks.clear();
        }
        _queue_condition.notify_all();
    }

    // schedule an asynchronous taks
    std::shared_future<void> _run_async(std::function<void()> task) {
        // Wrap the Task in a packaged_task to return a future object.
        std::packaged_task<void()> packaged_task(std::move(task));
        auto future = packaged_task.get_future();
        {
            std::unique_lock<std::mutex> lock_guard(_queue_lock);
            assert(!_stop_flag &&
                   "Queuing a thread during ThreadPool destruction");
            _tasks.push_back(std::move(packaged_task));
        }
        _queue_condition.notify_one();
        return future.share();
    }

    // wait for all tasks to finish
    void _wait() {
        std::unique_lock<std::mutex> lock_guard(_completion_lock);
        _completion_condition.wait(
            lock_guard, [&] { return _tasks.empty() && !_working_threads; });
    }

    // parallel for
    void _parallel_for(int count, const function<void(int idx)>& task) {
        for (auto idx = 0; idx < count; idx++) {
            _run_async([&task, idx]() { task(idx); });
        }
        _wait();
    }

    // implementation -------------------------------------------------
    void _thread_proc() {
        while (true) {
            std::packaged_task<void()> task;
            {
                std::unique_lock<std::mutex> lock_guard(_queue_lock);
                _queue_condition.wait(
                    lock_guard, [&] { return _stop_flag || !_tasks.empty(); });

                if (_stop_flag && _tasks.empty()) return;

                {
                    _working_threads++;
                    std::unique_lock<std::mutex> lock_guard(_completion_lock);
                }
                task = std::move(_tasks.front());
                _tasks.pop_front();
            }

            task();

            {
                std::unique_lock<std::mutex> lock_guard(_completion_lock);
                _working_threads--;
            }

            _completion_condition.notify_all();
        }
    }

    vector<std::thread> _threads;
    std::deque<std::packaged_task<void()>> _tasks;
    std::mutex _queue_lock;
    std::condition_variable _queue_condition;
    std::mutex _completion_lock;
    std::condition_variable _completion_condition;
    std::atomic<unsigned> _working_threads;
    bool _stop_flag = false;
};

/// Makes a thread pool
inline thread_pool* make_pool(
    int nthreads = std::thread::hardware_concurrency()) {
    return new thread_pool(nthreads);
}

/// Runs a task asynchronously onto the global thread pool
inline std::shared_future<void> run_async(
    thread_pool* pool, const function<void()>& task) {
    return pool->_run_async(task);
}

/// Wait for all jobs to finish on the global thread pool
inline void wait_pool(thread_pool* pool) { pool->_wait(); }

/// Clear all jobs on the global thread pool
inline void clear_pool(thread_pool* pool) { pool->_clear_pool(); }

/// Parallel for implementation on the global thread pool
inline void parallel_for(
    thread_pool* pool, int count, const function<void(int idx)>& task) {
    pool->_parallel_for(count, task);
}

/// Global pool
inline thread_pool* get_global_pool() {
    static auto pool = (thread_pool*)nullptr;
    if (!pool) pool = new thread_pool();
    return pool;
}

/// Runs a task asynchronously onto the global thread pool
inline std::shared_future<void> run_async(const function<void()>& task) {
    return run_async(get_global_pool(), task);
}

/// Wait for all jobs to finish on the global thread pool
inline void wait_pool() { wait_pool(get_global_pool()); }

/// Clear all jobs on the global thread pool
inline void clear_pool() { clear_pool(get_global_pool()); }

/// Parallel for implementation on the global thread pool
inline void parallel_for(int count, const function<void(int idx)>& task) {
    parallel_for(get_global_pool(), count, task);
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// TIMER
// -----------------------------------------------------------------------------
namespace ygl {

/// A simple wrapper for std::chrono.
struct timer {
    /// initialize a timer and start it if necessary
    timer(bool autostart = true) {
        if (autostart) start();
    }

    /// start a timer
    void start() {
        _start = std::chrono::steady_clock::now();
        _started = true;
    }

    /// stops a timer
    void stop() {
        _end = std::chrono::steady_clock::now();
        _started = false;
    }

    /// elapsed time
    double elapsed() {
        if (_started) stop();
        std::chrono::duration<double> diff = (_end - _start);
        return diff.count();
    }

   private:
    bool _started = false;
    std::chrono::time_point<std::chrono::steady_clock> _start, _end;
};

}  // namespace ygl

// -----------------------------------------------------------------------------
// SIMPLE SCENE SUPPORT
// -----------------------------------------------------------------------------
namespace ygl {

/// Scene Texture
struct texture {
    /// name
    string name;
    /// path
    string path;
    /// if loaded, ldr image
    image4b ldr;
    /// if loaded, hdr image
    image4f hdr;

    /// get texture width
    int width() const {
        if (ldr) return ldr.width();
        if (hdr) return hdr.width();
        return 0;
    }
    /// get texture height
    int height() const {
        if (ldr) return ldr.height();
        if (hdr) return hdr.height();
        return 0;
    }
};

/// Scene Texture Additional Information
struct texture_info {
    /// texture pointer
    texture* txt = nullptr;
    /// wrap s coordinate
    bool wrap_s = true;
    /// wrap t coordinate
    bool wrap_t = true;
    /// linear interpolation
    bool linear = true;
    /// mipmaping
    bool mipmap = true;
    /// texture strength (occlusion and normal)
    float scale = 1;

    /// check whether the texture if present
    operator bool() const { return (bool)txt; }
};

/// Material type
enum struct material_type {
    /// Microfacet material type (OBJ)
    specular_roughness = 0,
    /// Base and metallic material (metallic-roughness in glTF)
    metallic_roughness = 1,
    /// Diffuse and specular material (specular-glossness in glTF)
    specular_glossiness = 2,
};

/// Scene Material
struct material {
    // whole material data -------------------
    /// material name
    string name;
    /// double-sided rendering
    bool double_sided = false;
    /// material type
    material_type mtype = material_type::specular_roughness;

    // color information ---------------------
    /// emission color
    vec3f ke = {0, 0, 0};
    /// diffuse color / base color
    vec3f kd = {0, 0, 0};
    /// specular color / metallic factor
    vec3f ks = {0, 0, 0};
    /// transmission color
    vec3f kt = {0, 0, 0};
    /// roughness
    float rs = 0.0001;
    /// opacity
    float op = 1;

    // textures -------------------------------
    /// emission texture
    texture_info ke_txt = {};
    /// diffuse texture
    texture_info kd_txt = {};
    /// specular texture
    texture_info ks_txt = {};
    /// transmission texture
    texture_info kt_txt = {};
    /// roughness texture
    texture_info rs_txt = {};
    /// bump map texture (heighfield)
    texture_info bump_txt = {};
    /// displacement map texture (heighfield)
    texture_info disp_txt = {};
    /// normal texture
    texture_info norm_txt = {};
    /// occlusion texture
    texture_info occ_txt = {};
};

/// Shape data represented as an indexed array.
/// May contain only one of the points/lines/triangles/quads.
struct shape {
    /// shape name
    string name = "";
    /// path (used for saving in glTF)
    string path = "";
    /// shape material
    material* mat = nullptr;

    // shape elements -------------------------
    /// points
    vector<int> points;
    /// lines
    vector<vec2i> lines;
    /// triangles
    vector<vec3i> triangles;
    /// quads
    vector<vec4i> quads;
    /// face-varying indices for position
    vector<vec4i> quads_pos;
    /// face-varying indices for normal
    vector<vec4i> quads_norm;
    /// face-varying indices for texcoord
    vector<vec4i> quads_texcoord;

    // vertex data ----------------------------
    /// per-vertex position (3 float)
    vector<vec3f> pos;
    /// per-vertex normals (3 float)
    vector<vec3f> norm;
    /// per-vertex texcoord (2 float)
    vector<vec2f> texcoord;
    /// per-vertex second texcoord (2 float)
    vector<vec2f> texcoord1;
    /// per-vertex color (4 float)
    vector<vec4f> color;
    /// per-vertex radius (1 float)
    vector<float> radius;
    /// per-vertex tangent space (4 float)
    vector<vec4f> tangsp;

    // computed data --------------------------
    /// element CDF for sampling
    vector<float> elem_cdf;
    /// BVH
    bvh_tree* bvh = nullptr;
    /// bounding box (needs to be updated explicitly)
    bbox3f bbox = invalid_bbox3f;

    // clean
    ~shape() {
        if (bvh) delete bvh;
    }
};

/// Shape instance.
struct instance {
    // name
    string name;
    /// transform frame
    frame3f frame = identity_frame3f;
    /// shape instance
    shape* shp = nullptr;

    // computed data --------------------------
    /// bounding box (needs to be updated explicitly)
    bbox3f bbox = invalid_bbox3f;

    /// instance transform as matrix
    mat4f xform() const { return to_mat(frame); }
};

/// Scene Camera
struct camera {
    /// name
    string name;
    /// transform frame
    frame3f frame = identity_frame3f;
    /// ortho cam
    bool ortho = false;
    /// vertical field of view
    float yfov = 2;
    /// aspect ratio
    float aspect = 16.0f / 9.0f;
    /// focus distance
    float focus = 1;
    /// lens aperture
    float aperture = 0;
    /// near plane distance
    float near = 0.01f;
    /// far plane distance
    float far = 10000;
};

/// Envinonment map
struct environment {
    /// name
    string name;
    /// transform frame
    frame3f frame = identity_frame3f;
    /// emission coefficient
    vec3f ke = {0, 0, 0};
    /// emission texture
    texture_info ke_txt = {};
};

/// Light, either an instance or an environment.
/// This is only used internally to avoid looping over all objects every time.
struct light {
    /// instance
    instance* ist = nullptr;
    /// environment
    environment* env = nullptr;
};

/// Scene
struct scene {
    /// shape array
    vector<shape*> shapes;
    /// instance array
    vector<instance*> instances;
    /// material array
    vector<material*> materials;
    /// texture array
    vector<texture*> textures;
    /// camera array
    vector<camera*> cameras;
    /// environment array
    vector<environment*> environments;

    /// light array
    vector<light*> lights;

    // computed data --------------------------
    /// BVH
    bvh_tree* bvh = nullptr;
    /// bounding box (needs to be updated explicitly)
    bbox3f bbox = invalid_bbox3f;

    /// cleanup
    ~scene() {
        for (auto v : shapes)
            if (v) delete v;
        for (auto v : instances)
            if (v) delete v;
        for (auto v : materials)
            if (v) delete v;
        for (auto v : textures)
            if (v) delete v;
        for (auto v : cameras)
            if (v) delete v;
        for (auto v : environments)
            if (v) delete v;
        for (auto light : lights)
            if (light) delete light;
        if (bvh) delete bvh;
    }
};

/// Shape value interpolated using barycentric coordinates
template <typename T>
inline T eval_barycentric(
    const shape* shp, const vector<T>& vals, int eid, const vec4f& euv) {
    if (vals.empty()) return T();
    if (!shp->triangles.empty()) {
        return eval_barycentric_triangle(vals, shp->triangles[eid], euv);
    } else if (!shp->lines.empty()) {
        return eval_barycentric_line(vals, shp->lines[eid], euv);
    } else if (!shp->points.empty()) {
        return eval_barycentric_point(vals, shp->points[eid], euv);
    } else if (!shp->quads.empty()) {
        return eval_barycentric_quad(vals, shp->quads[eid], euv);
    } else {
        return vals[eid];  // points
    }
}

/// Shape position interpolated using barycentric coordinates
inline vec3f eval_pos(const shape* shp, int eid, const vec4f& euv) {
    return eval_barycentric(shp, shp->pos, eid, euv);
}

/// Shape normal interpolated using barycentric coordinates
inline vec3f eval_norm(const shape* shp, int eid, const vec4f& euv) {
    return normalize(eval_barycentric(shp, shp->norm, eid, euv));
}

/// Shape texcoord interpolated using barycentric coordinates
inline vec2f eval_texcoord(const shape* shp, int eid, const vec4f& euv) {
    return eval_barycentric(shp, shp->texcoord, eid, euv);
}

/// Shape texcoord interpolated using barycentric coordinates
inline vec4f eval_color(const shape* shp, int eid, const vec4f& euv) {
    return eval_barycentric(shp, shp->color, eid, euv);
}

/// Shape tangent space interpolated using barycentric coordinates
inline vec4f eval_tangsp(const shape* shp, int eid, const vec4f& euv) {
    return eval_barycentric(shp, shp->tangsp, eid, euv);
}

/// Instance position interpolated using barycentric coordinates
inline vec3f eval_pos(const instance* ist, int eid, const vec4f& euv) {
    return transform_point(
        ist->frame, eval_barycentric(ist->shp, ist->shp->pos, eid, euv));
}

/// Instance normal interpolated using barycentric coordinates
inline vec3f eval_norm(const instance* ist, int eid, const vec4f& euv) {
    return transform_direction(ist->frame,
        normalize(eval_barycentric(ist->shp, ist->shp->norm, eid, euv)));
}

/// Evaluate a texture
inline vec4f eval_texture(const texture_info& info, const vec2f& texcoord,
    bool srgb = true, const vec4f& def = {1, 1, 1, 1}) {
    if (!info.txt) return def;

    // get texture
    auto txt = info.txt;
    assert(txt->hdr || txt->ldr);

    auto lookup = [&def, &txt, &srgb](int i, int j) {
        if (txt->ldr)
            return (srgb) ? srgb_to_linear(txt->ldr[{i, j}]) :
                            byte_to_float(txt->ldr[{i, j}]);
        else if (txt->hdr)
            return txt->hdr[{i, j}];
        else
            return def;
    };

    // get image width/height
    auto w = txt->width(), h = txt->height();

    // get coordinates normalized for tiling
    auto s = 0.0f, t = 0.0f;
    if (!info.wrap_s) {
        s = clamp(texcoord.x, 0.0f, 1.0f) * w;
    } else {
        s = std::fmod(texcoord.x, 1.0f) * w;
        if (s < 0) s += w;
    }
    if (!info.wrap_t) {
        t = clamp(texcoord.y, 0.0f, 1.0f) * h;
    } else {
        t = std::fmod(texcoord.y, 1.0f) * h;
        if (t < 0) t += h;
    }

    // get image coordinates and residuals
    auto i = clamp((int)s, 0, w - 1), j = clamp((int)t, 0, h - 1);
    auto ii = (i + 1) % w, jj = (j + 1) % h;
    auto u = s - i, v = t - j;

    // nearest lookup
    if (!info.linear) return lookup(i, j);

    // handle interpolation
    return lookup(i, j) * (1 - u) * (1 - v) + lookup(i, jj) * (1 - u) * v +
           lookup(ii, j) * u * (1 - v) + lookup(ii, jj) * u * v;
}

/// Tesselate a shape into basic primitives
inline shape* tesselate_shape(const shape* shp) {
    if (!shp->quads_pos.empty()) {
        auto tshp = new shape();
        tshp->name = shp->name;
        tshp->path = shp->path;
        tshp->mat = shp->mat;
        std::tie(tshp->quads, tshp->pos, tshp->norm, tshp->texcoord) =
            convert_face_varying(shp->quads_pos, shp->quads_norm,
                shp->quads_texcoord, shp->pos, shp->norm, shp->texcoord);
        return tshp;
    }
    return nullptr;
}

/// Tesselate scene shapes and update pointers
inline void tesselate_shapes(scene* scn) {
    for (auto& shp : scn->shapes) {
        auto tshp = tesselate_shape(shp);
        if (!tshp) continue;
        for (auto ist : scn->instances) {
            if (ist->shp == shp) ist->shp = tshp;
        }
        swap(shp, tshp);
        delete tshp;
    }
}

/// Loading options
struct load_options {
    /// Whether to load textures
    bool load_textures = true;
    /// Skip missing files without giving and error
    bool skip_missing = true;
    /// Whether to flip the v coordinate in OBJ
    bool obj_flip_texcoord = true;
    /// Duplicate vertices if smoothing off in OBJ
    bool obj_facet_non_smooth = false;
    /// Whether to flip tr in OBJ
    bool obj_flip_tr = true;
    /// whether to preserve quads
    bool preserve_quads = false;
    /// whether to preserve face-varying faces
    bool preserve_facevarying = false;
};

/// Loads a scene. For now OBJ or glTF are supported.
/// Throws an exception if an error occurs.
inline scene* load_scene(const string& filename, const load_options& opts = {});

/// Save options
struct save_options {
    /// Whether to save textures
    bool save_textures = true;
    /// Skip missing files without giving and error
    bool skip_missing = true;
    /// Whether to flip the v coordinate in OBJ
    bool obj_flip_texcoord = true;
    /// Whether to flip tr in OBJ
    bool obj_flip_tr = true;
    /// Whether to use separate buffers in gltf
    bool gltf_separate_buffers = false;
};

/// Saves a scene. For now OBJ and glTF are supported.
/// Throws an exception if an error occurs.
inline void save_scene(
    const string& filename, const scene* scn, const save_options& opts);

/// Add elements options
struct add_elements_options {
    /// Add missing normal
    bool smooth_normals = true;
    /// Add missing radius for points and lines (<=0 for no adding)
    float pointline_radius = 0;
    /// Add missing trangent space
    bool tangent_space = true;
    /// texture data
    bool texture_data = true;
    /// Add instances
    bool shape_instances = true;
    /// Add default camera
    bool default_camera = true;
    /// Add an empty default environment
    bool default_environment = false;
    /// Add default names
    bool default_names = true;
    /// Add default paths
    bool default_paths = true;

    /// initialize to no element
    static add_elements_options none() {
        auto opts = add_elements_options();
        memset(&opts, 0, sizeof(opts));
        return opts;
    }
};

/// Add elements
inline void add_elements(scene* scn, const add_elements_options& opts = {});

/// Merge scene into one another. Note that the objects are _moved_ from
/// merge_from to merged_into, so merge_from will be empty after this function.
inline void merge_into(scene* merge_into, scene* merge_from);

/// Computes a shape bounding box (quick computation that ignores radius)
inline void update_bounds(shape* shp) {
    shp->bbox = invalid_bbox3f;
    for (auto p : shp->pos) shp->bbox += vec3f(p);
}

/// Updates the instance bounding box
inline void update_bounds(instance* ist, bool do_shape = true) {
    if (do_shape) update_bounds(ist->shp);
    ist->bbox = transform_bbox(ist->frame, ist->shp->bbox);
}

/// Updates the scene and scene's instances bounding boxes
inline void update_bounds(scene* scn, bool do_shapes = true) {
    if (do_shapes) {
        for (auto shp : scn->shapes) update_bounds(shp);
    }
    scn->bbox = invalid_bbox3f;
    if (!scn->instances.empty()) {
        for (auto ist : scn->instances) {
            update_bounds(ist, false);
            scn->bbox += ist->bbox;
        }
    } else {
        for (auto shp : scn->shapes) { scn->bbox += shp->bbox; }
    }
}

/// Flatten scene instances into separate meshes.
inline void flatten_instances(scene* scn) {
    if (scn->instances.empty()) return;
    auto shapes = scn->shapes;
    scn->shapes.clear();
    auto instances = scn->instances;
    scn->instances.clear();
    for (auto ist : instances) {
        if (!ist->shp) continue;
        auto xf = ist->xform();
        auto nshp = new shape(*ist->shp);
        for (auto& p : nshp->pos) p = transform_point(xf, p);
        for (auto& n : nshp->norm) n = transform_direction(xf, n);
        scn->shapes.push_back(nshp);
    }
    for (auto e : shapes) delete e;
    for (auto e : instances) delete e;
}

/// Initialize the lights
inline void update_lights(scene* scn, bool point_only);

/// Print scene information (call update bounds bes before)
inline void print_info(const scene* scn);

/// Build a shape BVH
inline void build_bvh(shape* shp, bool equalsize = true) {
    if (!shp->points.empty()) {
        shp->bvh =
            build_points_bvh(shp->points, shp->pos, shp->radius, equalsize);
    } else if (!shp->lines.empty()) {
        shp->bvh =
            build_lines_bvh(shp->lines, shp->pos, shp->radius, equalsize);
    } else if (!shp->triangles.empty()) {
        shp->bvh = build_triangles_bvh(shp->triangles, shp->pos, equalsize);
    } else if (!shp->quads.empty()) {
        shp->bvh = build_quads_bvh(shp->quads, shp->pos, equalsize);
    } else {
        shp->bvh =
            build_points_bvh(shp->pos.size(), shp->pos, shp->radius, equalsize);
    }
    shp->bbox = shp->bvh->nodes[0].bbox;
}

/// Build a scene BVH
inline void build_bvh(
    scene* scn, bool equalsize = true, bool do_shapes = true) {
    // do shapes
    if (do_shapes) {
        for (auto shp : scn->shapes) build_bvh(shp, equalsize);
    }

    // update instance bbox
    for (auto ist : scn->instances)
        ist->bbox = transform_bbox(ist->frame, ist->shp->bbox);

    // tree bvh
    scn->bvh = build_bvh((int)scn->instances.size(), equalsize,
        [scn](int eid) { return scn->instances[eid]->bbox; });
}

/// Refits a scene BVH
inline void refit_bvh(shape* shp) {
    if (!shp->points.empty()) {
        refit_points_bvh(shp->bvh, shp->points, shp->pos, shp->radius);
    } else if (!shp->lines.empty()) {
        refit_lines_bvh(shp->bvh, shp->lines, shp->pos, shp->radius);
    } else if (!shp->triangles.empty()) {
        refit_triangles_bvh(shp->bvh, shp->triangles, shp->pos);
    } else if (!shp->quads.empty()) {
        refit_quads_bvh(shp->bvh, shp->quads, shp->pos);
    } else {
        refit_points_bvh(shp->bvh, shp->pos, shp->radius);
    }
    shp->bbox = shp->bvh->nodes[0].bbox;
}

/// Refits a scene BVH
inline void refit_bvh(scene* scn, bool do_shapes = true) {
    if (do_shapes) {
        for (auto shp : scn->shapes) refit_bvh(shp);
    }

    // update instance bbox
    for (auto ist : scn->instances)
        ist->bbox = transform_bbox(ist->frame, ist->shp->bbox);

    // recompute bvh bounds
    refit_bvh(
        scn->bvh, 0, [scn](int eid) { return scn->instances[eid]->bbox; });
}

/// Intersect the shape with a ray. Find any interstion if early_exit,
/// otherwise find first intersection.
///
/// - Parameters:
///     - scn: scene to intersect
///     - ray: ray to be intersected
///     - early_exit: whether to stop at the first found hit
///     - ray_t: ray distance at intersection
///     - eid: shape element index
///     - euv: element barycentric coordinates
/// - Returns:
///     - whether it intersected
inline bool intersect_ray(const shape* shp, const ray3f& ray, bool early_exit,
    float& ray_t, int& eid, vec4f& euv) {
    // switch over shape type
    if (!shp->triangles.empty()) {
        if (intersect_triangles_bvh(shp->bvh, shp->triangles, shp->pos, ray,
                early_exit, ray_t, eid, (vec3f&)euv)) {
            euv = {euv.x, euv.y, euv.z, 0};
            return true;
        }
    } else if (!shp->quads.empty()) {
        if (intersect_quads_bvh(shp->bvh, shp->quads, shp->pos, ray, early_exit,
                ray_t, eid, euv)) {
            return true;
        }
    } else if (!shp->lines.empty()) {
        if (intersect_lines_bvh(shp->bvh, shp->lines, shp->pos, shp->radius,
                ray, early_exit, ray_t, eid, (vec2f&)euv)) {
            euv = {euv.x, euv.y, 0, 0};
            return true;
        }
    } else if (!shp->points.empty()) {
        if (intersect_points_bvh(shp->bvh, shp->points, shp->pos, shp->radius,
                ray, early_exit, ray_t, eid)) {
            euv = {1, 0, 0, 0};
            return true;
        }
    } else {
        if (intersect_points_bvh(
                shp->bvh, shp->pos, shp->radius, ray, early_exit, ray_t, eid)) {
            euv = {1, 0, 0, 0};
            return true;
        }
    }

    return false;
}

/// Intersect the instance with a ray. Find any interstion if early_exit,
/// otherwise find first intersection.
///
/// - Parameters:
///     - scn: scene to intersect
///     - ray: ray to be intersected
///     - early_exit: whether to stop at the first found hit
///     - ray_t: ray distance at intersection
///     - eid: shape element index
///     - euv: element barycentric coordinates
/// - Returns:
///     - whether it intersected
inline bool intersect_ray(const instance* ist, const ray3f& ray,
    bool early_exit, float& ray_t, int& eid, vec4f& euv) {
    return intersect_ray(ist->shp, transform_ray_inverse(ist->frame, ray),
        early_exit, ray_t, eid, euv);
}

/// Intersect the scene with a ray. Find any interstion if early_exit,
/// otherwise find first intersection.
///
/// - Parameters:
///     - scn: scene to intersect
///     - ray: ray to be intersected
///     - early_exit: whether to stop at the first found hit
///     - ray_t: ray distance at intersection
///     - iid: instance index
///     - eid: shape element index
///     - euv: element barycentric coordinates
/// - Returns:
///     - whether it intersected
inline bool intersect_ray(const scene* scn, const ray3f& ray, bool early_exit,
    float& ray_t, int& iid, int& eid, vec4f& euv) {
    return intersect_bvh(scn->bvh, ray, early_exit, ray_t, iid,
        [&eid, &euv, early_exit, scn](int iid, const ray3f& ray, float& ray_t) {
            return intersect_ray(
                scn->instances[iid], ray, early_exit, ray_t, eid, euv);
        });
}

/// Surface point.
struct intersection_point {
    /// distance of the hit along the ray or from the point
    float dist = 0;
    /// instance index
    int iid = -1;
    /// shape element index
    int eid = -1;
    /// shape barycentric coordinates
    vec4f euv = zero4f;

    /// check if intersection is valid
    operator bool() const { return eid >= 0; }
};

/// Intersect the scene with a ray. Find any interstion if early_exit,
/// otherwise find first intersection.
///
/// - Parameters:
///     - scn: scene to intersect
///     - ray: ray to be intersected
///     - early_exit: whether to stop at the first found hit
/// - Returns:
///     - intersection record
inline intersection_point intersect_ray(
    const scene* scn, const ray3f& ray, bool early_exit) {
    auto isec = intersection_point();
    if (!intersect_ray(
            scn, ray, early_exit, isec.dist, isec.iid, isec.eid, isec.euv))
        return {};
    return isec;
}

/// Finds the closest element that overlaps a point within a given distance.
///
/// - Parameters:
///     - scn: scene to intersect
///     - pos: point position
///     - max_dist: maximu valid distance
///     - early_exit: whether to stop at the first found hit
///     - dist: distance at intersection
///     - eid: shape element index
///     - euv: element barycentric coordinates
/// - Returns:
///     - whether it intersected
inline bool overlap_point(const shape* shp, const vec3f& pos, float max_dist,
    bool early_exit, float& dist, int& eid, vec4f& euv) {
    // switch over shape type
    if (!shp->triangles.empty()) {
        if (overlap_triangles_bvh(shp->bvh, shp->triangles, shp->pos,
                shp->radius, pos, max_dist, early_exit, dist, eid,
                (vec3f&)euv)) {
            euv = {euv.x, euv.y, euv.z, 0};
            return true;
        }
    } else if (!shp->quads.empty()) {
        if (overlap_quads_bvh(shp->bvh, shp->quads, shp->pos, shp->radius, pos,
                max_dist, early_exit, dist, eid, euv)) {
            return true;
        }
    } else if (!shp->lines.empty()) {
        if (overlap_lines_bvh(shp->bvh, shp->lines, shp->pos, shp->radius, pos,
                max_dist, early_exit, dist, eid, (vec2f&)euv)) {
            euv = {euv.x, euv.y, 0, 0};
            return true;
        }
    } else if (!shp->points.empty()) {
        if (overlap_points_bvh(shp->bvh, shp->points, shp->pos, shp->radius,
                pos, max_dist, early_exit, dist, eid)) {
            euv = {1, 0, 0, 0};
            return true;
        }
    } else {
        if (overlap_points_bvh(shp->bvh, shp->pos, shp->radius, pos, max_dist,
                early_exit, dist, eid)) {
            euv = {1, 0, 0, 0};
        }
        return true;
    }

    return false;
}

/// Finds the closest element that overlaps a point within a given distance.
///
/// - Parameters:
///     - scn: scene to intersect
///     - pos: point position
///     - max_dist: maximu valid distance
///     - early_exit: whether to stop at the first found hit
///     - dist: distance at intersection
///     - eid: shape element index
///     - euv: element barycentric coordinates
/// - Returns:
///     - whether it intersected
inline bool overlap_point(const instance* ist, const vec3f& pos, float max_dist,
    bool early_exit, float& dist, int& eid, vec4f& euv) {
    return overlap_point(ist->shp, transform_point_inverse(ist->frame, pos),
        max_dist, early_exit, dist, eid, euv);
}

/// Finds the closest element that overlaps a point within a given distance.
///
/// - Parameters:
///     - scn: scene to intersect
///     - pos: point position
///     - max_dist: maximu valid distance
///     - early_exit: whether to stop at the first found hit
///     - dist: distance at intersection
///     - iid: instance index
///     - eid: shape element index
///     - euv: element barycentric coordinates
/// - Returns:
///     - whether it intersected
inline bool overlap_point(const scene* scn, const vec3f& pos, float max_dist,
    bool early_exit, float& dist, int& iid, int& eid, vec4f& euv) {
    return overlap_bvh(scn->bvh, pos, max_dist, early_exit, dist, iid,
        [&eid, &euv, early_exit, scn](
            int iid, const vec3f& pos, float max_dist, float& dist) {
            return overlap_point(
                scn->instances[iid], pos, max_dist, early_exit, dist, eid, euv);
        });
}

/// Find the list of overlaps between instance bounds.
inline void overlap_instance_bounds(const scene* scn1, const scene* scn2,
    bool skip_duplicates, bool skip_self, vector<vec2i>& overlaps) {
    overlaps.clear();
    overlap_bvh_elems(scn1->bvh, scn2->bvh, skip_duplicates, skip_self,
        overlaps, [scn1, scn2](int i1, int i2) {
            return overlap_bbox(
                scn1->instances[i1]->bbox, scn2->instances[i2]->bbox);
        });
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// PATH TRACING
// -----------------------------------------------------------------------------
namespace ygl {

// convenient typedef for bytes
using byte = unsigned char;

/// Type of rendering algorithm (shader)
enum struct trace_shader_type {
    /// pathtrace
    pathtrace = 0,
    /// eye hight for quick previews
    eyelight,
    /// direct illumination
    direct,
    /// pathtrace without MIS (usedful ony for debugging)
    pathtrace_nomis,
    /// debug normal
    debug_normal,
    /// debug albedo
    debug_albedo,
    /// debug texcoord
    debug_texcoord,
};

inline const vector<pair<string, trace_shader_type>>& trace_shader_names() {
    static auto names = vector<pair<string, trace_shader_type>>{
        {"path", trace_shader_type::pathtrace},
        {"eye", trace_shader_type::eyelight},
        {"direct", trace_shader_type::direct},
        {"path_nomis", trace_shader_type::pathtrace_nomis},
        {"normal", trace_shader_type::debug_normal},
        {"albedo", trace_shader_type::debug_albedo},
        {"texcoord", trace_shader_type::debug_texcoord},
    };
    return names;
}

/// Random number generator type
enum struct trace_rng_type {
    /// uniform random numbers
    uniform = 0,
    /// stratified random numbers
    stratified,
};

inline const vector<pair<string, trace_rng_type>>& trace_rng_names() {
    static auto names = vector<pair<string, trace_rng_type>>{
        {"uniform", trace_rng_type::uniform},
        {"stratified", trace_rng_type::stratified}};
    return names;
}

/// Filter type
enum struct trace_filter_type {
    /// box filter
    box = 1,
    /// hat filter
    triangle = 2,
    /// cubic spline
    cubic = 3,
    /// Catmull-Rom spline
    catmull_rom = 4,
    /// Mitchell-Netrevalli
    mitchell = 5
};

inline const vector<pair<string, trace_filter_type>>& trace_filter_names() {
    static auto names =
        vector<pair<string, trace_filter_type>>{{"box", trace_filter_type::box},
            {"triangle", trace_filter_type::triangle},
            {"cubic", trace_filter_type::cubic},
            {"catmull-rom", trace_filter_type::catmull_rom},
            {"mitchell", trace_filter_type::mitchell}};
    return names;
}

/// Rendering params
struct trace_params {
    /// camera id
    int camera_id = 0;
    /// width
    int width = 0;
    /// height
    int height = 0;
    /// number of samples
    int nsamples = 256;
    /// sampler type
    trace_shader_type stype = trace_shader_type::pathtrace;
    /// wheter to test transmission in shadows
    bool shadow_notransmission = false;
    /// random number generation type
    trace_rng_type rtype = trace_rng_type::stratified;
    /// filter type
    trace_filter_type ftype = trace_filter_type::box;
    /// ambient lighting
    vec3f amb = {0, 0, 0};
    /// view environment map
    bool envmap_invisible = false;
    /// minimum ray depth
    int min_depth = 3;
    /// maximum ray depth
    int max_depth = 8;
    /// final pixel clamping
    float pixel_clamp = 10;
    /// ray intersection epsilon
    float ray_eps = 1e-4f;
    /// parallel execution
    bool parallel = true;
    /// seed for the random number generators
    uint32_t seed = 0;
    /// block size for parallel batches (probably leave it as is)
    int block_size = 32;
};

/// Make image blocks
inline vector<bbox2i> trace_blocks(const trace_params& params) {
    vector<bbox2i> blocks;
    for (int j = 0; j < params.height; j += params.block_size) {
        for (int i = 0; i < params.width; i += params.block_size) {
            blocks.push_back(
                {{i, j}, {min(i + params.block_size, params.width),
                             min(j + params.block_size, params.height)}});
        }
    }
    return blocks;
}

/// Make a 2D array of random number generators for parallelization
inline image<rng_pcg32> trace_rngs(const trace_params& params) {
    auto rngs = image<rng_pcg32>(params.width, params.height);
    for (auto j : range(params.height)) {
        for (auto i : range(params.width)) {
            rngs[{i, j}] =
                init_rng(params.seed, (j * params.width + i) * 2 + 1);
        }
    }
    return rngs;
}

/// Renders a block of samples
///
/// Notes: It is safe to call the function in parallel on different blocks.
/// But two threads should not access the same pixels at the same time. If
/// the same block is rendered with different samples, samples have to be
/// sequential.
///
/// - Parameters:
///     - scn: trace scene
///     - img: pixel data in RGBA format (width/height in params)
///     - block: range of pixels to render
///     - samples_min, samples_max: range of samples to render
///     - params: trace params
inline void trace_block(const scene* scn, image4f& img, const bbox2i& block,
    int samples_min, int samples_max, image<rng_pcg32>& rngs,
    const trace_params& params);

/// Trace the next samples in [samples_min, samples_max) range.
/// Samples have to be traced consecutively.
inline void trace_samples(const scene* scn, image4f& img, int samples_min,
    int samples_max, image<rng_pcg32>& rngs, const trace_params& params) {
    auto blocks = trace_blocks(params);
    if (params.parallel) {
        parallel_for((int)blocks.size(), [&img, scn, samples_min, samples_max,
                                             &blocks, &params, &rngs](int idx) {
            trace_block(
                scn, img, blocks[idx], samples_min, samples_max, rngs, params);
        });
    } else {
        for (auto idx = 0; idx < (int)blocks.size(); idx++) {
            trace_block(
                scn, img, blocks[idx], samples_min, samples_max, rngs, params);
        }
    }
}

/// Renders a filtered block of samples
///
/// Notes: It is safe to call the function in parallel on different blocks.
/// But two threads should not access the same pixels at the same time. If
/// the same block is rendered with different samples, samples have to be
/// sequential.
///
/// - Parameters:
///     - scn: trace scene
///     - img: pixel data in RGBA format (width/height in params)
///     - acc: accumulation buffer in RGBA format (width/height in params)
///     - weight: weight buffer in float format (width/height in params)
///     - block: range of pixels to render
///     - samples_min, samples_max: range of samples to render
///     - image_mutex: mutex for locking
///     - params: trace params
inline void trace_block_filtered(const scene* scn, image4f& img, image4f& acc,
    imagef& weight, const bbox2i& block, int samples_min, int samples_max,
    image<rng_pcg32>& rngs, std::mutex& image_mutex,
    const trace_params& params);

/// Trace the next samples in [samples_min, samples_max) range.
/// Samples have to be traced consecutively.
inline void trace_filtered_samples(const scene* scn, image4f& img, image4f& acc,
    imagef& weight, int samples_min, int samples_max, image<rng_pcg32>& rngs,
    const trace_params& params) {
    auto blocks = trace_blocks(params);
    std::mutex image_mutex;
    if (params.parallel) {
        parallel_for((int)blocks.size(),
            [&img, &acc, &weight, scn, samples_min, samples_max, &blocks,
                &params, &image_mutex, &rngs](int idx) {
                trace_block_filtered(scn, img, acc, weight, blocks[idx],
                    samples_min, samples_max, rngs, image_mutex, params);
            });
    } else {
        for (auto idx = 0; idx < (int)blocks.size(); idx++) {
            trace_block_filtered(scn, img, acc, weight, blocks[idx],
                samples_min, samples_max, rngs, image_mutex, params);
        }
    }
}

/// Trace the whole image
inline image4f trace_image(const scene* scn, const trace_params& params) {
    auto img = image4f(params.width, params.height);
    auto rngs = trace_rngs(params);
    if (params.ftype == trace_filter_type::box) {
        trace_samples(scn, img, 0, params.nsamples, rngs, params);
    } else {
        auto acc = image4f(params.width, params.height);
        auto weight = imagef(params.width, params.height);
        trace_filtered_samples(
            scn, img, acc, weight, 0, params.nsamples, rngs, params);
    }
    return img;
}

/// Starts an anyncrhounous renderer with a maximum of 256 samples.
inline void trace_async_start(const scene* scn, image4f& img,
    image<rng_pcg32>& rngs, const trace_params& params, thread_pool* pool,
    const function<void(int)>& callback) {
    auto blocks = trace_blocks(params);
    for (auto sample = 0; sample < params.nsamples; sample++) {
        for (auto& block : blocks) {
            auto is_last = (block == blocks.back());
            run_async(pool, [&img, scn, sample, block, &params, callback, &rngs,
                                is_last]() {
                trace_block(scn, img, block, sample, sample + 1, rngs, params);
                if (is_last) callback(sample);
            });
        }
    }
}

/// Stop the asynchronous renderer.
inline void trace_async_stop(thread_pool* pool) {
    if (!pool) return;
    clear_pool(pool);
    wait_pool(pool);
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// WAVEFRONT OBJ SUPPORT
// -----------------------------------------------------------------------------
namespace ygl {

#if YGL_SCENEIO

/// Face vertex
struct obj_vertex {
    /// position
    int pos;
    /// texcoord
    int texcoord;
    /// normal
    int norm;
    /// color [extension]
    int color;
    /// radius [extension]
    int radius;

    /// Constructor (copies members initializing missing ones to -1)
    obj_vertex(int pos = -1, int texcoord = -1, int norm = -1, int color = -1,
        int radius = -1)
        : pos(pos)
        , texcoord(texcoord)
        , norm(norm)
        , color(color)
        , radius(radius) {}
};

// Comparison for unordred_map
inline bool operator==(const obj_vertex& a, const obj_vertex& b) {
    return a.pos == b.pos && a.texcoord == b.texcoord && a.norm == b.norm &&
           a.color == b.color && a.radius == b.radius;
}

/// element type
enum struct obj_element_type : uint16_t {
    /// lists of points
    point = 1,
    /// polylines
    line = 2,
    /// polygon faces
    face = 3,
    /// tetrahedrons
    tetra = 4,
};

/// Element vertex indices
struct obj_element {
    /// starting vertex index
    uint32_t start;
    /// element type
    obj_element_type type;
    /// number of vertices
    uint16_t size;
};

/// Element group
struct obj_group {
    // group data ---------------------------
    /// material name
    string matname;
    /// group name
    string groupname;
    /// smoothing
    bool smoothing = true;

    // element data -------------------------
    /// element vertices
    vector<obj_vertex> verts;
    /// element faces
    vector<obj_element> elems;
};

/// Obj object
struct obj_object {
    // object data --------------------------
    /// object name
    string name;

    // element data -------------------------
    /// element groups
    vector<obj_group> groups;
};

/// Texture information for OBJ
struct obj_texture_info {
    /// the texture path
    string path = "";
    /// whether to clamp tp th edge
    bool clamp = false;
    /// the scale for bump and displacement
    float scale = 1;
    /// the rest of the unknown properties
    unordered_map<string, vector<string>> unknown_props;
};

// comparison for texture info
inline bool operator==(const obj_texture_info& a, const obj_texture_info& b) {
    if (a.path.empty() && b.path.empty()) return true;
    if (a.path != b.path) return false;
    return a.clamp == b.clamp && a.scale == b.scale &&
           a.unknown_props == b.unknown_props;
}

/// OBJ texture. Texture data is loaded only if desired.
struct obj_texture {
    // whole texture data ------------------
    /// texture path
    string path;
    /// Width
    int width = 0;
    /// Height
    int height = 0;
    /// Number of Channels
    int ncomp = 0;
    /// Buffer data for 8-bit images
    vector<uint8_t> datab;
    /// Buffer data for float images
    vector<float> dataf;
};

/// OBJ material
struct obj_material {
    // whole material data ------------------
    /// material name
    string name;
    /// MTL illum mode
    int illum = 0;

    // color information --------------------
    /// emission color
    vec3f ke = {0, 0, 0};
    /// ambient color
    vec3f ka = {0, 0, 0};
    /// diffuse color
    vec3f kd = {0, 0, 0};
    /// specular color
    vec3f ks = {0, 0, 0};
    /// reflection color
    vec3f kr = {0, 0, 0};
    /// transmision color
    vec3f kt = {0, 0, 0};
    /// phong exponent for ks
    float ns = 1;
    /// index of refraction
    float ior = 1;
    /// opacity
    float op = 1;

    // texture names for the above properties
    /// emission texture
    obj_texture_info ke_txt;
    /// ambient texture
    obj_texture_info ka_txt;
    /// diffuse texture
    obj_texture_info kd_txt;
    /// specular texture
    obj_texture_info ks_txt;
    /// reflection texture
    obj_texture_info kr_txt;
    /// transmission texture
    obj_texture_info kt_txt;
    /// specular exponent texture
    obj_texture_info ns_txt;
    /// opacity texture
    obj_texture_info op_txt;
    /// index of refraction
    obj_texture_info ior_txt;
    /// bump map texture (heighfield)
    obj_texture_info bump_txt;
    /// displacement map texture (heighfield)
    obj_texture_info disp_txt;
    /// normal map texture
    obj_texture_info norm_txt;

    // unknown properties ---------------------
    /// unknown string props
    unordered_map<string, vector<string>> unknown_props;
};

/// Camera [extension]
struct obj_camera {
    /// camera name
    string name;
    /// transform frame (affine matrix)
    frame3f frame = identity_frame3f;
    /// orthografic camera
    bool ortho = false;
    /// vertical field of view
    float yfov = 2 * atan(0.5f);
    /// aspect ratio
    float aspect = 16.0f / 9.0f;
    /// lens aperture
    float aperture = 0;
    /// focus distance
    float focus = 1;
};

/// Environment [extension]
struct obj_environment {
    /// environment name
    string name;
    /// transform frame (affine matrix)
    frame3f frame = identity_frame3f;
    /// material name
    string matname;
};

/// Instance [extension]
struct obj_instance {
    /// instance name
    string name;
    /// transform frame (affine matrix)
    frame3f frame = identity_frame3f;
    /// object name
    string objname;
};

/// OBJ asset
struct obj_scene {
    // vertex data -------------------------
    /// vertex positions
    vector<vec3f> pos;
    /// vertex normals
    vector<vec3f> norm;
    /// vertex texcoord
    vector<vec2f> texcoord;
    /// vertex color [extension]
    vector<vec4f> color;
    /// vertex radius [extension]
    vector<float> radius;

    // scene objects -----------------------
    /// objects
    vector<obj_object*> objects;
    /// materials
    vector<obj_material*> materials;
    /// textures
    vector<obj_texture*> textures;
    /// cameras [extension]
    vector<obj_camera*> cameras;
    /// env maps [extension]
    vector<obj_environment*> environments;
    /// instances [extension]
    vector<obj_instance*> instances;

    /// cleanup
    ~obj_scene() {
        for (auto v : objects)
            if (v) delete v;
        for (auto v : materials)
            if (v) delete v;
        for (auto v : textures)
            if (v) delete v;
        for (auto v : cameras)
            if (v) delete v;
        for (auto v : environments)
            if (v) delete v;
        for (auto v : instances)
            if (v) delete v;
    }
};

/// Load OBJ
///
/// - Parameters:
///     - filename: filename
///     - load_texture: whether to load textures
///     - skip_missing: whether to skip missing files
///     - flip_texcoord: whether to flip the v coordinate
///     - flip_tr: whether to flip the Tr value
/// - Return:
///     - obj (nullptr on error)
inline obj_scene* load_obj(const string& filename, bool load_textures = false,
    bool skip_missing = false, bool flip_texcoord = true, bool flip_tr = true);

/// Save OBJ
///
/// - Parameters:
///     - filename: filename
///     - model: obj data to save
///     - save_textures: whether to save textures
///     - skip_missing: whether to skip missing files
///     - flip_texcoord: whether to flip the v coordinate
///     - flip_tr: whether to flip the Tr value
/// - Returns:
///     - whether an error occurred
inline void save_obj(const string& filename, const obj_scene* model,
    bool save_textures = false, bool skip_missing = false,
    bool flip_texcoord = true, bool flip_tr = true);

/// Shape. May contain only one of the points/lines/triangles.
struct obj_shape {
    /// name of the group that enclosed it
    string name = "";
    /// name of the material
    string matname = "";

    // shape elements -------------------------
    /// points
    vector<int> points;
    /// lines
    vector<vec2i> lines;
    /// triangles
    vector<vec3i> triangles;
    /// tetrahedrons
    vector<vec4i> tetras;

    // vertex data ----------------------------
    /// per-vertex position (3 float)
    vector<vec3f> pos;
    /// per-vertex normals (3 float)
    vector<vec3f> norm;
    /// per-vertex texcoord (2 float)
    vector<vec2f> texcoord;
    /// [extension] per-vertex color (4 float)
    vector<vec4f> color;
    /// [extension] per-vertex radius (1 float)
    vector<float> radius;
};

/// Mesh
struct obj_mesh {
    // name
    string name;
    /// primitives
    vector<obj_shape> shapes;

    /// cleanup
    ~obj_mesh();
};

/// Gets a mesh from an OBJ object.
inline obj_mesh* get_mesh(
    const obj_scene* model, const obj_object& oobj, bool facet_non_smooth);

#endif

}  // namespace ygl

// -----------------------------------------------------------------------------
// KHRONOS GLTF SUPPORT
// -----------------------------------------------------------------------------
namespace ygl {

#if YGL_SCENEIO

/// Json alias
using json = nlohmann::json;

/// Generic buffer data.
using buffer_data = vector<unsigned char>;

/// Generic image data.
struct image_data {
    /// Width
    int width = 0;
    /// Height
    int height = 0;
    /// Number of Channels
    int ncomp = 0;
    /// Buffer data for 8-bit images
    vector<uint8_t> datab;
    /// Buffer data for float images
    vector<float> dataf;
};

/// glTFid
template <typename T>
struct glTFid {
    /// defaoult constructor to an invalid id
    constexpr glTFid() : _id(-1) {}
    /// explicit conversion from integer
    constexpr explicit glTFid(int id) : _id(id) {}
    /// explicit convcersion to integer
    constexpr explicit operator int() const { return _id; }

    /// check if it is valid
    bool is_valid() const { return _id >= 0; }
    /// check if it is valid
    explicit operator bool() const { return _id >= 0; }

   private:
    // id
    int _id = -1;
};

// #codegen begin type

// forward declaration
struct glTFProperty;
struct glTFChildOfRootProperty;
struct glTFAccessorSparseIndices;
struct glTFAccessorSparseValues;
struct glTFAccessorSparse;
struct glTFAccessor;
struct glTFAnimationChannelTarget;
struct glTFAnimationChannel;
struct glTFAnimationSampler;
struct glTFAnimation;
struct glTFAsset;
struct glTFBuffer;
struct glTFBufferView;
struct glTFCameraOrthographic;
struct glTFCameraPerspective;
struct glTFCamera;
struct glTFImage;
struct glTFTextureInfo;
struct glTFTexture;
struct glTFMaterialNormalTextureInfo;
struct glTFMaterialOcclusionTextureInfo;
struct glTFMaterialPbrMetallicRoughness;
struct glTFMaterialPbrSpecularGlossiness;
struct glTFMaterial;
struct glTFMeshPrimitive;
struct glTFMesh;
struct glTFNode;
struct glTFSampler;
struct glTFScene;
struct glTFSkin;
struct glTF;

/// Generic glTF object
struct glTFProperty {
    /// Extensions.
    map<string, json> extensions = {};
    /// Extra data.
    json extras = {};
};

/// Generic glTF named object
struct glTFChildOfRootProperty : glTFProperty {
    /// The user-defined name of this object.
    string name = "";
};

/// Values for glTFAccessorSparseIndices::componentType
enum class glTFAccessorSparseIndicesComponentType {
    /// Not set
    NotSet = -1,
    // UnsignedByte
    UnsignedByte = 5121,
    // UnsignedShort
    UnsignedShort = 5123,
    // UnsignedInt
    UnsignedInt = 5125,
};

/// Indices of those attributes that deviate from their initialization value.
struct glTFAccessorSparseIndices : glTFProperty {
    /// The index of the bufferView with sparse indices. Referenced bufferView
    /// can't have ARRAY_BUFFER or ELEMENT_ARRAY_BUFFER target. [required]
    glTFid<glTFBufferView> bufferView = {};
    /// The offset relative to the start of the bufferView in bytes. Must be
    /// aligned.
    int byteOffset = 0;
    /// The indices data type. [required]
    glTFAccessorSparseIndicesComponentType componentType =
        glTFAccessorSparseIndicesComponentType::NotSet;
};

/// Array of size `accessor.sparse.count` times number of components storing the
/// displaced accessor attributes pointed by `accessor.sparse.indices`.
struct glTFAccessorSparseValues : glTFProperty {
    /// The index of the bufferView with sparse values. Referenced bufferView
    /// can't have ARRAY_BUFFER or ELEMENT_ARRAY_BUFFER target. [required]
    glTFid<glTFBufferView> bufferView = {};
    /// The offset relative to the start of the bufferView in bytes. Must be
    /// aligned.
    int byteOffset = 0;
};

/// Sparse storage of attributes that deviate from their initialization value.
struct glTFAccessorSparse : glTFProperty {
    /// Number of entries stored in the sparse array. [required]
    int count = 0;
    /// Index array of size `count` that points to those accessor attributes
    /// that deviate from their initialization value. Indices must strictly
    /// increase. [required]
    glTFAccessorSparseIndices* indices = nullptr;
    /// Array of size `count` times number of components, storing the displaced
    /// accessor attributes pointed by `indices`. Substituted values must have
    /// the same `componentType` and number of components as the base accessor.
    /// [required]
    glTFAccessorSparseValues* values = nullptr;

    /// Cleanup
    ~glTFAccessorSparse() {
        if (indices) delete indices;
        if (values) delete values;
    }
};

/// Values for glTFAccessor::componentType
enum class glTFAccessorComponentType {
    /// Not set
    NotSet = -1,
    // Byte
    Byte = 5120,
    // UnsignedByte
    UnsignedByte = 5121,
    // Short
    Short = 5122,
    // UnsignedShort
    UnsignedShort = 5123,
    // UnsignedInt
    UnsignedInt = 5125,
    // Float
    Float = 5126,
};

/// Values for glTFAccessor::type
enum class glTFAccessorType {
    /// Not set
    NotSet = -1,
    // Scalar
    Scalar = 0,
    // Vec2
    Vec2 = 1,
    // Vec3
    Vec3 = 2,
    // Vec4
    Vec4 = 3,
    // Mat2
    Mat2 = 4,
    // Mat3
    Mat3 = 5,
    // Mat4
    Mat4 = 6,
};

/// A typed view into a bufferView.  A bufferView contains raw binary data.  An
/// accessor provides a typed view into a bufferView or a subset of a bufferView
/// similar to how WebGL's `vertexAttribPointer()` defines an attribute in a
/// buffer.
struct glTFAccessor : glTFChildOfRootProperty {
    /// The index of the bufferView.
    glTFid<glTFBufferView> bufferView = {};
    /// The offset relative to the start of the bufferView in bytes.
    int byteOffset = 0;
    /// The datatype of components in the attribute. [required]
    glTFAccessorComponentType componentType = glTFAccessorComponentType::NotSet;
    /// Specifies whether integer data values should be normalized.
    bool normalized = false;
    /// The number of attributes referenced by this accessor. [required]
    int count = 0;
    /// Specifies if the attribute is a scalar, vector, or matrix. [required]
    glTFAccessorType type = glTFAccessorType::NotSet;
    /// Maximum value of each component in this attribute.
    vector<float> max = {};
    /// Minimum value of each component in this attribute.
    vector<float> min = {};
    /// Sparse storage of attributes that deviate from their initialization
    /// value.
    glTFAccessorSparse* sparse = nullptr;

    /// Cleanup
    ~glTFAccessor() {
        if (sparse) delete sparse;
    }
};

/// Values for glTFAnimationChannelTarget::path
enum class glTFAnimationChannelTargetPath {
    /// Not set
    NotSet = -1,
    // Translation
    Translation = 0,
    // Rotation
    Rotation = 1,
    // Scale
    Scale = 2,
    // Weights
    Weights = 3,
};

/// The index of the node and TRS property that an animation channel targets.
struct glTFAnimationChannelTarget : glTFProperty {
    /// The index of the node to target. [required]
    glTFid<glTFNode> node = {};
    /// The name of the node's TRS property to modify, or the "weights" of the
    /// Morph Targets it instantiates. [required]
    glTFAnimationChannelTargetPath path =
        glTFAnimationChannelTargetPath::NotSet;
};

/// Targets an animation's sampler at a node's property.
struct glTFAnimationChannel : glTFProperty {
    /// The index of a sampler in this animation used to compute the value for
    /// the target. [required]
    glTFid<glTFAnimationSampler> sampler = {};
    /// The index of the node and TRS property to target. [required]
    glTFAnimationChannelTarget* target = nullptr;

    /// Cleanup
    ~glTFAnimationChannel() {
        if (target) delete target;
    }
};

/// Values for glTFAnimationSampler::interpolation
enum class glTFAnimationSamplerInterpolation {
    /// Not set
    NotSet = -1,
    // The animated values are linearly interpolated between keyframes. When
    // targeting a rotation, spherical linear interpolation (slerp) should be
    // used to interpolate quaternions. The number output of elements must equal
    // the number of input elements.
    Linear = 0,
    // The animated values remain constant to the output of the first keyframe,
    // until the next keyframe. The number of output elements must equal the
    // number of input elements.
    Step = 1,
    // The animation's interpolation is computed using a uniform Catmull-Rom
    // spline. The number of output elements must equal two more than the number
    // of input elements. The first and last output elements represent the start
    // and end tangents of the spline. There must be at least four keyframes
    // when using this interpolation.
    CatmullRomSpline = 2,
    // The animation's interpolation is computed using a cubic spline with
    // specified tangents. The number of output elements must equal three times
    // the number of input elements. For each input element, the output stores
    // three elements, an in-tangent, a spline vertex, and an out-tangent. There
    // must be at least two keyframes when using this interpolation.
    CubicSpline = 3,
};

/// Combines input and output accessors with an interpolation algorithm to
/// define a keyframe graph (but not its target).
struct glTFAnimationSampler : glTFProperty {
    /// The index of an accessor containing keyframe input values, e.g., time.
    /// [required]
    glTFid<glTFAccessor> input = {};
    /// Interpolation algorithm.
    glTFAnimationSamplerInterpolation interpolation =
        glTFAnimationSamplerInterpolation::Linear;
    /// The index of an accessor, containing keyframe output values. [required]
    glTFid<glTFAccessor> output = {};
};

/// A keyframe animation.
struct glTFAnimation : glTFChildOfRootProperty {
    /// An array of channels, each of which targets an animation's sampler at a
    /// node's property. Different channels of the same animation can't have
    /// equal targets. [required]
    vector<glTFAnimationChannel*> channels = {};
    /// An array of samplers that combines input and output accessors with an
    /// interpolation algorithm to define a keyframe graph (but not its target).
    /// [required]
    vector<glTFAnimationSampler*> samplers = {};

    /// typed access for nodes
    glTFAnimationChannel* get(const glTFid<glTFAnimationChannel>& id) const {
        if (!id) return nullptr;
        return channels.at((int)id);
    }
    /// typed access for nodes
    glTFAnimationSampler* get(const glTFid<glTFAnimationSampler>& id) const {
        if (!id) return nullptr;
        return samplers.at((int)id);
    }
    /// Cleanup
    ~glTFAnimation() {
        for (auto v : channels)
            if (v) delete v;
        for (auto v : samplers)
            if (v) delete v;
    }
};

/// Metadata about the glTF asset.
struct glTFAsset : glTFProperty {
    /// A copyright message suitable for display to credit the content creator.
    string copyright = "";
    /// Tool that generated this glTF model.  Useful for debugging.
    string generator = "";
    /// The glTF version that this asset targets. [required]
    string version = "";
    /// The minimum glTF version that this asset targets.
    string minVersion = "";
};

/// A buffer points to binary geometry, animation, or skins.
struct glTFBuffer : glTFChildOfRootProperty {
    /// The uri of the buffer.
    string uri = "";
    /// The length of the buffer in bytes. [required]
    int byteLength = 0;
    /// Stores buffer content after loading. [required]
    buffer_data data = {};
};

/// Values for glTFBufferView::target
enum class glTFBufferViewTarget {
    /// Not set
    NotSet = -1,
    // ArrayBuffer
    ArrayBuffer = 34962,
    // ElementArrayBuffer
    ElementArrayBuffer = 34963,
};

/// A view into a buffer generally representing a subset of the buffer.
struct glTFBufferView : glTFChildOfRootProperty {
    /// The index of the buffer. [required]
    glTFid<glTFBuffer> buffer = {};
    /// The offset into the buffer in bytes.
    int byteOffset = 0;
    /// The length of the bufferView in bytes. [required]
    int byteLength = 0;
    /// The stride, in bytes.
    int byteStride = 0;
    /// The target that the GPU buffer should be bound to.
    glTFBufferViewTarget target = glTFBufferViewTarget::NotSet;
};

/// An orthographic camera containing properties to create an orthographic
/// projection matrix.
struct glTFCameraOrthographic : glTFProperty {
    /// The floating-point horizontal magnification of the view. [required]
    float xmag = 0;
    /// The floating-point vertical magnification of the view. [required]
    float ymag = 0;
    /// The floating-point distance to the far clipping plane. `zfar` must be
    /// greater than `znear`. [required]
    float zfar = 0;
    /// The floating-point distance to the near clipping plane. [required]
    float znear = 0;
};

/// A perspective camera containing properties to create a perspective
/// projection matrix.
struct glTFCameraPerspective : glTFProperty {
    /// The floating-point aspect ratio of the field of view.
    float aspectRatio = 0;
    /// The floating-point vertical field of view in radians. [required]
    float yfov = 0;
    /// The floating-point distance to the far clipping plane.
    float zfar = 0;
    /// The floating-point distance to the near clipping plane. [required]
    float znear = 0;
};

/// Values for glTFCamera::type
enum class glTFCameraType {
    /// Not set
    NotSet = -1,
    // Perspective
    Perspective = 0,
    // Orthographic
    Orthographic = 1,
};

/// A camera's projection.  A node can reference a camera to apply a transform
/// to place the camera in the scene.
struct glTFCamera : glTFChildOfRootProperty {
    /// An orthographic camera containing properties to create an orthographic
    /// projection matrix.
    glTFCameraOrthographic* orthographic = nullptr;
    /// A perspective camera containing properties to create a perspective
    /// projection matrix.
    glTFCameraPerspective* perspective = nullptr;
    /// Specifies if the camera uses a perspective or orthographic projection.
    /// [required]
    glTFCameraType type = glTFCameraType::NotSet;

    /// Cleanup
    ~glTFCamera() {
        if (orthographic) delete orthographic;
        if (perspective) delete perspective;
    }
};

/// Values for glTFImage::mimeType
enum class glTFImageMimeType {
    /// Not set
    NotSet = -1,
    // ImageJpeg
    ImageJpeg = 0,
    // ImagePng
    ImagePng = 1,
};

/// Image data used to create a texture. Image can be referenced by URI or
/// `bufferView` index. `mimeType` is required in the latter case.
struct glTFImage : glTFChildOfRootProperty {
    /// The uri of the image.
    string uri = "";
    /// The image's MIME type.
    glTFImageMimeType mimeType = glTFImageMimeType::NotSet;
    /// The index of the bufferView that contains the image. Use this instead of
    /// the image's uri property.
    glTFid<glTFBufferView> bufferView = {};
    /// Stores image content after loading.
    image_data data = {};
};

/// Reference to a texture.
struct glTFTextureInfo : glTFProperty {
    /// The index of the texture. [required]
    glTFid<glTFTexture> index = {};
    /// The set index of texture's TEXCOORD attribute used for texture
    /// coordinate mapping.
    int texCoord = 0;
};

/// A texture and its sampler.
struct glTFTexture : glTFChildOfRootProperty {
    /// The index of the sampler used by this texture. When undefined, a sampler
    /// with repeat wrapping and auto filtering should be used.
    glTFid<glTFSampler> sampler = {};
    /// The index of the image used by this texture.
    glTFid<glTFImage> source = {};
};

/// Normal texture information.
struct glTFMaterialNormalTextureInfo : glTFTextureInfo {
    /// The scalar multiplier applied to each normal vector of the normal
    /// texture.
    float scale = 1;
};

/// Occlusion texture information.
struct glTFMaterialOcclusionTextureInfo : glTFTextureInfo {
    /// A scalar multiplier controlling the amount of occlusion applied.
    float strength = 1;
};

/// A set of parameter values that are used to define the metallic-roughness
/// material model from Physically-Based Rendering (PBR) methodology.
struct glTFMaterialPbrMetallicRoughness : glTFProperty {
    /// The material's base color factor.
    vec4f baseColorFactor = {1, 1, 1, 1};
    /// The base color texture.
    glTFTextureInfo* baseColorTexture = nullptr;
    /// The metalness of the material.
    float metallicFactor = 1;
    /// The roughness of the material.
    float roughnessFactor = 1;
    /// The metallic-roughness texture.
    glTFTextureInfo* metallicRoughnessTexture = nullptr;

    /// Cleanup
    ~glTFMaterialPbrMetallicRoughness() {
        if (baseColorTexture) delete baseColorTexture;
        if (metallicRoughnessTexture) delete metallicRoughnessTexture;
    }
};

/// glTF extension that defines the specular-glossiness material model from
/// Physically-Based Rendering (PBR) methodology.
struct glTFMaterialPbrSpecularGlossiness : glTFProperty {
    /// The reflected diffuse factor of the material.
    vec4f diffuseFactor = {1, 1, 1, 1};
    /// The diffuse texture.
    glTFTextureInfo* diffuseTexture = nullptr;
    /// The specular RGB color of the material.
    vec3f specularFactor = {1, 1, 1};
    /// The glossiness or smoothness of the material.
    float glossinessFactor = 1;
    /// The specular-glossiness texture.
    glTFTextureInfo* specularGlossinessTexture = nullptr;

    /// Cleanup
    ~glTFMaterialPbrSpecularGlossiness() {
        if (diffuseTexture) delete diffuseTexture;
        if (specularGlossinessTexture) delete specularGlossinessTexture;
    }
};

/// Values for glTFMaterial::alphaMode
enum class glTFMaterialAlphaMode {
    /// Not set
    NotSet = -1,
    // The alpha value is ignored and the rendered output is fully opaque.
    Opaque = 0,
    // The rendered output is either fully opaque or fully transparent depending
    // on the alpha value and the specified alpha cutoff value.
    Mask = 1,
    // The alpha value is used to composite the source and destination areas.
    // The rendered output is combined with the background using the normal
    // painting operation (i.e. the Porter and Duff over operator).
    Blend = 2,
};

/// The material appearance of a primitive.
struct glTFMaterial : glTFChildOfRootProperty {
    /// A set of parameter values that are used to define the metallic-roughness
    /// material model from Physically-Based Rendering (PBR) methodology. When
    /// not specified, all the default values of `pbrMetallicRoughness` apply.
    glTFMaterialPbrMetallicRoughness* pbrMetallicRoughness = nullptr;
    /// A set of parameter values that are used to define the
    /// specular-glossiness material model from Physically-Based Rendering (PBR)
    /// methodology. When not specified, all the default values of
    /// `pbrMetallicRoughness` apply.
    glTFMaterialPbrSpecularGlossiness* pbrSpecularGlossiness = nullptr;
    /// The normal map texture.
    glTFMaterialNormalTextureInfo* normalTexture = nullptr;
    /// The occlusion map texture.
    glTFMaterialOcclusionTextureInfo* occlusionTexture = nullptr;
    /// The emissive map texture.
    glTFTextureInfo* emissiveTexture = nullptr;
    /// The emissive color of the material.
    vec3f emissiveFactor = {0, 0, 0};
    /// The alpha rendering mode of the material.
    glTFMaterialAlphaMode alphaMode = glTFMaterialAlphaMode::Opaque;
    /// The alpha cutoff value of the material.
    float alphaCutoff = 0.5;
    /// Specifies whether the material is double sided.
    bool doubleSided = false;

    /// Cleanup
    ~glTFMaterial() {
        if (pbrMetallicRoughness) delete pbrMetallicRoughness;
        if (pbrSpecularGlossiness) delete pbrSpecularGlossiness;
        if (normalTexture) delete normalTexture;
        if (occlusionTexture) delete occlusionTexture;
        if (emissiveTexture) delete emissiveTexture;
    }
};

/// Values for glTFMeshPrimitive::mode
enum class glTFMeshPrimitiveMode {
    /// Not set
    NotSet = -1,
    // Points
    Points = 0,
    // Lines
    Lines = 1,
    // LineLoop
    LineLoop = 2,
    // LineStrip
    LineStrip = 3,
    // Triangles
    Triangles = 4,
    // TriangleStrip
    TriangleStrip = 5,
    // TriangleFan
    TriangleFan = 6,
};

/// Geometry to be rendered with the given material.
struct glTFMeshPrimitive : glTFProperty {
    /// A dictionary object, where each key corresponds to mesh attribute
    /// semantic and each value is the index of the accessor containing
    /// attribute's data. [required]
    map<string, glTFid<glTFAccessor>> attributes = {};
    /// The index of the accessor that contains the indices.
    glTFid<glTFAccessor> indices = {};
    /// The index of the material to apply to this primitive when rendering.
    glTFid<glTFMaterial> material = {};
    /// The type of primitives to render.
    glTFMeshPrimitiveMode mode = glTFMeshPrimitiveMode::Triangles;
    /// An array of Morph Targets, each  Morph Target is a dictionary mapping
    /// attributes (only `POSITION`, `NORMAL`, and `TANGENT` supported) to their
    /// deviations in the Morph Target.
    vector<map<string, glTFid<glTFAccessor>>> targets = {};
};

/// A set of primitives to be rendered.  A node can contain one mesh.  A node's
/// transform places the mesh in the scene.
struct glTFMesh : glTFChildOfRootProperty {
    /// An array of primitives, each defining geometry to be rendered with a
    /// material. [required]
    vector<glTFMeshPrimitive*> primitives = {};
    /// Array of weights to be applied to the Morph Targets.
    vector<float> weights = {};

    /// Cleanup
    ~glTFMesh() {
        for (auto v : primitives)
            if (v) delete v;
    }
};

/// A node in the node hierarchy.  When the node contains `skin`, all
/// `mesh.primitives` must contain `JOINTS_0` and `WEIGHTS_0` attributes.  A
/// node can have either a `matrix` or any combination of
/// `translation`/`rotation`/`scale` (TRS) properties. TRS properties are
/// converted to matrices and postmultiplied in the `T * R * S` order to compose
/// the transformation matrix; first the scale is applied to the vertices, then
/// the rotation, and then the translation. If none are provided, the transform
/// is the identity. When a node is targeted for animation (referenced by an
/// animation.channel.target), only TRS properties may be present; `matrix` will
/// not be present.
struct glTFNode : glTFChildOfRootProperty {
    /// The index of the camera referenced by this node.
    glTFid<glTFCamera> camera = {};
    /// The indices of this node's children.
    vector<glTFid<glTFNode>> children = {};
    /// The index of the skin referenced by this node.
    glTFid<glTFSkin> skin = {};
    /// A floating-point 4x4 transformation matrix stored in column-major order.
    mat4f matrix = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    /// The index of the mesh in this node.
    glTFid<glTFMesh> mesh = {};
    /// The node's unit quaternion rotation in the order (x, y, z, w), where w
    /// is the scalar.
    quat4f rotation = {0, 0, 0, 1};
    /// The node's non-uniform scale.
    vec3f scale = {1, 1, 1};
    /// The node's translation.
    vec3f translation = {0, 0, 0};
    /// The weights of the instantiated Morph Target. Number of elements must
    /// match number of Morph Targets of used mesh.
    vector<float> weights = {};
};

/// Values for glTFSampler::magFilter
enum class glTFSamplerMagFilter {
    /// Not set
    NotSet = -1,
    // Nearest
    Nearest = 9728,
    // Linear
    Linear = 9729,
};

/// Values for glTFSampler::minFilter
enum class glTFSamplerMinFilter {
    /// Not set
    NotSet = -1,
    // Nearest
    Nearest = 9728,
    // Linear
    Linear = 9729,
    // NearestMipmapNearest
    NearestMipmapNearest = 9984,
    // LinearMipmapNearest
    LinearMipmapNearest = 9985,
    // NearestMipmapLinear
    NearestMipmapLinear = 9986,
    // LinearMipmapLinear
    LinearMipmapLinear = 9987,
};

/// glTFSampler::wrapS
enum class glTFSamplerWrapS {
    /// Not set
    NotSet = -1,
    // ClampToEdge
    ClampToEdge = 33071,
    // MirroredRepeat
    MirroredRepeat = 33648,
    // Repeat
    Repeat = 10497,
};

/// glTFSampler::wrapT
enum class glTFSamplerWrapT {
    /// Not set
    NotSet = -1,
    // ClampToEdge
    ClampToEdge = 33071,
    // MirroredRepeat
    MirroredRepeat = 33648,
    // Repeat
    Repeat = 10497,
};

/// Texture sampler properties for filtering and wrapping modes.
struct glTFSampler : glTFChildOfRootProperty {
    /// Magnification filter.
    glTFSamplerMagFilter magFilter = glTFSamplerMagFilter::NotSet;
    /// Minification filter.
    glTFSamplerMinFilter minFilter = glTFSamplerMinFilter::NotSet;
    /// s wrapping mode.
    glTFSamplerWrapS wrapS = glTFSamplerWrapS::Repeat;
    /// t wrapping mode.
    glTFSamplerWrapT wrapT = glTFSamplerWrapT::Repeat;
};

/// The root nodes of a scene.
struct glTFScene : glTFChildOfRootProperty {
    /// The indices of each root node.
    vector<glTFid<glTFNode>> nodes = {};
};

/// Joints and matrices defining a skin.
struct glTFSkin : glTFChildOfRootProperty {
    /// The index of the accessor containing the floating-point 4x4 inverse-bind
    /// matrices.  The default is that each matrix is a 4x4 identity matrix,
    /// which implies that inverse-bind matrices were pre-applied.
    glTFid<glTFAccessor> inverseBindMatrices = {};
    /// The index of the node used as a skeleton root. When undefined, joints
    /// transforms resolve to scene root.
    glTFid<glTFNode> skeleton = {};
    /// Indices of skeleton nodes, used as joints in this skin. [required]
    vector<glTFid<glTFNode>> joints = {};
};

/// The root object for a glTF asset.
struct glTF : glTFProperty {
    /// Names of glTF extensions used somewhere in this asset.
    vector<string> extensionsUsed = {};
    /// Names of glTF extensions required to properly load this asset.
    vector<string> extensionsRequired = {};
    /// An array of accessors.
    vector<glTFAccessor*> accessors = {};
    /// An array of keyframe animations.
    vector<glTFAnimation*> animations = {};
    /// Metadata about the glTF asset. [required]
    glTFAsset* asset = nullptr;
    /// An array of buffers.
    vector<glTFBuffer*> buffers = {};
    /// An array of bufferViews.
    vector<glTFBufferView*> bufferViews = {};
    /// An array of cameras.
    vector<glTFCamera*> cameras = {};
    /// An array of images.
    vector<glTFImage*> images = {};
    /// An array of materials.
    vector<glTFMaterial*> materials = {};
    /// An array of meshes.
    vector<glTFMesh*> meshes = {};
    /// An array of nodes.
    vector<glTFNode*> nodes = {};
    /// An array of samplers.
    vector<glTFSampler*> samplers = {};
    /// The index of the default scene.
    glTFid<glTFScene> scene = {};
    /// An array of scenes.
    vector<glTFScene*> scenes = {};
    /// An array of skins.
    vector<glTFSkin*> skins = {};
    /// An array of textures.
    vector<glTFTexture*> textures = {};

    /// typed access for nodes
    glTFAccessor* get(const glTFid<glTFAccessor>& id) const {
        if (!id) return nullptr;
        return accessors.at((int)id);
    }
    /// typed access for nodes
    glTFAnimation* get(const glTFid<glTFAnimation>& id) const {
        if (!id) return nullptr;
        return animations.at((int)id);
    }
    /// typed access for nodes
    glTFBuffer* get(const glTFid<glTFBuffer>& id) const {
        if (!id) return nullptr;
        return buffers.at((int)id);
    }
    /// typed access for nodes
    glTFBufferView* get(const glTFid<glTFBufferView>& id) const {
        if (!id) return nullptr;
        return bufferViews.at((int)id);
    }
    /// typed access for nodes
    glTFCamera* get(const glTFid<glTFCamera>& id) const {
        if (!id) return nullptr;
        return cameras.at((int)id);
    }
    /// typed access for nodes
    glTFImage* get(const glTFid<glTFImage>& id) const {
        if (!id) return nullptr;
        return images.at((int)id);
    }
    /// typed access for nodes
    glTFMaterial* get(const glTFid<glTFMaterial>& id) const {
        if (!id) return nullptr;
        return materials.at((int)id);
    }
    /// typed access for nodes
    glTFMesh* get(const glTFid<glTFMesh>& id) const {
        if (!id) return nullptr;
        return meshes.at((int)id);
    }
    /// typed access for nodes
    glTFNode* get(const glTFid<glTFNode>& id) const {
        if (!id) return nullptr;
        return nodes.at((int)id);
    }
    /// typed access for nodes
    glTFSampler* get(const glTFid<glTFSampler>& id) const {
        if (!id) return nullptr;
        return samplers.at((int)id);
    }
    /// typed access for nodes
    glTFScene* get(const glTFid<glTFScene>& id) const {
        if (!id) return nullptr;
        return scenes.at((int)id);
    }
    /// typed access for nodes
    glTFSkin* get(const glTFid<glTFSkin>& id) const {
        if (!id) return nullptr;
        return skins.at((int)id);
    }
    /// typed access for nodes
    glTFTexture* get(const glTFid<glTFTexture>& id) const {
        if (!id) return nullptr;
        return textures.at((int)id);
    }
    /// Cleanup
    ~glTF() {
        for (auto v : accessors)
            if (v) delete v;
        for (auto v : animations)
            if (v) delete v;
        if (asset) delete asset;
        for (auto v : buffers)
            if (v) delete v;
        for (auto v : bufferViews)
            if (v) delete v;
        for (auto v : cameras)
            if (v) delete v;
        for (auto v : images)
            if (v) delete v;
        for (auto v : materials)
            if (v) delete v;
        for (auto v : meshes)
            if (v) delete v;
        for (auto v : nodes)
            if (v) delete v;
        for (auto v : samplers)
            if (v) delete v;
        for (auto v : scenes)
            if (v) delete v;
        for (auto v : skins)
            if (v) delete v;
        for (auto v : textures)
            if (v) delete v;
    }
};
// #codegen end type
// -----------------------------------------------------------

/// Loads a gltf file from disk
///
/// - Parameters:
///     - filename: scene filename
///     - load_bin/load_img: load binary data
///     - skip_missing: do not throw an exception if a file is missing
/// - Returns:
///     - gltf data loaded (nullptr on error)
inline glTF* load_gltf(const string& filename, bool load_bin = true,
    bool load_img = false, bool skip_missing = false);

/// Loads a binary gltf file from disk
///
/// - Parameters:
///     - filename: scene filename
///     - other params as above
/// - Returns:
///     - gltf data loaded (nullptr on error)
inline glTF* load_binary_gltf(const string& filename, bool load_bin = true,
    bool load_img = false, bool skip_missing = false);

/// Saves a scene to disk
///
/// - Parameters:
///     - filename: scene filename
///     - gltf: data to save
///     - save_bin/save_images: save binary data
inline void save_gltf(const string& filename, const glTF* gltf,
    bool save_bin = true, bool save_images = false);

/// Saves a scene to disk
///
/// - Parameters:
///     - filename: scene filename
///     - gltf: data to save
///     - save_bin/save_images: save binary data
inline void save_binary_gltf(const string& filename, const glTF* gltf,
    bool save_bin = true, bool save_images = false);

/// Computes the local node transform and its inverse.
inline mat4f node_transform(const glTFNode* node);

/// A view for gltf array buffers that allows for typed access.
struct accessor_view {
    /// construct a view from an accessor
    accessor_view(const glTF* gltf, const glTFAccessor* accessor);

    /// number of elements in the view
    int size() const { return _size; }
    /// number of elements in the view
    int count() const { return _size; }
    /// number of components per element
    int ncomp() const { return _ncomp; }
    /// check whether the view is valid
    bool valid() const { return _valid; }

    /// get the idx-th element of fixed length width default values
    template <int N>
    vec<float, N> getv(int idx, const vec<float, N>& def) const {
        auto v = def;
        for (auto i = 0; i < min(_ncomp, N); i++) v[i] = get(idx, i);
        return v;
    }
    /// get the idx-th element of fixed length
    template <int N>
    vec<float, N> getv(int idx) const {
        return getv(idx, vec<float, N>());
    }

    /// get the idx-th element of fixed length as a matrix
    template <int N, int M>
    mat<float, N, M> getm(int idx) const {
        auto v = mat<float, N, M>();
        assert(_ncomp == N * M);
        for (auto j = 0; j < M; j++)
            for (auto i = 0; i < N; i++) v[j][i] = get(idx, j * N + i);
        return v;
    }

    /// get the c-th component of the idx-th element
    float get(int idx, int c = 0) const;

    /// get the idx-th element as integer with fixed length
    template <int N>
    vec<int, N> getiv(int idx, const vec<int, N>& def) const {
        auto v = def;
        for (auto i = 0; i < min(_ncomp, N); i++) { v[i] = geti(idx, i); }
        return v;
    }
    /// get the idx-th element as integer
    template <int N>
    vec<int, N> getiv(int idx) const {
        return getiv(idx, vec<int, N>());
    }

    /// get the c-th component of the idx-th element as integer
    int geti(int idx, int c = 0) const;

   private:
    const unsigned char* _data = nullptr;
    int _size = 0;
    int _stride = 0;
    int _ncomp = 0;
    glTFAccessorComponentType _ctype;
    bool _normalize = false;
    bool _valid = false;

    static int _num_components(glTFAccessorType type);
    static int _ctype_size(glTFAccessorComponentType componentType);
};

#endif

}  // namespace ygl

// -----------------------------------------------------------------------------
// OPENGL FUNCTIONS
// -----------------------------------------------------------------------------
namespace ygl {

#if YGL_OPENGL

/// Shape types
enum struct gl_etype : int {
    /// points
    point = 1,
    /// lines
    line = 2,
    /// triangles
    triangle = 3,
    /// quads
    quad = 4,
};

/// Light types
enum struct gl_ltype : int {
    /// point lights
    point = 0,
    /// directional lights
    directional = 1,
};

/// Checks for GL error and then prints
inline bool gl_check_error(bool print = true) {
    auto ok = glGetError();
    if (ok == GL_NO_ERROR) return true;
    if (!print) return false;
    switch (ok) {
        case GL_NO_ERROR: printf("GL_NO_ERROR\n"); break;
        case GL_INVALID_ENUM: printf("GL_INVALID_ENUM\n"); break;
        case GL_INVALID_VALUE: printf("GL_INVALID_VALUE\n"); break;
        case GL_INVALID_OPERATION: printf("GL_INVALID_OPERATION\n"); break;
        case GL_INVALID_FRAMEBUFFER_OPERATION:
            printf("GL_INVALID_FRAMEBUFFER_OPERATION\n");
            break;
        case GL_OUT_OF_MEMORY: printf("GL_OUT_OF_MEMORY\n"); break;
        default: printf("<UNKNOWN GL ERROR>\n"); break;
    }
    return false;
}

/// Clear window
inline void gl_clear_buffers(const vec4f& background = {0, 0, 0, 0}) {
    assert(gl_check_error());
    glClearColor(background[0], background[1], background[2], background[3]);
    glClearDepth(1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    assert(gl_check_error());
}

/// Enable/disable depth test
inline void gl_enable_depth_test(bool enabled) {
    assert(gl_check_error());
    if (enabled)
        glEnable(GL_DEPTH_TEST);
    else
        glDisable(GL_DEPTH_TEST);
    assert(gl_check_error());
}

/// Enable/disable culling
inline void gl_enable_culling(bool enabled) {
    assert(gl_check_error());
    if (enabled)
        glEnable(GL_CULL_FACE);
    else
        glDisable(GL_CULL_FACE);
    assert(gl_check_error());
}

/// Enable/disable wireframe
inline void gl_enable_wireframe(bool enabled) {
    assert(gl_check_error());
    if (enabled)
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    else
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    assert(gl_check_error());
}

/// Enable/disable edges. Attempts to avoid z-fighting but the method is not
/// robust.
inline void gl_enable_edges(bool enabled, float tolerance = 0.9999f) {
    assert(gl_check_error());
    if (enabled) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDepthRange(0, tolerance);
    } else {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDepthRange(0, 1);
    }
    assert(gl_check_error());
}

/// Enable/disable blending
inline void gl_enable_blending(bool enabled) {
    assert(gl_check_error());
    if (enabled) {
        glEnable(GL_BLEND);
    } else {
        glDisable(GL_BLEND);
    }
    assert(gl_check_error());
}

/// Set blending to over operator
inline void gl_set_blend_over() {
    assert(gl_check_error());
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    assert(gl_check_error());
}

/// Line width
inline void gl_line_width(float w) {
    assert(gl_check_error());
    glLineWidth(min(max(w, 0.0f), 1.0f));
    assert(gl_check_error());
}

/// Set viewport
inline void gl_set_viewport(const vec4i& v) {
    assert(gl_check_error());
    glViewport(v.x, v.y, v.z, v.w);
    assert(gl_check_error());
}

/// Set viewport
inline void gl_set_viewport(const vec2i& v) {
    assert(gl_check_error());
    glViewport(0, 0, v.x, v.y);
    assert(gl_check_error());
}

// This is a public API. See above for documentation.
inline void gl_read_imagef(float* pixels, int w, int h, int nc) {
    assert(gl_check_error());
    int formats[4] = {GL_RED, GL_RG, GL_RGB, GL_RGBA};
    glReadPixels(0, 0, w, h, formats[nc - 1], GL_FLOAT, pixels);
    assert(gl_check_error());
}

// -----------------------------------------------------------------------------
// TEXTURE FUNCTIONS
// -----------------------------------------------------------------------------

/// Opengl texture object
struct gl_texture {
    // texture handle
    uint _tid = 0;
    // width
    int _width = 0;
    // height
    int _height = 0;
    // ncomp
    int _ncomp = 0;
    // floats
    bool _float = false;
    // srgb
    bool _srgb = true;
    // mipmap
    bool _mipmap = true;
};

// Implementation of make_texture.
inline void _init_texture(gl_texture& txt, int w, int h, int nc,
    const void* pixels, bool floats, bool linear, bool mipmap, bool as_float,
    bool as_srgb) {
    txt._width = w;
    txt._height = h;
    txt._ncomp = nc;
    txt._float = as_float;
    txt._srgb = as_srgb;
    txt._mipmap = mipmap;
    assert(!as_srgb || !as_float);
    assert(gl_check_error());
    int formats_ub[4] = {GL_RED, GL_RG, GL_RGB, GL_RGBA};
    int formats_sub[4] = {GL_RED, GL_RG, GL_SRGB, GL_SRGB_ALPHA};
    int formats_f[4] = {GL_R32F, GL_RG32F, GL_RGB32F, GL_RGBA32F};
    int* formats =
        (as_float) ? formats_f : ((as_srgb) ? formats_sub : formats_ub);
    assert(gl_check_error());
    glGenTextures(1, &txt._tid);
    glBindTexture(GL_TEXTURE_2D, txt._tid);
    glTexImage2D(GL_TEXTURE_2D, 0, formats[nc - 1], w, h, 0, formats_ub[nc - 1],
        (floats) ? GL_FLOAT : GL_UNSIGNED_BYTE, pixels);
    if (mipmap) glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
        (linear) ? GL_LINEAR : GL_NEAREST);
    if (mipmap) {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
            (linear) ? GL_LINEAR_MIPMAP_LINEAR : GL_NEAREST_MIPMAP_NEAREST);
    } else {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
            (linear) ? GL_LINEAR : GL_NEAREST);
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    assert(gl_check_error());
}

// Implementation of update_texture.
inline void _update_texture(
    gl_texture& txt, int w, int h, int nc, const void* pixels, bool floats) {
    txt._width = w;
    txt._height = h;
    assert(gl_check_error());
    int formats[4] = {GL_RED, GL_RG, GL_RGB, GL_RGBA};
    glBindTexture(GL_TEXTURE_2D, txt._tid);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, formats[nc - 1],
        (floats) ? GL_FLOAT : GL_UNSIGNED_BYTE, pixels);
    if (txt._mipmap) glGenerateMipmap(GL_TEXTURE_2D);
    assert(gl_check_error());
}

/// Creates a texture with pixels values of size w, h with nc number of
/// components (1-4).
/// Internally use float if as_float and filtering if filter.
/// Returns the texture id.
inline gl_texture make_texture(int w, int h, int nc, const float* pixels,
    bool linear, bool mipmap, bool as_float) {
    auto txt = gl_texture();
    _init_texture(txt, w, h, nc, pixels, true, linear, mipmap, as_float, false);
    return txt;
}

/// Creates a texture with pixels values of size w, h with nc number of
/// components (1-4).
/// Internally use srgb lookup if as_srgb and filtering if filter.
/// Returns the texture id.
inline gl_texture make_texture(int w, int h, int nc,
    const unsigned char* pixels, bool linear, bool mipmap, bool as_srgb) {
    auto txt = gl_texture();
    _init_texture(txt, w, h, nc, pixels, false, linear, mipmap, false, as_srgb);
    return txt;
}

/// Creates a texture from an image.
/// Internally use float if as_float and filtering if filter.
/// Returns the texture id.
template <int N>
inline gl_texture make_texture(
    const image<vec<float, N>>& img, bool linear, bool mipmap, bool as_float) {
    return make_texture(img.width(), img.height(), N, (float*)img.data(),
        linear, mipmap, as_float);
}

/// Creates a texture from an image.
/// Internally use srgb lookup if as_srgb and filtering if filter.
/// Returns the texture id.
template <int N>
inline gl_texture make_texture(
    const image<vec<byte, N>>& img, bool linear, bool mipmap, bool as_srgb) {
    return make_texture(img.width(), img.height(), N, (byte*)img.data(), linear,
        mipmap, as_srgb);
}

/// Updates the texture tid with new image data.
template <int N>
inline void update_texture(gl_texture& txt, const image<vec<float, N>>& img) {
    update_texture(txt, img.width(), img.height(), N, (float*)img.data());
}

/// Updates the texture tid with new image data.
template <int N>
inline void update_texture(gl_texture& txt, const image<vec<byte, N>>& img) {
    update_texture(img.width(), img.height(), N, (byte*)img.data());
}

/// Updates the texture tid with new image data.
inline void update_texture(
    gl_texture& txt, int w, int h, int nc, const float* pixels) {
    _update_texture(txt, w, h, nc, pixels, true);
}

/// Updates the texture tid with new image data.
inline void update_texture(
    gl_texture& txt, int w, int h, int nc, const unsigned char* pixels) {
    _update_texture(txt, w, h, nc, pixels, false);
}

/// Binds a texture to a texture unit
inline void bind_texture(const gl_texture& txt, uint unit) {
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_2D, txt._tid);
}

/// Unbinds
inline void unbind_texture(const gl_texture& txt, uint unit) {
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_2D, 0);
}

/// Get id
inline uint get_texture_id(const gl_texture& txt) { return txt._tid; }

/// Check if defined
inline bool is_texture_valid(const gl_texture& txt) { return (bool)txt._tid; }

/// Destroys the texture tid.
inline void clear_texture(gl_texture& txt) {
    assert(gl_check_error());
    glDeleteTextures(1, &txt._tid);
    txt._tid = 0;
    assert(gl_check_error());
}

/// Wrap values for texture
enum struct gl_texture_wrap {
    /// not set
    not_set = 0,
    /// repeat
    repeat = 1,
    /// clamp to edge
    clamp = 2,
    /// mirror
    mirror = 3,
};

/// Filter values for texture
enum struct gl_texture_filter {
    /// not set
    not_set = 0,
    /// linear
    linear = 1,
    /// nearest
    nearest = 2,
    /// mip-mapping
    linear_mipmap_linear = 3,
    /// mip-mapping
    nearest_mipmap_nearest = 4,
    /// mip-mapping
    linear_mipmap_nearest = 5,
    /// mip-mapping
    nearest_mipmap_linear = 6,
};

/// Texture information for parameter setting.
struct gl_texture_info {
    /// texture
    gl_texture txt = {};
    /// texture coordinate set
    int texcoord = 0;
    /// texture strength/scale (used by some models)
    float scale = 1;
    /// wrap mode
    gl_texture_wrap wrap_s = gl_texture_wrap::not_set;
    /// wrap mode
    gl_texture_wrap wrap_t = gl_texture_wrap::not_set;
    /// filter mode
    gl_texture_filter filter_mag = gl_texture_filter::not_set;
    /// filter mode
    gl_texture_filter filter_min = gl_texture_filter::not_set;

    /// default constructor
    gl_texture_info() {}
    /// constructor from texture id only
    gl_texture_info(const gl_texture& tid) : txt(tid) {}
};

// -----------------------------------------------------------------------------
// VERTEX ARRAY BUFFER
// -----------------------------------------------------------------------------

/// OpenGL vertex/element buffer
struct gl_vertex_buffer {
    // buffer id
    uint _bid = 0;
    // number of elements
    int _num = 0;
    // number of components
    int _ncomp = 0;
    // whether is is floats
    bool _float = true;
};

// Creates a buffer with num elements of size size stored in values, where
// content is dyanamic if dynamic.
inline void _init_vertex_buffer(gl_vertex_buffer& buf, int n, int nc,
    const void* values, bool as_float, bool dynamic) {
    buf._num = n;
    buf._ncomp = nc;
    buf._float = as_float;
    assert(gl_check_error());
    buf._bid = (GLuint)0;
    glGenBuffers(1, &buf._bid);
    glBindBuffer(GL_ARRAY_BUFFER, buf._bid);
    glBufferData(GL_ARRAY_BUFFER,
        buf._num * buf._ncomp * ((as_float) ? sizeof(float) : sizeof(int)),
        values, (dynamic) ? GL_DYNAMIC_DRAW : GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    assert(gl_check_error());
}

// Updates the buffer bid with new data.
inline void _update_vertex_buffer(
    gl_vertex_buffer& buf, int n, int nc, const void* values, bool as_float) {
    buf._num = n;
    buf._ncomp = nc;
    buf._float = as_float;
    assert(gl_check_error());
    glBindBuffer(GL_ARRAY_BUFFER, buf._bid);
    glBufferSubData(GL_ARRAY_BUFFER, 0,
        buf._num * buf._ncomp * ((as_float) ? sizeof(float) : sizeof(int)),
        values);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    assert(gl_check_error());
}

/// Creates a buffer.
inline gl_vertex_buffer make_vertex_buffer(
    int num, int ncomp, const float* values, bool dynamic = false) {
    auto buf = gl_vertex_buffer();
    _init_vertex_buffer(buf, num, ncomp, values, true, dynamic);
    return buf;
}

/// Creates a buffer.
inline gl_vertex_buffer make_vertex_buffer(
    int num, int ncomp, const int* values, bool dynamic = false) {
    auto buf = gl_vertex_buffer();
    _init_vertex_buffer(buf, num, ncomp, values, true, dynamic);
    return buf;
}

/// Creates a buffer.
inline gl_vertex_buffer make_vertex_buffer(
    const vector<float>& values, bool dynamic = false) {
    return make_vertex_buffer(values.size(), 1, values.data(), dynamic);
}

/// Creates a buffer.
template <int N>
inline gl_vertex_buffer make_vertex_buffer(
    const vector<vec<float, N>>& values, bool dynamic = false) {
    return make_vertex_buffer(
        values.size(), N, (const float*)values.data(), dynamic);
}

/// Creates a buffer.
inline gl_vertex_buffer make_vertex_buffer(
    const vector<int>& values, bool dynamic = false) {
    return make_vertex_buffer(values.size(), 1, values.data(), dynamic);
}

/// Creates a buffer.
template <int N>
inline gl_vertex_buffer make_vertex_buffer(
    const vector<vec<int, N>>& values, bool dynamic = false) {
    return make_vertex_buffer(
        values.size(), N, (const int*)values.data(), dynamic);
}

/// Updates the buffer with new data.
inline void update_vertex_buffer(
    gl_vertex_buffer& buf, int num, int ncomp, const float* values) {
    _update_vertex_buffer(buf, num, ncomp, values, true);
}

/// Updates the buffer with new data.
inline void update_vertex_buffer(
    gl_vertex_buffer& buf, int num, int ncomp, const int* values) {
    _update_vertex_buffer(buf, num, ncomp, values, false);
}

/// Updates the buffer bid with new data.
inline void update_vertex_buffer(
    gl_vertex_buffer& buf, const vector<float>& values) {
    update_vertex_buffer(buf, values.size(), 1, values.data());
}

/// Updates the buffer bid with new data.
template <int N>
inline void update_vertex_buffer(
    gl_vertex_buffer& buf, const vector<vec<float, N>>& values) {
    update_vertex_buffer(buf, values.size(), N, (const float*)values.data());
}

/// Updates the buffer bid with new data.
inline void update_vertex_buffer(
    gl_vertex_buffer& buf, const vector<int>& values) {
    update_vertex_buffer(buf, values.size(), 1, values.data());
}

/// Updates the buffer bid with new data.
template <int N>
inline void update_vertex_buffer(
    gl_vertex_buffer& buf, const vector<vec<int, N>>& values) {
    update_vertex_buffer(buf, values.size(), N, (const int*)values.data());
}

/// Bind the buffer at a particular attribute location
inline void bind_vertex_buffer(const gl_vertex_buffer& buf, uint vattr) {
    glEnableVertexAttribArray(vattr);
    glBindBuffer(GL_ARRAY_BUFFER, buf._bid);
    glVertexAttribPointer(vattr, buf._ncomp, GL_FLOAT, false, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

/// Unbind the buffer
inline void unbind_vertex_buffer(const gl_vertex_buffer& buf, uint vattr) {
    glDisableVertexAttribArray(vattr);
}

/// Get id
inline uint get_vertex_buffer_id(const gl_vertex_buffer& buf) {
    return buf._bid;
}

/// Check if defined
inline bool is_vertex_buffer_valid(const gl_vertex_buffer& buf) {
    return (bool)buf._bid;
}

/// Destroys the buffer
inline void clear_vertex_buffer(gl_vertex_buffer& buf) {
    assert(gl_check_error());
    glDeleteBuffers(1, &buf._bid);
    buf._bid = 0;
    assert(gl_check_error());
}

// -----------------------------------------------------------------------------
// VERTEX ELEMENTS BUFFER
// -----------------------------------------------------------------------------

/// OpenGL vertex/element buffer
struct gl_element_buffer {
    // buffer id
    uint _bid = 0;
    // number of elements
    int _num = 0;
    // number of components
    int _ncomp = 0;
};

// Creates a buffer with num elements of size size stored in values, where
// content is dyanamic if dynamic.
// Returns the buffer id.
inline void _init_element_buffer(
    gl_element_buffer& buf, int n, int nc, const int* values, bool dynamic) {
    buf._num = n;
    buf._ncomp = nc;
    assert(gl_check_error());
    buf._bid = (GLuint)0;
    glGenBuffers(1, &buf._bid);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buf._bid);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, buf._num * buf._ncomp * sizeof(int),
        values, (dynamic) ? GL_DYNAMIC_DRAW : GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    assert(gl_check_error());
}

// Updates the buffer bid with new data.
inline void _update_element_buffer(
    gl_element_buffer& buf, int n, int nc, const int* values) {
    buf._num = n;
    buf._ncomp = nc;
    assert(gl_check_error());
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buf._bid);
    glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0,
        buf._num * buf._ncomp * sizeof(int), values);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    assert(gl_check_error());
}

/// Creates a buffer.
inline gl_element_buffer make_element_buffer(
    int num, int ncomp, const int* values, bool dynamic = false) {
    auto buf = gl_element_buffer();
    _init_element_buffer(buf, num, ncomp, values, dynamic);
    return buf;
}

/// Creates a buffer.
inline gl_element_buffer make_element_buffer(
    const vector<int>& values, bool dynamic = false) {
    return make_element_buffer(values.size(), 1, values.data(), dynamic);
}

/// Creates a buffer.
template <int N>
inline gl_element_buffer make_element_buffer(
    const vector<vec<int, N>>& values, bool dynamic = false) {
    return make_element_buffer(
        values.size(), N, (const int*)values.data(), dynamic);
}

/// Updates the buffer with new data.
inline void update_element_buffer(
    gl_element_buffer& buf, int num, int ncomp, const int* values) {
    _update_element_buffer(buf, num, ncomp, values);
}

/// Updates the buffer bid with new data.
inline void update_element_buffer(
    gl_element_buffer& buf, const vector<int>& values) {
    update_element_buffer(buf, values.size(), 1, values.data());
}

/// Updates the buffer bid with new data.
template <int N>
inline void update_element_buffer(
    gl_element_buffer& buf, const vector<vec<int, N>>& values) {
    update_element_buffer(values.size(), N, (const int*)values.data());
}

/// Draws elements.
inline void draw_elems(const gl_element_buffer& buf) {
    if (!buf._bid) return;
    assert(gl_check_error());
    int mode = 0;
    switch (buf._ncomp) {
        case 1: mode = GL_POINTS; break;
        case 2: mode = GL_LINES; break;
        case 3: mode = GL_TRIANGLES; break;
        case 4: mode = GL_QUADS; break;
        default: assert(false);
    };
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buf._bid);
    glDrawElements(mode, buf._ncomp * buf._num, GL_UNSIGNED_INT, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    assert(gl_check_error());
}

/// Get id
inline uint get_element_buffer_id(const gl_element_buffer& buf) {
    return buf._bid;
}

/// Check if defined
inline bool is_element_buffer_valid(const gl_element_buffer& buf) {
    return (bool)buf._bid;
}

/// Destroys the buffer
inline void clear_element_buffer(gl_element_buffer& buf) {
    assert(gl_check_error());
    glDeleteBuffers(1, &buf._bid);
    buf._bid = 0;
    assert(gl_check_error());
}

// -----------------------------------------------------------------------------
// PROGRAM FUNCTIONS
// -----------------------------------------------------------------------------

/// OpenGL program
struct gl_program {
    // program id
    uint _pid = 0;
    // vertex shader id
    uint _vid = 0;
    // fragment shader id
    uint _fid = 0;
    // vertex array object
    uint _vao = 0;
};

/// Creates and OpenGL program from vertex and fragment code. Returns the
/// program id. Optionally return vertex and fragment shader ids. A VAO is
/// created.
inline gl_program make_program(const string& vertex, const string& fragment) {
    auto prog = gl_program();

    assert(gl_check_error());
    glGenVertexArrays(1, &prog._vao);
    glBindVertexArray(prog._vao);
    assert(gl_check_error());

    int errflags;
    char errbuf[10000];

    // create vertex
    prog._vid = glCreateShader(GL_VERTEX_SHADER);
    const char* vertex_str = vertex.c_str();
    glShaderSource(prog._vid, 1, &vertex_str, NULL);
    glCompileShader(prog._vid);
    glGetShaderiv(prog._vid, GL_COMPILE_STATUS, &errflags);
    if (!errflags) {
        glGetShaderInfoLog(prog._vid, 10000, 0, errbuf);
        throw runtime_error(string("shader not compiled\n\n") + errbuf);
    }
    assert(glGetError() == GL_NO_ERROR);

    // create fragment
    prog._fid = glCreateShader(GL_FRAGMENT_SHADER);
    const char* fragment_str = fragment.c_str();
    glShaderSource(prog._fid, 1, &fragment_str, NULL);
    glCompileShader(prog._fid);
    glGetShaderiv(prog._fid, GL_COMPILE_STATUS, &errflags);
    if (!errflags) {
        glGetShaderInfoLog(prog._fid, 10000, 0, errbuf);
        throw runtime_error(string("shader not compiled\n\n") + errbuf);
    }
    assert(glGetError() == GL_NO_ERROR);

    // create program
    prog._pid = glCreateProgram();
    glAttachShader(prog._pid, prog._vid);
    glAttachShader(prog._pid, prog._fid);
    glLinkProgram(prog._pid);
    glValidateProgram(prog._pid);
    glGetProgramiv(prog._pid, GL_LINK_STATUS, &errflags);
    if (!errflags) {
        glGetProgramInfoLog(prog._pid, 10000, 0, errbuf);
        throw runtime_error(string("program not linked\n\n") + errbuf);
    }
    glGetProgramiv(prog._pid, GL_VALIDATE_STATUS, &errflags);
    if (!errflags) {
        glGetProgramInfoLog(prog._pid, 10000, 0, errbuf);
        throw runtime_error(string("program not linked\n\n") + errbuf);
    }
    assert(gl_check_error());

    glBindVertexArray(0);
    assert(gl_check_error());

    return prog;
}

/// Destroys the program pid and optionally the sahders vid and fid.
inline void clear_program(gl_program& prog) {
    assert(gl_check_error());
    glDetachShader(prog._pid, prog._vid);
    glDeleteShader(prog._vid);
    prog._vid = 0;
    glDetachShader(prog._pid, prog._fid);
    glDeleteShader(prog._fid);
    prog._fid = 0;
    glDeleteProgram(prog._pid);
    prog._pid = 0;
    glDeleteVertexArrays(1, &prog._vao);
    prog._vao = 0;
    assert(gl_check_error());
}

/// Get uniform location (simple GL wrapper that avoids GL includes)
inline int get_program_uniform_location(
    const gl_program& prog, const string& name) {
    assert(gl_check_error());
    return glGetUniformLocation(prog._pid, name.c_str());
    assert(gl_check_error());
}

/// Get uniform location (simple GL wrapper that avoids GL includes)
inline int get_program_attrib_location(
    const gl_program& prog, const string& name) {
    assert(gl_check_error());
    return glGetAttribLocation(prog._pid, name.c_str());
    assert(gl_check_error());
}

/// Get the names of all uniforms
inline vector<pair<string, int>> get_program_uniforms_names(
    const gl_program& prog) {
    auto num = 0;
    assert(gl_check_error());
    glGetProgramiv(prog._pid, GL_ACTIVE_UNIFORMS, &num);
    assert(gl_check_error());
    auto names = vector<pair<string, int>>();
    for (auto i = 0; i < num; i++) {
        char name[4096];
        auto size = 0, length = 0;
        GLenum type;
        glGetActiveUniform(prog._pid, i, 4096, &length, &size, &type, name);
        if (length > 3 && name[length - 1] == ']' && name[length - 2] == '0' &&
            name[length - 3] == '[')
            name[length - 3] = 0;
        auto loc = glGetUniformLocation(prog._pid, name);
        if (loc < 0) continue;
        names.push_back({name, loc});
        assert(gl_check_error());
    }
    return names;
}

/// Get the names of all attributes
inline vector<pair<string, int>> get_program_attributes_names(
    const gl_program& prog) {
    auto num = 0;
    assert(gl_check_error());
    glGetProgramiv(prog._pid, GL_ACTIVE_ATTRIBUTES, &num);
    assert(gl_check_error());
    auto names = vector<pair<string, int>>();
    for (auto i = 0; i < num; i++) {
        char name[4096];
        auto size = 0;
        GLenum type;
        glGetActiveAttrib(prog._pid, i, 4096, nullptr, &size, &type, name);
        auto loc = glGetAttribLocation(prog._pid, name);
        if (loc < 0) continue;
        names.push_back({name, loc});
        assert(gl_check_error());
    }
    return names;
}

/// Set uniform integer values val for program pid and variable loc.
/// The values have nc number of components (1-4) and count elements
/// (for arrays).
inline bool set_program_uniform(
    gl_program& prog, int pos, const int* val, int ncomp, int count) {
    assert(ncomp >= 1 && ncomp <= 4);
    assert(gl_check_error());
    if (pos < 0) return false;
    switch (ncomp) {
        case 1: glUniform1iv(pos, count, val); break;
        case 2: glUniform2iv(pos, count, val); break;
        case 3: glUniform3iv(pos, count, val); break;
        case 4: glUniform4iv(pos, count, val); break;
        default: assert(false);
    }
    assert(gl_check_error());
    return true;
}

/// Set uniform float values val for program pid and variable var.
/// The values have nc number of components (1-4) and count elements
/// (for arrays).
inline bool set_program_uniform(
    gl_program& prog, int pos, const float* val, int ncomp, int count) {
    assert((ncomp >= 1 && ncomp <= 4) || (ncomp == 16) || (ncomp == 12));
    assert(gl_check_error());
    if (pos < 0) return false;
    switch (ncomp) {
        case 1: glUniform1fv(pos, count, val); break;
        case 2: glUniform2fv(pos, count, val); break;
        case 3: glUniform3fv(pos, count, val); break;
        case 4: glUniform4fv(pos, count, val); break;
        case 12: glUniformMatrix4x3fv(pos, count, false, val); break;
        case 16: glUniformMatrix4fv(pos, count, false, val); break;
        default: assert(false); return 0;
    }
    assert(gl_check_error());
    return true;
}

/// Set uniform float values val for program pid and variable var.
inline bool set_program_uniform(gl_program& prog, int var, bool val) {
    auto vali = (int)val;
    return set_program_uniform(prog, var, &vali, 1, 1);
}

/// Set uniform float values val for program pid and variable var.
inline bool set_program_uniform(gl_program& prog, int var, int val) {
    return set_program_uniform(prog, var, &val, 1, 1);
}

/// Set uniform float values val for program pid and variable var.
inline bool set_program_uniform(gl_program& prog, int var, float val) {
    return set_program_uniform(prog, var, &val, 1, 1);
}

/// Set uniform float values val for program pid and variable var.
template <int N>
inline bool set_program_uniform(
    gl_program& prog, int var, const vec<float, N>& val) {
    return set_program_uniform(prog, var, val.data(), N, 1);
}

/// Set uniform float values val for program pid and variable var.
template <int N>
inline bool set_program_uniform(
    gl_program& prog, int var, const vec<int, N>& val) {
    return set_program_uniform(prog, var, val.data(), N, 1);
}

/// Set uniform float values val for program pid and variable var.
inline bool set_program_uniform(gl_program& prog, int var, const mat4f& val) {
    return set_program_uniform(prog, var, (float*)val.data(), 16, 1);
}

/// Set uniform float values val for program pid and variable var.
inline bool set_program_uniform(gl_program& prog, int var, const frame3f& val) {
    return set_program_uniform(prog, var, (float*)val.data(), 12, 1);
}

/// Set uniform float values val for program pid and variable var.
inline bool set_program_uniform(
    gl_program& prog, int var, const int* val, int num) {
    return set_program_uniform(prog, var, val, 1, num);
}

/// Set uniform float values val for program pid and variable var.
inline bool set_program_uniform(
    gl_program& prog, int var, const float* val, int num) {
    return set_program_uniform(prog, var, val, 1, num);
}

/// Set uniform float values val for program pid and variable var.
template <int N>
inline bool set_program_uniform(
    gl_program& prog, int var, const vec<float, N>* val, int num) {
    return set_program_uniform(prog, var, val->data(), N, num);
}

/// Set uniform float values val for program pid and variable var.
template <int N>
inline bool set_program_uniform(
    gl_program& prog, int var, const vec<int, N>* val, int num) {
    return set_program_uniform(prog, var, val->data(), N, num);
}

/// Set uniform float values val for program pid and variable var.
inline bool set_program_uniform(
    gl_program& prog, int var, const mat4f* val, int num) {
    return set_program_uniform(prog, var, (float*)val, 16, num);
}

/// Set uniform float values val for program pid and variable var.
template <typename T>
inline bool set_program_uniform(
    gl_program& prog, const string& var, const T& val) {
    auto loc = get_program_uniform_location(prog, var);
    if (loc < 0) return false;
    return set_program_uniform(prog, loc, val);
}

/// Set uniform float values val for program pid and variable var.
template <typename T>
inline bool set_program_uniform(
    gl_program& prog, const string& var, const T* val, int num) {
    auto loc = get_program_uniform_location(prog, var);
    if (loc < 0) return false;
    return set_program_uniform(loc, val, num);
}

/// Set uniform texture id tid and unit tunit for program pid and variable
/// var.
inline bool set_program_uniform_texture(
    gl_program& prog, int pos, const gl_texture_info& tinfo, uint tunit) {
    static const auto wrap_mode_map =
        map<gl_texture_wrap, uint>{{gl_texture_wrap::repeat, GL_REPEAT},
            {gl_texture_wrap::clamp, GL_CLAMP_TO_EDGE},
            {gl_texture_wrap::mirror, GL_MIRRORED_REPEAT}};
    static const auto filter_mode_map = map<gl_texture_filter, uint>{
        {gl_texture_filter::nearest, GL_NEAREST},
        {gl_texture_filter::linear, GL_LINEAR},
        {gl_texture_filter::nearest_mipmap_nearest, GL_NEAREST_MIPMAP_NEAREST},
        {gl_texture_filter::linear_mipmap_nearest, GL_LINEAR_MIPMAP_NEAREST},
        {gl_texture_filter::nearest_mipmap_linear, GL_NEAREST_MIPMAP_LINEAR},
        {gl_texture_filter::linear_mipmap_linear, GL_LINEAR_MIPMAP_LINEAR}};

    assert(gl_check_error());
    if (pos < 0) return false;
    if (is_texture_valid(tinfo.txt)) {
        bind_texture(tinfo.txt, tunit);
        if (tinfo.wrap_s != gl_texture_wrap::not_set)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,
                wrap_mode_map.at(tinfo.wrap_s));
        if (tinfo.wrap_t != gl_texture_wrap::not_set)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,
                wrap_mode_map.at(tinfo.wrap_t));
        if (tinfo.filter_min != gl_texture_filter::not_set)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                filter_mode_map.at(tinfo.filter_min));
        if (tinfo.filter_mag != gl_texture_filter::not_set)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
                filter_mode_map.at(tinfo.filter_mag));
        glUniform1i(pos, tunit);
    } else {
        unbind_texture(tinfo.txt, tunit);
        glUniform1i(pos, tunit);
    }
    assert(gl_check_error());
    return true;
}

/// Set uniform texture id tid and unit tunit for program pid and variable
/// var. Optionally sets the int variable varon to 0/1 whether the texture
/// is enable on not.
inline bool set_program_uniform_texture(gl_program& prog, int var, int varon,
    const gl_texture_info& tinfo, uint tunit) {
    if (!set_program_uniform_texture(prog, var, tinfo, tunit)) return false;
    if (!set_program_uniform(prog, varon, is_texture_valid(tinfo.txt)))
        return false;
    return true;
}

/// Set uniform texture id tid and unit tunit for program pid and variable
/// var.
inline bool set_program_uniform_texture(gl_program& prog, const string& var,
    const gl_texture_info& tinfo, uint tunit) {
    auto loc = get_program_uniform_location(prog, var);
    if (loc < 0) return false;
    return set_program_uniform_texture(prog, loc, tinfo, tunit);
}

/// Set uniform texture id tid and unit tunit for program pid and variable
/// var. Optionally sets the int variable varon to 0/1 whether the texture
/// is enable on not.
inline bool set_program_uniform_texture(gl_program& prog, const string& var,
    const string& varon, const gl_texture_info& tinfo, uint tunit) {
    auto loc = get_program_uniform_location(prog, var);
    if (loc < 0) return false;
    auto locon = get_program_uniform_location(prog, varon);
    if (locon < 0) return false;
    return set_program_uniform_texture(prog, loc, locon, tinfo, tunit);
}

/// Sets a constant value for a vertex attribute for program pid and
/// variable var. The attribute has nc components.
inline bool set_program_vertattr(
    gl_program& prog, int pos, const float* value, int nc) {
    assert(nc >= 1 && nc <= 4);
    assert(gl_check_error());
    if (pos < 0) return false;
    glDisableVertexAttribArray(pos);
    switch (nc) {
        case 1: glVertexAttrib1fv(pos, value); break;
        case 2: glVertexAttrib2fv(pos, value); break;
        case 3: glVertexAttrib3fv(pos, value); break;
        case 4: glVertexAttrib4fv(pos, value); break;
        default: assert(false); break;
    }
    assert(gl_check_error());
    return true;
}

/// Sets a constant value for a vertex attribute for program pid and
/// variable var. The attribute has nc components.
inline bool set_program_vertattr(
    gl_program& prog, int pos, const int* value, int nc) {
    assert(nc >= 1 && nc <= 4);
    assert(gl_check_error());
    if (pos < 0) return false;
    glDisableVertexAttribArray(pos);
    switch (nc) {
        case 1: glVertexAttribI1iv(pos, value); break;
        case 2: glVertexAttribI2iv(pos, value); break;
        case 3: glVertexAttribI3iv(pos, value); break;
        case 4: glVertexAttribI4iv(pos, value); break;
        default: assert(false); break;
    }
    assert(gl_check_error());
    return true;
}

/// Sets a vartex attribute for program pid and variable var to the buffer
/// bid. The attribute has nc components and per-vertex values values.
inline bool set_program_vertattr(
    gl_program& prog, const string& var, const gl_vertex_buffer& buf) {
    assert(gl_check_error());
    int pos = glGetAttribLocation(prog._pid, var.c_str());
    if (pos < 0) return false;
    if (is_vertex_buffer_valid(buf)) {
        glEnableVertexAttribArray(pos);
        bind_vertex_buffer(buf, pos);
    } else {
        glDisableVertexAttribArray(pos);
        unbind_vertex_buffer(buf, pos);
    }
    assert(gl_check_error());
    return true;
}

/// Sets a vartex attribute for program pid and variable var. The attribute
/// has nc components and either buffer bid or a single value def
/// (if bid is zero). Convenience wrapper to above functions.
inline bool set_program_vertattr(gl_program& prog, int pos,
    const gl_vertex_buffer& buf, int nc, const float* def) {
    assert(nc >= 1 && nc <= 4);
    assert(gl_check_error());
    if (pos < 0) return false;
    if (is_vertex_buffer_valid(buf)) {
        assert(gl_check_error());
        glEnableVertexAttribArray(pos);
        assert(gl_check_error());
        bind_vertex_buffer(buf, pos);
        assert(gl_check_error());
    } else {
        glDisableVertexAttribArray(pos);
        unbind_vertex_buffer(buf, pos);
        if (def) {
            switch (nc) {
                case 1: glVertexAttrib1fv(pos, def); break;
                case 2: glVertexAttrib2fv(pos, def); break;
                case 3: glVertexAttrib3fv(pos, def); break;
                case 4: glVertexAttrib4fv(pos, def); break;
                default: assert(false); break;
            }
        }
    }
    assert(gl_check_error());
    return true;
}

/// Sets a vartex attribute for program pid and variable var. The attribute
/// is either a buffer bid or a single value def
/// (if bid is zero). Convenience wrapper to above functions.
template <int N>
inline bool set_program_vertattr(gl_program& prog, int var,
    const gl_vertex_buffer& buf, const vec<float, N>& def) {
    return set_program_vertattr(prog, var, buf, N, def.data());
}

/// Sets a vartex attribute for program pid and variable var. The attribute
/// is either a buffer bid or a single value def
/// (if bid is zero). Convenience wrapper to above functions.
template <int N>
inline bool set_program_vertattr(gl_program& prog, const string& var,
    const gl_vertex_buffer& buf, const vec<float, N>& def) {
    auto loc = get_program_attrib_location(prog, var);
    if (loc < 0) return false;
    return set_program_vertattr(prog, loc, buf, def);
}

/// Check whether it is valid
inline bool is_program_valid(const gl_program& prog) { return (bool)prog._pid; }

/// Binds a program
inline void bind_program(const gl_program& prog) {
    assert(gl_check_error());
    if (!prog._pid) return;
    glBindVertexArray(prog._vao);
    glUseProgram(prog._pid);
    assert(gl_check_error());
}

/// Unbind a program
inline void unbind_program(const gl_program& prog) {
    assert(gl_check_error());
    glUseProgram(0);
    glBindVertexArray(0);
    assert(gl_check_error());
}

// -----------------------------------------------------------------------------
// IMAGE SHADER FUNCTIONS
// -----------------------------------------------------------------------------

/// A shader for displaying images
struct gl_stdimage_program {
    // program
    gl_program _prog = {};
    // vertex array
    gl_vertex_buffer _vbo = {};
    // element array
    gl_element_buffer _ebo = {};

    string _header =
        R"(
#version 330

        float pi = 3.14159265;

        uniform vec2 offset;
        uniform vec2 win_size;
        uniform float zoom;

        uniform sampler2D img;

        )";

    string _vert =
        R"(
        layout(location = 0) in vec2 vert_texcoord;

        out vec2 texcoord;

        void main() {
            vec2 size = textureSize(img, 0).xy;
            texcoord = vert_texcoord.xy;
            vec2 pos = offset + size * vert_texcoord.xy * zoom;
            vec2 upos = 2 * pos / win_size - vec2(1,1);
            upos.y = - upos.y;
            gl_Position = vec4(upos.x, upos.y, 0, 1);
        }

        )";

    string _frag_tonemap =
        R"(
        struct Tonemap {
            int type;
            float exposure;
            float gamma;
        };
        uniform Tonemap tonemap;

        vec3 eval_filmic(vec3 x) {
            float a = 2.51f;
            float b = 0.03f;
            float c = 2.43f;
            float d = 0.59f;
            float e = 0.14f;
            return clamp((x*(a*x+b))/(x*(c*x+d)+e),0,1);
        }

        vec3 eval_tonemap(vec3 c) {
            // final color correction
            c = c*pow(2,tonemap.exposure);
            if(tonemap.type == 1) {
                c = pow(c,vec3(1/2.2));
            } else if(tonemap.type == 2) {
                c = pow(c,vec3(1/tonemap.gamma));
            } else if(tonemap.type == 3) {
                c = eval_filmic(c);
            }
            return c;
        }

        )";

    string _frag_main =
        R"(
        in vec2 texcoord;
        out vec4 color;

        void main() {
            vec4 c = texture(img,texcoord);
            c.xyz = eval_tonemap(c.xyz);
            color = c;
        }
        )";
};

/// Initialize the program. Call with true only after the GL is initialized.
inline gl_stdimage_program make_stdimage_program() {
    auto prog = gl_stdimage_program();
    prog._prog = make_program(prog._header + prog._vert,
        prog._header + prog._frag_tonemap + prog._frag_main);

    prog._vbo = make_vertex_buffer(
        vector<vec2f>{{0, 0}, {0, 1}, {1, 1}, {1, 0}}, false);
    prog._ebo = make_element_buffer(vector<vec3i>{{0, 1, 2}, {0, 2, 3}}, false);
    return prog;
}

/// As above but includes an exposure/gamma correction.
inline void draw_image(gl_stdimage_program& prog, const gl_texture& txt,
    const vec2i& win_size, const vec2f& offset, float zoom, float exposure,
    tonemap_type tmtype, float gamma) {
    assert(is_texture_valid(txt));

    bind_program(prog._prog);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    bind_texture(txt, 0);
    set_program_uniform(prog._prog, "zoom", zoom);
    set_program_uniform(
        prog._prog, "win_size", vec2f{(float)win_size.x, (float)win_size.y});
    set_program_uniform(prog._prog, "offset", offset);
    set_program_uniform(prog._prog, "tonemap.type", (int)tmtype);
    set_program_uniform(prog._prog, "tonemap.exposure", exposure);
    set_program_uniform(prog._prog, "tonemap.gamma", gamma);
    set_program_uniform_texture(prog._prog, "img", txt, 0);

    set_program_vertattr(prog._prog, "vert_texcoord", prog._vbo, vec2f{0, 0});
    draw_elems(prog._ebo);

    unbind_program(prog._prog);

    glDisable(GL_BLEND);

    assert(gl_check_error());
}

/// Draw an texture tid of size img_w, img_h on a window of size win_w,
/// win_h with top-left corner at ox, oy with a zoom zoom.
inline void draw_image(gl_stdimage_program& prog, const gl_texture& txt,
    const vec2i& win_size, const vec2f& offset, float zoom) {
    assert(gl_check_error());
    draw_image(prog, txt, win_size, offset, zoom, 0, tonemap_type::none, 1);
    assert(gl_check_error());
}

// -----------------------------------------------------------------------------
// STANDARD SHADER FUNCTIONS
// -----------------------------------------------------------------------------

/// Shade with a physically-based standard shader based on Phong/GGX.
/// Filmic tone mapping from
/// https://knarkowicz.wordpress.com/2016/01/06/aces-filmic-tone-mapping-curve/
struct gl_stdsurface_program {
    // gl program
    gl_program _prog = {};

#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Woverlength-strings"
#endif
    string _vert_header =
        R"(
#version 330

        )";

    string _vert_skinning =
        R"(
#define SKIN_NONE 0
#define SKIN_STD 1
#define SKIN_GLTF 2

        uniform int skin_type = 0;
        uniform mat4 skin_xforms[32];
        layout(location = 6) in vec4 vert_skin_weights;            // vertex skinning weights
        layout(location = 7) in ivec4 vert_skin_joints;            // vertex skinning joints (in mesh coordinate frame)

        vec3 transform_point(mat4 m, vec3 p) {
            vec4 p4 = m * vec4(p,1);
            return p4.xyz / p4.w;
        }

        vec3 transform_normal(mat4 m, vec3 p) {
            vec4 p4 = m * vec4(p,0);
            return p4.xyz;
        }

        void apply_skin(inout vec3 pos, inout vec3 norm) {
            if(skin_type == 0) {
                return;
            } else if(skin_type == SKIN_STD) {
                vec4 w = vert_skin_weights;
                ivec4 j = ivec4(vert_skin_joints);
                pos = transform_point( skin_xforms[j.x], pos ) * w.x +
                transform_point( skin_xforms[j.y], pos ) * w.y +
                transform_point( skin_xforms[j.z], pos ) * w.z +
                transform_point( skin_xforms[j.w], pos ) * w.w;
                norm = normalize(
                                 transform_normal( skin_xforms[j.x], norm ) * w.x +
                                 transform_normal( skin_xforms[j.y], norm ) * w.y +
                                 transform_normal( skin_xforms[j.z], norm ) * w.z +
                                 transform_normal( skin_xforms[j.w], norm ) * w.w);
            } else if(skin_type == SKIN_GLTF) {
                vec4 w = vert_skin_weights;
                ivec4 j = ivec4(vert_skin_joints);
                mat4 xf = skin_xforms[j.x] * w.x + skin_xforms[j.y] * w.y +
                skin_xforms[j.z] * w.z + skin_xforms[j.w] * w.w;
                pos = transform_point(xf, pos);
                norm = normalize(transform_normal(xf, norm));
            }
        }
        )";

    string _vert_main =
        R"(
        layout(location = 0) in vec3 vert_pos;            // vertex position (in mesh coordinate frame)
        layout(location = 1) in vec3 vert_norm;           // vertex normal (in mesh coordinate frame)
        layout(location = 2) in vec2 vert_texcoord;       // vertex texcoords
        layout(location = 3) in vec4 vert_color;          // vertex color
        layout(location = 4) in vec4 vert_tangsp;         // vertex tangent space

        uniform mat4 shape_xform;           // shape transform

        struct Camera {
            mat4 xform;          // camera xform
            mat4 xform_inv;      // inverse of the camera frame (as a matrix)
            mat4 proj;           // camera projection
        };
        uniform Camera camera;      // camera data

        out vec3 pos;                   // [to fragment shader] vertex position (in world coordinate)
        out vec3 norm;                  // [to fragment shader] vertex normal (in world coordinate)
        out vec2 texcoord;              // [to fragment shader] vertex texture coordinates
        out vec4 color;                 // [to fragment shader] vertex color
        out vec4 tangsp;                // [to fragment shader] vertex tangent space

        // main function
        void main() {
            // copy values
            pos = vert_pos;
            norm = vert_norm;
            tangsp = vert_tangsp;

            // world projection
            pos = (shape_xform * vec4(pos,1)).xyz;
            norm = (shape_xform * vec4(norm,0)).xyz;
            tangsp.xyz = (shape_xform * vec4(tangsp.xyz,0)).xyz;

            // skinning
            apply_skin(pos, norm);

            // copy other vertex properties
            texcoord = vert_texcoord;
            color = vert_color;

            // clip
            gl_Position = camera.proj * camera.xform_inv * vec4(pos,1);
        }
        )";

    string _frag_header =
        R"(
#version 330

        float pi = 3.14159265;

        )";

    string _frag_tonemap =
        R"(
        struct Tonemap {
            int type;       // tonemap type (TM_...)
            float exposure; // image exposure
            float gamma;    // image gamma
        };
        uniform Tonemap tonemap;

        vec3 eval_filmic(vec3 x) {
            float a = 2.51f;
            float b = 0.03f;
            float c = 2.43f;
            float d = 0.59f;
            float e = 0.14f;
            return clamp((x*(a*x+b))/(x*(c*x+d)+e),0,1);
        }

        vec3 eval_tonemap(vec3 c) {
            // final color correction
            c = c*pow(2,tonemap.exposure);
            if(tonemap.type == 1) {
                c = pow(c,vec3(1/2.2));
            } else if(tonemap.type == 2) {
                c = pow(c,vec3(1/tonemap.gamma));
            } else if(tonemap.type == 3) {
                c = eval_filmic(c);
            }
            return c;
        }

        )";

    string _frag_lighting =
        R"(
        struct Lighting {
            bool eyelight;        // eyelight shading
            vec3 amb;             // ambient light
            int lnum;              // number of lights
            int ltype[16];         // light type (0 -> point, 1 -> directional)
            vec3 lpos[16];         // light positions
            vec3 lke[16];          // light intensities
        };
        uniform Lighting lighting;

        void eval_light(int lid, vec3 pos, out vec3 cl, out vec3 wi) {
            cl = vec3(0,0,0);
            wi = vec3(0,0,0);
            if(lighting.ltype[lid] == 0) {
                // compute point light color at pos
                cl = lighting.lke[lid] / pow(length(lighting.lpos[lid]-pos),2);
                // compute light direction at pos
                wi = normalize(lighting.lpos[lid]-pos);
            }
            else if(lighting.ltype[lid] == 1) {
                // compute light color
                cl = lighting.lke[lid];
                // compute light direction
                wi = normalize(lighting.lpos[lid]);
            }
        }

        )";

    string _frag_brdf =
        R"(
        struct Brdf {
            int type;
            vec3 ke;
            vec3 kd;
            vec3 ks;
            float rs;
            float op;
            bool cutout;
        };

        vec3 brdfcos(Brdf brdf, vec3 n, vec3 wi, vec3 wo) {
            if(brdf.type == 0) return vec3(0);
            vec3 wh = normalize(wi+wo);
            float ns = 2/(brdf.rs*brdf.rs)-2;
            float ndi = dot(wi,n), ndo = dot(wo,n), ndh = dot(wh,n);
            if(brdf.type == 1) {
                return ((1+dot(wo,wi))/2) * brdf.kd/pi;
            } else if(brdf.type == 2) {
                float si = sqrt(1-ndi*ndi);
                float so = sqrt(1-ndo*ndo);
                float sh = sqrt(1-ndh*ndh);
                if(si <= 0) return vec3(0);
                vec3 diff = si * brdf.kd / pi;
                if(sh<=0) return diff;
                float d = ((2+ns)/(2*pi)) * pow(si,ns);
                vec3 spec = si * brdf.ks * d / (4*si*so);
                return diff+spec;
            } else if(brdf.type == 3 || brdf.type == 4) {
                if(ndi<=0 || ndo <=0) return vec3(0);
                vec3 diff = ndi * brdf.kd / pi;
                if(ndh<=0) return diff;
                if(brdf.type == 4) {
                    float d = ((2+ns)/(2*pi)) * pow(ndh,ns);
                    vec3 spec = ndi * brdf.ks * d / (4*ndi*ndo);
                    return diff+spec;
                } else {
                    float cos2 = ndh * ndh;
                    float tan2 = (1 - cos2) / cos2;
                    float alpha2 = brdf.rs * brdf.rs;
                    float d = alpha2 / (pi * cos2 * cos2 * (alpha2 + tan2) * (alpha2 + tan2));
                    float lambda_o = (-1 + sqrt(1 + (1 - ndo * ndo) / (ndo * ndo))) / 2;
                    float lambda_i = (-1 + sqrt(1 + (1 - ndi * ndi) / (ndi * ndi))) / 2;
                    float g = 1 / (1 + lambda_o + lambda_i);
                    vec3 spec = ndi * brdf.ks * d * g / (4*ndi*ndo);
                    return diff+spec;
                }
            }
        }

        )";

    string _frag_material =
        R"(
        struct Material {
            int mtype;         // material type
            int etype;         // element type
            vec3 ke;           // material ke
            vec3 kd;           // material kd
            vec3 ks;           // material ks
            float rs;          // material rs
            float op;          // material op

            bool txt_ke_on;    // material ke texture on
            sampler2D txt_ke;  // material ke texture
            bool txt_kd_on;    // material kd texture on
            sampler2D txt_kd;  // material kd texture
            bool txt_ks_on;    // material ks texture on
            sampler2D txt_ks;  // material ks texture
            bool txt_rs_on;    // material rs texture on
            sampler2D txt_rs;  // material rs texture

            bool txt_norm_on;    // material norm texture on
            sampler2D txt_norm;  // material norm texture
            sampler2D txt_norm_scale;  // material norm scale

            bool txt_occ_on;    // material occ texture on
            sampler2D txt_occ;  // material occ texture
            sampler2D txt_occ_scale;  // material occ scale

            bool use_phong;       // material use phong
            bool double_sided;    // material double sided
            bool alpha_cutout;    // material alpha cutout
        };
        uniform Material material;

        void eval_material(vec2 texcoord, vec4 color, out int type, out vec3 ke,
                           out vec3 kd, out vec3 ks, out float rs, out float op, out bool cutout) {
            ke = color.xyz * material.ke;
            kd = color.xyz * material.kd;
            ks = color.xyz * material.ks;
            rs = material.rs;
            op = color.w * material.op;

            vec3 ke_txt = (material.txt_ke_on) ? texture(material.txt_ke,texcoord).xyz : vec3(1);
            vec4 kd_txt = (material.txt_kd_on) ? texture(material.txt_kd,texcoord) : vec4(1);
            vec4 ks_txt = (material.txt_ks_on) ? texture(material.txt_ks,texcoord) : vec4(1);
            float rs_txt = (material.txt_rs_on) ? texture(material.txt_rs,texcoord).x : 1;

            // scale common values
            ke *= ke_txt;

            // get material color from textures and adjust values
            if(material.mtype == 0) {
                type = 0;
            } else if(material.mtype == 1) {
                type = material.etype;
                kd *= kd_txt.xyz;
                ks *= ks_txt.xyz;
                rs *= rs_txt;
                rs = rs*rs;
            } else if(material.mtype == 2) {
                type = material.etype;
                vec3 kb = kd * kd_txt.xyz;
                float km = ks.x * ks_txt.z;
                kd = kb * (1 - km);
                ks = kb * km + vec3(0.04) * (1 - km);
                rs *= ks_txt.y;
                rs = rs*rs;
                op *= kd_txt.w;
            } else if(material.mtype == 3) {
                type = material.etype;
                kd *= kd_txt.xyz;
                ks *= ks_txt.xyz;
                rs *= ks_txt.w;
                rs = (1 - rs) * (1 - rs);
                op *= kd_txt.w;
            }

            cutout = material.alpha_cutout && op == 0;
        }

        vec3 apply_normal_map(vec2 texcoord, vec3 norm, vec4 tangsp) {
            if(!material.txt_norm_on) return norm;
            vec3 tangu = normalize(tangsp.xyz);
            vec3 tangv = normalize(cross(tangu, norm));
            if(tangsp.w < 0) tangv = -tangv;
            vec3 txt = 2 * pow(texture(material.txt_norm,texcoord).xyz, vec3(1/2.2)) - 1;
            return normalize( tangu * txt.x + tangv * txt.y + norm * txt.z );
        }

        )";

    string _frag_main =
        R"(
        in vec3 pos;                   // [from vertex shader] position in world space
        in vec3 norm;                  // [from vertex shader] normal in world space (need normalization)
        in vec2 texcoord;              // [from vertex shader] texcoord
        in vec4 color;                 // [from vertex shader] color
        in vec4 tangsp;                // [from vertex shader] tangent space

        struct Camera {
            mat4 xform;          // camera xform
            mat4 xform_inv;      // inverse of the camera frame (as a matrix)
            mat4 proj;           // camera projection
        };
        uniform Camera camera;      // camera data

        uniform vec4 highlight;   // highlighted color

        out vec4 frag_color;        // eyelight shading

        // main
        void main() {
            // view vector
            vec3 wo = normalize( (camera.xform*vec4(0,0,0,1)).xyz - pos );

            // re-normalize normals
            vec3 n = normalize(norm);

            // apply normal map
            n = apply_normal_map(texcoord, n, tangsp);

            // use faceforward to ensure the normals points toward us
            if(material.double_sided) n = faceforward(n,-wo,n);

            // get material color from textures
            Brdf brdf;
            eval_material(texcoord, color, brdf.type, brdf.ke, brdf.kd, brdf.ks, brdf.rs, brdf.op, brdf.cutout);

            // exit if needed
            if(brdf.cutout) discard;

            // emission
            vec3 c = brdf.ke;

            // check early exit
            if(brdf.kd != vec3(0,0,0) || brdf.ks != vec3(0,0,0)) {
                // eyelight shading
                if(lighting.eyelight) {
                    vec3 wi = wo;
                    c += pi * brdfcos(brdf,n,wi,wo);
                } else {
                    // accumulate ambient
                    c += lighting.amb * brdf.kd;
                    // foreach light
                    for(int lid = 0; lid < lighting.lnum; lid ++) {
                        vec3 cl = vec3(0,0,0); vec3 wi = vec3(0,0,0);
                        eval_light(lid, pos, cl, wi);
                        c += cl * brdfcos(brdf,n,wi,wo);
                    }
                }
            }

            // final color correction
            c = eval_tonemap(c);

            // highlighting
            if(highlight.w > 0) {
                if(mod(int(gl_FragCoord.x)/4 + int(gl_FragCoord.y)/4, 2)  == 0)
                    c = highlight.xyz * highlight.w + c * (1-highlight.w);
            }

            // output final color by setting gl_FragColor
            frag_color = vec4(c,brdf.op);
        }
        )";
#ifndef _WIN32
#pragma GCC diagnostic pop
#endif
};

/// Initialize a standard shader. Call with true only after the gl has
/// been initialized
inline gl_stdsurface_program make_stdsurface_program() {
    assert(gl_check_error());
    auto prog = gl_stdsurface_program();
    prog._prog =
        make_program(prog._vert_header + prog._vert_skinning + prog._vert_main,
            prog._frag_header + prog._frag_tonemap + prog._frag_lighting +
                prog._frag_brdf + prog._frag_material + prog._frag_main);
    assert(gl_check_error());
    return prog;
}

/// Check if the program is valid
inline bool is_program_valid(const gl_stdsurface_program& prog) {
    return is_program_valid(prog._prog);
}

/// Starts a frame by setting exposure/gamma values, camera transforms and
/// projection. Sets also whether to use full shading or a quick eyelight
/// preview.
inline void begin_stdsurface_frame(gl_stdsurface_program& prog,
    bool shade_eyelight, float img_exposure, tonemap_type img_tonemap,
    float img_gamma, const mat4f& camera_xform, const mat4f& camera_xform_inv,
    const mat4f& camera_proj) {
    static auto eyelight_id =
        get_program_uniform_location(prog._prog, "lighting.eyelight");
    static auto exposure_id =
        get_program_uniform_location(prog._prog, "tonemap.exposure");
    static auto gamma_id =
        get_program_uniform_location(prog._prog, "tonemap.gamma");
    static auto type_id =
        get_program_uniform_location(prog._prog, "tonemap.type");
    static auto xform_id =
        get_program_uniform_location(prog._prog, "camera.xform");
    static auto xform_inv_id =
        get_program_uniform_location(prog._prog, "camera.xform_inv");
    static auto proj_id =
        get_program_uniform_location(prog._prog, "camera.proj");
    assert(gl_check_error());
    bind_program(prog._prog);
    set_program_uniform(prog._prog, eyelight_id, shade_eyelight);
    set_program_uniform(prog._prog, exposure_id, img_exposure);
    set_program_uniform(prog._prog, gamma_id, img_gamma);
    set_program_uniform(prog._prog, type_id, (int)img_tonemap);
    set_program_uniform(prog._prog, xform_id, camera_xform);
    set_program_uniform(prog._prog, xform_inv_id, camera_xform_inv);
    set_program_uniform(prog._prog, proj_id, camera_proj);
    assert(gl_check_error());
}

/// Ends a frame.
inline void end_stdsurface_frame(gl_stdsurface_program& prog) {
    assert(gl_check_error());
    unbind_program(prog._prog);
    glBindVertexArray(0);
    glUseProgram(0);
    assert(gl_check_error());
}

/// Set num lights with position pos, color ke, type ltype. Also set the
/// ambient illumination amb.
inline void set_stdsurface_lights(gl_stdsurface_program& prog, const vec3f& amb,
    int num, vec3f* pos, vec3f* ke, gl_ltype* type) {
    static auto amb_id =
        get_program_uniform_location(prog._prog, "lighting.amb");
    static auto lnum_id =
        get_program_uniform_location(prog._prog, "lighting.lnum");
    static auto lpos_id =
        get_program_uniform_location(prog._prog, "lighting.lpos");
    static auto lke_id =
        get_program_uniform_location(prog._prog, "lighting.lke");
    static auto ltype_id =
        get_program_uniform_location(prog._prog, "lighting.ltype");
    assert(gl_check_error());
    set_program_uniform(prog._prog, amb_id, amb);
    set_program_uniform(prog._prog, lnum_id, num);
    set_program_uniform(prog._prog, lpos_id, pos, num);
    set_program_uniform(prog._prog, lke_id, ke, num);
    set_program_uniform(prog._prog, ltype_id, (int*)type, num);
    assert(gl_check_error());
}

/// Begins drawing a shape with transform xform.
inline void begin_stdsurface_shape(
    gl_stdsurface_program& prog, const mat4f& xform) {
    static auto xform_id =
        get_program_uniform_location(prog._prog, "shape_xform");
    assert(gl_check_error());
    set_program_uniform(prog._prog, xform_id, xform);
    assert(gl_check_error());
}

/// End shade drawing.
inline void end_stdsurface_shape(gl_stdsurface_program& prog) {
    assert(gl_check_error());
    for (int i = 0; i < 16; i++) glDisableVertexAttribArray(i);
    assert(gl_check_error());
}

/// Set the object as highlighted.
inline void set_stdsurface_highlight(
    gl_stdsurface_program& prog, const vec4f& highlight) {
    static auto highlight_id =
        get_program_uniform_location(prog._prog, "highlight");
    set_program_uniform(prog._prog, highlight_id, highlight);
}

/// Set material values with emission ke, diffuse kd, specular ks and
/// specular roughness rs, opacity op. Indicates textures ids with the
/// correspoinding XXX_txt variables. Sets also normal and occlusion
/// maps. Works for points/lines/triangles (diffuse for points,
/// Kajiya-Kay for lines, GGX/Phong for triangles).
/// Material type matches the scene material type.
inline void set_stdsurface_material(gl_stdsurface_program& prog,
    material_type mtype, gl_etype etype, const vec3f& ke, const vec3f& kd,
    const vec3f& ks, float rs, float op, const gl_texture_info& ke_txt,
    const gl_texture_info& kd_txt, const gl_texture_info& ks_txt,
    const gl_texture_info& rs_txt, const gl_texture_info& norm_txt,
    const gl_texture_info& occ_txt, bool use_phong, bool double_sided,
    bool alpha_cutout) {
    static auto mtype_id =
        get_program_uniform_location(prog._prog, "material.mtype");
    static auto etype_id =
        get_program_uniform_location(prog._prog, "material.etype");
    static auto ke_id = get_program_uniform_location(prog._prog, "material.ke");
    static auto kd_id = get_program_uniform_location(prog._prog, "material.kd");
    static auto ks_id = get_program_uniform_location(prog._prog, "material.ks");
    static auto rs_id = get_program_uniform_location(prog._prog, "material.rs");
    static auto op_id = get_program_uniform_location(prog._prog, "material.op");
    static auto ke_txt_id =
        get_program_uniform_location(prog._prog, "material.txt_ke");
    static auto ke_txt_on_id =
        get_program_uniform_location(prog._prog, "material.txt_ke_on");
    static auto kd_txt_id =
        get_program_uniform_location(prog._prog, "material.txt_kd");
    static auto kd_txt_on_id =
        get_program_uniform_location(prog._prog, "material.txt_kd_on");
    static auto ks_txt_id =
        get_program_uniform_location(prog._prog, "material.txt_ks");
    static auto ks_txt_on_id =
        get_program_uniform_location(prog._prog, "material.txt_ks_on");
    static auto rs_txt_id =
        get_program_uniform_location(prog._prog, "material.txt_rs");
    static auto rs_txt_on_id =
        get_program_uniform_location(prog._prog, "material.txt_rs_on");
    static auto norm_txt_id =
        get_program_uniform_location(prog._prog, "material.txt_norm");
    static auto norm_txt_on_id =
        get_program_uniform_location(prog._prog, "material.txt_norm_on");
    static auto occ_txt_id =
        get_program_uniform_location(prog._prog, "material.txt_occ");
    static auto occ_txt_on_id =
        get_program_uniform_location(prog._prog, "material.txt_occ_on");
    static auto norm_scale_id =
        get_program_uniform_location(prog._prog, "material.norm_scale");
    static auto occ_scale_id =
        get_program_uniform_location(prog._prog, "material.occ_scale");
    static auto use_phong_id =
        get_program_uniform_location(prog._prog, "material.use_phong");
    static auto double_sided_id =
        get_program_uniform_location(prog._prog, "material.double_sided");
    static auto alpha_cutout_id =
        get_program_uniform_location(prog._prog, "material.alpha_cutout");

    static auto mtypes = unordered_map<material_type, int>{
        {material_type::specular_roughness, 1},
        {material_type::metallic_roughness, 2},
        {material_type::specular_glossiness, 3}};

    assert(gl_check_error());
    set_program_uniform(prog._prog, mtype_id, mtypes.at(mtype));
    set_program_uniform(prog._prog, etype_id, (int)etype);
    set_program_uniform(prog._prog, ke_id, ke);
    set_program_uniform(prog._prog, kd_id, kd);
    set_program_uniform(prog._prog, ks_id, ks);
    set_program_uniform(prog._prog, rs_id, rs);
    set_program_uniform(prog._prog, op_id, op);
    set_program_uniform_texture(prog._prog, ke_txt_id, ke_txt_on_id, ke_txt, 0);
    set_program_uniform_texture(prog._prog, kd_txt_id, kd_txt_on_id, kd_txt, 1);
    set_program_uniform_texture(prog._prog, ks_txt_id, ks_txt_on_id, ks_txt, 2);
    set_program_uniform_texture(prog._prog, rs_txt_id, rs_txt_on_id, rs_txt, 3);
    set_program_uniform_texture(
        prog._prog, norm_txt_id, norm_txt_on_id, norm_txt, 4);
    set_program_uniform_texture(
        prog._prog, occ_txt_id, occ_txt_on_id, occ_txt, 5);
    set_program_uniform(prog._prog, norm_scale_id, norm_txt.scale);
    set_program_uniform(prog._prog, occ_scale_id, occ_txt.scale);
    set_program_uniform(prog._prog, use_phong_id, use_phong);
    set_program_uniform(prog._prog, double_sided_id, double_sided);
    set_program_uniform(prog._prog, alpha_cutout_id, alpha_cutout);
    assert(gl_check_error());
}

/// Set vertex data with buffers for position pos, normals norm, texture
/// coordinates texcoord, per-vertex color color and tangent space tangsp.
inline void set_stdsurface_vert(gl_stdsurface_program& prog,
    const gl_vertex_buffer& pos, const gl_vertex_buffer& norm,
    const gl_vertex_buffer& texcoord, const gl_vertex_buffer& color,
    const gl_vertex_buffer& tangsp) {
    static auto pos_id = get_program_attrib_location(prog._prog, "vert_pos");
    static auto norm_id = get_program_attrib_location(prog._prog, "vert_norm");
    static auto texcoord_id =
        get_program_attrib_location(prog._prog, "vert_texcoord");
    static auto color_id =
        get_program_attrib_location(prog._prog, "vert_color");
    static auto tangsp_id =
        get_program_attrib_location(prog._prog, "vert_tangsp");
    assert(gl_check_error());
    set_program_vertattr(prog._prog, pos_id, pos, zero3f);
    set_program_vertattr(prog._prog, norm_id, norm, zero3f);
    set_program_vertattr(prog._prog, texcoord_id, texcoord, zero2f);
    set_program_vertattr(prog._prog, color_id, color, one4f);
    set_program_vertattr(prog._prog, tangsp_id, tangsp, zero4f);
    assert(gl_check_error());
}

/// Set vertex data with buffers for skinning.
inline void set_stdsurface_vert_skinning(gl_stdsurface_program& prog,
    const gl_vertex_buffer& weights, const gl_vertex_buffer& joints,
    int nxforms, const mat4f* xforms) {
    static auto type_id = get_program_uniform_location(prog._prog, "skin_type");
    static auto xforms_id =
        get_program_uniform_location(prog._prog, "skin_xforms");
    static auto weights_id =
        get_program_attrib_location(prog._prog, "vert_skin_weights");
    static auto joints_id =
        get_program_attrib_location(prog._prog, "vert_skin_joints");
    int type = 1;
    set_program_uniform(prog._prog, type_id, type);
    set_program_uniform(prog._prog, xforms_id, xforms, min(nxforms, 32));
    set_program_vertattr(prog._prog, weights_id, weights, zero4f);
    set_program_vertattr(prog._prog, joints_id, joints, zero4f);
}

/// Set vertex data with buffers for skinning.
inline void set_stdsurface_vert_gltf_skinning(gl_stdsurface_program& prog,
    const gl_vertex_buffer& weights, const gl_vertex_buffer& joints,
    int nxforms, const mat4f* xforms) {
    static auto type_id = get_program_uniform_location(prog._prog, "skin_type");
    static auto xforms_id =
        get_program_uniform_location(prog._prog, "skin_xforms");
    static auto weights_id =
        get_program_attrib_location(prog._prog, "vert_skin_weights");
    static auto joints_id =
        get_program_attrib_location(prog._prog, "vert_skin_joints");
    int type = 2;
    set_program_uniform(prog._prog, type_id, type);
    set_program_uniform(prog._prog, xforms_id, xforms, min(nxforms, 32));
    set_program_vertattr(prog._prog, weights_id, weights, zero4f);
    set_program_vertattr(prog._prog, joints_id, joints, zero4f);
}

/// Disables vertex skinning.
inline void set_stdsurface_vert_skinning_off(gl_stdsurface_program& prog) {
    static auto type_id = get_program_uniform_location(prog._prog, "skin_type");
    // static auto xforms_id = get_program_uniform_location(prog._prog,
    // "skin_xforms");
    static auto weights_id =
        get_program_attrib_location(prog._prog, "vert_skin_weights");
    static auto joints_id =
        get_program_attrib_location(prog._prog, "vert_skin_joints");
    int type = 0;
    set_program_uniform(prog._prog, type_id, type);
    set_program_vertattr(prog._prog, weights_id, {}, zero4f);
    set_program_vertattr(prog._prog, joints_id, {}, zero4f);
}
}

// -----------------------------------------------------------------------------
// OPENGL WINDOWS AND WIDGETS
// -----------------------------------------------------------------------------
namespace ygl {

#if YGL_GLFW

// Forward declaration
struct gl_window;

/// Text callback
typedef void (*gl_text_callback)(gl_window*, unsigned int);

/// Mouse callback
typedef void (*gl_mouse_callback)(gl_window*, int button, bool press, int mods);

/// Window refresh callback
typedef void (*gl_refresh_callback)(gl_window*);

/// Window
struct gl_window {
    GLFWwindow* _gwin = nullptr;
    void* _user_pointer = nullptr;
    int _widget_width = 320;
    bool _widget_enabled = false;
    gl_text_callback _text_cb = nullptr;
    gl_mouse_callback _mouse_cb = nullptr;
    gl_refresh_callback _refresh_cb = nullptr;
};

// Support
inline void _glfw_error_cb(int error, const char* description) {
    printf("GLFW error: %s\n", description);
}

// Support
inline void _glfw_text_cb(GLFWwindow* gwin, unsigned key) {
    auto win = (gl_window*)glfwGetWindowUserPointer(gwin);
    if (win->_widget_enabled) {
        ImGui_ImplGlfwGL3_CharCallback(win->_gwin, key);
    }
    if (win->_text_cb) win->_text_cb(win, key);
}

// Support
inline void _glfw_key_cb(
    GLFWwindow* gwin, int key, int scancode, int action, int mods) {
    auto win = (gl_window*)glfwGetWindowUserPointer(gwin);
    if (win->_widget_enabled) {
        ImGui_ImplGlfwGL3_KeyCallback(win->_gwin, key, scancode, action, mods);
    }
}

// Support
inline void _glfw_mouse_cb(GLFWwindow* gwin, int button, int action, int mods) {
    auto win = (gl_window*)glfwGetWindowUserPointer(gwin);
    if (win->_widget_enabled) {
        ImGui_ImplGlfwGL3_MouseButtonCallback(win->_gwin, button, action, mods);
    }
    if (win->_mouse_cb) win->_mouse_cb(win, button, action == GLFW_PRESS, mods);
}

// Support
inline void _glfw_scroll_cb(GLFWwindow* gwin, double xoffset, double yoffset) {
    auto win = (gl_window*)glfwGetWindowUserPointer(gwin);
    if (win->_widget_enabled) {
        ImGui_ImplGlfwGL3_ScrollCallback(win->_gwin, xoffset, yoffset);
    }
}

// Support
inline void _glfw_refresh_cb(GLFWwindow* gwin) {
    auto win = (gl_window*)glfwGetWindowUserPointer(gwin);
    if (win->_refresh_cb) win->_refresh_cb(win);
}

/// Initialize gl_window
inline gl_window* make_window(int width, int height, const string& title,
    void* user_pointer = nullptr, bool width_exclude_widgets = false) {
    auto win = new gl_window();
    // gl_window
    win->_user_pointer = user_pointer;
    if (width_exclude_widgets) width += win->_widget_width;

    // gl_window
    if (!glfwInit()) throw runtime_error("cannot open gl_window");

    // profile creation
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
#if __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    win->_gwin = glfwCreateWindow(width, height, title.c_str(), 0, 0);
    glfwMakeContextCurrent(win->_gwin);
    glfwSetWindowUserPointer(win->_gwin, win);

    glfwSetErrorCallback(_glfw_error_cb);

    glfwSetCharCallback(win->_gwin, _glfw_text_cb);
    glfwSetKeyCallback(win->_gwin, _glfw_key_cb);
    glfwSetMouseButtonCallback(win->_gwin, _glfw_mouse_cb);
    glfwSetScrollCallback(win->_gwin, _glfw_scroll_cb);

    glfwSetWindowRefreshCallback(win->_gwin, _glfw_refresh_cb);

// init gl extensions
#ifndef __APPLE__
    if (!glewInit()) return nullptr;
#endif
    return win;
}

/// Set gl_window callbacks
inline void set_window_callbacks(gl_window* win, gl_text_callback text_cb,
    gl_mouse_callback mouse_cb, gl_refresh_callback refresh_cb) {
    win->_text_cb = text_cb;
    win->_mouse_cb = mouse_cb;
    win->_refresh_cb = refresh_cb;
    if (win->_text_cb) glfwSetCharCallback(win->_gwin, _glfw_text_cb);
}

/// Clear gl_window
inline void clear_window(gl_window* win) {
    if (win->_gwin) {
        glfwDestroyWindow(win->_gwin);
        glfwTerminate();
        win->_gwin = nullptr;
    }
    if (win->_widget_enabled) {
        ImGui_ImplGlfwGL3_Shutdown();
        win->_widget_enabled = false;
    }
}

/// Gets the user poiner
inline void* get_user_pointer(gl_window* win) { return win->_user_pointer; }

/// Set gl_window title
inline void set_window_title(gl_window* win, const string& title) {
    glfwSetWindowTitle(win->_gwin, title.c_str());
}

/// Wait events
inline void wait_events(gl_window* win) { glfwWaitEvents(); }

/// Poll events
inline void poll_events(gl_window* win) { glfwPollEvents(); }

/// Swap buffers
inline void swap_buffers(gl_window* win) { glfwSwapBuffers(win->_gwin); }

/// Should close
inline bool should_close(gl_window* win) {
    return glfwWindowShouldClose(win->_gwin);
}

/// Mouse button
inline int get_mouse_button(gl_window* win) {
    auto mouse1 =
        glfwGetMouseButton(win->_gwin, GLFW_MOUSE_BUTTON_1) == GLFW_PRESS;
    auto mouse2 =
        glfwGetMouseButton(win->_gwin, GLFW_MOUSE_BUTTON_2) == GLFW_PRESS;
    auto mouse3 =
        glfwGetMouseButton(win->_gwin, GLFW_MOUSE_BUTTON_3) == GLFW_PRESS;
    if (mouse1) return 1;
    if (mouse2) return 2;
    if (mouse3) return 3;
#if 0
        if (action == GLFW_RELEASE) {
            vparams.mouse_button = 0;
        } else if (button == GLFW_MOUSE_BUTTON_1 && !mods) {
            vparams.mouse_button = 1;
        } else if (button == GLFW_MOUSE_BUTTON_1 && (mods & GLFW_MOD_CONTROL)) {
            vparams.mouse_button = 2;
        } else if (button == GLFW_MOUSE_BUTTON_1 && (mods & GLFW_MOD_SHIFT)) {
            vparams.mouse_button = 3;
        } else if (button == GLFW_MOUSE_BUTTON_2) {
            vparams.mouse_button = 2;
        } else {
            vparams.mouse_button = 0;
        }
#endif
    return 0;
}

/// Mouse position
inline vec2i get_mouse_pos(gl_window* win) {
    double x, y;
    glfwGetCursorPos(win->_gwin, &x, &y);
    return {(int)x, (int)y};
}

/// Mouse position
inline vec2f get_mouse_posf(gl_window* win) {
    double x, y;
    glfwGetCursorPos(win->_gwin, &x, &y);
    return {(float)x, (float)y};
}

/// Window size
inline vec2i get_window_size(gl_window* win, bool exclude_widgets = false) {
    auto ret = vec2i{0, 0};
    glfwGetWindowSize(win->_gwin, &ret.x, &ret.y);
    if (exclude_widgets && win->_widget_enabled) ret.x -= win->_widget_width;
    return ret;
}

/// Check if a key is pressed (not all keys are supported)
inline bool get_key(gl_window* win, int key) {
    key = std::toupper(key);
    return glfwGetKey(win->_gwin, key) == GLFW_PRESS;
}

/// Framebuffer size
inline vec2i get_framebuffer_size(
    gl_window* win, bool exclude_widgets = false) {
    auto ret = vec2i{0, 0};
    glfwGetFramebufferSize(win->_gwin, &ret.x, &ret.y);
    if (exclude_widgets && win->_widget_enabled) {
        auto ws = get_window_size(win);
        ret.x -= (win->_widget_width * ret.x) / ws.x;
    }
    return ret;
}

/// Widgets
inline int get_widget_size(gl_window* win) { return win->_widget_width; }

/// Read pixels
inline vector<vec4b> get_screenshot(
    gl_window* win, vec2i& wh, bool flipy = true, bool back = false) {
    wh = get_framebuffer_size(win);
    auto pixels = vector<vec4b>(wh[0] * wh[1]);
    glReadBuffer((back) ? GL_BACK : GL_FRONT);
    glReadPixels(0, 0, wh[0], wh[1], GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
    if (flipy) {
        vector<vec4b> line(wh[0]);
        for (int j = 0; j < wh[1] / 2; j++) {
            memcpy(line.data(), pixels.data() + j * wh[0] * 4, wh[0] * 4);
            memcpy(pixels.data() + j * wh[0] * 4,
                pixels.data() + (wh[1] - 1 - j) * wh[0] * 4, wh[0] * 4);
            memcpy(pixels.data() + (wh[1] - 1 - j) * wh[0] * 4, line.data(),
                wh[0] * 4);
        }
    }
    return pixels;
}

/// Save a screenshot to disk
inline void save_screenshot(gl_window* win, const string& imfilename) {
    auto wh = vec2i{0, 0};
    auto pixels = get_screenshot(win, wh);
    save_image(imfilename, wh.x, wh.y, 4, (unsigned char*)pixels.data());
}

#if YGL_IMGUI

/// Initialize widgets
inline void init_widgets(gl_window* win) {
    ImGui_ImplGlfwGL3_Init(win->_gwin, false);
    ImGui::GetStyle().WindowRounding = 0;
    ImGui::GetIO().IniFilename = nullptr;
    win->_widget_enabled = true;
}

/// Begin draw widget
inline bool begin_widgets(gl_window* win, const string& title) {
    ImGui_ImplGlfwGL3_NewFrame();
    auto size = get_window_size(win);
    ImGui::SetNextWindowSize({(float)win->_widget_width, (float)size[1]});
    ImGui::SetNextWindowPos({(float)(size[0] - win->_widget_width), (float)0});
    auto flags = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize;
    ImGui::Begin(title.c_str(), nullptr, flags);
    // ImGui::ShowTestWindow();
    // ImGui::ShowStyleEditor();
    return true;
}

/// End draw widget
inline void end_widgets(gl_window* win) {
    ImGui::End();
    ImGui::Render();
}

/// Whether widget are active
inline bool get_widget_active(gl_window* win) {
    if (!win->_widget_enabled) return false;
    auto io = &ImGui::GetIO();
    return io->WantTextInput || io->WantCaptureMouse || io->WantCaptureKeyboard;
}

/// Horizontal separator
inline void draw_separator_widget(gl_window* win) { ImGui::Separator(); }

/// Indent widget
inline void draw_indent_widget_begin(gl_window* win) { ImGui::Indent(); }

/// Indent widget
inline void draw_indent_widget_end(gl_window* win) { ImGui::Unindent(); }

/// Continue line with next widget
inline void draw_continue_widget(gl_window* win) { ImGui::SameLine(); }

/// Label widget
inline void draw_label_widget(
    gl_window* win, const string& lbl, const string& msg) {
    ImGui::LabelText(lbl.c_str(), "%s", msg.c_str());
}

/// Label widget
inline void draw_label_widget(
    gl_window* win, const string& lbl, const char* msg) {
    ImGui::LabelText(lbl.c_str(), "%s", msg);
}

/// Label widget
inline void draw_label_widget(
    gl_window* win, const string& lbl, int val, const std::string& fmt = "%d") {
    ImGui::LabelText(lbl.c_str(), fmt.c_str(), val);
}

/// Label widget
inline void draw_label_widget(gl_window* win, const string& lbl,
    const vec2i& val, const std::string& fmt = "[%d, %d]") {
    ImGui::LabelText(lbl.c_str(), fmt.c_str(), val[0], val[1]);
}

/// Label widget
inline void draw_label_widget(gl_window* win, const string& lbl,
    const vec3i& val, const std::string& fmt = "[%d, %d, %d]") {
    ImGui::LabelText(lbl.c_str(), fmt.c_str(), val[0], val[1], val[2]);
}

/// Label widget
inline void draw_label_widget(gl_window* win, const string& lbl,
    const vec4i& val, const std::string& fmt = "[%d, %d, %d, %d]") {
    ImGui::LabelText(lbl.c_str(), fmt.c_str(), val[0], val[1], val[2], val[3]);
}

/// Label widget
inline void draw_label_widget(gl_window* win, const string& lbl, float val,
    const std::string& fmt = "%g") {
    ImGui::LabelText(lbl.c_str(), fmt.c_str(), val);
}

/// Label widget
inline void draw_label_widget(gl_window* win, const string& lbl,
    const vec2f& val, const std::string& fmt = "[%g, %g]") {
    ImGui::LabelText(lbl.c_str(), fmt.c_str(), val[0], val[1]);
}

/// Label widget
inline void draw_label_widget(gl_window* win, const string& lbl,
    const vec3f& val, const std::string& fmt = "[%g, %g, %g]") {
    ImGui::LabelText(lbl.c_str(), fmt.c_str(), val[0], val[1], val[2]);
}

/// Label widget
inline void draw_label_widget(gl_window* win, const string& lbl,
    const vec4f& val, const std::string& fmt = "[%g, %g, %g, %g]") {
    ImGui::LabelText(lbl.c_str(), fmt.c_str(), val[0], val[1], val[2], val[3]);
}

/// Label widget
inline void draw_label_widget(gl_window* win, const string& lbl,
    const quat4f& val, const std::string& fmt = "[%g, %g, %g, %g]") {
    ImGui::LabelText(lbl.c_str(), fmt.c_str(), val[0], val[1], val[2], val[3]);
}

/// Label widget
void draw_label_widget(gl_window* win, const string& lbl, const mat4f& val,
    const std::string& fmt =
        "[[%g, %g, %g, %g], [%g, %g, %g, %g], [%g, %g, %g, %g], [%g, %g, "
        "%g, "
        "%g]]") {
    ImGui::LabelText(lbl.c_str(), fmt.c_str(), val[0][0], val[1][0], val[2][0],
        val[3][0], val[0][1], val[1][1], val[2][1], val[3][1], val[0][2],
        val[1][2], val[2][2], val[3][2], val[0][3], val[1][3], val[2][3],
        val[3][3]);
}

/// Text widget
inline bool draw_value_widget(gl_window* win, const string& lbl, string& str) {
    char buf[4096];
    if (str.length() >= 4096) throw runtime_error("bad memory");
    memcpy(buf, str.c_str(), str.size());
    buf[str.size()] = 0;
    auto ret = ImGui::InputText(lbl.c_str(), buf, 4096);
    str = buf;
    return ret;
}

/// Slider widget
inline bool draw_value_widget(gl_window* win, const string& lbl, int& val,
    int min, int max, int incr = 1) {
    return ImGui::SliderInt(lbl.c_str(), &val, min, max);
}

/// Slider widget
inline bool draw_value_widget(gl_window* win, const string& lbl, vec2i& val,
    int min, int max, int incr = 1) {
    return ImGui::SliderInt2(lbl.c_str(), (int*)&val.x, min, max);
}

/// Slider widget
inline bool draw_value_widget(gl_window* win, const string& lbl, vec3i& val,
    int min, int max, int incr = 1) {
    return ImGui::SliderInt3(lbl.c_str(), (int*)&val.x, min, max);
}

/// Slider widget
inline bool draw_value_widget(gl_window* win, const string& lbl, vec4i& val,
    int min, int max, int incr = 1) {
    return ImGui::SliderInt4(lbl.c_str(), (int*)&val.x, min, max);
}

/// Slider widget
inline bool draw_value_widget(gl_window* win, const string& lbl, float& val,
    float min, float max, float incr = 1) {
    return ImGui::SliderFloat(lbl.c_str(), (float*)&val, min, max);
}

/// Slider widget
inline bool draw_value_widget(gl_window* win, const string& lbl, vec2f& val,
    float min, float max, float incr = 1) {
    return ImGui::SliderFloat2(lbl.c_str(), (float*)&val.x, min, max);
}

/// Slider widget
inline bool draw_value_widget(gl_window* win, const string& lbl, vec3f& val,
    float min, float max, float incr = 1) {
    return ImGui::SliderFloat3(lbl.c_str(), (float*)&val.x, min, max);
}

/// Slider widget
inline bool draw_value_widget(gl_window* win, const string& lbl, vec4f& val,
    float min, float max, float incr = 1) {
    return ImGui::SliderFloat4(lbl.c_str(), (float*)&val.x, min, max);
}

/// Slider widget
inline bool draw_value_widget(gl_window* win, const string& lbl, mat4f& val,
    float min, float max, float incr = 1) {
    auto mod = false;
    for (auto i = 0; i < 4; i++) {
        auto modr = draw_value_widget(
            win, format("{}[{}]", lbl, i), val[i], min, max, incr);
        mod = mod || modr;
    }
    return mod;
}

/// Slider widget
inline bool draw_value_widget(gl_window* win, const string& lbl, frame3f& val,
    float min, float max, float incr = 1) {
    auto modx = draw_value_widget(win, lbl + ".x", val.x, min, max, incr);
    auto mody = draw_value_widget(win, lbl + ".y", val.y, min, max, incr);
    auto modz = draw_value_widget(win, lbl + ".z", val.z, min, max, incr);
    auto modo = draw_value_widget(win, lbl + ".o", val.o, min, max, incr);
    return modx || mody || modz || modo;
}

/// Slider widget
inline bool draw_value_widget(
    gl_window* win, const string& lbl, quat4f& val, float incr = 1) {
    auto mod = draw_value_widget(win, lbl, *(vec4f*)&val, -1, 1, incr);
    if (mod) val = normalize(val);
    return mod;
}

/// Slider widget
inline bool draw_color_widget(gl_window* win, const string& lbl, vec4f& val) {
    return ImGui::ColorEdit4(lbl.c_str(), (float*)&val.x);
}

/// Slider widget
inline bool draw_color_widget(gl_window* win, const string& lbl, vec4b& val) {
    auto valf = ImGui::ColorConvertU32ToFloat4(*(uint32_t*)&val);
    if (ImGui::ColorEdit4(lbl.c_str(), &valf.x)) {
        auto valb = ImGui::ColorConvertFloat4ToU32(valf);
        *(uint32_t*)&val = valb;
        return true;
    }
    return false;
}

// Support
inline bool _enum_widget_labels_ptr(void* data, int idx, const char** out) {
    auto labels = (vector<pair<string, int>>*)data;
    *out = labels->at(idx).first.c_str();
    return true;
}

// Support
inline bool _enum_widget_labels_int(void* data, int idx, const char** out) {
    auto labels = (vector<pair<string, int>>*)data;
    *out = labels->at(idx).first.c_str();
    return true;
}

/// Combo widget
inline bool draw_value_widget(gl_window* win, const string& lbl, int& val,
    const vector<pair<string, int>>& labels) {
    auto cur = -1;
    for (auto idx = 0; idx < labels.size(); idx++) {
        if (labels[idx].second == val) cur = idx;
    }
    assert(cur >= 0);
    auto ok = ImGui::Combo(lbl.c_str(), &cur, _enum_widget_labels_int,
        (void*)&labels, (int)labels.size());
    val = labels[cur].second;
    return ok;
}

/// Slider widget
inline bool list_widget(gl_window* win, const string& lbl, int& val,
    const vector<pair<string, int>>& labels) {
    auto cur = -1;
    for (auto idx = 0; idx < labels.size(); idx++) {
        if (labels[idx].second == val) cur = idx;
    }
    assert(cur >= 0);
    auto ok = ImGui::ListBox(lbl.c_str(), &cur, _enum_widget_labels_int,
        (void*)&labels, (int)labels.size());
    val = labels[cur].second;
    return ok;
}

/// Combo widget
inline bool draw_value_widget(gl_window* win, const string& lbl, void*& val,
    const vector<pair<string, void*>>& labels) {
    auto cur = -1;
    for (auto idx = 0; idx < labels.size(); idx++) {
        if (labels[idx].second == val) cur = idx;
    }
    assert(cur >= 0);
    auto ok = ImGui::Combo(lbl.c_str(), &cur, _enum_widget_labels_ptr,
        (void*)&labels, (int)labels.size());
    val = labels[cur].second;
    return ok;
}

/// Enum widget
template <typename T>
inline bool draw_value_widget(gl_window* win, const string& lbl, T& val,
    const vector<pair<string, T>>& labels) {
    return draw_value_widget(
        win, lbl, (int&)val, (const vector<pair<string, int>>&)labels);
}

/// Enum widget
template <typename T>
inline bool draw_value_widget(gl_window* win, const string& lbl, T*& val,
    const vector<pair<string, T*>>& labels) {
    return draw_value_widget(
        win, lbl, (void*&)val, (const vector<pair<string, void*>>&)labels);
}

/// Bool widget
inline bool draw_value_widget(gl_window* win, const string& lbl, bool& val) {
    return ImGui::Checkbox(lbl.c_str(), &val);
}

/// Button widget
inline bool draw_button_widget(gl_window* win, const string& lbl) {
    return ImGui::Button(lbl.c_str());
}

/// Collapsible header
inline bool draw_header_widget(gl_window* win, const string& lbl) {
    return ImGui::CollapsingHeader(lbl.c_str());
}

/// Start tree node
inline bool draw_tree_widget_begin(gl_window* win, const string& lbl) {
    return ImGui::TreeNode(lbl.c_str());
}

/// Collapsible header
inline void draw_tree_widget_end(gl_window* win) { ImGui::TreePop(); }

/// Start selectable tree node
inline bool draw_tree_widget_begin(
    gl_window* win, const string& lbl, void*& selection, void* content) {
    ImGuiTreeNodeFlags node_flags =
        ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick;
    if (selection == content) node_flags |= ImGuiTreeNodeFlags_Selected;
    auto open = ImGui::TreeNodeEx(content, node_flags, "%s", lbl.c_str());
    if (ImGui::IsItemClicked()) selection = content;
    return open;
}

/// Start selectable tree node
inline bool draw_tree_widget_begin(gl_window* win, const string& lbl,
    void*& selection, void* content, const vec4f& col) {
    ImGui::PushStyleColor(ImGuiCol_Text, {col.x, col.y, col.z, col.w});
    auto ret = draw_tree_widget_begin(win, lbl, selection, content);
    ImGui::PopStyleColor();
    return ret;
}

/// End selectable tree node
inline void draw_tree_widget_end(gl_window* win, void* content) {
    ImGui::TreePop();
}

/// Selectable tree leaf node
inline void draw_tree_widget_leaf(
    gl_window* win, const string& lbl, void*& selection, void* content) {
    ImGuiTreeNodeFlags node_flags =
        ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;
    if (selection == content) node_flags |= ImGuiTreeNodeFlags_Selected;
    ImGui::TreeNodeEx(content, node_flags, "%s", lbl.c_str());
    if (ImGui::IsItemClicked()) selection = content;
}

/// Selectable tree leaf node
inline void draw_tree_widget_leaf(gl_window* win, const string& lbl,
    void*& selection, void* content, const vec4f& col) {
    ImGui::PushStyleColor(ImGuiCol_Text, {col.x, col.y, col.z, col.w});
    draw_tree_widget_leaf(win, lbl, selection, content);
    ImGui::PopStyleColor();
}

/// Image widget
inline void draw_image_widget(
    gl_window* win, int tid, const vec2i& size, const vec2i& imsize) {
    auto w = ImGui::GetContentRegionAvailWidth();
    auto s = vec2f{(float)size.x, (float)size.y};
    auto a = (float)imsize.x / (float)imsize.y;
    if (!s.x && !s.y) {
        s.x = w;
        s.y = w / a;
    } else if (s.x && !s.y) {
        s.y = s.x / a;
    } else if (!s.x && s.y) {
        s.x = s.y * a;
    } else {
        auto as = s.x / s.y;
        if (as / a > 1) {
            s.x = s.y * a;
        } else {
            s.y = s.x / a;
        }
    }
    if (s.x > w) {
        s.x = w;
        s.y = w / a;
    }
    ImGui::Image((void*)(size_t)tid, {s.x, s.y});
}

/// Scroll region
inline void draw_scroll_widget_begin(
    gl_window* win, const string& lbl, int height, bool border) {
    ImGui::BeginChild(lbl.c_str(), ImVec2(0, height), border);
}

/// Scroll region
inline void draw_scroll_widget_end(gl_window* win) { ImGui::EndChild(); }

/// Scroll region
inline void draw_scroll_widget_here(gl_window* win) { ImGui::SetScrollHere(); }

/// Group ids
inline void draw_groupid_widget_begin(gl_window* win, int gid) {
    ImGui::PushID(gid);
}

/// Group ids
inline void draw_groupid_widget_begin(gl_window* win, void* gid) {
    ImGui::PushID(gid);
}

/// Group ids
inline void draw_groupid_widget_end(gl_window* win) { ImGui::PopID(); }

/// Text color
inline void draw_tree_widget_color_begin(gl_window* win, const vec4f& color) {
    ImGui::PushStyleColor(
        ImGuiCol_Text, {color[0], color[1], color[2], color[3]});
}

/// Text color
inline void draw_tree_widget_color_end(gl_window* win) {
    ImGui::PopStyleColor();
}

/// Tonemapping widgets
inline void draw_tonemap_widgets(gl_window* win, const string& lbl,
    float& exposure, tonemap_type& tonemap, float& gamma) {
    draw_value_widget(win, lbl + "tonemap", tonemap, tonemap_names());
    draw_value_widget(win, lbl + "exposure", exposure, -20, 20, 1);
    draw_value_widget(win, lbl + "gamma", gamma, 0.1, 5, 0.1);
}

#if YGL_SCENEUI
/// Draws a widget that can selected the camera
inline bool draw_camera_widget(
    gl_window* win, const string& lbl, scene* scn, camera*& cam) {
    auto camera_names = vector<pair<string, camera*>>{};
    for (auto cam : scn->cameras) camera_names.push_back({cam->name, cam});
    return draw_value_widget(win, lbl, cam, camera_names);
}

/// Draws widgets for a whole scene. Used for quickly making demos.
inline bool draw_scene_widgets(gl_window* win, const string& lbl, scene* scn,
    void*& selection, const unordered_map<texture*, gl_texture>& gl_txt);
#endif

#endif

#endif

#endif

}  // namespace ygl

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR PATH TRACE
// -----------------------------------------------------------------------------

namespace ygl {

namespace _impl_trace {

// Phong exponent to roughness. Public API, see above.
inline float specular_exponent_to_roughness(float n) {
    return sqrtf(2 / (n + 2));
}

// Specular to fresnel eta. Public API, see above.
inline void specular_fresnel_from_ks(const vec3f& ks, vec3f& es, vec3f& esk) {
    es = {(1 + sqrt(ks.x)) / (1 - sqrt(ks.x)),
        (1 + sqrt(ks.y)) / (1 - sqrt(ks.y)),
        (1 + sqrt(ks.z)) / (1 - sqrt(ks.z))};
    esk = {0, 0, 0};
}

// -----------------------------------------------------------------------------
// RANDOM NUMBER GENERATION
// -----------------------------------------------------------------------------

// Random number smp. Handles random number generation for stratified
// sampling and correlated multi-jittered sampling.
struct sampler {
    rng_pcg32& rng;        // random number state
    uint32_t pixel_hash;   // pixel hash
    int s, d;              // sample and dimension indices
    int ns, ns2;           // number of samples and its square root
    trace_rng_type rtype;  // random number type
};

// Initialize a smp ot type rtype for pixel i, j with ns total samples.
//
// Implementation Notes: we use hash functions to scramble the pixel ids
// to avoid introducing unwanted correlation between pixels. These should not
// around according to the RNG documentaion, but we still found bad cases.
// Scrambling avoids it.
inline sampler make_sampler(
    rng_pcg32& rng, int i, int j, int s, int ns, trace_rng_type rtype) {
    // we use various hashes to scramble the pixel values
    return {rng, hash_uint32((uint32_t)(j + 1) << 16 | (uint32_t)(i + 1)), s, 0,
        ns, (int)round(sqrt((float)ns)), rtype};
}

// Generates a 1-dimensional sample.
//
// Implementation Notes: For deterministic sampling (stratified and cmjs) we
// compute a 64bit sample and use hashing to avoid correlation. Then permutation
// are computed with CMJS procedures.
inline float sample_next1f(sampler& smp) {
    switch (smp.rtype) {
        case trace_rng_type::uniform: {
            return next_rand1f(smp.rng);
        } break;
        case trace_rng_type::stratified: {
            smp.d += 1;
            auto p = hash_uint64_32(
                (uint64_t)smp.pixel_hash | (uint64_t)smp.d << 32);
            auto s = hash_permute(smp.s, smp.ns, p);
            return (s + next_rand1f(smp.rng)) / smp.ns;
        } break;
        default: {
            assert(false);
            return 0;
        }
    }
}

// Generates a 2-dimensional sample.
//
// Implementation notes: see above. Note that using deterministic keyed
// permutaton we can use stratified sampling without preallocating samples.
inline vec2f sample_next2f(sampler& smp) {
    switch (smp.rtype) {
        case trace_rng_type::uniform: {
            return {next_rand1f(smp.rng), next_rand1f(smp.rng)};
        } break;
        case trace_rng_type::stratified: {
            smp.d += 2;
            auto p = hash_uint64_32(
                (uint64_t)smp.pixel_hash | (uint64_t)smp.d << 32);
            auto s = hash_permute(smp.s, smp.ns, p);
            return {(s % smp.ns2 + next_rand1f(smp.rng)) / smp.ns2,
                (s / smp.ns2 + next_rand1f(smp.rng)) / smp.ns2};
        } break;
        default: {
            assert(false);
            return {0, 0};
        }
    }
}

// Creates a 1-dimensional sample in [0,num-1]
inline int sample_next1i(sampler& smp, int num) {
    return clamp(int(sample_next1f(smp) * num), 0, num - 1);
}

// Brdf type
enum struct brdf_type { none = 0, microfacet = 1, kajiya_kay = 2, point = 3 };

// Brdf
struct brdf {
    brdf_type type = brdf_type::none;  // type
    vec3f kd = {0, 0, 0};              // diffuse
    vec3f ks = {0, 0, 0};              // specular
    float rs = 0;                      // specular roughness
    vec3f kt = {0, 0, 0};              // transmission (thin glass)
    operator bool() const { return type != brdf_type::none; }
    vec3f rho() const { return kd + ks + kt; }
};

// Emission type
enum struct emission_type {
    none = 0,
    diffuse = 1,
    point = 2,
    line = 3,
    env = 4,
};

// Emission
struct emission {
    emission_type type = emission_type::none;
    vec3f ke = zero3f;
    operator bool() const { return type != emission_type::none; }
};

// Surface point with geometry and material data. Supports point on envmap too.
// This is the key data manipulated in the path tracer.
struct point {
    const instance* ist = nullptr;     // instance
    const environment* env = nullptr;  // environment
    frame3f frame = identity_frame3f;  // local frame
    vec3f wo = zero3f;                 // outgoing direction
    emission em = {};                  // emission
    brdf fr = {};                      // brdf
};

// Generates a ray ray_o, ray_d from a camera cam for image plane coordinate
// uv and the lens coordinates luv.
inline ray3f eval_camera(const camera* cam, const vec2f& uv, const vec2f& luv) {
    auto h = 2 * tan(cam->yfov / 2);
    auto w = h * cam->aspect;
    auto o = vec3f{luv.x * cam->aperture, luv.y * cam->aperture, 0};
    auto q = vec3f{w * cam->focus * (uv.x - 0.5f),
        h * cam->focus * (uv.y - 0.5f), -cam->focus};
    return ray3f(transform_point(cam->frame, o),
        transform_direction(cam->frame, normalize(q - o)));
}

// Evaluates emission.
inline vec3f eval_emission(const point& pt) {
    auto& em = pt.em;
    auto& wo = pt.wo;
    auto& wn = pt.frame.z;

    if (!em) return zero3f;
    auto ke = zero3f;
    switch (em.type) {
        case emission_type::diffuse:
            ke += (dot(wn, wo) > 0) ? em.ke : zero3f;
            break;
        case emission_type::point: ke += em.ke; break;
        case emission_type::line: ke += em.ke; break;
        case emission_type::env: ke += em.ke; break;
        default: assert(false); break;
    }
    return ke;
}

// Compute the fresnel term for dielectrics. Implementation from
// https://seblagarde.wordpress.com/2013/04/29/memo-on-fresnel-equations/
inline vec3f eval_fresnel_dielectric(float cosw, const vec3f& eta_) {
    auto eta = eta_;
    if (cosw < 0) {
        eta = 1.0f / eta;
        cosw = -cosw;
    }

    auto sin2 = 1 - cosw * cosw;
    auto eta2 = eta * eta;

    auto cos2t = vec3f{1, 1, 1} - sin2 / eta2;
    if (cos2t.x < 0 || cos2t.y < 0 || cos2t.z < 0)
        return vec3f{1, 1, 1};  // tir

    auto t0 = vec3f{sqrt(cos2t.x), sqrt(cos2t.y), sqrt(cos2t.z)};
    auto t1 = eta * t0;
    auto t2 = eta * cosw;

    auto rs = (vec3f{cosw, cosw, cosw} - t1) / (vec3f{cosw, cosw, cosw} + t1);
    auto rp = (t0 - t2) / (t0 + t2);

    return (rs * rs + rp * rp) / 2.0f;
}

// Compute the fresnel term for metals. Implementation from
// https://seblagarde.wordpress.com/2013/04/29/memo-on-fresnel-equations/
inline vec3f eval_fresnel_metal(
    float cosw, const vec3f& eta, const vec3f& etak) {
    if (etak == zero3f) return eval_fresnel_dielectric(cosw, eta);

    cosw = clamp(cosw, (float)-1, (float)1);
    auto cos2 = cosw * cosw;
    auto sin2 = clamp(1 - cos2, (float)0, (float)1);
    auto eta2 = eta * eta;
    auto etak2 = etak * etak;

    auto t0 = eta2 - etak2 - vec3f{sin2, sin2, sin2};
    auto a2plusb2_2 = t0 * t0 + 4.0f * eta2 * etak2;
    auto a2plusb2 =
        vec3f{sqrt(a2plusb2_2.x), sqrt(a2plusb2_2.y), sqrt(a2plusb2_2.z)};
    auto t1 = a2plusb2 + vec3f{cos2, cos2, cos2};
    auto a_2 = (a2plusb2 + t0) / 2.0f;
    auto a = vec3f{sqrt(a_2.x), sqrt(a_2.y), sqrt(a_2.z)};
    auto t2 = 2.0f * a * cosw;
    auto rs = (t1 - t2) / (t1 + t2);

    auto t3 = vec3f{cos2, cos2, cos2} * a2plusb2 +
              vec3f{sin2, sin2, sin2} * vec3f{sin2, sin2, sin2};
    auto t4 = t2 * sin2;
    auto rp = rs * (t3 - t4) / (t3 + t4);

    return (rp + rs) / 2.0f;
}

// Schlick approximation of Fresnel term
inline vec3f eval_fresnel_schlick(const vec3f& ks, float cosw) {
    return ks +
           (vec3f{1, 1, 1} - ks) * pow(clamp(1.0f - cosw, 0.0f, 1.0f), 5.0f);
}

// Schlick approximation of Fresnel term weighted by roughness.
// This is a hack, but works better than not doing it.
inline vec3f eval_fresnel_schlick(const vec3f& ks, float cosw, float rs) {
    auto fks = eval_fresnel_schlick(ks, cosw);
    return lerp(ks, fks, rs);
}

// Evaluates the GGX distribution and geometric term
inline float eval_ggx(float rs, float ndh, float ndi, float ndo) {
    // evaluate GGX
    auto alpha2 = rs * rs;
    auto di = (ndh * ndh) * (alpha2 - 1) + 1;
    auto d = alpha2 / (pif * di * di);
#ifndef YTRACE_GGX_SMITH
    auto lambda_o = (-1 + sqrt(1 + alpha2 * (1 - ndo * ndo) / (ndo * ndo))) / 2;
    auto lambda_i = (-1 + sqrt(1 + alpha2 * (1 - ndi * ndi) / (ndi * ndi))) / 2;
    auto g = 1 / (1 + lambda_o + lambda_i);
#else
    auto go = (2 * ndo) / (ndo + sqrt(alpha2 + (1 - alpha2) * ndo * ndo));
    auto gi = (2 * ndi) / (ndi + sqrt(alpha2 + (1 - alpha2) * ndi * ndi));
    auto g = go * gi;
#endif
    return d * g;
}

// Evaluates the GGX pdf
inline float pdf_ggx(float rs, float ndh) {
    auto cos2 = ndh * ndh;
    auto tan2 = (1 - cos2) / cos2;
    auto alpha2 = rs * rs;
    auto d = alpha2 / (pif * cos2 * cos2 * (alpha2 + tan2) * (alpha2 + tan2));
    return d;
}

// Sample the GGX distribution
inline vec3f sample_ggx(float rs, const vec2f& rn) {
    auto tan2 = rs * rs * rn.y / (1 - rn.y);
    auto rz = sqrt(1 / (tan2 + 1)), rr = sqrt(1 - rz * rz),
         rphi = 2 * pif * rn.x;
    // set to wh
    auto wh_local = vec3f{rr * cos(rphi), rr * sin(rphi), rz};
    return wh_local;
}

// Evaluates the BRDF scaled by the cosine of the incoming direction.
//
// Implementation notes:
// - ggx from [Heitz 2014] and [Walter 2007] and [Lagarde 2014]
// "Understanding the Masking-Shadowing Function in Microfacet-Based BRDFs"
// http://jcgt.org/published/0003/02/03/
// - "Microfacet Models for Refraction through Rough Surfaces" EGSR 07
// https://www.cs.cornell.edu/~srm/publications/EGSR07-btdf.pdf
// - uses Kajiya-Kay for hair
// - uses a hack for points
inline vec3f eval_brdfcos(const point& pt, const vec3f& wi) {
    // grab variables
    auto& fr = pt.fr;
    auto& wn = pt.frame.z;
    auto& wo = pt.wo;

    // exit if not needed
    if (!fr) return zero3f;

    // accumulate brdfcos for each lobe
    auto brdfcos = zero3f;
    switch (fr.type) {
        // reflection terms
        case brdf_type::microfacet: {
            // compute wh
            auto wh = normalize(wo + wi);

            // compute dot products
            auto ndo = dot(wn, wo), ndi = dot(wn, wi),
                 ndh = clamp(dot(wh, wn), (float)-1, (float)1);

            // diffuse term
            if (ndi > 0 && ndo > 0) { brdfcos += fr.kd * ndi / pif; }

            // specular term
            if (ndi > 0 && ndo > 0 && ndh > 0) {
                // microfacet term
                auto dg = eval_ggx(fr.rs, ndh, ndi, ndo);

                // handle fresnel
                auto odh = clamp(dot(wo, wh), 0.0f, 1.0f);
                auto ks = eval_fresnel_schlick(fr.ks, odh, fr.rs);

                // sum up
                brdfcos += ks * ndi * dg / (4 * ndi * ndo);
            }

            // transmission hack
            if (wo == -wi) brdfcos += fr.kt;
        } break;
#if 0
                    // transmission terms
                case brdf_type::transmission_lambert:
                case brdf_type::transmission_ggx: {
                    // compute dot products
                    auto ndo = dot(wn, wo), ndi = dot(wn, wi);

                    // exit if needed
                    if (ndi >= 0 || ndo <= 0) continue;

                    // flip direction
                    ndi = -ndi;
                    auto wi_ = -wi;

                    // diffuse term
                    if (fl.type == brdf_type::reflection_lambert) {
                        brdfcos += weight * fl.rho * ndi / pif;
                    }
                    // specular term
                    else {
                        // compute wh
                        auto wh = normalize(wo + wi_);

                        // compute dot products
                        auto ndh = clamp(dot(wh, wn), (float)-1, (float)1);

                        // exit if needed
                        if (ndh <= 0) continue;

                        // microfacet term
                        auto dg = eval_ggx(fl.roughness, ndh, ndi, ndo);

                        // handle fresnel
                        auto odh = clamp(dot(wo, wh), 0.0f, 1.0f);
                        auto ks = eval_fresnel_schlick(fl.rho, odh, fl.roughness);

                        // sum up
                        brdfcos += weight * ks * ndi * dg / (4 * ndi * ndo);
                    }
                } break;
                    // refraction term (GGX)
                case brdf_type::refraction_ggx: {
                    // compute dot products
                    auto ndo = dot(wn, wo), ndi = dot(wn, wi);

                    // HACK
                    // eta
                    auto eta = 1.4f;

                    // HACK
                    auto kt = fl.rho;

                    // exit if needed
                    if (ndi * ndo >= 0) continue;

                    // flip eta if necessary
                    if (ndo < 0) eta = 1 / eta;

                    // compute wh
                    auto wh = normalize(wo + eta * wi);

                    // flip from pbrt
                    if (dot(wn, wh) < 0) wh = -wh;

                    // compute dot products
                    auto ndh = clamp(dot(wh, wn), (float)-1, (float)1),
                    odh = dot(wh, wo), idh = dot(wh, wi);

                    // microfacet term
                    auto dg = eval_ggx(fl.roughness, abs(ndh), abs(ndi), abs(ndo));

                    // fresnel
                    auto f = 1 - eval_fresnel_schlick({0.04f, 0.04f, 0.04f}, ndh).x;

                    // sum up
                    brdfcos += weight * kt * abs(ndi) *
                    (abs(idh) * abs(odh) * f * dg) /
                    (abs(ndi) * abs(ndo) * (eta * idh + odh) *
                     (eta * idh + odh));

                } break;
#endif
        // hair (Kajiya-Kay)
        case brdf_type::kajiya_kay: {
            // compute wh
            auto wh = normalize(wo + wi);

            // compute dot products
            auto ndo = dot(wn, wo), ndi = dot(wn, wi),
                 ndh = clamp(dot(wh, wn), (float)0, (float)1);

            // take sines
            auto so = sqrt(clamp(1 - ndo * ndo, (float)0, (float)1)),
                 si = sqrt(clamp(1 - ndi * ndi, (float)0, (float)1)),
                 sh = sqrt(clamp(1 - ndh * ndh, (float)0, (float)1));

            // exit if needed

            // diffuse term (Kajiya-Kay)
            if (si > 0 && so > 0) { brdfcos += fr.kd * si / pif; }

            // specular term (Kajiya-Kay)
            if (si > 0 && so > 0 && sh > 0) {
                auto ns = 2 / (fr.rs * fr.rs) - 2;
                auto d = (ns + 2) * pow(sh, ns) / (2 + pif);
                brdfcos += fr.ks * si * d / (4.0f * si * so);
            }

            // transmission hack
            if (wo == -wi) brdfcos += fr.kt;
        } break;
        // points
        case brdf_type::point: {
            // diffuse term
            auto ido = dot(wo, wi);
            brdfcos += fr.kd * (2 * ido + 1) / (2 * pif);

            // transmission hack
            if (wo == -wi) brdfcos += fr.kt;
        } break;
        default: assert(false); break;
    }

    // check
    assert(isfinite(brdfcos));

    // done
    return brdfcos;
}

// Compute the weight for sampling the BRDF
inline float weight_brdfcos(const point& pt, const vec3f& wi) {
    // grab variables
    auto& fr = pt.fr;
    auto& wn = pt.frame.z;
    auto& wo = pt.wo;

    // skip if no component
    if (!fr) return 0;

    // probability of each lobe
    auto kdw = max_element_val(fr.kd), ksw = max_element_val(fr.ks),
         ktw = max_element_val(fr.kt);
    auto kaw = kdw + ksw + ktw;
    kdw /= kaw;
    ksw /= kaw;
    ktw /= kaw;

    // accumulate the probability over all lobes
    auto pdf = 0.0f;
    // sample the lobe
    switch (fr.type) {
        // reflection term
        case brdf_type::microfacet: {
            // compute wh
            auto wh = normalize(wi + wo);

            // compute dot products
            auto ndo = dot(wn, wo), ndi = dot(wn, wi), ndh = dot(wn, wh);

            // diffuse term (hemipherical cosine probability)
            if (ndo > 0 && ndi > 0) { pdf += kdw * ndi / pif; }

            // specular term (GGX)
            if (ndo > 0 && ndi > 0 && ndh > 0) {
                // probability proportional to d adjusted by wh projection
                auto d = pdf_ggx(fr.rs, ndh);
                auto hdo = dot(wo, wh);
                pdf += ksw * d / (4 * hdo);
            }

            // transmission hack
            if (wi == -wo) pdf += ktw;
        } break;
#if 0
                    // transmission term
                case brdf_type::transmission_lambert:
                case brdf_type::transmission_ggx: {
                    // compute dot products
                    auto ndo = dot(wn, wo), ndi = dot(wn, wi);

                    // check to make sure we are above the surface
                    if (ndo <= 0 || ndi >= 0) continue;

                    // flip
                    ndi = -ndi;
                    auto wi_ = -wi;

                    // diffuse term
                    if (fl.type == brdf_type::reflection_lambert) {
                        // hemipherical cosine probability
                        pdf += weights[lid] * ndi / pif;
                    }
                    // specular term (GGX)
                    else {
                        // compute wh
                        auto wh = normalize(wi_ + wo);

                        // compute dot products
                        auto ndh = dot(wn, wh);

                        // check to make sure we are above the surface
                        if (ndh <= 0) continue;

                        // specular term (GGX)
                        // probability proportional to d adjusted by wh projection
                        auto d = pdf_ggx(fl.roughness, ndh);
                        auto hdo = dot(wo, wh);
                        pdf += weights[lid] * d / (4 * hdo);
                    }
                } break;
                    // refraction term (GGX)
                case brdf_type::refraction_ggx: {
                    // compute dot products
                    auto ndo = dot(wn, wo), ndi = dot(wn, wi);

                    // exit if needed
                    if (ndi * ndo >= 0) continue;

                    // HACK
                    // eta
                    auto eta = 1.4f;

                    // flip eta if necessary
                    if (ndo < 0) eta = 1 / eta;

                    // compute wh
                    auto wh = normalize(wo + eta * wi);

                    // compute dot products
                    auto ndh = clamp(dot(wh, wn), (float)-1, (float)1),
                    odh = dot(wh, wo), idh = dot(wh, wi);

                    // specular term (GGX)
                    // probability proportional to d weighted by change of variable
                    auto d = pdf_ggx(fl.roughness, abs(ndh));
                    // pdf += weights[lid] * d * eta * eta * abs(idh) /
                    //        ((odh + eta * idh) * (odh + eta * idh));

                    static vec2f acc = zero2f;
                    static vec2i count = zero2i;

                    auto x =
                    d * abs(idh) / ((odh + eta * idh) * (odh + eta * idh));
                    auto idx = (ndo < 0) ? 0 : 1;
                    acc[idx] += x;
                    count[idx] += 1;
                    pdf += weights[lid] * d * abs(idh) /
                    ((odh + eta * idh) * (odh + eta * idh));

                    // check
                    assert(isfinite(pdf));
                } break;
#endif
        // hair (Kajiya-Kay)
        case brdf_type::kajiya_kay: {
            // diffuse and specular
            pdf += (kdw + ksw) * 4 * pif;
            // transmission hack
            if (wi == -wo) pdf += ktw;
        } break;
        // point
        case brdf_type::point: {
            // diffuse and specular
            pdf += (kdw + ksw) * 4 * pif;
            // transmission hack
            if (wi == -wo) pdf += ktw;
        } break;
        default: assert(false); break;
    }

    // check for missed pdf
    if (!pdf) return 0;

    // check
    assert(isfinite(pdf));

    // done
    return 1 / pdf;
}

// reflected vector
inline vec3f reflect(const vec3f& w, const vec3f& n) {
    return -w + 2 * dot(n, w) * n;
}

// refracted vector
inline vec3f refract(const vec3f& w, const vec3f& n, float eta) {
    // auto k = 1.0 - eta * eta * (1.0 - dot(n, w) * dot(n, w));
    auto k = 1 - eta * eta * max(0.0f, 1 - dot(n, w) * dot(n, w));
    if (k < 0) return zero3f;  // tir
    return -w * eta + (eta * dot(n, w) - sqrt(k)) * n;
}

// Picks a direction based on the BRDF
inline vec3f sample_brdfcos(const point& pt, float rnl, const vec2f& rn) {
    // grab variables
    auto& fr = pt.fr;
    auto& wn = pt.frame.z;
    auto& fp = pt.frame;
    auto& wo = pt.wo;

    // skip if no component
    if (!fr) return zero3f;

    // probability of each lobe
    auto kdw = max_element_val(fr.kd), ksw = max_element_val(fr.ks),
         ktw = max_element_val(fr.kt);
    auto kaw = kdw + ksw + ktw;
    kdw /= kaw;
    ksw /= kaw;
    ktw /= kaw;

    // sample selected lobe
    switch (fr.type) {
        // reflection term
        case brdf_type::microfacet: {
            // compute cosine
            auto ndo = dot(wn, wo);

            // check to make sure we are above the surface
            if (ndo <= 0) return zero3f;

            // sample according to diffuse
            if (rnl < kdw) {
                // sample wi with hemispherical cosine distribution
                auto rz = sqrtf(rn.y), rr = sqrtf(1 - rz * rz),
                     rphi = 2 * pif * rn.x;
                // set to wi
                auto wi_local = vec3f{rr * cosf(rphi), rr * sinf(rphi), rz};
                return transform_direction(fp, wi_local);
            }
            // sample according to specular GGX
            else if (rnl < kdw + ksw) {
                // sample wh with ggx distribution
                auto wh_local = sample_ggx(fr.rs, rn);
                auto wh = transform_direction(fp, wh_local);
                // compute wi
                return normalize(wh * 2.0f * dot(wo, wh) - wo);
            }
            // transmission hack
            else if (rnl < kdw + ksw + ktw) {
                // continue ray direction
                return -wo;
            } else
                assert(false);
        } break;
#if 0
                    // tranbsmission term
                case brdf_type::transmission_lambert:
                case brdf_type::transmission_ggx: {
                    // compute cosine
                    auto ndo = dot(wn, wo);

                    // check to make sure we are above the surface
                    if (ndo <= 0) return zero3f;

                    // sample according to diffuse
                    if (fl.type == brdf_type::reflection_lambert) {
                        // sample wi with hemispherical cosine distribution
                        auto rz = sqrtf(rn.y), rr = sqrtf(1 - rz * rz),
                        rphi = 2 * pif * rn.x;
                        // set to wi
                        auto wi_local = vec3f{rr * cosf(rphi), rr * sinf(rphi), rz};
                        return -transform_direction(fp, wi_local);
                    }
                    // sample according to specular GGX
                    else {
                        // sample wh with ggx distribution
                        auto wh_local = sample_ggx(fl.roughness, rn);
                        auto wh = transform_direction(fp, wh_local);
                        // compute wi
                        return -normalize(wh * 2.0f * dot(wo, wh) - wo);
                    }
                } break;
                    // transmission term (GGX)
                case brdf_type::refraction_ggx: {
                    // compute cosine
                    auto ndo = dot(wn, wo);

                    // HACK
                    // eta
                    auto eta = 1.4f;

                    // flip eta if necessary
                    if (ndo < 0) eta = 1 / eta;

                    // sample according to specular (GGX or Phong)
                    // sample wh with ggx distribution
                    auto wh_local = sample_ggx(fl.roughness, rn);
                    auto wh = transform_direction(fp, wh_local);

                    // wi
                    auto e = 1 / eta;
                    auto wi = zero3f;
                    auto odh = dot(wn, wo);
                    auto k = 1 - e * e * std::max(0.0f, 1 - odh * odh);
                    if (k < 0)
                        wi = zero3f;  // tir
                    else if (ndo < 0) {
                        wi = normalize(-wo * e + (e * odh + std::sqrt(k)) * wh);
                        assert(dot(wn, wi) * ndo <= 0);
                    } else {
                        wi = normalize(-wo * e + (e * odh - std::sqrt(k)) * wh);
                        assert(dot(wn, wi) * ndo <= 0);
                    }

                    // check
                    assert(isfinite(wi));

                    // done
                    return wi;
                } break;
#endif
        // hair (Kajiya-Kay)
        case brdf_type::kajiya_kay: {
            // diffuse and specular
            if (rnl < kdw + ksw) {
                // sample wi with uniform spherical distribution
                auto rz = 2 * rn.y - 1, rr = sqrtf(1 - rz * rz),
                     rphi = 2 * pif * rn.x;
                auto wi_local = vec3f{rr * cosf(rphi), rr * sinf(rphi), rz};
                return transform_direction(fp, wi_local);
            }
            // transmission hack
            else if (rnl < kdw + ksw + ktw) {
                // continue ray direction
                return -wo;
            } else
                assert(false);
        } break;
        // diffuse term point
        case brdf_type::point: {
            // diffuse and specular
            if (rnl < kdw + ksw) {
                // sample wi with uniform spherical distribution
                auto rz = 2 * rn.y - 1, rr = sqrtf(1 - rz * rz),
                     rphi = 2 * pif * rn.x;
                auto wi_local = vec3f{rr * cosf(rphi), rr * sinf(rphi), rz};
                return transform_direction(fp, wi_local);
            }
            // transmission hack
            else if (rnl < kdw + ksw + ktw) {
                // continue ray direction
                return -wo;
            } else
                assert(false);
        } break;
        default: assert(false); break;
    }

    // done
    return zero3f;
}

// Create a point for an environment map. Resolves material with textures.
inline point eval_envpoint(const environment* env, const vec3f& wo) {
    // set shape data
    auto pt = point();

    // env
    pt.env = env;

    // direction
    pt.wo = wo;

    // maerial
    auto ke = env->ke;
    if (env->ke_txt) {
        auto w = transform_direction(inverse(env->frame), -wo);
        auto theta = (acos(clamp(w.y, (float)-1, (float)1)) / pif);
        auto phi = atan2(w.z, w.x) / (2 * pif);
        auto texcoord = vec2f{phi, theta};
        ke *= eval_texture(env->ke_txt, texcoord).xyz();
    }

    // create emission lobe
    if (ke != zero3f) { pt.em = {emission_type::env, ke}; }

    // done
    return pt;
}

// Create a point for a shape. Resolves geometry and material with textures.
inline point eval_shapepoint(
    const instance* ist, int eid, const vec4f& euv, const vec3f& wo) {
    // set shape data
    auto pt = point();

    // instance
    pt.ist = ist;

    // direction
    pt.wo = wo;

    // shortcuts
    auto shp = ist->shp;
    auto mat = ist->shp->mat;

    // compute points and weights
    auto pos = eval_pos(ist->shp, eid, euv);
    auto norm = eval_norm(ist->shp, eid, euv);
    auto texcoord = eval_texcoord(ist->shp, eid, euv);
    auto color = eval_color(ist->shp, eid, euv);

    // handle normal map
    if (mat->norm_txt) {
        auto tangsp = eval_tangsp(ist->shp, eid, euv);
        auto txt = eval_texture(mat->norm_txt, texcoord, false).xyz() * 2.0f -
                   vec3f{1, 1, 1};
        auto ntxt = normalize(vec3f{txt.x, -txt.y, txt.z});
        auto frame =
            make_frame3_fromzx({0, 0, 0}, norm, {tangsp.x, tangsp.y, tangsp.z});
        frame.y *= tangsp.w;
        norm = transform_direction(frame, ntxt);
    }

    // correct for double sided
    if (mat->double_sided && dot(norm, wo) < 0) norm = -norm;

    // creating frame
    pt.frame = make_frame3_fromz(transform_point(ist->frame, pos),
        transform_direction(ist->frame, norm));

    // handle color
    auto kx_scale = vec4f{1, 1, 1, 1};
    if (!shp->color.empty()) kx_scale *= color;

    // handle occlusion
    if (mat->occ_txt)
        kx_scale.xyz() *= eval_texture(mat->occ_txt, texcoord).xyz();

    // sample emission
    auto ke = mat->ke * kx_scale.xyz();

    // sample reflectance
    auto kd = zero4f, ks = zero4f, kt = zero4f;
    switch (mat->mtype) {
        case material_type::specular_roughness: {
            pt.fr.type = brdf_type::microfacet;
            kd = vec4f{mat->kd, mat->op} * kx_scale *
                 eval_texture(mat->kd_txt, texcoord);
            ks = vec4f{mat->ks, mat->rs} * vec4f{kx_scale.xyz(), 1} *
                 eval_texture(mat->ks_txt, texcoord);
            kt = vec4f{mat->kt, mat->rs} * vec4f{kx_scale.xyz(), 1} *
                 eval_texture(mat->kt_txt, texcoord);
        } break;
        case material_type::metallic_roughness: {
            pt.fr.type = brdf_type::microfacet;
            auto kb = vec4f{mat->kd, mat->op} * kx_scale *
                      eval_texture(mat->kd_txt, texcoord);
            auto km = vec2f{mat->ks.x, mat->rs};
            if (mat->ks_txt) {
                auto ks_txt = eval_texture(mat->ks_txt, texcoord);
                km.x *= ks_txt.y;
                km.y *= ks_txt.z;
            }
            kd = vec4f{kb.xyz() * (1 - km.x), kb.w};
            ks =
                vec4f{kb.xyz() * km.x + vec3f{0.04f, 0.04f, 0.04f} * (1 - km.x),
                    km.y};
        } break;
        case material_type::specular_glossiness: {
            pt.fr.type = brdf_type::microfacet;
            kd = vec4f{mat->kd, mat->op} * kx_scale *
                 eval_texture(mat->kd_txt, texcoord);
            ks = vec4f{mat->ks, mat->rs} * vec4f{kx_scale.xyz(), 1} *
                 eval_texture(mat->ks_txt, texcoord);
            ks.w = 1 - ks.w;  // glossiness -> roughness
        } break;
#if 0
                case reflectance_type::thin_glass: {
                    auto ks = vec4f{mat->ks, mat->rs} *
                    vec4f{kx_scale.xyz(), 1};
                    auto kt = vec4f{mat->kt, mat->rs} *
                    vec4f{kx_scale.xyz(), 1};
                    if (shp->texcoord && mat->ks_txt)
                        ks.xyz() *=
                        eval_texture(mat->ks_txt, texcoord).xyz();
                    if (shp->texcoord && mat->kt_txt)
                        kt.xyz() *=
                        eval_texture(mat->kt_txt, texcoord).xyz();
                    pt.op = 1;
                    pt.fr.fls[pt.nbrdfs].type = brdf_type::transparent;
                    pt.fr.fls[pt.nbrdfs].rho = kt.xyz();
                    pt.fr.fls[pt.nbrdfs].roughness = 0;
                    if (pt.fr.fls[pt.nbrdfs].rho != zero3f) pt.nbrdfs++;
                    pt.fr.fls[pt.nbrdfs].type = brdf_type::reflection_ggx;
                    pt.fr.fls[pt.nbrdfs].rho = ks.xyz();
                    pt.fr.fls[pt.nbrdfs].roughness = ks.w;
                    if (pt.fr.fls[pt.nbrdfs].rho != zero3f) pt.nbrdfs++;
                } break;
#endif
    }

    // set up final values
    pt.em.ke = ke * kd.w;
    pt.fr.kd = kd.xyz() * kd.w;
    pt.fr.ks =
        (ks.xyz() != zero3f && ks.w < 0.9999f) ? ks.xyz() * kd.w : zero3f;
    pt.fr.rs = (ks.xyz() != zero3f && ks.w < 0.9999f) ? ks.w * ks.w : 0;
    pt.fr.kt = {1 - kd.w, 1 - kd.w, 1 - kd.w};
    if (kt.xyz() != zero3f) pt.fr.kt *= kt.xyz();

    // setup brdf and emission
    if (!shp->points.empty()) {
        if (kd.xyz() != zero3f && ks.xyz() != zero3f && kt.xyz() != zero3f)
            pt.fr.type = brdf_type::point;
        if (ke != zero3f) pt.em.type = emission_type::point;
    } else if (!shp->lines.empty()) {
        if (kd.xyz() != zero3f && ks.xyz() != zero3f && kt.xyz() != zero3f)
            pt.fr.type = brdf_type::kajiya_kay;
        if (ke != zero3f) pt.em.type = emission_type::line;
    } else if (!shp->triangles.empty()) {
        if (kd.xyz() != zero3f && ks.xyz() != zero3f && kt.xyz() != zero3f)
            pt.fr.type = brdf_type::microfacet;
        if (ke != zero3f) pt.em.type = emission_type::diffuse;
    }

    // done
    return pt;
}

// Sample weight for a light point.
inline float weight_light(const point& lpt, const point& pt) {
    if (!lpt.em) return 0;
    // support only one lobe for now
    switch (lpt.em.type) {
        case emission_type::env: {
            return 4 * pif;
        } break;
        case emission_type::point: {
            auto d = dist(lpt.frame.o, pt.frame.o);
            return lpt.ist->shp->elem_cdf.back() / (d * d);
        } break;
        case emission_type::line: {
            assert(false);
            return 0;
        } break;
        case emission_type::diffuse: {
            auto d = dist(lpt.frame.o, pt.frame.o);
            return lpt.ist->shp->elem_cdf.back() *
                   abs(dot(lpt.frame.z, lpt.wo)) / (d * d);
        } break;
        default: {
            assert(false);
            return 0;
        } break;
    }
}

// Picks a point on a light.
inline point sample_light(
    const light* lgt, const point& pt, float rne, const vec2f& rn) {
    if (lgt->ist) {
        auto shp = lgt->ist->shp;
        auto eid = 0;
        auto euv = zero4f;
        if (!shp->triangles.empty()) {
            std::tie(eid, (vec3f&)euv) =
                sample_triangles(shp->elem_cdf, rne, rn);
        } else if (!shp->quads.empty()) {
            std::tie(eid, (vec4f&)euv) = sample_quads(shp->elem_cdf, rne, rn);
        } else if (!shp->lines.empty()) {
            std::tie(eid, (vec2f&)euv) = sample_lines(shp->elem_cdf, rne, rn.x);
        } else if (!shp->points.empty()) {
            eid = sample_points(shp->elem_cdf, rne);
            euv = {1, 0, 0, 0};
        } else {
            assert(false);
        }
        auto lpt = eval_shapepoint(lgt->ist, eid, euv, zero3f);
        lpt.wo = normalize(pt.frame.o - lpt.frame.o);
        return lpt;
    } else if (lgt->env) {
        auto z = -1 + 2 * rn.y;
        auto rr = sqrt(clamp(1 - z * z, (float)0, (float)1));
        auto phi = 2 * pif * rn.x;
        auto wo = vec3f{cos(phi) * rr, z, sin(phi) * rr};
        auto lpt = eval_envpoint(lgt->env, wo);
        return lpt;
    } else {
        assert(false);
        return {};
    }
}

// Offsets a ray origin to avoid self-intersection.
inline ray3f offset_ray(
    const point& pt, const vec3f& w, const trace_params& params) {
    if (dot(w, pt.frame.z) > 0) {
        return ray3f(
            pt.frame.o + pt.frame.z * params.ray_eps, w, params.ray_eps);
    } else {
        return ray3f(
            pt.frame.o - pt.frame.z * params.ray_eps, w, params.ray_eps);
    }
}

// Offsets a ray origin to avoid self-intersection.
inline ray3f offset_ray(
    const point& pt, const point& pt2, const trace_params& params) {
    auto ray_dist = (!pt2.env) ? dist(pt.frame.o, pt2.frame.o) : flt_max;
    if (dot(pt2.frame.o - pt.frame.o, pt.frame.z) > 0) {
        return ray3f(pt.frame.o + pt.frame.z * params.ray_eps, -pt2.wo,
            params.ray_eps, ray_dist - 2 * params.ray_eps);
    } else {
        return ray3f(pt.frame.o - pt.frame.z * params.ray_eps, -pt2.wo,
            params.ray_eps, ray_dist - 2 * params.ray_eps);
    }
}

// Intersects a ray with the scn and return the point (or env
// point).
inline point intersect_scene(const scene* scn, const ray3f& ray) {
    auto iid = 0, eid = 0;
    auto euv = zero4f;
    auto ray_t = 0.0f;
    if (intersect_ray(scn, ray, false, ray_t, iid, eid, euv)) {
        return eval_shapepoint(scn->instances[iid], eid, euv, -ray.d);
    } else if (!scn->environments.empty()) {
        return eval_envpoint(scn->environments[0], -ray.d);
    } else {
        return {};
    }
}

// Test occlusion
inline vec3f eval_transmission(const scene* scn, const point& pt,
    const point& lpt, const trace_params& params) {
    if (params.shadow_notransmission) {
        auto shadow_ray = offset_ray(pt, lpt, params);
        return (intersect_ray(scn, shadow_ray, true)) ? zero3f : vec3f{1, 1, 1};
    } else {
        auto cpt = pt;
        auto weight = vec3f{1, 1, 1};
        for (auto bounce = 0; bounce < params.max_depth; bounce++) {
            cpt = intersect_scene(scn, offset_ray(cpt, lpt, params));
            if (!cpt.ist) break;
            weight *= cpt.fr.kt;
            if (weight == zero3f) break;
        }
        return weight;
    }
}

// Mis weight
inline float weight_mis(float w0, float w1) {
    if (!w0 || !w1) return 1;
    return (1 / w0) / (1 / w0 + 1 / w1);
}

// Recursive path tracing.
inline vec3f eval_li_pathtrace(const scene* scn, const ray3f& ray, sampler& smp,
    const trace_params& params, bool& hit) {
    // intersection
    auto pt = intersect_scene(scn, ray);
    hit = pt.ist;

    // emission
    auto l = eval_emission(pt);
    if (!pt.fr || scn->lights.empty()) return l;

    // trace path
    auto weight = vec3f{1, 1, 1};
    auto emission = false;
    for (auto bounce = 0; bounce < params.max_depth; bounce++) {
        // emission
        if (emission) l += weight * eval_emission(pt);

        // direct – light
        auto lgt = scn->lights[sample_next1i(smp, (int)scn->lights.size())];
        auto lpt =
            sample_light(lgt, pt, sample_next1f(smp), sample_next2f(smp));
        auto lw = weight_light(lpt, pt) * (float)scn->lights.size();
        auto lke = eval_emission(lpt);
        auto lbc = eval_brdfcos(pt, -lpt.wo);
        auto lld = lke * lbc * lw;
        if (lld != zero3f) {
            l += weight * lld * eval_transmission(scn, pt, lpt, params) *
                 weight_mis(lw, weight_brdfcos(pt, -lpt.wo));
        }

        // direct – brdf
        auto bpt = intersect_scene(
            scn, offset_ray(pt,
                     sample_brdfcos(pt, sample_next1f(smp), sample_next2f(smp)),
                     params));
        auto bw = weight_brdfcos(pt, -bpt.wo);
        auto bke = eval_emission(bpt);
        auto bbc = eval_brdfcos(pt, -bpt.wo);
        auto bld = bke * bbc * bw;
        if (bld != zero3f) {
            l += weight * bld * weight_mis(bw, weight_light(bpt, pt));
        }

        // skip recursion if path ends
        if (bounce == params.max_depth - 1) break;
        if (!bpt.fr) break;

        // continue path
        weight *= eval_brdfcos(pt, -bpt.wo) * weight_brdfcos(pt, -bpt.wo);
        if (weight == zero3f) break;

        // roussian roulette
        if (bounce > 2) {
            auto rrprob = 1.0f - min(max_element_val(pt.fr.rho()), 0.95f);
            if (sample_next1f(smp) < rrprob) break;
            weight *= 1 / (1 - rrprob);
        }

        // continue path
        pt = bpt;
        emission = false;
    }

    return l;
}

// Recursive path tracing.
inline vec3f eval_li_pathtrace_nomis(const scene* scn, const ray3f& ray,
    sampler& smp, const trace_params& params, bool& hit) {
    // intersection
    auto pt = intersect_scene(scn, ray);
    hit = pt.ist;

    // emission
    auto l = eval_emission(pt);
    if (!pt.fr) return l;

    // trace path
    auto weight = vec3f{1, 1, 1};
    auto emission = false;
    for (auto bounce = 0; bounce < params.max_depth; bounce++) {
        // emission
        if (emission) l += weight * eval_emission(pt);

        // direct
        auto lgt = scn->lights[sample_next1i(smp, (int)scn->lights.size())];
        auto lpt =
            sample_light(lgt, pt, sample_next1f(smp), sample_next2f(smp));
        auto ld = eval_emission(lpt) * eval_brdfcos(pt, -lpt.wo) *
                  weight_light(lpt, pt) * (float)scn->lights.size();
        if (ld != zero3f) {
            l += weight * ld * eval_transmission(scn, pt, lpt, params);
        }

        // skip recursion if path ends
        if (bounce == params.max_depth - 1) break;

        // roussian roulette
        if (bounce > 2) {
            auto rrprob = 1.0f - min(max_element_val(pt.fr.rho()), 0.95f);
            if (sample_next1f(smp) < rrprob) break;
            weight *= 1 / (1 - rrprob);
        }

        // continue path
        {
            auto wi =
                sample_brdfcos(pt, sample_next1f(smp), sample_next2f(smp));
            weight *= eval_brdfcos(pt, wi) * weight_brdfcos(pt, wi);
            if (weight == zero3f) break;

            pt = intersect_scene(scn, offset_ray(pt, wi, params));
            emission = false;
            if (!pt.fr) break;
        }
    }

    return l;
}

// Recursive path tracing.
inline vec3f eval_li_pathtrace_hack(const scene* scn, const ray3f& ray,
    sampler& smp, const trace_params& params, bool& hit) {
    // intersection
    auto pt = intersect_scene(scn, ray);
    hit = pt.ist;

    // emission
    auto l = eval_emission(pt);
    if (!pt.fr || scn->lights.empty()) return l;

    // trace path
    auto weight = vec3f{1, 1, 1};
    for (auto bounce = 0; bounce < params.max_depth; bounce++) {
        // direct
        auto lgt = scn->lights[sample_next1i(smp, (int)scn->lights.size())];
        auto lpt =
            sample_light(lgt, pt, sample_next1f(smp), sample_next2f(smp));
        auto ld = eval_emission(lpt) * eval_brdfcos(pt, -lpt.wo) *
                  weight_light(lpt, pt) * (float)scn->lights.size();
        if (ld != zero3f) {
            l += weight * ld * eval_transmission(scn, pt, lpt, params);
        }

        // skip recursion if path ends
        if (bounce == params.max_depth - 1) break;

        // roussian roulette
        if (bounce > 2) {
            auto rrprob = 1.0f - min(max_element_val(pt.fr.rho()), 0.95f);
            if (sample_next1f(smp) < rrprob) break;
            weight *= 1 / (1 - rrprob);
        }

        // continue path
        {
            auto wi =
                sample_brdfcos(pt, sample_next1f(smp), sample_next2f(smp));
            weight *= eval_brdfcos(pt, wi) * weight_brdfcos(pt, wi);
            if (weight == zero3f) break;

            pt = intersect_scene(scn, offset_ray(pt, wi, params));
            if (!pt.fr) break;
        }
    }

    return l;
}

// Direct illumination.
inline vec3f eval_li_direct(const scene* scn, const ray3f& ray, int bounce,
    sampler& smp, const trace_params& params, bool& hit) {
    // intersection
    auto pt = intersect_scene(scn, ray);
    if (!bounce) hit = pt.ist;

    // emission
    auto l = eval_emission(pt);
    if (!pt.fr || scn->lights.empty()) return l;

    // ambient
    l += params.amb * pt.fr.rho();

    // direct
    for (auto& lgt : scn->lights) {
        auto lpt =
            sample_light(lgt, pt, sample_next1f(smp), sample_next2f(smp));
        auto ld = eval_emission(lpt) * eval_brdfcos(pt, -lpt.wo) *
                  weight_light(lpt, pt);
        if (ld == zero3f) continue;
        l += ld * eval_transmission(scn, pt, lpt, params);
    }

    // exit if needed
    if (bounce >= params.max_depth) return l;

    // opacity
    if (pt.fr.kt != zero3f) {
        auto ray = offset_ray(pt, -pt.wo, params);
        l += pt.fr.kt * eval_li_direct(scn, ray, bounce + 1, smp, params, hit);
    }

    // done
    return l;
}

// Direct illumination.
inline vec3f eval_li_direct(const scene* scn, const ray3f& ray, sampler& smp,
    const trace_params& params, bool& hit) {
    return eval_li_direct(scn, ray, 0, smp, params, hit);
}

// Eyelight for quick previewing.
inline vec3f eval_li_eyelight(const scene* scn, const ray3f& ray, int bounce,
    sampler& smp, const trace_params& params, bool& hit) {
    // intersection
    auto pt = intersect_scene(scn, ray);
    if (!bounce) hit = pt.ist;

    // emission
    auto l = eval_emission(pt);
    if (!pt.fr) return l;

    // brdf*light
    l += eval_brdfcos(pt, pt.wo) * pif;

    // opacity
    if (bounce >= params.max_depth) return l;
    if (pt.fr.kt != zero3f) {
        auto ray = offset_ray(pt, -pt.wo, params);
        l +=
            pt.fr.kt * eval_li_eyelight(scn, ray, bounce + 1, smp, params, hit);
    }

    // done
    return l;
}

// Eyelight for quick previewing.
inline vec3f eval_li_eyelight(const scene* scn, const ray3f& ray, sampler& smp,
    const trace_params& params, bool& hit) {
    return eval_li_eyelight(scn, ray, 0, smp, params, hit);
}

// Debug previewing.
inline vec3f eval_li_debug_normal(const scene* scn, const ray3f& ray,
    sampler& smp, const trace_params& params, bool& hit) {
    // intersection
    auto isec = intersect_ray(scn, ray, false);
    hit = (bool)isec;
    if (!hit) return {0, 0, 0};

    // texcoord
    auto norm = eval_norm(scn->instances[isec.iid], isec.eid, isec.euv);
    return norm * 0.5f + vec3f{0.5f, 0.5f, 0.5f};
}

// Debug previewing.
inline vec3f eval_li_debug_albedo(const scene* scn, const ray3f& ray,
    sampler& smp, const trace_params& params, bool& hit) {
    // intersection
    auto pt = intersect_scene(scn, ray);
    hit = pt.ist;

    return pt.fr.rho();
}

// Debug previewing.
inline vec3f eval_li_debug_texcoord(const scene* scn, const ray3f& ray,
    sampler& smp, const trace_params& params, bool& hit) {
    // intersection
    auto isec = intersect_ray(scn, ray, false);
    hit = (bool)isec;
    if (!hit) return {0, 0, 0};

    // texcoord
    auto texcoord =
        eval_texcoord(scn->instances[isec.iid]->shp, isec.eid, isec.euv);
    return {texcoord.x, texcoord.y, 0};
}

// Shader function callback.
using eval_li_fn = vec3f (*)(const scene* scn, const ray3f& ray, sampler& smp,
    const trace_params& params, bool& hit);

// Get a shader function
inline eval_li_fn get_shader(const trace_params& params) {
    switch (params.stype) {
        case trace_shader_type::eyelight: return eval_li_eyelight;
        case trace_shader_type::direct: return eval_li_direct;
        case trace_shader_type::pathtrace: return eval_li_pathtrace;
        case trace_shader_type::pathtrace_nomis: return eval_li_pathtrace_nomis;
        case trace_shader_type::debug_albedo: return eval_li_debug_albedo;
        case trace_shader_type::debug_normal: return eval_li_debug_normal;
        case trace_shader_type::debug_texcoord: return eval_li_debug_texcoord;
        default: return nullptr;
    }
}

// triangle filter (public domain from stb_image_resize)
inline float filter_triangle(float x) {
    x = (float)fabs(x);

    if (x <= 1.0f)
        return 1 - x;
    else
        return 0;
}

// cubic filter (public domain from stb_image_resize)
inline float filter_cubic(float x) {
    x = (float)fabs(x);
    if (x < 1.0f)
        return (4 + x * x * (3 * x - 6)) / 6;
    else if (x < 2.0f)
        return (8 + x * (-12 + x * (6 - x))) / 6;
    else
        return 0.0f;
}

// catmull-rom filter (public domain from stb_image_resize)
inline float filter_catmullrom(float x) {
    x = (float)fabs(x);
    if (x < 1.0f)
        return 1 - x * x * (2.5f - 1.5f * x);
    else if (x < 2.0f)
        return 2 - x * (4 + x * (0.5f * x - 2.5f));
    else
        return 0.0f;
}

// mitchell filter (public domain from stb_image_resize)
inline float filter_mitchell(float x) {
    x = (float)fabs(x);
    if (x < 1.0f)
        return (16 + x * x * (21 * x - 36)) / 18;
    else if (x < 2.0f)
        return (32 + x * (-60 + x * (36 - 7 * x))) / 18;
    else
        return 0.0f;
}

// filter function
using filter_fn = float (*)(float);

// Get a filter function
inline filter_fn get_filter(const trace_params& params) {
    switch (params.ftype) {
        case trace_filter_type::box: return nullptr;
        case trace_filter_type::triangle: return filter_triangle;
        case trace_filter_type::cubic: return filter_cubic;
        case trace_filter_type::catmull_rom: return filter_catmullrom;
        case trace_filter_type::mitchell: return filter_mitchell;
        default: return nullptr;
    }
}

// Get a filter size
inline int get_filter_size(const trace_params& params) {
    switch (params.ftype) {
        case trace_filter_type::box: return 0;
        case trace_filter_type::triangle: return 1;
        case trace_filter_type::cubic:
        case trace_filter_type::catmull_rom:
        case trace_filter_type::mitchell: return 2;
        default: return 0;
    }
}

// Renders a block of pixels. Public API, see above.
inline void trace_block(const scene* scn, image4f& img, int block_x,
    int block_y, int block_width, int block_height, int samples_min,
    int samples_max, image<rng_pcg32>& rngs, const trace_params& params) {
    auto cam = scn->cameras[params.camera_id];
    auto shade = get_shader(params);
    for (auto j = block_y; j < block_y + block_height; j++) {
        for (auto i = block_x; i < block_x + block_width; i++) {
            auto lp = zero4f;
            for (auto s = samples_min; s < samples_max; s++) {
                auto smp = make_sampler(
                    rngs[{i, j}], i, j, s, params.nsamples, params.rtype);
                auto rn = sample_next2f(smp);
                auto uv = vec2f{
                    (i + rn.x) / params.width, 1 - (j + rn.y) / params.height};
                auto ray = eval_camera(cam, uv, sample_next2f(smp));
                auto hit = false;
                auto l = shade(scn, ray, smp, params, hit);
                if (!hit && params.envmap_invisible) continue;
                if (!isfinite(l)) {
                    log_error("NaN detected");
                    continue;
                }
                if (params.pixel_clamp > 0) l = clamplen(l, params.pixel_clamp);
                lp += {l, 1};
            }
            if (samples_min) {
                img[{i, j}] = (img[{i, j}] * (float)samples_min + lp) /
                              (float)samples_max;
            } else {
                img[{i, j}] = lp / (float)samples_max;
            }
        }
    }
}

// Trace a block of samples
inline void trace_block(const scene* scn, image4f& img, const bbox2i& block,
    int samples_min, int samples_max, image<rng_pcg32>& rngs,
    const trace_params& params) {
    auto shade = get_shader(params);
    auto cam = scn->cameras[params.camera_id];
    for (auto j = block.min.y; j < block.max.y; j++) {
        for (auto i = block.min.x; i < block.max.x; i++) {
            auto lp = zero4f;
            for (auto s = samples_min; s < samples_max; s++) {
                auto smp = make_sampler(
                    rngs[{i, j}], i, j, s, params.nsamples, params.rtype);
                auto rn = sample_next2f(smp);
                auto uv = vec2f{
                    (i + rn.x) / params.width, 1 - (j + rn.y) / params.height};
                auto ray = eval_camera(cam, uv, sample_next2f(smp));
                bool hit = false;
                auto l = shade(scn, ray, smp, params, hit);
                if (!hit || params.envmap_invisible) continue;
                if (!isfinite(l)) {
                    log_error("NaN detected");
                    continue;
                }
                if (params.pixel_clamp > 0) l = clamplen(l, params.pixel_clamp);
                lp += {l, 1};
            }
            if (samples_min) {
                img[{i, j}] = (img[{i, j}] * (float)samples_min + lp) /
                              (float)samples_max;
            } else {
                img[{i, j}] = lp / (float)samples_max;
            }
        }
    }
}

// Trace a block of samples
inline void trace_block_filtered(const scene* scn, image4f& img, image4f& acc,
    imagef& weight, const bbox2i& block, int samples_min, int samples_max,
    image<rng_pcg32>& rngs, std::mutex& image_mutex,
    const trace_params& params) {
    auto shade = get_shader(params);
    auto cam = scn->cameras[params.camera_id];
    auto filter = get_filter(params);
    auto filter_size = get_filter_size(params);
    static constexpr const int pad = 2;
    auto block_size = diagonal(block);
    auto acc_buffer = image4f(block_size.x + pad * 2, block_size.y + pad * 2);
    auto weight_buffer = imagef(block_size.x + pad * 2, block_size.y + pad * 2);
    for (auto j = block.min.y; j < block.max.y; j++) {
        for (auto i = block.min.x; i < block.max.x; i++) {
            for (auto s = samples_min; s < samples_max; s++) {
                auto smp = make_sampler(
                    rngs[{i, j}], i, j, s, params.nsamples, params.rtype);
                auto rn = sample_next2f(smp);
                auto uv = vec2f{
                    (i + rn.x) / params.width, 1 - (j + rn.y) / params.height};
                auto ray = eval_camera(cam, uv, sample_next2f(smp));
                auto hit = false;
                auto l = shade(scn, ray, smp, params, hit);
                if (!hit || params.envmap_invisible) continue;
                if (!isfinite(l)) {
                    log_error("NaN detected");
                    continue;
                }
                if (params.pixel_clamp > 0) l = clamplen(l, params.pixel_clamp);
                if (params.ftype == trace_filter_type::box) {
                    auto bi = i - block.min.x, bj = j - block.min.y;
                    acc_buffer[{bi + pad, bj + pad}] += {l, 1};
                    weight_buffer[{bi + pad, bj + pad}] += 1;
                } else {
                    auto bi = i - block.min.x, bj = j - block.min.y;
                    for (auto fj = -filter_size; fj <= filter_size; fj++) {
                        for (auto fi = -filter_size; fi <= filter_size; fi++) {
                            auto w = filter(fi - uv.x + 0.5f) *
                                     filter(fj - uv.y + 0.5f);
                            acc_buffer[{bi + fi + pad, bj + fj + pad}] +=
                                {l * w, w};
                            weight_buffer[{bi + fi + pad, bj + fj + pad}] += w;
                        }
                    }
                }
            }
        }
    }
    if (params.ftype == trace_filter_type::box) {
        for (auto j = block.min.y; j < block.max.y; j++) {
            for (auto i = block.min.x; i < block.max.x; i++) {
                auto bi = i - block.min.x, bj = j - block.min.y;
                acc[{i, j}] += acc_buffer[{bi + pad, bj + pad}];
                weight[{i, j}] += weight_buffer[{bi + pad, bj + pad}];
                img[{i, j}] = acc[{i, j}] / weight[{i, j}];
            }
        }
    } else {
        std::unique_lock<std::mutex> lock_guard(image_mutex);
        auto width = acc.width(), height = acc.height();
        for (auto j = max(block.min.y - filter_size, 0);
             j < min(block.max.y + filter_size, height); j++) {
            for (auto i = max(block.min.x - filter_size, 0);
                 i < min(block.max.x + filter_size, width); i++) {
                auto bi = i - block.min.x, bj = j - block.min.y;
                acc[{i, j}] += acc_buffer[{bi + pad, bj + pad}];
                weight[{i, j}] += weight_buffer[{bi + pad, bj + pad}];
                img[{i, j}] = acc[{i, j}] / weight[{i, j}];
            }
        }
    }
}

}  // namespace _impl_trace

// Renders a block of samples
inline void trace_block(const scene* scn, image4f& img, const bbox2i& block,
    int samples_min, int samples_max, image<rng_pcg32>& rngs,
    const trace_params& params) {
    _impl_trace::trace_block(
        scn, img, block, samples_min, samples_max, rngs, params);
}

// Renders a filtered block of samples
inline void trace_block_filtered(const scene* scn, image4f& img, image4f& acc,
    imagef& weight, const bbox2i& block, int samples_min, int samples_max,
    image<rng_pcg32>& rngs, std::mutex& image_mutex,
    const trace_params& params) {
    _impl_trace::trace_block_filtered(scn, img, acc, weight, block, samples_min,
        samples_max, rngs, image_mutex, params);
}

}  // namespace ygl

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR WAVEFRONT OBJ
// -----------------------------------------------------------------------------
namespace ygl {

#if YGL_SCENEIO

namespace _impl_obj {

// Parse a value
template <typename T>
inline void parse_val(stringstream& ss, T& v) {
    ss >> v;
}

// Parse a value
template <typename T, int N>
inline void parse_val(stringstream& ss, vec<T, N>& v) {
    for (auto i = 0; i < N; i++) parse_val(ss, v[i]);
}

// Parse a value
template <typename T, int N>
inline void parse_val(stringstream& ss, frame<T, N>& v) {
    for (auto i = 0; i < N + 1; i++) parse_val(ss, v[i]);
}

// Parse texture options and name
inline void parse_texture(stringstream& ss, obj_texture_info& info,
    vector<string>& textures, unordered_set<string>& texture_set,
    bool bump = false) {
    // get tokens
    auto tokens = vector<string>();
    while (ss) {
        auto s = string();
        ss >> s;
        if (!s.empty()) tokens.push_back(s);
    }

    // exit if no tokens
    if (tokens.empty()) return;

    // texture name
    info.path = tokens.back();
    for (auto& c : info.path)
        if (c == '\\') c = '/';

    // texture options
    auto last = string();
    for (auto& tok : tokens) {
        if (tok == tokens.back()) break;
        if (tok[0] == '-') {
            last = tok;
            info.unknown_props[last] = {};
        } else {
            info.unknown_props[last].push_back(tok);
        }
    }

    // clamp
    if (info.unknown_props.find("-clamp") != info.unknown_props.end() &&
        info.unknown_props.at("-clamp").size() > 0) {
        auto& clamp_vec = info.unknown_props.at("-clamp");
        auto clamp_str = (clamp_vec.empty()) ? "" : clamp_vec.front();
        info.clamp = clamp_str == "on" || clamp_str == "1";
        info.unknown_props.erase("-clamp");
    }

    if (info.unknown_props.find("-bm") != info.unknown_props.end() &&
        info.unknown_props.at("-bm").size() > 0) {
        auto& bm_vec = info.unknown_props.at("-bm");
        auto bm_str = (bm_vec.empty()) ? "" : bm_vec.front();
        info.scale = std::atof(bm_str.c_str());
        info.unknown_props.erase("-bm");
    }

    // insert texture
    if (!info.path.empty() &&
        texture_set.find(info.path) == texture_set.end()) {
        textures.push_back(info.path);
        texture_set.insert(info.path);
    }
}

// Load MTL
inline vector<obj_material*> load_mtl(
    const string& filename, bool flip_tr, vector<string>& textures) {
    // clear materials
    auto materials = vector<obj_material*>();

    // clear textures
    textures.clear();
    auto texture_set = unordered_set<string>();

    // open file
    auto fs = fstream(filename, ios_base::in);
    if (!fs) throw runtime_error("cannot open filename " + filename);
    fs.exceptions(ios_base::failbit);

    // add a material preemptively to avoid crashes
    materials.push_back(new obj_material());

    // read the file line by line
    char linebuf[4096];
    auto linenum = 0;
    while (fs.getline(linebuf, 4096)) {
        // prepare to parse
        linenum += 1;
        auto ss = stringstream(linebuf);
        auto cmd = string();
        ss >> cmd;

        // skip empty and comments
        if (cmd.empty() || cmd[0] == '#') continue;

        // possible token values
        if (cmd == "newmtl") {
            materials.push_back(new obj_material());
            parse_val(ss, materials.back()->name);
        } else if (cmd == "illum") {
            parse_val(ss, materials.back()->illum);
        } else if (cmd == "Ke") {
            parse_val(ss, materials.back()->ke);
        } else if (cmd == "Ka") {
            parse_val(ss, materials.back()->ka);
        } else if (cmd == "Kd") {
            parse_val(ss, materials.back()->kd);
        } else if (cmd == "Ks") {
            parse_val(ss, materials.back()->ks);
        } else if (cmd == "Kr") {
            parse_val(ss, materials.back()->kr);
        } else if (cmd == "Kt" || cmd == "Tf") {
            auto vals = zero3f;
            auto ntok = 0;
            while (ss) parse_val(ss, vals[ntok++]);
            if (ntok >= 3)
                materials.back()->kt = vals;
            else
                materials.back()->kt = {vals.x, vals.x, vals.x};
        } else if (cmd == "Tr") {
            auto vals = zero3f;
            auto ntok = 0;
            while (ss) parse_val(ss, vals[ntok++]);
            if (ntok >= 3)
                materials.back()->kt = vals;
            else
                materials.back()->op = (flip_tr) ? 1 - vals.x : vals.x;
        } else if (cmd == "Ns") {
            parse_val(ss, materials.back()->ns);
        } else if (cmd == "d") {
            parse_val(ss, materials.back()->op);
        } else if (cmd == "Ni") {
            parse_val(ss, materials.back()->ior);
        } else if (cmd == "map_Ke") {
            parse_texture(ss, materials.back()->ke_txt, textures, texture_set);
        } else if (cmd == "map_Ka") {
            parse_texture(ss, materials.back()->ka_txt, textures, texture_set);
        } else if (cmd == "map_Kd") {
            parse_texture(ss, materials.back()->kd_txt, textures, texture_set);
        } else if (cmd == "map_Ks") {
            parse_texture(ss, materials.back()->ks_txt, textures, texture_set);
        } else if (cmd == "map_Kr") {
            parse_texture(ss, materials.back()->kr_txt, textures, texture_set);
        } else if (cmd == "map_Tr") {
            parse_texture(ss, materials.back()->kt_txt, textures, texture_set);
        } else if (cmd == "map_Ns") {
            parse_texture(ss, materials.back()->ns_txt, textures, texture_set);
        } else if (cmd == "map_d") {
            parse_texture(ss, materials.back()->op_txt, textures, texture_set);
        } else if (cmd == "map_Ni") {
            parse_texture(ss, materials.back()->ior_txt, textures, texture_set);
        } else if (cmd == "map_bump" || cmd == "bump") {
            parse_texture(
                ss, materials.back()->bump_txt, textures, texture_set);
        } else if (cmd == "map_disp" || cmd == "disp") {
            parse_texture(
                ss, materials.back()->disp_txt, textures, texture_set);
        } else if (cmd == "map_norm" || cmd == "norm") {
            parse_texture(
                ss, materials.back()->norm_txt, textures, texture_set);
        } else {
            // copy into strings
            while (ss) {
                materials.back()->unknown_props[cmd].push_back({});
                parse_val(ss, materials.back()->unknown_props[cmd].back());
            }
        }
    }

    // remove first fake material
    materials.erase(materials.begin());

    // done
    return materials;
}

// Loads textures for an scene.
inline void load_textures(
    obj_scene* asset, const string& dirname, bool skip_missing) {
    for (auto txt : asset->textures) {
        auto filename = dirname + txt->path;
        for (auto& c : filename)
            if (c == '\\') c = '/';
#if YGL_IMAGEIO
        if (is_hdr_filename(filename)) {
            txt->dataf =
                load_imagef(filename, txt->width, txt->height, txt->ncomp);
        } else {
            txt->datab =
                load_image(filename, txt->width, txt->height, txt->ncomp);
        }
#endif
        if (txt->datab.empty() && txt->dataf.empty()) {
            if (skip_missing) continue;
            throw runtime_error("cannot laod image " + filename);
        }
    }
}

// Parses an OBJ vertex list. Handles negative values.
inline void parse_vertlist(
    stringstream& ss, vector<obj_vertex>& elems, const obj_vertex& vert_size) {
    elems.clear();
    while (true) {
        auto tok = string();
        parse_val(ss, tok);
        if (tok.empty()) break;
        auto toks = split(tok, "/");
        if (toks.empty()) break;
        auto v = obj_vertex{-1, -1, -1, -1, -1};
        for (auto i = 0; i < min(5, (int)toks.size()); i++) {
            if (toks[i] == "") continue;
            ((int*)&v)[i] = (int)atoi(toks[i].c_str());
            ((int*)&v)[i] = (((int*)&v)[i] < 0) ?
                                ((int*)&vert_size)[i] + ((int*)&v)[i] :
                                ((int*)&v)[i] - 1;
        }
        elems.push_back(v);
    }
}

// Loads an OBJ
inline obj_scene* load_obj(const string& filename, bool load_txt,
    bool skip_missing, bool flip_texcoord, bool flip_tr) {
    // clear obj
    auto asset = unique_ptr<obj_scene>(new obj_scene());

    // open file
    auto fs = fstream(filename, ios_base::in);
    if (!fs) throw runtime_error("cannot open filename " + filename);
    fs.exceptions(ios_base::failbit);

    // initializing obj
    asset->objects.push_back(new obj_object());
    asset->objects.back()->groups.push_back({});

    // allocate buffers to avoid re-allocing
    auto cur_elems = vector<obj_vertex>();
    auto cur_matname = string();
    auto cur_mtllibs = vector<string>();

    // keep track of array lengths
    auto vert_size = obj_vertex{0, 0, 0, 0, 0};

    // read the file line by line
    char linebuf[4096];
    auto linenum = 0;
    while (fs.getline(linebuf, 4096)) {
        // prepare to parse
        linenum += 1;
        auto ss = stringstream(linebuf);
        auto cmd = string();
        ss >> cmd;

        // skip empty and comments
        if (cmd.empty() || cmd[0] == '#') continue;

        // possible token values
        if (cmd == "v") {
            vert_size.pos += 1;
            asset->pos.push_back({});
            parse_val(ss, asset->pos.back());
        } else if (cmd == "vn") {
            vert_size.norm += 1;
            asset->norm.push_back({});
            parse_val(ss, asset->norm.back());
        } else if (cmd == "vt") {
            vert_size.texcoord += 1;
            asset->texcoord.push_back({});
            parse_val(ss, asset->texcoord.back());
            if (flip_texcoord)
                asset->texcoord.back().y = 1 - asset->texcoord.back().y;
        } else if (cmd == "vc") {
            vert_size.color += 1;
            asset->color.push_back({});
            parse_val(ss, asset->color.back());
        } else if (cmd == "vr") {
            vert_size.radius += 1;
            asset->radius.push_back({});
            parse_val(ss, asset->radius.back());
        } else if (cmd == "f") {
            parse_vertlist(ss, cur_elems, vert_size);
            auto& g = asset->objects.back()->groups.back();
            g.elems.push_back({(uint32_t)g.verts.size(), obj_element_type::face,
                (uint16_t)cur_elems.size()});
            g.verts.insert(g.verts.end(), cur_elems.begin(), cur_elems.end());
        } else if (cmd == "l") {
            parse_vertlist(ss, cur_elems, vert_size);
            auto& g = asset->objects.back()->groups.back();
            g.elems.push_back({(uint32_t)g.verts.size(), obj_element_type::line,
                (uint16_t)cur_elems.size()});
            g.verts.insert(g.verts.end(), cur_elems.begin(), cur_elems.end());
        } else if (cmd == "p") {
            parse_vertlist(ss, cur_elems, vert_size);
            auto& g = asset->objects.back()->groups.back();
            g.elems.push_back({(uint32_t)g.verts.size(),
                obj_element_type::point, (uint16_t)cur_elems.size()});
            g.verts.insert(g.verts.end(), cur_elems.begin(), cur_elems.end());
        } else if (cmd == "t") {
            parse_vertlist(ss, cur_elems, vert_size);
            auto& g = asset->objects.back()->groups.back();
            g.elems.push_back({(uint32_t)g.verts.size(),
                obj_element_type::tetra, (uint16_t)cur_elems.size()});
            g.verts.insert(g.verts.end(), cur_elems.begin(), cur_elems.end());
        } else if (cmd == "o") {
            auto name = string();
            parse_val(ss, name);
            asset->objects.push_back(new obj_object{name, {}});
            asset->objects.back()->groups.push_back({cur_matname, ""});
        } else if (cmd == "usemtl") {
            auto name = string();
            parse_val(ss, name);
            cur_matname = name;
            asset->objects.back()->groups.push_back({cur_matname, ""});
        } else if (cmd == "g") {
            auto name = string();
            parse_val(ss, name);
            asset->objects.back()->groups.push_back({cur_matname, name});
        } else if (cmd == "s") {
            auto name = string();
            parse_val(ss, name);
            auto smoothing = name == string("on");
            if (asset->objects.back()->groups.back().smoothing != smoothing) {
                asset->objects.back()->groups.push_back(
                    {cur_matname, name, smoothing});
            }
        } else if (cmd == "mtllib") {
            auto name = string();
            parse_val(ss, name);
            if (name != string("")) {
                auto found = false;
                for (auto lib : cur_mtllibs) {
                    if (lib == name) {
                        found = true;
                        break;
                    }
                }
                if (!found) cur_mtllibs.push_back(name);
            }
        } else if (cmd == "c") {
            auto cam = new obj_camera();
            parse_val(ss, cam->name);
            parse_val(ss, cam->ortho);
            parse_val(ss, cam->yfov);
            parse_val(ss, cam->aspect);
            parse_val(ss, cam->aperture);
            parse_val(ss, cam->focus);
            parse_val(ss, cam->frame);
            asset->cameras.push_back(cam);
        } else if (cmd == "e") {
            auto env = new obj_environment();
            parse_val(ss, env->name);
            parse_val(ss, env->matname);
            parse_val(ss, env->frame);
            asset->environments.push_back(env);
        } else if (cmd == "i") {
            auto ist = new obj_instance();
            parse_val(ss, ist->name);
            parse_val(ss, ist->objname);
            parse_val(ss, ist->frame);
            asset->instances.push_back(ist);
        } else {
            // unused
        }
    }

    // cleanup unused
    for (auto o : asset->objects) {
        auto end = std::remove_if(o->groups.begin(), o->groups.end(),
            [](const obj_group& x) { return x.verts.empty(); });
        o->groups.erase(end, o->groups.end());
    }
    // TODO: possible memory leak
    auto end = std::remove_if(asset->objects.begin(), asset->objects.end(),
        [](const obj_object* x) { return x->groups.empty(); });
    asset->objects.erase(end, asset->objects.end());

    // parse materials
    auto dirname = path_dirname(filename);
    unordered_set<string> texture_set;
    for (auto mtllib : cur_mtllibs) {
        auto mtlname = dirname + mtllib;
        vector<string> textures;
        auto materials = load_mtl(mtlname, flip_tr, textures);
        asset->materials.insert(
            asset->materials.end(), materials.begin(), materials.end());
        for (auto& txt : textures) {
            if (texture_set.find(txt) != texture_set.end()) continue;
            asset->textures.push_back(new obj_texture{txt});
            texture_set.insert(txt);
        }
    }

    // load textures
    if (load_txt) load_textures(asset.get(), dirname, skip_missing);

    // done
    return asset.release();
}

// write to stream
template <typename T>
inline void dump_val(fstream& fs, const T& v) {
    fs << v;
}

// write to stream
template <typename T, int N>
inline void dump_val(fstream& fs, const vec<T, N>& v) {
    for (auto i = 0; i < N; i++) {
        if (i) fs << ' ';
        dump_val(fs, v[i]);
    }
}

// write to stream
template <typename T, int N>
inline void dump_val(fstream& fs, const frame<T, N>& v) {
    for (auto i = 0; i < N + 1; i++) {
        if (i) fs << ' ';
        dump_val(fs, v[i]);
    }
}

// write to stream
inline void dump_val(fstream& fs, const obj_texture_info& v) {
    for (auto&& kv : v.unknown_props) {
        dump_val(fs, kv.first + " ");
        for (auto&& vv : kv.second) dump_val(fs, vv + " ");
    }
    if (v.clamp) dump_val(fs, "-clamp on ");
    dump_val(fs, v.path);
}

// write to stream
template <typename T>
inline void dump_named_val(fstream& fs, const string& name, const T& v) {
    dump_val(fs, name);
    fs << ' ';
    dump_val(fs, v);
    fs << '\n';
}

// write to stream
template <typename T>
inline void dump_opt_val(
    fstream& fs, const string& name, const T& v, const T& def = {}) {
    if (v == def) return;
    dump_named_val(fs, name, v);
}

// write an OBJ vertex triplet using only the indices that are active
inline void dump_objverts(
    fstream& fs, const char* str, int nv, const obj_vertex* verts) {
    dump_val(fs, str);
    for (auto v = 0; v < nv; v++) {
        auto& vert = verts[v];
        auto vert_ptr = &vert.pos;
        auto nto_write = 0;
        for (auto i = 0; i < 5; i++) {
            if (vert_ptr[i] >= 0) nto_write = i + 1;
        }
        for (auto i = 0; i < nto_write; i++) {
            if (vert_ptr[i] >= 0) {
                dump_val(fs, ((i == 0) ? ' ' : '/'));
                dump_val(fs, vert_ptr[i] + 1);
            } else {
                dump_val(fs, '/');
            }
        }
    }
    dump_val(fs, '\n');
}

// Save an MTL file
inline void save_mtl(const string& filename,
    const vector<obj_material*>& materials, bool flip_tr) {
    // open file
    auto fs = fstream(filename, ios_base::out);
    if (!fs) throw runtime_error("cannot open filename " + filename);
    fs.exceptions(ios_base::failbit);

    // for each material, dump all the values
    for (auto mat : materials) {
        dump_named_val(fs, "newmtl", mat->name);
        dump_named_val(fs, "  illum", mat->illum);
        dump_opt_val(fs, "  Ke", mat->ke);
        dump_opt_val(fs, "  Ka", mat->ka);
        dump_opt_val(fs, "  Kd", mat->kd);
        dump_opt_val(fs, "  Ks", mat->ks);
        dump_opt_val(fs, "  Kr", mat->kr);
        dump_opt_val(fs, "  Tf", mat->kt);
        dump_opt_val(fs, "  Ns", mat->ns, 0.0f);
        dump_opt_val(fs, "  d", mat->op, 1.0f);
        dump_opt_val(fs, "  Ni", mat->ior, 1.0f);
        dump_opt_val(fs, "  map_Ke", mat->ke_txt);
        dump_opt_val(fs, "  map_Ka", mat->ka_txt);
        dump_opt_val(fs, "  map_Kd", mat->kd_txt);
        dump_opt_val(fs, "  map_Ks", mat->ks_txt);
        dump_opt_val(fs, "  map_Kr", mat->kr_txt);
        dump_opt_val(fs, "  map_Kt", mat->kt_txt);
        dump_opt_val(fs, "  map_Ns", mat->ns_txt);
        dump_opt_val(fs, "  map_d", mat->op_txt);
        dump_opt_val(fs, "  map_Ni", mat->ior_txt);
        dump_opt_val(fs, "  map_bump", mat->bump_txt);
        dump_opt_val(fs, "  map_disp", mat->disp_txt);
        dump_opt_val(fs, "  map_norm", mat->norm_txt);
        for (auto&& kv : mat->unknown_props) {
            dump_val(fs, kv.first);
            for (auto&& v : kv.second) {
                dump_val(fs, " ");
                dump_val(fs, v);
            }
            dump_val(fs, "\n");
        }
        dump_val(fs, "\n");
    }
}

// Loads textures for an scene.
inline void save_textures(
    const obj_scene* asset, const string& dirname, bool skip_missing) {
    for (auto txt : asset->textures) {
        if (txt->datab.empty() && txt->dataf.empty()) continue;
        auto filename = dirname + txt->path;
        for (auto& c : filename)
            if (c == '\\') c = '/';
        auto ok = false;
#if YGL_IMAGEIO
        if (!txt->datab.empty()) {
            ok = save_image(filename, txt->width, txt->height, txt->ncomp,
                txt->datab.data());
        }
        if (!txt->dataf.empty()) {
            ok = save_imagef(filename, txt->width, txt->height, txt->ncomp,
                txt->dataf.data());
        }
#endif
        if (!ok) {
            if (skip_missing) continue;
            throw runtime_error("cannot save image " + filename);
        }
    }
}

// Save an OBJ
inline void save_obj(const string& filename, const obj_scene* asset,
    bool save_txt, bool skip_missing, bool flip_texcoord, bool flip_tr) {
    // open file
    auto fs = fstream(filename, ios_base::out);
    if (!fs) throw runtime_error("cannot open filename " + filename);
    fs.exceptions(ios_base::failbit);

    // linkup to mtl
    auto dirname = path_dirname(filename);
    auto basename = filename.substr(dirname.length());
    basename = basename.substr(0, basename.length() - 4);
    if (!asset->materials.empty()) {
        dump_named_val(fs, "mtllib", basename + ".mtl");
    }

    // save cameras
    for (auto cam : asset->cameras) {
        dump_val(fs, "c ");
        dump_val(fs, cam->name);
        dump_val(fs, " ");
        dump_val(fs, cam->ortho);
        dump_val(fs, " ");
        dump_val(fs, cam->yfov);
        dump_val(fs, " ");
        dump_val(fs, cam->aspect);
        dump_val(fs, " ");
        dump_val(fs, cam->aperture);
        dump_val(fs, " ");
        dump_val(fs, cam->focus);
        dump_val(fs, " ");
        dump_val(fs, cam->frame);
        dump_val(fs, '\n');
    }

    // save envs
    for (auto env : asset->environments) {
        dump_val(fs, "e ");
        dump_val(fs, env->name);
        dump_val(fs, " ");
        dump_val(fs, env->matname);
        dump_val(fs, " ");
        dump_val(fs, env->frame);
        dump_val(fs, '\n');
    }

    // save instances
    for (auto ist : asset->instances) {
        dump_val(fs, "i ");
        dump_val(fs, ist->name);
        dump_val(fs, " ");
        dump_val(fs, ist->objname);
        dump_val(fs, " ");
        dump_val(fs, ist->frame);
        dump_val(fs, '\n');
    }

    // save all vertex data
    for (auto& v : asset->pos) dump_named_val(fs, "v", v);
    if (flip_texcoord) {
        for (auto& v : asset->texcoord)
            dump_named_val(fs, "vt", vec2f{v.x, 1 - v.y});
    } else {
        for (auto& v : asset->texcoord) dump_named_val(fs, "vt", v);
    }
    for (auto& v : asset->norm) dump_named_val(fs, "vn", v);
    for (auto& v : asset->color) dump_named_val(fs, "vc", v);
    for (auto& v : asset->radius) dump_named_val(fs, "vr", v);

    // save element data
    const char* elem_labels[] = {"", "p", "l", "f", "t"};
    for (auto object : asset->objects) {
        dump_named_val(fs, "o", object->name);
        for (auto& group : object->groups) {
            dump_opt_val(fs, "usemtl", group.matname);
            dump_opt_val(fs, "g", group.groupname);
            if (!group.smoothing) dump_named_val(fs, "s", "off");
            for (auto elem : group.elems) {
                dump_objverts(fs, elem_labels[(int)elem.type], elem.size,
                    group.verts.data() + elem.start);
            }
        }
    }

    // save materials
    if (!asset->materials.empty())
        save_mtl(dirname + basename + ".mtl", asset->materials, flip_tr);

    // save textures
    if (save_txt) save_textures(asset, dirname, skip_missing);
}

// A hash function for vecs
struct vertex_hash {
    std::hash<int> Th;
    size_t operator()(const obj_vertex& vv) const {
        auto v = (const int*)&vv;
        size_t h = 0;
        for (auto i = 0; i < sizeof(obj_vertex) / sizeof(int); i++) {
            // embads hash_combine below
            h ^= (Th(v[i]) + 0x9e3779b9 + (h << 6) + (h >> 2));
        }
        return h;
    }
};

// Flattens an scene
inline obj_mesh* get_mesh(
    const obj_scene* model, const obj_object& oshape, bool facet_non_smooth) {
    // convert meshes
    auto msh = new obj_mesh();
    msh->name = oshape.name;
    for (auto& group : oshape.groups) {
        if (group.verts.empty()) continue;
        if (group.elems.empty()) continue;
        msh->shapes.emplace_back();
        auto prim = &msh->shapes.back();
        prim->name = group.groupname;
        prim->matname = group.matname;

        // insert all vertices
        unordered_map<obj_vertex, int, vertex_hash> vert_map;
        vector<int> vert_ids;
        // vert_map.clear();
        // vert_ids.clear();
        for (auto& vert : group.verts) {
            if (vert_map.find(vert) == vert_map.end()) {
                // split in two to avoid undefined behaviour
                auto size = (int)vert_map.size();
                vert_map[vert] = size;
            }
            vert_ids.push_back(vert_map.at(vert));
        }

        // convert elements
        for (auto& elem : group.elems) {
            switch (elem.type) {
                case obj_element_type::point: {
                    for (auto i = elem.start; i < elem.start + elem.size; i++) {
                        prim->points.push_back(vert_ids[i]);
                    }
                } break;
                case obj_element_type::line: {
                    for (auto i = elem.start; i < elem.start + elem.size - 1;
                         i++) {
                        prim->lines.push_back({vert_ids[i], vert_ids[i + 1]});
                    }
                } break;
                case obj_element_type::face: {
                    for (auto i = elem.start + 2; i < elem.start + elem.size;
                         i++) {
                        prim->triangles.push_back({vert_ids[elem.start],
                            vert_ids[i - 1], vert_ids[i]});
                    }
                } break;
                case obj_element_type::tetra: {
                    for (auto i = elem.start; i < elem.start + elem.size;
                         i += 4) {
                        if (i + 3 >= vert_ids.size()) continue;
                        prim->tetras.push_back({vert_ids[i], vert_ids[i + 1],
                            vert_ids[i + 2], vert_ids[i + 3]});
                    }
                } break;
                default: { assert(false); }
            }
        }

        // check for errors
        // copy vertex data
        auto v = group.verts[0];
        if (v.pos >= 0) prim->pos.resize(vert_map.size());
        if (v.texcoord >= 0) prim->texcoord.resize(vert_map.size());
        if (v.norm >= 0) prim->norm.resize(vert_map.size());
        if (v.color >= 0) prim->color.resize(vert_map.size());
        if (v.radius >= 0) prim->radius.resize(vert_map.size());
        for (auto& kv : vert_map) {
            if (v.pos >= 0 && kv.first.pos >= 0) {
                prim->pos[kv.second] = model->pos[kv.first.pos];
            }
            if (v.texcoord >= 0 && kv.first.texcoord >= 0) {
                prim->texcoord[kv.second] = model->texcoord[kv.first.texcoord];
            }
            if (v.norm >= 0 && kv.first.norm >= 0) {
                prim->norm[kv.second] = model->norm[kv.first.norm];
            }
            if (v.color >= 0 && kv.first.color >= 0) {
                prim->color[kv.second] = model->color[kv.first.color];
            }
            if (v.radius >= 0 && kv.first.radius >= 0) {
                prim->radius[kv.second] = model->radius[kv.first.radius];
            }
        }

        // fix smoothing
        if (!group.smoothing && facet_non_smooth) {
            auto faceted_ = obj_shape();
            auto faceted = &faceted_;
            faceted->name = prim->name;
            faceted->matname = prim->matname;
            auto pidx = vector<int>();
            for (auto point : prim->points) {
                faceted->points.push_back((int)pidx.size());
                pidx.push_back(point);
            }
            for (auto line : prim->lines) {
                faceted->lines.push_back(
                    {(int)pidx.size() + 0, (int)pidx.size() + 1});
                pidx.push_back(line.x);
                pidx.push_back(line.y);
            }
            for (auto triangle : prim->triangles) {
                faceted->triangles.push_back({(int)pidx.size() + 0,
                    (int)pidx.size() + 1, (int)pidx.size() + 2});
                pidx.push_back(triangle.x);
                pidx.push_back(triangle.y);
                pidx.push_back(triangle.z);
            }
            for (auto tetra : prim->tetras) {
                faceted->tetras.push_back(
                    {(int)pidx.size() + 0, (int)pidx.size() + 1,
                        (int)pidx.size() + 2, (int)pidx.size() + 3});
                pidx.push_back(tetra.x);
                pidx.push_back(tetra.y);
                pidx.push_back(tetra.z);
                pidx.push_back(tetra.w);
            }
            for (auto idx : pidx) {
                if (!prim->pos.empty()) faceted->pos.push_back(prim->pos[idx]);
                if (!prim->norm.empty())
                    faceted->norm.push_back(prim->norm[idx]);
                if (!prim->texcoord.empty())
                    faceted->texcoord.push_back(prim->texcoord[idx]);
                if (!prim->color.empty())
                    faceted->color.push_back(prim->color[idx]);
                if (!prim->radius.empty())
                    faceted->radius.push_back(prim->radius[idx]);
            }
            *prim = *faceted;
        }
    }

    // done
    return msh;
}

}  // namespace _impl_obj

// Loads an OBJ
inline obj_scene* load_obj(const string& filename, bool load_txt,
    bool skip_missing, bool flip_texcoord, bool flip_tr) {
    return _impl_obj::load_obj(
        filename, load_txt, skip_missing, flip_texcoord, flip_tr);
}

// Save an OBJ
inline void save_obj(const string& filename, const obj_scene* asset,
    bool save_txt, bool skip_missing, bool flip_texcoord, bool flip_tr) {
    _impl_obj::save_obj(
        filename, asset, save_txt, skip_missing, flip_texcoord, flip_tr);
}

// Flattens an scene
inline obj_mesh* get_mesh(
    const obj_scene* model, const obj_object& oshape, bool facet_non_smooth) {
    return _impl_obj::get_mesh(model, oshape, facet_non_smooth);
}

#endif

}  // namespace ygl

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR KHRONOS GLTF
// -----------------------------------------------------------------------------
namespace ygl {

#if YGL_SCENEIO

namespace _impl_gltf {

// #codegen begin func ---------------------------------------------------------

// Parse error
struct parse_stack {
    vector<string> path = {"glTF"};
    string pathname() {
        auto p = std::string();
        for (auto n : path) p += '/' + n;
        return p;
    }
};

// Parse support function.
template <typename T>
inline void parse(vector<T>& vals, const json& js, parse_stack& err) {
    if (!js.is_array()) throw runtime_error("array expected");
    vals.resize(js.size());
    for (auto i = 0; i < js.size(); i++) {
        // this is contrived to support for vector<bool>
        auto v = T();
        parse(v, js[i], err);
        vals[i] = v;
    }
}

// Parse support function.
template <typename T, int N>
inline void parse(vec<T, N>& vals, const json& js, parse_stack& err) {
    if (!js.is_array()) throw runtime_error("array expected");
    if (N != js.size()) throw runtime_error("wrong array size");
    for (auto i = 0; i < N; i++) { parse(vals[i], js[i], err); }
}

// Parse support function.
template <typename T, int N>
inline void parse(quat<T, N>& vals, const json& js, parse_stack& err) {
    if (!js.is_array()) throw runtime_error("array expected");
    if (N != js.size()) throw runtime_error("wrong array size");
    for (auto i = 0; i < N; i++) { parse(vals[i], js[i], err); }
}

// Parse support function.
template <typename T, int N, int M>
inline void parse(mat<T, N, M>& vals, const json& js, parse_stack& err) {
    if (!js.is_array()) throw runtime_error("array expected");
    if (N * M != js.size()) throw runtime_error("wrong array size");
    for (auto j = 0; j < M; j++) {
        for (auto i = 0; i < N; i++) { parse(vals[j][i], js[j * N + i], err); }
    }
}

// Parse support function.
template <typename T>
inline void parse(map<string, T>& vals, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    for (auto kv = js.begin(); kv != js.end(); ++kv) {
        parse(vals[kv.key()], kv.value(), err);
    }
}

// Parse support function.
template <typename T>
inline void parse_attr(
    T& val, const char* name, const json& js, parse_stack& err) {
    auto iter = js.find(name);
    if (iter == js.end()) return;
    err.path.push_back(name);
    parse(val, *iter, err);
    err.path.pop_back();
}

// Parse int function.
inline void parse(int& val, const json& js, parse_stack& err) {
    if (!js.is_number_integer()) throw runtime_error("integer expected");
    val = js;
}

// Parse float function.
inline void parse(float& val, const json& js, parse_stack& err) {
    if (!js.is_number()) throw runtime_error("number expected");
    val = js;
}

// Parse bool function.
inline void parse(bool& val, const json& js, parse_stack& err) {
    if (!js.is_boolean()) throw runtime_error("bool expected");
    val = js;
}

// Parse std::string function.
inline void parse(string& val, const json& js, parse_stack& err) {
    if (!js.is_string()) throw runtime_error("string expected");
    val = js;
}

// Parse json function.
inline void parse(json& val, const json& js, parse_stack& err) { val = js; }

// Parse id function.
template <typename T>
inline void parse(glTFid<T>& val, const json& js, parse_stack& err) {
    if (!js.is_number_integer()) throw runtime_error("int expected");
    val = glTFid<T>((int)js);
}

// Parses a glTFProperty object
inline void parse(glTFProperty*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFProperty();

    parse_attr(val->extensions, "extensions", js, err);
    parse_attr(val->extras, "extras", js, err);
}

// Parses a glTFChildOfRootProperty object
inline void parse(
    glTFChildOfRootProperty*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFChildOfRootProperty();
    parse((glTFProperty*&)val, js, err);
    parse_attr(val->name, "name", js, err);
}
// Parse a glTFAccessorSparseIndicesComponentType enum
inline void parse(glTFAccessorSparseIndicesComponentType& val, const json& js,
    parse_stack& err) {
    static map<int, glTFAccessorSparseIndicesComponentType> table = {
        {5121, glTFAccessorSparseIndicesComponentType::UnsignedByte},
        {5123, glTFAccessorSparseIndicesComponentType::UnsignedShort},
        {5125, glTFAccessorSparseIndicesComponentType::UnsignedInt},
    };
    auto v = int();
    parse(v, js, err);
    if (table.find(v) == table.end()) throw runtime_error("bad enum value");
    val = table[v];
}

// Parses a glTFAccessorSparseIndices object
inline void parse(
    glTFAccessorSparseIndices*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFAccessorSparseIndices();
    parse((glTFProperty*&)val, js, err);
    if (!js.count("bufferView"))
        throw runtime_error("missing required variable");
    parse_attr(val->bufferView, "bufferView", js, err);
    parse_attr(val->byteOffset, "byteOffset", js, err);
    if (!js.count("componentType"))
        throw runtime_error("missing required variable");
    parse_attr(val->componentType, "componentType", js, err);
}

// Parses a glTFAccessorSparseValues object
inline void parse(
    glTFAccessorSparseValues*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFAccessorSparseValues();
    parse((glTFProperty*&)val, js, err);
    if (!js.count("bufferView"))
        throw runtime_error("missing required variable");
    parse_attr(val->bufferView, "bufferView", js, err);
    parse_attr(val->byteOffset, "byteOffset", js, err);
}

// Parses a glTFAccessorSparse object
inline void parse(glTFAccessorSparse*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFAccessorSparse();
    parse((glTFProperty*&)val, js, err);
    if (!js.count("count")) throw runtime_error("missing required variable");
    parse_attr(val->count, "count", js, err);
    if (!js.count("indices")) throw runtime_error("missing required variable");
    parse_attr(val->indices, "indices", js, err);
    if (!js.count("values")) throw runtime_error("missing required variable");
    parse_attr(val->values, "values", js, err);
}
// Parse a glTFAccessorComponentType enum
inline void parse(
    glTFAccessorComponentType& val, const json& js, parse_stack& err) {
    static map<int, glTFAccessorComponentType> table = {
        {5120, glTFAccessorComponentType::Byte},
        {5121, glTFAccessorComponentType::UnsignedByte},
        {5122, glTFAccessorComponentType::Short},
        {5123, glTFAccessorComponentType::UnsignedShort},
        {5125, glTFAccessorComponentType::UnsignedInt},
        {5126, glTFAccessorComponentType::Float},
    };
    auto v = int();
    parse(v, js, err);
    if (table.find(v) == table.end()) throw runtime_error("bad enum value");
    val = table[v];
}

// Parse a glTFAccessorType enum
inline void parse(glTFAccessorType& val, const json& js, parse_stack& err) {
    static map<string, glTFAccessorType> table = {
        {"SCALAR", glTFAccessorType::Scalar},
        {"VEC2", glTFAccessorType::Vec2},
        {"VEC3", glTFAccessorType::Vec3},
        {"VEC4", glTFAccessorType::Vec4},
        {"MAT2", glTFAccessorType::Mat2},
        {"MAT3", glTFAccessorType::Mat3},
        {"MAT4", glTFAccessorType::Mat4},
    };
    auto v = string();
    parse(v, js, err);
    if (table.find(v) == table.end()) throw runtime_error("bad enum value");
    val = table[v];
}

// Parses a glTFAccessor object
inline void parse(glTFAccessor*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFAccessor();
    parse((glTFChildOfRootProperty*&)val, js, err);
    parse_attr(val->bufferView, "bufferView", js, err);
    parse_attr(val->byteOffset, "byteOffset", js, err);
    if (!js.count("componentType"))
        throw runtime_error("missing required variable");
    parse_attr(val->componentType, "componentType", js, err);
    parse_attr(val->normalized, "normalized", js, err);
    if (!js.count("count")) throw runtime_error("missing required variable");
    parse_attr(val->count, "count", js, err);
    if (!js.count("type")) throw runtime_error("missing required variable");
    parse_attr(val->type, "type", js, err);
    parse_attr(val->max, "max", js, err);
    parse_attr(val->min, "min", js, err);
    parse_attr(val->sparse, "sparse", js, err);
}
// Parse a glTFAnimationChannelTargetPath enum
inline void parse(
    glTFAnimationChannelTargetPath& val, const json& js, parse_stack& err) {
    static map<string, glTFAnimationChannelTargetPath> table = {
        {"translation", glTFAnimationChannelTargetPath::Translation},
        {"rotation", glTFAnimationChannelTargetPath::Rotation},
        {"scale", glTFAnimationChannelTargetPath::Scale},
        {"weights", glTFAnimationChannelTargetPath::Weights},
    };
    auto v = string();
    parse(v, js, err);
    if (table.find(v) == table.end()) throw runtime_error("bad enum value");
    val = table[v];
}

// Parses a glTFAnimationChannelTarget object
inline void parse(
    glTFAnimationChannelTarget*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFAnimationChannelTarget();
    parse((glTFProperty*&)val, js, err);
    if (!js.count("node")) throw runtime_error("missing required variable");
    parse_attr(val->node, "node", js, err);
    if (!js.count("path")) throw runtime_error("missing required variable");
    parse_attr(val->path, "path", js, err);
}

// Parses a glTFAnimationChannel object
inline void parse(
    glTFAnimationChannel*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFAnimationChannel();
    parse((glTFProperty*&)val, js, err);
    if (!js.count("sampler")) throw runtime_error("missing required variable");
    parse_attr(val->sampler, "sampler", js, err);
    if (!js.count("target")) throw runtime_error("missing required variable");
    parse_attr(val->target, "target", js, err);
}
// Parse a glTFAnimationSamplerInterpolation enum
inline void parse(
    glTFAnimationSamplerInterpolation& val, const json& js, parse_stack& err) {
    static map<string, glTFAnimationSamplerInterpolation> table = {
        {"LINEAR", glTFAnimationSamplerInterpolation::Linear},
        {"STEP", glTFAnimationSamplerInterpolation::Step},
        {"CATMULLROMSPLINE",
            glTFAnimationSamplerInterpolation::CatmullRomSpline},
        {"CUBICSPLINE", glTFAnimationSamplerInterpolation::CubicSpline},
    };
    auto v = string();
    parse(v, js, err);
    if (table.find(v) == table.end()) throw runtime_error("bad enum value");
    val = table[v];
}

// Parses a glTFAnimationSampler object
inline void parse(
    glTFAnimationSampler*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFAnimationSampler();
    parse((glTFProperty*&)val, js, err);
    if (!js.count("input")) throw runtime_error("missing required variable");
    parse_attr(val->input, "input", js, err);
    parse_attr(val->interpolation, "interpolation", js, err);
    if (!js.count("output")) throw runtime_error("missing required variable");
    parse_attr(val->output, "output", js, err);
}

// Parses a glTFAnimation object
inline void parse(glTFAnimation*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFAnimation();
    parse((glTFChildOfRootProperty*&)val, js, err);
    if (!js.count("channels")) throw runtime_error("missing required variable");
    parse_attr(val->channels, "channels", js, err);
    if (!js.count("samplers")) throw runtime_error("missing required variable");
    parse_attr(val->samplers, "samplers", js, err);
}

// Parses a glTFAsset object
inline void parse(glTFAsset*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFAsset();
    parse((glTFProperty*&)val, js, err);
    parse_attr(val->copyright, "copyright", js, err);
    parse_attr(val->generator, "generator", js, err);
    if (!js.count("version")) throw runtime_error("missing required variable");
    parse_attr(val->version, "version", js, err);
    parse_attr(val->minVersion, "minVersion", js, err);
}

// Parses a glTFBuffer object
inline void parse(glTFBuffer*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFBuffer();
    parse((glTFChildOfRootProperty*&)val, js, err);
    parse_attr(val->uri, "uri", js, err);
    if (!js.count("byteLength"))
        throw runtime_error("missing required variable");
    parse_attr(val->byteLength, "byteLength", js, err);
}
// Parse a glTFBufferViewTarget enum
inline void parse(glTFBufferViewTarget& val, const json& js, parse_stack& err) {
    static map<int, glTFBufferViewTarget> table = {
        {34962, glTFBufferViewTarget::ArrayBuffer},
        {34963, glTFBufferViewTarget::ElementArrayBuffer},
    };
    auto v = int();
    parse(v, js, err);
    if (table.find(v) == table.end()) throw runtime_error("bad enum value");
    val = table[v];
}

// Parses a glTFBufferView object
inline void parse(glTFBufferView*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFBufferView();
    parse((glTFChildOfRootProperty*&)val, js, err);
    if (!js.count("buffer")) throw runtime_error("missing required variable");
    parse_attr(val->buffer, "buffer", js, err);
    parse_attr(val->byteOffset, "byteOffset", js, err);
    if (!js.count("byteLength"))
        throw runtime_error("missing required variable");
    parse_attr(val->byteLength, "byteLength", js, err);
    parse_attr(val->byteStride, "byteStride", js, err);
    parse_attr(val->target, "target", js, err);
}

// Parses a glTFCameraOrthographic object
inline void parse(
    glTFCameraOrthographic*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFCameraOrthographic();
    parse((glTFProperty*&)val, js, err);
    if (!js.count("xmag")) throw runtime_error("missing required variable");
    parse_attr(val->xmag, "xmag", js, err);
    if (!js.count("ymag")) throw runtime_error("missing required variable");
    parse_attr(val->ymag, "ymag", js, err);
    if (!js.count("zfar")) throw runtime_error("missing required variable");
    parse_attr(val->zfar, "zfar", js, err);
    if (!js.count("znear")) throw runtime_error("missing required variable");
    parse_attr(val->znear, "znear", js, err);
}

// Parses a glTFCameraPerspective object
inline void parse(
    glTFCameraPerspective*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFCameraPerspective();
    parse((glTFProperty*&)val, js, err);
    parse_attr(val->aspectRatio, "aspectRatio", js, err);
    if (!js.count("yfov")) throw runtime_error("missing required variable");
    parse_attr(val->yfov, "yfov", js, err);
    parse_attr(val->zfar, "zfar", js, err);
    if (!js.count("znear")) throw runtime_error("missing required variable");
    parse_attr(val->znear, "znear", js, err);
}
// Parse a glTFCameraType enum
inline void parse(glTFCameraType& val, const json& js, parse_stack& err) {
    static map<string, glTFCameraType> table = {
        {"perspective", glTFCameraType::Perspective},
        {"orthographic", glTFCameraType::Orthographic},
    };
    auto v = string();
    parse(v, js, err);
    if (table.find(v) == table.end()) throw runtime_error("bad enum value");
    val = table[v];
}

// Parses a glTFCamera object
inline void parse(glTFCamera*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFCamera();
    parse((glTFChildOfRootProperty*&)val, js, err);
    parse_attr(val->orthographic, "orthographic", js, err);
    parse_attr(val->perspective, "perspective", js, err);
    if (!js.count("type")) throw runtime_error("missing required variable");
    parse_attr(val->type, "type", js, err);
}
// Parse a glTFImageMimeType enum
inline void parse(glTFImageMimeType& val, const json& js, parse_stack& err) {
    static map<string, glTFImageMimeType> table = {
        {"image/jpeg", glTFImageMimeType::ImageJpeg},
        {"image/png", glTFImageMimeType::ImagePng},
    };
    auto v = string();
    parse(v, js, err);
    if (table.find(v) == table.end()) throw runtime_error("bad enum value");
    val = table[v];
}

// Parses a glTFImage object
inline void parse(glTFImage*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFImage();
    parse((glTFChildOfRootProperty*&)val, js, err);
    parse_attr(val->uri, "uri", js, err);
    parse_attr(val->mimeType, "mimeType", js, err);
    parse_attr(val->bufferView, "bufferView", js, err);
}

// Parses a glTFTextureInfo object
inline void parse(glTFTextureInfo*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFTextureInfo();
    parse((glTFProperty*&)val, js, err);
    if (!js.count("index")) throw runtime_error("missing required variable");
    parse_attr(val->index, "index", js, err);
    parse_attr(val->texCoord, "texCoord", js, err);
}

// Parses a glTFTexture object
inline void parse(glTFTexture*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFTexture();
    parse((glTFChildOfRootProperty*&)val, js, err);
    parse_attr(val->sampler, "sampler", js, err);
    parse_attr(val->source, "source", js, err);
}

// Parses a glTFMaterialNormalTextureInfo object
inline void parse(
    glTFMaterialNormalTextureInfo*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFMaterialNormalTextureInfo();
    parse((glTFTextureInfo*&)val, js, err);
    parse_attr(val->scale, "scale", js, err);
}

// Parses a glTFMaterialOcclusionTextureInfo object
inline void parse(
    glTFMaterialOcclusionTextureInfo*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFMaterialOcclusionTextureInfo();
    parse((glTFTextureInfo*&)val, js, err);
    parse_attr(val->strength, "strength", js, err);
}

// Parses a glTFMaterialPbrMetallicRoughness object
inline void parse(
    glTFMaterialPbrMetallicRoughness*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFMaterialPbrMetallicRoughness();
    parse((glTFProperty*&)val, js, err);
    parse_attr(val->baseColorFactor, "baseColorFactor", js, err);
    parse_attr(val->baseColorTexture, "baseColorTexture", js, err);
    parse_attr(val->metallicFactor, "metallicFactor", js, err);
    parse_attr(val->roughnessFactor, "roughnessFactor", js, err);
    parse_attr(
        val->metallicRoughnessTexture, "metallicRoughnessTexture", js, err);
}

// Parses a glTFMaterialPbrSpecularGlossiness object
inline void parse(
    glTFMaterialPbrSpecularGlossiness*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFMaterialPbrSpecularGlossiness();
    parse((glTFProperty*&)val, js, err);
    parse_attr(val->diffuseFactor, "diffuseFactor", js, err);
    parse_attr(val->diffuseTexture, "diffuseTexture", js, err);
    parse_attr(val->specularFactor, "specularFactor", js, err);
    parse_attr(val->glossinessFactor, "glossinessFactor", js, err);
    parse_attr(
        val->specularGlossinessTexture, "specularGlossinessTexture", js, err);
}
// Parse a glTFMaterialAlphaMode enum
inline void parse(
    glTFMaterialAlphaMode& val, const json& js, parse_stack& err) {
    static map<string, glTFMaterialAlphaMode> table = {
        {"OPAQUE", glTFMaterialAlphaMode::Opaque},
        {"MASK", glTFMaterialAlphaMode::Mask},
        {"BLEND", glTFMaterialAlphaMode::Blend},
    };
    auto v = string();
    parse(v, js, err);
    if (table.find(v) == table.end()) throw runtime_error("bad enum value");
    val = table[v];
}

// Parses a glTFMaterial object
inline void parse(glTFMaterial*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFMaterial();
    parse((glTFChildOfRootProperty*&)val, js, err);
    parse_attr(val->pbrMetallicRoughness, "pbrMetallicRoughness", js, err);
    parse_attr(val->normalTexture, "normalTexture", js, err);
    parse_attr(val->occlusionTexture, "occlusionTexture", js, err);
    parse_attr(val->emissiveTexture, "emissiveTexture", js, err);
    parse_attr(val->emissiveFactor, "emissiveFactor", js, err);
    parse_attr(val->alphaMode, "alphaMode", js, err);
    parse_attr(val->alphaCutoff, "alphaCutoff", js, err);
    parse_attr(val->doubleSided, "doubleSided", js, err);
    if (js.count("extensions")) {
        auto& js_ext = js["extensions"];
        parse_attr(val->pbrSpecularGlossiness,
            "KHR_materials_pbrSpecularGlossiness", js_ext, err);
    }
}
// Parse a glTFMeshPrimitiveMode enum
inline void parse(
    glTFMeshPrimitiveMode& val, const json& js, parse_stack& err) {
    static map<int, glTFMeshPrimitiveMode> table = {
        {0, glTFMeshPrimitiveMode::Points},
        {1, glTFMeshPrimitiveMode::Lines},
        {2, glTFMeshPrimitiveMode::LineLoop},
        {3, glTFMeshPrimitiveMode::LineStrip},
        {4, glTFMeshPrimitiveMode::Triangles},
        {5, glTFMeshPrimitiveMode::TriangleStrip},
        {6, glTFMeshPrimitiveMode::TriangleFan},
    };
    auto v = int();
    parse(v, js, err);
    if (table.find(v) == table.end()) throw runtime_error("bad enum value");
    val = table[v];
}

// Parses a glTFMeshPrimitive object
inline void parse(glTFMeshPrimitive*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFMeshPrimitive();
    parse((glTFProperty*&)val, js, err);
    if (!js.count("attributes"))
        throw runtime_error("missing required variable");
    parse_attr(val->attributes, "attributes", js, err);
    parse_attr(val->indices, "indices", js, err);
    parse_attr(val->material, "material", js, err);
    parse_attr(val->mode, "mode", js, err);
    parse_attr(val->targets, "targets", js, err);
}

// Parses a glTFMesh object
inline void parse(glTFMesh*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFMesh();
    parse((glTFChildOfRootProperty*&)val, js, err);
    if (!js.count("primitives"))
        throw runtime_error("missing required variable");
    parse_attr(val->primitives, "primitives", js, err);
    parse_attr(val->weights, "weights", js, err);
}

// Parses a glTFNode object
inline void parse(glTFNode*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFNode();
    parse((glTFChildOfRootProperty*&)val, js, err);
    parse_attr(val->camera, "camera", js, err);
    parse_attr(val->children, "children", js, err);
    parse_attr(val->skin, "skin", js, err);
    parse_attr(val->matrix, "matrix", js, err);
    parse_attr(val->mesh, "mesh", js, err);
    parse_attr(val->rotation, "rotation", js, err);
    parse_attr(val->scale, "scale", js, err);
    parse_attr(val->translation, "translation", js, err);
    parse_attr(val->weights, "weights", js, err);
}
// Parse a glTFSamplerMagFilter enum
inline void parse(glTFSamplerMagFilter& val, const json& js, parse_stack& err) {
    static map<int, glTFSamplerMagFilter> table = {
        {9728, glTFSamplerMagFilter::Nearest},
        {9729, glTFSamplerMagFilter::Linear},
    };
    auto v = int();
    parse(v, js, err);
    if (table.find(v) == table.end()) throw runtime_error("bad enum value");
    val = table[v];
}

// Parse a glTFSamplerMinFilter enum
inline void parse(glTFSamplerMinFilter& val, const json& js, parse_stack& err) {
    static map<int, glTFSamplerMinFilter> table = {
        {9728, glTFSamplerMinFilter::Nearest},
        {9729, glTFSamplerMinFilter::Linear},
        {9984, glTFSamplerMinFilter::NearestMipmapNearest},
        {9985, glTFSamplerMinFilter::LinearMipmapNearest},
        {9986, glTFSamplerMinFilter::NearestMipmapLinear},
        {9987, glTFSamplerMinFilter::LinearMipmapLinear},
    };
    auto v = int();
    parse(v, js, err);
    if (table.find(v) == table.end()) throw runtime_error("bad enum value");
    val = table[v];
}

// Parse a glTFSamplerWrapS enum
inline void parse(glTFSamplerWrapS& val, const json& js, parse_stack& err) {
    static map<int, glTFSamplerWrapS> table = {
        {33071, glTFSamplerWrapS::ClampToEdge},
        {33648, glTFSamplerWrapS::MirroredRepeat},
        {10497, glTFSamplerWrapS::Repeat},
    };
    auto v = int();
    parse(v, js, err);
    if (table.find(v) == table.end()) throw runtime_error("bad enum value");
    val = table[v];
}

// Parse a glTFSamplerWrapT enum
inline void parse(glTFSamplerWrapT& val, const json& js, parse_stack& err) {
    static map<int, glTFSamplerWrapT> table = {
        {33071, glTFSamplerWrapT::ClampToEdge},
        {33648, glTFSamplerWrapT::MirroredRepeat},
        {10497, glTFSamplerWrapT::Repeat},
    };
    auto v = int();
    parse(v, js, err);
    if (table.find(v) == table.end()) throw runtime_error("bad enum value");
    val = table[v];
}

// Parses a glTFSampler object
inline void parse(glTFSampler*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFSampler();
    parse((glTFChildOfRootProperty*&)val, js, err);
    parse_attr(val->magFilter, "magFilter", js, err);
    parse_attr(val->minFilter, "minFilter", js, err);
    parse_attr(val->wrapS, "wrapS", js, err);
    parse_attr(val->wrapT, "wrapT", js, err);
}

// Parses a glTFScene object
inline void parse(glTFScene*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFScene();
    parse((glTFChildOfRootProperty*&)val, js, err);
    parse_attr(val->nodes, "nodes", js, err);
}

// Parses a glTFSkin object
inline void parse(glTFSkin*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTFSkin();
    parse((glTFChildOfRootProperty*&)val, js, err);
    parse_attr(val->inverseBindMatrices, "inverseBindMatrices", js, err);
    parse_attr(val->skeleton, "skeleton", js, err);
    if (!js.count("joints")) throw runtime_error("missing required variable");
    parse_attr(val->joints, "joints", js, err);
}

// Parses a glTF object
inline void parse(glTF*& val, const json& js, parse_stack& err) {
    if (!js.is_object()) throw runtime_error("object expected");
    if (!val) val = new glTF();
    parse((glTFProperty*&)val, js, err);
    parse_attr(val->extensionsUsed, "extensionsUsed", js, err);
    parse_attr(val->extensionsRequired, "extensionsRequired", js, err);
    parse_attr(val->accessors, "accessors", js, err);
    parse_attr(val->animations, "animations", js, err);
    if (!js.count("asset")) throw runtime_error("missing required variable");
    parse_attr(val->asset, "asset", js, err);
    parse_attr(val->buffers, "buffers", js, err);
    parse_attr(val->bufferViews, "bufferViews", js, err);
    parse_attr(val->cameras, "cameras", js, err);
    parse_attr(val->images, "images", js, err);
    parse_attr(val->materials, "materials", js, err);
    parse_attr(val->meshes, "meshes", js, err);
    parse_attr(val->nodes, "nodes", js, err);
    parse_attr(val->samplers, "samplers", js, err);
    parse_attr(val->scene, "scene", js, err);
    parse_attr(val->scenes, "scenes", js, err);
    parse_attr(val->skins, "skins", js, err);
    parse_attr(val->textures, "textures", js, err);
}

// Dump support function.
template <typename T>
inline void dump(const vector<T>& vals, json& js, parse_stack& err) {
    js = json::array();
    for (auto i = 0; i < vals.size(); i++) { dump(vals[i], js[i], err); }
}

// Dump support function.
template <typename T, int N>
inline void dump(const vec<T, N>& vals, json& js, parse_stack& err) {
    js = json::array();
    for (auto i = 0; i < N; i++) { dump(vals[i], js[i], err); }
}

// Dump support function.
template <typename T, int N>
inline void dump(const quat<T, N>& vals, json& js, parse_stack& err) {
    js = json::array();
    for (auto i = 0; i < N; i++) { dump(vals[i], js[i], err); }
}

// Dump support function.
template <typename T, int N, int M>
inline void dump(const mat<T, N, M>& vals, json& js, parse_stack& err) {
    js = json::array();
    for (auto j = 0; j < M; j++) {
        for (auto i = 0; i < N; i++) { dump(vals[j][i], js[j * N + i], err); }
    }
}

// Dump support function.
template <typename T>
inline void dump(const map<string, T>& vals, json& js, parse_stack& err) {
    js = json::object();
    for (auto&& kv : vals) { dump(kv.second, js[kv.first], err); }
}

// Dump support function.
template <typename T>
inline void dump_attr(
    const T& val, const char* name, json& js, parse_stack& err) {
    err.path.push_back(name);
    dump(val, js[name], err);
    err.path.pop_back();
}

// Converts int to json.
inline void dump(const int& val, json& js, parse_stack& err) { js = val; }

// Converts float to json.
inline void dump(const float& val, json& js, parse_stack& err) { js = val; }

// Converts bool to json.
inline void dump(const bool& val, json& js, parse_stack& err) { js = val; }

// Converts string to json.
inline void dump(const string& val, json& js, parse_stack& err) { js = val; }

// Converts json to json.
inline void dump(const json& val, json& js, parse_stack& err) { js = val; }

// Converts __TYPE__ to json.
template <typename T>
inline void dump(const glTFid<T>& val, json& js, parse_stack& err) {
    js = (int)val;
}

// Converts a glTFProperty object to JSON
inline void dump(const glTFProperty* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();

    if (!val->extensions.empty())
        dump_attr(val->extensions, "extensions", js, err);
    if (!val->extras.is_null()) dump_attr(val->extras, "extras", js, err);
}

// Converts a glTFChildOfRootProperty object to JSON
inline void dump(
    const glTFChildOfRootProperty* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFProperty*)val, js, err);
    if (val->name != "") dump_attr(val->name, "name", js, err);
}
// Converts a glTFAccessorSparseIndicesComponentType enum to JSON
inline void dump(const glTFAccessorSparseIndicesComponentType& val, json& js,
    parse_stack& err) {
    static map<glTFAccessorSparseIndicesComponentType, int> table = {
        {glTFAccessorSparseIndicesComponentType::UnsignedByte, 5121},
        {glTFAccessorSparseIndicesComponentType::UnsignedShort, 5123},
        {glTFAccessorSparseIndicesComponentType::UnsignedInt, 5125},
    };
    auto v = table.at(val);
    dump(v, js, err);
}

// Converts a glTFAccessorSparseIndices object to JSON
inline void dump(
    const glTFAccessorSparseIndices* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFProperty*)val, js, err);
    dump_attr(val->bufferView, "bufferView", js, err);
    if (val->byteOffset != 0) dump_attr(val->byteOffset, "byteOffset", js, err);
    dump_attr(val->componentType, "componentType", js, err);
}

// Converts a glTFAccessorSparseValues object to JSON
inline void dump(
    const glTFAccessorSparseValues* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFProperty*)val, js, err);
    dump_attr(val->bufferView, "bufferView", js, err);
    if (val->byteOffset != 0) dump_attr(val->byteOffset, "byteOffset", js, err);
}

// Converts a glTFAccessorSparse object to JSON
inline void dump(const glTFAccessorSparse* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFProperty*)val, js, err);
    dump_attr(val->count, "count", js, err);
    dump_attr(val->indices, "indices", js, err);
    dump_attr(val->values, "values", js, err);
}
// Converts a glTFAccessorComponentType enum to JSON
inline void dump(
    const glTFAccessorComponentType& val, json& js, parse_stack& err) {
    static map<glTFAccessorComponentType, int> table = {
        {glTFAccessorComponentType::Byte, 5120},
        {glTFAccessorComponentType::UnsignedByte, 5121},
        {glTFAccessorComponentType::Short, 5122},
        {glTFAccessorComponentType::UnsignedShort, 5123},
        {glTFAccessorComponentType::UnsignedInt, 5125},
        {glTFAccessorComponentType::Float, 5126},
    };
    auto v = table.at(val);
    dump(v, js, err);
}

// Converts a glTFAccessorType enum to JSON
inline void dump(const glTFAccessorType& val, json& js, parse_stack& err) {
    static map<glTFAccessorType, string> table = {
        {glTFAccessorType::Scalar, "SCALAR"},
        {glTFAccessorType::Vec2, "VEC2"},
        {glTFAccessorType::Vec3, "VEC3"},
        {glTFAccessorType::Vec4, "VEC4"},
        {glTFAccessorType::Mat2, "MAT2"},
        {glTFAccessorType::Mat3, "MAT3"},
        {glTFAccessorType::Mat4, "MAT4"},
    };
    auto v = table.at(val);
    dump(v, js, err);
}

// Converts a glTFAccessor object to JSON
inline void dump(const glTFAccessor* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFChildOfRootProperty*)val, js, err);
    if (val->bufferView.is_valid())
        dump_attr(val->bufferView, "bufferView", js, err);
    if (val->byteOffset != 0) dump_attr(val->byteOffset, "byteOffset", js, err);
    dump_attr(val->componentType, "componentType", js, err);
    if (val->normalized != false)
        dump_attr(val->normalized, "normalized", js, err);
    dump_attr(val->count, "count", js, err);
    dump_attr(val->type, "type", js, err);
    if (!val->max.empty()) dump_attr(val->max, "max", js, err);
    if (!val->min.empty()) dump_attr(val->min, "min", js, err);
    if (val->sparse != nullptr) dump_attr(val->sparse, "sparse", js, err);
}
// Converts a glTFAnimationChannelTargetPath enum to JSON
inline void dump(
    const glTFAnimationChannelTargetPath& val, json& js, parse_stack& err) {
    static map<glTFAnimationChannelTargetPath, string> table = {
        {glTFAnimationChannelTargetPath::Translation, "translation"},
        {glTFAnimationChannelTargetPath::Rotation, "rotation"},
        {glTFAnimationChannelTargetPath::Scale, "scale"},
        {glTFAnimationChannelTargetPath::Weights, "weights"},
    };
    auto v = table.at(val);
    dump(v, js, err);
}

// Converts a glTFAnimationChannelTarget object to JSON
inline void dump(
    const glTFAnimationChannelTarget* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFProperty*)val, js, err);
    dump_attr(val->node, "node", js, err);
    dump_attr(val->path, "path", js, err);
}

// Converts a glTFAnimationChannel object to JSON
inline void dump(const glTFAnimationChannel* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFProperty*)val, js, err);
    dump_attr(val->sampler, "sampler", js, err);
    dump_attr(val->target, "target", js, err);
}
// Converts a glTFAnimationSamplerInterpolation enum to JSON
inline void dump(
    const glTFAnimationSamplerInterpolation& val, json& js, parse_stack& err) {
    static map<glTFAnimationSamplerInterpolation, string> table = {
        {glTFAnimationSamplerInterpolation::Linear, "LINEAR"},
        {glTFAnimationSamplerInterpolation::Step, "STEP"},
        {glTFAnimationSamplerInterpolation::CatmullRomSpline,
            "CATMULLROMSPLINE"},
        {glTFAnimationSamplerInterpolation::CubicSpline, "CUBICSPLINE"},
    };
    auto v = table.at(val);
    dump(v, js, err);
}

// Converts a glTFAnimationSampler object to JSON
inline void dump(const glTFAnimationSampler* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFProperty*)val, js, err);
    dump_attr(val->input, "input", js, err);
    if (val->interpolation != glTFAnimationSamplerInterpolation::Linear)
        dump_attr(val->interpolation, "interpolation", js, err);
    dump_attr(val->output, "output", js, err);
}

// Converts a glTFAnimation object to JSON
inline void dump(const glTFAnimation* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFChildOfRootProperty*)val, js, err);
    dump_attr(val->channels, "channels", js, err);
    dump_attr(val->samplers, "samplers", js, err);
}

// Converts a glTFAsset object to JSON
inline void dump(const glTFAsset* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFProperty*)val, js, err);
    if (val->copyright != "") dump_attr(val->copyright, "copyright", js, err);
    if (val->generator != "") dump_attr(val->generator, "generator", js, err);
    dump_attr(val->version, "version", js, err);
    if (val->minVersion != "")
        dump_attr(val->minVersion, "minVersion", js, err);
}

// Converts a glTFBuffer object to JSON
inline void dump(const glTFBuffer* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFChildOfRootProperty*)val, js, err);
    if (val->uri != "") dump_attr(val->uri, "uri", js, err);
    dump_attr(val->byteLength, "byteLength", js, err);
}
// Converts a glTFBufferViewTarget enum to JSON
inline void dump(const glTFBufferViewTarget& val, json& js, parse_stack& err) {
    static map<glTFBufferViewTarget, int> table = {
        {glTFBufferViewTarget::ArrayBuffer, 34962},
        {glTFBufferViewTarget::ElementArrayBuffer, 34963},
    };
    auto v = table.at(val);
    dump(v, js, err);
}

// Converts a glTFBufferView object to JSON
inline void dump(const glTFBufferView* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFChildOfRootProperty*)val, js, err);
    dump_attr(val->buffer, "buffer", js, err);
    if (val->byteOffset != 0) dump_attr(val->byteOffset, "byteOffset", js, err);
    dump_attr(val->byteLength, "byteLength", js, err);
    if (val->byteStride != 0) dump_attr(val->byteStride, "byteStride", js, err);
    if (val->target != glTFBufferViewTarget::NotSet)
        dump_attr(val->target, "target", js, err);
}

// Converts a glTFCameraOrthographic object to JSON
inline void dump(
    const glTFCameraOrthographic* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFProperty*)val, js, err);
    dump_attr(val->xmag, "xmag", js, err);
    dump_attr(val->ymag, "ymag", js, err);
    dump_attr(val->zfar, "zfar", js, err);
    dump_attr(val->znear, "znear", js, err);
}

// Converts a glTFCameraPerspective object to JSON
inline void dump(const glTFCameraPerspective* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFProperty*)val, js, err);
    if (val->aspectRatio != 0)
        dump_attr(val->aspectRatio, "aspectRatio", js, err);
    dump_attr(val->yfov, "yfov", js, err);
    if (val->zfar != 0) dump_attr(val->zfar, "zfar", js, err);
    dump_attr(val->znear, "znear", js, err);
}
// Converts a glTFCameraType enum to JSON
inline void dump(const glTFCameraType& val, json& js, parse_stack& err) {
    static map<glTFCameraType, string> table = {
        {glTFCameraType::Perspective, "perspective"},
        {glTFCameraType::Orthographic, "orthographic"},
    };
    auto v = table.at(val);
    dump(v, js, err);
}

// Converts a glTFCamera object to JSON
inline void dump(const glTFCamera* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFChildOfRootProperty*)val, js, err);
    if (val->orthographic != nullptr)
        dump_attr(val->orthographic, "orthographic", js, err);
    if (val->perspective != nullptr)
        dump_attr(val->perspective, "perspective", js, err);
    dump_attr(val->type, "type", js, err);
}
// Converts a glTFImageMimeType enum to JSON
inline void dump(const glTFImageMimeType& val, json& js, parse_stack& err) {
    static map<glTFImageMimeType, string> table = {
        {glTFImageMimeType::ImageJpeg, "image/jpeg"},
        {glTFImageMimeType::ImagePng, "image/png"},
    };
    auto v = table.at(val);
    dump(v, js, err);
}

// Converts a glTFImage object to JSON
inline void dump(const glTFImage* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFChildOfRootProperty*)val, js, err);
    if (val->uri != "") dump_attr(val->uri, "uri", js, err);
    if (val->mimeType != glTFImageMimeType::NotSet)
        dump_attr(val->mimeType, "mimeType", js, err);
    if (val->bufferView.is_valid())
        dump_attr(val->bufferView, "bufferView", js, err);
}

// Converts a glTFTextureInfo object to JSON
inline void dump(const glTFTextureInfo* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFProperty*)val, js, err);
    dump_attr(val->index, "index", js, err);
    if (val->texCoord != 0) dump_attr(val->texCoord, "texCoord", js, err);
}

// Converts a glTFTexture object to JSON
inline void dump(const glTFTexture* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFChildOfRootProperty*)val, js, err);
    if (val->sampler.is_valid()) dump_attr(val->sampler, "sampler", js, err);
    if (val->source.is_valid()) dump_attr(val->source, "source", js, err);
}

// Converts a glTFMaterialNormalTextureInfo object to JSON
inline void dump(
    const glTFMaterialNormalTextureInfo* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFTextureInfo*)val, js, err);
    if (val->scale != 1) dump_attr(val->scale, "scale", js, err);
}

// Converts a glTFMaterialOcclusionTextureInfo object to JSON
inline void dump(
    const glTFMaterialOcclusionTextureInfo* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFTextureInfo*)val, js, err);
    if (val->strength != 1) dump_attr(val->strength, "strength", js, err);
}

// Converts a glTFMaterialPbrMetallicRoughness object to JSON
inline void dump(
    const glTFMaterialPbrMetallicRoughness* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFProperty*)val, js, err);
    if (val->baseColorFactor != vec4f{1, 1, 1, 1})
        dump_attr(val->baseColorFactor, "baseColorFactor", js, err);
    if (val->baseColorTexture != nullptr)
        dump_attr(val->baseColorTexture, "baseColorTexture", js, err);
    if (val->metallicFactor != 1)
        dump_attr(val->metallicFactor, "metallicFactor", js, err);
    if (val->roughnessFactor != 1)
        dump_attr(val->roughnessFactor, "roughnessFactor", js, err);
    if (val->metallicRoughnessTexture != nullptr)
        dump_attr(
            val->metallicRoughnessTexture, "metallicRoughnessTexture", js, err);
}

// Converts a glTFMaterialPbrSpecularGlossiness object to JSON
inline void dump(
    const glTFMaterialPbrSpecularGlossiness* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFProperty*)val, js, err);
    if (val->diffuseFactor != vec4f{1, 1, 1, 1})
        dump_attr(val->diffuseFactor, "diffuseFactor", js, err);
    if (val->diffuseTexture != nullptr)
        dump_attr(val->diffuseTexture, "diffuseTexture", js, err);
    if (val->specularFactor != vec3f{1, 1, 1})
        dump_attr(val->specularFactor, "specularFactor", js, err);
    if (val->glossinessFactor != 1)
        dump_attr(val->glossinessFactor, "glossinessFactor", js, err);
    if (val->specularGlossinessTexture != nullptr)
        dump_attr(val->specularGlossinessTexture, "specularGlossinessTexture",
            js, err);
}
// Converts a glTFMaterialAlphaMode enum to JSON
inline void dump(const glTFMaterialAlphaMode& val, json& js, parse_stack& err) {
    static map<glTFMaterialAlphaMode, string> table = {
        {glTFMaterialAlphaMode::Opaque, "OPAQUE"},
        {glTFMaterialAlphaMode::Mask, "MASK"},
        {glTFMaterialAlphaMode::Blend, "BLEND"},
    };
    auto v = table.at(val);
    dump(v, js, err);
}

// Converts a glTFMaterial object to JSON
inline void dump(const glTFMaterial* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFChildOfRootProperty*)val, js, err);
    if (val->pbrMetallicRoughness != nullptr)
        dump_attr(val->pbrMetallicRoughness, "pbrMetallicRoughness", js, err);
    if (val->normalTexture != nullptr)
        dump_attr(val->normalTexture, "normalTexture", js, err);
    if (val->occlusionTexture != nullptr)
        dump_attr(val->occlusionTexture, "occlusionTexture", js, err);
    if (val->emissiveTexture != nullptr)
        dump_attr(val->emissiveTexture, "emissiveTexture", js, err);
    if (val->emissiveFactor != vec3f{0, 0, 0})
        dump_attr(val->emissiveFactor, "emissiveFactor", js, err);
    if (val->alphaMode != glTFMaterialAlphaMode::Opaque)
        dump_attr(val->alphaMode, "alphaMode", js, err);
    if (val->alphaCutoff != 0.5)
        dump_attr(val->alphaCutoff, "alphaCutoff", js, err);
    if (val->doubleSided != false)
        dump_attr(val->doubleSided, "doubleSided", js, err);

    if (val->pbrSpecularGlossiness != nullptr) {
        auto& js_ext = js["extensions"];
        dump_attr(val->pbrSpecularGlossiness,
            "KHR_materials_pbrSpecularGlossiness", js_ext, err);
    }
}
// Converts a glTFMeshPrimitiveMode enum to JSON
inline void dump(const glTFMeshPrimitiveMode& val, json& js, parse_stack& err) {
    static map<glTFMeshPrimitiveMode, int> table = {
        {glTFMeshPrimitiveMode::Points, 0},
        {glTFMeshPrimitiveMode::Lines, 1},
        {glTFMeshPrimitiveMode::LineLoop, 2},
        {glTFMeshPrimitiveMode::LineStrip, 3},
        {glTFMeshPrimitiveMode::Triangles, 4},
        {glTFMeshPrimitiveMode::TriangleStrip, 5},
        {glTFMeshPrimitiveMode::TriangleFan, 6},
    };
    auto v = table.at(val);
    dump(v, js, err);
}

// Converts a glTFMeshPrimitive object to JSON
inline void dump(const glTFMeshPrimitive* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFProperty*)val, js, err);
    dump_attr(val->attributes, "attributes", js, err);
    if (val->indices.is_valid()) dump_attr(val->indices, "indices", js, err);
    if (val->material.is_valid()) dump_attr(val->material, "material", js, err);
    if (val->mode != glTFMeshPrimitiveMode::Triangles)
        dump_attr(val->mode, "mode", js, err);
    if (!val->targets.empty()) dump_attr(val->targets, "targets", js, err);
}

// Converts a glTFMesh object to JSON
inline void dump(const glTFMesh* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFChildOfRootProperty*)val, js, err);
    dump_attr(val->primitives, "primitives", js, err);
    if (!val->weights.empty()) dump_attr(val->weights, "weights", js, err);
}

// Converts a glTFNode object to JSON
inline void dump(const glTFNode* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFChildOfRootProperty*)val, js, err);
    if (val->camera.is_valid()) dump_attr(val->camera, "camera", js, err);
    if (!val->children.empty()) dump_attr(val->children, "children", js, err);
    if (val->skin.is_valid()) dump_attr(val->skin, "skin", js, err);
    if (val->matrix !=
        mat4f{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}})
        dump_attr(val->matrix, "matrix", js, err);
    if (val->mesh.is_valid()) dump_attr(val->mesh, "mesh", js, err);
    if (val->rotation != quat4f{0, 0, 0, 1})
        dump_attr(val->rotation, "rotation", js, err);
    if (val->scale != vec3f{1, 1, 1}) dump_attr(val->scale, "scale", js, err);
    if (val->translation != vec3f{0, 0, 0})
        dump_attr(val->translation, "translation", js, err);
    if (!val->weights.empty()) dump_attr(val->weights, "weights", js, err);
}
// Converts a glTFSamplerMagFilter enum to JSON
inline void dump(const glTFSamplerMagFilter& val, json& js, parse_stack& err) {
    static map<glTFSamplerMagFilter, int> table = {
        {glTFSamplerMagFilter::Nearest, 9728},
        {glTFSamplerMagFilter::Linear, 9729},
    };
    auto v = table.at(val);
    dump(v, js, err);
}

// Converts a glTFSamplerMinFilter enum to JSON
inline void dump(const glTFSamplerMinFilter& val, json& js, parse_stack& err) {
    static map<glTFSamplerMinFilter, int> table = {
        {glTFSamplerMinFilter::Nearest, 9728},
        {glTFSamplerMinFilter::Linear, 9729},
        {glTFSamplerMinFilter::NearestMipmapNearest, 9984},
        {glTFSamplerMinFilter::LinearMipmapNearest, 9985},
        {glTFSamplerMinFilter::NearestMipmapLinear, 9986},
        {glTFSamplerMinFilter::LinearMipmapLinear, 9987},
    };
    auto v = table.at(val);
    dump(v, js, err);
}

// Converts a glTFSamplerWrapS enum to JSON
inline void dump(const glTFSamplerWrapS& val, json& js, parse_stack& err) {
    static map<glTFSamplerWrapS, int> table = {
        {glTFSamplerWrapS::ClampToEdge, 33071},
        {glTFSamplerWrapS::MirroredRepeat, 33648},
        {glTFSamplerWrapS::Repeat, 10497},
    };
    auto v = table.at(val);
    dump(v, js, err);
}

// Converts a glTFSamplerWrapT enum to JSON
inline void dump(const glTFSamplerWrapT& val, json& js, parse_stack& err) {
    static map<glTFSamplerWrapT, int> table = {
        {glTFSamplerWrapT::ClampToEdge, 33071},
        {glTFSamplerWrapT::MirroredRepeat, 33648},
        {glTFSamplerWrapT::Repeat, 10497},
    };
    auto v = table.at(val);
    dump(v, js, err);
}

// Converts a glTFSampler object to JSON
inline void dump(const glTFSampler* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFChildOfRootProperty*)val, js, err);
    if (val->magFilter != glTFSamplerMagFilter::NotSet)
        dump_attr(val->magFilter, "magFilter", js, err);
    if (val->minFilter != glTFSamplerMinFilter::NotSet)
        dump_attr(val->minFilter, "minFilter", js, err);
    if (val->wrapS != glTFSamplerWrapS::Repeat)
        dump_attr(val->wrapS, "wrapS", js, err);
    if (val->wrapT != glTFSamplerWrapT::Repeat)
        dump_attr(val->wrapT, "wrapT", js, err);
}

// Converts a glTFScene object to JSON
inline void dump(const glTFScene* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFChildOfRootProperty*)val, js, err);
    if (!val->nodes.empty()) dump_attr(val->nodes, "nodes", js, err);
}

// Converts a glTFSkin object to JSON
inline void dump(const glTFSkin* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFChildOfRootProperty*)val, js, err);
    if (val->inverseBindMatrices.is_valid())
        dump_attr(val->inverseBindMatrices, "inverseBindMatrices", js, err);
    if (val->skeleton.is_valid()) dump_attr(val->skeleton, "skeleton", js, err);
    dump_attr(val->joints, "joints", js, err);
}

// Converts a glTF object to JSON
inline void dump(const glTF* val, json& js, parse_stack& err) {
    if (!js.is_object()) js = json::object();
    dump((const glTFProperty*)val, js, err);
    if (!val->extensionsUsed.empty())
        dump_attr(val->extensionsUsed, "extensionsUsed", js, err);
    if (!val->extensionsRequired.empty())
        dump_attr(val->extensionsRequired, "extensionsRequired", js, err);
    if (!val->accessors.empty())
        dump_attr(val->accessors, "accessors", js, err);
    if (!val->animations.empty())
        dump_attr(val->animations, "animations", js, err);
    dump_attr(val->asset, "asset", js, err);
    if (!val->buffers.empty()) dump_attr(val->buffers, "buffers", js, err);
    if (!val->bufferViews.empty())
        dump_attr(val->bufferViews, "bufferViews", js, err);
    if (!val->cameras.empty()) dump_attr(val->cameras, "cameras", js, err);
    if (!val->images.empty()) dump_attr(val->images, "images", js, err);
    if (!val->materials.empty())
        dump_attr(val->materials, "materials", js, err);
    if (!val->meshes.empty()) dump_attr(val->meshes, "meshes", js, err);
    if (!val->nodes.empty()) dump_attr(val->nodes, "nodes", js, err);
    if (!val->samplers.empty()) dump_attr(val->samplers, "samplers", js, err);
    if (val->scene.is_valid()) dump_attr(val->scene, "scene", js, err);
    if (!val->scenes.empty()) dump_attr(val->scenes, "scenes", js, err);
    if (!val->skins.empty()) dump_attr(val->skins, "skins", js, err);
    if (!val->textures.empty()) dump_attr(val->textures, "textures", js, err);
}
// #codegen end func

// Get directory name (including '/').
inline string _get_dirname(const string& filename) {
    auto pos = filename.rfind('/');
    if (pos == string::npos) pos = filename.rfind('\\');
    if (pos == string::npos) return "";
    return filename.substr(0, pos + 1);
}

// Get extension name
static inline string _get_extension(const string& filename) {
    auto pos = filename.rfind(".");
    if (pos == string::npos) return "";
    return filename.substr(pos);
}

// Get base name.
inline string _get_basename(const string& filename) {
    auto dirname = _get_dirname(filename);
    auto extension = _get_extension(filename);
    return filename.substr(
        dirname.size(), filename.size() - dirname.size() - extension.size());
}

// Fix path
inline string _fix_path(const string& path_) {
    auto path = path_;
    for (auto& c : path)
        if (c == '\\') c = '/';
    return path;
}

// Load a binary file in memory
// http://stackoverflow.com/questions/116038/what-is-the-best-way-to-read-an-entire-file-into-a-stdstring-in-c
vector<unsigned char> load_binfile(const string& filename, bool skip_missing) {
    std::ifstream ifs(
        filename.c_str(), std::ios::in | std::ios::binary | std::ios::ate);
    if (!ifs) {
        if (skip_missing) return {};
        throw runtime_error("could not open file " + filename);
    }
    std::ifstream::pos_type fileSize = ifs.tellg();
    ifs.seekg(0, std::ios::beg);
    vector<unsigned char> bytes(fileSize);
    ifs.read((char*)&bytes[0], fileSize);
    return bytes;
}

// Saves text.
void save_textfile(const string& filename, const string& txt) {
    auto f = fopen(filename.c_str(), "wt");
    if (!f) throw runtime_error("cannot write file " + filename);
    fwrite(txt.c_str(), 1, (int)txt.size(), f);
    fclose(f);
}

// Saves binary.
void save_binfile(const string& filename, const vector<unsigned char>& bin,
    bool skip_missing) {
    auto f = fopen(filename.c_str(), "wb");
    if (!f && !skip_missing)
        throw runtime_error("cannot write file " + filename);
    fwrite(bin.data(), 1, (int)bin.size(), f);
    fclose(f);
}

// Check if a string starts with a prefix
static inline bool startsiwith(const string& str, const string& prefix) {
    if (str.length() < prefix.length()) return false;
    return str.substr(0, prefix.length()) == prefix;
}

// Load buffer data.
void load_buffers(glTF* gltf, const string& dirname, bool skip_missing) {
    for (auto buffer : gltf->buffers) {
        if (buffer->uri == "") continue;
        if (startsiwith(buffer->uri, "data:")) {
            // assume it is base64 and find ','
            auto pos = buffer->uri.find(',');
            if (pos == buffer->uri.npos) {
                if (skip_missing) continue;
                throw runtime_error("could not decode base64 data");
            }
            // decode
            auto data = base64_decode(buffer->uri.substr(pos + 1));
            buffer->data = vector<unsigned char>((unsigned char*)data.c_str(),
                (unsigned char*)data.c_str() + data.length());
        } else {
            buffer->data =
                load_binfile(_fix_path(dirname + buffer->uri), skip_missing);
            if (buffer->data.empty()) {
                if (skip_missing) continue;
                throw runtime_error("could not load binary file " +
                                    _fix_path(dirname + buffer->uri));
            }
        }
        if (buffer->byteLength != buffer->data.size()) {
            if (skip_missing) continue;
            throw runtime_error("mismatched buffer size");
        }
    }
}

// Loads images.
void load_images(glTF* gltf, const string& dirname, bool skip_missing) {
    for (auto image : gltf->images) {
        image->data = image_data();
        auto filename = string();
#if YGL_IMAGEIO
        if (image->bufferView || startsiwith(image->uri, "data:")) {
            auto buffer = string();
            auto data = (unsigned char*)nullptr;
            auto data_size = 0;
            if (image->bufferView) {
                auto view = gltf->get(image->bufferView);
                auto buffer = gltf->get(view->buffer);
                if (!view || !buffer || view->byteStride) {
                    if (skip_missing) continue;
                    throw runtime_error("invalid image buffer view");
                }
                if (image->mimeType == glTFImageMimeType::ImagePng)
                    filename = "internal_data.png";
                else if (image->mimeType == glTFImageMimeType::ImageJpeg)
                    filename = "internal_data.jpg";
                else {
                    if (skip_missing) continue;
                    throw runtime_error("unsupported image format");
                }
                data = buffer->data.data() + view->byteOffset;
                data_size = view->byteLength;
            } else {
                // assume it is base64 and find ','
                auto pos = image->uri.find(',');
                if (pos == image->uri.npos) {
                    if (skip_missing) continue;
                    throw runtime_error("could not decode base64 data");
                }
                auto header = image->uri.substr(0, pos);
                for (auto format : {"png", "jpg", "jpeg", "tga", "ppm", "hdr"})
                    if (header.find(format) != header.npos)
                        filename = string("fake.") + format;
                if (is_hdr_filename(filename)) {
                    if (skip_missing) continue;
                    throw runtime_error("unsupported embedded image format " +
                                        header.substr(0, pos));
                }
                // decode
                buffer = base64_decode(image->uri.substr(pos + 1));
                data_size = (int)buffer.size();
                data = (unsigned char*)buffer.data();
            }
            if (is_hdr_filename(filename)) {
                image->data.dataf = load_imagef_from_memory(filename, data,
                    data_size, image->data.width, image->data.height,
                    image->data.ncomp);
            } else {
                image->data.datab = load_image_from_memory(filename, data,
                    data_size, image->data.width, image->data.height,
                    image->data.ncomp);
            }
        } else {
            filename = _fix_path(dirname + image->uri);
            if (is_hdr_filename(filename)) {
                image->data.dataf = load_imagef(filename, image->data.width,
                    image->data.height, image->data.ncomp);
            } else {
                image->data.datab = load_image(filename, image->data.width,
                    image->data.height, image->data.ncomp);
            }
        }
#endif
        if (image->data.dataf.empty() && image->data.datab.empty()) {
            if (skip_missing) continue;
            throw runtime_error("cannot load image " + filename);
        }
    }
}

// Loads a gltf.
glTF* load_gltf(
    const string& filename, bool load_bin, bool load_image, bool skip_missing) {
    // clear data
    auto gltf = unique_ptr<glTF>(new glTF());

    // load json
    std::ifstream stream(filename.c_str());
    if (!stream) throw runtime_error("could not load json " + filename);
    auto js = json();
    try {
        stream >> js;
    } catch (const exception& e) {
        throw runtime_error(
            string("could not load json with error ") + e.what());
    }

    // parse json
    auto stack = parse_stack();
    auto gltf_ = gltf.get();
    try {
        parse(gltf_, js, stack);
    } catch (const exception& e) {
        throw runtime_error("error parsing gltf at " + stack.pathname() +
                            " with error " + string(e.what()));
    }

    // load external resources
    auto dirname = _get_dirname(filename);
    if (load_bin) load_buffers(gltf.get(), dirname, skip_missing);
    if (load_image) load_images(gltf.get(), dirname, skip_missing);

    // done
    return gltf.release();
}

// Save buffer data.
void save_buffers(const glTF* gltf, const string& dirname, bool skip_missing) {
    for (auto buffer : gltf->buffers) {
        if (startsiwith(buffer->uri, "data:")) {
            if (skip_missing) continue;
            throw runtime_error("saving of embedded data not supported");
        }
        save_binfile(dirname + buffer->uri, buffer->data, skip_missing);
    }
}

// Save images.
void save_images(const glTF* gltf, const string& dirname, bool skip_missing) {
    for (auto image : gltf->images) {
        if (startsiwith(image->uri, "data:")) {
            if (skip_missing) continue;
            throw runtime_error("saving of embedded data not supported");
        }
        auto filename = dirname + image->uri;
        auto ok = false;
#if YGL_IMAGEIO
        if (!image->data.datab.empty()) {
            ok = save_image(filename, image->data.width, image->data.height,
                image->data.ncomp, image->data.datab.data());
        }
        if (!image->data.dataf.empty()) {
            ok = save_imagef(filename, image->data.width, image->data.height,
                image->data.ncomp, image->data.dataf.data());
        }
#endif
        if (!ok) {
            if (skip_missing) continue;
            throw runtime_error("cannot save image " + filename);
        }
    }
}

// Saves a gltf.
void save_gltf(
    const string& filename, const glTF* gltf, bool save_bin, bool save_image) {
    // dumps json
    auto js = json();
    auto stack = parse_stack();
    dump(gltf, js, stack);

    // save json
    save_textfile(filename, js.dump(2));

    // save external resources
    auto dirname = _get_dirname(filename);
    if (save_bin) save_buffers(gltf, dirname, false);
    if (save_image) save_images(gltf, dirname, false);
}

// reading shortcut
template <typename T>
inline void read(FILE* f, T* v, int count) {
    if (fread(v, sizeof(T), count, f) != count)
        throw runtime_error("could not read binary file");
}

// writing shortcut
template <typename T>
inline void fwrite(FILE* f, const T* v, int count) {
    if (fwrite(v, sizeof(T), count, f) != count)
        runtime_error("could not write binary file");
}

// Loads a binary gltf.
glTF* load_binary_gltf(
    const string& filename, bool load_bin, bool load_image, bool skip_missing) {
    // clear data
    auto gltf = unique_ptr<glTF>(new glTF());

    // opens binary file
    auto f = fopen(filename.c_str(), "rb");
    if (!f) throw runtime_error("could not load binary file " + filename);

    // read magic
    uint32_t magic;
    read(f, &magic, 1);
    if (magic != 0x46546C67) throw runtime_error("corrupted glb format");

    // read version
    uint32_t version;
    read(f, &version, 1);
    if (version != 1 && version != 2)
        throw runtime_error("unsupported glb version");

    // read length
    uint32_t length;
    read(f, &length, 1);

    // data
    auto json_bytes = vector<char>();
    auto buffer_bytes = vector<unsigned char>();
    uint32_t buffer_length = 0;

    if (version == 1) {
        // read content length and format
        uint32_t json_length, json_format;
        read(f, &json_length, 1);
        read(f, &json_format, 1);

        // read json bytes
        json_bytes.resize(json_length);
        read(f, json_bytes.data(), json_length);

        // read buffer bytes
        if (load_bin) {
            buffer_bytes.resize(length - json_length - 20);
            read(f, buffer_bytes.data(), (int)buffer_bytes.size());
            buffer_length = (int)buffer_bytes.size();
        }
    }

    if (version == 2) {
        // read content length and format
        uint32_t json_length, json_format;
        read(f, &json_length, 1);
        read(f, &json_format, 1);
        if (json_format != 0x4E4F534A) {
            throw runtime_error("corrupt binary format");
            return nullptr;
        }

        // read json bytes
        json_bytes.resize(json_length);
        read(f, json_bytes.data(), (int)json_bytes.size());

        // read content length and format
        uint32_t buffer_format;
        read(f, &buffer_length, 1);
        read(f, &buffer_format, 1);
        if (buffer_format != 0x004E4942)
            throw runtime_error("corrupt binary format");

        // read buffer bytes
        if (load_bin) {
            buffer_bytes.resize(buffer_length);
            read(f, buffer_bytes.data(), (int)buffer_bytes.size());
        }
    }

    // load json
    auto js = json();
    try {
        json_bytes.push_back(0);
        js = json::parse(json_bytes.data());
    } catch (const exception& e) {
        throw runtime_error(
            string("could not load json with error ") + e.what());
    }

    // parse json
    auto stack = parse_stack();
    auto gltf_ = gltf.get();
    try {
        parse(gltf_, js, stack);
    } catch (const exception& e) {
        throw runtime_error("cannot parse gltf json with error at " +
                            stack.pathname() + string(" with error ") +
                            e.what());
        return nullptr;
    }

    // fix internal buffer
    auto buffer = gltf->buffers.at(0);
    buffer->byteLength = buffer_length;
    if (version == 2) buffer->uri = "";
    if (load_bin) { buffer->data = buffer_bytes; }

    // load external resources
    auto dirname = _get_dirname(filename);
    if (load_bin) load_buffers(gltf.get(), dirname, skip_missing);
    if (load_image) load_images(gltf.get(), dirname, skip_missing);

    // close
    fclose(f);

    // done
    return gltf.release();
}

// Saves a binary gltf.
void save_binary_gltf(
    const string& filename, const glTF* gltf, bool save_bin, bool save_image) {
    // opens binary file
    auto f = fopen(filename.c_str(), "wb");
    if (!f) throw runtime_error("could not write binary file");

    // dumps json
    auto js = json();
    auto stack = parse_stack();
    dump(gltf, js, stack);

    // fix string
    auto js_str = js.dump(2);
    if (js_str.length() % 4) {
        auto count = js_str.length() % 4;
        for (auto c = 0; c < count; c++) js_str += " ";
    }
    uint32_t json_length = (uint32_t)js_str.size();

    // internal buffer
    auto buffer = gltf->buffers.at(0);
    uint32_t buffer_length = buffer->byteLength;
    if (buffer_length % 4) buffer_length += 4 - buffer_length % 4;

    // write header
    uint32_t magic = 0x46546C67;
    fwrite(f, &magic, 1);
    uint32_t version = 2;
    fwrite(f, &version, 1);
    uint32_t length = 12 + 8 + json_length + 8 + buffer_length;
    fwrite(f, &length, 1);

    // write json
    uint32_t json_type = 0x4E4F534A;
    fwrite(f, &json_length, 1);
    fwrite(f, &json_type, 1);
    fwrite(f, js_str.data(), (int)json_length);

    if (save_bin) {
        uint32_t buffer_type = 0x004E4942;
        fwrite(f, &buffer_length, 1);
        fwrite(f, &buffer_type, 1);
        fwrite(f, buffer->data.data(), (int)buffer->data.size());
        char pad = 0;
        for (auto i = 0; i < buffer_length - buffer->data.size(); i++)
            fwrite(f, &pad, 1);
    }

    // close
    fclose(f);

    // save external resources
    auto dirname = _get_dirname(filename);
    if (save_bin) save_buffers(gltf, dirname, false);
    if (save_image) save_images(gltf, dirname, false);
}

}  // namespace _impl_gltf

// Loads a gltf file from disk
inline glTF* load_gltf(
    const string& filename, bool load_bin, bool load_img, bool skip_missing) {
    return _impl_gltf::load_gltf(filename, load_bin, load_img, skip_missing);
}

// Loads a binary gltf file from disk
inline glTF* load_binary_gltf(
    const string& filename, bool load_bin, bool load_img, bool skip_missing) {
    return _impl_gltf::load_binary_gltf(
        filename, load_bin, load_img, skip_missing);
}

// Saves a scene to disk
inline void save_gltf(
    const string& filename, const glTF* gltf, bool save_bin, bool save_images) {
    _impl_gltf::save_gltf(filename, gltf, save_bin, save_images);
}

// Saves a scene to disk
inline void save_binary_gltf(
    const string& filename, const glTF* gltf, bool save_bin, bool save_images) {
    _impl_gltf::save_binary_gltf(filename, gltf, save_bin, save_images);
}

inline accessor_view::accessor_view(
    const glTF* gltf, const glTFAccessor* accessor) {
    _size = accessor->count;
    _ncomp = _num_components(accessor->type);
    _ctype = accessor->componentType;
    _normalize = accessor->normalized;
    auto buffer_view = gltf->get(accessor->bufferView);
    _stride = (buffer_view->byteStride) ? buffer_view->byteStride :
                                          (_ctype_size(_ctype) * _ncomp);
    auto buffer = gltf->get(buffer_view->buffer);
    _data =
        buffer->data.data() + accessor->byteOffset + buffer_view->byteOffset;
    auto remaining_buffer_bytes =
        buffer->data.size() - (_data - buffer->data.data());
    auto view_bytes = _size * _stride;
    _valid = remaining_buffer_bytes >= view_bytes;
    if (!_valid) throw runtime_error("corrupted glTF accessor view");
}

inline float accessor_view::get(int idx, int c) const {
    auto i = min(max(c, 0), ncomp() - 1);
    auto valb = _data + _stride * idx + i * _ctype_size(_ctype);
    // use double for integer conversion to attempt to maintain precision
    if (!_normalize) {
        switch (_ctype) {
            case glTFAccessorComponentType::Float:
                return (float)(*(float*)valb);
            case glTFAccessorComponentType::Byte: return (float)(*(char*)valb);
            case glTFAccessorComponentType::UnsignedByte:
                return (float)(*(unsigned char*)valb);
            case glTFAccessorComponentType::Short:
                return (float)(*(short*)valb);
            case glTFAccessorComponentType::UnsignedShort:
                return (float)(*(unsigned short*)valb);
            case glTFAccessorComponentType::UnsignedInt:
                return (float)(*(unsigned int*)valb);
            case glTFAccessorComponentType::NotSet:
                throw runtime_error("bad enum value");
                break;
        }

    } else {
        switch (_ctype) {
            case glTFAccessorComponentType::Float:
                return (float)(*(float*)valb);
            case glTFAccessorComponentType::Byte:
                return (float)max(c / 127.0, -1.0);
            case glTFAccessorComponentType::UnsignedByte:
                return (float)(c / 255.0);
            case glTFAccessorComponentType::Short:
                return (float)(max(c / 32767.0, -1.0));
            case glTFAccessorComponentType::UnsignedShort:
                return (float)(c / 65535.0);
            case glTFAccessorComponentType::UnsignedInt:
                return (float)(max(c / 2147483647.0, -1.0));
            case glTFAccessorComponentType::NotSet:
                throw runtime_error("bad enum value");
                break;
        }
    }
    return 0;
}

inline int accessor_view::geti(int idx, int c) const {
    auto i = min(max(c, 0), ncomp() - 1);
    auto valb = _data + _stride * idx + i * _ctype_size(_ctype);
    // use double for integer conversion to attempt to maintain precision
    switch (_ctype) {
        case glTFAccessorComponentType::Float: return (int)(*(float*)valb);
        case glTFAccessorComponentType::Byte: return (int)(*(char*)valb);
        case glTFAccessorComponentType::UnsignedByte:
            return (int)(*(unsigned char*)valb);
        case glTFAccessorComponentType::Short: return (int)(*(short*)valb);
        case glTFAccessorComponentType::UnsignedShort:
            return (int)(*(unsigned short*)valb);
        case glTFAccessorComponentType::UnsignedInt:
            return (int)(*(unsigned int*)valb);
        case glTFAccessorComponentType::NotSet:
            throw runtime_error("bad enum value");
            break;
    }
    return 0;
}

inline int accessor_view::_num_components(glTFAccessorType type) {
    switch (type) {
        case glTFAccessorType::Scalar: return 1;
        case glTFAccessorType::Vec2: return 2;
        case glTFAccessorType::Vec3: return 3;
        case glTFAccessorType::Vec4: return 4;
        case glTFAccessorType::Mat2: return 4;
        case glTFAccessorType::Mat3: return 9;
        case glTFAccessorType::Mat4: return 16;
        default: assert(false); return 0;
    }
}

inline int accessor_view::_ctype_size(glTFAccessorComponentType componentType) {
    switch (componentType) {
        case glTFAccessorComponentType::Byte: return 1;
        case glTFAccessorComponentType::UnsignedByte: return 1;
        case glTFAccessorComponentType::Short: return 2;
        case glTFAccessorComponentType::UnsignedShort: return 2;
        case glTFAccessorComponentType::UnsignedInt: return 4;
        case glTFAccessorComponentType::Float: return 4;
        default: assert(false); return 0;
    }
}

// Math support
inline mat4f node_transform(const glTFNode* node) {
    return translation_mat4(node->translation) * rotation_mat4(node->rotation) *
           scaling_mat4(node->scale) * node->matrix;
}

#endif

}  // namespace ygl

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR SIMPLE SCENE
// -----------------------------------------------------------------------------
namespace ygl {

namespace _impl_scn {

#if YGL_SCENEIO

// Flattens an scene
inline scene* obj_to_scene(const obj_scene* obj, const load_options& opts) {
    // clear scene
    auto scn = new scene();

    struct obj_vertex_hash {
        std::hash<int> Th;
        size_t operator()(const obj_vertex& vv) const {
            auto v = (const int*)&vv;
            size_t h = 0;
            for (auto i = 0; i < sizeof(obj_vertex) / sizeof(int); i++) {
                // embads hash_combine below
                h ^= (Th(v[i]) + 0x9e3779b9 + (h << 6) + (h >> 2));
            }
            return h;
        }
    };

    // convert textures
    auto tmap = unordered_map<string, texture*>{{"", nullptr}};
    for (auto otxt : obj->textures) {
        auto txt = new texture();
        txt->name = otxt->path;
        txt->path = otxt->path;
        if (!otxt->datab.empty()) {
            txt->ldr = image4b(otxt->width, otxt->height);
            for (auto j = 0; j < otxt->height; j++) {
                for (auto i = 0; i < otxt->width; i++) {
                    auto v = otxt->datab.data() +
                             (otxt->width * j + i) * otxt->ncomp;
                    switch (otxt->ncomp) {
                        case 1:
                            txt->ldr.at(i, j) = {v[0], v[0], v[0], 255};
                            break;
                        case 2: txt->ldr.at(i, j) = {v[0], v[1], 0, 255}; break;
                        case 3:
                            txt->ldr.at(i, j) = {v[0], v[1], v[2], 255};
                            break;
                        case 4:
                            txt->ldr.at(i, j) = {v[0], v[1], v[2], v[3]};
                            break;
                        default: assert(false); break;
                    }
                }
            }
        } else if (!otxt->dataf.empty()) {
            txt->hdr = image4f(otxt->width, otxt->height);
            for (auto j = 0; j < otxt->height; j++) {
                for (auto i = 0; i < otxt->width; i++) {
                    auto v = otxt->dataf.data() +
                             (otxt->width * j + i) * otxt->ncomp;
                    switch (otxt->ncomp) {
                        case 1:
                            txt->hdr.at(i, j) = {v[0], v[0], v[0], 1};
                            break;
                        case 2: txt->hdr.at(i, j) = {v[0], v[1], 0, 1}; break;
                        case 3:
                            txt->hdr.at(i, j) = {v[0], v[1], v[2], 1};
                            break;
                        case 4:
                            txt->hdr.at(i, j) = {v[0], v[1], v[2], v[3]};
                            break;
                        default: assert(false); break;
                    }
                }
            }
        }
        scn->textures.push_back(txt);
        tmap[txt->path] = txt;
    }

    auto add_texture = [&tmap](const obj_texture_info& oinfo) {
        auto info = texture_info();
        if (oinfo.path == "") return info;
        info.txt = tmap.at(oinfo.path);
        info.wrap_s = !oinfo.clamp;
        info.wrap_t = !oinfo.clamp;
        info.scale = oinfo.scale;
        return info;
    };

    // convert materials and build textures
    auto mmap = unordered_map<string, material*>{{"", nullptr}};
    for (auto omat : obj->materials) {
        auto mat = new material();
        mat->name = omat->name;
        mat->mtype = material_type::specular_roughness;
        mat->ke = {omat->ke.x, omat->ke.y, omat->ke.z};
        mat->kd = {omat->kd.x, omat->kd.y, omat->kd.z};
        mat->ks = {omat->ks.x, omat->ks.y, omat->ks.z};
        mat->kt = {omat->kt.x, omat->kt.y, omat->kt.z};
        mat->rs = pow(2 / (omat->ns + 2), 1 / 4.0f);
        mat->op = omat->op;
        mat->ke_txt = add_texture(omat->ke_txt);
        mat->kd_txt = add_texture(omat->kd_txt);
        mat->ks_txt = add_texture(omat->ks_txt);
        mat->kt_txt = add_texture(omat->kt_txt);
        mat->rs_txt = add_texture(omat->ns_txt);
        mat->norm_txt = add_texture(omat->norm_txt);
        mat->bump_txt = add_texture(omat->bump_txt);
        mat->disp_txt = add_texture(omat->disp_txt);
        switch (omat->illum) {
            case 0:  // Color on and Ambient off
            case 1:  // Color on and Ambient on
            case 2:  // Highlight on
            case 3:  // Reflection on and Ray trace on
                mat->op = 1;
                mat->kt = {0, 0, 0};
                break;
            case 4:  // Transparency: Glass on
                // Reflection: Ray trace on
                break;
            case 5:  // Reflection: Fresnel on and Ray trace on
                mat->op = 1;
                mat->kt = {0, 0, 0};
                break;
            case 6:  // Transparency: Refraction on
                     // Reflection: Fresnel off and Ray trace on
            case 7:  // Transparency: Refraction on
                // Reflection: Fresnel on and Ray trace on
                break;
            case 8:  // Reflection on and Ray trace off
                mat->op = 1;
                mat->kt = {0, 0, 0};
                break;
            case 9:  // Transparency: Glass on
                // Reflection: Ray trace off
                break;
        }
        scn->materials.push_back(mat);
        mmap[mat->name] = mat;
    }

    // convert meshes
    auto omap = unordered_map<string, vector<shape*>>{{"", {}}};
    for (auto omsh : obj->objects) {
        omap[omsh->name] = {};
        for (auto& oshp : omsh->groups) {
            if (oshp.verts.empty()) continue;
            if (oshp.elems.empty()) continue;

            auto shp = new shape();
            shp->name = omsh->name + oshp.groupname;
            shp->mat = mmap[oshp.matname];

            // check to see if this shuold be face-varying or flat quads
            auto as_facevarying = false, as_quads = false;
            if (opts.preserve_quads || opts.preserve_facevarying) {
                auto m = 10000, M = -1;
                for (auto& elem : oshp.elems) {
                    if (elem.type != obj_element_type::face) {
                        m = 2;
                        break;
                    } else {
                        m = min(m, (int)elem.size);
                        M = max(M, (int)elem.size);
                    }
                }
                if (m >= 3 && M == 4) as_quads = opts.preserve_quads;
                if (m >= 3 && M <= 4)
                    as_facevarying = opts.preserve_facevarying;
            }

            // in case of facevarying, check if there is really a need for it
            if (as_facevarying) {
                for (auto& elem : oshp.elems) {
                    for (auto i = elem.start; i < elem.start + elem.size; i++) {
                        auto& v = oshp.verts[i];
                        if (v.norm >= 0 && v.pos != v.norm)
                            as_facevarying = false;
                        if (v.texcoord >= 0 && v.pos != v.texcoord)
                            as_facevarying = false;
                        if (v.color >= 0) as_facevarying = false;
                        if (v.radius >= 0) as_facevarying = false;
                    }
                    if (!as_facevarying) break;
                }
            }

            if (!as_facevarying) {
                // insert all vertices
                unordered_map<obj_vertex, int, obj_vertex_hash> vert_map;
                vector<int> vert_ids;
                for (auto& vert : oshp.verts) {
                    if (vert_map.find(vert) == vert_map.end()) {
                        auto s = (int)vert_map.size();
                        vert_map[vert] = s;
                    }
                    vert_ids.push_back(vert_map.at(vert));
                }

                // convert elements
                for (auto& elem : oshp.elems) {
                    switch (elem.type) {
                        case obj_element_type::point: {
                            for (auto i = elem.start;
                                 i < elem.start + elem.size; i++) {
                                shp->points.push_back(vert_ids[i]);
                            }
                        } break;
                        case obj_element_type::line: {
                            for (auto i = elem.start;
                                 i < elem.start + elem.size - 1; i++) {
                                shp->lines.push_back(
                                    {vert_ids[i], vert_ids[i + 1]});
                            }
                        } break;
                        case obj_element_type::face: {
                            if (as_quads) {
                                shp->quads.push_back({vert_ids[elem.start + 0],
                                    vert_ids[elem.start + 1],
                                    vert_ids[elem.start + 2],
                                    vert_ids[elem.start +
                                             ((elem.size == 3) ? 2 : 3)]});
                            } else if (elem.size == 3) {
                                shp->triangles.push_back(
                                    {vert_ids[elem.start + 0],
                                        vert_ids[elem.start + 1],
                                        vert_ids[elem.start + 2]});
                            } else {
                                for (auto i = elem.start + 2;
                                     i < elem.start + elem.size; i++) {
                                    shp->triangles.push_back(
                                        {vert_ids[elem.start], vert_ids[i - 1],
                                            vert_ids[i]});
                                }
                            }
                        } break;
                        default: { assert(false); }
                    }
                }

                // copy vertex data
                auto v = oshp.verts[0];
                if (v.pos >= 0) shp->pos.resize(vert_map.size());
                if (v.texcoord >= 0) shp->texcoord.resize(vert_map.size());
                if (v.norm >= 0) shp->norm.resize(vert_map.size());
                if (v.color >= 0) shp->color.resize(vert_map.size());
                if (v.radius >= 0) shp->radius.resize(vert_map.size());
                for (auto& kv : vert_map) {
                    if (v.pos >= 0 && kv.first.pos >= 0) {
                        auto v = obj->pos[kv.first.pos];
                        shp->pos[kv.second] = {v.x, v.y, v.z};
                    }
                    if (v.texcoord >= 0 && kv.first.texcoord >= 0) {
                        auto v = obj->texcoord[kv.first.texcoord];
                        shp->texcoord[kv.second] = {v.x, v.y};
                    }
                    if (v.norm >= 0 && kv.first.norm >= 0) {
                        auto v = obj->norm[kv.first.norm];
                        shp->norm[kv.second] = {v.x, v.y, v.z};
                    }
                    if (v.color >= 0 && kv.first.color >= 0) {
                        auto v = obj->color[kv.first.color];
                        shp->color[kv.second] = {v.x, v.y, v.z, v.w};
                    }
                    if (v.radius >= 0 && kv.first.radius >= 0) {
                        shp->radius[kv.second] = obj->radius[kv.first.radius];
                    }
                }

                // fix smoothing
                if (!oshp.smoothing && opts.obj_facet_non_smooth) {
                    auto faceted = new shape();
                    faceted->name = shp->name;
                    auto pidx = vector<int>();
                    for (auto point : shp->points) {
                        faceted->points.push_back((int)pidx.size());
                        pidx.push_back(point);
                    }
                    for (auto line : shp->lines) {
                        faceted->lines.push_back(
                            {(int)pidx.size() + 0, (int)pidx.size() + 1});
                        pidx.push_back(line.x);
                        pidx.push_back(line.y);
                    }
                    for (auto triangle : shp->triangles) {
                        faceted->triangles.push_back({(int)pidx.size() + 0,
                            (int)pidx.size() + 1, (int)pidx.size() + 2});
                        pidx.push_back(triangle.x);
                        pidx.push_back(triangle.y);
                        pidx.push_back(triangle.z);
                    }
                    for (auto idx : pidx) {
                        if (!shp->pos.empty())
                            faceted->pos.push_back(shp->pos[idx]);
                        if (!shp->norm.empty())
                            faceted->norm.push_back(shp->norm[idx]);
                        if (!shp->texcoord.empty())
                            faceted->texcoord.push_back(shp->texcoord[idx]);
                        if (!shp->color.empty())
                            faceted->color.push_back(shp->color[idx]);
                        if (!shp->radius.empty())
                            faceted->radius.push_back(shp->radius[idx]);
                    }
                    delete shp;
                    shp = faceted;
                }
            } else {
                // insert all vertices
                unordered_map<int, int> pos_map, norm_map, texcoord_map;
                vector<int> pos_ids, norm_ids, texcoord_ids;
                for (auto& vert : oshp.verts) {
                    if (vert.pos >= 0) {
                        if (pos_map.find(vert.pos) == pos_map.end()) {
                            auto s = (int)pos_map.size();
                            pos_map[vert.pos] = s;
                        }
                        pos_ids.push_back(pos_map.at(vert.pos));
                    } else {
                        if (!pos_ids.empty())
                            throw runtime_error("malformed obj");
                    }
                    if (vert.norm >= 0) {
                        if (norm_map.find(vert.norm) == norm_map.end()) {
                            auto s = (int)norm_map.size();
                            norm_map[vert.norm] = s;
                        }
                        norm_ids.push_back(norm_map.at(vert.norm));
                    } else {
                        if (!norm_ids.empty())
                            throw runtime_error("malformed obj");
                    }
                    if (vert.texcoord >= 0) {
                        if (texcoord_map.find(vert.texcoord) ==
                            texcoord_map.end()) {
                            auto s = (int)texcoord_map.size();
                            texcoord_map[vert.texcoord] = s;
                        }
                        texcoord_ids.push_back(texcoord_map.at(vert.texcoord));
                    } else {
                        if (!texcoord_ids.empty())
                            throw runtime_error("malformed obj");
                    }
                }

                // convert elements
                for (auto& elem : oshp.elems) {
                    if (elem.type != obj_element_type::face)
                        throw runtime_error("malformed obj");
                    if (elem.size < 3 || elem.size > 4)
                        throw runtime_error("malformed obj");
                    if (!pos_ids.empty()) {
                        shp->quads_pos.push_back({pos_ids[elem.start + 0],
                            pos_ids[elem.start + 1], pos_ids[elem.start + 2],
                            pos_ids[elem.start + ((elem.size == 3) ? 2 : 3)]});
                    }
                    if (!texcoord_ids.empty()) {
                        shp->quads_texcoord.push_back(
                            {texcoord_ids[elem.start + 0],
                                texcoord_ids[elem.start + 1],
                                texcoord_ids[elem.start + 2],
                                texcoord_ids[elem.start +
                                             ((elem.size == 3) ? 2 : 3)]});
                    }
                    if (!norm_ids.empty()) {
                        shp->quads_norm.push_back({norm_ids[elem.start + 0],
                            norm_ids[elem.start + 1], norm_ids[elem.start + 2],
                            norm_ids[elem.start + ((elem.size == 3) ? 2 : 3)]});
                    }
                }

                // copy vertex data
                shp->pos.resize(pos_map.size());
                shp->texcoord.resize(texcoord_map.size());
                shp->norm.resize(norm_map.size());
                for (auto& kv : pos_map) {
                    shp->pos[kv.second] = obj->pos[kv.first];
                }
                for (auto& kv : texcoord_map) {
                    shp->texcoord[kv.second] = obj->texcoord[kv.first];
                }
                for (auto& kv : norm_map) {
                    shp->norm[kv.second] = obj->norm[kv.first];
                }

                // fix smoothing
                if (!oshp.smoothing && opts.obj_facet_non_smooth) {}
            }
            scn->shapes.push_back(shp);
            omap[omsh->name].push_back(shp);
        }
    }

    // convert cameras
    for (auto ocam : obj->cameras) {
        auto cam = new camera();
        cam->name = ocam->name;
        cam->ortho = ocam->ortho;
        cam->yfov = ocam->yfov;
        cam->aspect = ocam->aspect;
        cam->aperture = ocam->aperture;
        cam->focus = ocam->focus;
        cam->frame = ocam->frame;
        scn->cameras.push_back(cam);
    }

    // convert envs
    unordered_set<material*> env_mat;
    for (auto oenv : obj->environments) {
        auto env = new environment();
        env->name = oenv->name;
        for (auto mat : scn->materials) {
            if (mat->name == oenv->matname) {
                env->ke = mat->ke;
                env->ke_txt = mat->ke_txt;
                env_mat.insert(mat);
            }
        }
        env->frame = oenv->frame;
        scn->environments.push_back(env);
    }

    // remove env materials
    for (auto shp : scn->shapes) env_mat.erase(shp->mat);
    for (auto mat : env_mat) {
        auto end =
            std::remove(scn->materials.begin(), scn->materials.end(), mat);
        scn->materials.erase(end, scn->materials.end());
        delete mat;
    }

    // convert instances
    for (auto oist : obj->instances) {
        for (auto shp : omap[oist->objname]) {
            auto ist = new instance();
            ist->name = oist->name;
            ist->shp = shp;
            ist->frame = oist->frame;
            scn->instances.push_back(ist);
        }
    }

    // done
    return scn;
}

// Load an obj scene
inline scene* load_obj_scene(const string& filename, const load_options& opts) {
    auto oscn = unique_ptr<obj_scene>(load_obj(filename, opts.load_textures,
        opts.skip_missing, opts.obj_flip_texcoord, opts.obj_flip_tr));
    auto scn = unique_ptr<scene>(obj_to_scene(oscn.get(), opts));
    return scn.release();
}

// Save an scene
inline obj_scene* scene_to_obj(const scene* scn) {
    auto obj = new obj_scene();

    auto add_texture = [](const texture_info& info, bool bump = false) {
        auto oinfo = obj_texture_info();
        if (!info.txt) return oinfo;
        oinfo.path = info.txt->path;
        oinfo.clamp = !info.wrap_s && !info.wrap_t;
        if (bump) oinfo.scale = info.scale;
        return oinfo;
    };

    // convert textures
    for (auto txt : scn->textures) {
        auto otxt = new obj_texture();
        otxt->path = txt->path;
        if (txt->hdr) {
            otxt->width = txt->hdr.width();
            otxt->height = txt->hdr.height();
            otxt->ncomp = 4;
            otxt->dataf.assign((float*)txt->hdr.data(),
                (float*)txt->hdr.data() +
                    txt->hdr.width() * txt->hdr.height() * 4);
        }
        if (txt->ldr) {
            otxt->width = txt->ldr.width();
            otxt->height = txt->ldr.height();
            otxt->ncomp = 4;
            otxt->datab.assign((uint8_t*)txt->ldr.data(),
                (uint8_t*)txt->ldr.data() +
                    txt->ldr.width() * txt->ldr.height() * 4);
        }
        obj->textures.push_back(otxt);
    }

    // convert materials
    for (auto mat : scn->materials) {
        auto omat = new obj_material();
        omat->name = mat->name;
        omat->ke = {mat->ke.x, mat->ke.y, mat->ke.z};
        omat->ke_txt = add_texture(mat->ke_txt);
        switch (mat->mtype) {
            case material_type::specular_roughness: {
                omat->kd = {mat->kd.x, mat->kd.y, mat->kd.z};
                omat->ks = {mat->ks.x, mat->ks.y, mat->ks.z};
                omat->kt = {mat->kt.x, mat->kt.y, mat->kt.z};
                omat->ns = (mat->rs) ? 2 / pow(mat->rs, 4.0f) - 2 : 1e6;
                omat->op = mat->op;
                omat->kd_txt = add_texture(mat->kd_txt);
                omat->ks_txt = add_texture(mat->ks_txt);
                omat->kt_txt = add_texture(mat->kt_txt);
            } break;
            case material_type::metallic_roughness: {
                if (mat->rs == 1 && mat->ks.x == 0) {
                    omat->kd = mat->kd;
                    omat->ks = {0, 0, 0};
                    omat->ns = 1;
                } else {
                    auto kd = mat->kd * (1 - 0.04f) * (1 - mat->ks.x);
                    auto ks = mat->kd * mat->ks.x +
                              vec3f{0.04f, 0.04f, 0.04f} * (1 - mat->ks.x);
                    omat->kd = {kd.x, kd.y, kd.z};
                    omat->ks = {ks.x, ks.y, ks.z};
                    omat->ns = (mat->rs) ? 2 / pow(mat->rs, 4.0f) - 2 : 1e6;
                }
                omat->op = mat->op;
                if (mat->ks.x < 0.5f) {
                    omat->kd_txt = add_texture(mat->kd_txt);
                } else {
                    omat->ks_txt = add_texture(mat->ks_txt);
                }
            } break;
            case material_type::specular_glossiness: {
                omat->kd = {mat->kd.x, mat->kd.y, mat->kd.z};
                omat->ks = {mat->ks.x, mat->ks.y, mat->ks.z};
                omat->ns = (mat->rs) ? 2 / pow(1 - mat->rs, 4.0f) - 2 : 1e6;
                omat->op = mat->op;
                omat->kd_txt = add_texture(mat->kd_txt);
                omat->ks_txt = add_texture(mat->ks_txt);
            } break;
        }
        omat->bump_txt = add_texture(mat->bump_txt, true);
        omat->disp_txt = add_texture(mat->disp_txt, true);
        omat->norm_txt = add_texture(mat->norm_txt, true);
        if (mat->op < 1 || mat->kt != zero3f) {
            omat->illum = 4;
        } else {
            omat->illum = 2;
        }
        obj->materials.push_back(omat);
    }

    // convert shapes
    for (auto shp : scn->shapes) {
        auto offset = obj_vertex{(int)obj->pos.size(),
            (int)obj->texcoord.size(), (int)obj->norm.size(),
            (int)obj->color.size(), (int)obj->radius.size()};
        for (auto& v : shp->pos) obj->pos.push_back({v.x, v.y, v.z});
        for (auto& v : shp->norm) obj->norm.push_back({v.x, v.y, v.z});
        for (auto& v : shp->texcoord) obj->texcoord.push_back({v.x, v.y});
        for (auto& v : shp->color) obj->color.push_back({v.x, v.y, v.z, v.w});
        for (auto& v : shp->radius) obj->radius.push_back(v);
        auto object = new obj_object();
        object->name = shp->name;
        object->groups.emplace_back();
        auto group = &object->groups.back();
        group->matname = (shp->mat) ? shp->mat->name : "";
        for (auto point : shp->points) {
            group->elems.push_back(
                {(uint32_t)group->verts.size(), obj_element_type::point, 1});
            group->verts.push_back(
                {(shp->pos.empty()) ? -1 : offset.pos + point,
                    (shp->texcoord.empty()) ? -1 : offset.texcoord + point,
                    (shp->norm.empty()) ? -1 : offset.norm + point,
                    (shp->color.empty()) ? -1 : offset.color + point,
                    (shp->radius.empty()) ? -1 : offset.radius + point});
        }
        for (auto line : shp->lines) {
            group->elems.push_back(
                {(uint32_t)group->verts.size(), obj_element_type::line, 2});
            for (auto vid : line) {
                group->verts.push_back(
                    {(shp->pos.empty()) ? -1 : offset.pos + vid,
                        (shp->texcoord.empty()) ? -1 : offset.texcoord + vid,
                        (shp->norm.empty()) ? -1 : offset.norm + vid,
                        (shp->color.empty()) ? -1 : offset.color + vid,
                        (shp->radius.empty()) ? -1 : offset.radius + vid});
            }
        }
        for (auto triangle : shp->triangles) {
            group->elems.push_back(
                {(uint32_t)group->verts.size(), obj_element_type::face, 3});
            for (auto vid : triangle) {
                group->verts.push_back(
                    {(shp->pos.empty()) ? -1 : offset.pos + vid,
                        (shp->texcoord.empty()) ? -1 : offset.texcoord + vid,
                        (shp->norm.empty()) ? -1 : offset.norm + vid,
                        (shp->color.empty()) ? -1 : offset.color + vid,
                        (shp->radius.empty()) ? -1 : offset.radius + vid});
            }
        }
        for (auto quad : shp->quads) {
            group->elems.push_back(
                {(uint32_t)group->verts.size(), obj_element_type::face, 4});
            for (auto vid : quad) {
                group->verts.push_back(
                    {(shp->pos.empty()) ? -1 : offset.pos + vid,
                        (shp->texcoord.empty()) ? -1 : offset.texcoord + vid,
                        (shp->norm.empty()) ? -1 : offset.norm + vid,
                        (shp->color.empty()) ? -1 : offset.color + vid,
                        (shp->radius.empty()) ? -1 : offset.radius + vid});
            }
        }
        obj->objects.emplace_back(object);
    }

    // convert cameras
    for (auto cam : scn->cameras) {
        auto ocam = new obj_camera();
        ocam->name = cam->name;
        ocam->ortho = cam->ortho;
        ocam->yfov = cam->yfov;
        ocam->aspect = cam->aspect;
        ocam->focus = cam->focus;
        ocam->aperture = cam->aperture;
        ocam->frame = cam->frame;
        obj->cameras.push_back(ocam);
    }

    // convert envs
    for (auto env : scn->environments) {
        auto oenv = new obj_environment();
        auto omat = new obj_material();
        omat->name = env->name + "_mat";
        omat->ke = env->ke;
        omat->ke_txt = add_texture(env->ke_txt);
        oenv->name = env->name;
        oenv->matname = omat->name;
        oenv->frame = env->frame;
        obj->materials.push_back(omat);
        obj->environments.push_back(oenv);
    }

    // convert instances
    for (auto ist : scn->instances) {
        auto oist = new obj_instance();
        oist->name = ist->name;
        oist->objname = (ist->shp) ? ist->shp->name : "<undefined>";
        oist->frame = ist->frame;
        obj->instances.emplace_back(oist);
    }

    return obj;
}

// Save an obj scene
inline void save_obj_scene(
    const string& filename, const scene* scn, const save_options& opts) {
    auto oscn = unique_ptr<obj_scene>(scene_to_obj(scn));
    save_obj(filename, oscn.get(), opts.save_textures, opts.skip_missing,
        opts.obj_flip_texcoord, opts.obj_flip_tr);
}

// Instance gltf cameras and meshes
inline void gltf_node_to_instances(scene* scn, const vector<camera>& cameras,
    const vector<vector<shape*>>& meshes, const glTF* gltf,
    glTFid<glTFNode> nid, const mat4f& xf) {
    auto nde = gltf->get(nid);
    auto xform = xf * node_transform(nde);
    if (nde->camera) {
        auto cam = new camera(cameras[(int)nde->camera]);
        cam->frame = to_frame(xform);
        scn->cameras.push_back(cam);
    }
    if (nde->mesh) {
        for (auto shp : meshes[(int)nde->mesh]) {
            auto ist = new instance();
            ist->name = nde->name;
            ist->frame = to_frame(xform);
            ist->shp = shp;
            scn->instances.push_back(ist);
        }
    }
    for (auto cid : nde->children)
        gltf_node_to_instances(scn, cameras, meshes, gltf, cid, xform);
}

// Flattens a gltf file into a flattened asset.
inline scene* gltf_to_scene(const glTF* gltf) {
    // clear asset
    auto scn = new scene();

    // convert images
    for (auto gtxt : gltf->images) {
        auto txt = new texture();
        txt->name = gtxt->name;
        txt->path =
            (startswith(gtxt->uri, "data:")) ? string("inlines") : gtxt->uri;
        if (!gtxt->data.datab.empty()) {
            txt->ldr = image4b(gtxt->data.width, gtxt->data.height);
            for (auto j = 0; j < gtxt->data.height; j++) {
                for (auto i = 0; i < gtxt->data.width; i++) {
                    auto v = gtxt->data.datab.data() +
                             (gtxt->data.width * j + i) * gtxt->data.ncomp;
                    switch (gtxt->data.ncomp) {
                        case 1:
                            txt->ldr.at(i, j) = {v[0], v[0], v[0], 255};
                            break;
                        case 2: txt->ldr.at(i, j) = {v[0], v[1], 0, 255}; break;
                        case 3:
                            txt->ldr.at(i, j) = {v[0], v[1], v[2], 255};
                            break;
                        case 4:
                            txt->ldr.at(i, j) = {v[0], v[1], v[2], v[3]};
                            break;
                        default: assert(false); break;
                    }
                }
            }
        } else if (!gtxt->data.dataf.empty()) {
            txt->hdr = image4f(gtxt->data.width, gtxt->data.height);
            for (auto j = 0; j < gtxt->data.height; j++) {
                for (auto i = 0; i < gtxt->data.width; i++) {
                    auto v = gtxt->data.dataf.data() +
                             (gtxt->data.width * j + i) * gtxt->data.ncomp;
                    switch (gtxt->data.ncomp) {
                        case 1:
                            txt->hdr.at(i, j) = {v[0], v[0], v[0], 1};
                            break;
                        case 2: txt->hdr.at(i, j) = {v[0], v[1], 0, 1}; break;
                        case 3:
                            txt->hdr.at(i, j) = {v[0], v[1], v[2], 1};
                            break;
                        case 4:
                            txt->hdr.at(i, j) = {v[0], v[1], v[2], v[3]};
                            break;
                        default: assert(false); break;
                    }
                }
            }
        }
        scn->textures.push_back(txt);
    }

    // add a texture
    auto add_texture = [gltf, scn](glTFTextureInfo* ginfo, bool normal = false,
                           bool occlusion = false) {
        auto info = texture_info();
        if (!ginfo) return info;
        auto gtxt = gltf->get(ginfo->index);
        if (!gtxt || !gtxt->source) return info;
        auto txt = scn->textures.at((int)gtxt->source);
        if (!txt) return info;
        info.txt = scn->textures.at((int)gtxt->source);
        auto gsmp = gltf->get(gtxt->sampler);
        if (gsmp) {
            info.linear = gsmp->magFilter != glTFSamplerMagFilter::Nearest;
            info.mipmap = gsmp->minFilter != glTFSamplerMinFilter::Linear &&
                          gsmp->minFilter != glTFSamplerMinFilter::Nearest;
            info.wrap_s = gsmp->wrapS != glTFSamplerWrapS::ClampToEdge;
            info.wrap_t = gsmp->wrapT != glTFSamplerWrapT::ClampToEdge;
        }
        if (normal) {
            auto ninfo = (glTFMaterialNormalTextureInfo*)ginfo;
            info.scale = ninfo->scale;
        }
        if (occlusion) {
            auto ninfo = (glTFMaterialOcclusionTextureInfo*)ginfo;
            info.scale = ninfo->strength;
        }
        return info;
    };

    // convert materials
    for (auto gmat : gltf->materials) {
        auto mat = new material();
        mat->name = gmat->name;
        mat->ke = gmat->emissiveFactor;
        mat->ke_txt = add_texture(gmat->emissiveTexture);
        if (gmat->pbrMetallicRoughness) {
            mat->mtype = material_type::metallic_roughness;
            auto gmr = gmat->pbrMetallicRoughness;
            mat->kd = {gmr->baseColorFactor[0], gmr->baseColorFactor[1],
                gmr->baseColorFactor[2]};
            mat->op = gmr->baseColorFactor[3];
            mat->ks = {
                gmr->metallicFactor, gmr->metallicFactor, gmr->metallicFactor};
            mat->rs = gmr->roughnessFactor;
            mat->kd_txt = add_texture(gmr->baseColorTexture);
            mat->ks_txt = add_texture(gmr->metallicRoughnessTexture);
        }
        if (gmat->pbrSpecularGlossiness) {
            mat->mtype = material_type::specular_glossiness;
            auto gsg = gmat->pbrSpecularGlossiness;
            mat->kd = {gsg->diffuseFactor[0], gsg->diffuseFactor[1],
                gsg->diffuseFactor[2]};
            mat->op = gsg->diffuseFactor[3];
            mat->ks = gsg->specularFactor;
            mat->rs = gsg->glossinessFactor;
            mat->kd_txt = add_texture(gsg->diffuseTexture);
            mat->ks_txt = add_texture(gsg->specularGlossinessTexture);
        }
        mat->norm_txt = add_texture(gmat->normalTexture, true, false);
        mat->occ_txt = add_texture(gmat->occlusionTexture, false, true);
        mat->double_sided = gmat->doubleSided;
        scn->materials.push_back(mat);
    }

    // convert meshes
    auto meshes = vector<vector<shape*>>();
    for (auto gmesh : gltf->meshes) {
        meshes.push_back({});
        // primitives
        for (auto gprim : gmesh->primitives) {
            auto shp = new shape();
            if (gprim->material) {
                shp->mat = scn->materials[(int)gprim->material];
            }
            // vertex data
            for (auto gattr : gprim->attributes) {
                auto semantic = gattr.first;
                auto vals = accessor_view(gltf, gltf->get(gattr.second));
                if (semantic == "POSITION") {
                    shp->pos.reserve(vals.size());
                    for (auto i = 0; i < vals.size(); i++)
                        shp->pos.push_back(vals.getv<3>(i));
                } else if (semantic == "NORMAL") {
                    shp->norm.reserve(vals.size());
                    for (auto i = 0; i < vals.size(); i++)
                        shp->norm.push_back(vals.getv<3>(i));
                } else if (semantic == "TEXCOORD" || semantic == "TEXCOORD_0") {
                    shp->texcoord.reserve(vals.size());
                    for (auto i = 0; i < vals.size(); i++)
                        shp->texcoord.push_back(vals.getv<2>(i));
                } else if (semantic == "TEXCOORD_1") {
                    shp->texcoord1.reserve(vals.size());
                    for (auto i = 0; i < vals.size(); i++)
                        shp->texcoord1.push_back(vals.getv<2>(i));
                } else if (semantic == "COLOR" || semantic == "COLOR_0") {
                    shp->color.reserve(vals.size());
                    for (auto i = 0; i < vals.size(); i++)
                        shp->color.push_back(vals.getv<4>(i, {0, 0, 0, 1}));
                } else if (semantic == "TANGENT") {
                    shp->tangsp.reserve(vals.size());
                    for (auto i = 0; i < vals.size(); i++)
                        shp->tangsp.push_back(vals.getv<4>(i));
                } else if (semantic == "RADIUS") {
                    shp->radius.reserve(vals.size());
                    for (auto i = 0; i < vals.size(); i++)
                        shp->radius.push_back(vals.get(i, 0));
                } else {
                    // ignore
                }
            }
            // indices
            if (!gprim->indices) {
                switch (gprim->mode) {
                    case glTFMeshPrimitiveMode::Triangles: {
                        shp->triangles.reserve(shp->pos.size() / 3);
                        for (auto i = 0; i < shp->pos.size() / 3; i++) {
                            shp->triangles.push_back(
                                {i * 3 + 0, i * 3 + 1, i * 3 + 2});
                        }
                    } break;
                    case glTFMeshPrimitiveMode::TriangleFan: {
                        shp->triangles.reserve(shp->pos.size() - 2);
                        for (auto i = 2; i < shp->pos.size(); i++) {
                            shp->triangles.push_back({0, i - 1, i});
                        }
                    } break;
                    case glTFMeshPrimitiveMode::TriangleStrip: {
                        shp->triangles.reserve(shp->pos.size() - 2);
                        for (auto i = 2; i < shp->pos.size(); i++) {
                            shp->triangles.push_back({i - 2, i - 1, i});
                        }
                    } break;
                    case glTFMeshPrimitiveMode::Lines: {
                        shp->lines.reserve(shp->pos.size() / 2);
                        for (auto i = 0; i < shp->pos.size() / 2; i++) {
                            shp->lines.push_back({i * 2 + 0, i * 2 + 1});
                        }
                    } break;
                    case glTFMeshPrimitiveMode::LineLoop: {
                        shp->lines.reserve(shp->pos.size());
                        for (auto i = 1; i < shp->pos.size(); i++) {
                            shp->lines.push_back({i - 1, i});
                        }
                        shp->lines.back() = {(int)shp->pos.size() - 1, 0};
                    } break;
                    case glTFMeshPrimitiveMode::LineStrip: {
                        shp->lines.reserve(shp->pos.size() - 1);
                        for (auto i = 1; i < shp->pos.size(); i++) {
                            shp->lines.push_back({i - 1, i});
                        }
                    } break;
                    case glTFMeshPrimitiveMode::NotSet:
                    case glTFMeshPrimitiveMode::Points: {
                        shp->points.reserve(shp->pos.size());
                        for (auto i = 0; i < shp->pos.size(); i++) {
                            shp->points.push_back(i);
                        }
                    } break;
                }
            } else {
                auto indices = accessor_view(gltf, gltf->get(gprim->indices));
                switch (gprim->mode) {
                    case glTFMeshPrimitiveMode::Triangles: {
                        shp->triangles.reserve(indices.size());
                        for (auto i = 0; i < indices.size() / 3; i++) {
                            shp->triangles.push_back({indices.geti(i * 3 + 0),
                                indices.geti(i * 3 + 1),
                                indices.geti(i * 3 + 2)});
                        }
                    } break;
                    case glTFMeshPrimitiveMode::TriangleFan: {
                        shp->triangles.reserve(indices.size() - 2);
                        for (auto i = 2; i < indices.size(); i++) {
                            shp->triangles.push_back({indices.geti(0),
                                indices.geti(i - 1), indices.geti(i)});
                        }
                    } break;
                    case glTFMeshPrimitiveMode::TriangleStrip: {
                        shp->triangles.reserve(indices.size() - 2);
                        for (auto i = 2; i < indices.size(); i++) {
                            shp->triangles.push_back({indices.geti(i - 2),
                                indices.geti(i - 1), indices.geti(i)});
                        }
                    } break;
                    case glTFMeshPrimitiveMode::Lines: {
                        shp->lines.reserve(indices.size() / 2);
                        for (auto i = 0; i < indices.size() / 2; i++) {
                            shp->lines.push_back({indices.geti(i * 2 + 0),
                                indices.geti(i * 2 + 1)});
                        }
                    } break;
                    case glTFMeshPrimitiveMode::LineLoop: {
                        shp->lines.reserve(indices.size());
                        for (auto i = 1; i < indices.size(); i++) {
                            shp->lines.push_back(
                                {indices.geti(i - 1), indices.geti(i)});
                        }
                        shp->lines.back() = {
                            indices.geti(indices.size() - 1), indices.geti(0)};
                    } break;
                    case glTFMeshPrimitiveMode::LineStrip: {
                        shp->lines.reserve(indices.size() - 1);
                        for (auto i = 1; i < indices.size(); i++) {
                            shp->lines.push_back(
                                {indices.geti(i - 1), indices.geti(i)});
                        }
                    } break;
                    case glTFMeshPrimitiveMode::NotSet:
                    case glTFMeshPrimitiveMode::Points: {
                        shp->points.reserve(indices.size());
                        for (auto i = 0; i < indices.size(); i++) {
                            shp->points.push_back(indices.geti(i));
                        }
                    } break;
                }
            }
            scn->shapes.push_back(shp);
            meshes.back().push_back(shp);
        }
    }

    // convert cameras
    auto cameras = vector<camera>();
    for (auto gcam : gltf->cameras) {
        cameras.push_back({});
        auto cam = &cameras.back();
        cam->name = gcam->name;
        cam->ortho = gcam->type == glTFCameraType::Orthographic;
        if (cam->ortho) {
            auto ortho = gcam->orthographic;
            cam->yfov = ortho->ymag;
            cam->aspect = ortho->xmag / ortho->ymag;
            cam->near = ortho->znear;
            cam->far = ortho->zfar;
        } else {
            auto persp = gcam->perspective;
            cam->yfov = persp->yfov;
            cam->aspect = persp->aspectRatio;
            if (!cam->aspect) cam->aspect = 16.0f / 9.0f;
            cam->near = persp->znear;
            cam->far = persp->zfar;
        }
    }

    // instance meshes and cameras
    if (gltf->scene) {
        for (auto nid : gltf->get(gltf->scene)->nodes) {
            gltf_node_to_instances(
                scn, cameras, meshes, gltf, nid, identity_mat4f);
        }
    } else if (!gltf->nodes.empty()) {
        // set up node children and root nodes
        auto is_root = vector<bool>(gltf->nodes.size(), true);
        for (auto nid = 0; nid < gltf->nodes.size(); nid++) {
            for (auto cid : gltf->get(glTFid<glTFNode>(nid))->children)
                is_root[(int)cid] = false;
        }
        for (auto nid = 0; nid < gltf->nodes.size(); nid++) {
            if (!is_root[nid]) continue;
            gltf_node_to_instances(scn, cameras, meshes, gltf,
                glTFid<glTFNode>(nid), identity_mat4f);
        }
    }

    return scn;
}

// Load an gltf scene
inline scene* load_gltf_scene(
    const string& filename, const load_options& opts) {
    auto gscn = unique_ptr<glTF>(
        load_gltf(filename, true, opts.load_textures, opts.skip_missing));
    auto scn = unique_ptr<scene>(gltf_to_scene(gscn.get()));
    if (!scn) {
        throw runtime_error("could not convert gltf scene");
        return nullptr;
    }
    return scn.release();
}

// Unflattnes gltf
inline glTF* scene_to_gltf(
    const scene* scn, const string& buffer_uri, bool separate_buffers) {
    auto gltf = unique_ptr<glTF>(new glTF());

    // add asset info
    gltf->asset = new glTFAsset();
    gltf->asset->generator = "Yocto/gltf";
    gltf->asset->version = "2.0";

    // convert cameras
    for (auto cam : scn->cameras) {
        auto gcam = new glTFCamera();
        gcam->name = cam->name;
        gcam->type = (cam->ortho) ? glTFCameraType::Orthographic :
                                    glTFCameraType::Perspective;
        if (cam->ortho) {
            auto ortho = new glTFCameraOrthographic();
            ortho->ymag = cam->yfov;
            ortho->xmag = cam->aspect * cam->yfov;
            ortho->znear = cam->near;
            ortho->znear = cam->far;
            gcam->orthographic = ortho;
        } else {
            auto persp = new glTFCameraPerspective();
            persp->yfov = cam->yfov;
            persp->aspectRatio = cam->aspect;
            persp->znear = cam->near;
            persp->zfar = cam->far;
            gcam->perspective = persp;
        }
        gltf->cameras.push_back(gcam);
    }

    // convert images
    for (auto txt : scn->textures) {
        auto gimg = new glTFImage();
        gimg->uri = txt->path;
        if (txt->hdr) {
            gimg->data.width = txt->hdr.width();
            gimg->data.height = txt->hdr.height();
            gimg->data.ncomp = 4;
            gimg->data.dataf.assign((float*)txt->hdr.data(),
                (float*)txt->hdr.data() +
                    txt->hdr.width() * txt->hdr.height() * 4);
        }
        if (txt->ldr) {
            gimg->data.width = txt->ldr.width();
            gimg->data.height = txt->ldr.height();
            gimg->data.ncomp = 4;
            gimg->data.datab.assign((uint8_t*)txt->ldr.data(),
                (uint8_t*)txt->ldr.data() +
                    txt->ldr.width() * txt->ldr.height() * 4);
        }
        gltf->images.push_back(gimg);
    }

    // index of an object
    auto index = [](const auto& vec, auto* val) -> int {
        auto pos = find(vec.begin(), vec.end(), val);
        if (pos == vec.end()) return -1;
        return (int)(pos - vec.begin());
    };

    // add a texture and sampler
    auto add_texture = [&gltf, &index, scn](const texture_info& info,
                           bool norm = false, bool occ = false) {
        if (!info.txt) return (glTFTextureInfo*)nullptr;
        auto gtxt = new glTFTexture();
        gtxt->name = info.txt->name;
        gtxt->source = glTFid<glTFImage>(index(scn->textures, info.txt));

        /// check if it is default
        auto is_default =
            info.wrap_s && info.wrap_t && info.linear && info.mipmap;

        if (!is_default) {
            auto gsmp = new glTFSampler();
            gsmp->wrapS = (info.wrap_s) ? glTFSamplerWrapS::Repeat :
                                          glTFSamplerWrapS::ClampToEdge;
            gsmp->wrapT = (info.wrap_t) ? glTFSamplerWrapT::Repeat :
                                          glTFSamplerWrapT::ClampToEdge;
            gsmp->minFilter = (info.mipmap) ?
                                  glTFSamplerMinFilter::LinearMipmapLinear :
                                  glTFSamplerMinFilter::Nearest;
            gsmp->magFilter = (info.linear) ? glTFSamplerMagFilter::Linear :
                                              glTFSamplerMagFilter::Nearest;
            gtxt->sampler = glTFid<glTFSampler>((int)gltf->samplers.size());
            gltf->samplers.push_back(gsmp);
        }
        gltf->textures.push_back(gtxt);
        if (norm) {
            auto ginfo = new glTFMaterialNormalTextureInfo();
            ginfo->index = glTFid<glTFTexture>{(int)gltf->textures.size() - 1};
            ginfo->scale = info.scale;
            return (glTFTextureInfo*)ginfo;
        } else if (occ) {
            auto ginfo = new glTFMaterialOcclusionTextureInfo();
            ginfo->index = glTFid<glTFTexture>{(int)gltf->textures.size() - 1};
            ginfo->strength = info.scale;
            return (glTFTextureInfo*)ginfo;
        } else {
            auto ginfo = new glTFTextureInfo();
            ginfo->index = glTFid<glTFTexture>{(int)gltf->textures.size() - 1};
            return ginfo;
        }
    };

    // convert materials
    for (auto mat : scn->materials) {
        auto gmat = new glTFMaterial();
        gmat->name = mat->name;
        gmat->emissiveFactor = mat->ke;
        gmat->emissiveTexture = add_texture(mat->ke_txt);
        switch (mat->mtype) {
            case material_type::specular_roughness: {
                gmat->pbrSpecularGlossiness =
                    new glTFMaterialPbrSpecularGlossiness();
                auto gsg = gmat->pbrSpecularGlossiness;
                gsg->diffuseFactor = {
                    mat->kd[0], mat->kd[1], mat->kd[2], mat->op};
                gsg->specularFactor = mat->ks;
                gsg->glossinessFactor = mat->rs;
                gsg->diffuseTexture = add_texture(mat->kd_txt);
                gsg->specularGlossinessTexture = add_texture(mat->ks_txt);
            } break;
            case material_type::metallic_roughness: {
                gmat->pbrMetallicRoughness =
                    new glTFMaterialPbrMetallicRoughness();
                auto gmr = gmat->pbrMetallicRoughness;
                gmr->baseColorFactor = {
                    mat->kd.x, mat->kd.y, mat->kd.z, mat->op};
                gmr->metallicFactor = mat->ks.x;
                gmr->roughnessFactor = mat->rs;
                gmr->baseColorTexture = add_texture(mat->kd_txt);
                gmr->metallicRoughnessTexture = add_texture(mat->ks_txt);
            } break;
            case material_type::specular_glossiness: {
                gmat->pbrSpecularGlossiness =
                    new glTFMaterialPbrSpecularGlossiness();
                auto gsg = gmat->pbrSpecularGlossiness;
                gsg->diffuseFactor = {
                    mat->kd[0], mat->kd[1], mat->kd[2], mat->op};
                gsg->specularFactor = mat->ks;
                gsg->glossinessFactor = mat->rs;
                gsg->diffuseTexture = add_texture(mat->kd_txt);
                gsg->specularGlossinessTexture = add_texture(mat->ks_txt);
            } break;
        }
        gmat->normalTexture = (glTFMaterialNormalTextureInfo*)add_texture(
            mat->norm_txt, true, false);
        gmat->occlusionTexture = (glTFMaterialOcclusionTextureInfo*)add_texture(
            mat->occ_txt, false, true);
        gmat->doubleSided = mat->double_sided;
        gltf->materials.push_back(gmat);
    }

    // add buffer
    auto add_buffer = [&gltf](const string& buffer_uri) {
        auto gbuffer = new glTFBuffer();
        gltf->buffers.push_back(gbuffer);
        gbuffer->uri = buffer_uri;
        return gbuffer;
    };

    // init buffers
    auto gbuffer_global = add_buffer(buffer_uri);

    // add an optional buffer
    auto add_opt_buffer = [&gbuffer_global, buffer_uri, &add_buffer,
                              separate_buffers](const string& uri) {
        if (separate_buffers && uri != "") {
            return add_buffer(uri);
        } else {
            if (!gbuffer_global) gbuffer_global = add_buffer(buffer_uri);
            return gbuffer_global;
        }
    };

    // attribute handling
    auto add_accessor = [&gltf, &index](glTFBuffer* gbuffer, const string& name,
                            glTFAccessorType type,
                            glTFAccessorComponentType ctype, int count,
                            int csize, const void* data, bool save_min_max) {
        gltf->bufferViews.push_back(new glTFBufferView());
        auto bufferView = gltf->bufferViews.back();
        bufferView->buffer = glTFid<glTFBuffer>(index(gltf->buffers, gbuffer));
        bufferView->byteOffset = (int)gbuffer->data.size();
        bufferView->byteStride = 0;
        bufferView->byteLength = count * csize;
        gbuffer->data.resize(gbuffer->data.size() + bufferView->byteLength);
        gbuffer->byteLength += bufferView->byteLength;
        auto ptr = gbuffer->data.data() + gbuffer->data.size() -
                   bufferView->byteLength;
        bufferView->target = glTFBufferViewTarget::ArrayBuffer;
        memcpy(ptr, data, bufferView->byteLength);
        gltf->accessors.push_back(new glTFAccessor());
        auto accessor = gltf->accessors.back();
        accessor->bufferView =
            glTFid<glTFBufferView>((int)gltf->bufferViews.size() - 1);
        accessor->byteOffset = 0;
        accessor->componentType = ctype;
        accessor->count = count;
        accessor->type = type;
        if (save_min_max && count &&
            ctype == glTFAccessorComponentType::Float) {
            switch (type) {
                case glTFAccessorType::Scalar: {
                    auto bbox = make_bbox(count, (vec1f*)data);
                    accessor->min = {bbox.min.x};
                    accessor->max = {bbox.max.x};
                } break;
                case glTFAccessorType::Vec2: {
                    auto bbox = make_bbox(count, (vec2f*)data);
                    accessor->min = {bbox.min.x, bbox.min.y};
                    accessor->max = {bbox.max.x, bbox.max.y};
                } break;
                case glTFAccessorType::Vec3: {
                    auto bbox = make_bbox(count, (vec3f*)data);
                    accessor->min = {bbox.min.x, bbox.min.y, bbox.min.z};
                    accessor->max = {bbox.max.x, bbox.max.y, bbox.max.z};
                } break;
                case glTFAccessorType::Vec4: {
                    auto bbox = make_bbox(count, (vec4f*)data);
                    accessor->min = {
                        bbox.min.x, bbox.min.y, bbox.min.z, bbox.min.w};
                    accessor->max = {
                        bbox.max.x, bbox.max.y, bbox.max.z, bbox.max.w};
                } break;
                default: break;
            }
        }
        return glTFid<glTFAccessor>((int)gltf->accessors.size() - 1);
    };

    // convert meshes
    for (auto shp : scn->shapes) {
        auto gbuffer = add_opt_buffer(shp->path);
        auto gmesh = new glTFMesh();
        gmesh->name = shp->name;
        auto gprim = new glTFMeshPrimitive();
        gprim->material = glTFid<glTFMaterial>(index(scn->materials, shp->mat));
        if (!shp->pos.empty())
            gprim->attributes["POSITION"] =
                add_accessor(gbuffer, shp->name + "_pos",
                    glTFAccessorType::Vec3, glTFAccessorComponentType::Float,
                    (int)shp->pos.size(), sizeof(vec3f), shp->pos.data(), true);
        if (!shp->norm.empty())
            gprim->attributes["NORMAL"] = add_accessor(gbuffer,
                shp->name + "_norm", glTFAccessorType::Vec3,
                glTFAccessorComponentType::Float, (int)shp->norm.size(),
                sizeof(vec3f), shp->norm.data(), false);
        if (!shp->texcoord.empty())
            gprim->attributes["TEXCOORD_0"] = add_accessor(gbuffer,
                shp->name + "_texcoord", glTFAccessorType::Vec2,
                glTFAccessorComponentType::Float, (int)shp->texcoord.size(),
                sizeof(vec2f), shp->texcoord.data(), false);
        if (!shp->texcoord1.empty())
            gprim->attributes["TEXCOORD_1"] = add_accessor(gbuffer,
                shp->name + "_texcoord1", glTFAccessorType::Vec2,
                glTFAccessorComponentType::Float, (int)shp->texcoord1.size(),
                sizeof(vec2f), shp->texcoord1.data(), false);
        if (!shp->color.empty())
            gprim->attributes["COLOR_0"] = add_accessor(gbuffer,
                shp->name + "_color", glTFAccessorType::Vec4,
                glTFAccessorComponentType::Float, (int)shp->color.size(),
                sizeof(vec4f), shp->color.data(), false);
        if (!shp->radius.empty())
            gprim->attributes["RADIUS"] = add_accessor(gbuffer,
                shp->name + "_radius", glTFAccessorType::Scalar,
                glTFAccessorComponentType::Float, (int)shp->radius.size(),
                sizeof(float), shp->radius.data(), false);
        // auto elem_as_uint = shp->pos.size() >
        // numeric_limits<unsigned short>::max();
        if (!shp->points.empty()) {
            gprim->indices = add_accessor(gbuffer, shp->name + "_points",
                glTFAccessorType::Scalar,
                glTFAccessorComponentType::UnsignedInt, (int)shp->points.size(),
                sizeof(int), (int*)shp->points.data(), false);
            gprim->mode = glTFMeshPrimitiveMode::Points;
        } else if (!shp->lines.empty()) {
            gprim->indices = add_accessor(gbuffer, shp->name + "_lines",
                glTFAccessorType::Scalar,
                glTFAccessorComponentType::UnsignedInt,
                (int)shp->lines.size() * 2, sizeof(int),
                (int*)shp->lines.data(), false);
            gprim->mode = glTFMeshPrimitiveMode::Lines;
        } else if (!shp->triangles.empty()) {
            gprim->indices = add_accessor(gbuffer, shp->name + "_triangles",
                glTFAccessorType::Scalar,
                glTFAccessorComponentType::UnsignedInt,
                (int)shp->triangles.size() * 3, sizeof(int),
                (int*)shp->triangles.data(), false);
            gprim->mode = glTFMeshPrimitiveMode::Triangles;
        } else if (!shp->quads.empty()) {
            auto triangles = convert_quads_to_triangles(shp->quads);
            gprim->indices = add_accessor(gbuffer, shp->name + "_quads",
                glTFAccessorType::Scalar,
                glTFAccessorComponentType::UnsignedInt,
                (int)triangles.size() * 3, sizeof(int), (int*)triangles.data(),
                false);
            gprim->mode = glTFMeshPrimitiveMode::Triangles;
        } else {
            assert(false);
        }
        gmesh->primitives.push_back(gprim);
        gltf->meshes.push_back(gmesh);
    }

    // instances
    for (auto ist : scn->instances) {
        auto gnode = new glTFNode();
        gnode->name = ist->name;
        gnode->mesh = glTFid<glTFMesh>(index(scn->shapes, ist->shp));
        gnode->matrix = to_mat(ist->frame);
        gltf->nodes.push_back(gnode);
    }

    // cameras
    for (auto cam : scn->cameras) {
        auto gnode = new glTFNode();
        gnode->name = cam->name;
        gnode->camera = glTFid<glTFCamera>(index(scn->cameras, cam));
        gnode->matrix = to_mat(cam->frame);
        gltf->nodes.push_back(gnode);
    }

    // scenes
    if (!gltf->nodes.empty()) {
        auto gscene = new glTFScene();
        gscene->name = "scene";
        for (auto i = 0; i < gltf->nodes.size(); i++) {
            gscene->nodes.push_back(glTFid<glTFNode>(i));
        }
        gltf->scenes.push_back(gscene);
        gltf->scene = glTFid<glTFScene>(0);
    }

    // done
    return gltf.release();
}

// Load a scene
inline scene* load_scene(const string& filename, const load_options& opts) {
    auto ext = path_extension(filename);
    if (ext == ".obj" || ext == ".OBJ") return load_obj_scene(filename, opts);
    if (ext == ".gltf" || ext == ".GLTF")
        return load_gltf_scene(filename, opts);
    throw runtime_error("unsupported extension " + ext);
    return nullptr;
}

// Save a gltf scene
inline void save_gltf_scene(
    const string& filename, const scene* scn, const save_options& opts) {
    auto buffer_uri = path_basename(filename) + ".bin";
    auto gscn = unique_ptr<glTF>(
        scene_to_gltf(scn, buffer_uri, opts.gltf_separate_buffers));
    save_gltf(filename, gscn.get(), true, opts.save_textures);
}

// Save a scene
inline void save_scene(
    const string& filename, const scene* scn, const save_options& opts) {
    auto ext = path_extension(filename);
    if (ext == ".obj" || ext == ".OBJ")
        return save_obj_scene(filename, scn, opts);
    if (ext == ".gltf" || ext == ".GLTF")
        return save_gltf_scene(filename, scn, opts);
    throw runtime_error("unsupported extension " + ext);
}

// Add missing values and elements
inline void add_elements(scene* scn, const add_elements_options& opts) {
    if (opts.smooth_normals) {
        for (auto shp : scn->shapes) {
            if (!shp->norm.empty()) continue;
            shp->norm.resize(shp->pos.size());
            if (!shp->points.empty()) {
                shp->norm.assign(shp->pos.size(), {0, 0, 1});
            } else if (!shp->lines.empty()) {
                compute_tangents(shp->lines, shp->pos, shp->norm);
            } else if (!shp->triangles.empty()) {
                compute_normals(shp->triangles, shp->pos, shp->norm);
            } else if (!shp->quads.empty()) {
                compute_normals(shp->quads, shp->pos, shp->norm);
            }
        }
    }

    if (opts.tangent_space) {
        for (auto shp : scn->shapes) {
            if (!shp->tangsp.empty() || shp->triangles.empty() ||
                shp->texcoord.empty() || (shp->mat))
                continue;
            shp->tangsp.resize(shp->pos.size());
            compute_tangent_frame(shp->triangles, shp->pos, shp->norm,
                shp->texcoord, shp->tangsp);
        }
    }

    if (opts.pointline_radius > 0) {
        for (auto shp : scn->shapes) {
            if ((shp->points.empty() && shp->lines.empty()) ||
                !shp->radius.empty())
                continue;
            shp->radius.resize(shp->pos.size(), opts.pointline_radius);
        }
    }

    if (opts.texture_data) {
        for (auto txt : scn->textures) {
            if (!txt->hdr && !txt->ldr) {
                printf("unable to load texture %s\n", txt->path.c_str());
                txt->ldr = image4b(1, 1, {255, 255, 255, 255});
            }
        }
    }

    if (opts.shape_instances) {
        if (!scn->instances.empty()) return;
        for (auto shp : scn->shapes) {
            auto ist = new instance();
            ist->name = shp->name;
            ist->shp = shp;
            scn->instances.push_back(ist);
        }
    }

    if (opts.default_names || opts.default_paths) {
        auto cid = 0;
        for (auto cam : scn->cameras) {
            if (cam->name.empty())
                cam->name = "unnamed_camera_" + std::to_string(cid);
            cid++;
        }

        auto tid = 0;
        for (auto txt : scn->textures) {
            if (txt->name.empty())
                txt->name = "unnamed_texture_" + std::to_string(tid);
            tid++;
        }

        auto mid = 0;
        for (auto mat : scn->materials) {
            if (mat->name.empty())
                mat->name = "unnamed_material_" + std::to_string(mid);
            mid++;
        }

        auto sid = 0;
        for (auto shp : scn->shapes) {
            if (shp->name.empty())
                shp->name = "unnamed_shape_" + std::to_string(sid);
            sid++;
        }

        auto iid = 0;
        for (auto ist : scn->instances) {
            if (ist->name.empty())
                ist->name = "unnamed_instance_" + std::to_string(iid);
            iid++;
        }

        auto eid = 0;
        for (auto env : scn->environments) {
            if (env->name.empty())
                env->name = "unnamed_environment_" + std::to_string(eid);
            eid++;
        }
    }

    if (opts.default_paths) {
        for (auto txt : scn->textures) {
            if (txt->path != "") continue;
            txt->path = txt->name + ".png";
        }
        for (auto shp : scn->shapes) {
            if (shp->path != "") continue;
            shp->path = shp->name + ".bin";
        }
    }

    if (opts.default_camera && scn->cameras.empty()) {
        update_bounds(scn);
        auto bbox = scn->bbox;
        auto bbox_center = center(bbox);
        auto bbox_size = diagonal(bbox);
        auto bbox_msize = max(bbox_size[0], max(bbox_size[1], bbox_size[2]));
        // set up camera
        auto cam = new camera();
        cam->name = "default_camera";
        auto camera_dir = vec3f{1, 0.4f, 1};
        auto from = camera_dir * bbox_msize + bbox_center;
        auto to = bbox_center;
        auto up = vec3f{0, 1, 0};
        cam->frame = lookat_frame3(from, to, up);
        cam->ortho = false;
        cam->aspect = 16.0f / 9.0f;
        cam->yfov = 2 * atanf(0.5f);
        cam->aperture = 0;
        cam->focus = length(to - from);
        scn->cameras.push_back(cam);
    }

    // default environment
    if (opts.default_environment && scn->environments.empty()) {
        auto env = new environment();
        env->name = "default_environment";
        scn->environments.push_back(env);
    }
}

// Merge scene into one another
inline void merge_into(scene* merge_into, scene* merge_from) {
    merge_into->cameras.insert(merge_from->cameras.begin(),
        merge_from->cameras.end(), merge_into->cameras.end());
    merge_from->cameras.clear();
    merge_into->textures.insert(merge_from->textures.begin(),
        merge_from->textures.end(), merge_into->textures.end());
    merge_from->textures.clear();
    merge_into->materials.insert(merge_from->materials.begin(),
        merge_from->materials.end(), merge_into->materials.end());
    merge_from->materials.clear();
    merge_into->shapes.insert(merge_from->shapes.begin(),
        merge_from->shapes.end(), merge_into->shapes.end());
    merge_from->shapes.clear();
    merge_into->instances.insert(merge_from->instances.begin(),
        merge_from->instances.end(), merge_into->instances.end());
    merge_from->instances.clear();
    merge_into->environments.insert(merge_from->environments.begin(),
        merge_from->environments.end(), merge_into->environments.end());
    merge_from->environments.clear();
}

// Initialize the lights
inline void update_lights(scene* scn, bool point_only) {
    for (auto lgt : scn->lights) delete lgt;
    scn->lights.clear();

    for (auto ist : scn->instances) {
        if (ist->shp->mat->ke == zero3f) continue;
        if (point_only && ist->shp->points.empty()) continue;
        auto lgt = new light();
        lgt->ist = ist;
        if (point_only) continue;
        auto shp = ist->shp;
        if (shp->elem_cdf.empty()) {
            if (!shp->points.empty()) {
                shp->elem_cdf = sample_points_cdf(shp->points.size());
            } else if (!ist->shp->lines.empty()) {
                shp->elem_cdf = sample_lines_cdf(shp->lines, shp->pos);
            } else if (!shp->triangles.empty()) {
                shp->elem_cdf = sample_triangles_cdf(shp->triangles, shp->pos);
            }
        }
        scn->lights.push_back(lgt);
    }

    for (auto env : scn->environments) {
        if (point_only) continue;
        if (env->ke == zero3f) continue;
        auto lgt = new light();
        lgt->env = env;
        scn->lights.push_back(lgt);
    }
}

#endif

// Print scene info (call update bounds bes before)
inline void print_info(const scene* scn) {
    auto nverts = 0, nnorms = 0, ntexcoords = 0, npoints = 0, nlines = 0,
         ntriangles = 0, nquads = 0;
    for (auto shp : scn->shapes) {
        nverts += shp->pos.size();
        nnorms += shp->norm.size();
        ntexcoords += shp->texcoord.size();
        npoints += shp->points.size();
        nlines += shp->lines.size();
        ntriangles += shp->triangles.size();
        nquads += shp->quads.size();
    }

    auto bbox = scn->bbox;
    auto bboxc = vec3f{(bbox.max[0] + bbox.min[0]) / 2,
        (bbox.max[1] + bbox.min[1]) / 2, (bbox.max[2] + bbox.min[2]) / 2};
    auto bboxs = vec3f{bbox.max[0] - bbox.min[0], bbox.max[1] - bbox.min[1],
        bbox.max[2] - bbox.min[2]};

    printf("number of cameras:      %d\n", (int)scn->cameras.size());
    printf("number of shapes:       %d\n", (int)scn->shapes.size());
    printf("number of instances:    %d\n", (int)scn->instances.size());
    printf("number of materials:    %d\n", (int)scn->materials.size());
    printf("number of textures:     %d\n", (int)scn->textures.size());
    printf("number of environments: %d\n", (int)scn->environments.size());
    printf("number of vertices:     %d\n", nverts);
    printf("number of normals:      %d\n", nnorms);
    printf("number of texcoords:    %d\n", ntexcoords);
    printf("number of points:       %d\n", npoints);
    printf("number of lines:        %d\n", nlines);
    printf("number of triangles:    %d\n", ntriangles);
    printf("number of quads:        %d\n", nquads);
    printf("\n");
    printf("bbox min:    %g %g %g\n", bbox.min[0], bbox.min[1], bbox.min[2]);
    printf("bbox max:    %g %g %g\n", bbox.max[0], bbox.max[1], bbox.max[2]);
    printf("bbox center: %g %g %g\n", bboxc[0], bboxc[1], bboxc[2]);
    printf("bbox size:   %g %g %g\n", bboxs[0], bboxs[1], bboxs[2]);
    printf("\n");
}

}  // namespace _impl_scn

#if YGL_SCENEIO

// Load a scene
inline scene* load_scene(const string& filename, const load_options& opts) {
    return _impl_scn::load_scene(filename, opts);
}

// Save a scene
inline void save_scene(
    const string& filename, const scene* scn, const save_options& opts) {
    _impl_scn::save_scene(filename, scn, opts);
}

// Add missing values and elements
inline void add_elements(scene* scn, const add_elements_options& opts) {
    _impl_scn::add_elements(scn, opts);
}

// Merge scene into one another
inline void merge_into(scene* merge_into, scene* merge_from) {
    _impl_scn::merge_into(merge_into, merge_from);
}

#endif

// Initialize the lights
inline void update_lights(scene* scn, bool point_only) {
    _impl_scn::update_lights(scn, point_only);
}

// Print scene info (call update bounds bes before)
inline void print_info(const scene* scn) { _impl_scn::print_info(scn); }

}  // namespace ygl

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR SIMPLE SCENE UI
// -----------------------------------------------------------------------------
namespace ygl {

#if YGL_SCENEUI && YGL_IMGUI

namespace __impl_scn_widgets {

inline void draw_tree_widgets(
    gl_window* win, const string& lbl, camera* cam, void*& selection) {
    draw_tree_widget_leaf(win, lbl + cam->name, selection, cam);
}

inline void draw_tree_widgets(
    gl_window* win, const string& lbl, texture* txt, void*& selection) {
    draw_tree_widget_leaf(win, lbl + txt->path, selection, txt);
}

inline void draw_tree_widgets(
    gl_window* win, const string& lbl, texture_info* info, void*& selection) {
    draw_tree_widgets(win, lbl, info->txt, selection);
}

inline void draw_tree_widgets(
    gl_window* win, const string& lbl, material* mat, void*& selection) {
    if (draw_tree_widget_begin(win, lbl + mat->name, selection, mat)) {
        if (mat->ke_txt.txt)
            draw_tree_widgets(win, "ke: ", mat->ke_txt.txt, selection);
        if (mat->kd_txt.txt)
            draw_tree_widgets(win, "kd: ", mat->kd_txt.txt, selection);
        if (mat->ks_txt.txt)
            draw_tree_widgets(win, "ks: ", mat->ks_txt.txt, selection);
        if (mat->rs_txt.txt)
            draw_tree_widgets(win, "rs: ", mat->rs_txt.txt, selection);
        if (mat->norm_txt.txt)
            draw_tree_widgets(win, "norm: ", mat->norm_txt.txt, selection);
        if (mat->bump_txt.txt)
            draw_tree_widgets(win, "bump: ", mat->bump_txt.txt, selection);
        if (mat->disp_txt.txt)
            draw_tree_widgets(win, "disp: ", mat->disp_txt.txt, selection);
        draw_tree_widget_end(win);
    }
}

inline void draw_tree_widgets(
    gl_window* win, const string& lbl, shape* shp, void*& selection) {
    if (draw_tree_widget_begin(win, lbl + shp->name, selection, shp)) {
        if (shp->mat) draw_tree_widgets(win, "mat: ", shp->mat, selection);
        draw_tree_widget_end(win);
    }
}

inline void draw_tree_widgets(
    gl_window* win, const string& lbl, instance* ist, void*& selection) {
    if (draw_tree_widget_begin(win, lbl + ist->name, selection, ist)) {
        if (ist->shp) draw_tree_widgets(win, "shape: ", ist->shp, selection);
        draw_tree_widget_end(win);
    }
}

inline void draw_tree_widgets(
    gl_window* win, const string& lbl, scene* scn, void*& selection) {
    if (draw_tree_widget_begin(win, lbl + "cameras")) {
        for (auto cam : scn->cameras)
            draw_tree_widgets(win, "", cam, selection);
        draw_tree_widget_end(win);
    }

    if (draw_tree_widget_begin(win, lbl + "shapes")) {
        for (auto msh : scn->shapes) draw_tree_widgets(win, "", msh, selection);
        draw_tree_widget_end(win);
    }

    if (draw_tree_widget_begin(win, lbl + "instances")) {
        for (auto ist : scn->instances)
            draw_tree_widgets(win, "", ist, selection);
        draw_tree_widget_end(win);
    }

    if (draw_tree_widget_begin(win, lbl + "materials")) {
        for (auto mat : scn->materials)
            draw_tree_widgets(win, "", mat, selection);
        draw_tree_widget_end(win);
    }

    if (draw_tree_widget_begin(win, lbl + "textures")) {
        for (auto txt : scn->textures)
            draw_tree_widgets(win, "", txt, selection);
        draw_tree_widget_end(win);
    }
}

inline bool draw_elem_widgets(gl_window* win, scene* scn, texture* txt,
    void*& selection, const unordered_map<texture*, gl_texture>& gl_txt) {
    auto edited = false;
    draw_separator_widget(win);
    draw_label_widget(win, "path", txt->path);
    auto size = formatf("%d x %d @ 4 %s", txt->width(), txt->height(),
        (txt->ldr) ? "byte" : "float");
    draw_label_widget(win, "size", size);
    if (contains(gl_txt, txt)) {
        draw_image_widget(win, get_texture_id(gl_txt.at(txt)), {128, 128},
            {txt->width(), txt->height()});
    }
    return edited;
}

inline bool draw_elem_widgets(gl_window* win, scene* scn, material* mat,
    void*& selection, const unordered_map<texture*, gl_texture>& gl_txt) {
    auto edited = false;
    static auto mtype_names = vector<pair<string, material_type>>{
        {"generic", material_type::specular_roughness},
        {"metallic_roughness", material_type::metallic_roughness},
        {"specular_glossiness", material_type::specular_glossiness},
    };

    auto txt_names = vector<pair<string, texture*>>{{"<none>", nullptr}};
    for (auto txt : scn->textures) txt_names.push_back({txt->path, txt});

    draw_separator_widget(win);
    draw_label_widget(win, "name", mat->name);
    edited = edited || draw_value_widget(win, "mtype", mat->mtype, mtype_names);
    edited = edited || draw_value_widget(win, "ke", mat->ke, 0, 1000);
    edited = edited || draw_value_widget(win, "kd", mat->kd, 0, 1);
    edited = edited || draw_value_widget(win, "ks", mat->ks, 0, 1);
    edited = edited || draw_value_widget(win, "kt", mat->kt, 0, 1);
    edited = edited || draw_value_widget(win, "rs", mat->rs, 0, 1);

    auto txt_widget = [&txt_names](gl_window* win, const string& lbl,
                          texture_info& info) {
        auto edited = false;
        edited = edited || draw_value_widget(win, lbl, info.txt, txt_names);
        if (info.txt) {
            edited =
                edited || draw_value_widget(win, lbl + " wrap_s", info.wrap_s);
            edited =
                edited || draw_value_widget(win, lbl + " wrap_t", info.wrap_t);
            edited =
                edited || draw_value_widget(win, lbl + " linear", info.linear);
            edited =
                edited || draw_value_widget(win, lbl + " mipmap", info.mipmap);
        }
        return edited;
    };
    edited = edited || txt_widget(win, "ke_txt", mat->ke_txt);
    edited = edited || txt_widget(win, "kd_txt", mat->kd_txt);
    edited = edited || txt_widget(win, "ks_txt", mat->ks_txt);
    edited = edited || txt_widget(win, "kt_txt", mat->kt_txt);
    edited = edited || txt_widget(win, "norm_txt", mat->norm_txt);
    edited = edited || txt_widget(win, "bump_txt", mat->bump_txt);
    edited = edited || txt_widget(win, "disp_txt", mat->disp_txt);
    return edited;
}

inline bool draw_elem_widgets(gl_window* win, scene* scn, shape* shp,
    void*& selection, const unordered_map<texture*, gl_texture>& gl_txt) {
    auto mat_names = vector<pair<string, material*>>{{"<none>", nullptr}};
    for (auto mat : scn->materials) mat_names.push_back({mat->name, mat});

    auto edited = false;
    draw_separator_widget(win);
    draw_label_widget(win, "name", shp->name);
    edited = edited || draw_value_widget(win, "material", shp->mat, mat_names);
    draw_label_widget(win, "verts", (int)shp->pos.size());
    if (!shp->triangles.empty())
        draw_label_widget(win, "triangles", (int)shp->triangles.size());
    if (!shp->lines.empty())
        draw_label_widget(win, "lines", (int)shp->lines.size());
    if (!shp->points.empty())
        draw_label_widget(win, "points", (int)shp->points.size());
    return edited;
}

inline bool draw_elem_widgets(gl_window* win, scene* scn, camera* cam,
    void*& selection, const unordered_map<texture*, gl_texture>& gl_txt) {
    auto edited = false;
    draw_separator_widget(win);
    draw_label_widget(win, "name", cam->name);
    edited = edited || draw_value_widget(win, "frame", cam->frame, -10, 10);
    edited = edited || draw_value_widget(win, "yfov", cam->yfov, 0.1, 4);
    edited = edited || draw_value_widget(win, "aspect", cam->aspect, 0.1, 4);
    edited = edited || draw_value_widget(win, "focus", cam->focus, 0.01, 10);
    edited = edited || draw_value_widget(win, "aperture", cam->aperture, 0, 1);
    return edited;
}

inline bool draw_elem_widgets(gl_window* win, scene* scn, instance* ist,
    void*& selection, const unordered_map<texture*, gl_texture>& gl_txt) {
    auto shp_names = vector<pair<string, shape*>>{{"<none>", nullptr}};
    for (auto shp : scn->shapes) shp_names.push_back({shp->name, shp});

    auto edited = false;
    draw_separator_widget(win);
    draw_label_widget(win, "name", ist->name);
    draw_value_widget(win, "frame", ist->frame, -10, 10);
    draw_value_widget(win, "shape", ist->shp, shp_names);
    return edited;
}

inline bool draw_elem_widgets(gl_window* win, scene* scn, void*& selection,
    const unordered_map<texture*, gl_texture>& gl_txt) {
    for (auto cam : scn->cameras) {
        if (cam == selection)
            return draw_elem_widgets(win, scn, cam, selection, gl_txt);
    }

    for (auto shp : scn->shapes) {
        if (shp == selection)
            return draw_elem_widgets(win, scn, shp, selection, gl_txt);
    }

    for (auto ist : scn->instances) {
        if (ist == selection)
            return draw_elem_widgets(win, scn, ist, selection, gl_txt);
    }

    for (auto mat : scn->materials) {
        if (mat == selection)
            return draw_elem_widgets(win, scn, mat, selection, gl_txt);
    }

    for (auto txt : scn->textures) {
        if (txt == selection)
            return draw_elem_widgets(win, scn, txt, selection, gl_txt);
    }

    return false;
}

inline bool draw_scene_widgets(gl_window* win, const string& lbl, scene* scn,
    void*& selection, const unordered_map<texture*, gl_texture>& gl_txt) {
    if (draw_header_widget(win, lbl)) {
        draw_scroll_widget_begin(win, "model", 240, false);
        draw_tree_widgets(win, "", scn, selection);
        draw_scroll_widget_end(win);
        return draw_elem_widgets(win, scn, selection, gl_txt);
    } else
        return false;
}

#if 0
        inline void draw_edit_widgets(gl_window* win, scene* scn,
            void*&  selection, const yshade_state* state) {
            static auto shape_names =
                vector<pair<string, int>>{{"cube", 0}, {"sphere", 1}};
            static auto shape_type = 0;
            static char txt_filename[1024] = "grid.png";

            auto selected_ist = (instance*)nullptr;
            auto selected_shp = (shape*)nullptr;
            auto selected_cam = (camera*)nullptr;
            auto selected_txt = (texture*)nullptr;
            auto selected_mat = (material*)nullptr;

            if (*selection) {
                for (auto ptr : scn->instances)
                    if (ptr == *selection) selected_node = ptr;
                for (auto ptr : scn->scenes)
                    if (ptr == *selection) selected_scene = ptr;
                for (auto ptr : scn->meshes)
                    if (ptr == *selection) selected_mesh = ptr;
                for (auto ptr : scn->cameras)
                    if (ptr == *selection) selected_cam = ptr;
                for (auto ptr : scn->materials)
                    if (ptr == *selection) selected_mat = ptr;
                for (auto ptr : scn->textures)
                    if (ptr == *selection) selected_txt = ptr;
            }

            static auto auto_parent = true;
            draw_value_widget(win, "set parent from selection", &auto_parent);

            if (draw_button_widget(win, "add mesh")) {
                static auto count = 0;
                auto mesh = new ygltf::mesh();
                mesh->name = "<new mesh " + to_string(count++) + ">";
                auto shp = new ygltf::shape();
                mesh->shapes.push_back(shp);
                shp->name = "<new shape " + to_string(count - 1) + ">";
                switch (shape_type) {
                    case 0: {
                        make_uvcube(
                            1, 1, shp->triangles, shp->pos, shp->norm, shp->texcoord);
                    } break;
                    case 1: {
                        make_uvsphere(
                            1, 1, shp->triangles, shp->pos, shp->norm, shp->texcoord);
                    } break;
                }
                if (auto_parent && selected_node) selected_node->msh = mesh;
                gscn->meshes.push_back(mesh);
                *selection = mesh;
            }
            draw_value_widget(win, "shape type", &shape_type, shape_names);

            if (draw_button_widget(win, "add camera")) {
                static auto count = 0;
                auto cam = new ygltf::camera();
                cam->name = "<new camera " + to_string(count++) + ">";
                if (auto_parent && selected_node) selected_node->cam = cam;
                gscn->cameras.push_back(cam);
                *selection = cam;
            }

            if (draw_button_widget(win, "add node")) {
                static auto count = 0;
                auto node = new ygltf::node();
                node->name = "<new node " + to_string(count++) + ">";
                if (auto_parent && selected_node)
                    selected_node->children.push_back(node);
                gscn->nodes.push_back(node);
                *selection = node;
            }

            if (draw_button_widget(win, "add texture")) {
                static auto count = 0;
                auto txt = new ygltf::texture();
                txt->name = "<new texture " + to_string(count++) + ">";
                txt->path = txt_filename;
                auto scn = (app_state*)get_user_pointer(win,);
                auto dirname = yu::path::get_dirname(scn->filename);
                try {
                    if (yimg::is_hdr_filename(txt->path)) {
                        txt->hdr = yimg::load_image4f(dirname + txt->path);
                    } else {
                        txt->ldr = yimg::load_image4b(dirname + txt->path);
                    }
                } catch (...) { txt->ldr = image4b(1, 1, {255, 255, 255, 255}); }
                gscn->textures.push_back(txt);
                *selection = txt;
            }
            draw_text_widget(win, "texture", txt_filename, sizeof(txt_filename));

            if (draw_button_widget(win, "delete")) {
                if (selected_cam) {
                    for (auto node : gscn->nodes)
                        if (node->cam == selected_cam) node->cam = nullptr;
                    remove(gscn->cameras, selected_cam);
                    delete selected_cam;
                    *selection = nullptr;
                }
                if (selected_mesh) {
                    for (auto node : gscn->nodes)
                        if (node->msh == selected_mesh) node->msh = nullptr;
                    remove(gscn->meshes, selected_mesh);
                    delete selected_mesh;
                    *selection = nullptr;
                }
                if (selected_mat) {
                    for (auto mesh : gscn->meshes)
                        for (auto shp : mesh->shapes)
                            if (shp->mat == selected_mat) shp->mat = nullptr;
                    remove(gscn->materials, selected_mat);
                    delete selected_mat;
                    *selection = nullptr;
                }
                if (selected_txt) {
                    for (auto mat : gscn->materials) {
                        if (mat->emission_txt == selected_txt)
                            mat->emission_txt = nullptr;
                        if (mat->normal_txt == selected_txt) mat->normal_txt = nullptr;
                        if (mat->occlusion_txt == selected_txt)
                            mat->occlusion_txt = nullptr;
                        if (mat->metallic_roughness) {
                            if (mat->metallic_roughness->base_txt == selected_txt)
                                mat->metallic_roughness->base_txt = nullptr;
                            if (mat->metallic_roughness->metallic_txt == selected_txt)
                                mat->metallic_roughness->metallic_txt = nullptr;
                        }
                        if (mat->specular_glossiness) {
                            if (mat->specular_glossiness->diffuse_txt == selected_txt)
                                mat->specular_glossiness->diffuse_txt = nullptr;
                            if (mat->specular_glossiness->specular_txt == selected_txt)
                                mat->specular_glossiness->specular_txt = nullptr;
                        }
                    }
                    remove(gscn->textures, selected_txt);
                    delete selected_txt;
                    *selection = nullptr;
                }
            }
        }
#endif

}  // namespace __impl_scn_widgets

inline bool draw_scene_widgets(gl_window* win, const string& lbl, scene* scn,
    void*& selection, const unordered_map<texture*, gl_texture>& gl_txt) {
    return __impl_scn_widgets::draw_scene_widgets(
        win, lbl, scn, selection, gl_txt);
}

#endif

}  // namespace ygl

// HACK to avoid compilation with MSVC2015 without dirtying code
#ifdef constexpr
#undef constexpr
#endif

// -----------------------------------------------------------------------------
// IMCLUDE THE IMAGEIO FUNCTIONALITY IN THE BUILD
// -----------------------------------------------------------------------------

#if YGL_IMAGEIO && YGL_IMAGEIO_IMPLEMENTATION
#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-variable"
#ifndef __clang__
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
#endif

#ifndef __clang_analyzer__

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include "stb_image_resize.h"

#define TINYEXR_IMPLEMENTATION
#include "tinyexr.h"

#ifndef _WIN32
#pragma GCC diagnostic pop
#endif

#endif

#endif

// -----------------------------------------------------------------------------
// IMCLUDE THE IMGUI FUNCTIONALITY IN THE BUILD
// -----------------------------------------------------------------------------

#if YGL_IMGUI && YGL_IMGUI_IMPLEMENTATION
#include "imgui/imgui.cpp"
#include "imgui/imgui_draw.cpp"
#include "imgui/imgui_impl_glfw_gl3.cpp"
#endif

#endif
