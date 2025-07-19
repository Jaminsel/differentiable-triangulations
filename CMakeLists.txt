cmake_minimum_required(VERSION 3.15) 
project(WeightedTriangulationOptimization)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
    message(FATAL_ERROR "Source and build directories cannot be the same. Go use the /build directory.")
endif()

### Configure output locations
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Include vcpkg toolchain
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "C:/Users/jemme/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")
endif()

set(GMP_INCLUDE_DIR "C:/Users/jemme/vcpkg/installed/x64-windows/include")
set(GMP_LIBRARIES "C:/Users/jemme/vcpkg/installed/x64-windows/lib/gmp.lib")
set(MPFR_INCLUDE_DIR "C:/Users/jemme/vcpkg/installed/x64-windows/include")
set(MPFR_LIBRARY "C:/Users/jemme/vcpkg/installed/x64-windows/lib/mpfr.lib")
set(TINYAD_DIR "C:/Users/jemme/Documents/7.Semester_Informatik/BA/GettingStarted/WeightedTriangulationOptimization/libs/TinyAD") 
add_subdirectory(${TINYAD_DIR}) #libs/TinyAD

# include_directories(C:/Users/jemme/vcpkg/installed/x64-windows/include)
# link_directories(C:/Users/jemme/vcpkg/installed/x64-windows/lib)

# Locate CGAL
# find_package(GMP REQUIRED)
# find_package(CGAL CONFIG REQUIRED) 
find_package(CGAL REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(Clipper2 CONFIG REQUIRED)

if (NOT CGAL_FOUND)
    message(FATAL_ERROR "CGAL was not found :(")
endif()

# Compiler options
set( CMAKE_EXPORT_COMPILE_COMMANDS 1 ) # Emit a compile flags file to support completion engines 

# Compiler flags
if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang" OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  # using Clang (linux or apple) or GCC
  message("Using clang/gcc compiler flags")
  SET(BASE_CXX_FLAGS "-std=c++17 -Wall -Wextra -Werror -g3")
  SET(DISABLED_WARNINGS " -Wno-unused-parameter -Wno-unused-variable -Wno-unused-function -Wno-deprecated-declarations -Wno-missing-braces")
  SET(TRACE_INCLUDES " -H -Wno-error=unused-command-line-argument")

  if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    message("Setting clang-specific options")
    SET(BASE_CXX_FLAGS "${BASE_CXX_FLAGS} -ferror-limit=5 -fcolor-diagnostics")
    SET(CMAKE_CXX_FLAGS_DEBUG          "-fsanitize=address -fno-limit-debug-info")
  elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    SET(BASE_CXX_FLAGS "${BASE_CXX_FLAGS} -fmax-errors=5")
    message("Setting gcc-specific options")
    SET(DISABLED_WARNINGS "${DISABLED_WARNINGS} -Wno-maybe-uninitialized -Wno-format-zero-length -Wno-unused-but-set-parameter -Wno-unused-but-set-variable")
  endif()

  SET(CMAKE_CXX_FLAGS "${BASE_CXX_FLAGS} ${DISABLED_WARNINGS}")
  #SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TRACE_INCLUDES}") # uncomment if you need to track down where something is getting included from
  SET(CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXX_FLAGS_DEBUG} -g3")
  SET(CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG")
  SET(CMAKE_CXX_FLAGS_RELEASE        "-march=native -O3 -DNDEBUG")
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  # using Visual Studio C++
  message("Using Visual Studio compiler flags")
  set(BASE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  set(BASE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP") # parallel build
  SET(DISABLED_WARNINGS "${DISABLED_WARNINGS} /wd\"4267\"")  # ignore conversion to smaller type (fires more aggressively than the gcc version, which is annoying)
  SET(DISABLED_WARNINGS "${DISABLED_WARNINGS} /wd\"4244\"")  # ignore conversion to smaller type (fires more aggressively than the gcc version, which is annoying)
  SET(DISABLED_WARNINGS "${DISABLED_WARNINGS} /wd\"4305\"")  # ignore truncation on initialization
  SET(CMAKE_CXX_FLAGS "${BASE_CXX_FLAGS} ${DISABLED_WARNINGS}")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd")
  add_compile_definitions(_USE_MATH_DEFINES)

  add_definitions(/D "_CRT_SECURE_NO_WARNINGS")
  add_definitions (-DNOMINMAX)
else()
  # unrecognized
  message( FATAL_ERROR "Unrecognized compiler [${CMAKE_CXX_COMPILER_ID}]" )
endif()

include(FetchContent)

FetchContent_Declare(
  portable-file-dialogs
  GIT_REPOSITORY https://github.com/samhocevar/portable-file-dialogs.git
  GIT_TAG 0.1.0
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE)
FetchContent_MakeAvailable(portable-file-dialogs)

FetchContent_Declare(
  polyscope
  GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
  GIT_TAG master
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE)
FetchContent_MakeAvailable(polyscope)

# FetchContent_Declare(
#   eigen
#   GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
#   GIT_TAG master
#   GIT_SHALLOW TRUE
#   GIT_PROGRESS TRUE)
# FetchContent_MakeAvailable(eigen)

# Add executable
add_executable(WeightedTriangulationOptimization 
  main.cpp
  visualization_utils.cpp
  globals.cpp
  flip_utils.cpp
)

# Add header-only dependencies
target_include_directories(WeightedTriangulationOptimization PRIVATE 
  "${polyscope_SOURCE_DIR}/deps/args"
  "${polyscope_SOURCE_DIR}/deps/json/include"
  ${eigen_SOURCE_DIR}
)

# Link libraries
target_link_libraries(WeightedTriangulationOptimization PRIVATE polyscope CGAL::CGAL TinyAD  Eigen3::Eigen Clipper2::Clipper2)
#target_link_libraries(main PRIVATE CGAL::CGAL)