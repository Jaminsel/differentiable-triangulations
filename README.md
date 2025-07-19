Visualization:   Polyscope 
Meshing:         CGAL 
Differentiation: TinyAD 

Configure, Build and Run

mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=C:/Users/jemme/vcpkg/scripts/buildsystems/vcpkg.cmake ..
cmake --build .

or 

cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=C:/Users/jemme/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build 
cmake --build build --parallel
build\Debug\WeightedTriangulationOptimization.exe
.\build\bin\Debug\WeightedTriangulationOptimization.exe

cmake -DCMAKE_TOOLCHAIN_FILE=C:/Users/jemme/vcpkg/scripts/buildsystems/vcpkg.cmake -DCMAKE_VERBOSE_MAKEFILE=ON ..

cmake --build build --config Release --parallel
.\build\bin\Release\WeightedTriangulationOptimization.exe

Plot data collected during optimization
python plot_data.py