# G-SLAM

### Build GTSAM

```bash
mkdir -p build
cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=ON \
  -DGTSAM_WITH_TBB=ON \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF \
  -DGTSAM_BUILD_PYTHON=OFF \
  -DGTSAM_UNSTABLE_BUILD_PYTHON=OFF \
  -DCMAKE_CXX_FLAGS="-Wno-error=array-bounds -Wno-error=overloaded-virtual"

cmake --build . -j$(nproc) 
sudo cmake --install build
sudo ldconfig
cd ..

