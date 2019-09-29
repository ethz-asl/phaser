cmake_minimum_required(VERSION 3.1.0)
project(packlo)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS         "-Wall -Wextra -msse -msse2 -msse3 -msse4")
set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}_lib

   lib/SOFT/csecond.c
   lib/SOFT/s2_legendreTransforms.c
   lib/SOFT/soft_fftw_nt.c
   lib/SOFT/wrap_s2_rotate_fftw.c
   lib/SOFT/makeweights.c
   lib/SOFT/s2_primitive.c
   lib/SOFT/soft_fftw_pc.c
   lib/SOFT/wrap_soft_fftw.c
   lib/SOFT/makeWigner.c
   lib/SOFT/s2_semi_fly.c
   lib/SOFT/utils_so3.c
   lib/SOFT/wrap_soft_fftw_cor2.c
   lib/SOFT/rotate_so3_fftw.c
   lib/SOFT/s2_semi_memo.c
   lib/SOFT/utils_vec_cx.c
   lib/SOFT/rotate_so3_utils.c
   lib/SOFT/so3_correlate_fftw.c
   lib/SOFT/wignerTransforms.c
   lib/SOFT/s2_cospmls.c
   lib/SOFT/soft_fftw.c
   lib/SOFT/wignerTransforms_fftw.c

   src/model/point-cloud.cc
   src/model/function-value.cc

   src/common/datasource.cc
   src/common/spherical-projection.cc
   src/common/spherical-sampler.cc
   src/common/rotation-utils.cc
   src/common/statistics-manager.cc

   src/controller/distributor.cc
	 src/backend/registration/base-registration.cc
	 src/backend/registration/sph-registration.cc
	 src/backend/registration/sph-registration-mock-rotated.cc
	 src/backend/registration/sph-registration-mock-cutted.cc
   src/backend/correlation/spherical-correlation.cc
   src/backend/alignment/range-based-aligner.cc

   src/visualization/debug-visualizer.cc
   src/visualization/plotty-visualizer.cc

	 src/packlo-node.cc
)

cs_add_executable(${PROJECT_NAME} tools/packlo-app.cc)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib fftw3)

cs_install()
cs_export()
