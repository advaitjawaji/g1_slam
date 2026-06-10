# ===================================================================================
#  The ZED CMake configuration file
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    SET(ZED_DIR "/usr/local/zed/")
#    FIND_PACKAGE(zed REQUIRED )
#    INCLUDE_DIRECTORIES(${ZED_INCLUDE_DIRS})
#    link_directories(${ZED_LIBRARY_DIR})
#    TARGET_LINK_LIBRARIES(MY_TARGET_NAME ${ZED_LIBRARIES})
#
#    This file will define the following variables:
#      - ZED_LIBRARIES           : The list of dynamic libraries (ZED SDK) to links against.
#      - ZED_STATIC_LIBRARIES    : The list of static libraries (ZED SDK) to links against.
#      - ZED_STATIC_DEP_LIBRARIES: The list of static libraries (ZED SDK dependencies) to links against.
#      - ZED_LIBRARY_DIR         : The directory where lib files are.
#      - ZED_INCLUDE_DIRS        : The ZED include directories.
#      - ZED_CUDA_VERSION        : The CUDA version required by the ZED SDK.
#      - CUDA_DEP_LIBRARIES_ZED  : The CUDA dependencies libraries used by the ZED SDK.
# ===================================================================================

set(ZED_FOUND FALSE CACHE BOOL "" FORCE)

if(DEFINED ENV{ZED_DIR})
    set(ZED_PATH $ENV{ZED_DIR})
    message(STATUS "Using ZED SDK from custom path: ${ZED_PATH}")
else()
    set(ZED_PATH "/usr/local/zed")
endif()

set(ZED_INCLUDE_DIRS "${ZED_PATH}/include")
set(ZED_LIBRARY_DIR "${ZED_PATH}/lib")

if(ZED_INCLUDE_DIRS)

    if(LINK_SHARED_ZED)
        set(ZED_STATIC_DEPENDENCIES_SEARCH OFF)
    else()
        set(ZED_STATIC_DEPENDENCIES_SEARCH TRUE)
    endif()

    find_package(BLAS REQUIRED)
    set (LIB_PATH_64 "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/")

    set (ZED_LIBRARIES_DEBUG "${ZED_LIBRARY_DIR}/libsl_zedd.so")
    set (ZED_LIBRARIES_STATIC_DEBUG  "${ZED_LIBRARY_DIR}/libsl_zed_staticd.a")
    
    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_definitions(-D_DEBUG)
    endif()
        
    if(EXISTS ${ZED_LIBRARIES_DEBUG})
	    add_definitions(-DALLOW_BUILD_DEBUG)
    else()
        set (ZED_LIBRARIES_DEBUG "${ZED_LIBRARY_DIR}/libsl_zed.so") ##Fallback to release
        set (ZED_LIBRARIES_STATIC_DEBUG  "${ZED_LIBRARY_DIR}/libsl_zed_static.a")##Fallback to release
    endif()

    set (ZED_LIBRARIES_RELEASE "${ZED_LIBRARY_DIR}/libsl_zed.so")
    set (ZED_LIBRARIES_STATIC_RELEASE "${ZED_LIBRARY_DIR}/libsl_zed_static.a")

    set(ZED_LIBRARIES debug ${ZED_LIBRARIES_DEBUG} optimized ${ZED_LIBRARIES_RELEASE})
    set(ZED_STATIC_LIBRARIES debug ${ZED_LIBRARIES_STATIC_DEBUG} optimized ${ZED_LIBRARIES_STATIC_RELEASE})

    set(ZED_LIBRARIES
        ${ZED_LIBRARIES}
        ${BLAS_LIBRARIES}
        "${LIB_PATH_64}libusb-1.0.so")

    set(ZED_STATIC_LIBRARIES
        "${ZED_LIBRARY_DIR}/libsl_zed_static.a")

    set(ZED_FOUND TRUE CACHE BOOL "" FORCE)
    set(USE_CUVID TRUE)

    SET(ZED_CUDA_VERSION 13)

    find_package(CUDA 12 REQUIRED)
    if(CUDA_FOUND)
        set(CUDA_PATH "/usr/local/cuda-${CUDA_VERSION_MAJOR}.${CUDA_VERSION_MINOR}/lib64/")

        set(CUBLAS_STATIC_PATH "${CUDA_PATH}libcublas_static.a")
        exec_program(find ARGS "${CUDA_PATH} -type f -name libcublas_static.a" OUTPUT_VARIABLE CUBLAS_STATIC_PRESENT) 
        exec_program(find ARGS "${CUDA_PATH} -type f -name libcublasLt_static.a" OUTPUT_VARIABLE CUBLASLT_STATIC_PRESENT) 

        if(CUBLASLT_STATIC_PRESENT) # CUDA 11
            set(CUBLAS_STATIC_PATH "${CUDA_PATH}libcublas_static.a" "${CUDA_PATH}libcublasLt_static.a")
        elseif((NOT CUBLAS_STATIC_PRESENT)) # For CUDA 10.1/10.2 libcublas_static is in /usr/lib/x86_64-linux-gnu/ not in /usr/local/cuda/lib64/
            set(CUBLAS_STATIC_PATH "${LIB_PATH_64}libcublas_static.a" "${LIB_PATH_64}libcublasLt_static.a")
        endif()

  	    set(CUDA_LIBRARIES_ZED_STATIC
            "${CUDA_PATH}libcudart_static.a"
            "${CUDA_PATH}libcudadevrt.a"
            "${CUDA_PATH}libculibos.a"
            "${CUBLAS_STATIC_PATH}")
    endif()

    SET(AI_LIBS_ROOT_DIR ${CUDA_TOOLKIT_ROOT_DIR})
    exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
    if (CMAKE_SYSTEM_NAME2 MATCHES "aarch64" )
        unset(CUDA_USE_STATIC_CUDA_RUNTIME CACHE)
        option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
        set(USE_CUVID FALSE) ## cuvid not available in Tegra. Use hw enc/dec
        SET(AI_LIBS_ROOT_DIR "/usr/lib/aarch64-linux-gnu/")
    endif()

    if (USE_CUVID)
    	find_library(NVENCODEAPI_LIB nvidia-encode HINTS "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/")
        if (NVENCODEAPI_LIB)
            set(ZED_LIBRARIES ${ZED_LIBRARIES} ${NVENCODEAPI_LIB})
        endif()
    endif()

    if(ZED_STATIC_DEPENDENCIES_SEARCH)
        set(ZED_STATIC_DEP_LIBRARIES ${CUDA_LIBRARIES_ZED_STATIC} "${LIB_PATH_64}libpng.a" "${LIB_PATH_64}libturbojpeg.a" z pthread dl gomp rt "${LIB_PATH_64}libusb-1.0.so" dl)
    endif()

    if (USE_CUVID)
        exec_program(find ARGS "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/ -type f -name libnvcuvid*" OUTPUT_VARIABLE CUVID_LIBRARY) ##Try first path
        if(NOT CUVID_LIBRARY)
	        exec_program(find ARGS "/usr/lib/ -type f -name libnvcuvid*" OUTPUT_VARIABLE CUVID_LIBRARY) ##Try second path
        endif()
        find_library(NVENCODEAPI_LIB nvidia-encode HINTS "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/")
        set(ZED_STATIC_DEP_LIBRARIES ${ZED_STATIC_DEP_LIBRARIES} ${CUVID_LIBRARY} ${NVENCODEAPI_LIB})
        ## CUVID is used in desktop version.
    else()
        exec_program(find ARGS "${LIB_PATH_64}tegra/ -type f -name libnvbuf_utils*" OUTPUT_VARIABLE NVBUF_UTILS_LIBRARY)
        set(ZED_INTERNAL_STATIC_DEP_LIBRARIES ${ZED_INTERNAL_STATIC_DEP_LIBRARIES} v4l2 nvargus_socketclient nvbufsurftransform nvbufsurface EGL pthread)
        if (NVBUF_UTILS_LIBRARY)
         set(ZED_INTERNAL_STATIC_DEP_LIBRARIES ${ZED_INTERNAL_STATIC_DEP_LIBRARIES} nvbuf_utils)
        endif()
        ## v4l2 and nvbuf_utils are installed during JetPack flash in the multimedia API package
    endif()

    set(ZED_STATIC_LIBRARIES ${ZED_STATIC_LIBRARIES} ${ZED_STATIC_DEP_LIBRARIES} ${ZED_INTERNAL_STATIC_DEP_LIBRARIES})
endif()
