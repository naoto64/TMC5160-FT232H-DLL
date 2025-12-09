cmake_minimum_required(VERSION 3.20)
project(tmc5160_ft232h_dll LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THIRD_PARTY_DIR "${CMAKE_CURRENT_LIST_DIR}/third_party")
set(D2XX_DIR        "${THIRD_PARTY_DIR}/d2xx")

if (CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(ARCH_DIR "x64")
else()
  set(ARCH_DIR "x86")
endif()

find_path(FTDI_D2XX_INCLUDE_DIR
  NAMES ftd2xx.h
  PATHS "${D2XX_DIR}/include" "${D2XX_DIR}"
  NO_DEFAULT_PATH
)

set(D2XX_LIB_CANDIDATES
  "${D2XX_DIR}/lib/${ARCH_DIR}"
  "${D2XX_DIR}/${ARCH_DIR}"
  "${D2XX_DIR}/amd64"
  "${D2XX_DIR}/i386"
  "${D2XX_DIR}/lib"
)

find_library(FTDI_D2XX_LIBRARY
  NAMES ftd2xx FTD2XX
  PATHS ${D2XX_LIB_CANDIDATES}
  NO_DEFAULT_PATH
)

if (NOT FTDI_D2XX_INCLUDE_DIR OR NOT FTDI_D2XX_LIBRARY)
  message(STATUS "FTDI_D2XX_INCLUDE_DIR=${FTDI_D2XX_INCLUDE_DIR}")
  message(STATUS "FTDI_D2XX_LIBRARY=${FTDI_D2XX_LIBRARY}")
  message(FATAL_ERROR "D2XX not found under third_party/d2xx")
endif()

add_library(tmc5160 SHARED
  src/dll_api.cpp
  src/ftdi_mpsse_spi.cpp
  src/tmc5160.cpp
)

target_include_directories(tmc5160 PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  "${FTDI_D2XX_INCLUDE_DIR}"
)

target_compile_definitions(tmc5160 PRIVATE
  TMC5160_DLL_EXPORTS
  NOMINMAX
  WIN32_LEAN_AND_MEAN
)

target_link_libraries(tmc5160 PRIVATE
  "${FTDI_D2XX_LIBRARY}"
)

# copy ftd2xx.dll next to tmc5160.dll
set(D2XX_DLL_CANDIDATES
  "${D2XX_DIR}/ftd2xx.dll"
  "${D2XX_DIR}/lib/${ARCH_DIR}/ftd2xx.dll"
  "${D2XX_DIR}/${ARCH_DIR}/ftd2xx.dll"
  "${D2XX_DIR}/amd64/ftd2xx.dll"
  "${D2XX_DIR}/i386/ftd2xx.dll"
)
foreach(p IN LISTS D2XX_DLL_CANDIDATES)
  if (EXISTS "${p}")
    add_custom_command(TARGET tmc5160 POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different "${p}" "$<TARGET_FILE_DIR:tmc5160>"
    )
    break()
  endif()
endforeach()
