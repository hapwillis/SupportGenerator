# Disable building extras we won't need (pure C++ project)
set(NANOGUI_BUILD_EXAMPLE OFF CACHE BOOL " " FORCE)
set(NANOGUI_BUILD_PYTHON  OFF CACHE BOOL " " FORCE)
set(NANOGUI_INSTALL       OFF CACHE BOOL " " FORCE)
# you could add the NANOGUI_BUILD_SHARED part here

# Add the configurations from nanogui
add_subdirectory(ext/nanogui)  # this executes ext/nanogui/CMakeLists.txt for you

# For reliability of parallel build, make the NanoGUI targets dependencies
set_property(TARGET glfw glfw_objects nanogui PROPERTY FOLDER "dependencies")

# Various preprocessor definitions have been generated by NanoGUI
add_definitions(${NANOGUI_EXTRA_DEFS})

# On top of adding the path to nanogui/include, you may need extras
include_directories(ext/nanogui/include) # make sure you have this
include_directories(${NANOGUI_EXTRA_INCS}) # where GLFW, Eigen, etc come from

# Compile a target using NanoGUI
add_executable(myTarget myTarget.cpp)  # <-- your code

# Lastly, additional libraries may have been built for you.  In addition to linking
# against NanoGUI, we need to link against those as well.
target_link_libraries(myTarget nanogui ${NANOGUI_EXTRA_LIBS})