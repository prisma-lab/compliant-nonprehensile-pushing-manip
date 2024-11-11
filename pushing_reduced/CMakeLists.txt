cmake_minimum_required(VERSION 3.5)
project(raisim_pushing_reduced LANGUAGES CXX C)

#==
# Dependencies
#==

find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})
find_package(X11 REQUIRED)
#find_package(pinocchio REQUIRED)

if(WIN32)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/win32/mt_release/bin")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/win32/mt_debug/bin")
    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../rsc" DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE})
    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../rsc" DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG})
else()
    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../rsc" DESTINATION ${CMAKE_BINARY_DIR}/pushing_reduced)
endif()

#change this
set(ACADOS_PATH "~/acados")

include_directories ( ${EIGEN3_INCLUDE_DIRS} )
#include_directories ( ${pinocchio_INCLUDE_DIRS})

#adding acados
include_directories("${ACADOS_PATH}/include/")
include_directories("${ACADOS_PATH}/include/blasfeo/include")
include_directories("${ACADOS_PATH}/include/hpipm/include")
include_directories(src/c_generated_code/)
include_directories(src/c_generated_code/pushing_reduced_cost/)
include_directories(src/c_generated_code/pushing_reduced_constraints)
include_directories(src/c_generated_code/pushing_reduced_model)
link_directories("${ACADOS_PATH}/lib")

message("${ACADOS_PATH}/include")

add_library(acados_solver_pushing_reduced src/c_generated_code/acados_solver_pushing_reduced.c)
add_library(libacados_sim_solver_pushing_reduced src/c_generated_code/acados_sim_solver_pushing_reduced.c)

#==
# Build Targets
#==

function(create_executable app_name file_name file_name_2)
    add_executable(${app_name} ${file_name} ${file_name_2} src/c_generated_code/pushing_reduced_model/pushing_reduced_expl_vde_forw.c src/c_generated_code/pushing_reduced_model/pushing_reduced_expl_ode_fun.c src/c_generated_code/pushing_reduced_model/pushing_reduced_expl_ode_hess.c src/c_generated_code/pushing_reduced_model/pushing_reduced_expl_vde_adj.c src/c_generated_code/pushing_reduced_constraints/pushing_reduced_constr_h_fun.c src/c_generated_code/pushing_reduced_constraints/pushing_reduced_constr_h_fun_jac_uxt_zt.c src/c_generated_code/pushing_reduced_constraints/pushing_reduced_constr_h_fun_jac_uxt_zt_hess.c)
    set_target_properties(${app_name} PROPERTIES MACOSX_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../raisim/mac/lib")
    if(WIN32)
        target_link_libraries(${app_name} PUBLIC raisim::raisim Ws2_32 Winmm)
        target_compile_options(${app_name} PRIVATE "/MP")
    else()
        #target_link_libraries(${app_name} PUBLIC raisim::raisim pthread pinocchio::pinocchio)
        target_link_libraries(${app_name} PUBLIC raisim::raisim pthread)
    endif()

    if(APPLE)
        execute_process(COMMAND sysctl -q hw.optional.arm64
                OUTPUT_VARIABLE _sysctl_stdout
                ERROR_VARIABLE _sysctl_stderr
                RESULT_VARIABLE _sysctl_result
                )
        if(_sysctl_result EQUAL 0 AND _sysctl_stdout MATCHES "hw.optional.arm64: 1")
            target_compile_options(${app_name} PRIVATE -mcpu=lightning)
        endif()
    endif()

    target_include_directories(${app_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${X11_INCLUDE_DIR} include)
    target_link_libraries(${app_name} PUBLIC ${X11_LIBRARIES} libacados_sim_solver_pushing_reduced acados_solver_pushing_reduced acados hpipm blasfeo m)

endfunction()

# server examples
#create_executable(MainPushingInv_reduced_p2p src/main_pushing_manip_inv_reduced_p2p.cpp src/Pushing_controller_reduced_p2p.cpp)
#create_executable(MainPushingInv_reduced_eight src/main_pushing_manip_inv_reduced_eight.cpp src/Pushing_controller_reduced_eight.cpp)
create_executable(pushing_p2p src/Raisim_pushing.cpp src/Pushing_controller.cpp)

