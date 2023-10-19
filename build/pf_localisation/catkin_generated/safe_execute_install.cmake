execute_process(COMMAND "/home/kpafg/ir_workspace/build/pf_localisation/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/kpafg/ir_workspace/build/pf_localisation/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
