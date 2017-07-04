@[if DEVELSPACE]@
# base dir in develspace
set(dynamic_reconfigure_BASE_DIR "@(CMAKE_CURRENT_SOURCE_DIR)")
@[else]@
# base dir in installspace
set(dynamic_reconfigure_BASE_DIR "${dynamic_reconfigure_DIR}/..")
@[end if]@

include(${dynamic_reconfigure_BASE_DIR}/cmake/dynamic_reconfigure-macros.cmake)
