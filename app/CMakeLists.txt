add_executable(run_mapping_online run_mapping_online.cc)
target_link_libraries(run_mapping_online
        ${PROJECT_NAME} gflags
        ${PCL_LIBRARY_DIRS}
        /usr/local/lib/libtbb.so
        /usr/local/lib/libtbbmalloc.so
        /usr/local/lib/libtbbmalloc_proxy.so
        )
add_executable(run_mapping_offline run_mapping_offline.cc)
target_link_libraries(run_mapping_offline
        ${PROJECT_NAME} gflags
        ${PCL_LIBRARY_DIRS}
        /usr/local/lib/libtbb.so
        /usr/local/lib/libtbbmalloc.so
        /usr/local/lib/libtbbmalloc_proxy.so
        )
