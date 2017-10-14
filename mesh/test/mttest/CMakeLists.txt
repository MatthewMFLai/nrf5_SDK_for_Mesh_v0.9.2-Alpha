# Library for writing multithreaded tests, used to test for IRQ-safety problems.

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads)

if(CMAKE_USE_PTHREADS_INIT)
    include_directories("."
        ${${SOFTDEVICE}_INCLUDE_DIRS})
    add_library(mttest STATIC mttest.c)
    target_link_libraries(mttest Threads::Threads)
endif()

