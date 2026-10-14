# Run by ctest (libs/slam/tests/CMakeLists.txt):
#   cmake -DNM=<nm> -DCXXFILT=<c++filt, may be empty> -DBASELINE=<object>
#         -DCONSUMER=<object> -DLIBRARIES=<library>|<library>|...
#         -P check_consumer_instantiations.cmake
#
# Fails when CONSUMER defines a weak function (an inline function or a template
# instantiation: nm type W) that one of LIBRARIES also defines and BASELINE does
# not. Such a function exists twice at link time and the linker may keep the
# consumer's copy for the library's own calls as well, so a consumer built at
# -O0 runs part of the library unoptimized (see follower_consumer.cpp). BASELINE
# is the same consumer without the code under test, so what an app instantiates
# anyway (the Slam, its subscriptions, reading a snapshot) is not counted.

cmake_minimum_required(VERSION 3.22)

foreach(var NM BASELINE CONSUMER LIBRARIES)
    if(NOT ${var})
        message(FATAL_ERROR "check_consumer_instantiations: ${var} is not set")
    endif()
endforeach()

# The output of `nm --defined-only` for `file`, with a trailing newline.
function(read_symbols file out_var)
    execute_process(COMMAND "${NM}" --defined-only "${file}"
        OUTPUT_VARIABLE text ERROR_VARIABLE error RESULT_VARIABLE result)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "${NM} failed on ${file}: ${error}")
    endif()
    set(${out_var} "${text}\n" PARENT_SCOPE)
endfunction()

# The mangled names of the weak functions defined in `file`.
function(read_weak_functions file out_var)
    read_symbols("${file}" text)
    string(REGEX MATCHALL "[^\n]+" lines "${text}")
    set(names "")
    foreach(line IN LISTS lines)
        if(line MATCHES "^[0-9a-fA-F]* W ([^ ]+)$")
            list(APPEND names "${CMAKE_MATCH_1}")
        endif()
    endforeach()
    list(REMOVE_DUPLICATES names)
    set(${out_var} "${names}" PARENT_SCOPE)
endfunction()

read_weak_functions("${BASELINE}" baseline)
read_weak_functions("${CONSUMER}" consumer)
string(REPLACE "|" ";" libraries "${LIBRARIES}")
set(library_text "")
foreach(library IN LISTS libraries)
    read_symbols("${library}" text)
    string(APPEND library_text "${text}")
endforeach()

# Guard against a vacuous pass (an empty object, an nm output this script does
# not parse): both objects instantiate plenty, and the baseline already shares
# a few functions with the libraries (the Slam's inline accessors, at least).
list(LENGTH baseline num_baseline)
list(LENGTH consumer num_consumer)
if(num_baseline LESS 100 OR num_consumer LESS 100)
    message(FATAL_ERROR "check_consumer_instantiations: too few weak functions parsed "
        "(baseline ${num_baseline}, consumer ${num_consumer}); is ${NM} a GNU-compatible nm?")
endif()
set(num_shared_baseline 0)
foreach(name IN LISTS baseline)
    string(FIND "${library_text}" " W ${name}\n" position)
    if(NOT position EQUAL -1)
        math(EXPR num_shared_baseline "${num_shared_baseline} + 1")
    endif()
endforeach()
if(num_shared_baseline EQUAL 0)
    message(FATAL_ERROR "check_consumer_instantiations: the baseline shares no weak function with "
        "the libraries, so the check could not see an override; are LIBRARIES right?")
endif()

# What the code under test adds, and of that what the libraries define too.
set(added "${consumer}")
list(REMOVE_ITEM added ${baseline})
set(overrides "")
foreach(name IN LISTS added)
    string(FIND "${library_text}" " W ${name}\n" position)
    if(NOT position EQUAL -1)
        list(APPEND overrides "${name}")
    endif()
endforeach()

list(LENGTH added num_added)
list(LENGTH overrides num_overrides)
message(STATUS "baseline: ${num_baseline} weak functions (${num_shared_baseline} also in the libraries); "
    "consumer adds ${num_added}, of which ${num_overrides} the libraries define too")
if(num_overrides GREATER 0)
    set(report "")
    foreach(name IN LISTS overrides)
        set(shown "${name}")
        if(CXXFILT)
            execute_process(COMMAND "${CXXFILT}" "${name}" OUTPUT_VARIABLE shown OUTPUT_STRIP_TRAILING_WHITESPACE)
        endif()
        string(APPEND report "  ${shown}\n")
    endforeach()
    message(FATAL_ERROR "The consumer instantiates ${num_overrides} function(s) that the libraries "
        "define too; built at -O0 and linked first, its copies replace the libraries' optimized "
        "ones in their own hot paths. Define the code under test in a library source "
        "(INSTANTIATE_CLASS) instead of in its header:\n${report}")
endif()
