# convert_vocabulary <in.txt> <out.bin>: DBoW2 / ORB-SLAM text vocabulary to our binary format.
add_executable(convert_vocabulary main.cpp)
target_link_libraries(convert_vocabulary PRIVATE voyah_algs)
