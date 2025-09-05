protoc --cpp_out=. iVisionCommunication.proto
protoc --cpp_out=. CameraCommunication.proto
mv iVisionCommunication.pb.h ../inc
mv CameraCommunication.pb.h ../inc
mv iVisionCommunication.pb.cc iVisionCommunication.pb.cpp
mv CameraCommunication.pb.cc CameraCommunication.pb.cpp
