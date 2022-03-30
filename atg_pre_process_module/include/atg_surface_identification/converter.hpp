#ifndef CONVERTER_HPP
#define CONVERTER_HPP
#include <string>

class Converter
{
public:
  static void pcd2obj(const std::string& inputFilename, const std::string& outputFilename = "output.obj");
  static void obj2pcd(const std::string& inputFilename, const std::string& outputFilename = "output.pcd");
};

#endif // CONVERTER_HPP
