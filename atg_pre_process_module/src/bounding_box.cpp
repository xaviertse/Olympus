#include "./include/atg_surface_identification/atg_surface_identification.hpp"
#include <iostream>

int main(int argc, char **argv)
{
  std::cout << "Running ATG bounding box calculation\n";

  //read filename from input variable through argv
  if (argc < 2){
    std::cout << "No input file variable found!\n";
    return 0;
  }
  else
  {
    std::string filename = std::string(argv[1]);
    std::cout << "Reading file: "<< filename<<"\n";
  }

  //start bounding box calculation
  ATG_surface_identification::ATG_SURF_ID app;
  app.bounding_box("file");

  std::cout << "Successful run of atg boundingbox!\n";
  return 0;
}
