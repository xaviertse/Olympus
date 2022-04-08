#include "./include/atg_surface_identification/atg_surface_identification.hpp"
#include <iostream>
#include <stdlib.h>

int main(int argc, char **argv)
{
  std::cout << "Running ATG segmentation\n";

  //read filename from input variable through argv
  if (argc < 7){//expecting 6 inputs
    std::cout << "No valid input variables found!\n";
    return 0;
  }
  //else //found file to read
  std::string filename = std::string(argv[1]);
  std::cout << "Reading file: "<< filename<<"\n";



  //start segmentation seq
  ATG_surface_identification::ATG_SURF_ID app;
  app.param_smoothness     /*= 4.0;   */=strtod(argv[2],NULL);
  app.param_curvature      /*= 1.0;   */=strtod(argv[3],NULL);
  app.param_minClusteSize  /*= 100;   */=strtod(argv[4],NULL);
  app.param_KSearch        /*= 7.0;   */=strtod(argv[5],NULL);
  app.param_neighbour      /*= 50.0;  */=strtod(argv[6],NULL);

  std::cout << "Reading param_smoothness:    "<< app.param_smoothness   <<"\n";
  std::cout << "Reading param_curvature:     "<< app.param_curvature    <<"\n";
  std::cout << "Reading param_minClusteSize: "<< app.param_minClusteSize<<"\n";
  std::cout << "Reading param_KSearch:       "<< app.param_KSearch      <<"\n";
  std::cout << "Reading param_neighbour:     "<< app.param_neighbour    <<"\n";
  app.segmentation(filename);


  std::cout << "Successful run of atg segmentation!\n";
  return 0;
}
