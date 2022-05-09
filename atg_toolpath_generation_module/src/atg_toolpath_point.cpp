#include <include/atg_toolpath_generation/atg_toolpath_generation.hpp>
#include <iostream>

int main(int argc, char **argv)
{
  std::cout << "Running ATG Toolpath Point\n";

  //read filename from input variable through argv
  if (argc < 2){
    std::cout << "No input file variable found!\n";
    return 0;
  }
  //else //found file to read
  std::string filename = std::string(argv[1]);
  std::cout << "Reading file: "<< filename<<"\n";

  //start bounding box calculation
  ATG_toolpath_generation::ATG_TPG app;
  app.external_on_      =   atoi(argv[2]);
  app.internal_on_      =   atoi(argv[3]);
  app.resolution_       = strtof(argv[4],NULL);
  app.step_size_        = strtof(argv[5],NULL);
  app.offset_           = strtof(argv[6],NULL);
  app.path_rotate_      = strtof(argv[7],NULL);
  app.downsample_       =   atoi(argv[8]);
  app.ksearch_tp_       =   atoi(argv[9]);
  app.normal_flip_      =   atoi(argv[10]);
  app.section_range_min_= strtof(argv[11],NULL);
  app.section_range_max_= strtof(argv[12],NULL);
  app.reverse_toolpath_ =   atoi(argv[13]);
  app.forced_resolution_=   atoi(argv[14]);
  app.hole_patch_size_  = strtof(argv[15],NULL);
  app.lift_height_      = strtof(argv[16],NULL);
  app.ksearch_radius_   = strtof(argv[17],NULL); //point only
  app.ksearch_threshold_= strtof(argv[18],NULL); //point only
  app.Point(filename);

  std::cout << "Successful run of atg toolpath Point!\n";
  return 0;

}
