#include <iostream>
#include "opencv2/surface_matching.hpp"
#include "opencv2/surface_matching/ppf_helpers.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/opencv.hpp"
#include <fstream>

using namespace std;
using namespace cv;
using namespace ppf_match_3d;

const char *DETECTOR_FILENAME = "detector.xml";

static void help(const string &errorMessage)
{
  cout << "Program init error : " << errorMessage << endl;
  cout << "\nUsage : ppf_matching [input model file] [input scene file]" << endl;
  cout << "\nPlease start again with new parameters" << endl;
}

int main(int argc, char **argv)
{
  cout << "****************************************************" << endl;
  cout << "* Surface Matching demonstration: demonstrates the use of surface matching"
          " using point pair features."
       << endl;
  cout << "* The sample loads a model and a scene, where the model lies in a different"
          " pose than the training.\n"
          "* It then trains, serializes and deserializes the model, and searches for it in the input scene.\n"
          "* The detected poses are further refined by ICP and printed to the standard output."
       << endl;
  cout << "****************************************************" << endl;

  if (argc < 3)
  {
    help("Not enough input arguments");
    exit(1);
  }

#if (defined __x86_64__ || defined _M_X64)
  cout << "Running on 64 bits" << endl;
#else
  cout << "Running on 32 bits" << endl;
#endif

#ifdef _OPENMP
  cout << "Running with OpenMP" << endl;
#else
  cout << "Running without OpenMP and without TBB" << endl;
#endif

  string modelFileName = (string)argv[1];
  string sceneFileName = (string)argv[2];

  Mat pc = loadPLYSimple(modelFileName.c_str(), 0);

  //compute normals for the model of object
  Mat pcNormals;
  Vec3d viewpoint(0, 0, 0);
  computeNormalsPC3d(pc, pcNormals, 6, false, viewpoint);

  int64 tick1, tick2;

  ifstream detectorFile(DETECTOR_FILENAME);

  if (!detectorFile.good())
  {
    {
      // Train the model

      cout << "Training..." << endl;
      tick1 = cv::getTickCount();
      ppf_match_3d::PPF3DDetector detector(0.03, 0.03);
      detector.trainModel(pcNormals);
      tick2 = cv::getTickCount();

      cout << "Training complete in "
           << (double)(tick2 - tick1) / cv::getTickFrequency()
           << " sec" << endl;

      // Serialize the model

      cout << "Serializing..." << endl;

      tick1 = cv::getTickCount();
      FileStorage fsOut(DETECTOR_FILENAME, FileStorage::WRITE);
      detector.write(fsOut);
      fsOut.release();
      tick2 = cv::getTickCount();

      cout << "Serialization complete in "
           << (double)(tick2 - tick1) / cv::getTickFrequency()
           << " sec" << endl;
    }
  }
  else
  {
    cout << "Found detector file: Skipping training phase" << endl;
  }

  detectorFile.close();

  // Read the serialized model

  ppf_match_3d::PPF3DDetector detectorDes;

  cout << "Deserializing..." << endl;

  tick1 = cv::getTickCount();
  FileStorage fsLoad(DETECTOR_FILENAME, FileStorage::READ);
  detectorDes.read(fsLoad.root());
  fsLoad.release();
  tick2 = cv::getTickCount();

  cout << "Deserialization complete in "
       << (double)(tick2 - tick1) / cv::getTickFrequency()
       << " sec" << endl;

  // Read the scene
  Mat pointsAndNormals;
  Mat pcTest = loadTXTSimple(sceneFileName.c_str());
  //Mat pcTest = loadPLYSimple(sceneFileName.c_str(), 0);
  // compute the normals for scene points
  //Vec3d viewpoint(0, 0, 0);
  computeNormalsPC3d(pcTest, pointsAndNormals, 6, false, viewpoint);

  // Match the model to the scene and get the pose

  cout << "Starting matching using the deserialized model..." << endl;

  vector<Pose3DPtr> results;
  tick1 = cv::getTickCount();
  detectorDes.match(pointsAndNormals, results, 1.0 / 10.0, 0.03);
  tick2 = cv::getTickCount();

  cout << "PPF Elapsed Time " << (tick2 - tick1) / cv::getTickFrequency()
       << " sec" << endl;

  // Check results size from match call above

  size_t results_size = results.size();
  cout << "Number of matching poses: " << results_size << endl;

  if (results_size == 0)
  {
    cout << "No matching poses found. Exiting." << endl;
    exit(0);
  }

  // Get only first N results - but adjust to results size if num of results are less than that specified by N

  size_t N = 2;

  if (results_size < N)
  {
    cout << "Reducing matching poses to be reported (as specified in code): "
         << N << " to the number of matches found: " << results_size << endl;
    N = results_size;
  }

  vector<Pose3DPtr> resultsSub(results.begin(), results.begin() + N);

  // Create an instance of ICP

  ICP icp(100, 0.005f, 2.5f, 8);
  int64 t1 = cv::getTickCount();

  // Register for all selected poses

  cout << "Performing ICP on " << N << " poses..." << endl;

  icp.registerModelToScene(pcNormals, pointsAndNormals, resultsSub);
  int64 t2 = cv::getTickCount();

  cout << "ICP Elapsed Time " << (t2 - t1) / cv::getTickFrequency() << " sec" << endl;

  cout << "Poses: " << endl;

  // Debug first five poses

  for (size_t i = 0; i < resultsSub.size(); i++)
  {
    Pose3DPtr result = resultsSub[i];
    cout << endl
         << "Pose Result " << i << endl;
    result->printPose();

    if (i == 0)
    {
      Mat pct = transformPCPose(pcNormals, result->pose);
      writePLY(pct, "para6700PCTrans.ply");
    }
  }

  return 0;
}
