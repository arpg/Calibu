#define USAGE\
  "\nUsage: findtargets -cam <image source> [-cmod <camera file>]  [-o <output file>]\n"\
  "\n"\
  "Example image source: proto:///Users/joe_shmoe/Data/DataLog.log\n"\
  "Example camera file: cameras.xml\n"\
  "\n"\
  "This program will process images with visible AR tags and produce\n"\
  "a text file with the following format:\n"\
  "-----------------------------------------------------\n"\
  " m, the number of poses that see any landmarks\n"\
  " n, the number of unique landmarks seen\n"\
  " k, number of measurements\n"\
  " pose_id, landmark_id, u, v\n"\
  "   .\n"\
  "   .\n"\
  "   .\n"\
  " pose_id, landmark_id, u, v\n"\
  " pose_1\n"\
  " pose_2\n"\
  "   .\n"\
  "   .\n"\
  "   .\n"\
  " pose_m\n"\
  " landmark_1\n"\
  " landmark_2\n"\
  "   .\n"\
  "   .\n"\
  "   .\n"\
  " landmark_n\n"\
  "-----------------------------------------------------\n"\
  "This file is suitable as an initialization for ground-truth pose\n"\
  "estimation. NB:\n"\
  "  each pose is a 1x6 vector:  x,y,z,p,q,r\n"\
  "  each landmark is 1x4 vector:  id,x,y,z\n"\
  "\n\n"
