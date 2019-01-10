#include <iostream>
using namespace std;

#include "OdometryExtractor.hpp"

void usage()
{
    cout << "Bad arguments" << endl;
    throw invalid_argument("Bad argument");
}

int main(int argc, char** argv)
{
    if(argc < 6)
        usage();

    std::string basePath(argv[1]);
    std::string outputPath(argv[2]);
    std::string inputTopic(argv[3]);
    std::string outputDeltaTopic(argv[4]);
    std::string outputAttitudeTopic(argv[5]);

    //Infuse::OdometryExtractor generator("/home/pnarvor/work/infuse/data/log_data_acquisition_2018_10_25_16_37_06/my_bag", "/rmp400/PoseInfuse", "/rmp400/Delta", "/rmp400/Attitude");
    Infuse::OdometryExtractor generator(basePath, inputTopic, outputDeltaTopic, outputAttitudeTopic, outputPath);
    generator.extract();

    return 0;
}

//#include <rosbag/bag.h>
//#include <rosbag/view.h>
//#include <infuse_msgs/asn1_bitstream.h>
//#include <infuse_asn1_types/TransformWithCovariance.h>
//#include <infuse_asn1_conversions/asn1_pom_conversions.hpp>
//
//int main(int argc, char** argv)
//{
//    if(argc < 2)
//    {
//        cout << "No file, aborting" << endl;
//        return -1;
//    }
//
//    rosbag::Bag bagIn;
//    bagIn.open(argv[1], rosbag::bagmode::Read);
//    rosbag::Bag bagOut;
//    bagOut.open("out.bag", rosbag::bagmode::Write);
//
//    rosbag::View view(bagIn);
//    std::vector<const rosbag::ConnectionInfo *> connection_infos = view.getConnections();
//    std::vector<std::string> topics;
//
//    cout << "Listing topics... ";
//    for(auto it = connection_infos.begin(); it != connection_infos.end(); it++)
//        topics.push_back((*it)->topic);
//    cout << "Done !" << endl;
//
//    int count;
//    //rosbag::View viewTopics;
//    for(auto topic = topics.begin(); topic != topics.end(); topic++)
//    {
//        cout << "Writing topic " << *topic << "... ";
//        rosbag::View viewTopics(bagIn, rosbag::TopicQuery(*topic));
//        count = 0;
//        infuse_msgs::asn1_bitstream::ConstPtr m;
//        for(rosbag::View::iterator it = viewTopics.begin(); it != viewTopics.end(); it++)
//        {
//            m = it->instantiate<infuse_msgs::asn1_bitstream>();
//            if(!m)
//                continue;
//            //cout << m->header.stamp << endl;
//            bagOut.write(*topic, m->header.stamp, *m);
//            count++;
//        }
//
//        cout << "Done ! " << count << " messages written." << endl;
//    }
//
//    bagOut.close();
//    bagIn.close();
//
//    //getchar();
//
//    return 0;
//}


