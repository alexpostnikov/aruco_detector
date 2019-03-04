#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/viz.hpp>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

Scalar colorTab[] =
    {
        Scalar(0, 0, 255),
        Scalar(0, 255, 0),
        Scalar(255, 100, 100),
        Scalar(255,0,255),
        Scalar(0,255,255)
    };

// static void help()
// {
//     cout << "\nThis program demonstrates kmeans clustering.\n"
//             "It generates an image with random points, then assigns a random number of cluster\n"
//             "centers and uses kmeans to move those cluster centers to their representitive location\n"
//             "Call\n"
//             "./kmeans\n" << endl;
// }

int main( int /*argc*/, char** /*argv*/ )
{
    std::vector<Point3f> points;
    ifstream infile("tvec_1.txt");
    
    double a,b,c;
    while (infile >> a >> b >> c)
    {
        points.push_back(Point3f(a,b,c));
    }
    cout << "points size" << points.size()<< endl;;
    
    Mat pointsMat(points.size(), 1, CV_32FC3), labels;
    Mat colorMat(points.size(), 1, CV_8UC4);
    
    cout <<" pointsMat.rows "<< pointsMat.rows << endl;
    for (int i=0 ; i < pointsMat.rows; i++)
    {
        pointsMat.at<Point3f>(i,0) = (points[i]);
    }
    
    // for (int clusterCount = 2; clusterCount< 20 ; clusterCount++)
    // {
        std::vector<Point3f> centers;
        int clusterCount = 6;
        //cout << pointsMat << endl;
        double compactness = kmeans(pointsMat, clusterCount, labels,
        TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1),
           3, KMEANS_PP_CENTERS, centers);
        cout <<"labels.size "<< labels.size() << endl;
        for (int i=0 ; i <pointsMat.rows; i++)
        {
            int clusterIdx = labels.at<int>(i);
            colorMat.row(i) = (Scalar(colorTab[clusterIdx][0], colorTab[clusterIdx][1], colorTab[clusterIdx][2], 50 ));
        }
        cout << endl;

        cv::viz::Viz3d viewer;
        viewer = cv::viz::Viz3d("Point Cloud");
        cv::viz::WCloud cloud (pointsMat, colorMat);
        viewer.showWidget( "Cloud", cloud );
        viewer.spinOnce(10000,true);
        cout << "compactness " << compactness << " clusterCount " << clusterCount << endl;

        int classes[clusterCount]; 
        int index = -1; 
        int max = -1;
        memset(classes, 0, sizeof(classes[0]) * clusterCount);
        int * labels_ptr = labels.ptr<int>(0);
        for (int i = 0; i < labels.rows; ++i)
            classes[*labels_ptr++]++;
        for (int i = 0; i < clusterCount; ++i)
            {
            if (classes[i] > max)
                {
                max = classes[i];
                index = i;
                }
            }

        cout <<  "bigest claster: "<< index << "numb of points in cluster " << max << endl;
        

        // int i;
        // cin >> i;
    
    waitKey(0);

}



//     const int MAX_CLUSTERS = 5;
//     Scalar colorTab[] =
//     {
//         Scalar(0, 0, 255),
//         Scalar(0,255,0),
//         Scalar(255,100,100),
//         Scalar(255,0,255),
//         Scalar(0,255,255)
//     };

//     Mat img(500, 500, CV_8UC3);
//     RNG rng(12345);

//     //for(;;)
    
//         int k, clusterCount = rng.uniform(2, MAX_CLUSTERS+1);
//         int i, sampleCount = rng.uniform(1, 1001);
//         Mat points(sampleCount, 1, CV_32FC2), labels;
    

//     cv::viz::Viz3d viewer;
//     viewer = cv::viz::Viz3d( "Point Cloud" );
//     clusterCount = 3;
//     sampleCount = 100;
//     Mat my_points(sampleCount, 1, CV_32FC3), colorMat(sampleCount, 1, CV_8UC4);
//     std::vector<Point3f> centers;
//     for( k = 0; k < clusterCount; k++ )
//         {
//             Point3f center;
//             center.x = rng.uniform(0, img.cols);
//             center.y = rng.uniform(0, img.rows);
//             center.z = rng.uniform(0, img.rows);
//             Mat pointChunk = my_points.rowRange(k*sampleCount/clusterCount,
//                                              k == clusterCount - 1 ? sampleCount :
//                                              (k+1)*sampleCount/clusterCount);
//             rng.fill(pointChunk, RNG::NORMAL, Scalar(center.x, center.y, center.z), Scalar(img.cols*0.05, img.rows*0.05,  img.rows*0.05));
//         }

//     randShuffle(my_points, 1, &rng);

//     double compactness = kmeans(my_points, clusterCount, labels,
//         TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0),
//            3, KMEANS_PP_CENTERS, centers);

//     img = Scalar::all(0);

//     for( i = 0; i < sampleCount; i++ )
//     {
//         int clusterIdx = labels.at<int>(i);
//         Point3f ipt = my_points.at<Point3f>(i);
//         circle( img, Point2f(ipt.x, ipt.y), 2, colorTab[clusterIdx], FILLED, LINE_AA );
//         colorMat.row(i) = (Scalar(colorTab[clusterIdx][0], colorTab[clusterIdx][1], colorTab[clusterIdx][2], 50 ));
//         //colorMat.row(i) = cv::viz::Color::maroon();//, colorTab[clusterIdx](2), 50 ));
//     }
//     for (i = 0; i < (int)centers.size(); ++i)
//     {
//         Point3f c = centers[i];
//         circle( img, Point2f(c.x,c.y), 40, colorTab[i], 1, LINE_AA );
//     }
//     cout << "Compactness: " << compactness << endl;

//     imshow("clusters", img);

//     waitKey(0);
//     cout <<" colorMat.size() "<<  colorMat.size()  << " colorMat.row(1) "<<  colorMat.row(1) <<endl;
//     cout <<" my_points.size() "<<  my_points.size()  << " my_points.row(1) "<<  my_points.row(1) <<endl;
//     cv::viz::WCloud cloud (my_points, colorMat);
//     viewer.showWidget( "Cloud", cloud );
//     viewer.spin();
//     waitKey(0);    
    

//     return 0;
// }
