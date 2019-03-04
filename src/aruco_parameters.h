

#ifdef __CONSTFROMHFILE_

#define LEN_OF_MARKER         (0.07)            //in meters
#define USED_BOARD_TYPE       cv::aruco::DICT_6X6_50
#define LEN_OF_CUBE           (0.08)            //in meters
#define CUBE0_SIDES_QR_IDS    {99, 99, 99, 99, 99, 99}
#define CUBE1_SIDES_QR_IDS    {99, 99, 99, 99, 99, 99}
#define CUBE2_SIDES_QR_IDS    {7, 8, 20, 10, 12, 99} //face, left ,back, right, up
#define CUBE3_SIDES_QR_IDS    {99, 99, 99, 99, 99, 99}


#define LEN_OF_BUFFER	        100
#define NUMB_OF_STATIC_MARKERS	4
#define NUMB_CUBE_SIDES         6
//#define STATIC_MARKERS_IDS	    {5, 0 , 2,   1 , 3, 6}
#define STATIC_MARKERS_IDS	    {5,  0,  3, 6} 
#define STATIC_MARKERS_SIZES	{0.16, 0.16, 0.16, 0.16}//, 0.16





// #define CUBE4_SIDES_QR_IDS    {4, 99, 99, 99, 99, 99}
// #define CUBE5_SIDES_QR_IDS    {5, 99, 99, 99, 99, 99}
// #define CUBE6_SIDES_QR_IDS    {6, 99, 99, 99, 99, 99}
// #define CUBE7_SIDES_QR_IDS    {7, 99, 99, 99, 99, 99}

#endif