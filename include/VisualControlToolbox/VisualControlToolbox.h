/*
 *      VISUAL CONTROL TOOLBOX
 *
 *      DEVELOPER:  Edgar I. Chávez-Aparicio
 *      MAIL:       edgar.chavez@cimat.mx
 *
 *      DESCRIPTION:
 *          This toolbox is a Header-only library developed for ROS1. It has a
 *          set of simple structures an method used in the research of
 *          image-based control in the CIMAT Robotics and Intelligent Systems
 *          group.
 *      DEPENDENCIES:
 *          +   opencv2
 */

#ifndef VCT_H
#define VCT_H
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/softfloat.hpp>
#include <opencv2/aruco.hpp>
#include <Eigen/Dense>




namespace vct
{
////////////////////////////////////////////////////////////////////////////
//  PARAMETERS (Struct)
//      Abstraction of image information
//      Data:
//          +   Orb detection parameters
//          +   Point matching parameters
//          +   Camera intrinsics
////////////////////////////////////////////////////////////////////////////
typedef struct parameters {

    // Image proessing parameters
    float feature_threshold=0.5;
    int nfeatures=250;
    float scaleFactor=1.2;
    int nlevels=8;
    int edgeThreshold=15; // Changed default (31);
    int firstLevel=0;
    int WTA_K=2;
    cv::ORB::ScoreType scoreType=cv::ORB::HARRIS_SCORE;
    int patchSize=30;
    int fastThreshold=20;
    float flann_ratio=0.7;
    cv::Ptr<cv::ORB> orb;

    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    // Camera parameters
    cv::Mat K;

    void load(const ros::NodeHandle &nh)
    {
        // Load intrinsic parameters
        XmlRpc::XmlRpcValue kConfig;
        K = cv::Mat(3,3, CV_64F, double(0));
        if (nh.hasParam("camera_intrinsic_parameters"))
        {
            nh.getParam("camera_intrinsic_parameters", kConfig);
            if (kConfig.getType() == XmlRpc::XmlRpcValue::TypeArray)
            for (int i=0;i<9;i++)
            {
                std::ostringstream ostr;
                ostr << kConfig[i];
                std::istringstream istr(ostr.str());
                istr >> K.at<double>(i/3,i%3);
            }
        }


    }
    void initOrb(const ros::NodeHandle &nh)
    {
        // Load error threshold parameter
        feature_threshold=nh.param(std::string("feature_error_threshold"),
                                   std::numeric_limits<double>::max());
        // Load feature detector parameters
        nfeatures=nh.param(std::string("nfeatures"),100);
        scaleFactor=nh.param(std::string("scaleFactor"),1.2);
        nlevels=nh.param(std::string("nlevels"),5);
        edgeThreshold=nh.param(std::string("edgeThreshold"),15);
        patchSize=nh.param(std::string("patchSize"),30);
        fastThreshold=nh.param(std::string("fastThreshold"),20);
        flann_ratio=nh.param(std::string("flann_ratio"),0.7);
        if (scaleFactor <= 1.)
        {
            scaleFactor = 1.2;
            ROS_INFO("Params scale factor over ride at 1.2 to avoid orb bug ");
        }

        orb = cv::ORB::create(
        nfeatures,
        scaleFactor,
        nlevels,
        edgeThreshold,
        firstLevel,
        WTA_K,
        scoreType,
        patchSize,
        fastThreshold);
    }
    void initArucos(const ros::NodeHandle &nh)
    {
        //  Dictionary selection

        int dict=cv::aruco::DICT_6X6_250;
        if(nh.hasParam("dictionary"))
        {
            std::string dict_str;
            nh.getParam("dictionary", dict_str);
            if(dict_str == std::string("4X4_50"))
                dict = cv::aruco::DICT_4X4_50;
            else if(dict_str == std::string("4X4_100"))
                dict = cv::aruco::DICT_4X4_100 ;
            else if(dict_str == std::string("4X4_250"))
                dict = cv::aruco::DICT_4X4_250 ;
            else if(dict_str == std::string("4X4_1000"))
                dict = cv::aruco::DICT_4X4_1000 ;
            else if(dict_str == std::string("5X5_50"))
                dict = cv::aruco::DICT_5X5_50 ;
            else if(dict_str == std::string("5X5_100"))
                dict = cv::aruco::DICT_5X5_100 ;
            else if(dict_str == std::string("5X5_250"))
                dict = cv::aruco::DICT_5X5_250 ;
            else if(dict_str == std::string("5X5_1000"))
                dict = cv::aruco::DICT_5X5_1000 ;
            else if(dict_str == std::string("6X6_50"))
                dict = cv::aruco::DICT_6X6_50 ;
            else if(dict_str == std::string("6X6_100"))
                dict = cv::aruco::DICT_6X6_100 ;
            else if(dict_str == std::string("6X6_250"))
                dict = cv::aruco::DICT_6X6_250 ;
            else if(dict_str == std::string("6X6_1000"))
                dict = cv::aruco::DICT_6X6_1000 ;
            else if(dict_str == std::string("7X7_50"))
                dict = cv::aruco::DICT_7X7_50 ;
            else if(dict_str == std::string("7X7_100"))
                dict = cv::aruco::DICT_7X7_100 ;
            else if(dict_str == std::string("7X7_250"))
                dict = cv::aruco::DICT_7X7_250 ;
            else if(dict_str == std::string("7X7_1000"))
                dict = cv::aruco::DICT_7X7_1000 ;
            else if(dict_str == std::string("ARUCO_ORIGINAL"))
                dict = cv::aruco::DICT_ARUCO_ORIGINAL ;
            else if(dict_str == std::string("APRILTAG_16h5"))
                dict = cv::aruco::DICT_APRILTAG_16h5 ;
            else if(dict_str == std::string("APRILTAG_25h9"))
                dict = cv::aruco::DICT_APRILTAG_25h9 ;
            else if(dict_str == std::string("APRILTAG_36h10"))
                dict = cv::aruco::DICT_APRILTAG_36h10 ;
            else if(dict_str == std::string("APRILTAG_36h11"))
                dict = cv::aruco::DICT_APRILTAG_36h11 ;
            else
                ROS_WARN("[VC Tools] Incorrect ArUco Dictionary Selection");
        }
        dictionary = cv::aruco::getPredefinedDictionary(dict);
        //  Detection parameters
        parameters = cv::aruco::DetectorParameters::create();
        if(nh.hasParam("adaptiveThreshWinSizeMin"))
            nh.getParam("adaptiveThreshWinSizeMin", parameters->adaptiveThreshWinSizeMin);
        if(nh.hasParam("adaptiveThreshWinSizeMax"))
            nh.getParam("adaptiveThreshWinSizeMax", parameters->adaptiveThreshWinSizeMax);
        if(nh.hasParam("adaptiveThreshWinSizeStep"))
            nh.getParam("adaptiveThreshWinSizeStep", parameters->adaptiveThreshWinSizeStep);
        if(nh.hasParam("adaptiveThreshConstant"))
            nh.getParam("adaptiveThreshConstant", parameters->adaptiveThreshConstant);
        if(nh.hasParam("minMarkerPerimeterRate"))
            nh.getParam("minMarkerPerimeterRate", parameters->minMarkerPerimeterRate);
        if(nh.hasParam("maxMarkerPerimeterRate"))
            nh.getParam("maxMarkerPerimeterRate", parameters->maxMarkerPerimeterRate);
        if(nh.hasParam("polygonalApproxAccuracyRate"))
            nh.getParam("polygonalApproxAccuracyRate", parameters->polygonalApproxAccuracyRate);
        if(nh.hasParam("minCornerDistanceRate"))
            nh.getParam("minCornerDistanceRate", parameters->minCornerDistanceRate);
        if(nh.hasParam("minDistanceToBorder"))
            nh.getParam("minDistanceToBorder", parameters->minDistanceToBorder);
        if(nh.hasParam("minMarkerDistanceRate"))
            nh.getParam("minMarkerDistanceRate", parameters->minMarkerDistanceRate);
        if(nh.hasParam("cornerRefinementMethod"))
        {
            std::string method;
            nh.getParam("cornerRefinementMethod", method);
            if (method == std::string("CORNER_REFINE_NONE"))
                parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
            else if (method == std::string("CORNER_REFINE_SUBPIX"))
                parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
            else if (method == std::string("CORNER_REFINE_CONTOUR"))
                parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
            else if (method == std::string("CORNER_REFINE_APRILTAG"))
                parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
            ROS_INFO("[VS Tools] Corner refinement method changed: %s", method.c_str());

        }
        if(nh.hasParam("cornerRefinementWinSize"))
            nh.getParam("cornerRefinementWinSize", parameters->cornerRefinementWinSize);
        if(nh.hasParam("cornerRefinementMaxIterations"))
            nh.getParam("cornerRefinementMaxIterations", parameters->cornerRefinementMaxIterations);
        if(nh.hasParam("cornerRefinementMinAccuracy"))
            nh.getParam("cornerRefinementMinAccuracy", parameters->cornerRefinementMinAccuracy);
        if(nh.hasParam("markerBorderBits"))
            nh.getParam("markerBorderBits", parameters->markerBorderBits);
        if(nh.hasParam("perspectiveRemovePixelPerCell"))
            nh.getParam("perspectiveRemovePixelPerCell", parameters->perspectiveRemovePixelPerCell);
        if(nh.hasParam("perspectiveRemoveIgnoredMarginPerCell"))
            nh.getParam("perspectiveRemoveIgnoredMarginPerCell", parameters->perspectiveRemoveIgnoredMarginPerCell);
        if(nh.hasParam("maxErroneousBitsInBorderRate"))
            nh.getParam("maxErroneousBitsInBorderRate", parameters->maxErroneousBitsInBorderRate);
        if(nh.hasParam("minOtsuStdDev"))
            nh.getParam("minOtsuStdDev", parameters->minOtsuStdDev);
        if(nh.hasParam("errorCorrectionRate"))
            nh.getParam("errorCorrectionRate", parameters->errorCorrectionRate);
        if(nh.hasParam("aprilTagMinClusterPixels"))
            nh.getParam("aprilTagMinClusterPixels", parameters->aprilTagMinClusterPixels);
        if(nh.hasParam("aprilTagMaxNmaxima"))
            nh.getParam("aprilTagMaxNmaxima", parameters->aprilTagMaxNmaxima);
        if(nh.hasParam("aprilTagCriticalRad"))
            nh.getParam("aprilTagCriticalRad", parameters->aprilTagCriticalRad);
        if(nh.hasParam("aprilTagMaxLineFitMse"))
            nh.getParam("aprilTagMaxLineFitMse", parameters->aprilTagMaxLineFitMse);
        if(nh.hasParam("aprilTagMinWhiteBlackDiff"))
            nh.getParam("aprilTagMinWhiteBlackDiff", parameters->aprilTagMinWhiteBlackDiff);
        if(nh.hasParam("aprilTagDeglitch"))
            nh.getParam("aprilTagDeglitch", parameters->aprilTagDeglitch);
        if(nh.hasParam("aprilTagQuadDecimate"))
            nh.getParam("aprilTagQuadDecimate", parameters->aprilTagQuadDecimate);
        if(nh.hasParam("aprilTagQuadSigma"))
            nh.getParam("aprilTagQuadSigma", parameters->aprilTagQuadSigma);
        if(nh.hasParam("detectInvertedMarker"))
            nh.getParam("detectInvertedMarker", parameters->detectInvertedMarker);

        ROS_INFO("[VC Toolbox] Param Win Min = %d", parameters->adaptiveThreshWinSizeMin);
        ROS_INFO("[VC Toolbox] Param Win Max = %d", parameters->adaptiveThreshWinSizeMax);
    }

} parameters;

void camera_norm(const parameters & params, cv::Mat & P);

//  Estructura de datos referente a los resultados
//      de los emparejamientos de puntos y la homografía
typedef struct matching_result{
    cv::Mat H;      //  Homografía
    cv::Mat img_matches;    // Imagen para salida de matches
    cv::Mat p1;     // puntos de imagen (u,v) Referencia
    cv::Mat p2;     // puntos de imagen (u.v) Actual
    double mean_feature_error=1e10;

    void camera_norm(const parameters & params)
    {
        vct::camera_norm(params, p1);
        vct::camera_norm(params, p2);
    }

} matching_result;

//  Estructura que contiene la información de
//      la pose referencia ()
typedef struct desired_configuration {
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> kp;
    cv::Mat img;
    std::vector<std::vector<cv::Point2f>> arucos;
    std::vector<int> arucos_ids;
} desired_configuration;


////////////////////////////////////////////////////////////////////////////
//  VARIOUS METHODS
////////////////////////////////////////////////////////////////////////////



//  Selecciona una pose a partir de poses previamente calculadas
//  INPUT:
//      Rs = Rotaciones candidatas
//      Ts = Traslaciones candidatas
//      Ns = Normales al plano obsevado candidatas
//      result = resultados
//      selected = bandera: indica si se ha seleccionado un candidato
//                          previa al cálculo
//  OUTPUT:
//      Rbest = mejor rotación
//      tbest = mejor traslación
int select_decomposition(
    const std::vector<cv::Mat> & Rs,
    const std::vector<cv::Mat> &Ts,
    const std::vector<cv::Mat> &Ns,
    const matching_result& result,
    bool & selected,
    cv::Mat & Rbest,
    cv::Mat & tbest)
{
        if(selected == false) {
        // To store the matrix Rotation and translation that best fix
        // Constructing the extrinsic parameters matrix for the actual image
        cv::Mat P2 = cv::Mat::eye(3, 4, CV_64F);

        double th = 0.1, nz = 1.0; //max value for z in the normal plane
        // Preparing the points for the test
        std::vector<cv::Point2f> pp1;
        std::vector<cv::Point2f> pp2;
        pp1.push_back(cv::Point2f(result.p1.at<float>(0,0),result.p1.at<float>(0,1)));
        pp2.push_back(cv::Point2f(result.p2.at<float>(0,0),result.p2.at<float>(0,1)));

        // For every rotation matrix
        for(int i=0;i<Rs.size();i++){
            // Constructing the extrinsic parameters matrix for the desired image
            cv::Mat P1;
            cv::hconcat(Rs[i],Ts[i],P1);

            // To store the result
            cv::Mat p3D;
            //obtaining 3D point
            cv::triangulatePoints(P1,P2,pp1,pp2,p3D);

            // Transforming to homogeneus
            cv::Mat point(4,1,CV_64F);
            point.at<double>(0,0) = p3D.at<float>(0,0) /p3D.at<float>(3,0);
            point.at<double>(1,0) = p3D.at<float>(1,0) /p3D.at<float>(3,0);
            point.at<double>(2,0) = p3D.at<float>(2,0) /p3D.at<float>(3,0);
            point.at<double>(3,0) = p3D.at<float>(3,0) /p3D.at<float>(3,0);
            // Verify if the point is in front of the camera.
            //      Also if is similar to [0 0 1] o [0 0 -1]
            // Giving preference to the first
            if(point.at<double>(2,0) >= 0.0 &&
                fabs(fabs(Ns[i].at<double>(2,0))-1.0) < th ){
                if(nz > 0){
                    Rs[i].copyTo(Rbest);
                    Ts[i].copyTo(tbest);
                    nz = Ns[i].at<double>(2,0);
                    selected = true;
                }
            }
        }
        // Process again, it is probably only in z axiw rotation, and we want the one with the highest nz component

        if (selected == false){
            double max = -1;
            for(int i=0;i<Rs.size();i++){
                // Constructing the extrinsic parameters matrix for the desired image
                cv::Mat P1;
                cv::hconcat(Rs[i],Ts[i],P1);
                //to store the result
                cv::Mat p3D;
                //obtaining 3D point
                cv::triangulatePoints(P1,P2,pp1,pp2,p3D);
                // Transforming to homogeneus
                cv::Mat point(4,1,CV_64F);
                point.at<double>(0,0) = p3D.at<float>(0,0) /p3D.at<float>(3,0);
                point.at<double>(1,0) = p3D.at<float>(1,0) /p3D.at<float>(3,0);
                point.at<double>(2,0) = p3D.at<float>(2,0) /p3D.at<float>(3,0);
                point.at<double>(3,0) = p3D.at<float>(3,0) /p3D.at<float>(3,0);

                if(point.at<double>(2,0) >= 0.0 && fabs(Ns[i].at<double>(2,0)) > max){
                    Rs[i].copyTo(Rbest);
                    Ts[i].copyTo(tbest);
                    max = fabs(Ns[i].at<double>(2,0));
                    selected = true;
                }
            }
        }
        //if not of them has been selected
        //now, we are not going to do everything again
    } else {//if we already selected one, select the closest to that one

        double min_t = 1e8, min_r = 1e8;
        cv::Mat t_best_for_now, r_best_for_now;
        //choose the closest to the previous one
        for(int i=0;i<Rs.size();i++){
            double norm_diff_rot = cv::norm(Rs[i],Rbest);
            double norm_diff_t = cv::norm(Ts[i],tbest);
            if(norm_diff_rot < min_r){ Rs[i].copyTo(r_best_for_now); min_r=norm_diff_rot; }
            if(norm_diff_t < min_t){ Ts[i].copyTo(t_best_for_now); min_t=norm_diff_t; }
        }

        //save the best but dont modify it yet
        r_best_for_now.copyTo(Rbest);
        t_best_for_now.copyTo(tbest);
    }
    return 0;
};


//  Search of vectors
int find(int q, std::vector<int> vec)
{
    int len =vec.size();
    for (int i=0; i < len; i++)
        if (q == vec[i])
            return i;
    return -1;
}

//  Calcula las correspondencias de ArUcos entre dos imágenes
//  INPUT:
//      img = imagen ingresada
//      params = parámetros de la cámara
//      desired_configuration = información de referencia
//  OUTPUT:
//      result = resultados
int compute_arucos(
    const cv::Mat&img,
    const parameters & params,
    const desired_configuration & Desired_Configuration,
    matching_result& result)
{
    //  Detect arucos
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > _rejected;
    cv::aruco::detectMarkers(img,
                             params.dictionary,
                             corners,
                             ids,
                             params.parameters,
                             _rejected);

    //  Save and plot
    result.p1.release();
    result.p2.release();
    img.copyTo(result.img_matches);
    int len = Desired_Configuration.arucos_ids.size();

    if (ids.size() < 1)
        return -1;

    for (int i = 0; i < len; i++)
    {
        int j = find(Desired_Configuration.arucos_ids[i], ids);
        if (j >= 0 )
        {
            //  Save to result
            result.p1.push_back(Desired_Configuration.arucos[i]);
            result.p2.push_back(corners[j]);
        }
    }
    if (result.p1.empty())
        return -1;
    cv::Mat tmp;
    tmp = result.p1.reshape(1);
    tmp.convertTo(result.p1,CV_64F);
    tmp = result.p2.reshape(1);
    tmp.convertTo(result.p2,CV_64F);

    /******    computing error to stop ********/
    result.mean_feature_error = norm(result.p1, result.p2)/((double)result.p1.rows);

    // ROS_INFO("[VC Tools] |Error| = %e", result.mean_feature_error);
    // Draw reference
    cv::aruco::drawDetectedMarkers(result.img_matches,
                                    Desired_Configuration.arucos,
                                    Desired_Configuration.arucos_ids,
                                    cv::Scalar(0,0,255));
    //  draw current
    cv::aruco::drawDetectedMarkers(result.img_matches,
                                    corners,
                                    ids,cv::Scalar(0,255,0));
    return 0;
}
//  Calcula los emparejamientos entre dos imágenes
//  INPUT:
//      img = imagen ingresada
//      params = parámetros de la cámara
//      desired_configuration = información de referencia
//  OUTPUT:
//      result = resultados
int compute_descriptors(
    const cv::Mat&img,
    const parameters & params,
    const desired_configuration & Desired_Configuration,
    matching_result& result)
{
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> kp;

    //  ORB detector
    params.orb->detect(img, kp);
    if (kp.size()==0)
        return -1;
    params.orb->compute(img, kp, descriptors);


    //  FLANN matcher
    cv::FlannBasedMatcher matcher(new cv::flann::LshIndexParams(20, 10, 2));
    std::vector<std::vector<cv::DMatch>> matches;
    matcher.knnMatch(
        Desired_Configuration.descriptors,
        descriptors,
        matches,2);

    //  Filter good matches
    std::vector<cv::DMatch> goodMatches;
    for(int i = 0; i < matches.size(); ++i) {
        if (matches[i][0].distance < matches[i][1].distance * params.flann_ratio)
                goodMatches.push_back(matches[i][0]);
        }
    if (goodMatches.size()==0)
        return -1;

    //  Convert
    result.p1.release();
    result.p2.release();
    result.p1 = cv::Mat(goodMatches.size(),2,CV_64F);
    result.p2 = cv::Mat(goodMatches.size(),2,CV_64F);

    for(int i = 0; i < goodMatches.size(); i++){

        int idx = goodMatches[i].queryIdx;

        cv::Mat tmp = cv::Mat(Desired_Configuration.kp[idx].pt).t();
        tmp.copyTo(result.p1.row(i));
        tmp.release();
        idx = goodMatches[i].trainIdx;
        tmp = cv::Mat(kp[idx].pt).t();
        tmp.copyTo(result.p2.row(i));
    }

    //  ERROR
    cv::Mat a = cv::Mat(result.p1);
    cv::Mat b = cv::Mat(result.p2);
    result.mean_feature_error = norm(a,b)/((double)result.p1.rows);

    //  DRAW

    result.img_matches = cv::Mat::zeros(img.rows, img.cols * 2, img.type());
    cv::drawMatches(
        Desired_Configuration.img,
        Desired_Configuration.kp,
        img, kp, goodMatches,
        result.img_matches,
        cv::Scalar::all(-1),
        cv::Scalar::all(-1),
        std::vector<char>(),
        cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    return 0;
};

//  Calcula los ángulos de Euler
//      a partir de una matriz de rotación
cv::Vec3f rotationMatrixToEulerAngles(const cv::Mat &R)
{

    //  sy = sqrt(R[0,0]**2 +R[1,0]**2)
    float sy = R.at<double>(0,0);
    sy *= R.at<double>(0,0);
    sy +=  R.at<double>(1,0) * R.at<double>(1,0) ;
    sy = sqrt(sy);

    float x, y, z;
    //  If not singular
    if (sy >= 1e-6){
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }else{
        //  IF Singular
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);
};

//  Aplica la normalización de los puntos de imagen in situ
//      sobre un objeto de sesultado
//      TODO: hacer un metodo propio de vc_matching_result

// void camera_norm(const parameters & params,
//                        matching_result& result){
//         //  Normalización in situ
//
//     //  p1
//     result.p1.col(0) = result.p1.col(0)-params.K.at<double>(0,2);
//     result.p1.col(1) = result.p1.col(1)-params.K.at<double>(1,2);
//     result.p1.col(0) = result.p1.col(0).mul(1.0/params.K.at<double>(0,0));
//     result.p1.col(1) = result.p1.col(1).mul(1.0/params.K.at<double>(1,1));
//
//     //  p2
//     result.p2.col(0) = result.p2.col(0)-params.K.at<double>(0,2);
//     result.p2.col(1) = result.p2.col(1)-params.K.at<double>(1,2);
//     result.p2.col(0) = result.p2.col(0).mul(1.0/params.K.at<double>(0,0));
//     result.p2.col(1) = result.p2.col(1).mul(1.0/params.K.at<double>(1,1));
//
//
//     return;
// }
void camera_norm(const parameters & params,
                       cv::Mat & P){
        //  Normalización in situ

    cv::Mat _K;
    params.K.convertTo(_K, P.type());

    // std::cout << _K << std::endl << std::flush;
    // std::cout << _K.type() << std::endl << std::flush;
    // std::cout << _K.rows << std::endl << std::flush;
    // std::cout << _K.cols << std::endl << std::flush;
    // std::cout << params.K << std::endl << std::flush;
    std::cout << params.K.type() << std::endl << std::flush;
    std::cout << params.K.rows << std::endl << std::flush;
    std::cout << params.K.cols << std::endl << std::flush;
    std::cout << P << std::endl << std::flush;
    std::cout << P.type() << std::endl << std::flush;
    std::cout << P.rows << std::endl << std::flush;
    std::cout << P.cols << std::endl << std::flush;
    //  p1
    // P.col(0) = P.col(0)-_K.at<double>(0,2);
    // P.col(1) = P.col(1)-_K.at<double>(1,2);
    // P.col(0) = P.col(0).mul(1.0/_K.at<double>(0,0));
    // P.col(1) = P.col(1).mul(1.0/_K.at<double>(1,1));
    P.col(0) = P.col(0)-params.K.at<double>(0,2);
    P.col(1) = P.col(1)-params.K.at<double>(1,2);
    P.col(0) = P.col(0).mul(1.0/params.K.at<double>(0,0));
    P.col(1) = P.col(1).mul(1.0/params.K.at<double>(1,1));

    return;
}


//  Calcula la matriz de interacción para los puntos actuales
//      de un objeto de resultado.
cv::Mat interaction_Mat(cv::Mat & p, cv::Mat & Z)
{
    int n = p.rows;
    int type = p.type();

    if (n != Z.rows)
    {
        std::cout << "Z vector mismatches dimetions \n" <<  std::flush;
        return cv::Mat();
    }

    //     cv::Mat L= cv::Mat::zeros(n,12,CV_64F) ;
    cv::Mat L= cv::Mat::zeros(n,12,type) ;

    //  Calculos
    //   -1/Z
    //     L.col(0) = - cv::Mat::ones(n,1,CV_64F)/Z;
    L.col(0) = - cv::Mat::ones(n,1,type)/Z;
    //     L.col(1) = 0
    //  p[0,:]/Z
    L.col(2) = p.col(0)/Z;
    //  p[0,:] * p[1,:]
    L.col(3) = p.col(0).mul(p.col(1)) ;
    //  -(1 + p[0,:]^2)
    L.col(4) = -1. * (cv::Mat::ones(n,1,type)- p.col(0).mul(p.col(0))) ;
    //  p[1,:]
    p.col(1).copyTo(L.col(5));
    //     L.col(6) = 0
    //  -1/Z
    L.col(0).copyTo(L.col(7));
    //  p[1,:]/Z
    L.col(8) = p.col(1)/Z;
    //  1 + p[1,:]^2
    L.col(9) = cv::Mat::ones(n,1,type) + p.col(1).mul(p.col(1)) ;
    //  p[0,:] * p[1,:]
    L.col(10) = -1.* p.col(0).mul(p.col(1)) ;
    //  -p[0,:]
    L.col(11) = -1.0 * p.col(0);
    //  END 6 DOF
    return L.reshape(1,2*n);
};

//  Calcula la Pseudo Inversa Moore Penrose de L
//      Calcula el determinante en el proceso
cv::Mat Moore_Penrose_PInv(cv::Mat L,double & det)
{
    cv::Mat Lt = L.t();
    cv::Mat Ls = Lt*L;
    det = cv::determinant(Ls);
    return Ls.inv()*Lt;
};

//  Calcula la Homografía entre
//      la cofiguración deseada y la imagen ingresada
//  INPUT:
//      img = imagen ingresada
//      params = parámetros de la cámara
//      desired_configuration = información de referencia
//  OUTPUT:
//      result = resultados
int compute_homography(
    const cv::Mat & img,
    const parameters & params,
    const desired_configuration & Desired_Configuration,
    matching_result& result)
{
    //  TODO: revisar si hace falta normalizar
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> kp;

    //  ORB Detect
    params.orb->detect(img, kp);
    if (kp.size()==0)
        return -1;
    params.orb->compute(img, kp, descriptors);


    //  FLANN matcher
    cv::FlannBasedMatcher matcher(new cv::flann::LshIndexParams(20, 10, 2));
    std::vector<std::vector<cv::DMatch>> matches;
    matcher.knnMatch(
        Desired_Configuration.descriptors,
        descriptors,matches,2);

    //  Filter goodmatches
    std::vector<cv::DMatch> goodMatches;
    for(int i = 0; i < matches.size(); ++i) {
        if (matches[i][0].distance < matches[i][1].distance * params.flann_ratio)
                goodMatches.push_back(matches[i][0]);
        }
    if (goodMatches.size()==0)
        return -1;

    //  Convert
    result.p1.release();
    result.p2.release();
    result.p1 = cv::Mat(goodMatches.size(),2,CV_64F);
    result.p2 = cv::Mat(goodMatches.size(),2,CV_64F);

    for(int i = 0; i < goodMatches.size(); i++){
        //-- Get the keypoints from the good matches

        int idx = goodMatches[i].queryIdx;

        cv::Mat tmp = cv::Mat(Desired_Configuration.kp[idx].pt).t();
        tmp.copyTo(result.p1.row(i));
        tmp.release();
        idx = goodMatches[i].trainIdx;
        tmp = cv::Mat(kp[idx].pt).t();
        tmp.copyTo(result.p2.row(i));
    }

    //  ERROR
    result.mean_feature_error = norm(result.p1,result.p2)/(double)result.p1.rows;
    // Finding homography
    cv::Mat a ,b;
    result.p1.convertTo(a,CV_32FC2);
    result.p2.convertTo(b,CV_32FC2);

    //  Homography
    result.H = cv::findHomography(a,b ,cv::RANSAC, 0.5);
    if (result.H.rows==0)
        return -1;

    //  DRAW
    result.img_matches = cv::Mat::zeros(img.rows, img.cols * 2, img.type());
    cv::drawMatches(
        Desired_Configuration.img,
        Desired_Configuration.kp,
        img, kp, goodMatches,
        result.img_matches,
        cv::Scalar::all(-1),
        cv::Scalar::all(-1),
        std::vector<char>(),
        cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    return 0;

};

////////////////////////////////////////////////////////////////////////////
//  STATE (Class)
//      Abstraction of a camera state
//      Data:
//          +   position and orientation
//          +   Control parameters
//          +   Aquired Image information
//      Methods:
//          +   Inizialization
//          +   Save data
//          +   State update
////////////////////////////////////////////////////////////////////////////
class state {
    public:
    /* defining where the drone will move and integrating system*/
    double X= 0.0,Y= 0.0,Z= 0.0,Yaw= 0.0,Pitch= 0.0,Roll= 0.0;
    bool initialized=false;

    /* Control parameters  */
    double Vx=0.0,Vy=0.0,Vz=0.0;
    double Vyaw=0.0, Vroll = 0.0, Vpitch = 0.0;
    double Kv=1.0;
    double Kw=1.0;
    double dt=0.025;

    // Image proessing parameters
    parameters params;
    //  Desired configuration
    desired_configuration Desired_Configuration;

    //  Best approximations
    bool selected = false;
    cv::Mat t_best;
    cv::Mat R_best; //to save the rot and trans

    // Methods
    state()
    {
        X=0;
        Y=0;
        Z=0;

        Yaw=0;
        Pitch=0;
        Roll=0;
        initialized=false;
        dt=0.025;
        Kv=1.;
        Kw=1.;
    };

    //  Asumes that the State has been mapped to the global frame of reference
    std::pair<Eigen::VectorXd,float> update()
    {

        //  V2 From simulated positions
        X = X + Kv*Vx*dt;
        Y = Y + Kv*Vy*dt;
        Z = Z + Kv*Vz*dt;
        Yaw =  Yaw + Kw*Vyaw*dt;
        Eigen::VectorXd position;
        position.resize(3);
        position(0) = (float) X;
        position(1) = (float) Y;
        position(2) = (float) Z;

        return std::make_pair(position, (float) this -> Yaw);
        // return std::make_pair(position,(float) this->Yaw);
    };

    void load(const ros::NodeHandle &nh)
    {
        // // Load intrinsic parameters
        // XmlRpc::XmlRpcValue kConfig;
        // this->params.K = cv::Mat(3,3, CV_64F, double(0));
        // if (nh.hasParam("camera_intrinsic_parameters")) {
        //     nh.getParam("camera_intrinsic_parameters", kConfig);
        //     if (kConfig.getType() == XmlRpc::XmlRpcValue::TypeArray)
        //     for (int i=0;i<9;i++) {
        //         std::ostringstream ostr;
        //         ostr << kConfig[i];
        //         std::istringstream istr(ostr.str());
        //         istr >> this->params.K.at<double>(i/3,i%3);
        //     }
        // }
        // // Load error threshold parameter
        // this->params.feature_threshold=nh.param(std::string("feature_error_threshold"),std::numeric_limits<double>::max());
        // // Load feature detector parameters
        // this->params.nfeatures=nh.param(std::string("nfeatures"),100);
        // this->params.scaleFactor=nh.param(std::string("scaleFactor"),1.2);
        // this->params.nlevels=nh.param(std::string("nlevels"),5);
        // this->params.edgeThreshold=nh.param(std::string("edgeThreshold"),15);
        // this->params.patchSize=nh.param(std::string("patchSize"),30);
        // this->params.fastThreshold=nh.param(std::string("fastThreshold"),20);
        // this->params.flann_ratio=nh.param(std::string("flann_ratio"),0.7);
        // if (this->params.scaleFactor <= 1.)
        // {
        //     this->params.scaleFactor = 1.2;
        //     ROS_INFO("Params scale factor over ride at 1.2 to avoid orb bug ");
        // }
        params.load(nh);
        // Load gain parameters
        this->Kv=nh.param(std::string("gain_v"),0.0);
        this->Kw=nh.param(std::string("gain_w"),0.0);

        // Load sampling time parameter
        this->dt=nh.param(std::string("dt"),0.01);
    };

    void initialize(const double &x,
                    const double &y,
                    const double &z,
                    const double &yaw)
    {
                        this->X = x;
        this->Y = y;
        this->Z = z;
        this->Yaw = yaw;
        this->initialized = true;
    };

    void save_data(double time, std::string directory)
    {
        std::ofstream outfile(directory, std::ios::app | std::ios::binary);

        outfile.write((char *) & time,sizeof(double));
        outfile.write((char *) & X,sizeof(float));
        outfile.write((char *) & Y,sizeof(float));
        outfile.write((char *) & Z,sizeof(float));
        outfile.write((char *) & Yaw,sizeof(float));
        outfile.write((char *) & Pitch,sizeof(float));
        outfile.write((char *) & Roll,sizeof(float));
        outfile.write((char *) & Vx,sizeof(float));
        outfile.write((char *) & Vy,sizeof(float));
        outfile.write((char *) & Vz,sizeof(float));
        outfile.write((char *) & Vyaw,sizeof(float));
        outfile.write((char *) & Vpitch,sizeof(float));
        outfile.write((char *) & Vroll,sizeof(float));


        outfile.close();
    };

};  //  STATE


////////////////////////////////////////////////////////////////////////////
//  CONTROLLERS
////////////////////////////////////////////////////////////////////////////

//  Controllers

// int homography(cv::Mat img,
//                state & state,
//                matching_result & matching_result
//               );
//
// int chaumette(cv::Mat img,
//                state & state,
//                matching_result & matching_result
//               );


int IBVS(state & state,
            matching_result & Matching_result)
{
    //  Normalización in situ
    // camera_norm(state.params, Matching_result);
    Matching_result.camera_norm(state.params);

    //  Error
    cv::Mat err = Matching_result.p2-Matching_result.p1;

    int n = Matching_result.p2.rows;
    int type = Matching_result.p2.type();
    cv::Mat Z = cv::Mat::ones(n,1,type);
    Z = state.Z * Z;

    //  L = M_{s_i}
    cv::Mat L = interaction_Mat(Matching_result.p2,Z);
    if (L.empty())
        return -1;
    //  L = L^{-1}
    double det=0.0;
    L = Moore_Penrose_PInv(L,det);
    if (det < 1e-13)
    {
        ROS_WARN("[VC Tools] L = %e", det);
        return -1;
    }

    //  U = L^{-1} e
    cv::Mat U = -1. *  L*err.reshape(1,L.cols);

    //  UPDATE
    // U = cv::Mat(U,CV_32F); // para generalizar el tipo de cariable
    state.Vx += (double) U.at<double>(0,0);
    state.Vy += (double) U.at<double>(1,0);
    state.Vz += (double) U.at<double>(2,0);
    state.Vroll += (double) U.at<double>(3,0);
    state.Vpitch += (double) U.at<double>(4,0);
    state.Vyaw += (double) U.at<double>(5,0);



    return 0;
};

int homography(state & State,
        matching_result & result)
{
    // Decompose homography*/
    std::vector<cv::Mat> Rs;
    std::vector<cv::Mat> Ts;
    std::vector<cv::Mat> Ns;
    cv::decomposeHomographyMat(result.H,State.params.K,Rs,Ts,Ns);

    // Select decomposition
    select_decomposition(
        Rs,Ts,Ns,
        result,
        State.selected,
        State.R_best,
        State.t_best);

    /**********Computing velocities in the axis*/
    //velocities from homography decomposition
    State.Vx += (float) State.t_best.at<double>(0,1);
    State.Vy += (float) State.t_best.at<double>(0,0);
    State.Vz += (float) State.t_best.at<double>(0,2); //due to camera framework
    //velocities from homography decomposition and euler angles.
    cv::Vec3f angles = rotationMatrixToEulerAngles(State.R_best);
    State.Vyaw += (float) -angles[2];//due to camera framework
    return 0;
};

// Controller selection array only for vc_controller.h
// typedef int (*funlist) (cv::Mat img,
//                         state & state,
//                         matching_result & matching_result
//                        );
// funlist controllers[] = {&homography,&IBVS};

//   FUNCTION ARRAY FOR CONTROL MEHTOD SELECTION
//      1)  Homography based [Montijano 2016]
//      2)  Classic IBVS with point matcing    [Chaumette 2009]
//      3)  Classic IBVS with ArUcos   [Chaumette 2009]

typedef int (*contList) (state & state,
                        matching_result & matching_result
                    );
const contList controllers[] = {&homography,&IBVS, &IBVS};

typedef int (*prepList) (const cv::Mat&img,
        const parameters & params,
        const desired_configuration & desired_configuration,
        matching_result& result);
const prepList preprocessors[] = {
    &compute_homography,
    &compute_descriptors,
    & compute_arucos
};
}// vct namespace
#endif //   VCT_H
