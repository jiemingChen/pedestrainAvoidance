//
// Created by jieming on 01.02.20.
//

#include <boost/bind.hpp>
#include <boost/bind/bind.hpp>
#include "LaserRecognizer.h"
LaserRecognizer::LaserRecognizer(){
    pfeature[0] = boost::bind(&LaserRecognizer::feature1,this);
    pfeature[1] = boost::bind(&LaserRecognizer::feature2,this);
    pfeature[2] = boost::bind(&LaserRecognizer::feature3,this);
    pfeature[3] = boost::bind(&LaserRecognizer::feature4,this);
    pfeature[4] = boost::bind(&LaserRecognizer::feature5,this);
}

SamplePoint LaserRecognizer::transform(const std::pair<float, float>& beam, const int i){

        float y = -beam.first*cos(beam.second);
        float x = beam.first*sin(beam.second);
        return {x,y,i};
}

std::vector<std::vector<SamplePoint>> LaserRecognizer::segment(const sensor_msgs::LaserScanConstPtr & scanPtr) {
    segments.clear();  //!!!!
    // 1. remove elems that exceed maximum and minimum
    for (int i = 0; i < scanPtr->ranges.size(); i++) {
        if ((scanPtr->ranges[i] > scanPtr->range_max) || (scanPtr->ranges[i] < scanPtr->range_min)) {
            continue;
        }
#if 0
        auto angle = scanPtr->angle_min + 1.57 + scanPtr->angle_increment * i;
        auto x = scanPtr->ranges[i] * cos(angle);
        auto y = scanPtr->ranges[i] * sin(angle);
        if (!( (y<x-0.4545) || (y<-x-0.4545))){
            continue;
        }
#endif
        else {
            filteredScan.emplace_back(
                    std::make_pair(scanPtr->ranges[i], scanPtr->angle_min + 1.57 + scanPtr->angle_increment * i));
        }
    }
    // 2. segment
    std::vector<int> splitIdx;
    double dist;
    double threshold=0.4;

    for(int i=1; i<filteredScan.size(); i++) {
        double aa = pow(filteredScan[i].first, 2);
        double bb = pow(filteredScan[i - 1].first, 2);
        double ab = filteredScan[i].first * filteredScan[i - 1].first;
        double coss = cos(filteredScan[i].second - filteredScan[i - 1].second);
        dist = sqrt(  aa + bb - 2 * ab * coss );

        if(dist> threshold){
            splitIdx.push_back(i);
        }
    }
    splitIdx.push_back(filteredScan.size());

    // 3. store and convert to carti coord
    int start_idx=0;
    for(int& idx: splitIdx){
       for(int i=start_idx; i<idx; i++){
           if(idx-start_idx>=2){
               auto point = transform(filteredScan[i], i);
               samples.push_back(point);
           }
       }
       start_idx = idx;
       if(!samples.empty()){
           segments.push_back(samples);
           samples.clear();
       }
    }
    filteredScan.clear();
    return segments;
}


void LaserRecognizer::feature1(){
    std::vector<cv::Point> temp;
    std::vector<cv::Rect> rects;

    for(auto& segment: segments){
        for(auto& point: segment){
            temp.push_back(cv::Point(point.x, point.y));
        }
        rects.push_back(cv::boundingRect(temp));
        temp.clear();
    }
    for(auto& rect: rects){
        auto diagonal = pow(pow(rect.height,2)+pow(rect.width,2), 0.5);
        feature1Vector.push_back(diagonal);
    }
}

void LaserRecognizer::feature2(){
    float radius;
    Eigen::Vector3f circle;

    for(auto& segment: segments){
        Eigen::MatrixXf A(segment.size(),3);
        Eigen::MatrixXf b(segment.size(),1);
        for(int i=0; i<segment.size(); i++){
            A(i,0) = -segment[i].x*2;
            A(i,1) = -segment[i].y*2;
            A(i,2) = 1;
            b(i,0) = -(pow(segment[i].x, 2) + pow(segment[i].y, 2));
        }
        circle = (A.transpose()*A).inverse()*A.transpose()*b;
        radius = pow(-circle(2,0) + pow(circle(0,0),2) + pow(circle(1,0),2), 0.5);
        if(isnan(radius))
        {
            radius = 100;  //nan
        }
        feature2Vector.push_back(radius);
    }
}

void LaserRecognizer::feature3(){
    float width;
    for(auto& segment: segments){
        width = abs(segment.back().x - segment.front().x);
        feature3Vector.push_back(width);
    }
}

void LaserRecognizer::feature4(){
    float deviation=0;
    int len;
    int medInx;
    float medianX;
    float medianY;
    std::vector<float> yVal;

    for(auto& segment: segments){
        len = segment.size();
        // find median value of x,y
        for(auto& point: segment){
           yVal.push_back(point.y);
        }
        std::sort(yVal.begin(), yVal.end());

        if(len%2==1){
            medInx = floor(len/2);
            medianX = segment[medInx].x;
            medianY = yVal[medInx];
        }
        else{
            medInx = len/2;
            medianX = (segment[medInx].x + segment[medInx-1].x)/2.0;
            medianY = (yVal[medInx] + yVal[medInx-1])/2.0;
        }

        //store position
        candidatePosition.emplace_back(medianX, medianY);
        // calc deviation
        for(auto& point: segment){
            deviation += 1.0/len * sqrt(pow(medianX-point.x, 2) + pow(medianY-point.y, 2));
        }
        feature4Vector.push_back(deviation);

        yVal.clear();
        deviation=0;
    }
}

void LaserRecognizer::feature5(){
    float sumOfDist;
    for(auto& seg: segments) {
        sumOfDist=0;
        for(int i=0; i<seg.size()-1; i++){
            double xx = pow(seg[i].x-seg[i+1].x, 2);
            double yy = pow(seg[i].y-seg[i+1].y, 2);
            sumOfDist += float(pow(xx+yy, 0.5));
        }
        feature5Vector.push_back(sumOfDist);
    }
}

void LaserRecognizer::test(){
    feature1();
    feature2();
    feature3();
    feature4();

    for(int i=0; i<segments.size(); i++){
        int cnt=0;
        if(abs(segments[i][1].y) < 4 && abs(segments[i][1].x) < 4){
            if(feature1Vector[i] > featureThreshold[0]  && feature1Vector[i] < featureThreshold[1])
                cnt +=1;
            else
                cnt -= 1;
            if(feature2Vector[i] > featureThreshold[2]  && feature2Vector[i] < featureThreshold[3])
                cnt +=1;
            else
                cnt -= 1;
            if(feature3Vector[i] > featureThreshold[4]  && feature3Vector[i] < featureThreshold[5])
                cnt +=1;
            else
                cnt -= 1;
            if(feature4Vector[i] > featureThreshold[6]  && feature4Vector[i] < featureThreshold[7])
                cnt +=1;
            else
                cnt -= 1;
        }
        else{
            if(feature1Vector[i] > featureThreshold[0]  && feature1Vector[i] < featureThreshold[1])
                cnt +=1;
            else
                cnt -= 1;
        }
        if(cnt>0){
            std::cout<<"people"<<std::endl;
        }
        else{
            std::cout<<"no"<<std::endl;

        }

    }


    segments.clear();
    feature1Vector.clear();
    feature2Vector.clear();
    feature3Vector.clear();
    feature4Vector.clear();
}

void LaserRecognizer::generateDataSet(){
    for(int i=0; i<5; i++){
        pfeature[i]();
    }

    std::ofstream outFile;
    outFile.open("/home/jieming/simdata/laser/dataTrue13.csv", std::ios::out);
    outFile << "baseline" << "," <<"radius" <<","<<"width"<<","<<"deviate"  <<","<<"sumDist"  <<","<<"label" << std::endl;
    std::cout<<segments.size()<<std::endl;
    for(int i=0; i<segments.size(); i++){
        outFile << std::to_string(feature1Vector[i]) << ","  << std::to_string(feature2Vector[i]) << "," << std::to_string(feature3Vector[i]) << ","
                << std::to_string(feature4Vector[i]) << ","  << std::to_string(feature5Vector[i]) << ","  "1" << std::endl;
    }
    outFile.close();
    std::cout<<"!!";
}
#if 0
static void train_and_print_errs(cv::Ptr<StatModel> model, const cv::Ptr<TrainData>& data)
{
    bool ok = model->train(data);
    if( !ok )
    {
        printf("Training failed\n");
    }
    else
    {
        printf( "train error: %f\n", model->calcError(data, false, cv::noArray()) );
//        printf( "test error: %f\n\n", model->calcError(data, true, cv::noArray()) );
    }
}

void LaserRecognizer::train(){

    cv::Ptr<TrainData> data = TrainData::loadFromCSV(std::string("/home/jieming/train.csv"), 1, -1, -1);

//    data->setTrainTestSplitRatio(0.1);
    cv::Ptr<cv::ml::DTrees> dtree = cv::ml::DTrees::create();
    dtree->setMaxDepth(12);
    dtree->setMinSampleCount(2);
    dtree->setRegressionAccuracy(0);
    dtree->setUseSurrogates(false);
    dtree->setMaxCategories(2);
    dtree->setCVFolds(0);
    dtree->setUse1SERule(false);
    dtree->setTruncatePrunedTree(false);
    dtree->setPriors(cv::Mat());
    train_and_print_errs(dtree, data);
    dtree->save("/home/jieming/DT.xml");
    std::cout<<"!!";
    std::cout<<"!!";

}

void LaserRecognizer::dTreeTest(){
    std::cout << "======TEST====="<<std::endl;
    cv::Ptr<DTrees> dtree2 = DTrees::load<DTrees>("/home/jieming/DT.xml");

    for(int i=0; i<5; i++){
        pfeature[i]();
    }

    std::vector<float>testVec;

    if(!peoplePosition.empty()){
        peoplePosition.clear();
        //biao zhi wei
    }

    for(int i=0; i<segments.size(); i++) {
        testVec.push_back(feature1Vector[i]);
        testVec.push_back(feature2Vector[i]);
        testVec.push_back(feature3Vector[i]);
        testVec.push_back(feature4Vector[i]);
        testVec.push_back(feature5Vector[i]);
        float resultKind = dtree2->predict(testVec);
        std::cout << "rst"<<i <<"   " <<resultKind<<std::endl;
        testVec.clear();

        if(resultKind>0){
            peoplePosition.push_back(candidatePosition[i]);
        }
    }
    std::cout << "======TEST====="<<std::endl;


}
#endif
