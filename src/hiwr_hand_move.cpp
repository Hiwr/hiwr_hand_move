/*********************************************************************
*
*
* Copyright 2014 Worldline
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
* 
*     http://www.apache.org/licenses/LICENSE-2.0
* 
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
* 
***********************************************************************/

#include "hiwr_hand_move.h"

using namespace cv;

namespace hiwr_hand_move{
void HiwrHandMove::onInit() {
    ROS_DEBUG("[Hiwr Hand Move Nodelet][onInit] Processing tired frame");

    public_nh_ = getNodeHandle();
    private_nh_ = getMTPrivateNodeHandle();
    it_ = new image_transport::ImageTransport(public_nh_);

    if(!private_nh_.getParam("video_stream", video_stream_name_)){
        NODELET_ERROR("[Hiwr Hand Nodelet][onInit] Problem recovering the video stream");
        return;
    }
    im_available_ = false;

    // Publisher
    pub_ = public_nh_.advertise<std_msgs::UInt8>("/hiwr_hand_move/move", 1);

    // Subscriber
    image_sub_ = it_->subscribe(video_stream_name_.c_str(), 1,&HiwrHandMove::callback, this);
   

    data_received_ = false;
    loop_thread_ = std::thread(&HiwrHandMove::loop , this);

}

void HiwrHandMove::loop(){
    ros::Rate loop_rate(10);


    image_transport::ImageTransport its(public_nh_);
    image_transport::Publisher pub = its.advertise("/camTop/output_video_test", 1);

    int published_frame_=0;
    int last = -1;
    while(ros::ok()){

        // When receiving data, publishing them on 
        if(data_received_){
            frame_ = im_ptr_->image;

            ROS_INFO("[Hiwr Hand Nodelet][onInit] Pnew frame");

            char frame_id[20];
            sprintf(frame_id , "%d" , published_frame_);
            
            published_frame_++;
            cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
            cv_ptr->header.seq = published_frame_;
          
            cv_ptr->header.stamp = ros::Time::now();
            cv_ptr->header.frame_id = frame_id;
            cv_ptr->encoding = sensor_msgs::image_encodings::MONO8; //"mono8";//"bgr8";

            if( frame_memory_.size().width>0){

            Mat diff ; 
            absdiff(frame_, frame_memory_, diff);
            threshold( diff, diff, 35, 255,CV_THRESH_BINARY );

            Mat resize_;
            resize(diff,resize_, Size() , 0.1, 0.1, INTER_CUBIC);
            threshold( resize_, resize_, 200, 255,CV_THRESH_BINARY );

            Mat resize2_ ;// diff;
            resize(resize_,resize2_, Size() , 10, 10, INTER_CUBIC);
            threshold( resize2_, resize2_, 200, 255,CV_THRESH_BINARY );
            
            Mat edges;

            Canny( resize2_, edges, 100, 300, 3);
            
            //Mat contours; 
            vector<vector<Point> > contours;
            contours.clear();
            findContours(edges, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

            //if(contours.size().width>0){
            Mat final = Mat::zeros( edges.size(), CV_8UC1);
           
           double biggestContours= 0;
           double biggestContoursIndex= -1;
            for(int i=0; i< contours.size(); i++){
                double area = contourArea(contours[i]);
                 if(area > biggestContours){
                    biggestContours = area;
                    biggestContoursIndex = i;
                 }
           }

            Scalar color(255);
  

        if( biggestContours >1000){
            Rect rect = boundingRect( contours[biggestContoursIndex]);
            Point center( rect.x + rect.width/2, rect.y + rect.height/2);

            Point ref( edges.size().width/2, edges.size().height/2 );
            double res = cv::norm(center-ref);

            if(res<100){

            std::cout << "biggestContours is " << biggestContours << '\n';
            std::cout << "res is " << res << '\n';

            if(last<2){
                drawContours( final , contours, biggestContoursIndex, color);
                cv_ptr->image = final;
                pub.publish(cv_ptr->toImageMsg());

                msg_UInt8_.data = biggestContours;
                pub_.publish(msg_UInt8_);
                
            }

                last=0;
            }
            
            //msg_UInt8_.data = out.data[0];
            //pub_.publish(msg_UInt8_);
        
            }    
        }
            //}


        //    printf("new frame \n");
         //   Mat out(Size(1,1) , CV_8UC1) ;
          //  cv::resize(frame_ , out ,  out.size() );

            //msg_UInt8_.data = out.data[0];
            //pub_.publish(msg_UInt8_);

            data_received_ = false;
        }

        frame_memory_ = frame_;
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void HiwrHandMove::callback(const sensor_msgs::ImageConstPtr& msg){
    data_received_ = true;
    im_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
}

PLUGINLIB_DECLARE_CLASS(hiwr_hand_move, HiwrHandMove, hiwr_hand_move::HiwrHandMove, nodelet::Nodelet);
}
