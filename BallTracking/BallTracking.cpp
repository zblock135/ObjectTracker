#include "stdafx.h"
#include <windows.h>
#include <cv.h>
#include <highgui.h>
#include <time.h>


using namespace std;
using namespace cv;
IplImage* imgTracking;
int lastX = -1;
int lastY = -1;
int posX = 0;
int posY = 0;
int lastXb = -1;
int lastYb = -1;
int posXb = 0;
int posYb = 0;
double area=0;
double areab=0;
//This function threshold the HSV image and create a binary image
IplImage* GetThresholdedImage(IplImage* imgHSV){       
    IplImage* imgThresh=cvCreateImage(cvGetSize(imgHSV),IPL_DEPTH_8U, 1);
    cvInRangeS(imgHSV, cvScalar(75,160,60), cvScalar(130,2556,256), imgThresh);
	 
    return imgThresh;
}
IplImage* GetThresholdedImageb(IplImage* imgHSV){       
    IplImage* imgThresh1=cvCreateImage(cvGetSize(imgHSV),IPL_DEPTH_8U, 1);
    cvInRangeS(imgHSV, cvScalar(10,160,60), cvScalar(22,2556,256), imgThresh1);
	 
    return imgThresh1;
}
void LeftClick ( )
        {  
          INPUT input = {0};

           // left down 
          input.type      = INPUT_MOUSE;
          input.mi.dwFlags  = MOUSEEVENTF_LEFTDOWN;
          ::SendInput(1,&input,sizeof(INPUT));

          // left up
          ::ZeroMemory(&input,sizeof(INPUT)); // why zeroMemory? removing this code changes nothing that i can tell
          input.type      = INPUT_MOUSE; // why reset this variable? is it not already set?

          input.mi.dwFlags  = MOUSEEVENTF_LEFTUP;
          ::SendInput(1,&input,sizeof(INPUT));
        }
void trackObject(IplImage* imgThresh){
    // Calculate the moments of 'imgThresh'
    CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
    cvMoments(imgThresh, moments, 1);
    double moment10 = cvGetSpatialMoment(moments, 1, 0);
    double moment01 = cvGetSpatialMoment(moments, 0, 1);
    area = cvGetCentralMoment(moments, 0, 0);

     // if the area<1000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
    if(area>500){
        // calculate the position of the ball
        posX = moment10/area;
        posY = moment01/area;    
		double fontScale = 1;
		int thickness = 2;  
		std::string x = std::to_string(posX);
		std::string y = std::to_string(posY);
		if(lastX>=0 && lastY>=0 && posX>=0 && posY>=0)
        {
            // Draw a yellow line from the previous point to the current point
           // cvLine(imgTracking, cvPoint(posX, posY), cvPoint(lastX, lastY), cvScalar(0,0,255), 4);
			cv::putText(cv::Mat(imgTracking),x,cvPoint(0,200),FONT_HERSHEY_COMPLEX_SMALL, fontScale, Scalar::all(255));
			cv::putText(cv::Mat(imgTracking),y,cvPoint(0,300),FONT_HERSHEY_COMPLEX_SMALL, fontScale, Scalar::all(255));
        }
       

         lastX = posX;
         lastY = posY;
    }

     free(moments); 
}
void trackObject1(IplImage* imgThresh1){
    // Calculate the moments of 'imgThresh1'
    CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
    cvMoments(imgThresh1, moments, 1);
    double moment10b = cvGetSpatialMoment(moments, 1, 0);
    double moment01b = cvGetSpatialMoment(moments, 0, 1);
    areab = cvGetCentralMoment(moments, 0, 0);

     // if the area<1000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
    if(areab>500){
        // calculate the position of the ball
        posXb = moment10b/areab;
        posYb = moment01b/areab;    
		double fontScale = 1;
		int thickness = 2;  
		std::string xb = std::to_string(posXb);
		std::string yb = std::to_string(posYb);
		if(lastXb>=0 && lastYb>=0 && posXb>=0 && posYb>=0)
        {
            // Draw a yellow line from the previous point to the current point
           // cvLine(imgTracking, cvPoint(posX, posY), cvPoint(lastX, lastY), cvScalar(0,0,255), 4);
			cv::putText(cv::Mat(imgTracking),xb,cvPoint(50,200),FONT_HERSHEY_COMPLEX_SMALL, fontScale, Scalar::all(255));
			cv::putText(cv::Mat(imgTracking),yb,cvPoint(50,300),FONT_HERSHEY_COMPLEX_SMALL, fontScale, Scalar::all(255));
        }
       

         lastXb = posXb;
         lastYb = posYb;
    }

     free(moments); 
}

int main(){
	
		int x_center = 1920/2;
		int y_center = 1080/2;
		int camera_x_center = 640/2;
		int camera_y_center = 480/2;


      CvCapture* capture =0;       
      capture = cvCaptureFromCAM(0);
      if(!capture){
         printf("Capture failure\n");
         return -1;
      }
      
      IplImage* frame=0;
      frame = cvQueryFrame(capture);           
      if(!frame) return -1;
   
     //create a blank image and assigned to 'imgTracking' which has the same size of original video
     imgTracking=cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U, 3);

     cvNamedWindow("Video");     
     cvNamedWindow("Ball");
	 cvNamedWindow("orange");
	 int i=0;
      //iterate through each frames of the video     
      while(true){
		  
            cvZero(imgTracking); //covert the image, 'imgTracking' to black
		
   
            frame = cvQueryFrame(capture);           
            if(!frame) break;
            frame=cvCloneImage(frame); 
            
            cvSmooth(frame, frame, CV_GAUSSIAN,3,3); //smooth the original image using Gaussian kernel

            IplImage* imgHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3); 
            cvCvtColor(frame, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV
            IplImage* imgThresh = GetThresholdedImage(imgHSV);
			IplImage* imgThresh1 = GetThresholdedImageb(imgHSV);
          
            cvSmooth(imgThresh, imgThresh, CV_MEDIAN,5,5); //smooth the binary image using Gaussian kernel
			cvSmooth(imgThresh1, imgThresh1, CV_MEDIAN,5,5);
            //cv::medianBlur(
           //track the possition of the ball
           trackObject(imgThresh);
		   trackObject1(imgThresh1);
            // Add the tracking image and the frame
           cvAdd(frame, imgTracking, frame);

		   cvShowImage("orange",imgThresh1);
            cvShowImage("Ball", imgThresh);           
           cvShowImage("Video", frame);
		   if (area>500){
            SetCursorPos(x_center+(camera_x_center-(posX*.7+lastX*.3))*3.75, y_center-(camera_y_center-(posY*.7+lastY*.3))*3);
		   }
		   if ((areab>500) && (i%15==0)){
			   
			   LeftClick();
			   
		   }
           //Clean up used images
           cvReleaseImage(&imgHSV);
           cvReleaseImage(&imgThresh);            
           cvReleaseImage(&frame);
		   cvReleaseImage(&imgThresh1);
		   
            //Wait 10mS
            int c = cvWaitKey(10);
            //If 'ESC' is pressed, break the loop
            if((char)c==27 ) break; 
			i=i+1;
      }

      cvDestroyAllWindows() ;
      cvReleaseImage(&imgTracking);
      cvReleaseCapture(&capture);     
	  

      return 0;
}