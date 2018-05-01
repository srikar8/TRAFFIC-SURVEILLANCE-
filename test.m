clear all;
close all;
clc;
%  Steps to be followed
%     1) Read an input video and variable intialization
%     2) Extract the first frame as the backgorund frame
%     3) Apply background subtraction on the subsequent video frames, and
%     some morphological operations
%     4) Calculate the area of the detected object
%     5) If object is of interset , track the motion of the object
%     6) Find the centroid,add bounding box on the detected object and speed detection of the object.

% Read an input video and variable intialization
format shortg;
source = VideoReader('in4.mov');
% source = VideoReader('video.mp4');
nFrames = source.NumberOfFrames;
threshold = 60;
speed1=zeros(nFrames);
speed=zeros(nFrames);
n_centroid = zeros(20,20);
o_centroid = zeros(20,20);
array=zeros(0);
% Extract the first frame
mov(1).cdata = read(source,1);
ref_img = mov(1).cdata;
imshow(ref_img);
ref_img = rgb2gray(ref_img);
ref_img = double(ref_img);
[h,w] = size(ref_img);

% static brute-force background
% ref_img = imread('ref_img.png');
% imshow(ref_img);
% ref_img = double(ref_img);

old_frame_number = 1; 
frame_interval=1;
for x = 2:frame_interval:nFrames
%   Background subtraction and other morhpological operations
    mov(x).cdata = read(source,x);
    frame = mov(x).cdata; 
    frame_bw = rgb2gray(frame);
    frame_bw = double(frame_bw);
    frame_diff = frame_bw - ref_img;
    frame_diff = uint8(frame_diff);
    for i = 1:h
        for j = 1:w
            if(frame_diff(i,j) >= threshold)
%                thresholding is done only on the objects that are in this
%                ,range -> Main area (object will be tracked here!) 
                if (i >= 200 && i <= 800 && j>=500 && j <=1690)  
                    frame_diff(i,j) = 255;
                else
                    frame_diff(i,j) = 0;
                end
            else
                  frame_diff(i,j) = 0;
            end    
        end
    end
    if sum(frame_diff(:))>=50 %counts the number of whites 
        %     Applying Morphological operations
        new_frame_number = x;
        frame_diff = bwconvhull(frame_diff,'objects');
        se = strel('disk',10);
        frame_diff = imopen(frame_diff,se); %opening 
        se = strel('disk',10);
        frame_diff = imclose(frame_diff,se); %closing 
        frame_diff = imfill(frame_diff,'holes');
        frame_diff = bwareaopen(frame_diff,900,8);
        
        %   Object tracking
        track_objects = regionprops(frame_diff,'basic');
        n_centroid = cat(1,track_objects.Centroid);
      
        array=horzcat(array,n_centroid);
            [a1,b1]=size(array);
            if b1>=4
               %for a1=1:a1
                   v = ((array(a1,b1)-array(a1,b1-2))^2 + (array(a1,b1-1)-array(a1,b1-3))^2)^0.5;
                   speed1(a1,(b1/2)-1) = v;
                   speed(a1)=speed1(a1,(b1/2)-1);
                   if array(a1,b1)-array(a1,b1-2) > (array(a1,b1-2)-array(a1,b1)) && array(a1,b1-1)>900 && speed(a1)>=30
                       message = sprintf(' vehile is moving towards camera in right lane with speed greater than 30');
                       helpdlg(message); 
                   elseif array(a1,b1)-array(a1,b1-2) > (array(a1,b1-2)-array(a1,b1)) && array(a1,b1-1)>900 && speed(a1)<30
                       message = sprintf(' vehile is moving towards camera in right lane with "warning" speed less than 30');
                       helpdlg(message); 
                   elseif array(a1,b1)-array(a1,b1-2) > (array(a1,b1-2)-array(a1,b1)) && array(a1,b1-1)<900 && speed(a1)<30
                       message = sprintf('"warning" vehile is moving towards camera in left lane with speed less than 30');
                       helpdlg(message);
                   elseif array(a1,b1)-array(a1,b1-2) > (array(a1,b1-2)-array(a1,b1)) && array(a1,b1-1)<900 && speed(a1)>=30
                       message = sprintf('"warning" vehile is moving towards camera in left lane with "warning" speed greater than 30');
                       helpdlg(message);
                   elseif array(a1,b1)-array(a1,b1-2) <= (array(a1,b1-2)-array(a1,b1)) && array(a1,b1-1)<900 && speed(a1)<30
                       message = sprintf('vehile is moving away from the camera in left lane with speed less than 30');
                       helpdlg(message);
                   elseif array(a1,b1)-array(a1,b1-2) <= (array(a1,b1-2)-array(a1,b1)) && array(a1,b1-1)<900 && speed(a1)>=30
                       message = sprintf('vehile is moving away from the camera in left lane with "warning" speed greater than 30 ');
                       helpdlg(message);
                   elseif array(a1,b1)-array(a1,b1-2) <= (array(a1,b1-2)-array(a1,b1)) && array(a1,b1-1)>900 && speed(a1)>=30
                       message = sprintf('"warning" vehile is moving away from the camera in right lane with speed greater than 30 ');
                       helpdlg(message);
                   else
                       message = sprintf('"warning" vehile is moving away from the camera in right lane with "warning" speed less than 30');
                       helpdlg(message);
                   end
               %end
                
            end
            %speed = SpeedDetectionFunction(old_frame_number,new_frame_number);

            
            
        %   Calling Speed function that will calculate the speed of the object
        
        
        %   Speed Anomaly detection for single lane
        %if (speed < 16)
        %  message = sprintf('Speed anomaly detected,Vehicle must switch to left lane');
        % helpdlg(message);
        %imwrite(frame,'amonaly.png');
        %end
        
        %if size(n_centroid,1) < size(o_centroid,1)
         %   v1 = o_centroid(1:size(n_centroid,1),:);
          %  v2 = n_centroid;
           % v3 = v2;
       % else
        %    v1 = n_centroid(1:size(o_centroid,1),:);
         %   v2 = o_centroid;
          %  v3 = v1;
        %end
    %end
    
    
    %   Zone Marking and lane categorization
    imshow(frame);
   hold on        
        plot([500,1690],[800,800],'LineWidth',2.5,'Color','red'); %line
        plot([700,1050],[200,200],'LineWidth',2.5,'Color','red');%line
        p1 = [200,860];p2 = [800,1000];
        plot([p1(2),p2(2)],[p1(1),p2(1)],'LineWidth',2,'Color','blue');
        text(700,650,'Left lane','Color','white','FontSize',14)
        text(1050,650,'Right lane','Color','white','FontSize',14)
        text(800,250,'Exit Zone','Color','white','FontSize',14) 
        text(800,850,'Enterance Zone','Color','white','FontSize',14) 
        fontsize = 14;
         %   Applying bounding boxes on the tracked object 
         
         
         hold on 
         for index = 1:length(track_objects)
            box = track_objects(index).BoundingBox;
            plot(track_objects(index).Centroid(1),track_objects(index).Centroid(2),'b*');
            rectangle('position',box,'Edgecolor','green','LineWidth',2);
            text(track_objects(index).Centroid(1)+5,track_objects(index).Centroid(2)+5,num2str(round(speed(index))),'Color','red','FontSize',fontsize);
         end
         movegui(gcf);
         F(x) = getframe(figure(1));
         
    %     Updating the old_frame_number
     if(old_frame_number > frame_interval)
         old_frame_number = x - frame_interval;
     end
    end
end
message = sprintf('Speed of the detected vehicle after exiting the main area is: %.3f',speed);
helpdlg(message);