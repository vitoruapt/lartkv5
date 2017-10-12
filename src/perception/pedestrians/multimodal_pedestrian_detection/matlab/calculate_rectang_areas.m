%% extract bounding boxes of images
clc
clear all
close all

% dir_images = '/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/bag_1_full/';
% dir_txt = '/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/bag_1_full_txt/';

nfile = 5950;

pedect=0;

% file_image=sprintf('/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/bag_1_full/00%d.png',nfile);
% 
% file_txt=sprintf('/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/bag_1_full_txt/00%d.txt',nfile);

file_image=sprintf('/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/bag_2_full/00%d.png',nfile);

file_txt=sprintf('/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/bag_2_full_txt/00%d.txt',nfile);

fileID = fopen(file_txt, 'a');

if (pedect==0)
    fprintf(fileID, ' ');   
else
    a = imread(file_image);

    imshow(a);

    h=imrect(); % permite desenhar um rectangulo na imagem

    pos = getPosition(h); % permite extrair as propriedades do rectangulo
    
    while(pos==0)
    end
    
%     if (pos(1) < 0)
%         pos(1) = 0;        
%     end
%     if (pos(1) > 800)
%         pos(1) = size(a,2);        
%     end
    
    fprintf(fileID, '%03d   %03d   %03d   %03d\n',pos);        
end

fclose(fileID);

%% Extract algorithm performance
clc
clear all
close all

nfile_inicial = 7299;
nfile = 9029;
TP_Pedro_Algorithm = 0;
TN_Pedro_Algorithm = 0;
FP_Pedro_Algorithm = 0;
FN_Pedro_Algorithm = 0;
hist_Pedro_Algorithm = [];

TP_Fusion_Algorithm = 0;
TN_Fusion_Algorithm = 0;
FP_Fusion_Algorithm = 0;
FN_Fusion_Algorithm = 0;

TP_Fusion_2imagereset_Algorithm = 0;
TN_Fusion_2imagereset_Algorithm = 0;
FP_Fusion_2imagereset_Algorithm = 0;
FN_Fusion_2imagereset_Algorithm = 0;
hist_Fusion_2imagereset_Algorithm = [];

TP_Fusion_4imagereset_Algorithm = 0;
TN_Fusion_4imagereset_Algorithm = 0;
FP_Fusion_4imagereset_Algorithm = 0;
FN_Fusion_4imagereset_Algorithm = 0;
hist_Fusion_4imagereset_Algorithm = [];

TP_Fusion_10imagereset_Algorithm = 0;
TN_Fusion_10imagereset_Algorithm = 0;
FP_Fusion_10imagereset_Algorithm = 0;
FN_Fusion_10imagereset_Algorithm = 0;
hist_Fusion_10imagereset_Algorithm = [];

TP_Fusion_2imagereset_1p3_Algorithm = 0;
TN_Fusion_2imagereset_1p3_Algorithm = 0;
FP_Fusion_2imagereset_1p3_Algorithm = 0;
FN_Fusion_2imagereset_1p3_Algorithm = 0;
hist_Fusion_2imagereset_1p3_Algorithm = [];

TP_Fusion_4imagereset_1p3_Algorithm = 0;
TN_Fusion_4imagereset_1p3_Algorithm = 0;
FP_Fusion_4imagereset_1p3_Algorithm = 0;
FN_Fusion_4imagereset_1p3_Algorithm = 0;
hist_Fusion_4imagereset_1p3_Algorithm = [];

TP_Fusion_6imagereset_1p3_Algorithm = 0;
TN_Fusion_6imagereset_1p3_Algorithm = 0;
FP_Fusion_6imagereset_1p3_Algorithm = 0;
FN_Fusion_6imagereset_1p3_Algorithm = 0;
hist_Fusion_6imagereset_1p3_Algorithm = [];

TP_Fusion_8imagereset_1p3_Algorithm = 0;
TN_Fusion_8imagereset_1p3_Algorithm = 0;
FP_Fusion_8imagereset_1p3_Algorithm = 0;
FN_Fusion_8imagereset_1p3_Algorithm = 0;
hist_Fusion_8imagereset_1p3_Algorithm = [];

TP_Fusion_10imagereset_1p3_Algorithm = 0;
TN_Fusion_10imagereset_1p3_Algorithm = 0;
FP_Fusion_10imagereset_1p3_Algorithm = 0;
FN_Fusion_10imagereset_1p3_Algorithm = 0;
hist_Fusion_10imagereset_1p3_Algorithm = [];

TP_Fusion_20imagereset_1p3_Algorithm = 0;
TN_Fusion_20imagereset_1p3_Algorithm = 0;
FP_Fusion_20imagereset_1p3_Algorithm = 0;
FN_Fusion_20imagereset_1p3_Algorithm = 0;
hist_Fusion_20imagereset_1p3_Algorithm = [];

TP_Fusion_infimagereset_1p3_Algorithm = 0;
TN_Fusion_infimagereset_1p3_Algorithm = 0;
FP_Fusion_infimagereset_1p3_Algorithm = 0;
FN_Fusion_infimagereset_1p3_Algorithm = 0;
hist_Fusion_infimagereset_1p3_Algorithm = [];

num_BB_positive = 0;
num_BB_negative = 0;
% 209
for i=0:209
    
    nfile = nfile_inicial + i*10;
    
%     TP_file_txt=sprintf('/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/matlab/TP.txt');
%     TN_file_txt=sprintf('/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/matlab/TN.txt');
%     FP_file_txt=sprintf('/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/matlab/FP.txt');
%     FN_file_txt=sprintf('/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/matlab/FN.txt');
%     fileID_TP = fopen(TP_file_txt, 'a');
%     fileID_TN = fopen(TN_file_txt, 'a');
%     fileID_FP = fopen(FP_file_txt, 'a');
%     fileID_FN = fopen(FN_file_txt, 'a'); 

    bag_1_full_file_txt=sprintf('/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/bag_1_full_txt/00%d.txt',nfile);
    Pedro_Algorithm_file_txt=sprintf('/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/bag_1_Pedro_Algorithm_1_txts/%d.txt',nfile);
    Fusion_Algorithm_file_txt=sprintf('/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/bag_1_sensor_fusion_2_txts/%d.txt',nfile);
    Fusion_Algorithm_2imagereset_1p3_file_txt=sprintf('/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/bag_1_sensor_fusion_2image_reset_1.3_txts/%d.txt',nfile);
    Fusion_Algorithm_2imagereset_file_txt=sprintf('/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/bag_1_sensor_fusion_2image_reset_txts/%d.txt',nfile);
    Fusion_Algorithm_4imagereset_1p3_file_txt=sprintf('/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/bag_1_sensor_fusion_4image_reset_1.3_txts/%d.txt',nfile);
    Fusion_Algorithm_4imagereset_file_txt=sprintf('/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/bag_1_sensor_fusion_4image_reset_txts/%d.txt',nfile);
    Fusion_Algorithm_6imagereset_1p3_file_txt=sprintf('/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/bag_1_sensor_fusion_6image_reset_1.3_txts/%d.txt',nfile);
    Fusion_Algorithm_8imagereset_1p3_file_txt=sprintf('/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/bag_1_sensor_fusion_8image_reset_1.3_txts/%d.txt',nfile);
    Fusion_Algorithm_10imagereset_1p3_file_txt=sprintf('/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/bag_1_sensor_fusion_10image_reset_1.3_txts/%d.txt',nfile);
    Fusion_Algorithm_10imagereset_file_txt=sprintf('/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/bag_1_sensor_fusion_10image_reset_txts/%d.txt',nfile);
    Fusion_Algorithm_20imagereset_1p3_file_txt=sprintf('/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/bag_1_sensor_fusion_20image_reset_1.3_txts/%d.txt',nfile);
    Fusion_Algorithm_infimagereset_1p3_file_txt=sprintf('/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/bag_1_sensor_fusion_1.3_txts_copia/%d.txt',nfile+1);
    
    fileID_bag_1 = fopen(bag_1_full_file_txt, 'r');
    fileID_Pedro_Algorithm = fopen(Pedro_Algorithm_file_txt, 'r');
    fileID_Fusion_Algorithm = fopen(Fusion_Algorithm_file_txt, 'r');
    fileID_Fusion_2imagereset_1p3_Algorithm = fopen(Fusion_Algorithm_2imagereset_1p3_file_txt, 'r');
    fileID_Fusion_2imagereset_Algorithm = fopen(Fusion_Algorithm_2imagereset_file_txt, 'r');
    fileID_Fusion_4imagereset_1p3_Algorithm = fopen(Fusion_Algorithm_4imagereset_1p3_file_txt, 'r');
    fileID_Fusion_4imagereset_Algorithm = fopen(Fusion_Algorithm_4imagereset_file_txt, 'r');
    fileID_Fusion_6imagereset_1p3_Algorithm = fopen(Fusion_Algorithm_6imagereset_1p3_file_txt, 'r');
    fileID_Fusion_8imagereset_1p3_Algorithm = fopen(Fusion_Algorithm_8imagereset_1p3_file_txt, 'r');
    fileID_Fusion_10imagereset_1p3_Algorithm = fopen(Fusion_Algorithm_10imagereset_1p3_file_txt, 'r');
    fileID_Fusion_10imagereset_Algorithm = fopen(Fusion_Algorithm_10imagereset_file_txt, 'r');
    fileID_Fusion_20imagereset_1p3_Algorithm = fopen(Fusion_Algorithm_20imagereset_1p3_file_txt, 'r');
    fileID_Fusion_infimagereset_1p3_Algorithm = fopen(Fusion_Algorithm_infimagereset_1p3_file_txt, 'r');
    
    A_bag_1 = fscanf(fileID_bag_1,'%e');
    A_Pedro_Algorithm = fscanf(fileID_Pedro_Algorithm,'%f');
    A_Fusion_Algorithm = fscanf(fileID_Fusion_Algorithm,'%f');
    A_Fusion_2imagereset_1p3_Algorithm = fscanf(fileID_Fusion_2imagereset_1p3_Algorithm,'%f');
    A_Fusion_2imagereset_Algorithm = fscanf(fileID_Fusion_2imagereset_Algorithm,'%f');
    A_Fusion_4imagereset_1p3_Algorithm = fscanf(fileID_Fusion_4imagereset_1p3_Algorithm,'%f');
    A_Fusion_4imagereset_Algorithm = fscanf(fileID_Fusion_4imagereset_Algorithm,'%f');
    A_Fusion_6imagereset_1p3_Algorithm = fscanf(fileID_Fusion_6imagereset_1p3_Algorithm,'%f');
    A_Fusion_8imagereset_1p3_Algorithm = fscanf(fileID_Fusion_8imagereset_1p3_Algorithm,'%f');
    A_Fusion_10imagereset_1p3_Algorithm = fscanf(fileID_Fusion_10imagereset_1p3_Algorithm,'%f');
    A_Fusion_10imagereset_Algorithm = fscanf(fileID_Fusion_10imagereset_Algorithm,'%f');
    A_Fusion_20imagereset_1p3_Algorithm = fscanf(fileID_Fusion_20imagereset_1p3_Algorithm,'%f');
    A_Fusion_infimagereset_1p3_Algorithm = fscanf(fileID_Fusion_infimagereset_1p3_Algorithm,'%f');
        
    % Pedro algorithm
    if (size(A_bag_1,1) > 0)
        if (A_Pedro_Algorithm(1) == 0 && A_Pedro_Algorithm(2) == 0)
            % false negative - miss
%             fprintf(fileID_FN,'%d\n',(size(A_bag_1,1)/4));
            FN_Pedro_Algorithm = FN_Pedro_Algorithm + (size(A_bag_1,1)/4);
        else
            % Test to see if BB's correspond, can be true positive or false
            % positive
            bbs_reshape_pedro = reshape(A_Pedro_Algorithm,[5 size(A_Pedro_Algorithm,1)/5])';
            for q=1:size(bbs_reshape_pedro,1)
                bbs_reshape_pedro(q, 5) = 1;
            end
            bbs_correct_pedro = bbNms(bbs_reshape_pedro, 'thr',0.5, 'type','ms');
            
            intersec_data_pedro = zeros(1,size(bbs_correct_pedro, 1));
            for j=0:(size(A_bag_1)/4 - 1)
                for l=0:(size(bbs_correct_pedro,1) - 1)
                    if(bbs_correct_pedro(l+1, 1) ~= 0)
                        
                        A = [A_bag_1(1+4*j), A_bag_1(2+4*j), A_bag_1(3+4*j), A_bag_1(4+4*j)];
                        B = [bbs_correct_pedro(l+1, 1), bbs_correct_pedro(l+1, 2), bbs_correct_pedro(l+1, 3), bbs_correct_pedro(l+1, 4)];
                        
                        area_A = rectint(A,A);
                        area_intersec = rectint(A,B);

                        if (area_intersec/area_A >= 0.5)
%                             A_Pedro_Algorithm(1+5*j) = 0; 
%                             A_Pedro_Algorithm(2+5*j) = 0;
%                             A_Pedro_Algorithm(3+5*j) = 0;
%                             A_Pedro_Algorithm(4+5*j) = 0;
%                             fprintf(fileID_TP,'%d \n',1);
                            intersec_data_pedro(l+1) = intersec_data_pedro(l+1) +1;
                            TP_Pedro_Algorithm = TP_Pedro_Algorithm + 1;
                        end
                    end
                end
            end
            for k=0:(size(bbs_correct_pedro,1) - 1)
%                 if(A_Pedro_Algorithm(1+5*k) ~= 0)
                if(intersec_data_pedro(k+1) == 0)
%                     fprintf(fileID_FP,'%d \n',1);
                    FP_Pedro_Algorithm = FP_Pedro_Algorithm + 1;
                end
            end
        end
    else
        if (A_Pedro_Algorithm(1) == 0 && A_Pedro_Algorithm(2) == 0)
            % true negative - not found any pedestrians on either images
%             fprintf(fileID_TN,'%d \n',1);
            TN_Pedro_Algorithm = TN_Pedro_Algorithm + 1;
        else
            % false positive - false alarm
%             fprintf(fileID_FP,'%d \n',(size(A_Pedro_Algorithm,1))/5);
            bbs_reshape_pedro = reshape(A_Pedro_Algorithm,[5 size(A_Pedro_Algorithm,1)/5])';
            bbs_correct_pedro = bbNms(bbs_reshape_pedro, 'thr',0.5, 'type','ms');
            
            FP_Pedro_Algorithm = FP_Pedro_Algorithm + (size(bbs_correct_pedro,1));
        end
    end
    
    
    % Fusion algorithm
    if (size(A_bag_1,1) > 0)
        if (size(A_Fusion_Algorithm) == 0)
            % false negative - miss
%             fprintf(fileID_FN,'%d\n',(size(A_bag_1,1)/4));
            FN_Fusion_Algorithm = FN_Fusion_Algorithm + (size(A_bag_1,1)/4);
        else
            % Test to see if BB's correspond, can be true positive or false
            % positive
            bbs_reshape_fusion = reshape(A_Fusion_Algorithm,[5 size(A_Fusion_Algorithm,1)/5])';
            for q=1:size(bbs_reshape_fusion,1)
                bbs_reshape_fusion(q, 5) = 1;
            end
            bbs_correct_fusion = bbNms(bbs_reshape_fusion, 'thr',0.5, 'type','ms');
            
            intersec_data_fusion = zeros(1,size(bbs_correct_fusion, 1));
            for j=0:(size(A_bag_1)/4 - 1)
                for l=0:(size(bbs_correct_fusion,1) - 1)
                    if(bbs_correct_fusion(l+1, 1) ~= 0)
                        
                        A = [A_bag_1(1+4*j), A_bag_1(2+4*j), A_bag_1(3+4*j), A_bag_1(4+4*j)];
                        B = [bbs_correct_fusion(l+1, 3), bbs_correct_fusion(l+1, 2), bbs_correct_fusion(l+1, 1) - bbs_correct_fusion(l+1, 3), bbs_correct_fusion(l+1, 4) - bbs_correct_fusion(l+1, 2)];
%                           B = [A_Fusion_Algorithm(3+5*l), A_Fusion_Algorithm(2+5*l), A_Fusion_Algorithm(1+5*l) - A_Fusion_Algorithm(3+5*l), A_Fusion_Algorithm(4+5*l) - A_Fusion_Algorithm(2+5*l)];
                        % B = [x y width heigth]
                        
                        area_A = rectint(A,A);
                        area_intersec = rectint(A,B);

                        if (area_intersec/area_A >= 0.5)
%                             A_Fusion_Algorithm(1+5*j) = 0; 
%                             A_Fusion_Algorithm(2+5*j) = 0;
%                             A_Fusion_Algorithm(3+5*j) = 0;
%                             A_Fusion_Algorithm(4+5*j) = 0;
%                             fprintf(fileID_TP,'%d \n',1);
                            intersec_data_fusion(l+1) = intersec_data_fusion(l+1) +1;
                            TP_Fusion_Algorithm = TP_Fusion_Algorithm + 1;
                        end
                    end
                end
            end
            for k=0:(size(bbs_correct_fusion,1) - 1)
%                 if(A_Fusion_Algorithm(1+5*k) ~= 0)
                if(intersec_data_fusion(k+1) == 0)
%                     fprintf(fileID_FP,'%d \n',1);
                    FP_Fusion_Algorithm = FP_Fusion_Algorithm + 1;
                end
            end
        end
    else
        if (size(A_Fusion_Algorithm) == 0)
            % true negative - not found any pedestrians on either images
%             fprintf(fileID_TN,'%d \n',1);
            TN_Fusion_Algorithm = TN_Fusion_Algorithm + 1;
        else
            % false positive - false alarm
%             fprintf(fileID_FP,'%d \n',(size(A_Pedro_Algorithm,1))/5);
            bbs_reshape_fusion = reshape(A_Fusion_Algorithm,[5 size(A_Fusion_Algorithm,1)/5])';
            bbs_correct_fusion = bbNms(bbs_reshape_fusion, 'thr',0.5, 'type','ms');
            
            FP_Fusion_Algorithm = FP_Fusion_Algorithm + (size(bbs_correct_fusion,1));
        end
    end
    
    % Fusion 2 image reset algorithm
    if (size(A_bag_1,1) > 0)
        if (A_Fusion_2imagereset_Algorithm(1) == 0 && A_Fusion_2imagereset_Algorithm(2) == 0)
            % false negative - miss
%             fprintf(fileID_FN,'%d\n',(size(A_bag_1,1)/4));
            FN_Fusion_2imagereset_Algorithm = FN_Fusion_2imagereset_Algorithm + (size(A_bag_1,1)/4);
        else
            % Test to see if BB's correspond, can be true positive or false
            % positive
            bbs_reshape_fusion_2image = reshape(A_Fusion_2imagereset_Algorithm,[5 size(A_Fusion_2imagereset_Algorithm,1)/5])';
            for q=1:size(bbs_reshape_fusion_2image,1)
                bbs_reshape_fusion_2image(q, 5) = 1;
            end
            bbs_correct_fusion_2image = bbNms(bbs_reshape_fusion_2image, 'thr',0.5, 'type','ms');
            
            intersec_data_fusion_2image = zeros(1,size(bbs_correct_fusion_2image,1));
            for j=0:(size(A_bag_1)/4 - 1)
                for l=0:(size(bbs_correct_fusion_2image,1) - 1)
                    if(bbs_correct_fusion_2image(l+1, 1) ~= 0)
                        
                        A = [A_bag_1(1+4*j), A_bag_1(2+4*j), A_bag_1(3+4*j), A_bag_1(4+4*j)];
                        B = [bbs_correct_fusion_2image(l+1, 1), bbs_correct_fusion_2image(l+1, 2), bbs_correct_fusion_2image(l+1, 3), bbs_correct_fusion_2image(l+1, 4)];
                        
                        area_A = rectint(A,A);
                        area_intersec = rectint(A,B);

                        if (area_intersec/area_A >= 0.5)
%                             A_Fusion_2imagereset_Algorithm(1+5*j) = 0; 
%                             A_Fusion_2imagereset_Algorithm(2+5*j) = 0;
%                             A_Fusion_2imagereset_Algorithm(3+5*j) = 0;
%                             A_Fusion_2imagereset_Algorithm(4+5*j) = 0;
%                             fprintf(fileID_TP,'%d \n',1);
                            intersec_data_fusion_2image(l+1) = intersec_data_fusion_2image(l+1) +1;
                            TP_Fusion_2imagereset_Algorithm = TP_Fusion_2imagereset_Algorithm + 1;
                        end
                    end
                end
            end
            for k=0:(size(bbs_correct_fusion_2image,1) - 1)
%                 if(A_Fusion_2imagereset_Algorithm(1+5*k) ~= 0)
                if(intersec_data_fusion_2image(k+1) == 0)
%                     fprintf(fileID_FP,'%d \n',1);
                    FP_Fusion_2imagereset_Algorithm = FP_Fusion_2imagereset_Algorithm + 1;
                end
            end
        end
    else
        if (A_Fusion_2imagereset_Algorithm(1) == 0 && A_Fusion_2imagereset_Algorithm(2) == 0)
            % true negative - not found any pedestrians on either images
%             fprintf(fileID_TN,'%d \n',1);
            TN_Fusion_2imagereset_Algorithm = TN_Fusion_2imagereset_Algorithm + 1;
        else
            % false positive - false alarm
%             fprintf(fileID_FP,'%d \n',(size(A_Pedro_Algorithm,1))/5);
            bbs_reshape_fusion_2image = reshape(A_Fusion_2imagereset_Algorithm,[5 size(A_Fusion_2imagereset_Algorithm,1)/5])';
            bbs_correct_fusion_2image = bbNms(bbs_reshape_fusion_2image, 'thr',0.5, 'type','ms');
            
            FP_Fusion_2imagereset_Algorithm = FP_Fusion_2imagereset_Algorithm + (size(bbs_correct_fusion_2image,1));
        end
    end
    
    % Fusion 4 image reset algorithm
    if (size(A_bag_1,1) > 0)
        if (A_Fusion_4imagereset_Algorithm(1) == 0 && A_Fusion_4imagereset_Algorithm(2) == 0)
            % false negative - miss
%             fprintf(fileID_FN,'%d\n',(size(A_bag_1,1)/4));
            FN_Fusion_4imagereset_Algorithm = FN_Fusion_4imagereset_Algorithm + (size(A_bag_1,1)/4);
        else
            % Test to see if BB's correspond, can be true positive or false
            % positive
            bbs_reshape_fusion_4image = reshape(A_Fusion_4imagereset_Algorithm,[5 size(A_Fusion_4imagereset_Algorithm,1)/5])';
            for q=1:size(bbs_reshape_fusion_4image,1)
                bbs_reshape_fusion_4image(q, 5) = 1;
            end
            bbs_correct_fusion_4image = bbNms(bbs_reshape_fusion_4image, 'thr',0.5, 'type','ms');
            
            intersec_data_fusion_4image = zeros(1,size(bbs_correct_fusion_4image,1));
            for j=0:(size(A_bag_1)/4 - 1)
                for l=0:(size(bbs_correct_fusion_4image,1) - 1)
                    if(bbs_correct_fusion_4image(l+1, 1) ~= 0)
                        
                        A = [A_bag_1(1+4*j), A_bag_1(2+4*j), A_bag_1(3+4*j), A_bag_1(4+4*j)];
                        B = [bbs_correct_fusion_4image(l+1, 1), bbs_correct_fusion_4image(l+1, 2), bbs_correct_fusion_4image(l+1, 3), bbs_correct_fusion_4image(l+1, 4)];
                        
                        area_A = rectint(A,A);
                        area_intersec = rectint(A,B);

                        if (area_intersec/area_A >= 0.5)
%                             A_Fusion_4imagereset_Algorithm(1+5*j) = 0; 
%                             A_Fusion_4imagereset_Algorithm(2+5*j) = 0;
%                             A_Fusion_4imagereset_Algorithm(3+5*j) = 0;
%                             A_Fusion_4imagereset_Algorithm(4+5*j) = 0;
%                             fprintf(fileID_TP,'%d \n',1);
                            intersec_data_fusion_4image(l+1) = intersec_data_fusion_4image(l+1) +1;
                            TP_Fusion_4imagereset_Algorithm = TP_Fusion_4imagereset_Algorithm + 1;
                        end
                    end
                end
            end
            for k=0:(size(bbs_correct_fusion_4image,1) - 1)
%                 if(A_Fusion_4imagereset_Algorithm(1+5*k) ~= 0)
                if(intersec_data_fusion_4image(k+1) == 0)
%                     fprintf(fileID_FP,'%d \n',1);
                    FP_Fusion_4imagereset_Algorithm = FP_Fusion_4imagereset_Algorithm + 1;
                end
            end
        end
    else
        if (A_Fusion_4imagereset_Algorithm(1) == 0 && A_Fusion_4imagereset_Algorithm(2) == 0)
            % true negative - not found any pedestrians on either images
%             fprintf(fileID_TN,'%d \n',1);
            TN_Fusion_4imagereset_Algorithm = TN_Fusion_4imagereset_Algorithm + 1;
        else
            % false positive - false alarm
%             fprintf(fileID_FP,'%d \n',(size(A_Pedro_Algorithm,1))/5);
            bbs_reshape_fusion_4image = reshape(A_Fusion_4imagereset_Algorithm,[5 size(A_Fusion_4imagereset_Algorithm,1)/5])';
            bbs_correct_fusion_4image = bbNms(bbs_reshape_fusion_4image, 'thr',0.5, 'type','ms');
            
            FP_Fusion_4imagereset_Algorithm = FP_Fusion_4imagereset_Algorithm + (size(bbs_correct_fusion_4image,1));
        end
    end
    
    % Fusion 10 image reset algorithm
    if (size(A_bag_1,1) > 0)
        if (A_Fusion_10imagereset_Algorithm(1) == 0 && A_Fusion_10imagereset_Algorithm(2) == 0)
            % false negative - miss
%             fprintf(fileID_FN,'%d\n',(size(A_bag_1,1)/4));
            FN_Fusion_10imagereset_Algorithm = FN_Fusion_10imagereset_Algorithm + (size(A_bag_1,1)/4);
        else
            % Test to see if BB's correspond, can be true positive or false
            % positive
            bbs_reshape_fusion_10image = reshape(A_Fusion_10imagereset_Algorithm,[5 size(A_Fusion_10imagereset_Algorithm,1)/5])';
            for q=1:size(bbs_reshape_fusion_10image,1)
                bbs_reshape_fusion_10image(q, 5) = 1;
            end
            bbs_correct_fusion_10image = bbNms(bbs_reshape_fusion_10image, 'thr',0.5, 'type','ms');
            
            intersec_data_fusion_10image = zeros(1,size(bbs_correct_fusion_10image,1));
            for j=0:(size(A_bag_1)/4 - 1)
                for l=0:(size(bbs_correct_fusion_10image,1) - 1)
                    if(bbs_correct_fusion_10image(l+1, 1) ~= 0)
                        
                        A = [A_bag_1(1+4*j), A_bag_1(2+4*j), A_bag_1(3+4*j), A_bag_1(4+4*j)];
                        B = [bbs_correct_fusion_10image(l+1, 1), bbs_correct_fusion_10image(l+1, 2), bbs_correct_fusion_10image(l+1, 3), bbs_correct_fusion_10image(l+1, 4)];
                        
                        area_A = rectint(A,A);
                        area_intersec = rectint(A,B);

                        if (area_intersec/area_A >= 0.5)
%                             A_Fusion_10imagereset_Algorithm(1+5*j) = 0; 
%                             A_Fusion_10imagereset_Algorithm(2+5*j) = 0;
%                             A_Fusion_10imagereset_Algorithm(3+5*j) = 0;
%                             A_Fusion_10imagereset_Algorithm(4+5*j) = 0;
%                             fprintf(fileID_TP,'%d \n',1);
                            intersec_data_fusion_10image(l+1) = intersec_data_fusion_10image(l+1) +1;
                            TP_Fusion_10imagereset_Algorithm = TP_Fusion_10imagereset_Algorithm + 1;
                        end
                    end
                end
            end
            for k=0:(size(bbs_correct_fusion_10image,1) - 1)
%                 if(A_Fusion_10imagereset_Algorithm(1+5*k) ~= 0)
                if(intersec_data_fusion_10image(k+1) == 0)
%                     fprintf(fileID_FP,'%d \n',1);
                    FP_Fusion_10imagereset_Algorithm = FP_Fusion_10imagereset_Algorithm + 1;
                end
            end
        end
    else
        if (A_Fusion_10imagereset_Algorithm(1) == 0 && A_Fusion_10imagereset_Algorithm(2) == 0)
            % true negative - not found any pedestrians on either images
%             fprintf(fileID_TN,'%d \n',1);
            TN_Fusion_10imagereset_Algorithm = TN_Fusion_10imagereset_Algorithm + 1;
        else
            % false positive - false alarm
%             fprintf(fileID_FP,'%d \n',(size(A_Pedro_Algorithm,1))/5);
            bbs_reshape_fusion_10image = reshape(A_Fusion_10imagereset_Algorithm,[5 size(A_Fusion_10imagereset_Algorithm,1)/5])';
            bbs_correct_fusion_10image = bbNms(bbs_reshape_fusion_10image, 'thr',0.5, 'type','ms');
            
            FP_Fusion_10imagereset_Algorithm = FP_Fusion_10imagereset_Algorithm + (size(bbs_correct_fusion_10image,1));
        end
    end
    
    % Fusion 2 image reset 1.3m max width algorithm
    if (size(A_bag_1,1) > 0)
        if (A_Fusion_2imagereset_1p3_Algorithm(1) == 0 && A_Fusion_2imagereset_1p3_Algorithm(2) == 0)
            % false negative - miss
%             fprintf(fileID_FN,'%d\n',(size(A_bag_1,1)/4));
            FN_Fusion_2imagereset_1p3_Algorithm = FN_Fusion_2imagereset_1p3_Algorithm + (size(A_bag_1,1)/4);
        else
            % Test to see if BB's correspond, can be true positive or false
            % positive
            bbs_reshape_fusion_2image_1p3 = reshape(A_Fusion_2imagereset_1p3_Algorithm,[5 size(A_Fusion_2imagereset_1p3_Algorithm,1)/5])';
            for q=1:size(bbs_reshape_fusion_2image_1p3,1)
                bbs_reshape_fusion_2image_1p3(q, 5) = 1;
            end
            bbs_correct_fusion_2image_1p3 = bbNms(bbs_reshape_fusion_2image_1p3, 'thr',0.5, 'type','ms');
            
            intersec_data_fusion_2image_1p3 = zeros(1,size(bbs_correct_fusion_2image_1p3,1));
            for j=0:(size(A_bag_1)/4 - 1)
                for l=0:(size(bbs_correct_fusion_2image_1p3,1) - 1)
                    if(bbs_correct_fusion_2image_1p3(l+1, 1) ~= 0)
                        
                        A = [A_bag_1(1+4*j), A_bag_1(2+4*j), A_bag_1(3+4*j), A_bag_1(4+4*j)];
                        B = [bbs_correct_fusion_2image_1p3(l+1, 1), bbs_correct_fusion_2image_1p3(l+1, 2), bbs_correct_fusion_2image_1p3(l+1, 3), bbs_correct_fusion_2image_1p3(l+1, 4)];
                        
                        area_A = rectint(A,A);
                        area_intersec = rectint(A,B);

                        if (area_intersec/area_A >= 0.5)
%                             A_Fusion_2imagereset_1p3_Algorithm(1+5*j) = 0; 
%                             A_Fusion_2imagereset_1p3_Algorithm(2+5*j) = 0;
%                             A_Fusion_2imagereset_1p3_Algorithm(3+5*j) = 0;
%                             A_Fusion_2imagereset_1p3_Algorithm(4+5*j) = 0;
%                             fprintf(fileID_TP,'%d \n',1);
                            intersec_data_fusion_2image_1p3(l+1) = intersec_data_fusion_2image_1p3(l+1) +1;
                            TP_Fusion_2imagereset_1p3_Algorithm = TP_Fusion_2imagereset_1p3_Algorithm + 1;
                        end
                    end
                end
            end
            for k=0:(size(bbs_correct_fusion_2image_1p3,1) - 1)
%                 if(A_Fusion_2imagereset_1p3_Algorithm(1+5*k) ~= 0)
                if(intersec_data_fusion_2image_1p3(k+1) == 0)
%                     fprintf(fileID_FP,'%d \n',1);
                    FP_Fusion_2imagereset_1p3_Algorithm = FP_Fusion_2imagereset_1p3_Algorithm + 1;
                end
            end
        end
    else
        if (A_Fusion_2imagereset_1p3_Algorithm(1) == 0 && A_Fusion_2imagereset_1p3_Algorithm(2) == 0)
            % true negative - not found any pedestrians on either images
%             fprintf(fileID_TN,'%d \n',1);
            TN_Fusion_2imagereset_1p3_Algorithm = TN_Fusion_2imagereset_1p3_Algorithm + 1;
        else
            % false positive - false alarm
%             fprintf(fileID_FP,'%d \n',(size(A_Pedro_Algorithm,1))/5);
            bbs_reshape_fusion_2image_1p3 = reshape(A_Fusion_2imagereset_1p3_Algorithm,[5 size(A_Fusion_2imagereset_1p3_Algorithm,1)/5])';
            bbs_correct_fusion_2image_1p3 = bbNms(bbs_reshape_fusion_2image_1p3, 'thr',0.5, 'type','ms');
            
            FP_Fusion_2imagereset_1p3_Algorithm = FP_Fusion_2imagereset_1p3_Algorithm + (size(bbs_correct_fusion_2image_1p3,1));
        end
    end
    
    % Fusion 4 image reset 1.3m max width algorithm
    if (size(A_bag_1,1) > 0)
        if (A_Fusion_4imagereset_1p3_Algorithm(1) == 0 && A_Fusion_4imagereset_1p3_Algorithm(2) == 0)
            % false negative - miss
%             fprintf(fileID_FN,'%d\n',(size(A_bag_1,1)/4));
            FN_Fusion_4imagereset_1p3_Algorithm = FN_Fusion_4imagereset_1p3_Algorithm + (size(A_bag_1,1)/4);
        else
            % Test to see if BB's correspond, can be true positive or false
            % positive
            bbs_reshape_fusion_4image_1p3 = reshape(A_Fusion_4imagereset_1p3_Algorithm,[5 size(A_Fusion_4imagereset_1p3_Algorithm,1)/5])';
            for q=1:size(bbs_reshape_fusion_4image_1p3,1)
                bbs_reshape_fusion_4image_1p3(q, 5) = 1;
            end
            bbs_correct_fusion_4image_1p3 = bbNms(bbs_reshape_fusion_4image_1p3, 'thr',0.5, 'type','ms');
            
            intersec_data_fusion_4image_1p3 = zeros(1,size(bbs_correct_fusion_4image_1p3,1));
            for j=0:(size(A_bag_1)/4 - 1)
                for l=0:(size(bbs_correct_fusion_4image_1p3,1) - 1)
                    if(bbs_correct_fusion_4image_1p3(l+1, 1) ~= 0)
                        
                        A = [A_bag_1(1+4*j), A_bag_1(2+4*j), A_bag_1(3+4*j), A_bag_1(4+4*j)];
                        B = [bbs_correct_fusion_4image_1p3(l+1, 1), bbs_correct_fusion_4image_1p3(l+1, 2), bbs_correct_fusion_4image_1p3(l+1, 3), bbs_correct_fusion_4image_1p3(l+1, 4)];
                        
                        area_A = rectint(A,A);
                        area_intersec = rectint(A,B);

                        if (area_intersec/area_A >= 0.5)
%                             A_Fusion_4imagereset_1p3_Algorithm(1+5*j) = 0; 
%                             A_Fusion_4imagereset_1p3_Algorithm(2+5*j) = 0;
%                             A_Fusion_4imagereset_1p3_Algorithm(3+5*j) = 0;
%                             A_Fusion_4imagereset_1p3_Algorithm(4+5*j) = 0;
%                             fprintf(fileID_TP,'%d \n',1);
                            intersec_data_fusion_4image_1p3(l+1) = intersec_data_fusion_4image_1p3(l+1) +1;
                            TP_Fusion_4imagereset_1p3_Algorithm = TP_Fusion_4imagereset_1p3_Algorithm + 1;
                        end
                    end
                end
            end
            for k=0:(size(bbs_correct_fusion_4image_1p3,1) - 1)
%                 if(A_Fusion_4imagereset_1p3_Algorithm(1+5*k) ~= 0)
                if(intersec_data_fusion_4image_1p3(k+1) == 0)
%                     fprintf(fileID_FP,'%d \n',1);
                    FP_Fusion_4imagereset_1p3_Algorithm = FP_Fusion_4imagereset_1p3_Algorithm + 1;
                end
            end
        end
    else
        if (A_Fusion_4imagereset_1p3_Algorithm(1) == 0 && A_Fusion_4imagereset_1p3_Algorithm(2) == 0)
            % true negative - not found any pedestrians on either images
%             fprintf(fileID_TN,'%d \n',1);
            TN_Fusion_4imagereset_1p3_Algorithm = TN_Fusion_4imagereset_1p3_Algorithm + 1;
        else
            % false positive - false alarm
%             fprintf(fileID_FP,'%d \n',(size(A_Pedro_Algorithm,1))/5);
            bbs_reshape_fusion_4image_1p3 = reshape(A_Fusion_4imagereset_1p3_Algorithm,[5 size(A_Fusion_4imagereset_1p3_Algorithm,1)/5])';
            bbs_correct_fusion_4image_1p3 = bbNms(bbs_reshape_fusion_4image_1p3, 'thr',0.5, 'type','ms');
            
            FP_Fusion_4imagereset_1p3_Algorithm = FP_Fusion_4imagereset_1p3_Algorithm + (size(bbs_correct_fusion_4image_1p3,1));
        end
    end
    
    % Fusion 6 image reset 1.3m max width algorithm
    if (size(A_bag_1,1) > 0)
        if (A_Fusion_6imagereset_1p3_Algorithm(1) == 0 && A_Fusion_6imagereset_1p3_Algorithm(2) == 0)
            % false negative - miss
%             fprintf(fileID_FN,'%d\n',(size(A_bag_1,1)/4));
            FN_Fusion_6imagereset_1p3_Algorithm = FN_Fusion_6imagereset_1p3_Algorithm + (size(A_bag_1,1)/4);
        else
            % Test to see if BB's correspond, can be true positive or false
            % positive
            bbs_reshape_fusion_6image_1p3 = reshape(A_Fusion_6imagereset_1p3_Algorithm,[5 size(A_Fusion_6imagereset_1p3_Algorithm,1)/5])';
            for q=1:size(bbs_reshape_fusion_6image_1p3,1)
                bbs_reshape_fusion_6image_1p3(q, 5) = 1;
            end
            bbs_correct_fusion_6image_1p3 = bbNms(bbs_reshape_fusion_6image_1p3, 'thr',0.5, 'type','ms');
            
            intersec_data_fusion_6image_1p3 = zeros(1,size(bbs_correct_fusion_6image_1p3,1));
            for j=0:(size(A_bag_1)/4 - 1)
                for l=0:(size(bbs_correct_fusion_6image_1p3,1) - 1)
                    if(bbs_correct_fusion_6image_1p3(l+1, 1) ~= 0)
                        
                        A = [A_bag_1(1+4*j), A_bag_1(2+4*j), A_bag_1(3+4*j), A_bag_1(4+4*j)];
                        B = [bbs_correct_fusion_6image_1p3(l+1, 1), bbs_correct_fusion_6image_1p3(l+1, 2), bbs_correct_fusion_6image_1p3(l+1, 3), bbs_correct_fusion_6image_1p3(l+1, 4)];
                        
                        area_A = rectint(A,A);
                        area_intersec = rectint(A,B);

                        if (area_intersec/area_A >= 0.5)
%                             A_Fusion_6imagereset_1p3_Algorithm(1+5*j) = 0; 
%                             A_Fusion_6imagereset_1p3_Algorithm(2+5*j) = 0;
%                             A_Fusion_6imagereset_1p3_Algorithm(3+5*j) = 0;
%                             A_Fusion_6imagereset_1p3_Algorithm(4+5*j) = 0;
%                             fprintf(fileID_TP,'%d \n',1);
                            intersec_data_fusion_6image_1p3(l+1) = intersec_data_fusion_6image_1p3(l+1) +1;
                            TP_Fusion_6imagereset_1p3_Algorithm = TP_Fusion_6imagereset_1p3_Algorithm + 1;
                        end
                    end
                end
            end
            for k=0:(size(bbs_correct_fusion_6image_1p3,1) - 1)
%                 if(A_Fusion_6imagereset_1p3_Algorithm(1+5*k) ~= 0)
                if(intersec_data_fusion_6image_1p3(k+1) == 0)
%                     fprintf(fileID_FP,'%d \n',1);
                    FP_Fusion_6imagereset_1p3_Algorithm = FP_Fusion_6imagereset_1p3_Algorithm + 1;
                end
            end
        end
    else
        if (A_Fusion_6imagereset_1p3_Algorithm(1) == 0 && A_Fusion_6imagereset_1p3_Algorithm(2) == 0)
            % true negative - not found any pedestrians on either images
%             fprintf(fileID_TN,'%d \n',1);
            TN_Fusion_6imagereset_1p3_Algorithm = TN_Fusion_6imagereset_1p3_Algorithm + 1;
        else
            % false positive - false alarm
%             fprintf(fileID_FP,'%d \n',(size(A_Pedro_Algorithm,1))/5);
            bbs_reshape_fusion_6image_1p3 = reshape(A_Fusion_6imagereset_1p3_Algorithm,[5 size(A_Fusion_6imagereset_1p3_Algorithm,1)/5])';
            bbs_correct_fusion_6image_1p3 = bbNms(bbs_reshape_fusion_6image_1p3, 'thr',0.5, 'type','ms');
            
            FP_Fusion_6imagereset_1p3_Algorithm = FP_Fusion_6imagereset_1p3_Algorithm + (size(bbs_correct_fusion_6image_1p3,1));
        end
    end
    
    % Fusion 8 image reset 1.3m max width algorithm
    if (size(A_bag_1,1) > 0)
        if (A_Fusion_8imagereset_1p3_Algorithm(1) == 0 && A_Fusion_8imagereset_1p3_Algorithm(2) == 0)
            % false negative - miss
%             fprintf(fileID_FN,'%d\n',(size(A_bag_1,1)/4));
            FN_Fusion_8imagereset_1p3_Algorithm = FN_Fusion_8imagereset_1p3_Algorithm + (size(A_bag_1,1)/4);
        else
            % Test to see if BB's correspond, can be true positive or false
            % positive
            bbs_reshape_fusion_8image_1p3 = reshape(A_Fusion_8imagereset_1p3_Algorithm,[5 size(A_Fusion_8imagereset_1p3_Algorithm,1)/5])';
            for q=1:size(bbs_reshape_fusion_8image_1p3,1)
                bbs_reshape_fusion_8image_1p3(q, 5) = 1;
            end
            bbs_correct_fusion_8image_1p3 = bbNms(bbs_reshape_fusion_8image_1p3, 'thr',0.5, 'type','ms');
            
            intersec_data_fusion_8image_1p3 = zeros(1,size(bbs_correct_fusion_8image_1p3,1));
            for j=0:(size(A_bag_1)/4 - 1)
                for l=0:(size(bbs_correct_fusion_8image_1p3,1) - 1)
                    if(bbs_correct_fusion_8image_1p3(l+1, 1) ~= 0)
                        
                        A = [A_bag_1(1+4*j), A_bag_1(2+4*j), A_bag_1(3+4*j), A_bag_1(4+4*j)];
                        B = [bbs_correct_fusion_8image_1p3(l+1, 1), bbs_correct_fusion_8image_1p3(l+1, 2), bbs_correct_fusion_8image_1p3(l+1, 3), bbs_correct_fusion_8image_1p3(l+1, 4)];
                        
                        area_A = rectint(A,A);
                        area_intersec = rectint(A,B);

                        if (area_intersec/area_A >= 0.5)
%                             A_Fusion_6imagereset_1p3_Algorithm(1+5*j) = 0; 
%                             A_Fusion_6imagereset_1p3_Algorithm(2+5*j) = 0;
%                             A_Fusion_6imagereset_1p3_Algorithm(3+5*j) = 0;
%                             A_Fusion_6imagereset_1p3_Algorithm(4+5*j) = 0;
%                             fprintf(fileID_TP,'%d \n',1);
                            intersec_data_fusion_8image_1p3(l+1) = intersec_data_fusion_8image_1p3(l+1) +1;
                            TP_Fusion_8imagereset_1p3_Algorithm = TP_Fusion_8imagereset_1p3_Algorithm + 1;
                        end
                    end
                end
            end
            for k=0:(size(bbs_correct_fusion_8image_1p3,1) - 1)
%                 if(A_Fusion_6imagereset_1p3_Algorithm(1+5*k) ~= 0)
                if(intersec_data_fusion_8image_1p3(k+1) == 0)
%                     fprintf(fileID_FP,'%d \n',1);
                    FP_Fusion_8imagereset_1p3_Algorithm = FP_Fusion_8imagereset_1p3_Algorithm + 1;
                end
            end
        end
    else
        if (A_Fusion_8imagereset_1p3_Algorithm(1) == 0 && A_Fusion_8imagereset_1p3_Algorithm(2) == 0)
            % true negative - not found any pedestrians on either images
%             fprintf(fileID_TN,'%d \n',1);
            TN_Fusion_8imagereset_1p3_Algorithm = TN_Fusion_8imagereset_1p3_Algorithm + 1;
        else
            % false positive - false alarm
%             fprintf(fileID_FP,'%d \n',(size(A_Pedro_Algorithm,1))/5);
            bbs_reshape_fusion_8image_1p3 = reshape(A_Fusion_8imagereset_1p3_Algorithm,[5 size(A_Fusion_8imagereset_1p3_Algorithm,1)/5])';
            bbs_correct_fusion_8image_1p3 = bbNms(bbs_reshape_fusion_8image_1p3, 'thr',0.5, 'type','ms');
            
            FP_Fusion_8imagereset_1p3_Algorithm = FP_Fusion_8imagereset_1p3_Algorithm + (size(bbs_correct_fusion_8image_1p3,1));
        end
    end
    
    % Fusion 10 image reset 1.3m max width algorithm
    if (size(A_bag_1,1) > 0)
        if (A_Fusion_10imagereset_1p3_Algorithm(1) == 0 && A_Fusion_10imagereset_1p3_Algorithm(2) == 0)
            % false negative - miss
%             fprintf(fileID_FN,'%d\n',(size(A_bag_1,1)/4));
            FN_Fusion_10imagereset_1p3_Algorithm = FN_Fusion_10imagereset_1p3_Algorithm + (size(A_bag_1,1)/4);
        else
            % Test to see if BB's correspond, can be true positive or false
            % positive
            bbs_reshape_fusion_10image_1p3 = reshape(A_Fusion_10imagereset_1p3_Algorithm,[5 size(A_Fusion_10imagereset_1p3_Algorithm,1)/5])';
            for q=1:size(bbs_reshape_fusion_10image_1p3,1)
                bbs_reshape_fusion_10image_1p3(q, 5) = 1;
            end
            bbs_correct_fusion_10image_1p3 = bbNms(bbs_reshape_fusion_10image_1p3, 'thr',0.5, 'type','ms');
            
            intersec_data_fusion_10image_1p3 = zeros(1,size(bbs_correct_fusion_10image_1p3,1));
            for j=0:(size(A_bag_1)/4 - 1)
                for l=0:(size(bbs_correct_fusion_10image_1p3,1) - 1)
                    if(bbs_correct_fusion_10image_1p3(l+1, 1) ~= 0)
                        
                        A = [A_bag_1(1+4*j), A_bag_1(2+4*j), A_bag_1(3+4*j), A_bag_1(4+4*j)];
                        B = [bbs_correct_fusion_10image_1p3(l+1, 1), bbs_correct_fusion_10image_1p3(l+1, 2), bbs_correct_fusion_10image_1p3(l+1, 3), bbs_correct_fusion_10image_1p3(l+1, 4)];
                        
                        area_A = rectint(A,A);
                        area_intersec = rectint(A,B);

                        if (area_intersec/area_A >= 0.5)
%                             A_Fusion_10imagereset_1p3_Algorithm(1+5*j) = 0; 
%                             A_Fusion_10imagereset_1p3_Algorithm(2+5*j) = 0;
%                             A_Fusion_10imagereset_1p3_Algorithm(3+5*j) = 0;
%                             A_Fusion_10imagereset_1p3_Algorithm(4+5*j) = 0;
%                             fprintf(fileID_TP,'%d \n',1);
                            intersec_data_fusion_10image_1p3(l+1) = intersec_data_fusion_10image_1p3(l+1) +1;
                            TP_Fusion_10imagereset_1p3_Algorithm = TP_Fusion_10imagereset_1p3_Algorithm + 1;
                        end
                    end
                end
            end
            for k=0:(size(bbs_correct_fusion_10image_1p3,1) - 1)
%                 if(A_Fusion_6imagereset_1p3_Algorithm(1+5*k) ~= 0)
                if(intersec_data_fusion_10image_1p3(k+1) == 0)
%                     fprintf(fileID_FP,'%d \n',1);
                    FP_Fusion_10imagereset_1p3_Algorithm = FP_Fusion_10imagereset_1p3_Algorithm + 1;
                end
            end
        end
    else
        if (A_Fusion_10imagereset_1p3_Algorithm(1) == 0 && A_Fusion_10imagereset_1p3_Algorithm(2) == 0)
            % true negative - not found any pedestrians on either images
%             fprintf(fileID_TN,'%d \n',1);
            TN_Fusion_10imagereset_1p3_Algorithm = TN_Fusion_10imagereset_1p3_Algorithm + 1;
        else
            % false positive - false alarm
%             fprintf(fileID_FP,'%d \n',(size(A_Pedro_Algorithm,1))/5);
            bbs_reshape_fusion_10image_1p3 = reshape(A_Fusion_10imagereset_1p3_Algorithm,[5 size(A_Fusion_10imagereset_1p3_Algorithm,1)/5])';
            bbs_correct_fusion_10image_1p3 = bbNms(bbs_reshape_fusion_10image_1p3, 'thr',0.5, 'type','ms');
            FP_Fusion_10imagereset_1p3_Algorithm = FP_Fusion_10imagereset_1p3_Algorithm + (size(bbs_correct_fusion_10image_1p3,1));
        end
    end
    
    % Fusion 20 image reset 1.3m max width algorithm
    if (size(A_bag_1,1) > 0)
        if (A_Fusion_20imagereset_1p3_Algorithm(1) == 0 && A_Fusion_20imagereset_1p3_Algorithm(2) == 0)
            % false negative - miss
%             fprintf(fileID_FN,'%d\n',(size(A_bag_1,1)/4));
            FN_Fusion_20imagereset_1p3_Algorithm = FN_Fusion_20imagereset_1p3_Algorithm + (size(A_bag_1,1)/4);
        else
            % Test to see if BB's correspond, can be true positive or false
            % positive
            bbs_reshape_fusion_20image_1p3 = reshape(A_Fusion_20imagereset_1p3_Algorithm,[5 size(A_Fusion_20imagereset_1p3_Algorithm,1)/5])';
            for q=1:size(bbs_reshape_fusion_20image_1p3,1)
                bbs_reshape_fusion_20image_1p3(q, 5) = 1;
            end
            bbs_correct_fusion_20image_1p3 = bbNms(bbs_reshape_fusion_20image_1p3, 'thr',0.5, 'type','ms');
            
            intersec_data_fusion_20image_1p3 = zeros(1,size(bbs_correct_fusion_20image_1p3,1));
            for j=0:(size(A_bag_1)/4 - 1)
                for l=0:(size(bbs_correct_fusion_20image_1p3,1) - 1)
                    if(bbs_correct_fusion_20image_1p3(l+1, 1) ~= 0)
                        
                        A = [A_bag_1(1+4*j), A_bag_1(2+4*j), A_bag_1(3+4*j), A_bag_1(4+4*j)];
                        B = [bbs_correct_fusion_20image_1p3(l+1, 1), bbs_correct_fusion_20image_1p3(l+1, 2), bbs_correct_fusion_20image_1p3(l+1, 3), bbs_correct_fusion_20image_1p3(l+1, 4)];
                        
                        area_A = rectint(A,A);
                        area_intersec = rectint(A,B);

                        if (area_intersec/area_A >= 0.5)
%                             A_Fusion_20imagereset_1p3_Algorithm(1+5*j) = 0; 
%                             A_Fusion_20imagereset_1p3_Algorithm(2+5*j) = 0;
%                             A_Fusion_20imagereset_1p3_Algorithm(3+5*j) = 0;
%                             A_Fusion_20imagereset_1p3_Algorithm(4+5*j) = 0;
%                             fprintf(fileID_TP,'%d \n',1);
                            intersec_data_fusion_20image_1p3(l+1) = intersec_data_fusion_20image_1p3(l+1) +1;
                            TP_Fusion_20imagereset_1p3_Algorithm = TP_Fusion_20imagereset_1p3_Algorithm + 1;
                        end
                    end
                end
            end
            for k=0:(size(bbs_correct_fusion_20image_1p3,1) - 1)
%                 if(A_Fusion_6imagereset_1p3_Algorithm(1+5*k) ~= 0)
                if(intersec_data_fusion_20image_1p3(k+1) == 0)
%                     fprintf(fileID_FP,'%d \n',1);
                    FP_Fusion_20imagereset_1p3_Algorithm = FP_Fusion_20imagereset_1p3_Algorithm + 1;
                end
            end
        end
    else
        if (A_Fusion_20imagereset_1p3_Algorithm(1) == 0 && A_Fusion_20imagereset_1p3_Algorithm(2) == 0)
            % true negative - not found any pedestrians on either images
%             fprintf(fileID_TN,'%d \n',1);
            TN_Fusion_20imagereset_1p3_Algorithm = TN_Fusion_20imagereset_1p3_Algorithm + 1;
        else
            % false positive - false alarm
%             fprintf(fileID_FP,'%d \n',(size(A_Pedro_Algorithm,1))/5);
            bbs_reshape_fusion_20image_1p3 = reshape(A_Fusion_20imagereset_1p3_Algorithm,[5 size(A_Fusion_20imagereset_1p3_Algorithm,1)/5])';
            bbs_correct_fusion_20image_1p3 = bbNms(bbs_reshape_fusion_20image_1p3, 'thr',0.5, 'type','ms');
            FP_Fusion_20imagereset_1p3_Algorithm = FP_Fusion_20imagereset_1p3_Algorithm + (size(bbs_correct_fusion_20image_1p3,1));
        end
    end
    
    % Fusion inf image 1.3m max width algorithm
    if (size(A_bag_1,1) > 0)
        if (A_Fusion_infimagereset_1p3_Algorithm(1) == 0 && A_Fusion_infimagereset_1p3_Algorithm(2) == 0)
            % false negative - miss
%             fprintf(fileID_FN,'%d\n',(size(A_bag_1,1)/4));
            FN_Fusion_infimagereset_1p3_Algorithm = FN_Fusion_infimagereset_1p3_Algorithm + (size(A_bag_1,1)/4);
        else
            % Test to see if BB's correspond, can be true positive or false
            % positive
            bbs_reshape_fusion_infimage_1p3 = reshape(A_Fusion_infimagereset_1p3_Algorithm,[5 size(A_Fusion_infimagereset_1p3_Algorithm,1)/5])';
           
            for q=1:size(bbs_reshape_fusion_infimage_1p3,1)
                bbs_reshape_fusion_infimage_1p3(q, 5) = 1;
            end
            bbs_correct_fusion_infimage_1p3 = bbNms(bbs_reshape_fusion_infimage_1p3, 'thr',0.5, 'type','ms');
            
            intersec_data_fusion_infimage_1p3 = zeros(1,size(bbs_correct_fusion_infimage_1p3,1));
            for j=0:(size(A_bag_1)/4 - 1)
                for l=0:(size(bbs_correct_fusion_infimage_1p3,1) - 1)
                    if(bbs_correct_fusion_infimage_1p3(l+1, 1) ~= 0)
                        
                        A = [A_bag_1(1+4*j), A_bag_1(2+4*j), A_bag_1(3+4*j), A_bag_1(4+4*j)];
                        B = [bbs_correct_fusion_infimage_1p3(l+1, 1), bbs_correct_fusion_infimage_1p3(l+1, 2), bbs_correct_fusion_infimage_1p3(l+1, 3), bbs_correct_fusion_infimage_1p3(l+1, 4)];
                        
                        area_A = rectint(A,A);
                        area_intersec = rectint(A,B);

                        if (area_intersec/area_A >= 0.5)
%                             A_Fusion_infimagereset_1p3_Algorithm(1+5*j) = 0; 
%                             A_Fusion_infimagereset_1p3_Algorithm(2+5*j) = 0;
%                             A_Fusion_infimagereset_1p3_Algorithm(3+5*j) = 0;
%                             A_Fusion_infimagereset_1p3_Algorithm(4+5*j) = 0;
%                             fprintf(fileID_TP,'%d \n',1);
                            intersec_data_fusion_infimage_1p3(l+1) = intersec_data_fusion_infimage_1p3(l+1) +1;
                            TP_Fusion_infimagereset_1p3_Algorithm = TP_Fusion_infimagereset_1p3_Algorithm + 1;
                        end
                    end
                end
            end
            for k=0:(size(bbs_correct_fusion_infimage_1p3,1) - 1)
%                 if(A_Fusion_6imagereset_1p3_Algorithm(1+5*k) ~= 0)
                if(intersec_data_fusion_infimage_1p3(k+1) == 0)
%                     fprintf(fileID_FP,'%d \n',1);
                    FP_Fusion_infimagereset_1p3_Algorithm = FP_Fusion_infimagereset_1p3_Algorithm + 1;
                end
            end
        end
    else
        if (A_Fusion_infimagereset_1p3_Algorithm(1) == 0 && A_Fusion_infimagereset_1p3_Algorithm(2) == 0)
            % true negative - not found any pedestrians on either images
%             fprintf(fileID_TN,'%d \n',1);
            TN_Fusion_infimagereset_1p3_Algorithm = TN_Fusion_infimagereset_1p3_Algorithm + 1;
        else
            % false positive - false alarm
%             fprintf(fileID_FP,'%d \n',(size(A_Pedro_Algorithm,1))/5);
            bbs_reshape_fusion_infimage_1p3 = reshape(A_Fusion_infimagereset_1p3_Algorithm,[5 size(A_Fusion_infimagereset_1p3_Algorithm,1)/5])';
            bbs_correct_fusion_infimage_1p3 = bbNms(bbs_reshape_fusion_infimage_1p3, 'thr',0.5, 'type','ms');
            FP_Fusion_infimagereset_1p3_Algorithm = FP_Fusion_infimagereset_1p3_Algorithm + (size(bbs_correct_fusion_infimage_1p3,1));
        end
    end
    
    if (size(A_Pedro_Algorithm,1) == 9)
        A_Pedro_Algorithm(9) = A_Pedro_Algorithm(5);
    end
    
    hist_Pedro_Algorithm = [hist_Pedro_Algorithm A_Pedro_Algorithm(end,1)];
    
    hist_Fusion_2imagereset_Algorithm = [hist_Fusion_2imagereset_Algorithm A_Fusion_2imagereset_Algorithm(end,1)];
    
    hist_Fusion_4imagereset_Algorithm = [hist_Fusion_4imagereset_Algorithm A_Fusion_4imagereset_Algorithm(end,1)];
    
    hist_Fusion_10imagereset_Algorithm = [hist_Fusion_10imagereset_Algorithm A_Fusion_10imagereset_Algorithm(end,1)];
    
    hist_Fusion_2imagereset_1p3_Algorithm = [hist_Fusion_2imagereset_1p3_Algorithm A_Fusion_2imagereset_1p3_Algorithm(end,1)];
    
    hist_Fusion_4imagereset_1p3_Algorithm = [hist_Fusion_4imagereset_1p3_Algorithm A_Fusion_4imagereset_1p3_Algorithm(end,1)];
    
    hist_Fusion_6imagereset_1p3_Algorithm = [hist_Fusion_6imagereset_1p3_Algorithm A_Fusion_6imagereset_1p3_Algorithm(end,1)];
    
    hist_Fusion_8imagereset_1p3_Algorithm = [hist_Fusion_8imagereset_1p3_Algorithm A_Fusion_8imagereset_1p3_Algorithm(end,1)];
    
    hist_Fusion_10imagereset_1p3_Algorithm = [hist_Fusion_10imagereset_1p3_Algorithm A_Fusion_10imagereset_1p3_Algorithm(end,1)];
    
    hist_Fusion_20imagereset_1p3_Algorithm = [hist_Fusion_20imagereset_1p3_Algorithm A_Fusion_20imagereset_1p3_Algorithm(end,1)];
    
    hist_Fusion_infimagereset_1p3_Algorithm = [hist_Fusion_infimagereset_1p3_Algorithm A_Fusion_infimagereset_1p3_Algorithm(end,1)];
    
    num_BB_positive = num_BB_positive + size(A_bag_1,1)/4;
    
    if (size(A_bag_1,1)/4 == 0)
        num_BB_negative = num_BB_negative + 1;
    end
    
    fclose('all');
%     fclose(fileID_bag_1);
%     fclose(fileID_Pedro_Algorithm);
%     fclose(fileID_Fusion_Algorithm);
%     fclose(fileID_Fusion_2imagereset_Algorithm);
%     fclose(fileID_Fusion_4imagereset_Algorithm);
%     fclose(fileID_Fusion_10imagereset_Algorithm);
%     fclose(fileID_Fusion_2imagereset_1p3_Algorithm);
%     fclose(fileID_TP);
%     fclose(fileID_TN);
%     fclose(fileID_FP);
%     fclose(fileID_FN);

end


TP_ATLASCAR_Pedro_Algorithm = 0;
TN_ATLASCAR_Pedro_Algorithm = 0;
FP_ATLASCAR_Pedro_Algorithm = 0;
FN_ATLASCAR_Pedro_Algorithm = 0;
hist_ATLASCAR_Pedro_Algorithm = [];

TP_ATLASCAR_Fusion_2imagereset_1p3_Algorithm = 0;
TN_ATLASCAR_Fusion_2imagereset_1p3_Algorithm = 0;
FP_ATLASCAR_Fusion_2imagereset_1p3_Algorithm = 0;
FN_ATLASCAR_Fusion_2imagereset_1p3_Algorithm = 0;
hist_ATLASCAR_Fusion_2imagereset_1p3_Algorithm = [];

TP_ATLASCAR_Fusion_4imagereset_1p3_Algorithm = 0;
TN_ATLASCAR_Fusion_4imagereset_1p3_Algorithm = 0;
FP_ATLASCAR_Fusion_4imagereset_1p3_Algorithm = 0;
FN_ATLASCAR_Fusion_4imagereset_1p3_Algorithm = 0;
hist_ATLASCAR_Fusion_4imagereset_1p3_Algorithm = [];

TP_ATLASCAR_Fusion_6imagereset_1p3_Algorithm = 0;
TN_ATLASCAR_Fusion_6imagereset_1p3_Algorithm = 0;
FP_ATLASCAR_Fusion_6imagereset_1p3_Algorithm = 0;
FN_ATLASCAR_Fusion_6imagereset_1p3_Algorithm = 0;
hist_ATLASCAR_Fusion_6imagereset_1p3_Algorithm = [];

TP_ATLASCAR_Fusion_8imagereset_1p3_Algorithm = 0;
TN_ATLASCAR_Fusion_8imagereset_1p3_Algorithm = 0;
FP_ATLASCAR_Fusion_8imagereset_1p3_Algorithm = 0;
FN_ATLASCAR_Fusion_8imagereset_1p3_Algorithm = 0;
hist_ATLASCAR_Fusion_8imagereset_1p3_Algorithm = [];

TP_ATLASCAR_Fusion_10imagereset_1p3_Algorithm = 0;
TN_ATLASCAR_Fusion_10imagereset_1p3_Algorithm = 0;
FP_ATLASCAR_Fusion_10imagereset_1p3_Algorithm = 0;
FN_ATLASCAR_Fusion_10imagereset_1p3_Algorithm = 0;
hist_ATLASCAR_Fusion_10imagereset_1p3_Algorithm = [];

TP_ATLASCAR_Fusion_20imagereset_1p3_Algorithm = 0;
TN_ATLASCAR_Fusion_20imagereset_1p3_Algorithm = 0;
FP_ATLASCAR_Fusion_20imagereset_1p3_Algorithm = 0;
FN_ATLASCAR_Fusion_20imagereset_1p3_Algorithm = 0;
hist_ATLASCAR_Fusion_20imagereset_1p3_Algorithm = [];

TP_ATLASCAR_Fusion_40imagereset_1p3_Algorithm = 0;
TN_ATLASCAR_Fusion_40imagereset_1p3_Algorithm = 0;
FP_ATLASCAR_Fusion_40imagereset_1p3_Algorithm = 0;
FN_ATLASCAR_Fusion_40imagereset_1p3_Algorithm = 0;
hist_ATLASCAR_Fusion_40imagereset_1p3_Algorithm = [];

TP_ATLASCAR_Fusion_infimagereset_1p3_Algorithm = 0;
TN_ATLASCAR_Fusion_infimagereset_1p3_Algorithm = 0;
FP_ATLASCAR_Fusion_infimagereset_1p3_Algorithm = 0;
FN_ATLASCAR_Fusion_infimagereset_1p3_Algorithm = 0;
hist_ATLASCAR_Fusion_infimagereset_1p3_Algorithm = [];

num_BB_ATLASCAR_positive = 0;
num_BB_ATLASCAR_negative = 0;

nfile_ATLASCAR_inicial = 0;
for i=0:1190
% for i=0:5950
    nfile_ATLASCAR = nfile_ATLASCAR_inicial + i*5;
%     nfile_ATLASCAR = i;
    bag_2_full_file_txt=sprintf('/home/asus/workingcopies/lar4/src/perception/pedestrians/multimodal_pedestrian_detect/images/bag_2_full_txt/00%04d.txt',nfile_ATLASCAR);
    Pedro_ATLASCAR_Algorithm_file_txt=sprintf('/media/Novo volume/bags_tese_results/bag_2_Pedro_Algorithm_txts/%d.txt',nfile_ATLASCAR);
    Fusion_ATLASCAR_Algorithm_2imagereset_1p3_file_txt=sprintf('/media/Novo volume/bags_tese_results/bag_2_sensor_fusion_2image_reset_txts/%d.txt',nfile_ATLASCAR);
    Fusion_ATLASCAR_Algorithm_4imagereset_1p3_file_txt=sprintf('/media/Novo volume/bags_tese_results/bag_2_sensor_fusion_4image_reset_txts/%d.txt',nfile_ATLASCAR);
    Fusion_ATLASCAR_Algorithm_6imagereset_1p3_file_txt=sprintf('/media/Novo volume/bags_tese_results/bag_2_sensor_fusion_6image_reset_txts/%d.txt',nfile_ATLASCAR);
    Fusion_ATLASCAR_Algorithm_8imagereset_1p3_file_txt=sprintf('/media/Novo volume/bags_tese_results/bag_2_sensor_fusion_8image_reset_txts/%d.txt',nfile_ATLASCAR);
    Fusion_ATLASCAR_Algorithm_10imagereset_1p3_file_txt=sprintf('/media/Novo volume/bags_tese_results/bag_2_sensor_fusion_10image_reset_txts_1/%d.txt',nfile_ATLASCAR);
    Fusion_ATLASCAR_Algorithm_20imagereset_1p3_file_txt=sprintf('/media/Novo volume/bags_tese_results/bag_2_sensor_fusion_20image_reset_txts_1/%d.txt',nfile_ATLASCAR);
    Fusion_ATLASCAR_Algorithm_40imagereset_1p3_file_txt=sprintf('/media/Novo volume/bags_tese_results/bag_2_sensor_fusion_40image_reset_txts_1/%d.txt',nfile_ATLASCAR);
    Fusion_ATLASCAR_Algorithm_infimagereset_1p3_file_txt=sprintf('/media/Novo volume/bags_tese_results/bag_2_sensor_fusion_infimage_reset_txts_2/%d.txt',nfile_ATLASCAR);
    
    fileID_ATLASCAR_bag_2 = fopen(bag_2_full_file_txt, 'r');
    fileID_ATLASCAR_Pedro_Algorithm = fopen(Pedro_ATLASCAR_Algorithm_file_txt, 'r');
    fileID_ATLASCAR_Fusion_2imagereset_1p3_Algorithm = fopen(Fusion_ATLASCAR_Algorithm_2imagereset_1p3_file_txt, 'r');
    fileID_ATLASCAR_Fusion_4imagereset_1p3_Algorithm = fopen(Fusion_ATLASCAR_Algorithm_4imagereset_1p3_file_txt, 'r');
    fileID_ATLASCAR_Fusion_6imagereset_1p3_Algorithm = fopen(Fusion_ATLASCAR_Algorithm_6imagereset_1p3_file_txt, 'r');
    fileID_ATLASCAR_Fusion_8imagereset_1p3_Algorithm = fopen(Fusion_ATLASCAR_Algorithm_8imagereset_1p3_file_txt, 'r');
    fileID_ATLASCAR_Fusion_10imagereset_1p3_Algorithm = fopen(Fusion_ATLASCAR_Algorithm_10imagereset_1p3_file_txt, 'r');
    fileID_ATLASCAR_Fusion_20imagereset_1p3_Algorithm = fopen(Fusion_ATLASCAR_Algorithm_20imagereset_1p3_file_txt, 'r');
    fileID_ATLASCAR_Fusion_40imagereset_1p3_Algorithm = fopen(Fusion_ATLASCAR_Algorithm_40imagereset_1p3_file_txt, 'r');
    fileID_ATLASCAR_Fusion_infimagereset_1p3_Algorithm = fopen(Fusion_ATLASCAR_Algorithm_infimagereset_1p3_file_txt, 'r');
    
    A_ATLASCAR_bag_2 = fscanf(fileID_ATLASCAR_bag_2,'%e');
    A_ATLASCAR_Pedro_Algorithm = fscanf(fileID_ATLASCAR_Pedro_Algorithm,'%f');
    A_ATLASCAR_Fusion_2imagereset_1p3_Algorithm = fscanf(fileID_ATLASCAR_Fusion_2imagereset_1p3_Algorithm,'%f');
    A_ATLASCAR_Fusion_4imagereset_1p3_Algorithm = fscanf(fileID_ATLASCAR_Fusion_4imagereset_1p3_Algorithm,'%f');
    A_ATLASCAR_Fusion_6imagereset_1p3_Algorithm = fscanf(fileID_ATLASCAR_Fusion_6imagereset_1p3_Algorithm,'%f');
    A_ATLASCAR_Fusion_8imagereset_1p3_Algorithm = fscanf(fileID_ATLASCAR_Fusion_8imagereset_1p3_Algorithm,'%f');
    A_ATLASCAR_Fusion_10imagereset_1p3_Algorithm = fscanf(fileID_ATLASCAR_Fusion_10imagereset_1p3_Algorithm,'%f');
    A_ATLASCAR_Fusion_20imagereset_1p3_Algorithm = fscanf(fileID_ATLASCAR_Fusion_20imagereset_1p3_Algorithm,'%f');
    A_ATLASCAR_Fusion_40imagereset_1p3_Algorithm = fscanf(fileID_ATLASCAR_Fusion_40imagereset_1p3_Algorithm,'%f');
    A_ATLASCAR_Fusion_infimagereset_1p3_Algorithm = fscanf(fileID_ATLASCAR_Fusion_infimagereset_1p3_Algorithm,'%f');
    
    % ATLASCAR Pedro algorithm
    if (size(A_ATLASCAR_bag_2,1) > 0)
        if (A_ATLASCAR_Pedro_Algorithm(1) == 0 && A_ATLASCAR_Pedro_Algorithm(2) == 0)
            % false negative - miss
%             fprintf(fileID_FN,'%d\n',(size(A_bag_1,1)/4));
            FN_ATLASCAR_Pedro_Algorithm = FN_ATLASCAR_Pedro_Algorithm + (size(A_ATLASCAR_bag_2,1)/4);
        else
            % Test to see if BB's correspond, can be true positive or false
            % positive
            bbs_reshape_ATLASCAR_pedro = reshape(A_ATLASCAR_Pedro_Algorithm,[5 size(A_ATLASCAR_Pedro_Algorithm,1)/5])';
            for q=1:size(bbs_reshape_ATLASCAR_pedro,1)
                bbs_reshape_ATLASCAR_pedro(q, 5) = 1;
            end
            bbs_correct_ATLASCAR_pedro = bbNms(bbs_reshape_ATLASCAR_pedro, 'thr',0.5, 'type','ms');
            
            intersec_data_ATLASCAR_pedro = zeros(1,size(bbs_correct_ATLASCAR_pedro, 1));
            for j=0:(size(A_ATLASCAR_bag_2)/4 - 1)
                for l=0:(size(bbs_correct_ATLASCAR_pedro,1) - 1)
                    if(bbs_correct_ATLASCAR_pedro(l+1, 1) ~= 0)
                        
                        A = [A_ATLASCAR_bag_2(1+4*j), A_ATLASCAR_bag_2(2+4*j), A_ATLASCAR_bag_2(3+4*j), A_ATLASCAR_bag_2(4+4*j)];
                        B = [bbs_correct_ATLASCAR_pedro(l+1, 1), bbs_correct_ATLASCAR_pedro(l+1, 2), bbs_correct_ATLASCAR_pedro(l+1, 3), bbs_correct_ATLASCAR_pedro(l+1, 4)];
                        
                        area_A = rectint(A,A);
                        area_intersec = rectint(A,B);

                        if (area_intersec/area_A >= 0.5)
%                             A_Pedro_Algorithm(1+5*j) = 0; 
%                             A_Pedro_Algorithm(2+5*j) = 0;
%                             A_Pedro_Algorithm(3+5*j) = 0;
%                             A_Pedro_Algorithm(4+5*j) = 0;
%                             fprintf(fileID_TP,'%d \n',1);
                            intersec_data_ATLASCAR_pedro(l+1) = intersec_data_ATLASCAR_pedro(l+1) +1;
                            TP_ATLASCAR_Pedro_Algorithm = TP_ATLASCAR_Pedro_Algorithm + 1;
                        end
                    end
                end
            end
            for k=0:(size(bbs_correct_ATLASCAR_pedro,1) - 1)
%                 if(A_Pedro_Algorithm(1+5*k) ~= 0)
                if(intersec_data_ATLASCAR_pedro(k+1) == 0)
%                     fprintf(fileID_FP,'%d \n',1);
                    FP_ATLASCAR_Pedro_Algorithm = FP_ATLASCAR_Pedro_Algorithm + 1;
                end
            end
        end
    else
        if (A_ATLASCAR_Pedro_Algorithm(1) == 0 && A_ATLASCAR_Pedro_Algorithm(2) == 0)
            % true negative - not found any pedestrians on either images
            TN_ATLASCAR_Pedro_Algorithm = TN_ATLASCAR_Pedro_Algorithm + 1;
        else
            % false positive - false alarm
            bbs_reshape_ATLASCAR_pedro = reshape(A_ATLASCAR_Pedro_Algorithm,[5 size(A_ATLASCAR_Pedro_Algorithm,1)/5])';
            bbs_correct_ATLASCAR_pedro = bbNms(bbs_reshape_ATLASCAR_pedro, 'thr',0.5, 'type','ms');
            
            FP_ATLASCAR_Pedro_Algorithm = FP_ATLASCAR_Pedro_Algorithm + (size(bbs_correct_ATLASCAR_pedro,1));
        end
    end
    
    % Fusion ATLASCAR 2 image 1.3m max width algorithm
    if (size(A_ATLASCAR_bag_2,1) > 0)
        if (A_ATLASCAR_Fusion_2imagereset_1p3_Algorithm(1) == 0 && A_ATLASCAR_Fusion_2imagereset_1p3_Algorithm(2) == 0)
            % false negative - miss
%             fprintf(fileID_FN,'%d\n',(size(A_bag_1,1)/4));
            FN_ATLASCAR_Fusion_2imagereset_1p3_Algorithm = FN_ATLASCAR_Fusion_2imagereset_1p3_Algorithm + (size(A_ATLASCAR_bag_2,1)/4);
        else
            % Test to see if BB's correspond, can be true positive or false
            % positive
            bbs_reshape_ATLASCAR_fusion_2image_1p3 = reshape(A_ATLASCAR_Fusion_2imagereset_1p3_Algorithm,[5 size(A_ATLASCAR_Fusion_2imagereset_1p3_Algorithm,1)/5])';
            for q=1:size(bbs_reshape_ATLASCAR_fusion_2image_1p3,1)
                bbs_reshape_ATLASCAR_fusion_2image_1p3(q, 5) = 1;
            end
            bbs_correct_ATLASCAR_fusion_2image_1p3 = bbNms(bbs_reshape_ATLASCAR_fusion_2image_1p3, 'thr',0.5, 'type','ms');
            
            intersec_data_ATLASCAR_fusion_2image_1p3 = zeros(1,size(bbs_correct_ATLASCAR_fusion_2image_1p3,1));
            for j=0:(size(A_ATLASCAR_bag_2)/4 - 1)
                for l=0:(size(bbs_correct_ATLASCAR_fusion_2image_1p3,1) - 1)
                    if(bbs_correct_ATLASCAR_fusion_2image_1p3(l+1, 1) ~= 0)
                        
                        A = [A_ATLASCAR_bag_2(1+4*j), A_ATLASCAR_bag_2(2+4*j), A_ATLASCAR_bag_2(3+4*j), A_ATLASCAR_bag_2(4+4*j)];
                        B = [bbs_correct_ATLASCAR_fusion_2image_1p3(l+1, 1), bbs_correct_ATLASCAR_fusion_2image_1p3(l+1, 2), bbs_correct_ATLASCAR_fusion_2image_1p3(l+1, 3), bbs_correct_ATLASCAR_fusion_2image_1p3(l+1, 4)];
                        
                        area_A = rectint(A,A);
                        area_intersec = rectint(A,B);

                        if (area_intersec/area_A >= 0.5)
%                             A_Fusion_infimagereset_1p3_Algorithm(1+5*j) = 0; 
%                             A_Fusion_infimagereset_1p3_Algorithm(2+5*j) = 0;
%                             A_Fusion_infimagereset_1p3_Algorithm(3+5*j) = 0;
%                             A_Fusion_infimagereset_1p3_Algorithm(4+5*j) = 0;
%                             fprintf(fileID_TP,'%d \n',1);
                            intersec_data_ATLASCAR_fusion_2image_1p3(l+1) = intersec_data_ATLASCAR_fusion_2image_1p3(l+1) +1;
                            TP_ATLASCAR_Fusion_2imagereset_1p3_Algorithm = TP_ATLASCAR_Fusion_2imagereset_1p3_Algorithm + 1;
                        end
                    end
                end
            end
            for k=0:(size(bbs_correct_ATLASCAR_fusion_2image_1p3,1) - 1)
%                 if(A_Fusion_6imagereset_1p3_Algorithm(1+5*k) ~= 0)
                if(intersec_data_ATLASCAR_fusion_2image_1p3(k+1) == 0)
%                     fprintf(fileID_FP,'%d \n',1);
                    FP_ATLASCAR_Fusion_2imagereset_1p3_Algorithm = FP_ATLASCAR_Fusion_2imagereset_1p3_Algorithm + 1;
                end
            end
        end
    else
        if (A_ATLASCAR_Fusion_2imagereset_1p3_Algorithm(1) == 0 && A_ATLASCAR_Fusion_2imagereset_1p3_Algorithm(2) == 0)
            % true negative - not found any pedestrians on either images
%             fprintf(fileID_TN,'%d \n',1);
            TN_ATLASCAR_Fusion_2imagereset_1p3_Algorithm = TN_ATLASCAR_Fusion_2imagereset_1p3_Algorithm + 1;
        else
            % false positive - false alarm
%             fprintf(fileID_FP,'%d \n',(size(A_Pedro_Algorithm,1))/5);
            bbs_reshape_ATLASCAR_fusion_2image_1p3 = reshape(A_ATLASCAR_Fusion_2imagereset_1p3_Algorithm,[5 size(A_ATLASCAR_Fusion_2imagereset_1p3_Algorithm,1)/5])';
            bbs_correct_ATLASCAR_fusion_2image_1p3 = bbNms(bbs_reshape_ATLASCAR_fusion_2image_1p3, 'thr',0.5, 'type','ms');
            FP_ATLASCAR_Fusion_2imagereset_1p3_Algorithm = FP_ATLASCAR_Fusion_2imagereset_1p3_Algorithm + (size(bbs_correct_ATLASCAR_fusion_2image_1p3,1));
        end
    end
    
    % Fusion ATLASCAR 4 image 1.3m max width algorithm
    if (size(A_ATLASCAR_bag_2,1) > 0)
        if (A_ATLASCAR_Fusion_4imagereset_1p3_Algorithm(1) == 0 && A_ATLASCAR_Fusion_4imagereset_1p3_Algorithm(2) == 0)
            % false negative - miss
%             fprintf(fileID_FN,'%d\n',(size(A_bag_1,1)/4));
            FN_ATLASCAR_Fusion_4imagereset_1p3_Algorithm = FN_ATLASCAR_Fusion_4imagereset_1p3_Algorithm + (size(A_ATLASCAR_bag_2,1)/4);
        else
            % Test to see if BB's correspond, can be true positive or false
            % positive
            bbs_reshape_ATLASCAR_fusion_4image_1p3 = reshape(A_ATLASCAR_Fusion_4imagereset_1p3_Algorithm,[5 size(A_ATLASCAR_Fusion_4imagereset_1p3_Algorithm,1)/5])';
            for q=1:size(bbs_reshape_ATLASCAR_fusion_4image_1p3,1)
                bbs_reshape_ATLASCAR_fusion_4image_1p3(q, 5) = 1;
            end
            bbs_correct_ATLASCAR_fusion_4image_1p3 = bbNms(bbs_reshape_ATLASCAR_fusion_4image_1p3, 'thr',0.5, 'type','ms');
            
            intersec_data_ATLASCAR_fusion_4image_1p3 = zeros(1,size(bbs_correct_ATLASCAR_fusion_4image_1p3,1));
            for j=0:(size(A_ATLASCAR_bag_2)/4 - 1)
                for l=0:(size(bbs_correct_ATLASCAR_fusion_4image_1p3,1) - 1)
                    if(bbs_correct_ATLASCAR_fusion_4image_1p3(l+1, 1) ~= 0)
                        
                        A = [A_ATLASCAR_bag_2(1+4*j), A_ATLASCAR_bag_2(2+4*j), A_ATLASCAR_bag_2(3+4*j), A_ATLASCAR_bag_2(4+4*j)];
                        B = [bbs_correct_ATLASCAR_fusion_4image_1p3(l+1, 1), bbs_correct_ATLASCAR_fusion_4image_1p3(l+1, 2), bbs_correct_ATLASCAR_fusion_4image_1p3(l+1, 3), bbs_correct_ATLASCAR_fusion_4image_1p3(l+1, 4)];
                        
                        area_A = rectint(A,A);
                        area_intersec = rectint(A,B);

                        if (area_intersec/area_A >= 0.5)
%                             A_Fusion_infimagereset_1p3_Algorithm(1+5*j) = 0; 
%                             A_Fusion_infimagereset_1p3_Algorithm(2+5*j) = 0;
%                             A_Fusion_infimagereset_1p3_Algorithm(3+5*j) = 0;
%                             A_Fusion_infimagereset_1p3_Algorithm(4+5*j) = 0;
%                             fprintf(fileID_TP,'%d \n',1);
                            intersec_data_ATLASCAR_fusion_4image_1p3(l+1) = intersec_data_ATLASCAR_fusion_4image_1p3(l+1) +1;
                            TP_ATLASCAR_Fusion_4imagereset_1p3_Algorithm = TP_ATLASCAR_Fusion_4imagereset_1p3_Algorithm + 1;
                        end
                    end
                end
            end
            for k=0:(size(bbs_correct_ATLASCAR_fusion_4image_1p3,1) - 1)
%                 if(A_Fusion_6imagereset_1p3_Algorithm(1+5*k) ~= 0)
                if(intersec_data_ATLASCAR_fusion_4image_1p3(k+1) == 0)
%                     fprintf(fileID_FP,'%d \n',1);
                    FP_ATLASCAR_Fusion_4imagereset_1p3_Algorithm = FP_ATLASCAR_Fusion_4imagereset_1p3_Algorithm + 1;
                end
            end
        end
    else
        if (A_ATLASCAR_Fusion_4imagereset_1p3_Algorithm(1) == 0 && A_ATLASCAR_Fusion_4imagereset_1p3_Algorithm(2) == 0)
            % true negative - not found any pedestrians on either images
%             fprintf(fileID_TN,'%d \n',1);
            TN_ATLASCAR_Fusion_4imagereset_1p3_Algorithm = TN_ATLASCAR_Fusion_4imagereset_1p3_Algorithm + 1;
        else
            % false positive - false alarm
%             fprintf(fileID_FP,'%d \n',(size(A_Pedro_Algorithm,1))/5);
            bbs_reshape_ATLASCAR_fusion_4image_1p3 = reshape(A_ATLASCAR_Fusion_4imagereset_1p3_Algorithm,[5 size(A_ATLASCAR_Fusion_4imagereset_1p3_Algorithm,1)/5])';
            bbs_correct_ATLASCAR_fusion_4image_1p3 = bbNms(bbs_reshape_ATLASCAR_fusion_4image_1p3, 'thr',0.5, 'type','ms');
            FP_ATLASCAR_Fusion_4imagereset_1p3_Algorithm = FP_ATLASCAR_Fusion_4imagereset_1p3_Algorithm + (size(bbs_correct_ATLASCAR_fusion_4image_1p3,1));
        end
    end
    
    % Fusion ATLASCAR 6 image 1.3m max width algorithm
    if (size(A_ATLASCAR_bag_2,1) > 0)
        if (A_ATLASCAR_Fusion_6imagereset_1p3_Algorithm(1) == 0 && A_ATLASCAR_Fusion_6imagereset_1p3_Algorithm(2) == 0)
            % false negative - miss
%             fprintf(fileID_FN,'%d\n',(size(A_bag_1,1)/4));
            FN_ATLASCAR_Fusion_6imagereset_1p3_Algorithm = FN_ATLASCAR_Fusion_6imagereset_1p3_Algorithm + (size(A_ATLASCAR_bag_2,1)/4);
        else
            % Test to see if BB's correspond, can be true positive or false
            % positive
            bbs_reshape_ATLASCAR_fusion_6image_1p3 = reshape(A_ATLASCAR_Fusion_6imagereset_1p3_Algorithm,[5 size(A_ATLASCAR_Fusion_6imagereset_1p3_Algorithm,1)/5])';
            for q=1:size(bbs_reshape_ATLASCAR_fusion_6image_1p3,1)
                bbs_reshape_ATLASCAR_fusion_6image_1p3(q, 5) = 1;
            end
            bbs_correct_ATLASCAR_fusion_6image_1p3 = bbNms(bbs_reshape_ATLASCAR_fusion_6image_1p3, 'thr',0.5, 'type','ms');
            
            intersec_data_ATLASCAR_fusion_6image_1p3 = zeros(1,size(bbs_correct_ATLASCAR_fusion_6image_1p3,1));
            for j=0:(size(A_ATLASCAR_bag_2)/4 - 1)
                for l=0:(size(bbs_correct_ATLASCAR_fusion_6image_1p3,1) - 1)
                    if(bbs_correct_ATLASCAR_fusion_6image_1p3(l+1, 1) ~= 0)
                        
                        A = [A_ATLASCAR_bag_2(1+4*j), A_ATLASCAR_bag_2(2+4*j), A_ATLASCAR_bag_2(3+4*j), A_ATLASCAR_bag_2(4+4*j)];
                        B = [bbs_correct_ATLASCAR_fusion_6image_1p3(l+1, 1), bbs_correct_ATLASCAR_fusion_6image_1p3(l+1, 2), bbs_correct_ATLASCAR_fusion_6image_1p3(l+1, 3), bbs_correct_ATLASCAR_fusion_6image_1p3(l+1, 4)];
                        
                        area_A = rectint(A,A);
                        area_intersec = rectint(A,B);

                        if (area_intersec/area_A >= 0.5)
%                             A_Fusion_infimagereset_1p3_Algorithm(1+5*j) = 0; 
%                             A_Fusion_infimagereset_1p3_Algorithm(2+5*j) = 0;
%                             A_Fusion_infimagereset_1p3_Algorithm(3+5*j) = 0;
%                             A_Fusion_infimagereset_1p3_Algorithm(4+5*j) = 0;
%                             fprintf(fileID_TP,'%d \n',1);
                            intersec_data_ATLASCAR_fusion_6image_1p3(l+1) = intersec_data_ATLASCAR_fusion_6image_1p3(l+1) +1;
                            TP_ATLASCAR_Fusion_6imagereset_1p3_Algorithm = TP_ATLASCAR_Fusion_6imagereset_1p3_Algorithm + 1;
                        end
                    end
                end
            end
            for k=0:(size(bbs_correct_ATLASCAR_fusion_6image_1p3,1) - 1)
%                 if(A_Fusion_6imagereset_1p3_Algorithm(1+5*k) ~= 0)
                if(intersec_data_ATLASCAR_fusion_6image_1p3(k+1) == 0)
%                     fprintf(fileID_FP,'%d \n',1);
                    FP_ATLASCAR_Fusion_6imagereset_1p3_Algorithm = FP_ATLASCAR_Fusion_6imagereset_1p3_Algorithm + 1;
                end
            end
        end
    else
        if (A_ATLASCAR_Fusion_6imagereset_1p3_Algorithm(1) == 0 && A_ATLASCAR_Fusion_6imagereset_1p3_Algorithm(2) == 0)
            % true negative - not found any pedestrians on either images
%             fprintf(fileID_TN,'%d \n',1);
            TN_ATLASCAR_Fusion_6imagereset_1p3_Algorithm = TN_ATLASCAR_Fusion_6imagereset_1p3_Algorithm + 1;
        else
            % false positive - false alarm
%             fprintf(fileID_FP,'%d \n',(size(A_Pedro_Algorithm,1))/5);
            bbs_reshape_ATLASCAR_fusion_6image_1p3 = reshape(A_ATLASCAR_Fusion_6imagereset_1p3_Algorithm,[5 size(A_ATLASCAR_Fusion_6imagereset_1p3_Algorithm,1)/5])';
            bbs_correct_ATLASCAR_fusion_6image_1p3 = bbNms(bbs_reshape_ATLASCAR_fusion_6image_1p3, 'thr',0.5, 'type','ms');
            FP_ATLASCAR_Fusion_6imagereset_1p3_Algorithm = FP_ATLASCAR_Fusion_6imagereset_1p3_Algorithm + (size(bbs_correct_ATLASCAR_fusion_6image_1p3,1));
        end
    end
    
    % Fusion ATLASCAR 8 image 1.3m max width algorithm
    if (size(A_ATLASCAR_bag_2,1) > 0)
        if (A_ATLASCAR_Fusion_8imagereset_1p3_Algorithm(1) == 0 && A_ATLASCAR_Fusion_8imagereset_1p3_Algorithm(2) == 0)
            % false negative - miss
%             fprintf(fileID_FN,'%d\n',(size(A_bag_1,1)/4));
            FN_ATLASCAR_Fusion_8imagereset_1p3_Algorithm = FN_ATLASCAR_Fusion_8imagereset_1p3_Algorithm + (size(A_ATLASCAR_bag_2,1)/4);
        else
            % Test to see if BB's correspond, can be true positive or false
            % positive
            bbs_reshape_ATLASCAR_fusion_8image_1p3 = reshape(A_ATLASCAR_Fusion_8imagereset_1p3_Algorithm,[5 size(A_ATLASCAR_Fusion_8imagereset_1p3_Algorithm,1)/5])';
            for q=1:size(bbs_reshape_ATLASCAR_fusion_8image_1p3,1)
                bbs_reshape_ATLASCAR_fusion_8image_1p3(q, 5) = 1;
            end
            bbs_correct_ATLASCAR_fusion_8image_1p3 = bbNms(bbs_reshape_ATLASCAR_fusion_8image_1p3, 'thr',0.5, 'type','ms');
            
            intersec_data_ATLASCAR_fusion_8image_1p3 = zeros(1,size(bbs_correct_ATLASCAR_fusion_8image_1p3,1));
            for j=0:(size(A_ATLASCAR_bag_2)/4 - 1)
                for l=0:(size(bbs_correct_ATLASCAR_fusion_8image_1p3,1) - 1)
                    if(bbs_correct_ATLASCAR_fusion_8image_1p3(l+1, 1) ~= 0)
                        
                        A = [A_ATLASCAR_bag_2(1+4*j), A_ATLASCAR_bag_2(2+4*j), A_ATLASCAR_bag_2(3+4*j), A_ATLASCAR_bag_2(4+4*j)];
                        B = [bbs_correct_ATLASCAR_fusion_8image_1p3(l+1, 1), bbs_correct_ATLASCAR_fusion_8image_1p3(l+1, 2), bbs_correct_ATLASCAR_fusion_8image_1p3(l+1, 3), bbs_correct_ATLASCAR_fusion_8image_1p3(l+1, 4)];
                        
                        area_A = rectint(A,A);
                        area_intersec = rectint(A,B);

                        if (area_intersec/area_A >= 0.5)
%                             A_Fusion_infimagereset_1p3_Algorithm(1+5*j) = 0; 
%                             A_Fusion_infimagereset_1p3_Algorithm(2+5*j) = 0;
%                             A_Fusion_infimagereset_1p3_Algorithm(3+5*j) = 0;
%                             A_Fusion_infimagereset_1p3_Algorithm(4+5*j) = 0;
%                             fprintf(fileID_TP,'%d \n',1);
                            intersec_data_ATLASCAR_fusion_8image_1p3(l+1) = intersec_data_ATLASCAR_fusion_8image_1p3(l+1) +1;
                            TP_ATLASCAR_Fusion_8imagereset_1p3_Algorithm = TP_ATLASCAR_Fusion_8imagereset_1p3_Algorithm + 1;
                        end
                    end
                end
            end
            for k=0:(size(bbs_correct_ATLASCAR_fusion_8image_1p3,1) - 1)
%                 if(A_Fusion_6imagereset_1p3_Algorithm(1+5*k) ~= 0)
                if(intersec_data_ATLASCAR_fusion_8image_1p3(k+1) == 0)
%                     fprintf(fileID_FP,'%d \n',1);
                    FP_ATLASCAR_Fusion_8imagereset_1p3_Algorithm = FP_ATLASCAR_Fusion_8imagereset_1p3_Algorithm + 1;
                end
            end
        end
    else
        if (A_ATLASCAR_Fusion_8imagereset_1p3_Algorithm(1) == 0 && A_ATLASCAR_Fusion_8imagereset_1p3_Algorithm(2) == 0)
            % true negative - not found any pedestrians on either images
%             fprintf(fileID_TN,'%d \n',1);
            TN_ATLASCAR_Fusion_8imagereset_1p3_Algorithm = TN_ATLASCAR_Fusion_8imagereset_1p3_Algorithm + 1;
        else
            % false positive - false alarm
%             fprintf(fileID_FP,'%d \n',(size(A_Pedro_Algorithm,1))/5);
            bbs_reshape_ATLASCAR_fusion_8image_1p3 = reshape(A_ATLASCAR_Fusion_8imagereset_1p3_Algorithm,[5 size(A_ATLASCAR_Fusion_8imagereset_1p3_Algorithm,1)/5])';
            bbs_correct_ATLASCAR_fusion_8image_1p3 = bbNms(bbs_reshape_ATLASCAR_fusion_8image_1p3, 'thr',0.5, 'type','ms');
            FP_ATLASCAR_Fusion_8imagereset_1p3_Algorithm = FP_ATLASCAR_Fusion_8imagereset_1p3_Algorithm + (size(bbs_correct_ATLASCAR_fusion_8image_1p3,1));
        end
    end
    
    % Fusion ATLASCAR 10 image 1.3m max width algorithm
    if (size(A_ATLASCAR_bag_2,1) > 0)
        if (A_ATLASCAR_Fusion_10imagereset_1p3_Algorithm(1) == 0 && A_ATLASCAR_Fusion_10imagereset_1p3_Algorithm(2) == 0)
            % false negative - miss
%             fprintf(fileID_FN,'%d\n',(size(A_bag_1,1)/4));
            FN_ATLASCAR_Fusion_10imagereset_1p3_Algorithm = FN_ATLASCAR_Fusion_10imagereset_1p3_Algorithm + (size(A_ATLASCAR_bag_2,1)/4);
        else
            % Test to see if BB's correspond, can be true positive or false
            % positive
            bbs_reshape_ATLASCAR_fusion_10image_1p3 = reshape(A_ATLASCAR_Fusion_10imagereset_1p3_Algorithm,[5 size(A_ATLASCAR_Fusion_10imagereset_1p3_Algorithm,1)/5])';
            for q=1:size(bbs_reshape_ATLASCAR_fusion_10image_1p3,1)
                bbs_reshape_ATLASCAR_fusion_10image_1p3(q, 5) = 1;
            end
            bbs_correct_ATLASCAR_fusion_10image_1p3 = bbNms(bbs_reshape_ATLASCAR_fusion_10image_1p3, 'thr',0.5, 'type','ms');
            
            intersec_data_ATLASCAR_fusion_10image_1p3 = zeros(1,size(bbs_correct_ATLASCAR_fusion_10image_1p3,1));
            for j=0:(size(A_ATLASCAR_bag_2)/4 - 1)
                for l=0:(size(bbs_correct_ATLASCAR_fusion_10image_1p3,1) - 1)
                    if(bbs_correct_ATLASCAR_fusion_10image_1p3(l+1, 1) ~= 0)
                        
                        A = [A_ATLASCAR_bag_2(1+4*j), A_ATLASCAR_bag_2(2+4*j), A_ATLASCAR_bag_2(3+4*j), A_ATLASCAR_bag_2(4+4*j)];
                        B = [bbs_correct_ATLASCAR_fusion_10image_1p3(l+1, 1), bbs_correct_ATLASCAR_fusion_10image_1p3(l+1, 2), bbs_correct_ATLASCAR_fusion_10image_1p3(l+1, 3), bbs_correct_ATLASCAR_fusion_10image_1p3(l+1, 4)];
                        
                        area_A = rectint(A,A);
                        area_intersec = rectint(A,B);

                        if (area_intersec/area_A >= 0.5)
%                             A_Fusion_infimagereset_1p3_Algorithm(1+5*j) = 0; 
%                             A_Fusion_infimagereset_1p3_Algorithm(2+5*j) = 0;
%                             A_Fusion_infimagereset_1p3_Algorithm(3+5*j) = 0;
%                             A_Fusion_infimagereset_1p3_Algorithm(4+5*j) = 0;
%                             fprintf(fileID_TP,'%d \n',1);
                            intersec_data_ATLASCAR_fusion_10image_1p3(l+1) = intersec_data_ATLASCAR_fusion_10image_1p3(l+1) +1;
                            TP_ATLASCAR_Fusion_10imagereset_1p3_Algorithm = TP_ATLASCAR_Fusion_10imagereset_1p3_Algorithm + 1;
                        end
                    end
                end
            end
            for k=0:(size(bbs_correct_ATLASCAR_fusion_10image_1p3,1) - 1)
%                 if(A_Fusion_6imagereset_1p3_Algorithm(1+5*k) ~= 0)
                if(intersec_data_ATLASCAR_fusion_10image_1p3(k+1) == 0)
%                     fprintf(fileID_FP,'%d \n',1);
                    FP_ATLASCAR_Fusion_10imagereset_1p3_Algorithm = FP_ATLASCAR_Fusion_10imagereset_1p3_Algorithm + 1;
                end
            end
        end
    else
        if (A_ATLASCAR_Fusion_10imagereset_1p3_Algorithm(1) == 0 && A_ATLASCAR_Fusion_10imagereset_1p3_Algorithm(2) == 0)
            % true negative - not found any pedestrians on either images
%             fprintf(fileID_TN,'%d \n',1);
            TN_ATLASCAR_Fusion_10imagereset_1p3_Algorithm = TN_ATLASCAR_Fusion_10imagereset_1p3_Algorithm + 1;
        else
            % false positive - false alarm
%             fprintf(fileID_FP,'%d \n',(size(A_Pedro_Algorithm,1))/5);
            bbs_reshape_ATLASCAR_fusion_10image_1p3 = reshape(A_ATLASCAR_Fusion_10imagereset_1p3_Algorithm,[5 size(A_ATLASCAR_Fusion_10imagereset_1p3_Algorithm,1)/5])';
            bbs_correct_ATLASCAR_fusion_10image_1p3 = bbNms(bbs_reshape_ATLASCAR_fusion_10image_1p3, 'thr',0.5, 'type','ms');
            FP_ATLASCAR_Fusion_10imagereset_1p3_Algorithm = FP_ATLASCAR_Fusion_10imagereset_1p3_Algorithm + (size(bbs_correct_ATLASCAR_fusion_10image_1p3,1));
        end
    end
    
    % Fusion ATLASCAR 20 image 1.3m max width algorithm
    if (size(A_ATLASCAR_bag_2,1) > 0)
        if (A_ATLASCAR_Fusion_20imagereset_1p3_Algorithm(1) == 0 && A_ATLASCAR_Fusion_20imagereset_1p3_Algorithm(2) == 0)
            % false negative - miss
%             fprintf(fileID_FN,'%d\n',(size(A_bag_1,1)/4));
            FN_ATLASCAR_Fusion_20imagereset_1p3_Algorithm = FN_ATLASCAR_Fusion_20imagereset_1p3_Algorithm + (size(A_ATLASCAR_bag_2,1)/4);
        else
            % Test to see if BB's correspond, can be true positive or false
            % positive
            bbs_reshape_ATLASCAR_fusion_20image_1p3 = reshape(A_ATLASCAR_Fusion_20imagereset_1p3_Algorithm,[5 size(A_ATLASCAR_Fusion_20imagereset_1p3_Algorithm,1)/5])';
            for q=1:size(bbs_reshape_ATLASCAR_fusion_20image_1p3,1)
                bbs_reshape_ATLASCAR_fusion_20image_1p3(q, 5) = 1;
            end
            bbs_correct_ATLASCAR_fusion_20image_1p3 = bbNms(bbs_reshape_ATLASCAR_fusion_20image_1p3, 'thr',0.5, 'type','ms');
            
            intersec_data_ATLASCAR_fusion_20image_1p3 = zeros(1,size(bbs_correct_ATLASCAR_fusion_20image_1p3,1));
            for j=0:(size(A_ATLASCAR_bag_2)/4 - 1)
                for l=0:(size(bbs_correct_ATLASCAR_fusion_20image_1p3,1) - 1)
                    if(bbs_correct_ATLASCAR_fusion_20image_1p3(l+1, 1) ~= 0)
                        
                        A = [A_ATLASCAR_bag_2(1+4*j), A_ATLASCAR_bag_2(2+4*j), A_ATLASCAR_bag_2(3+4*j), A_ATLASCAR_bag_2(4+4*j)];
                        B = [bbs_correct_ATLASCAR_fusion_20image_1p3(l+1, 1), bbs_correct_ATLASCAR_fusion_20image_1p3(l+1, 2), bbs_correct_ATLASCAR_fusion_20image_1p3(l+1, 3), bbs_correct_ATLASCAR_fusion_20image_1p3(l+1, 4)];
                        
                        area_A = rectint(A,A);
                        area_intersec = rectint(A,B);

                        if (area_intersec/area_A >= 0.5)
%                             A_Fusion_infimagereset_1p3_Algorithm(1+5*j) = 0; 
%                             A_Fusion_infimagereset_1p3_Algorithm(2+5*j) = 0;
%                             A_Fusion_infimagereset_1p3_Algorithm(3+5*j) = 0;
%                             A_Fusion_infimagereset_1p3_Algorithm(4+5*j) = 0;
%                             fprintf(fileID_TP,'%d \n',1);
                            intersec_data_ATLASCAR_fusion_20image_1p3(l+1) = intersec_data_ATLASCAR_fusion_20image_1p3(l+1) +1;
                            TP_ATLASCAR_Fusion_20imagereset_1p3_Algorithm = TP_ATLASCAR_Fusion_20imagereset_1p3_Algorithm + 1;
                        end
                    end
                end
            end
            for k=0:(size(bbs_correct_ATLASCAR_fusion_20image_1p3,1) - 1)
%                 if(A_Fusion_6imagereset_1p3_Algorithm(1+5*k) ~= 0)
                if(intersec_data_ATLASCAR_fusion_20image_1p3(k+1) == 0)
%                     fprintf(fileID_FP,'%d \n',1);
                    FP_ATLASCAR_Fusion_20imagereset_1p3_Algorithm = FP_ATLASCAR_Fusion_20imagereset_1p3_Algorithm + 1;
                end
            end
        end
    else
        if (A_ATLASCAR_Fusion_20imagereset_1p3_Algorithm(1) == 0 && A_ATLASCAR_Fusion_20imagereset_1p3_Algorithm(2) == 0)
            % true negative - not found any pedestrians on either images
%             fprintf(fileID_TN,'%d \n',1);
            TN_ATLASCAR_Fusion_20imagereset_1p3_Algorithm = TN_ATLASCAR_Fusion_20imagereset_1p3_Algorithm + 1;
        else
            % false positive - false alarm
%             fprintf(fileID_FP,'%d \n',(size(A_Pedro_Algorithm,1))/5);
            bbs_reshape_ATLASCAR_fusion_20image_1p3 = reshape(A_ATLASCAR_Fusion_20imagereset_1p3_Algorithm,[5 size(A_ATLASCAR_Fusion_20imagereset_1p3_Algorithm,1)/5])';
            bbs_correct_ATLASCAR_fusion_20image_1p3 = bbNms(bbs_reshape_ATLASCAR_fusion_20image_1p3, 'thr',0.5, 'type','ms');
            FP_ATLASCAR_Fusion_20imagereset_1p3_Algorithm = FP_ATLASCAR_Fusion_20imagereset_1p3_Algorithm + (size(bbs_correct_ATLASCAR_fusion_20image_1p3,1));
        end
    end
    
    % Fusion ATLASCAR 40 image 1.3m max width algorithm
    if (size(A_ATLASCAR_bag_2,1) > 0)
        if (A_ATLASCAR_Fusion_40imagereset_1p3_Algorithm(1) == 0 && A_ATLASCAR_Fusion_40imagereset_1p3_Algorithm(2) == 0)
            % false negative - miss
%             fprintf(fileID_FN,'%d\n',(size(A_bag_1,1)/4));
            FN_ATLASCAR_Fusion_40imagereset_1p3_Algorithm = FN_ATLASCAR_Fusion_40imagereset_1p3_Algorithm + (size(A_ATLASCAR_bag_2,1)/4);
        else
            % Test to see if BB's correspond, can be true positive or false
            % positive
            bbs_reshape_ATLASCAR_fusion_40image_1p3 = reshape(A_ATLASCAR_Fusion_40imagereset_1p3_Algorithm,[5 size(A_ATLASCAR_Fusion_40imagereset_1p3_Algorithm,1)/5])';
            for q=1:size(bbs_reshape_ATLASCAR_fusion_40image_1p3,1)
                bbs_reshape_ATLASCAR_fusion_40image_1p3(q, 5) = 1;
            end
            bbs_correct_ATLASCAR_fusion_40image_1p3 = bbNms(bbs_reshape_ATLASCAR_fusion_40image_1p3, 'thr',0.5, 'type','ms');
            
            intersec_data_ATLASCAR_fusion_40image_1p3 = zeros(1,size(bbs_correct_ATLASCAR_fusion_40image_1p3,1));
            for j=0:(size(A_ATLASCAR_bag_2)/4 - 1)
                for l=0:(size(bbs_correct_ATLASCAR_fusion_40image_1p3,1) - 1)
                    if(bbs_correct_ATLASCAR_fusion_40image_1p3(l+1, 1) ~= 0)
                        
                        A = [A_ATLASCAR_bag_2(1+4*j), A_ATLASCAR_bag_2(2+4*j), A_ATLASCAR_bag_2(3+4*j), A_ATLASCAR_bag_2(4+4*j)];
                        B = [bbs_correct_ATLASCAR_fusion_40image_1p3(l+1, 1), bbs_correct_ATLASCAR_fusion_40image_1p3(l+1, 2), bbs_correct_ATLASCAR_fusion_40image_1p3(l+1, 3), bbs_correct_ATLASCAR_fusion_40image_1p3(l+1, 4)];
                        
                        area_A = rectint(A,A);
                        area_intersec = rectint(A,B);

                        if (area_intersec/area_A >= 0.5)
%                             A_Fusion_infimagereset_1p3_Algorithm(1+5*j) = 0; 
%                             A_Fusion_infimagereset_1p3_Algorithm(2+5*j) = 0;
%                             A_Fusion_infimagereset_1p3_Algorithm(3+5*j) = 0;
%                             A_Fusion_infimagereset_1p3_Algorithm(4+5*j) = 0;
%                             fprintf(fileID_TP,'%d \n',1);
                            intersec_data_ATLASCAR_fusion_40image_1p3(l+1) = intersec_data_ATLASCAR_fusion_40image_1p3(l+1) +1;
                            TP_ATLASCAR_Fusion_40imagereset_1p3_Algorithm = TP_ATLASCAR_Fusion_40imagereset_1p3_Algorithm + 1;
                        end
                    end
                end
            end
            for k=0:(size(bbs_correct_ATLASCAR_fusion_40image_1p3,1) - 1)
%                 if(A_Fusion_6imagereset_1p3_Algorithm(1+5*k) ~= 0)
                if(intersec_data_ATLASCAR_fusion_40image_1p3(k+1) == 0)
%                     fprintf(fileID_FP,'%d \n',1);
                    FP_ATLASCAR_Fusion_40imagereset_1p3_Algorithm = FP_ATLASCAR_Fusion_40imagereset_1p3_Algorithm + 1;
                end
            end
        end
    else
        if (A_ATLASCAR_Fusion_40imagereset_1p3_Algorithm(1) == 0 && A_ATLASCAR_Fusion_40imagereset_1p3_Algorithm(2) == 0)
            % true negative - not found any pedestrians on either images
%             fprintf(fileID_TN,'%d \n',1);
            TN_ATLASCAR_Fusion_40imagereset_1p3_Algorithm = TN_ATLASCAR_Fusion_40imagereset_1p3_Algorithm + 1;
        else
            % false positive - false alarm
%             fprintf(fileID_FP,'%d \n',(size(A_Pedro_Algorithm,1))/5);
            bbs_reshape_ATLASCAR_fusion_40image_1p3 = reshape(A_ATLASCAR_Fusion_40imagereset_1p3_Algorithm,[5 size(A_ATLASCAR_Fusion_40imagereset_1p3_Algorithm,1)/5])';
            bbs_correct_ATLASCAR_fusion_40image_1p3 = bbNms(bbs_reshape_ATLASCAR_fusion_40image_1p3, 'thr',0.5, 'type','ms');
            FP_ATLASCAR_Fusion_40imagereset_1p3_Algorithm = FP_ATLASCAR_Fusion_40imagereset_1p3_Algorithm + (size(bbs_correct_ATLASCAR_fusion_40image_1p3,1));
        end
    end
    
    % Fusion ATLASCAR inf image 1.3m max width algorithm
    if (size(A_ATLASCAR_bag_2,1) > 0)
        if (A_ATLASCAR_Fusion_infimagereset_1p3_Algorithm(1) == 0 && A_ATLASCAR_Fusion_infimagereset_1p3_Algorithm(2) == 0)
            % false negative - miss
            FN_ATLASCAR_Fusion_infimagereset_1p3_Algorithm = FN_ATLASCAR_Fusion_infimagereset_1p3_Algorithm + (size(A_ATLASCAR_bag_2,1)/4);
        else
            % Test to see if BB's correspond, can be true positive or false
            % positive
            bbs_reshape_ATLASCAR_fusion_infimage_1p3 = reshape(A_ATLASCAR_Fusion_infimagereset_1p3_Algorithm,[5 size(A_ATLASCAR_Fusion_infimagereset_1p3_Algorithm,1)/5])';
            for q=1:size(bbs_reshape_ATLASCAR_fusion_infimage_1p3,1)
                bbs_reshape_ATLASCAR_fusion_infimage_1p3(q, 5) = 1;
            end
            bbs_correct_ATLASCAR_fusion_infimage_1p3 = bbNms(bbs_reshape_ATLASCAR_fusion_infimage_1p3, 'thr',0.5, 'type','ms');
%             bbs_correct_ATLASCAR_fusion_infimage_1p3 = bbs_reshape_ATLASCAR_fusion_infimage_1p3;
            
            intersec_data_ATLASCAR_fusion_infimage_1p3 = zeros(1,size(bbs_correct_ATLASCAR_fusion_infimage_1p3,1));
            for j=0:(size(A_ATLASCAR_bag_2)/4 - 1)
                for l=0:(size(bbs_correct_ATLASCAR_fusion_infimage_1p3,1) - 1)
                    if(bbs_correct_ATLASCAR_fusion_infimage_1p3(l+1, 1) ~= 0)
                        
                        A = [A_ATLASCAR_bag_2(1+4*j), A_ATLASCAR_bag_2(2+4*j), A_ATLASCAR_bag_2(3+4*j), A_ATLASCAR_bag_2(4+4*j)];
                        B = [bbs_correct_ATLASCAR_fusion_infimage_1p3(l+1, 1), bbs_correct_ATLASCAR_fusion_infimage_1p3(l+1, 2), bbs_correct_ATLASCAR_fusion_infimage_1p3(l+1, 3), bbs_correct_ATLASCAR_fusion_infimage_1p3(l+1, 4)];
                        
                        area_A = rectint(A,A);
                        area_intersec = rectint(A,B);

                        if (area_intersec/area_A >= 0.5)
%                             A_Fusion_infimagereset_1p3_Algorithm(1+5*j) = 0; 
%                             A_Fusion_infimagereset_1p3_Algorithm(2+5*j) = 0;
%                             A_Fusion_infimagereset_1p3_Algorithm(3+5*j) = 0;
%                             A_Fusion_infimagereset_1p3_Algorithm(4+5*j) = 0;
%                             fprintf(fileID_TP,'%d \n',1);
                            intersec_data_ATLASCAR_fusion_infimage_1p3(l+1) = intersec_data_ATLASCAR_fusion_infimage_1p3(l+1) +1;
                            TP_ATLASCAR_Fusion_infimagereset_1p3_Algorithm = TP_ATLASCAR_Fusion_infimagereset_1p3_Algorithm + 1;
                        end
                    end
                end
            end
            if(size(bbs_correct_ATLASCAR_fusion_infimage_1p3,1) > 0)
                for k=0:(size(bbs_correct_ATLASCAR_fusion_infimage_1p3,1) - 1)
    %                 if(A_Fusion_6imagereset_1p3_Algorithm(1+5*k) ~= 0)
                    if(intersec_data_ATLASCAR_fusion_infimage_1p3(k+1) == 0)
    %                     fprintf(fileID_FP,'%d \n',1);
                        FP_ATLASCAR_Fusion_infimagereset_1p3_Algorithm = FP_ATLASCAR_Fusion_infimagereset_1p3_Algorithm + 1;
                    end
                end
            end
        end
    else
        if (A_ATLASCAR_Fusion_infimagereset_1p3_Algorithm(1) == 0 && A_ATLASCAR_Fusion_infimagereset_1p3_Algorithm(2) == 0)
            % true negative - not found any pedestrians on either images
%             fprintf(fileID_TN,'%d \n',1);
            TN_ATLASCAR_Fusion_infimagereset_1p3_Algorithm = TN_ATLASCAR_Fusion_infimagereset_1p3_Algorithm + 1;
        else
            % false positive - false alarm
%             fprintf(fileID_FP,'%d \n',(size(A_Pedro_Algorithm,1))/5);
            bbs_reshape_ATLASCAR_fusion_infimage_1p3 = reshape(A_ATLASCAR_Fusion_infimagereset_1p3_Algorithm,[5 size(A_ATLASCAR_Fusion_infimagereset_1p3_Algorithm,1)/5])';
            bbs_correct_ATLASCAR_fusion_infimage_1p3 = bbNms(bbs_reshape_ATLASCAR_fusion_infimage_1p3, 'thr',0.5, 'type','ms');
            FP_ATLASCAR_Fusion_infimagereset_1p3_Algorithm = FP_ATLASCAR_Fusion_infimagereset_1p3_Algorithm + (size(bbs_correct_ATLASCAR_fusion_infimage_1p3,1));
        end
    end
    
    hist_ATLASCAR_Pedro_Algorithm = [hist_ATLASCAR_Pedro_Algorithm A_ATLASCAR_Pedro_Algorithm(end,1)];
    hist_ATLASCAR_Fusion_2imagereset_1p3_Algorithm = [hist_ATLASCAR_Fusion_2imagereset_1p3_Algorithm A_ATLASCAR_Fusion_2imagereset_1p3_Algorithm(end,1)];
    hist_ATLASCAR_Fusion_4imagereset_1p3_Algorithm = [hist_ATLASCAR_Fusion_4imagereset_1p3_Algorithm A_ATLASCAR_Fusion_4imagereset_1p3_Algorithm(end,1)];
    hist_ATLASCAR_Fusion_6imagereset_1p3_Algorithm = [hist_ATLASCAR_Fusion_6imagereset_1p3_Algorithm A_ATLASCAR_Fusion_6imagereset_1p3_Algorithm(end,1)];
    hist_ATLASCAR_Fusion_8imagereset_1p3_Algorithm = [hist_ATLASCAR_Fusion_8imagereset_1p3_Algorithm A_ATLASCAR_Fusion_8imagereset_1p3_Algorithm(end,1)];
    hist_ATLASCAR_Fusion_10imagereset_1p3_Algorithm = [hist_ATLASCAR_Fusion_10imagereset_1p3_Algorithm A_ATLASCAR_Fusion_10imagereset_1p3_Algorithm(end,1)];
    hist_ATLASCAR_Fusion_20imagereset_1p3_Algorithm = [hist_ATLASCAR_Fusion_20imagereset_1p3_Algorithm A_ATLASCAR_Fusion_20imagereset_1p3_Algorithm(end,1)];
    hist_ATLASCAR_Fusion_40imagereset_1p3_Algorithm = [hist_ATLASCAR_Fusion_40imagereset_1p3_Algorithm A_ATLASCAR_Fusion_40imagereset_1p3_Algorithm(end,1)];
    hist_ATLASCAR_Fusion_infimagereset_1p3_Algorithm = [hist_ATLASCAR_Fusion_infimagereset_1p3_Algorithm A_ATLASCAR_Fusion_infimagereset_1p3_Algorithm(end,1)];

    num_BB_ATLASCAR_positive = num_BB_ATLASCAR_positive + size(A_ATLASCAR_bag_2,1)/4;
    
    if (size(A_ATLASCAR_bag_2,1)/4 == 0)
        num_BB_ATLASCAR_negative = num_BB_ATLASCAR_negative + 1;
    end
    
    fclose('all');
    
end    



% TPR_ATLASCAR_Fusion_infimagereset_1p3 =
% 
%     0.7017
% 
% 
% FPR_ATLASCAR_Fusion_infimagereset_1p3 =
% 
%     0.7231


% 
% hist(hist_Pedro_Algorithm,200)
% hold on;
% hist(hist_Fusion_2imagereset_Algorithm,200)
% hold on;
% hist(hist_Fusion_4imagereset_Algorithm,200)
% hold on;
% hist(hist_Fusion_10imagereset_Algorithm,200)
% hold on;
% 
% h = findobj(gca,'Type','patch');
% set(h(1),'Facecolor',[1 0 0]);
% set(h(2),'Facecolor',[0 0 1]); 
% set(h(3),'Facecolor',[0 1 0]);
% set(h(4),'Facecolor',[1 1 0]);

num_BB_positive;
TPR_Pedro = TP_Pedro_Algorithm/(TP_Pedro_Algorithm + FN_Pedro_Algorithm)
SPC_Pedro = TN_Pedro_Algorithm/(FP_Pedro_Algorithm + TN_Pedro_Algorithm);
PPV_Pedro = TP_Pedro_Algorithm/(TP_Pedro_Algorithm + FP_Pedro_Algorithm);
NPV_Pedro = TN_Pedro_Algorithm/(TN_Pedro_Algorithm + FN_Pedro_Algorithm);
FPR_Pedro = 1 - SPC_Pedro
FDR_Pedro = 1 - PPV_Pedro;
media_tempo_processamento_Pedro = mean(hist_Pedro_Algorithm)
max_tempo_processamento_Pedro = max(hist_Pedro_Algorithm);

TPR_Fusion = TP_Fusion_Algorithm/(TP_Fusion_Algorithm + FN_Fusion_Algorithm)
SPC_Fusion = TN_Fusion_Algorithm/(FP_Fusion_Algorithm + TN_Fusion_Algorithm);
PPV_Fusion = TP_Fusion_Algorithm/(TP_Fusion_Algorithm + FP_Fusion_Algorithm);
NPV_Fusion = TN_Fusion_Algorithm/(TN_Fusion_Algorithm + FN_Fusion_Algorithm);
FPR_Fusion = 1 - SPC_Fusion
FDR_Fusion = 1 - PPV_Fusion;

TPR_Fusion_2imagereset = TP_Fusion_2imagereset_Algorithm/(TP_Fusion_2imagereset_Algorithm + FN_Fusion_2imagereset_Algorithm)
SPC_Fusion_2imagereset = TN_Fusion_2imagereset_Algorithm/(FP_Fusion_2imagereset_Algorithm + TN_Fusion_2imagereset_Algorithm);
PPV_Fusion_2imagereset = TP_Fusion_2imagereset_Algorithm/(TP_Fusion_2imagereset_Algorithm + FP_Fusion_2imagereset_Algorithm);
NPV_Fusion_2imagereset = TN_Fusion_2imagereset_Algorithm/(TN_Fusion_2imagereset_Algorithm + FN_Fusion_2imagereset_Algorithm);
FPR_Fusion_2imagereset = 1 - SPC_Fusion_2imagereset
FDR_Fusion_2imagereset = 1 - PPV_Fusion_2imagereset;
media_tempo_processamento_Fusion_2image = mean(hist_Fusion_2imagereset_Algorithm)
max_tempo_processamento_Fusion_2image = max(hist_Fusion_2imagereset_Algorithm);

TPR_Fusion_2imagereset_1p3 = TP_Fusion_2imagereset_1p3_Algorithm/(TP_Fusion_2imagereset_1p3_Algorithm + FN_Fusion_2imagereset_1p3_Algorithm)
SPC_Fusion_2imagereset_1p3 = TN_Fusion_2imagereset_1p3_Algorithm/(FP_Fusion_2imagereset_1p3_Algorithm + TN_Fusion_2imagereset_1p3_Algorithm);
PPV_Fusion_2imagereset_1p3 = TP_Fusion_2imagereset_1p3_Algorithm/(TP_Fusion_2imagereset_1p3_Algorithm + FP_Fusion_2imagereset_1p3_Algorithm);
NPV_Fusion_2imagereset_1p3 = TN_Fusion_2imagereset_1p3_Algorithm/(TN_Fusion_2imagereset_1p3_Algorithm + FN_Fusion_2imagereset_1p3_Algorithm);
FPR_Fusion_2imagereset_1p3 = 1 - SPC_Fusion_2imagereset_1p3
FDR_Fusion_2imagereset_1p3 = 1 - PPV_Fusion_2imagereset_1p3;
media_tempo_processamento_Fusion_2imagereset_1p3 = mean(hist_Fusion_2imagereset_1p3_Algorithm)
max_tempo_processamento_Fusion_2imagereset_1p3 = max(hist_Fusion_2imagereset_1p3_Algorithm);

TPR_Fusion_4imagereset = TP_Fusion_4imagereset_Algorithm/(TP_Fusion_4imagereset_Algorithm + FN_Fusion_4imagereset_Algorithm)
SPC_Fusion_4imagereset = TN_Fusion_4imagereset_Algorithm/(FP_Fusion_4imagereset_Algorithm + TN_Fusion_4imagereset_Algorithm);
PPV_Fusion_4imagereset = TP_Fusion_4imagereset_Algorithm/(TP_Fusion_4imagereset_Algorithm + FP_Fusion_4imagereset_Algorithm);
NPV_Fusion_4imagereset = TN_Fusion_4imagereset_Algorithm/(TN_Fusion_4imagereset_Algorithm + FN_Fusion_4imagereset_Algorithm);
FPR_Fusion_4imagereset = 1 - SPC_Fusion_4imagereset
FDR_Fusion_4imagereset = 1 - PPV_Fusion_4imagereset;
media_tempo_processamento_Fusion_4imagereset = mean(hist_Fusion_4imagereset_Algorithm)
max_tempo_processamento_Fusion_4imagereset = max(hist_Fusion_4imagereset_Algorithm);

TPR_Fusion_4imagereset_1p3 = TP_Fusion_4imagereset_1p3_Algorithm/(TP_Fusion_4imagereset_1p3_Algorithm + FN_Fusion_4imagereset_1p3_Algorithm)
SPC_Fusion_4imagereset_1p3 = TN_Fusion_4imagereset_1p3_Algorithm/(FP_Fusion_4imagereset_1p3_Algorithm + TN_Fusion_4imagereset_1p3_Algorithm);
PPV_Fusion_4imagereset_1p3 = TP_Fusion_4imagereset_1p3_Algorithm/(TP_Fusion_4imagereset_1p3_Algorithm + FP_Fusion_4imagereset_1p3_Algorithm);
NPV_Fusion_4imagereset_1p3 = TN_Fusion_4imagereset_1p3_Algorithm/(TN_Fusion_4imagereset_1p3_Algorithm + FN_Fusion_4imagereset_1p3_Algorithm);
FPR_Fusion_4imagereset_1p3 = 1 - SPC_Fusion_4imagereset_1p3
FDR_Fusion_4imagereset_1p3 = 1 - PPV_Fusion_4imagereset_1p3;
media_tempo_processamento_Fusion_4imagereset_1p3 = mean(hist_Fusion_4imagereset_1p3_Algorithm)
max_tempo_processamento_Fusion_4imagereset_1p3 = max(hist_Fusion_4imagereset_1p3_Algorithm);

TPR_Fusion_10imagereset = TP_Fusion_10imagereset_Algorithm/(TP_Fusion_10imagereset_Algorithm + FN_Fusion_10imagereset_Algorithm);
SPC_Fusion_10imagereset = TN_Fusion_10imagereset_Algorithm/(FP_Fusion_10imagereset_Algorithm + TN_Fusion_10imagereset_Algorithm);
PPV_Fusion_10imagereset = TP_Fusion_10imagereset_Algorithm/(TP_Fusion_10imagereset_Algorithm + FP_Fusion_10imagereset_Algorithm);
NPV_Fusion_10imagereset = TN_Fusion_10imagereset_Algorithm/(TN_Fusion_10imagereset_Algorithm + FN_Fusion_10imagereset_Algorithm);
FPR_Fusion_10imagereset = 1 - SPC_Fusion_10imagereset;
FDR_Fusion_10imagereset = 1 - PPV_Fusion_10imagereset;
media_tempo_processamento_Fusion_10imagereset = mean(hist_Fusion_10imagereset_Algorithm);
max_tempo_processamento_Fusion_10imagereset = max(hist_Fusion_10imagereset_Algorithm);

TPR_Fusion_6imagereset_1p3 = TP_Fusion_6imagereset_1p3_Algorithm/(TP_Fusion_6imagereset_1p3_Algorithm + FN_Fusion_6imagereset_1p3_Algorithm)
SPC_Fusion_6imagereset_1p3 = TN_Fusion_6imagereset_1p3_Algorithm/(FP_Fusion_6imagereset_1p3_Algorithm + TN_Fusion_6imagereset_1p3_Algorithm);
PPV_Fusion_6imagereset_1p3 = TP_Fusion_6imagereset_1p3_Algorithm/(TP_Fusion_6imagereset_1p3_Algorithm + FP_Fusion_6imagereset_1p3_Algorithm);
NPV_Fusion_6imagereset_1p3 = TN_Fusion_6imagereset_1p3_Algorithm/(TN_Fusion_6imagereset_1p3_Algorithm + FN_Fusion_6imagereset_1p3_Algorithm);
FPR_Fusion_6imagereset_1p3 = 1 - SPC_Fusion_6imagereset_1p3
FDR_Fusion_6imagereset_1p3 = 1 - PPV_Fusion_6imagereset_1p3;
media_tempo_processamento_Fusion_6imagereset_1p3 = mean(hist_Fusion_6imagereset_1p3_Algorithm)
max_tempo_processamento_Fusion_6imagereset_1p3 = max(hist_Fusion_6imagereset_1p3_Algorithm);

TPR_Fusion_8imagereset_1p3 = TP_Fusion_8imagereset_1p3_Algorithm/(TP_Fusion_8imagereset_1p3_Algorithm + FN_Fusion_8imagereset_1p3_Algorithm)
SPC_Fusion_8imagereset_1p3 = TN_Fusion_8imagereset_1p3_Algorithm/(FP_Fusion_8imagereset_1p3_Algorithm + TN_Fusion_8imagereset_1p3_Algorithm);
PPV_Fusion_8imagereset_1p3 = TP_Fusion_8imagereset_1p3_Algorithm/(TP_Fusion_8imagereset_1p3_Algorithm + FP_Fusion_8imagereset_1p3_Algorithm);
NPV_Fusion_8imagereset_1p3 = TN_Fusion_8imagereset_1p3_Algorithm/(TN_Fusion_8imagereset_1p3_Algorithm + FN_Fusion_8imagereset_1p3_Algorithm);
FPR_Fusion_8imagereset_1p3 = 1 - SPC_Fusion_8imagereset_1p3
FDR_Fusion_8imagereset_1p3 = 1 - PPV_Fusion_8imagereset_1p3;
media_tempo_processamento_Fusion_8imagereset_1p3 = mean(hist_Fusion_8imagereset_1p3_Algorithm)
max_tempo_processamento_Fusion_8imagereset_1p3 = max(hist_Fusion_8imagereset_1p3_Algorithm);

TPR_Fusion_10imagereset_1p3 = TP_Fusion_10imagereset_1p3_Algorithm/(TP_Fusion_10imagereset_1p3_Algorithm + FN_Fusion_10imagereset_1p3_Algorithm)
SPC_Fusion_10imagereset_1p3 = TN_Fusion_10imagereset_1p3_Algorithm/(FP_Fusion_10imagereset_1p3_Algorithm + TN_Fusion_10imagereset_1p3_Algorithm);
PPV_Fusion_10imagereset_1p3 = TP_Fusion_10imagereset_1p3_Algorithm/(TP_Fusion_10imagereset_1p3_Algorithm + FP_Fusion_10imagereset_1p3_Algorithm);
NPV_Fusion_10imagereset_1p3 = TN_Fusion_10imagereset_1p3_Algorithm/(TN_Fusion_10imagereset_1p3_Algorithm + FN_Fusion_10imagereset_1p3_Algorithm);
FPR_Fusion_10imagereset_1p3 = 1 - SPC_Fusion_10imagereset_1p3
FDR_Fusion_10imagereset_1p3 = 1 - PPV_Fusion_10imagereset_1p3;
media_tempo_processamento_Fusion_10imagereset_1p3 = mean(hist_Fusion_10imagereset_1p3_Algorithm)
max_tempo_processamento_Fusion_10imagereset_1p3 = max(hist_Fusion_10imagereset_1p3_Algorithm);

TPR_Fusion_20imagereset_1p3 = TP_Fusion_20imagereset_1p3_Algorithm/(TP_Fusion_20imagereset_1p3_Algorithm + FN_Fusion_20imagereset_1p3_Algorithm)
SPC_Fusion_20imagereset_1p3 = TN_Fusion_20imagereset_1p3_Algorithm/(FP_Fusion_20imagereset_1p3_Algorithm + TN_Fusion_20imagereset_1p3_Algorithm);
PPV_Fusion_20imagereset_1p3 = TP_Fusion_20imagereset_1p3_Algorithm/(TP_Fusion_20imagereset_1p3_Algorithm + FP_Fusion_20imagereset_1p3_Algorithm);
NPV_Fusion_20imagereset_1p3 = TN_Fusion_20imagereset_1p3_Algorithm/(TN_Fusion_20imagereset_1p3_Algorithm + FN_Fusion_20imagereset_1p3_Algorithm);
FPR_Fusion_20imagereset_1p3 = 1 - SPC_Fusion_20imagereset_1p3
FDR_Fusion_20imagereset_1p3 = 1 - PPV_Fusion_20imagereset_1p3;
media_tempo_processamento_Fusion_20imagereset_1p3 = mean(hist_Fusion_20imagereset_1p3_Algorithm)
max_tempo_processamento_Fusion_20imagereset_1p3 = max(hist_Fusion_20imagereset_1p3_Algorithm);

TPR_Fusion_infimagereset_1p3 = TP_Fusion_infimagereset_1p3_Algorithm/(TP_Fusion_infimagereset_1p3_Algorithm + FN_Fusion_infimagereset_1p3_Algorithm)
SPC_Fusion_infimagereset_1p3 = TN_Fusion_infimagereset_1p3_Algorithm/(FP_Fusion_infimagereset_1p3_Algorithm + TN_Fusion_infimagereset_1p3_Algorithm);
PPV_Fusion_infimagereset_1p3 = TP_Fusion_infimagereset_1p3_Algorithm/(TP_Fusion_infimagereset_1p3_Algorithm + FP_Fusion_infimagereset_1p3_Algorithm);
NPV_Fusion_infimagereset_1p3 = TN_Fusion_infimagereset_1p3_Algorithm/(TN_Fusion_infimagereset_1p3_Algorithm + FN_Fusion_infimagereset_1p3_Algorithm);
FPR_Fusion_infimagereset_1p3 = 1 - SPC_Fusion_infimagereset_1p3
FDR_Fusion_infimagereset_1p3 = 1 - PPV_Fusion_infimagereset_1p3;
media_tempo_processamento_Fusion_infimagereset_1p3 = mean(hist_Fusion_infimagereset_1p3_Algorithm)
max_tempo_processamento_Fusion_infimagereset_1p3 = max(hist_Fusion_infimagereset_1p3_Algorithm);

QPedro = TP_Pedro_Algorithm/num_BB_positive;
Q2 = TP_Fusion_2imagereset_1p3_Algorithm/num_BB_positive;
Q4 = TP_Fusion_4imagereset_1p3_Algorithm/num_BB_positive;
Q6 = TP_Fusion_6imagereset_1p3_Algorithm/num_BB_positive;
Q8 = TP_Fusion_8imagereset_1p3_Algorithm/num_BB_positive;
Q10 = TP_Fusion_10imagereset_1p3_Algorithm/num_BB_positive;
Q20 = TP_Fusion_20imagereset_1p3_Algorithm/num_BB_positive;
Qinf = TP_Fusion_infimagereset_1p3_Algorithm/num_BB_positive;

QPedro_n = 1-TN_Pedro_Algorithm/num_BB_negative;
Q2_n = 1-TN_Fusion_2imagereset_1p3_Algorithm/num_BB_negative;
Q4_n = 1-TN_Fusion_4imagereset_1p3_Algorithm/num_BB_negative;
Q6_n = 1-TN_Fusion_6imagereset_1p3_Algorithm/num_BB_negative;
Q8_n = 1-TN_Fusion_8imagereset_1p3_Algorithm/num_BB_negative;
Q10_n = 1-TN_Fusion_10imagereset_1p3_Algorithm/num_BB_negative;
Q20_n = 1-TN_Fusion_20imagereset_1p3_Algorithm/num_BB_negative;
Qinf_n = 1-TN_Fusion_infimagereset_1p3_Algorithm/num_BB_negative;

figure;
x1_1p3m_tpr=[2, 4, 6, 8, 10, 20];
y1_1p3m_tpr=[TPR_Fusion_2imagereset_1p3, TPR_Fusion_4imagereset_1p3, TPR_Fusion_6imagereset_1p3, TPR_Fusion_8imagereset_1p3, TPR_Fusion_10imagereset_1p3, TPR_Fusion_20imagereset_1p3];
x1_1p3m_fpr=[2, 4, 6, 8, 10, 20];
y1_1p3m_fpr=[FPR_Fusion_2imagereset_1p3, FPR_Fusion_4imagereset_1p3, FPR_Fusion_6imagereset_1p3, FPR_Fusion_8imagereset_1p3, FPR_Fusion_10imagereset_1p3, FPR_Fusion_20imagereset_1p3];
x1_tpr_inf=[0, 40];
y1_tpr_inf=[TPR_Fusion_infimagereset_1p3, TPR_Fusion_infimagereset_1p3];
x1_fpr_inf=[0, 40];
y1_fpr_inf=[FPR_Fusion_infimagereset_1p3, FPR_Fusion_infimagereset_1p3];
xpedro_tpr=[1,20];
ypedro_tpr=[TPR_Pedro, TPR_Pedro];
xpedro_fpr=[1,20];
ypedro_fpr=[FPR_Pedro, FPR_Pedro];
plot(x1_1p3m_tpr, y1_1p3m_tpr, '-ob',  x1_1p3m_fpr, y1_1p3m_fpr, '--ob', x1_tpr_inf, y1_tpr_inf, '--k',  x1_fpr_inf, y1_fpr_inf, '-.k', xpedro_tpr, ypedro_tpr, '-r', xpedro_fpr, ypedro_fpr, '--r')
set(gca, 'xlim', [1 21], 'ylim', [0 1]);

TPR_ATLASCAR_Pedro = TP_ATLASCAR_Pedro_Algorithm/(TP_ATLASCAR_Pedro_Algorithm + FN_ATLASCAR_Pedro_Algorithm)
SPC_ATLASCAR_Pedro = TN_ATLASCAR_Pedro_Algorithm/(FP_ATLASCAR_Pedro_Algorithm + TN_ATLASCAR_Pedro_Algorithm);
PPV_ATLASCAR_Pedro = TP_ATLASCAR_Pedro_Algorithm/(TP_ATLASCAR_Pedro_Algorithm + FP_ATLASCAR_Pedro_Algorithm);
NPV_ATLASCAR_Pedro = TN_ATLASCAR_Pedro_Algorithm/(TN_ATLASCAR_Pedro_Algorithm + FN_ATLASCAR_Pedro_Algorithm);
FPR_ATLASCAR_Pedro = 1 - SPC_ATLASCAR_Pedro
FDR_ATLASCAR_Pedro = 1 - PPV_ATLASCAR_Pedro;
media_tempo_processamento_ATLASCAR_Pedro = mean(hist_ATLASCAR_Pedro_Algorithm)
max_tempo_processamento_ATLASCAR_Pedro = max(hist_ATLASCAR_Pedro_Algorithm);

TPR_ATLASCAR_Fusion_2imagereset_1p3 = TP_ATLASCAR_Fusion_2imagereset_1p3_Algorithm/(TP_ATLASCAR_Fusion_2imagereset_1p3_Algorithm + FN_ATLASCAR_Fusion_2imagereset_1p3_Algorithm)
SPC_ATLASCAR_Fusion_2imagereset_1p3 = TN_ATLASCAR_Fusion_2imagereset_1p3_Algorithm/(FP_ATLASCAR_Fusion_2imagereset_1p3_Algorithm + TN_ATLASCAR_Fusion_2imagereset_1p3_Algorithm);
PPV_ATLASCAR_Fusion_2imagereset_1p3 = TP_ATLASCAR_Fusion_2imagereset_1p3_Algorithm/(TP_ATLASCAR_Fusion_2imagereset_1p3_Algorithm + FP_ATLASCAR_Fusion_2imagereset_1p3_Algorithm);
NPV_ATLASCAR_Fusion_2imagereset_1p3 = TN_ATLASCAR_Fusion_2imagereset_1p3_Algorithm/(TN_ATLASCAR_Fusion_2imagereset_1p3_Algorithm + FN_ATLASCAR_Fusion_2imagereset_1p3_Algorithm);
FPR_ATLASCAR_Fusion_2imagereset_1p3 = 1 - SPC_ATLASCAR_Fusion_2imagereset_1p3
FDR_ATLASCAR_Fusion_2imagereset_1p3 = 1 - PPV_ATLASCAR_Fusion_2imagereset_1p3;
media_tempo_processamento_ATLASCAR_Fusion_2imagereset_1p3 = mean(hist_ATLASCAR_Fusion_2imagereset_1p3_Algorithm)
max_tempo_processamento_ATLASCAR_Fusion_2imagereset_1p3 = max(hist_ATLASCAR_Fusion_2imagereset_1p3_Algorithm);

TPR_ATLASCAR_Fusion_4imagereset_1p3 = TP_ATLASCAR_Fusion_4imagereset_1p3_Algorithm/(TP_ATLASCAR_Fusion_4imagereset_1p3_Algorithm + FN_ATLASCAR_Fusion_4imagereset_1p3_Algorithm)
SPC_ATLASCAR_Fusion_4imagereset_1p3 = TN_ATLASCAR_Fusion_4imagereset_1p3_Algorithm/(FP_ATLASCAR_Fusion_4imagereset_1p3_Algorithm + TN_ATLASCAR_Fusion_4imagereset_1p3_Algorithm);
PPV_ATLASCAR_Fusion_4imagereset_1p3 = TP_ATLASCAR_Fusion_4imagereset_1p3_Algorithm/(TP_ATLASCAR_Fusion_4imagereset_1p3_Algorithm + FP_ATLASCAR_Fusion_4imagereset_1p3_Algorithm);
NPV_ATLASCAR_Fusion_4imagereset_1p3 = TN_ATLASCAR_Fusion_4imagereset_1p3_Algorithm/(TN_ATLASCAR_Fusion_4imagereset_1p3_Algorithm + FN_ATLASCAR_Fusion_4imagereset_1p3_Algorithm);
FPR_ATLASCAR_Fusion_4imagereset_1p3 = 1 - SPC_ATLASCAR_Fusion_4imagereset_1p3
FDR_ATLASCAR_Fusion_4imagereset_1p3 = 1 - PPV_ATLASCAR_Fusion_4imagereset_1p3;
media_tempo_processamento_ATLASCAR_Fusion_4imagereset_1p3 = mean(hist_ATLASCAR_Fusion_4imagereset_1p3_Algorithm)
max_tempo_processamento_ATLASCAR_Fusion_4imagereset_1p3 = max(hist_ATLASCAR_Fusion_4imagereset_1p3_Algorithm);

TPR_ATLASCAR_Fusion_6imagereset_1p3 = TP_ATLASCAR_Fusion_6imagereset_1p3_Algorithm/(TP_ATLASCAR_Fusion_6imagereset_1p3_Algorithm + FN_ATLASCAR_Fusion_6imagereset_1p3_Algorithm)
SPC_ATLASCAR_Fusion_6imagereset_1p3 = TN_ATLASCAR_Fusion_6imagereset_1p3_Algorithm/(FP_ATLASCAR_Fusion_6imagereset_1p3_Algorithm + TN_ATLASCAR_Fusion_6imagereset_1p3_Algorithm);
PPV_ATLASCAR_Fusion_6imagereset_1p3 = TP_ATLASCAR_Fusion_6imagereset_1p3_Algorithm/(TP_ATLASCAR_Fusion_6imagereset_1p3_Algorithm + FP_ATLASCAR_Fusion_6imagereset_1p3_Algorithm);
NPV_ATLASCAR_Fusion_6imagereset_1p3 = TN_ATLASCAR_Fusion_6imagereset_1p3_Algorithm/(TN_ATLASCAR_Fusion_6imagereset_1p3_Algorithm + FN_ATLASCAR_Fusion_6imagereset_1p3_Algorithm);
FPR_ATLASCAR_Fusion_6imagereset_1p3 = 1 - SPC_ATLASCAR_Fusion_6imagereset_1p3
FDR_ATLASCAR_Fusion_6imagereset_1p3 = 1 - PPV_ATLASCAR_Fusion_6imagereset_1p3;
media_tempo_processamento_ATLASCAR_Fusion_6imagereset_1p3 = mean(hist_ATLASCAR_Fusion_6imagereset_1p3_Algorithm)
max_tempo_processamento_ATLASCAR_Fusion_6imagereset_1p3 = max(hist_ATLASCAR_Fusion_6imagereset_1p3_Algorithm);

TPR_ATLASCAR_Fusion_8imagereset_1p3 = TP_ATLASCAR_Fusion_8imagereset_1p3_Algorithm/(TP_ATLASCAR_Fusion_8imagereset_1p3_Algorithm + FN_ATLASCAR_Fusion_8imagereset_1p3_Algorithm)
SPC_ATLASCAR_Fusion_8imagereset_1p3 = TN_ATLASCAR_Fusion_8imagereset_1p3_Algorithm/(FP_ATLASCAR_Fusion_8imagereset_1p3_Algorithm + TN_ATLASCAR_Fusion_8imagereset_1p3_Algorithm);
PPV_ATLASCAR_Fusion_8imagereset_1p3 = TP_ATLASCAR_Fusion_8imagereset_1p3_Algorithm/(TP_ATLASCAR_Fusion_8imagereset_1p3_Algorithm + FP_ATLASCAR_Fusion_8imagereset_1p3_Algorithm);
NPV_ATLASCAR_Fusion_8imagereset_1p3 = TN_ATLASCAR_Fusion_8imagereset_1p3_Algorithm/(TN_ATLASCAR_Fusion_8imagereset_1p3_Algorithm + FN_ATLASCAR_Fusion_8imagereset_1p3_Algorithm);
FPR_ATLASCAR_Fusion_8imagereset_1p3 = 1 - SPC_ATLASCAR_Fusion_8imagereset_1p3
FDR_ATLASCAR_Fusion_8imagereset_1p3 = 1 - PPV_ATLASCAR_Fusion_8imagereset_1p3;
media_tempo_processamento_ATLASCAR_Fusion_8imagereset_1p3 = mean(hist_ATLASCAR_Fusion_8imagereset_1p3_Algorithm)
max_tempo_processamento_ATLASCAR_Fusion_8imagereset_1p3 = max(hist_ATLASCAR_Fusion_8imagereset_1p3_Algorithm);

TPR_ATLASCAR_Fusion_10imagereset_1p3 = TP_ATLASCAR_Fusion_10imagereset_1p3_Algorithm/(TP_ATLASCAR_Fusion_10imagereset_1p3_Algorithm + FN_ATLASCAR_Fusion_10imagereset_1p3_Algorithm)
SPC_ATLASCAR_Fusion_10imagereset_1p3 = TN_ATLASCAR_Fusion_10imagereset_1p3_Algorithm/(FP_ATLASCAR_Fusion_10imagereset_1p3_Algorithm + TN_ATLASCAR_Fusion_10imagereset_1p3_Algorithm);
PPV_ATLASCAR_Fusion_10imagereset_1p3 = TP_ATLASCAR_Fusion_10imagereset_1p3_Algorithm/(TP_ATLASCAR_Fusion_10imagereset_1p3_Algorithm + FP_ATLASCAR_Fusion_10imagereset_1p3_Algorithm);
NPV_ATLASCAR_Fusion_10imagereset_1p3 = TN_ATLASCAR_Fusion_10imagereset_1p3_Algorithm/(TN_ATLASCAR_Fusion_10imagereset_1p3_Algorithm + FN_ATLASCAR_Fusion_10imagereset_1p3_Algorithm);
FPR_ATLASCAR_Fusion_10imagereset_1p3 = 1 - SPC_ATLASCAR_Fusion_10imagereset_1p3
FDR_ATLASCAR_Fusion_10imagereset_1p3 = 1 - PPV_ATLASCAR_Fusion_10imagereset_1p3;
media_tempo_processamento_ATLASCAR_Fusion_10imagereset_1p3 = mean(hist_ATLASCAR_Fusion_10imagereset_1p3_Algorithm)
max_tempo_processamento_ATLASCAR_Fusion_10imagereset_1p3 = max(hist_ATLASCAR_Fusion_10imagereset_1p3_Algorithm);

TPR_ATLASCAR_Fusion_20imagereset_1p3 = TP_ATLASCAR_Fusion_20imagereset_1p3_Algorithm/(TP_ATLASCAR_Fusion_20imagereset_1p3_Algorithm + FN_ATLASCAR_Fusion_20imagereset_1p3_Algorithm)
SPC_ATLASCAR_Fusion_20imagereset_1p3 = TN_ATLASCAR_Fusion_20imagereset_1p3_Algorithm/(FP_ATLASCAR_Fusion_20imagereset_1p3_Algorithm + TN_ATLASCAR_Fusion_20imagereset_1p3_Algorithm);
PPV_ATLASCAR_Fusion_20imagereset_1p3 = TP_ATLASCAR_Fusion_20imagereset_1p3_Algorithm/(TP_ATLASCAR_Fusion_20imagereset_1p3_Algorithm + FP_ATLASCAR_Fusion_20imagereset_1p3_Algorithm);
NPV_ATLASCAR_Fusion_20imagereset_1p3 = TN_ATLASCAR_Fusion_20imagereset_1p3_Algorithm/(TN_ATLASCAR_Fusion_20imagereset_1p3_Algorithm + FN_ATLASCAR_Fusion_20imagereset_1p3_Algorithm);
FPR_ATLASCAR_Fusion_20imagereset_1p3 = 1 - SPC_ATLASCAR_Fusion_20imagereset_1p3
FDR_ATLASCAR_Fusion_20imagereset_1p3 = 1 - PPV_ATLASCAR_Fusion_20imagereset_1p3;
media_tempo_processamento_ATLASCAR_Fusion_20imagereset_1p3 = mean(hist_ATLASCAR_Fusion_20imagereset_1p3_Algorithm)
max_tempo_processamento_ATLASCAR_Fusion_20imagereset_1p3 = max(hist_ATLASCAR_Fusion_20imagereset_1p3_Algorithm);

TPR_ATLASCAR_Fusion_40imagereset_1p3 = TP_ATLASCAR_Fusion_40imagereset_1p3_Algorithm/(TP_ATLASCAR_Fusion_40imagereset_1p3_Algorithm + FN_ATLASCAR_Fusion_40imagereset_1p3_Algorithm)
SPC_ATLASCAR_Fusion_40imagereset_1p3 = TN_ATLASCAR_Fusion_40imagereset_1p3_Algorithm/(FP_ATLASCAR_Fusion_40imagereset_1p3_Algorithm + TN_ATLASCAR_Fusion_40imagereset_1p3_Algorithm);
PPV_ATLASCAR_Fusion_40imagereset_1p3 = TP_ATLASCAR_Fusion_40imagereset_1p3_Algorithm/(TP_ATLASCAR_Fusion_40imagereset_1p3_Algorithm + FP_ATLASCAR_Fusion_40imagereset_1p3_Algorithm);
NPV_ATLASCAR_Fusion_40imagereset_1p3 = TN_ATLASCAR_Fusion_40imagereset_1p3_Algorithm/(TN_ATLASCAR_Fusion_40imagereset_1p3_Algorithm + FN_ATLASCAR_Fusion_40imagereset_1p3_Algorithm);
FPR_ATLASCAR_Fusion_40imagereset_1p3 = 1 - SPC_ATLASCAR_Fusion_40imagereset_1p3
FDR_ATLASCAR_Fusion_40imagereset_1p3 = 1 - PPV_ATLASCAR_Fusion_40imagereset_1p3;
media_tempo_processamento_ATLASCAR_Fusion_40imagereset_1p3 = mean(hist_ATLASCAR_Fusion_40imagereset_1p3_Algorithm)
max_tempo_processamento_ATLASCAR_Fusion_40imagereset_1p3 = max(hist_ATLASCAR_Fusion_40imagereset_1p3_Algorithm);

TPR_ATLASCAR_Fusion_infimagereset_1p3 = TP_ATLASCAR_Fusion_infimagereset_1p3_Algorithm/(TP_ATLASCAR_Fusion_infimagereset_1p3_Algorithm + FN_ATLASCAR_Fusion_infimagereset_1p3_Algorithm)
SPC_ATLASCAR_Fusion_infimagereset_1p3 = TN_ATLASCAR_Fusion_infimagereset_1p3_Algorithm/(FP_ATLASCAR_Fusion_infimagereset_1p3_Algorithm + TN_ATLASCAR_Fusion_infimagereset_1p3_Algorithm);
PPV_ATLASCAR_Fusion_infimagereset_1p3 = TP_ATLASCAR_Fusion_infimagereset_1p3_Algorithm/(TP_ATLASCAR_Fusion_infimagereset_1p3_Algorithm + FP_ATLASCAR_Fusion_infimagereset_1p3_Algorithm);
NPV_ATLASCAR_Fusion_infimagereset_1p3 = TN_ATLASCAR_Fusion_infimagereset_1p3_Algorithm/(TN_ATLASCAR_Fusion_infimagereset_1p3_Algorithm + FN_ATLASCAR_Fusion_infimagereset_1p3_Algorithm);
FPR_ATLASCAR_Fusion_infimagereset_1p3 = 1 - SPC_ATLASCAR_Fusion_infimagereset_1p3
FDR_ATLASCAR_Fusion_infimagereset_1p3 = 1 - PPV_ATLASCAR_Fusion_infimagereset_1p3;
media_tempo_processamento_ATLASCAR_Fusion_infimagereset_1p3 = mean(hist_ATLASCAR_Fusion_infimagereset_1p3_Algorithm)
max_tempo_processamento_ATLASCAR_Fusion_infimagereset_1p3 = max(hist_ATLASCAR_Fusion_infimagereset_1p3_Algorithm);

figure;
x1_ATLASCAR_tpr=[2,4,6,8,10,20,40];
y1_ATLASCAR_tpr=[TPR_ATLASCAR_Fusion_2imagereset_1p3, TPR_ATLASCAR_Fusion_4imagereset_1p3, TPR_ATLASCAR_Fusion_6imagereset_1p3,TPR_ATLASCAR_Fusion_8imagereset_1p3, TPR_ATLASCAR_Fusion_10imagereset_1p3, TPR_ATLASCAR_Fusion_20imagereset_1p3, TPR_ATLASCAR_Fusion_40imagereset_1p3];
x1_ATLASCAR_fpr=[2,4,6,8,10,20,40];
y1_ATLASCAR_fpr=[FPR_ATLASCAR_Fusion_2imagereset_1p3, FPR_ATLASCAR_Fusion_4imagereset_1p3, FPR_ATLASCAR_Fusion_6imagereset_1p3, FPR_ATLASCAR_Fusion_8imagereset_1p3, FPR_ATLASCAR_Fusion_10imagereset_1p3, FPR_ATLASCAR_Fusion_20imagereset_1p3, FPR_ATLASCAR_Fusion_40imagereset_1p3];
x1_ATLASCAR_tpr_inf=[0, 40];
y1_ATLASCAR_tpr_inf=[TPR_ATLASCAR_Fusion_infimagereset_1p3, TPR_ATLASCAR_Fusion_infimagereset_1p3];
x1_ATLASCAR_fpr_inf=[0, 40];
y1_ATLASCAR_fpr_inf=[FPR_ATLASCAR_Fusion_infimagereset_1p3, FPR_ATLASCAR_Fusion_infimagereset_1p3];
xpedro_ATLASCAR_tpr=[0,40];
ypedro_ATLASCAR_tpr=[TPR_ATLASCAR_Pedro, TPR_ATLASCAR_Pedro];
xpedro_ATLASCAR_fpr=[0,40];
ypedro_ATLASCAR_fpr=[FPR_ATLASCAR_Pedro, FPR_ATLASCAR_Pedro];
% BreakXAxis(x1_ATLASCAR_tpr,y1_ATLASCAR_tpr,50,170,120)
% hold on
% BreakXAxis(x1_ATLASCAR_fpr,y1_ATLASCAR_fpr,50,170,120)

plot( x1_ATLASCAR_tpr, y1_ATLASCAR_tpr, '-ob',  x1_ATLASCAR_fpr, y1_ATLASCAR_fpr, '--ob', x1_ATLASCAR_tpr_inf, y1_ATLASCAR_tpr_inf, '--k',  x1_ATLASCAR_fpr_inf, y1_ATLASCAR_fpr_inf, '-.k', xpedro_ATLASCAR_tpr, ypedro_ATLASCAR_tpr, '-r', xpedro_ATLASCAR_fpr, ypedro_ATLASCAR_fpr, '--r')
set(gca, 'xlim', [0 41], 'ylim', [0 0.7]);
% legend(1, g_range[2], c('cars','trucks'), cex=0.8, col=c('blue','red'), pch=21:22, lty=1:2);

%plotyy try
% [hAx,hLine1,hLine2] = plotyy(x1_ATLASCAR_tpr, y1_ATLASCAR_tpr,x1_ATLASCAR_fpr, y1_ATLASCAR_fpr);
% hold on;
% set(hAx,'NextPlot','add');
% plot(hAx(1),0:41,0:1);
% plot(hAx(2),0:41,0:0.3);
% [hAx1,hLine3,hLine4] = plotyy(x1_ATLASCAR_tpr_inf, y1_ATLASCAR_tpr_inf,x1_ATLASCAR_fpr_inf, y1_ATLASCAR_fpr_inf);
% set(hAx1,'NextPlot','add');
% plot(hAx1(1),0:41,0:1);
% plot(hAx1(2),0:41,0:0.3);
% [hAx2,hLine5,hLine6] = plotyy(xpedro_ATLASCAR_tpr, ypedro_ATLASCAR_tpr,xpedro_ATLASCAR_fpr, ypedro_ATLASCAR_fpr);
% set(hAx2,'NextPlot','add');
% plot(hAx2(1),0:41,0:1);
% plot(hAx2(2),0:41,0:0.3);
% 
% 
% set(hAx(1),'YColor','r');
% set(hAx(2),'YColor','b');
% set(hLine1,'marker','o','color','red');
% set(hLine2,'marker','o','color','blue');
% set(hLine3,'marker','o','color','red');
% set(hLine4,'marker','o','color','blue');
% set(hLine5,'marker','o','color','red');
% set(hLine6,'marker','o','color','blue');
