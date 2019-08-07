img_right = rgb2gray(imread('images/images/right/im1.png'));
img_left = rgb2gray(imread('images/images/left/im0.png'));
% ind = 1;
% my_med = [];
% matlab_med = [];
% D_l = 'images/images/left/';
% D_r = 'images/images/right/';
% S_l = dir(fullfile(D_l,'/*.jpg')); % pattern to match filenames.
% S_r = dir(fullfile(D_r,'/*.jpg'));
% for k = 1:numel(S_l)
%     if k>2
%         break;
%     end
%     F_l = fullfile(D_l,S_l(k).name);
%     F_r = fullfile(D_r,S_r(k).name);
%     img_left = rgb2gray(imread(F_l));
     img_left = imgaussfilt(img_left,2);
%     img_right = rgb2gray(imread(F_r));
     img_right = imgaussfilt(img_right,2);
imshow(img_right);
    
    


% pause(2)
% K_left = [720.61939972 ,0, 329.09473525;
%           0, 718.2679432, 241.2749902;
%           0,0,1];
% 
% K_right = [721.76305553 ,0, 325.73071597;
%            0, 719.57667072, 232.59879978 ;
%            0,0,1];
% 
% dist_left = [0.07878318896465795, -0.10417825415719467, -0.0015292082838016885, -0.0010984804971462619];
% dist_right = [0.07035790609032969, -0.07374635923597579, -0.0014574404464373864, -0.0009576149044082899]; 

[img_ht,img_wd] = size(img_left);
% img_wd = size(img_right)(2);

%Extrinsics
% R = [0.9999739907353993, -0.001895728766010304, 0.00695874019920964;
%      0.0019027400594663194, 0.9999976887168724, -0.0010010700145185365;
%     -0.006956826358367533, 0.0010142846511641023, 0.9999752865914541];
% 
% T = [-0.059536582166935276, -0.00023160362075908626,-0.00010302428441691986];
% figure(11);
% montage(img_left);
% [x,y] = blockMatching(img_left(1:5,1:5), img_right, 1);
disparity_cell = {};
disp_max = -10000;
disp_min = 1000000;
k=1;
patch_size =10;
% patch_size =55;
catch_count = 0;
all_disp_val = [];
count_i = 0;


% dispImg=blockmatching_m(img_left,img_right,5,100);


disparity_map = zeros(ceil((480-patch_size)/patch_size),ceil((640-patch_size)/patch_size));
 for i = 1: patch_size: img_ht-(patch_size)
     count_i = count_i+1;count_j  = 0;
     for j = 1: patch_size: img_wd-(patch_size)
         count_j = (count_j+1);
         left_patch = img_left(i:i+patch_size-1,j:j+patch_size-1);
         try
%              [c_patch,r_patch ]= blockMatching(left_patch,img_right, i);
             [x_patch,y_patch ]= blockMatching(left_patch,img_right, i);
         catch exception
             figure(7);imshow(left_patch);
             catch_count = catch_count+1;
         end
%              if c_patch == NaN
            if y_patch == NaN
                 disp_val = NaN;
             else
%                  disp_val = abs(c_patch - j);
                 disp_val = abs(y_patch - j);
%                figure(1);
%                imshow(left_patch);
%                figure(2);
%                imshow(img_right(r_patch:r_patch+patch_size-1, c_patch:c_patch+patch_size-1));
%                pause(2);
             end
             disparity_map(count_i,count_j) = disp_val;
%              disp(y_patch);disp("i");disp(i);disp("j");disp(j);
             disparity_cell{k} = [i,j,disp_val];
             all_disp_val = [all_disp_val disp_val];
              if disp_max<disp_val
%                   disp(disp_val);
                  disp_max = disp_val;
              end
              if disp_min>disp_val
                  disp_min = disp_val;
              end
             k=k+1;
%              prev_y = y_patch;
         end
 end
 
%  all_disp_val = cell2mat(disparity_map);
%  all_disp_val = all_disp_val(3:3:end);
% disp("median = ");disp(median(A(~isnan(A))));
% figure("disp_map");
figure(12);
image(disparity_map,'CDataMapping','scaled')
colorbar;
median_disparity = median(disparity_map(~isnan(disparity_map(:))));
% my_med = [my_med median_disparity];

matlab_disparityMap = disparityBM(img_left,img_right);
matlab_disp_median = median(matlab_disparityMap(~isnan(matlab_disparityMap(:))));
% matlab_med = [matlab_med matlab_disp_median];
ind = ind +1;
% end
figure;imshow(matlab_disparityMap,[0,64]);title('Disparity Map');colormap jet;colorbar;

function [x_patch, y_patch] = blockMatching(patch, img,row_num)
    [patch_size,~] = size(patch);
    img_row = img(row_num+0:row_num+patch_size-1,:);
    c = normxcorr2(patch, img_row);
%     figure(3), surf(c), shading flat;
    [x_patch,y_patch] = find(c==max(c(:)));
    aa = size(y_patch);
    if aa(1)> 1
        y_patch = NaN;
    end
    x_patch = x_patch-(patch_size-1);
    y_patch = y_patch-(patch_size-1);
    % disp(size(img_row))
    % disp(size(patch))
end


function [c_patch, r_patch] = SSD_blockMatching(patch, img, row_num)
    [patch_size,~] = size(patch);
    sec_min_ssd_score = 1000000;
    min_ssd_score = 10000000;
    [h,w] = size(img);
%     disp("w = ");disp(w);
    for i = 1:patch_size:w-patch_size
%         disp(i);
        patch_2 = img(row_num:row_num+patch_size-1,i:i+patch_size-1);
        ssd = patch_SSD(patch,patch_2);
        if ssd< min_ssd_score
            sec_min_ssd_score = min_ssd_score;
            min_ssd_score = ssd;
            r_patch = row_num;
            c_patch = i;
        elseif ssd < sec_min_ssd_score
            sec_min_ssd_score = ssd;
        end
    end
    
    if min_ssd_score == sec_min_ssd_score
        r_patch = NaN;
        c_patch = NaN;
    end
end


function ssd = patch_SSD(I1,I2)
    diff_ = I1 - I2;
    ssd = sum(diff_(:).^2);
end

