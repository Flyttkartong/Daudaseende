function resized_image = resizeFirst(image1, image2)

scale = size(image2, 1)/size(image1, 1);
resized_image = imresize(image1, scale);

end
