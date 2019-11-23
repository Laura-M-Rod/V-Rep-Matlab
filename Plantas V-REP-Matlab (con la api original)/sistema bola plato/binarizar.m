function imgd=binarizar(uv_mat)
imGray=rgb2gray(uv_mat);
%Binarización de la imagen
imgd=imGray<=100;
[col,fila]=size(imgd);
% Se invierte los colores de la imagen
for i=1:col
    for j=1:fila
        if imgd(i,j)==0;
            imgd(i,j)=0;
        else
            imgd(i,j)=1;
        end
    end
end
