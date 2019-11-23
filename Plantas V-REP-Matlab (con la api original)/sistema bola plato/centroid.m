function [uv_seg]=centroid(imgd)
[regiones,num_regiones]=bwlabel(imgd);
objetos=regionprops(regiones,'all');
pos=1;
uv_seg=0;
for i=1:num_regiones
    aux=objetos(i).Area;
    if aux<200
     uv_seg(1,pos)=objetos(i).Centroid(1);
     uv_seg(2,pos)=objetos(i).Centroid(2);
     pos=pos+1;
    end
end
