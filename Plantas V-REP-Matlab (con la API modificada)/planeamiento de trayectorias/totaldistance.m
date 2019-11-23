function totaldistance=totaldistance(x,y)
acum=0;
tam=size(x);
for i=1: tam(2)-1
dist= ( (x(i)-x(i+1)).^2 + (y(i)-y(i+1)).^2  ).^(1/2);
acum=dist+acum;
end
totaldistance=acum;
end
