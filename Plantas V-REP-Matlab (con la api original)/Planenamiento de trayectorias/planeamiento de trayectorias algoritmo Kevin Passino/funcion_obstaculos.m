%  Obstacle function:
% Author: K. Passino, Version: 1/25/01
function J=funcion_obstaculos(x,A,posicionobs,w1)
     

	J=...
		w1*max([exp(-A*((x(1,1)-posicionobs(1,1))^2+(x(2,1)-posicionobs(2,1))^2)),... %2 y 6 
		exp(-A*((x(1,1)-posicionobs(1,2))^2+(x(2,1)-posicionobs(2,2))^2)),...          %2 y 4 
		exp(-A*((x(1,1)-posicionobs(1,3))^2+(x(2,1)-posicionobs(2,3))^2)),...         % -1.225 y 5.5
		exp(-A*((x(1,1)-posicionobs(1,4))^2+(x(2,1)-posicionobs(2,4))^2)),...         %-0.1 y 4.25
		exp(-A*((x(1,1)-posicionobs(1,5))^2+(x(2,1)-posicionobs(2,5))^2)),...         %2 y 3.875
		exp(-A*((x(1,1)-posicionobs(1,6))^2+(x(2,1)-posicionobs(2,6))^2))]);          %3.645 y 3.85
    
