dados intercalados na ordem:
"upperLegAngle
---
lowerLegAngle
---
footAngle
---
upperLegAngle
---
lowerLegAngle
---
..."

valor do nome do arquivo igual a 0,1km/h
	walk04.txt = 0,4km/h

sampling frequency
	15Hz


fs = 15;
Ts = 1/fs;
t = [1:length(sample)] * Ts;
