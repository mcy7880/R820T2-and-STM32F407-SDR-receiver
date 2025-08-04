%Converts coefficients from Filter Designer (fdatool) to form used in
%STM32.
clc;

b = Num;
a = Den;
filter_order = length(b) - 1;

fprintf('//Numerator filter coefficients\n');
fprintf('const float b[] = {');
for k=1:filter_order+1
   fprintf('%.32e', b(k));
   if (k ~= filter_order+1)
       fprintf(', ');
   end
end
fprintf('};\n\n');

fprintf('//Denominator filter coefficients\n');
fprintf('const float a[] = {');
for k=2:filter_order+1
   fprintf('%.32e', a(k));
   if (k ~= filter_order+1)
       fprintf(', ');
   end
end
fprintf('};\n');