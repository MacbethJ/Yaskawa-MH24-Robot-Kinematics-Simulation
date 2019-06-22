filename=uigetfile('*.stl');
[F,V,C]=stlmat(filename);
V(:,4)=1;
num=filename(2);
linkname=filename(1:end-4);
eval([linkname '.F' num '=F']);
eval([linkname '.V' num '=V']);
