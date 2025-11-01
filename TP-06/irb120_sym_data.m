thisFile = mfilename('fullpath');
thisDir = fileparts(thisFile);
cacheFile = fullfile(thisDir, 'irb120_cache.mat');
if isfile(cacheFile)
    fprintf('>> DH simbolico cargado desde %s\n', cacheFile);
    load(cacheFile,'R1','symq');
else
dh = [ 0      sym(0.290)   0        sym(-pi/2)   0;
       0      0            sym(0.270)  0         0;
       0      0            sym(0.070)  sym(-pi/2) 0;
       0      sym(0.302)   0        sym(pi/2)    0;
       0      0            0        sym(-pi/2)   0;
       0      sym(0.072)   0        0            0 ];


    R1 = SerialLink(dh,'name',"robot1");
    symq = sym("q",[1 6],"real");
    R1.offset = [0, sym(-pi/2), 0, 0, 0, 0];

    save(cacheFile,'R1','symq');
    fprintf('>> Cache guardado en %s\n', cacheFile);
end