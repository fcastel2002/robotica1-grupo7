thisFile = mfilename('fullpath');
thisDir = fileparts(thisFile);
cacheFile = fullfile(thisDir, 'irb120_cache.mat');
if isfile(cacheFile)
    fprintf('>> Datos precalculados cargados desde %s\n', cacheFile);
    load(cacheFile, 'J','detJ','R1','symq');
else
    dh = [ 0    sym(0.290)   0       sym(-pi/2)   0;
           0    0       sym(0.270)   0           0;
           0    0       sym(0.070)  sym(-pi/2)   0;
           0    sym(0.302)   0      sym(pi/2)    0;
           0    0            0      sym(-pi/2)   0;
           0    sym(0.072)   0       0           0 ];

    R1 = SerialLink(dh,'name',"robot1");
    symq = sym("q",[1 6],"real");
    R1.offset = [0, sym(-pi/2), 0, 0, 0, 0];

    digitsOld = digits;
    digits(3);

    %Tsym = vpa(simplify(R1.fkine(symq)),4);
    J = vpa(simplify(R1.jacob0(symq)),4);
    detJ = vpa(det(J));

    digits(digitsOld);

    save(cacheFile, 'J','detJ','R1','symq');
    fprintf('>> Cache guardado en %s\n', cacheFile);
end