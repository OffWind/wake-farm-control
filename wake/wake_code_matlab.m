function wake_code_matlab(arg1,arg2)
%This function is a placeholder for the mex function of the same name
%To get the real function, you need to compile it first.
%To compile you need a fortran and a C compiler. Type:
%   mex -setup
%
%Known issue with gfortran compiler on MAC
%if you get an error that says that gfortran cannot be found, you should
%manually change the file: /Users/username/.matlab/R20XXx/mexopts.sh
%Change FC='gfortran' to   FC='/usr/local/bin/gfortran'  two places
%Then you should be able to compile the function by:
%
%     mex wake_code_matlab.f90  wake_mex_wrapper.F
%
%
%If you already compiled the code, you use the help function to see how it actually works, 
%then be aware that this is still just a beta version.
%You need to run it with two input, which does not do anything yet, so they
%are just dummy values. For example, you can run:
%
%    wake_code_matlab(1,ones(1,4));
%
%Then the code should run and write the same files to the disk, as the
%fortran code does.
%
%You should then be able to visualize the wind field with 
%    plot_wind_field
%
%In the future the function will return a matrix of the wind field as well
%as writing to the disk.


%Created 19/07-2013 by MS
disp('You probably did not compile the function.')
disp('See help for this function, or type:')
disp(' help wake_code_matlab') 