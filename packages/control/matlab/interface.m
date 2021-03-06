function [methodinfo,structs,enuminfo]=test;
%This function was generated by the perl file prototypes.pl called from loadlibary.m on Thu Jun 28 18:24:32 2007
%perl options:'demo_controller.i -outfile=test.m'
ival={cell(1,0)}; % change 0 to the actual number of functions to preallocate the data.
fcns=struct('name',ival,'calltype',ival,'LHS',ival,'RHS',ival,'alias',ival);
structs=[];enuminfo=[];fcnNum=1;
%  void translationalController(MeasuredState* measuredState, DesiredState* desiredState, ControllerState* controllerState, double dt, double* translationalForces); 
fcns.name{fcnNum}='rotationalController'; ... 
    fcns.calltype{fcnNum}='cdecl'; ...
    fcns.LHS{fcnNum}=[]; ...
    fcns.RHS{fcnNum}={'MeasuredStatePtr', 'DesiredStatePtr', ...
                      'ControllerStatePtr', 'double', 'doublePtr'};fcnNum=fcnNum+1;

structs.DesiredState.packing=8;
structs.DesiredState.members=struct('speed', 'double', 'depth', 'double', ...
   'q0', 'double', 'q1', 'double', 'q2', 'double', 'q3', 'double',  ...
   'w0', 'double', 'w1', 'double', 'w2', 'double');
structs.MeasuredState.packing=8;
structs.MeasuredState.members=struct('depth', 'double', ...
    'a0', 'double', 'a1', 'double', 'a2', 'double', ...
    'q0', 'double', 'q1', 'double', 'q2', 'double', 'q3', 'double', ...
    'w0', 'double', 'w1', 'double', 'w2', 'double');
structs.ControllerState.packing=8;
structs.ControllerState.members=struct(...
    'angularPGain', 'double', 'angularDGain', 'double', ...
    'ie0', 'double', 'ie1', 'double', 'ie2', 'double',...
    'ie3', 'double', 'ie4', 'double', 'ie5', 'double',... 
    'ie6', 'double', 'ie7', 'double', 'ie8', 'double',...
    'depthPGain', 'double', 'speedPGain', 'double');
structs.TestStruct.packing=8;
structs.TestStruct.members=struct('depth', 'double', 'q0', 'double', 'q1', 'double', 'q2', 'double', 'q3', 'double');
methodinfo=fcns;