function y1 = rungekutta(fhandle,x0,y0,dx)
%RUNGEKUTTA 4th order Runge-Kutta method for solving a non-linear
%           differential equation numerically (approximately).
%
%  Y1 = RUNGEKUTTA(FHANDLE,XO,YO,DX) solves dY/dX = f(X,Y) where X is the
%  independent variable.  f is the nonlinear vector valued function
%  specified by the function handle FHANDLE, X0 and Y0 are the vector valued
%  initial conditions, and DX is the step size.  The function handle must
%  accept arguments as FHANDLE(XO,Y0).
%
%  Y1 = RUNGEKUTTA(FHANDLE,[],YO,DX) solves dY/dX = f(Y).  In this case the
%  function handle must accept arguments as FHANDLE(YO).
%
%  NOTE: The use of column vectors is assumed.
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    07-20-2003      rme         Created and written.

if isempty(x0)
  k1 = feval(fhandle,y0);
  k2 = feval(fhandle,y0+0.5*dx*k1);
  k3 = feval(fhandle,y0+0.5*dx*k2);
  k4 = feval(fhandle,y0+dx*k3);
else
  k1 = feval(fhandle,x0,y0);
  k2 = feval(fhandle,x0+0.5*dx,y0+0.5*dx*k1);
  k3 = feval(fhandle,x0+0.5*dx,y0+0.5*dx*k2);
  k4 = feval(fhandle,x0+dx,y0+dx*k3);  
end

y1 = y0 + dx/6*(k1 + 2*k2 + 2*k3 + k4);
