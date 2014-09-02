function Fn = defint(fun,xo,xn,n,rule,varargin);
%DEFINT numerically evaluate a definite integral
%   Fn = DEFINT(@fun,a,b,n,RULE,x1,...,xn) numerically integrates the definite
%   integral from [a,b] using n intervals.  x1,...,xn are optional arguments
%   that can be passed to feval(@fun,x1,...,xn).  RULE is an optional argument
%   which can be used to specify the integration scheme. 
%
%   Choices for RULE are: 'forward'    forward Euler
%                         'backward'   backward Euler
%                         'midpoint'   midpoint estimate
%                         'trapezoid'  trapezoidal rule
%                         'simpson'    Simpson's rule  (default)
%
%   Example:  log(2) = int (1/x)dx [1,2]
%
%   % create inline function
%   fun = inline('1/x');
%  
%   % evaluate integral using 5 partitions
%   Ln = defint(fun,1,2,5,'forward');
%   Rn = defint(fun,1,2,5,'backward');
%   Mn = defint(fun,1,2,5,'midpoint');
%   Tn = defint(fun,1,2,5,'trapezoid');
%   Sn = defint(fun,1,2,5,'simpson');
%
%   % display estiamtes and actual answer
%   fprintf('Ln=%g\nRn=%g\nMn=%g\nTn=%g\nSn=%g\nlog(2)=%g\n',Ln,Rn,Mn,Tn,Sn,log(2));
%
% History
% Date          Who        Comment
% ----------    ---        -----------------------------------
% 03-23-2005    rme        Created and written.


if ~exist('rule','var') || isempty('rule'); rule = 'simpson'; end;
rule = lower(rule);

dx = (xn-xo)/n;  % step size
x = xo+[0:n]*dx; % parition points

switch rule;
case {'midpoint','simpson'}; evalmidpts = true;
otherwise evalmidpts = false;
end;

fxo = feval(fun,x(1),varargin{:});
fxn = feval(fun,x(end),varargin{:});
fxp = zeros(size(fxn));
fxm = fxp;
for ii=2:(n+1);
  if ii<=n;
    % f(x) @ partition points
    fxp = fxp + feval(fun,x(ii),varargin{:});
  end;
  if evalmidpts;
    % f(x) @ midpoints
    xm  = 0.5*(x(ii)+x(ii-1));
    fxm = fxm + feval(fun,xm,varargin{:});
  end;
end;

% choose numerical integration scheme
switch rule;
case 'forward';
 % forward euler
 Fn = dx*(fxo + fxp);
case 'backward';
 % backward euler
 Fn = dx*(fxp + fxn);
case 'midpoint';
 % midpoint estimate
 Fn = dx*fxm;
case 'trapezoid';
 % trapezoidal rule
 Fn = dx/2*(fxo + 2*fxp + fxn);
case 'simpson';
 % simpson's rule (parabolic estimate)
 Fn = dx/6*(fxo + fxn + 2*fxp + 4*fxm);
otherwise;
 error(sprintf('unknown integration method: %s',rule));
end;
