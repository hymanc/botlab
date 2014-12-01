function J = numerical_jacobian(fhandle,x,y,Jpattern,varargin)
%NUMERICAL_JACOBIAN approximate Jacobian via finite-differencing.
%   J = NUMERICAL_JACOBIAN(FHANDLE,X) numerically evaluates the Jacobian of
%   the function referred to by FHANDLE around the point X by using
%   finite-differencing.
%
%   J = NUMERICAL_JACOBIAN(FHANDLE,X,Y) allows the user to supply the
%   value of the function evaluated at X.  This saves one function evaluation.
%  
%   J = NUMERICAL_JACOBIAN(FHANDLE,X,Y,JPATTERN) allows the user to specify
%   the sparsity pattern of the Jacobian.
%
%   J = NUMERICAL_JACOBIAN(FHANDLE,X,Y,JPATTERN,VARARGIN) allows the user
%   to pass any number of optional parameters to the function specified
%   by FHANDLE.
%
%   Example 1:
%   J = numerical_jacobian(@sin,pi);
%
%   Example 2:
%   To specify optional arguments to be passed to FHANDLE without
%   specifiying JPATTERN or Y the following syntax may be used:
%   J = numerial_jacobian(@myfun,x,[],[],'opt1',value,'opt2',value);
%
%
%-----------------------------------------------------------------
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    06-23-2003      op          Created and written.
%    08-05-2003      rme         Modified to pass optional function
%                                arguments and to evaluate Jacobian only
%                                at particular elements of x.
%    09-28-2003      rme         Changed input argument to accept
%                                sparsity pattern for Jacobian, and
%                                cleaned up the syntax for specifying
%                                optional parameters to be passed to FHANDLE.

error(nargchk(2,inf,nargin));

% parse input
%--------------------------------------------
OPTARGS = false;
if nargin >= 5; 
  OPTARGS  = true;
end

if ~exist('y','var') || isempty(y)
  if OPTARGS
    y = feval(fhandle,x,varargin{:});
  else
    y = feval(fhandle,x);
  end
end

xdim = length(x); % number of input parameters
ydim = length(y); % number of ouput parameters

if ~exist('Jpattern','var') || isempty(Jpattern)
  % calculate partial w.r.t each element of x
  deriv_wrt = [1:length(x)];
else
  if size(Jpattern,2) ~= xdim || size(Jpattern,1) ~= ydim
    error('Jpattern dimension is not consistent with dimension of x and f(x)');
  end
  deriv_wrt = find(sum(Jpattern));
end

if length(deriv_wrt) == xdim
  % calculate the full jacobian
  J = zeros(ydim,xdim);
  JSPARSE = false;
else
  % sparse Jacobian
  J = double(Jpattern);
  JSPARSE = true;
end
%--------------------------------------------

% vector of deltas to calculate J using HZ00 suggestion for delta size
dx = max(abs(1e-4*x),1e-6);

for k = deriv_wrt
  % initialize x_prime to x
  x_prime = x;
  % perturb x_prime along dimension k
  x_prime(k) = x_prime(k) + dx(k);
  % evaluate f(x) at x_prime
  if OPTARGS
    y_prime = feval(fhandle,x_prime,varargin{:});    
  else
    y_prime = feval(fhandle,x_prime);
  end
  % calculate partial w.r.t x_k
  DyDxk = (y_prime-y)/dx(k);
  if JSPARSE
    ind = find(J(:,k));
    J(ind,k) = DyDxk(ind);
  else
    J(:,k) = DyDxk;
  end
end
