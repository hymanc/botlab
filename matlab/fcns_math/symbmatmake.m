function out = symbmatmake(vn_,m_,n_,opts_)
% OUT = SYMBMATMAKE(VN,M,N,TYPE)
% OUT is a symbolic marix of vector of dimension M,N.
% Elements have name VN (default 'x') and are subscripted according to position
% M has default 2
% N has default 1
% TYPE is default empty, see syms for choices.
%
% SYMMATMAKE('a',2,3) returns
% [ a11, a12, a13]
% [ a21, a22, a23]

%%%%% © Natural Environment Research Council, (2002)
%%%%% Written by R.C.A. Hindmarsh.

% This program is free software; you can redistribute it and/or
% modify it under the terms of the GNU General Public License
% as published by the Free Software Foundation; either version 2
% of the License, or (at your option) any later version.

% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.

% You should have received a copy of the GNU General Public License
% along with this program; if not, write to the Free Software
% Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

if ~exist('opts_','var') opts_ = []; end
  
% Set up defaults
if  ~nargin,
   vn_ = 'x';
end
if nargin < 2
   m_ = 2;
end
if nargin < 3
   n_ = 1;
end
% Create symbolic value
if ~strcmp(class(vn_),'sym')
   eval(['syms ' vn_])
end
% Decide whether to concatenate horizontally or vertically
if m_ > 1
   fn_ = 'vertcat';
   morn_ = m_;
else
   fn_ = 'horzcat';
   morn_ = n_;
end
% Do the concatentation. Recursive calls if concatenating vectors to make
% matrix
for ii_ = 1:morn_
   if m_==1 | n_ == 1
       eval(['syms ', vn_, num2str(ii_), ' ', opts_])
   else % Create matrix
       evstring = [vn_, num2str(ii_),' = symbmatmake(''',vn_,num2str(ii_),''',',...
             num2str(1),',',num2str(n_),',', '''' opts_ '''' ');'];
       eval(evstring)
   end
	if ii_ == 1
     eval([vn_, ' = [',vn_, num2str(ii_),'];']);;
	else
     eval([vn_, ' = ',fn_,'(',vn_,',',vn_, num2str(ii_),');']);
	end
end
eval(['out = ',vn_,';']);
