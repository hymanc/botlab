function [T,norm_pts] = normalize_3d_pts(x_pts);
% NORMALIZE_3D_PTS  3D data normalization for DLT algorithm.
%   T = NORMALIZE_3D_PTS(X) returns the similarity transformation T
%   which scales and translates the 2D point cloud, represented by
%   the [4xN] homogenous matrix X, such that its centroid is
%   located at the origin and the average distance of a point (x,y,z)
%   from the origin is equal to sqrt(3).
%
%   [T,X_NORM] = NORMALIZE_3D_PTS(X) optionally returns the
%   normalized point cloud, i.e. X_NORM = T*X
%
%   Note: this data normalization is an *essential* step in the DLT
%   algorithm and must not be considered optional!
%
%    History:
%    Date            Who         What
%    -----------     -------     -----------------------------
%    11-09-2002      rme         Created and written.

%================================================================

% calculate the point cloud centroid
M = size(x_pts,2);
x_c = sum(x_pts(1,:))/M;
y_c = sum(x_pts(2,:))/M;
z_c = sum(x_pts(3,:))/M;

% translate the point cloud to the origin and calculate
% the average distance from the origin
d_avg = sum(((x_pts(1,:)-x_c).^2 + (x_pts(2,:)-y_c).^2 ...
	     + (x_pts(3,:)-z_c).^2).^0.5)/M;

% compose the similarity transform which maps the cloud centroid to
% the origin and scales the average distance from the origin to sqrt(3)
T = [sqrt(3)/d_avg*[eye(3), -[x_c,y_c,z_c]']; 0 0 0 1];

if nargout == 2
  norm_pts = T*x_pts;
end

